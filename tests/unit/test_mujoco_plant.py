"""共享植物（一份 MjData + 多控制器）与**控制权作用域**的回归测试（2026-09-24）。

背景（实测）：每个 Backend 自持 MjData ⇒ 每个本体一个独立世界（
`build/iraf-a6a14/joint-model-two-backends-probe.json`）；而 `stop()` 原本 `ctrl[:] = 0.0`
⇒ 联合模型里一个本体的急停会碰另一个本体的执行器。本文件锁定三件事：

1. 植物只有一个时间推进者，非 owner 推进即显式失败；
2. guest（注入植物的后端）只能写 `name_map` 界定的执行器，越界即显式失败；
3. `stop()` 只归零**自己拥有的**执行器（owner 的集合 = 模型全部 ⇒ 单本体逐位不变）。
"""

import threading
import time
import unittest
from types import SimpleNamespace

import mujoco

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_adapters.mujoco.plant import (MujocoPlant, PlantError, PlantOwnershipError,
                                        PlantWaitTimeout)

# 一只"狗"（1 个执行器）+ 一条"臂"（2 个关节/执行器，名字带 piper_ 前缀）
TINY_MODEL = """
<mujoco>
  <worldbody>
    <body name="base_link">
      <joint name="dog_j1" type="hinge"/>
      <geom size="0.05"/>
    </body>
    <body name="piper_link1">
      <joint name="piper_joint1" type="hinge"/>
      <geom size="0.05"/>
      <body name="piper_link2">
        <joint name="piper_joint2" type="hinge"/>
        <geom size="0.05"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="dog_a0" joint="dog_j1" kp="5"/>
    <position name="piper_joint1" joint="piper_joint1" kp="5"/>
    <position name="piper_joint2" joint="piper_joint2" kp="5"/>
  </actuator>
</mujoco>
"""

ARM_NAME_MAP = {"joint1": "piper_joint1", "joint2": "piper_joint2"}
GUEST_TIMEOUT_FACTOR = 5.0


class _Authority:
    def validate(self, lease):
        return True

    def release(self, lease):
        return None


class PlantOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(TINY_MODEL)
        self.data = mujoco.MjData(self.model)
        self.plant = MujocoPlant(self.model, self.data, owner="A", label="tiny")

    def test_owner_only_advances_time(self):
        self.assertEqual(1, self.plant.advance(1, caller="A"))
        self.assertEqual(1, self.plant.step_index)
        with self.assertRaises(PlantOwnershipError):
            self.plant.step_once("B")
        with self.assertRaises(PlantOwnershipError):
            self.plant.advance(1, caller="B")
        self.assertEqual(1, self.plant.step_index)  # 越权调用没有改动状态

    def test_wait_until_times_out_loudly(self):
        with self.assertRaises(PlantWaitTimeout):
            self.plant.wait_until(5, timeout=0.02, poll_seconds=0.005)

    def test_wait_until_returns_after_owner_advances(self):
        done = threading.Event()

        def waiter():
            self.plant.wait_until(3, timeout=1.0, poll_seconds=0.002)
            done.set()

        thread = threading.Thread(target=waiter)
        thread.start()
        time.sleep(0.02)
        self.assertFalse(done.is_set())
        self.plant.advance(3, caller="A")
        thread.join(timeout=1.0)
        self.assertTrue(done.is_set())

    def test_requires_explicit_model_and_owner(self):
        with self.assertRaises(PlantError):
            MujocoPlant(None, self.data, owner="A")
        with self.assertRaises(PlantError):
            MujocoPlant(self.model, self.data, owner=None)


class SharedPlantScopeTests(unittest.TestCase):
    """两个后端视图 + 一个植物：共享数据、按执行器划分控制权。"""

    def _views(self, guest_timeout=GUEST_TIMEOUT_FACTOR, guest_name_map=ARM_NAME_MAP):
        model_path = "build/tiny-shared.xml"  # 只作日志/标签用；模型由 patch 提供
        self.model = mujoco.MjModel.from_xml_string(TINY_MODEL)
        real_from_xml = mujoco.MjModel.from_xml_path
        mujoco.MjModel.from_xml_path = staticmethod(lambda path: self.model)
        self.addCleanup(setattr, mujoco.MjModel, "from_xml_path", real_from_xml)
        owner = MujocoBackend(model_path, SimpleNamespace(joints=("dog_j1",), name="go2"),
                             _Authority(), name_map=None)
        guest = MujocoBackend(model_path, SimpleNamespace(joints=("joint1", "joint2"), name="piper"),
                              _Authority(), name_map=guest_name_map, plant=owner.plant,
                              plant_guest_timeout_factor=guest_timeout)
        return owner, guest

    def test_views_share_one_plant(self):
        owner, guest = self._views()
        self.assertIs(owner.plant, guest.plant)
        self.assertIs(owner.data, guest.data)
        self.assertIs(owner.model, guest.model)

    def test_guest_writes_only_its_own_actuators(self):
        owner, guest = self._views()
        self.assertEqual({"piper_joint1", "piper_joint2"}, guest._owned_actuators)
        self.assertEqual({"dog_a0", "piper_joint1", "piper_joint2"}, owner._owned_actuators)
        guest._write_ctrl("piper_joint1", 0.3)
        with self.assertRaises(PlantError) as ctx:
            guest._write_ctrl("dog_a0", 9.9)
        self.assertIn("控制权越界", str(ctx.exception))
        owner._write_ctrl("dog_a0", 1.5)  # owner 拥有全部 ⇒ 不越界
        self.assertEqual(1.5, float(guest.data.ctrl[guest._actuators["dog_a0"]]))

    def test_scoped_stop_leaves_other_robot_untouched(self):
        owner, guest = self._views()
        for name, index in guest._actuators.items():
            guest.data.ctrl[index] = 1.0
        guest.stop(lease=None)
        self.assertEqual(0.0, float(guest.data.ctrl[guest._actuators["piper_joint1"]]))
        self.assertEqual(0.0, float(guest.data.ctrl[guest._actuators["piper_joint2"]]))
        # 主本体（狗）不受臂的急停影响
        self.assertEqual(1.0, float(guest.data.ctrl[guest._actuators["dog_a0"]]))
        # owner 的作用域 = 全部执行器（单本体语义不变）
        owner.stop(lease=None)
        self.assertEqual(0.0, float(owner.data.ctrl[owner._actuators["dog_a0"]]))

    def test_guest_requires_declared_timeout_and_name_map(self):
        with self.assertRaises(PlantError):
            self._views(guest_timeout=None)
        with self.assertRaises(PlantError):
            self._views(guest_name_map=None)
        with self.assertRaises(PlantError):
            self._views(guest_timeout=-1.0)

    def test_guest_step_waits_for_owner_advance(self):
        # 系数是**声明的墙钟余量**：本用例让 owner 晚 50 ms 才推进 ⇒ 必须给足够余量
        # （默认 5.0 在 timestep=0.002、3 步时只有 30 ms，会被判超时 —— 这正是"显式超时"该有的行为）
        owner, guest = self._views(guest_timeout=200.0)
        result = {}

        def runner():
            result["values"] = guest.step(3)

        thread = threading.Thread(target=runner)
        thread.start()
        time.sleep(0.05)
        self.assertEqual(0, owner.plant.step_index)      # guest 不能自己推进
        self.assertTrue(thread.is_alive())               # 它在等 owner
        owner.step(3)
        thread.join(timeout=1.0)
        self.assertEqual(3, owner.plant.step_index)
        self.assertEqual({"joint1": 0.0, "joint2": 0.0}, result["values"])

    def test_guest_gets_explicit_timeout_when_owner_stalls(self):
        owner, guest = self._views(guest_timeout=0.001)
        with self.assertRaises(PlantWaitTimeout):
            guest.step(50)


if __name__ == "__main__":
    unittest.main()
