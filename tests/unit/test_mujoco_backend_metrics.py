import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_adapters.mujoco.supervisor import MujocoSimulationSupervisor


class FakeControl:
    """假 ctrl 数组：记录每个下标被写成什么（安全停机的作用域判据用）。

    2026-09-24 起 `_safe_stop_controls` **按执行器作用域**归零（不再 `ctrl[:] = 0.0`，
    见 docs/debug/2026-09-24-joint-model-dog-arm.md §4.4）⇒ 断言从"整条被归零"改成
    "拥有的执行器被归零、且只归零它们"。
    """

    def __init__(self, size=2):
        self.values = {index: 1.0 for index in range(size)}

    def __setitem__(self, key, value):
        if isinstance(key, slice):
            for index in range(*key.indices(len(self.values))):
                self.values[index] = value
            return
        self.values[int(key)] = value

    def __getitem__(self, key):
        return self.values[int(key)]


class MujocoBackendMetricsTests(unittest.TestCase):
    def _backend(self, fault_injection_enabled=True):
        # nu=2：植物要求真的有一份 ctrl 数组（自带植物 ⇒ 拥有的执行器 = 模型全部）
        model = SimpleNamespace(nu=2, nkey=0, opt=SimpleNamespace(timestep=0.005),
                                actuator_trnid=[[0], [1]])
        data = SimpleNamespace(ctrl=FakeControl(2), qpos=[0.0, 0.0])
        profile = SimpleNamespace(joints=())
        patches = (
            patch(
                "iraf_adapters.mujoco.mujoco_backend.mujoco.MjModel",
                SimpleNamespace(from_xml_path=lambda path: model),
            ),
            patch(
                "iraf_adapters.mujoco.mujoco_backend.mujoco.MjData",
                return_value=data,
            ),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_forward"),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_step"),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_id2name",
                  side_effect=lambda model, kind, index: {0: "a0", 1: "a1"}.get(int(index))),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_name2id",
                  side_effect=lambda model, kind, name: -1),
        )
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        return MujocoBackend(
            "model.xml",
            profile,
            authority=SimpleNamespace(),
            fault_injection_enabled=fault_injection_enabled,
        )

    def _wait_for(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        self.fail("等待仿真状态变化超时")

    def test_reports_frequency_latency_and_overrun(self):
        backend = self._backend()
        supervisor = MujocoSimulationSupervisor(backend)
        self.addCleanup(supervisor.stop)
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 3)

        backend.inject_fault("step_delay", duration_ms=20)
        self._wait_for(
            lambda: supervisor.diagnostics()["step_overrun_count"] >= 1
        )
        status = supervisor.diagnostics()

        self.assertTrue(status["healthy"])
        self.assertEqual(200.0, status["target_frequency_hz"])
        self.assertGreater(status["effective_frequency_hz"], 0)
        self.assertGreaterEqual(status["max_step_duration_ms"], 20)

    def test_step_failure_is_fail_closed_and_requires_explicit_clear(self):
        backend = self._backend()
        supervisor = MujocoSimulationSupervisor(backend)
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 1)

        backend.inject_fault("step_failure")
        self._wait_for(lambda: supervisor.diagnostics()["last_error"] is not None)
        status = supervisor.diagnostics()

        self.assertFalse(status["healthy"])
        self.assertFalse(status["running"])
        self.assertEqual(1, status["step_failure_count"])
        self.assertTrue(backend.stopped)
        # 作用域归零：拥有的执行器（自带植物 ⇒ 全部）都归零，且只归零它们
        self.assertEqual({0: 0.0, 1: 0.0}, backend.data.ctrl.values)
        with self.assertRaisesRegex(RuntimeError, "尚未清除"):
            supervisor.start()

        self.assertTrue(supervisor.stop())
        self.assertTrue(supervisor.clear_faults())
        self.assertTrue(supervisor.start())
        self._wait_for(lambda: supervisor.diagnostics()["step_count"] >= 1)
        self.assertTrue(supervisor.diagnostics()["healthy"])
        self.assertTrue(supervisor.stop())

    def test_fault_injection_is_disabled_by_default(self):
        backend = self._backend(fault_injection_enabled=False)

        with self.assertRaisesRegex(PermissionError, "未启用"):
            backend.inject_fault("step_failure")


if __name__ == "__main__":
    unittest.main()
