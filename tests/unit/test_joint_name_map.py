"""联合模型 `name_map` 的回归测试（A 方案，2026-09-24）。

覆盖两件事：
1. `scene_builder._joint_manipulation` 按**可判定规则**生成 `name_map`，并把
   mapped / identical / conflicts / missing 四个桶如实留痕（冲突不进表）；
2. 后端把声明名（Profile 口径 `joint1`）解析到模型名（`piper_joint1`），
   且四条装配期校验（同值 / 声明名已存在 / 指向不存在 / 多对一）全部 fail-closed。

为什么必须测负向：这次的原始缺陷是"Profile 名 == 模型名"这个隐含前提被跨本体场景打破，
若只测正向，回归会退化成"命令被接受"就算过 —— 而真正的判据是**解析到的可以是别的名字**。
"""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend, _parse_name_map
from iraf_adapters.unitree.scene_builder import SceneBuildError

# 一个小模型：主本体（`base_link` = 狗躯干、`box_01` = 场景道具）+ 附加本体（link1/指爪）。
TINY_MODEL = """
<mujoco>
  <worldbody>
    <body name="base_link"><geom name="dog_geom" size="0.05"/></body>
    <body name="box_01"><geom name="box_01_geom" size="0.02"/></body>
    <body name="piper_base_link"><geom name="piper_dog_geom" size="0.03"/></body>
    <body name="piper_link1">
      <joint name="piper_joint1" type="hinge"/>
      <geom name="piper_g1" size="0.05"/>
      <body name="piper_left_finger"><geom name="piper_left_finger_geom" size="0.01"/></body>
    </body>
  </worldbody>
</mujoco>
"""


class JointNameMapBuildTests(unittest.TestCase):
    """构建器侧：声明名清单 → name_map（机械规则 + 四桶留痕）。"""

    def _report(self, model, declared_names):
        from iraf_adapters.unitree.scene_builder import _joint_manipulation

        with tempfile.TemporaryDirectory() as tmp:
            arm_report = Path(tmp) / "arm.json"
            arm_report.write_text(json.dumps({
                "target_id": "box_01",
                "gripper": {
                    "wrist_body": "link1",
                    "left_finger_body": "left_finger",
                    "left_finger_geom": "left_finger_geom",
                    "open_positions": {"joint1": 0.0},
                },
                "targets": [{"id": "box_01", "body": "box_01", "geom": "box_01_geom"}],
                "vision": None,
            }, ensure_ascii=False), encoding="utf-8")
            return _joint_manipulation(Path(tmp), arm_report, "piper_", {"bodies": [], "geoms": []},
                                       model, declared_names)

    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(TINY_MODEL)

    def test_mapped_names_enter_map_and_conflicts_do_not(self):
        """只有"加前缀才存在"的名字进表；主/附加本体同名（base_link）只能留痕，不静默选一个。"""
        result = self._report(self.model, {
            "bodies": ["base_link", "link1", "left_finger", "world"],
            "geoms": ["left_finger_geom", "dog_geom"],
            "sites": [], "joints": ["joint1"],
        })
        name_map = result["name_map"]
        # 加前缀才存在的名字：进表
        self.assertEqual(name_map["link1"], "piper_link1")
        self.assertEqual(name_map["left_finger"], "piper_left_finger")
        self.assertEqual(name_map["left_finger_geom"], "piper_left_finger_geom")
        self.assertEqual(name_map["joint1"], "piper_joint1")
        # 主本体也有的名字：冲突 ⇒ 不进表（留痕），也不静默映射成附加本体的对象
        # 主本体与附加本体**同名**（base_link / dog_geom 两处都存在）⇒ 映射有歧义：不进表只留痕
        self.assertNotIn("base_link", name_map)
        self.assertNotIn("dog_geom", name_map)
        facts = result["name_map_facts"]
        conflicts = {(item["kind"], item["declared"]) for item in facts["conflicts"]}
        self.assertIn(("bodies", "base_link"), conflicts)
        self.assertIn(("geoms", "dog_geom"), conflicts)
        mapped = {(item["kind"], item["declared"]) for item in facts["mapped"]}
        self.assertIn(("joints", "joint1"), mapped)
        self.assertIn(("bodies", "link1"), mapped)
        # 只有原名存在（world、box_01_geom）⇒ identical；两者都不存在 ⇒ missing（只留痕）
        identical = {(item["kind"], item["declared"]) for item in facts["identical"]}
        self.assertIn(("bodies", "world"), identical)
        self.assertEqual(facts["counts"]["mapped"], len(mapped))
        self.assertEqual(facts["counts"]["missing"], 0)

    def test_missing_referenced_name_fails_closed(self):
        """被 gripper 直接引用、却在联合模型里两个名字都不存在的名字 ⇒ 构建期显式失败。"""
        from iraf_adapters.unitree.scene_builder import _joint_manipulation

        with tempfile.TemporaryDirectory() as tmp:
            arm_report = Path(tmp) / "arm.json"
            arm_report.write_text(json.dumps({
                "gripper": {"wrist_body": "link9", "left_finger_geom": "g_ok"},
                "targets": [], "target_id": "box_01",
            }, ensure_ascii=False), encoding="utf-8")
            with self.assertRaises(SceneBuildError) as ctx:
                _joint_manipulation(Path(tmp), arm_report, "piper_",
                                    {"bodies": [], "geoms": []}, self.model,
                                    {"bodies": [], "geoms": [], "sites": [], "joints": []})


class NameMapResolutionTests(unittest.TestCase):
    """后端侧：解析 + 四条 fail-closed 校验（用 SimpleNamespace 假模型，不编译真模型）。"""

    def _backend(self, name_map, known):
        """known = {kind: {名字}}：控制 mj_name2id 的返回值，模拟模型里实际存在的对象。"""
        # actuator_trnid 必须能按 [i, 0] 取（真实模型是 2 维 ndarray；纯 list 会 TypeError）
        import numpy as np
        model = SimpleNamespace(nu=2, nkey=0, opt=SimpleNamespace(timestep=0.005),
                                actuator_trnid=np.array([[0], [1]], dtype=int))
        profile = SimpleNamespace(joints=("joint1", "joint2"))
        patches = (
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.MjModel",
                  SimpleNamespace(from_xml_path=lambda path: model)),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.MjData"),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_forward"),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_id2name",
                  side_effect=lambda model, kind, index: {0: "piper_joint1",
                                                          1: "piper_joint2"}.get(int(index))),
            patch("iraf_adapters.mujoco.mujoco_backend.mujoco.mj_name2id",
                  side_effect=lambda model, kind, name: 0 if str(name) in known else -1),
        )
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        return MujocoBackend("model.xml", profile, authority=SimpleNamespace(), name_map=name_map)

    def test_no_map_is_identity(self):
        backend = self._backend(None, known=set())
        self.assertEqual(backend._name_map, {})
        self.assertEqual(backend._model_name("joint1"), "joint1")
        self.assertEqual(backend._declaration_name("piper_joint1"), "piper_joint1")

    def test_mapped_name_resolves_both_ways(self):
        backend = self._backend({"joint1": "piper_joint1"}, known={"piper_joint1"})
        self.assertEqual(backend._model_name("joint1"), "piper_joint1")
        self.assertEqual(backend._declaration_name("piper_joint1"), "joint1")
        # 执行器通道解析：声明名 → 模型里的执行器名（这正是不映射时抛错的那条路径）
        self.assertEqual(backend._actuator_channel("joint1"), "piper_joint1")
        # 状态回写的键空间仍是**声明名**（last_positions / profile.joints 口径）
        self.assertEqual(backend._joint_name_of("piper_joint1"), "joint1")
        self.assertEqual(backend._joint_name_of("joint2"), "joint2")

    def test_unmapped_name_still_fails_loudly(self):
        backend = self._backend({"joint1": "piper_joint1"}, known={"piper_joint1"})
        with self.assertRaises(ValueError) as ctx:
            backend._actuator_channel("joint2")
        self.assertIn("找不到关节或执行器: joint2", str(ctx.exception))

    def test_validation_gates(self):
        cases = {
            "同值映射": ({"joint1": "joint1"}, {"piper_joint1"}),
            "声明名已存在于模型": ({"piper_joint1": "piper_joint2"}, {"piper_joint1", "piper_joint2"}),
            "指向不存在的模型名": ({"joint1": "piper_jointX"}, set()),
            "多对一": ({"joint1": "piper_joint1", "joint2": "piper_joint1"}, {"piper_joint1"}),
        }
        for label, (mapping, known) in cases.items():
            with self.subTest(label=label):
                with self.assertRaises(ValueError) as ctx:
                    self._backend(mapping, known)
                self.assertTrue(str(ctx.exception), "校验失败必须给出可操作原因：%s" % label)

    def test_parse_name_map_rejects_non_object(self):
        with self.assertRaises(ValueError):
            _parse_name_map([("joint1", "piper_joint1")], SimpleNamespace())


if __name__ == "__main__":
    unittest.main()
