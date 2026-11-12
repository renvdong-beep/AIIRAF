# -*- coding: utf-8 -*-
"""语义 → 场景推断：piper 不得落到联合世界（2026-11-11 实测回归）。"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_core.scene_registry import SceneRegistry  # noqa: E402


class SceneInferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reg = SceneRegistry(root=str(REPO / "config"),
                                profiles_root=str(REPO / "profiles"))
        cls.reg.reload_if_changed()

    def test_piper_text_prefers_standalone_scene(self):
        # joint-handoff 里也有 piper，旧行为会落到联合世界；这里必须选单机场景
        self.assertEqual("piper-pick", self.reg.infer("piper抓取box"))

    def test_ur5_text_prefers_standalone_scene(self):
        self.assertEqual("ur5-pick", self.reg.infer("ur5抓取box"))

    def test_go2_text_falls_to_joint_scene(self):
        # go2 只存在于联合世界（联合场景的资源之一）
        self.assertEqual("joint-handoff", self.reg.infer("go2站起来"))

    def test_handoff_text_maps_to_joint(self):
        self.assertEqual("joint-handoff", self.reg.infer("跑 handoff_lab 全量演示"))

    def test_chinese_box_phrases(self):
        """中文口语“盒子”不得影响机型判定（2026-11-12 使用者口径）。"""
        self.assertEqual("piper-pick", self.reg.infer("piper抓取盒子"))
        self.assertEqual("ur5-pick", self.reg.infer("ur抓取盒子"))

    def test_ur_alias_and_ur5e_alias(self):
        """口语 `ur` / 型号 `ur5e` 都要落到 ur5-pick；不能落默认 piper。"""
        self.assertEqual("ur5-pick", self.reg.infer("ur抓取盒子"))
        self.assertEqual("ur5-pick", self.reg.infer("ur5e抓取盒子"))

    def test_joint_chinese_alias(self):
        self.assertEqual("joint-handoff", self.reg.infer("联合世界交接"))

    def test_non_robot_text_yields_empty(self):
        self.assertEqual("", self.reg.infer("今天天气怎么样"))

    def test_empty_text_yields_empty(self):
        self.assertEqual("", self.reg.infer(""))
        self.assertEqual("", self.reg.infer(None))


if __name__ == "__main__":
    unittest.main()
