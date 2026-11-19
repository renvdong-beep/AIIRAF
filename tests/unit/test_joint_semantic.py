# -*- coding: utf-8 -*-
"""语义入口：prop 模板解析 + 别名词表（阶段 1/2）。"""
import importlib.util
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))
spec = importlib.util.spec_from_file_location("joint_semantic", REPO / "scripts/joint_semantic.py")
sem = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sem)


class SemanticPropsTest(unittest.TestCase):
    def setUp(self):
        self.library = sem.load_props_library(REPO)

    def test_library_has_common_props(self):
        for name in ("box", "cube", "ball", "cylinder", "cup", "tray", "button"):
            self.assertIn(name, self.library)

    def test_parse_explicit_position(self):
        props = sem.parse_props(["cup@0.2,-0.2,0.09"], self.library)
        self.assertEqual(1, len(props))
        self.assertEqual([0.2, -0.2, 0.09], props[0]["pose"]["pos_m"])
        self.assertEqual("cup_00", props[0]["id"])

    def test_box_id_is_box_01_for_pick_constraint(self):
        props = sem.parse_props(["box@0.28,-0.28,0.03"], self.library)
        self.assertEqual("box_01", props[0]["id"])

    def test_text_aliases(self):
        robots, props = sem.parse_text("piper 和狗，桌上一个杯子", self.library)
        self.assertEqual(["piper", "unitree_go2"], robots)
        self.assertEqual("cup_00", props[0]["id"])


if __name__ == "__main__":
    unittest.main()
