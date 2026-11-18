# -*- coding: utf-8 -*-
"""composer Profile 并集：piper + go2 = 20 关节、能力并集、资源映射。"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_adapters.mujoco.joint_compose import compose_profile, load_descriptor  # noqa: E402


class ComposeProfileTest(unittest.TestCase):
    def test_piper_go2_union(self):
        base = REPO / "scenes/handoff_lab/scene.yaml"
        descriptors = [load_descriptor(base, "piper"), load_descriptor(base, "unitree_go2")]
        profile = compose_profile(descriptors, scene_id="piper_go2_lab")
        spec = profile["spec"]
        self.assertEqual(20, len(spec["joints"]))
        for cap in ("pick_object", "stand", "locomote", "dock_for_handoff"):
            self.assertIn(cap, spec["capabilities"])
        self.assertEqual({"piper": "arm", "unitree_go2": "quadruped"},
                         spec["joint"]["resources"])


if __name__ == "__main__":
    unittest.main()
