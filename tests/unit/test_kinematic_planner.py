import unittest
from pathlib import Path

try:
    import mujoco  # noqa: F401
    HAS_MUJOCO = True
except Exception:  # pragma: no cover - 端侧/CI 未必装 mujoco
    HAS_MUJOCO = False

from iraf_adapters.kinematic.planner import PlanError, plan_pick, resolve_scene

REPO = Path(__file__).resolve().parents[2]


class ResolveSceneTest(unittest.TestCase):
    @unittest.skipUnless((REPO / "build/models/ur5-pick-scene.xml").is_file(),
                         "开发机无 build/ 场景")
    def test_alias(self):
        xml, report = resolve_scene("ur5-pick-planned", REPO)
        self.assertEqual(xml.name, "ur5-pick-scene.xml")
        self.assertTrue(report.is_file())

    def test_missing(self):
        with self.assertRaises(PlanError) as ctx:
            resolve_scene("no-such-scene", REPO)
        self.assertEqual(ctx.exception.code, "SCENE-NOT-FOUND")


@unittest.skipUnless(HAS_MUJOCO, "未安装 mujoco")
@unittest.skipUnless((REPO / "build/models/ur5-pick-scene.xml").is_file(),
                     "开发机无 build/ 场景")
class PlanPickTest(unittest.TestCase):
    def test_plan_labels_and_ik(self):
        result = plan_pick(scene_id="ur5-pick-planned", target_id="box_01")
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertTrue(result["planned"])
        self.assertFalse(result["physics"])
        self.assertEqual(result["backend"], "kinematic")
        self.assertLess(result["ik"]["error_m"], 1e-3)
        self.assertGreater(result["limits"]["min_margin_deg"], 0)
        self.assertIn("json", result["report"])

    def test_unknown_target(self):
        with self.assertRaises(PlanError) as ctx:
            plan_pick(scene_id="ur5-pick-planned", target_id="nope")
        self.assertEqual(ctx.exception.code, "TARGET-NOT-FOUND")


if __name__ == "__main__":
    unittest.main()