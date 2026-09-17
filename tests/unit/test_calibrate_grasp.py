import math
import unittest

from iraf_skills.common.manipulation import CalibrateGraspProvider


class FakeBackend:
    def calibrate_grasp(self, inputs, lease):
        return {
            "position_source": "geom_xpos",
            "finger_separation_m": 0.0203,
            "tcp_offset_from_link6_m": [0.08, 0.0, 0.01],
            "approach_axis_world": [0.99, 0.0, 0.1],
            "recommended_pregrasp_offset_m": 0.04,
            "grasp_center_m": [0.1, 0.0, 0.2],
        }


class CalibrateGraspProviderTests(unittest.TestCase):
    def test_returns_auditable_geometry_evidence(self):
        result = CalibrateGraspProvider(None, FakeBackend()).execute({}, None)
        self.assertTrue(result["accepted"])
        evidence = result["evidence"]
        self.assertEqual(evidence["position_source"], "geom_xpos")
        self.assertGreater(evidence["finger_separation_m"], 0.0)
        self.assertGreater(evidence["recommended_pregrasp_offset_m"], 0.0)
        self.assertTrue(math.isfinite(evidence["tcp_offset_from_link6_m"][0]))


if __name__ == "__main__":
    unittest.main()
