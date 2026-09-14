import unittest

from iraf_skills.common.vision import depth_to_point_cloud, filter_grasp_candidates, sample_points


class VisionProcessingTests(unittest.TestCase):
    def test_depth_projection_and_invalid_filter(self):
        points = depth_to_point_cloud([[1000, 0], [1000, 3000]], {"fx": 100, "fy": 100, "cx": 0, "cy": 0}, depth_scale=1000, max_depth_m=2)
        self.assertEqual([(0.0, 0.0, 1.0)], points)

    def test_sampling_is_deterministic(self):
        points = [(float(i), 0.0, 1.0) for i in range(3)]
        self.assertEqual(sample_points(points, 5), sample_points(points, 5))

    def test_collision_filter(self):
        candidates = [{"id": "hit", "position": [0.0, 0.0, 0.0]}, {"id": "free", "position": [1.0, 0.0, 0.0]}]
        accepted = filter_grasp_candidates(candidates, [(0.0, 0.0, 0.0)], 0.01)
        self.assertEqual(["free"], [item["id"] for item in accepted])


if __name__ == "__main__":
    unittest.main()
