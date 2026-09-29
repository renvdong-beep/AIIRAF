import unittest

from iraf_skills.common.vision import depth_to_point_cloud, filter_grasp_candidates, sample_points


class VisionProcessingTests(unittest.TestCase):
    def test_depth_projection_and_invalid_filter(self):
        """投影 + 无效/超距过滤：**期望值按实现口径逐点核对**。

        4 个像素（depth_scale=1000 ⇒ 单位 m，max_depth_m=2）：
          (0,0)=1000 → z=1.000 ✓ → (0.00, 0.00, 1.000)
          (0,1)=   0 → z=0     ✗ 无效（z≤0）
          (1,0)=1000 → z=1.000 ✓ → (0.00, 0.01, 1.000)   ← 原期望漏了这一点（2026-09-29 修正）
          (1,1)=3000 → z=3.000 ✗ 超 max_depth_m=2
        """
        points = depth_to_point_cloud([[1000, 0], [1000, 3000]], {"fx": 100, "fy": 100, "cx": 0, "cy": 0}, depth_scale=1000, max_depth_m=2)
        self.assertEqual([(0.0, 0.0, 1.0), (0.0, 0.01, 1.0)], points)
        # 超距像素必须被过滤（上一条断言已隐含，这里显式钉住"只出 2 点"）
        self.assertEqual(len(points), 2)

    def test_sampling_is_deterministic(self):
        points = [(float(i), 0.0, 1.0) for i in range(3)]
        self.assertEqual(sample_points(points, 5), sample_points(points, 5))

    def test_collision_filter(self):
        candidates = [{"id": "hit", "position": [0.0, 0.0, 0.0]}, {"id": "free", "position": [1.0, 0.0, 0.0]}]
        accepted = filter_grasp_candidates(candidates, [(0.0, 0.0, 0.0)], 0.01)
        self.assertEqual(["free"], [item["id"] for item in accepted])


if __name__ == "__main__":
    unittest.main()
