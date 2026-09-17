"""随机化采样器的契约测试（机器人无关层，不依赖 MuJoCo）。

覆盖的契约：
- 圆盘面积均匀（半径分布不是均匀的：E[r] = 2R/3，而非 R/2）；
- 采样点严格落在半径内，绝不越界；
- z 严格贴台面（top_z + half_size）；
- 同一 seed 完全可复现，不同 seed 序列不同；
- yaw 本期固定 0；
- 拒绝原因分类器把退化构型与泛化对齐失败区分开；
- 统计量在空输入时返回 None 而不是伪造 0。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_core.randomization import (  # noqa: E402
    RandomizationError,
    RandomizationSpec,
    UniformDiscSampler,
    classify_rejection,
    describe_distribution,
    euler_deg_for_yaw,
    rejection_counts,
)


class RandomizationSpecTests(unittest.TestCase):
    def test_rejects_non_positive_radius(self):
        for radius in (0.0, -0.01):
            with self.assertRaises(RandomizationError):
                RandomizationSpec(xy_radius_m=radius)

    def test_rejects_empty_yaw_choices(self):
        with self.assertRaises(RandomizationError):
            RandomizationSpec(xy_radius_m=0.05, yaw_choices_deg=())

    def test_default_yaw_is_fixed_to_zero(self):
        """本期 yaw 不随机，默认必须只有 0 这一项。"""
        spec = RandomizationSpec(xy_radius_m=0.05)
        self.assertEqual((0.0,), spec.yaw_choices_deg)
        self.assertFalse(spec.yaw_randomized)

    def test_from_config_default_radius_is_object_diameter(self):
        """未声明时半径 = 物品直径 = 2 * half_size。"""
        spec = RandomizationSpec.from_config({}, half_size_m=0.025)
        self.assertAlmostEqual(0.05, spec.xy_radius_m)
        self.assertEqual((0.0,), spec.yaw_choices_deg)

    def test_from_config_honours_declared_radius(self):
        config = {"grasp": {"randomization": {"xy_radius_m": 0.02, "max_attempts": 5}}}
        spec = RandomizationSpec.from_config(config, half_size_m=0.025)
        self.assertAlmostEqual(0.02, spec.xy_radius_m)
        self.assertEqual(5, spec.max_attempts)

    def test_to_dict_declares_axis_scope(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        payload = spec.to_dict()
        self.assertEqual(["xy"], payload["randomized_axes"])
        self.assertIn("yaw", payload["fixed_axes"])
        self.assertIn("yaw", payload["deferred_axes"])


class UniformDiscSamplerTests(unittest.TestCase):
    def test_all_samples_within_radius(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 1234)
        for index in range(400):
            sample = sampler.draw([0.19, 0.0], 0.0, 0.025, index, 1)
            distance = math.hypot(sample.x_m - 0.19, sample.y_m - 0.0)
            self.assertLessEqual(distance, 0.05 + 1e-12)
            self.assertAlmostEqual(distance, sample.offset_m, places=12)

    def test_z_is_strictly_on_table(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 7)
        for index in range(50):
            sample = sampler.draw([0.19, 0.0], 0.01, 0.025, index, 1)
            self.assertAlmostEqual(0.035, sample.z_m, places=12)

    def test_area_uniform_not_radius_uniform(self):
        """面积均匀采样的 E[r] 应为 2R/3；半径均匀会是 R/2，可据此判别实现。"""
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 99)
        radii = [
            sampler.draw([0.0, 0.0], 0.0, 0.025, index, 1).offset_m
            for index in range(4000)
        ]
        mean = sum(radii) / len(radii)
        expected = 2.0 * 0.05 / 3.0
        self.assertAlmostEqual(expected, mean, delta=0.002)
        # 与"半径均匀"的期望值区分开，避免实现被误改。
        self.assertGreater(abs(mean - 0.025), 0.004)

    def test_same_seed_is_reproducible(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        first = UniformDiscSampler(spec, 4242)
        second = UniformDiscSampler(spec, 4242)
        for index in range(20):
            a = first.draw([0.19, 0.0], 0.0, 0.025, index, 1).to_dict()
            b = second.draw([0.19, 0.0], 0.0, 0.025, index, 1).to_dict()
            self.assertEqual(a, b)

    def test_different_seeds_diverge(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        a = UniformDiscSampler(spec, 1).draw([0.19, 0.0], 0.0, 0.025, 0, 1)
        b = UniformDiscSampler(spec, 2).draw([0.19, 0.0], 0.0, 0.025, 0, 1)
        self.assertNotAlmostEqual(a.offset_m, b.offset_m, places=9)

    def test_yaw_fixed_to_zero_in_samples(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 31)
        for index in range(30):
            self.assertEqual(0.0, sampler.draw([0.19, 0.0], 0.0, 0.025, index, 1).yaw_deg)

    def test_rejects_malformed_nominal_xy(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 5)
        with self.assertRaises(RandomizationError):
            sampler.draw([0.19], 0.0, 0.025, 0, 1)

    def test_draws_counter_advances(self):
        spec = RandomizationSpec(xy_radius_m=0.05)
        sampler = UniformDiscSampler(spec, 8)
        self.assertEqual(0, sampler.draws)
        sampler.draw([0.19, 0.0], 0.0, 0.025, 0, 1)
        sampler.draw([0.19, 0.0], 0.0, 0.025, 1, 1)
        self.assertEqual(2, sampler.draws)


class EulerConversionTests(unittest.TestCase):
    def test_pure_yaw_maps_to_rz_only(self):
        self.assertEqual([0.0, 0.0, 90.0], euler_deg_for_yaw(90.0))
        self.assertEqual([0.0, 0.0, 0.0], euler_deg_for_yaw(0.0))


class RejectionClassificationTests(unittest.TestCase):
    def test_degenerate_is_matched(self):
        self.assertEqual(
            "IK_DEGENERATE", classify_rejection("关节处于退化构型，雅可比秩亏")
        )

    def test_alignment_tolerance_is_matched(self):
        self.assertEqual(
            "ALIGNMENT_TOLERANCE",
            classify_rejection("末端未到达目标抓取位姿: distance=0.018909m"),
        )

    def test_clearance_failure_is_matched(self):
        self.assertEqual(
            "IK_NOT_CONVERGED", classify_rejection("指尖离台间隙配平未收敛: tip_z=...")
        )

    def test_unknown_reason_is_not_swallowed(self):
        self.assertEqual("OTHER", classify_rejection("某种未归类的新错误"))

    def test_empty_reason_is_other(self):
        self.assertEqual("OTHER", classify_rejection(None))

    def test_counts_ignore_empty_categories(self):
        """空类别（通过的样本）不得被计入失败统计。"""
        records = [
            {"failure_category": "IK_DEGENERATE"},
            {"failure_category": "IK_DEGENERATE"},
            {"failure_category": "ALIGNMENT_TOLERANCE"},
            {"failure_category": ""},
        ]
        self.assertEqual(
            {"ALIGNMENT_TOLERANCE": 1, "IK_DEGENERATE": 2},
            rejection_counts(records, key="failure_category"),
        )

    def test_counts_use_default_key(self):
        records = [
            {"rejection_category": "IK_DEGENERATE"},
            {"rejection_category": ""},
            {},
        ]
        self.assertEqual({"IK_DEGENERATE": 1}, rejection_counts(records))

    def test_counts_on_empty_input(self):
        self.assertEqual({}, rejection_counts([]))


class DistributionTests(unittest.TestCase):
    def test_empty_input_returns_none_not_zero(self):
        """空输入必须返回 None：返回 0 会伪造出"零误差"的假指标。"""
        self.assertIsNone(describe_distribution([]))
        self.assertIsNone(describe_distribution([None, None]))

    def test_single_value_has_zero_std(self):
        result = describe_distribution([0.004])
        self.assertEqual(1, result["count"])
        self.assertEqual(0.0, result["std"])
        self.assertEqual(0.004, result["mean"])

    def test_known_statistics(self):
        result = describe_distribution([1.0, 2.0, 3.0, 4.0])
        self.assertEqual(4, result["count"])
        self.assertAlmostEqual(2.5, result["mean"])
        self.assertAlmostEqual(1.290994449, result["std"], places=8)
        self.assertEqual(1.0, result["min"])
        self.assertEqual(4.0, result["max"])

    def test_none_values_are_filtered(self):
        result = describe_distribution([1.0, None, 3.0])
        self.assertEqual(2, result["count"])
        self.assertAlmostEqual(2.0, result["mean"])


if __name__ == "__main__":
    unittest.main()
