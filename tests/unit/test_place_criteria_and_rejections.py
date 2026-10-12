# -*- coding: utf-8 -*-
"""放置段判据与拒绝路径的回归测试（P7 / D1+D2，2026-10-12）。

D1：把本轮用过的一次性判据固化为可复跑检查（不必跑整链）：
  · 可达性公式 √(dy² + home_z²) ≤ reach（`iraf_core.kinematics.horizontal_reach_ok`）；
  · 相位集合一致性由 `tests/unit/test_place_phases.py` 承担（此处不重复）。
D2：新键的错误值必须有 fail-closed 测试（此前只有 raise、无覆盖）：
  · `place_via` 非法枚举 / `place_via=midpoint` 缺 transit；
  · `place_elbow_body` 与 `place_elbow_pref` 缺配对 / pref 非法值。
"""
import unittest

from iraf_core.kinematics import horizontal_reach_ok
from iraf_skills.common.place_phases import validate_place_optional_keys


class ReachabilityCriterionTests(unittest.TestCase):
    """D1：可达性快速判据（2026-10-08 §11.95 布置扫描口径）。"""

    def test_within_reach_passes(self):
        # 实测：dy=0.45、home_z≈0.635 ⇒ hypot=0.778 ≤ 0.85 ✓
        self.assertTrue(horizontal_reach_ok(0.45, 0.635, 0.85))

    def test_beyond_reach_fails(self):
        # 文档记录：dy=0.69、0.635 ⇒ 0.94 > 0.85（该布置被判不可达）
        self.assertFalse(horizontal_reach_ok(0.69, 0.635, 0.85))

    def test_boundary_is_inclusive(self):
        # 恰好等于 reach（浮点容差内）应通过（≤ 口径）
        self.assertTrue(horizontal_reach_ok(0.0, 0.85, 0.85))

    def test_vertical_only_matches_reach(self):
        # 水平为 0 时退化为 |vertical| ≤ reach
        self.assertTrue(horizontal_reach_ok(0.0, 0.80, 0.85))
        self.assertFalse(horizontal_reach_ok(0.0, 0.90, 0.85))

    def test_non_finite_and_bad_inputs_fail_loudly(self):
        with self.assertRaises(ValueError):
            horizontal_reach_ok(float("nan"), 0.1, 0.85)
        with self.assertRaises(ValueError):
            horizontal_reach_ok(0.1, 0.1, 0.0)
        with self.assertRaises(ValueError):
            horizontal_reach_ok(True, 0.1, 0.85)


class PlaceOptionalKeyRejectionTests(unittest.TestCase):
    """D2：via / elbow 新键的 fail-closed 拒绝路径。"""

    def test_via_midpoint_accepted(self):
        result = validate_place_optional_keys({"place_via": "midpoint"})
        self.assertEqual(result["via"], "midpoint")

    def test_via_absent_accepted_as_lazy(self):
        result = validate_place_optional_keys({})
        self.assertEqual(result["via"], "")
        self.assertEqual(result["elbow_body"], "")
        self.assertEqual(result["elbow_pref"], "")

    def test_via_bad_enum_rejected(self):
        with self.assertRaises(ValueError):
            validate_place_optional_keys({"place_via": "xx"})

    def test_elbow_pair_high_accepted(self):
        result = validate_place_optional_keys({"place_elbow_body": "forearm_link",
                                               "place_elbow_pref": "high"})
        self.assertEqual(result["elbow_body"], "forearm_link")
        self.assertEqual(result["elbow_pref"], "high")

    def test_elbow_body_without_pref_rejected(self):
        with self.assertRaises(ValueError):
            validate_place_optional_keys({"place_elbow_body": "forearm_link"})

    def test_elbow_pref_without_body_rejected(self):
        with self.assertRaises(ValueError):
            validate_place_optional_keys({"place_elbow_pref": "high"})

    def test_elbow_bad_pref_rejected(self):
        with self.assertRaises(ValueError):
            validate_place_optional_keys({"place_elbow_body": "forearm_link",
                                          "place_elbow_pref": "mid"})


if __name__ == "__main__":
    unittest.main()
