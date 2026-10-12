# -*- coding: utf-8 -*-
"""位置类声明**交叉校验**的回归测试（P6 / A3，2026-10-12）。

校验的是纯判定逻辑：站位帧派生的目标 vs 声明的字面量，差 > 容差即失败。
这里用与 `scene_builder` 同一范式的独立实现锁定口径（不跑整链）。
"""
import unittest

import numpy as np


def cross_check(declared_literal, derived, tolerance_m=0.001):
    """与 scene_builder A3 同一口径：返回 (ok, delta_m)。"""
    declared = np.asarray(declared_literal, dtype=float)
    derived_vec = np.asarray(derived, dtype=float)
    delta = float(np.linalg.norm(declared - derived_vec))
    return bool(delta <= float(tolerance_m)), delta


class TargetCrossCheckTests(unittest.TestCase):
    def test_identical_passes(self):
        ok, delta = cross_check([0.15, 0.45, 0.375372], [0.15, 0.45, 0.375372])
        self.assertTrue(ok)
        self.assertEqual(delta, 0.0)

    def test_contradiction_fails_with_delta(self):
        ok, delta = cross_check([0.30, 0.45, 0.375372], [0.15, 0.45, 0.375372])
        self.assertFalse(ok)
        self.assertAlmostEqual(delta, 0.15, places=9)

    def test_within_tolerance_passes(self):
        ok, _ = cross_check([0.15, 0.45, 0.375372], [0.15, 0.45, 0.375872], tolerance_m=0.001)
        self.assertTrue(ok)

    def test_just_over_tolerance_fails(self):
        ok, delta = cross_check([0.15, 0.45, 0.375372], [0.15, 0.45, 0.377372], tolerance_m=0.001)
        self.assertFalse(ok)
        self.assertAlmostEqual(delta, 0.002, places=9)

    def test_station_move_without_literal_update_fails(self):
        """核心场景：站点移动 0.05 m 而字面量未改 ⇒ 必须被拦下。"""
        ok, delta = cross_check([0.15, 0.45, 0.375372], [0.15, 0.50, 0.375372])
        self.assertFalse(ok)
        self.assertAlmostEqual(delta, 0.05, places=9)


if __name__ == "__main__":
    unittest.main()
