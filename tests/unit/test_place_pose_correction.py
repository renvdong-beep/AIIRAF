"""`place` 放置点纠偏的契约测试（AGENTS.md 2.8：成功路径 + 拒绝路径）。

背景（docs/debug/2026-09-24-joint-model-dog-arm.md §11.23(48)）：托盘改为随载体运动后，
放置偏移 = 停靠误差 + 名义/实测量差，而放置四段航点是构建期按**名义停靠位姿**解出的 ⇒
三轮实测 offset 0.052625625 / 0.058340468 / 0.056955541（判据 0.06）⇒ 余量最薄 1.7 mm。
本文件测的是"把放置点纠到运行期实测位姿"这一步的**声明解析与纠偏量**（纯函数，唯一口径）。
"""

import unittest

from iraf_adapters.mujoco.payload_facts import (
    resolve_place_alignment_correction, resolve_place_pose_correction, resolve_touchdown_correction)

NOMINAL = [0.43486632, 0.0, 0.33288002]


class MeasureOnlyTests(unittest.TestCase):
    def test_off_does_not_measure(self):
        """mode=off ⇒ 不测量（与改动前逐位一致），返回里没有 delta。"""
        report = resolve_place_pose_correction({"mode": "off"}, NOMINAL, [1.0, 1.0, 1.0])
        self.assertEqual(report, {"mode": "off", "applied": False})

    def test_measure_only_reports_delta(self):
        """measure_only ⇒ 实测并留痕，但不施加。"""
        live = [NOMINAL[0] + 0.03, NOMINAL[1] - 0.004, NOMINAL[2] + 0.0015]
        report = resolve_place_pose_correction({"mode": "measure_only", "max_lateral_m": 0.05},
                                               NOMINAL, live)
        self.assertEqual(report["mode"], "measure_only")
        self.assertFalse(report["applied"])
        self.assertEqual(report["delta_world_m"], [0.03, -0.004, 0.0015])
        self.assertAlmostEqual(report["lateral_m"], (0.03 ** 2 + 0.004 ** 2) ** 0.5, places=9)
        self.assertAlmostEqual(report["vertical_m"], 0.0015, places=9)
        self.assertEqual(report["max_lateral_m"], 0.05)


class RejectionTests(unittest.TestCase):
    """每条拒绝路径都必须**显式失败**（不得静默按名义位姿照放、不得静默截断）。"""

    def test_missing_declaration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "缺少 place_pose_correction 声明"):
            resolve_place_pose_correction(None, NOMINAL, NOMINAL)
        with self.assertRaisesRegex(ValueError, "缺少 place_pose_correction 声明"):
            resolve_place_pose_correction({}, NOMINAL, NOMINAL)

    def test_unknown_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "mode 必须是"):
            resolve_place_pose_correction({"mode": "auto", "max_lateral_m": 0.05}, NOMINAL, NOMINAL)

    def test_missing_nominal_pose_is_rejected(self):
        """接收体记录里没有构建期名义位姿 ⇒ 无法量纠偏量 ⇒ 拒绝（不是"按名义位姿照放"）。"""
        with self.assertRaisesRegex(ValueError, "缺少 nominal_pose_m"):
            resolve_place_pose_correction({"mode": "measure_only", "max_lateral_m": 0.05},
                                          None, NOMINAL)

    def test_exceeding_declared_bound_is_rejected(self):
        """实测偏离超过声明上限 ⇒ 拒绝（该量说明载体没停到位；不静默截断）。"""
        live = [NOMINAL[0] + 0.08, NOMINAL[1], NOMINAL[2]]
        with self.assertRaisesRegex(ValueError, "声明上限"):
            resolve_place_pose_correction({"mode": "measure_only", "max_lateral_m": 0.05},
                                          NOMINAL, live)

    def test_apply_mode_requires_ik_parameters(self):
        """施加模式（lateral_only/full_pose）必须声明 IK 参数 —— 缺一个即显式失败。"""
        for mode in ("lateral_only", "full_pose"):
            with self.assertRaisesRegex(ValueError, "必须声明正的 ik_iterations"):
                resolve_place_pose_correction({"mode": mode, "max_lateral_m": 0.05}, NOMINAL, NOMINAL)
            with self.assertRaisesRegex(ValueError, "必须声明正的 max_residual_m"):
                resolve_place_pose_correction(
                    {"mode": mode, "max_lateral_m": 0.05, "ik_iterations": 800, "ik_step": 0.5},
                    NOMINAL, NOMINAL)

    def test_apply_mode_is_accepted_with_parameters(self):
        """参数齐备时**解析**通过（解析层不施加 ⇒ applied 保持 False，施加由后端完成并另写证据）。"""
        report = resolve_place_pose_correction(
            {"mode": "lateral_only", "max_lateral_m": 0.05, "ik_iterations": 800,
             "ik_step": 0.5, "max_residual_m": 0.0001},
            NOMINAL, [NOMINAL[0] + 0.03, NOMINAL[1], NOMINAL[2]])
        self.assertEqual(report["mode"], "lateral_only")
        self.assertFalse(report["applied"])
        self.assertAlmostEqual(report["lateral_m"], 0.03, places=9)
        self.assertEqual(report["ik_iterations"], 800)
        self.assertAlmostEqual(report["max_residual_m"], 0.0001, places=12)


class TouchdownCorrectionTests(unittest.TestCase):
    """竖向触地纠偏（2026-09-29 §11.25(f-4)）：要消的是**载荷底面与承载面的实测间隙**。

    为什么另开一组用例：原先那条 `resolve_place_pose_correction` 的量来自"托盘位姿 − 名义位姿"，
    实测施加后**更差**（0.051933449 → 0.059755439）⇒ 误差主项不在托盘位姿，而在方块在夹口里下滑
    （PLACE_TRACE 实测放置段内摆动 24.7 mm）。所以这一组钉住"按载荷底面算、按声明上下限判、缺失即失败"。
    """

    DECL = {"mode": "touchdown", "touch_clearance_m": 0.002,
            "max_vertical_m": 0.05, "residual_tolerance_m": 0.001}

    def test_needs_correction_when_payload_hangs_high(self):
        # 实测的典型情形：方块在承载面上方 13.3 mm ⇒ 需要往下走 11.3 mm
        report = resolve_touchdown_correction(self.DECL, 0.356372, 0.343000)
        self.assertTrue(report["applied"])
        self.assertAlmostEqual(report["gap_m"], 0.013372, places=9)
        self.assertAlmostEqual(report["delta_z_m"], -0.011372, places=9)
        self.assertLess(report["delta_z_m"], 0.0)

    def test_no_correction_when_within_tolerance(self):
        report = resolve_touchdown_correction(self.DECL, 0.3452, 0.3432)   # 间隙 2 mm = 目标
        self.assertFalse(report["applied"])
        self.assertAlmostEqual(report["residual_m"], 0.0, places=9)

    def test_beyond_limit_is_refused_explicitly(self):
        with self.assertRaises(ValueError) as ctx:
            resolve_touchdown_correction({**self.DECL, "max_vertical_m": 0.01}, 0.3633, 0.3430)
        self.assertIn("超过声明上限", str(ctx.exception))
        self.assertIn("不静默截断", str(ctx.exception))

    def test_missing_declaration_keys_fail(self):
        for key in ("touch_clearance_m", "max_vertical_m", "residual_tolerance_m"):
            with self.subTest(missing=key):
                broken = {k: v for k, v in self.DECL.items() if k != key}
                with self.assertRaises(ValueError) as ctx:
                    resolve_touchdown_correction(broken, 0.3452, 0.3432)
                self.assertIn(key, str(ctx.exception))

    def test_non_finite_measurement_fails(self):
        with self.assertRaises(ValueError):
            resolve_touchdown_correction(self.DECL, float("nan"), 0.3432)


class PlaceAlignmentCorrectionTests(unittest.TestCase):
    """横向放置纠偏（2026-09-29 §11.25(f-5)）：要消的是"载荷中心 − 托盘中心"。

    实测来源：`PLACE_TRACE` 显示释放前就已偏 47.4 mm（主因是搬运段在夹口里**侧滑 21.6 mm**：
    start 27.4 mm → after_transit 49.0 mm），释放瞬间再加 13.7 mm ⇒ 最终 61.8 mm 破 0.06 判据。
    故 delta 必须取"载荷相对托盘的位置"，而不是"托盘位姿差"（后者 D2 实测施加后更差）。
    """

    DECL = {"mode": "touchdown", "max_lateral_m": 0.05, "lateral_tolerance_m": 0.005}

    def test_delta_points_to_tray_center(self):
        report = resolve_place_alignment_correction(self.DECL, [0.449541, 0.048539],
                                                    [0.440934, 0.001917])
        self.assertTrue(report["applied"])
        self.assertAlmostEqual(report["lateral_m"], 0.047409823, places=8)
        # 载荷在托盘 +y 侧 ⇒ delta 应为 −y 方向（把载荷挪回中心）
        self.assertLess(report["delta_xy_m"][1], 0.0)
        self.assertAlmostEqual(report["delta_xy_m"][1], -0.046622, places=6)

    def test_within_tolerance_not_applied(self):
        report = resolve_place_alignment_correction(self.DECL, [0.4410, 0.0020],
                                                    [0.440934, 0.001917])
        self.assertFalse(report["applied"])
        self.assertLess(report["lateral_m"], self.DECL["lateral_tolerance_m"])

    def test_beyond_cap_is_refused(self):
        with self.assertRaises(ValueError) as ctx:
            resolve_place_alignment_correction({**self.DECL, "max_lateral_m": 0.03},
                                               [0.449541, 0.048539], [0.440934, 0.001917])
        self.assertIn("超过声明上限", str(ctx.exception))
        self.assertIn("不静默截断", str(ctx.exception))

    def test_missing_keys_and_bad_shape_fail(self):
        for key in ("lateral_tolerance_m", "max_lateral_m"):
            with self.subTest(missing=key):
                broken = {k: v for k, v in self.DECL.items() if k != key}
                with self.assertRaises(ValueError) as ctx:
                    resolve_place_alignment_correction(broken, [0.45, 0.05], [0.44, 0.0])
                self.assertIn(key, str(ctx.exception))
        with self.assertRaises(ValueError):
            resolve_place_alignment_correction(self.DECL, [0.45, 0.05, 0.0], [0.44, 0.0])


if __name__ == "__main__":
    unittest.main()
