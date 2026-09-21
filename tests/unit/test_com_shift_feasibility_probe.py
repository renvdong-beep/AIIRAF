"""契约测试：重心平移可行性只读探针（步骤 02b 决策包第 3 轮）。

覆盖三件事：

1. **纯函数层**（网格、区域度量、交集、射线扫描）—— 用**合成分配器**给出解析可验证的正例对照：
   半平面判据的可行点数是解析值、两个相反半平面的交集必须为 0、已知势垒距离必须被射线扫到。
   没有这些对照，「交集为空」就分不清是机制结果还是门禁恒空。
2. **fail-closed 形状**：缺 `gait.legs` / 缺 `balance` / 缺 `robot.profile` / Profile 缺
   `spec.model.trunk_body` / 缺 `gait.verification.height_target_m` / 模型不存在 / 网格参数非法
   ⇒ 各自的退出码，且**不得**写任何产物。
3. **现场构建产物上的结构不变量**（`build/` 是 gitignore 证据区，不在场则 `skip` 并注明理由）：
   三态划分守恒、有截断⇔有残差、形心 = 三个支撑足 xy 的均值、最近可行点与可行点数自洽。

判据只断**结构不变量**，不硬编码实测数字（声明/场景一改，快照式断言就会假失败）。
"""

import contextlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "probe_go2_com_shift_feasibility.py"
BASELINE = ROOT / "config/go2_loopback.yaml"
MODEL = ROOT / "build/scenes/handoff_lab/handoff_lab.xml"

# 合成分配器的解析参数（与真实半轴无关，只用于把「网格/交集/射线」的机器本身钉住）
HALF_A = 0.20
HALF_B = 0.14


def _load_probe():
    spec = importlib.util.spec_from_file_location("probe_com_shift_feasibility", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


try:
    _PROBE = _load_probe()
    _LOAD_ERROR = None
except Exception as exc:  # 环境缺 mujoco/numpy/yaml ⇒ 显式跳过，不制造假失败
    _PROBE = None
    _LOAD_ERROR = "%s: %s" % (type(exc).__name__, exc)


def _row(dx, dy, clamped):
    """合成分配器的一条输出（只填纯函数消费的键）。"""
    return {
        "offset_m": [float(dx), float(dy)],
        "clamped_legs": list(clamped),
        "min_normal_force_n": 0.0 if clamped else 1.0,
        "min_normal_over_mg": 0.0 if clamped else 0.01,
        "normal_sum_n": 0.0,
        "residual_force_n": 0.5 if clamped else 1.0e-15,
    }


@unittest.skipIf(_PROBE is None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
class SquareOffsetsTests(unittest.TestCase):
    def test_grid_is_deterministic_and_contains_origin(self):
        offsets, resolution = _PROBE.square_offsets(0.008, 0.002)
        self.assertEqual(len(offsets), 81)  # (2*4+1)^2
        self.assertEqual(resolution, 0.002)
        self.assertIn((0.0, 0.0), offsets)
        again, _ = _PROBE.square_offsets(0.008, 0.002)
        self.assertEqual(offsets, again)  # 确定性顺序（证据可复算）

    def test_degenerate_grid_requests_are_exit_code_1(self):
        for kwargs in ((0.0, 0.004), (-0.01, 0.004), (0.08, 0.0), (0.08, -0.004), (0.002, 0.004)):
            with self.assertRaises(_PROBE.ProbeError) as ctx:
                _PROBE.square_offsets(*kwargs)
            self.assertEqual(ctx.exception.code, 1, "参数 %r 应显式失败" % (kwargs,))


@unittest.skipIf(_PROBE is None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
class RegionAndIntersectionTests(unittest.TestCase):
    """合成分配器：可行 iff `HALF_B*dx + HALF_A*dy < 0`（半平面，点数可解析验证）。"""

    @staticmethod
    def _probe(dx, dy):
        clamped = [] if (HALF_B * dx + HALF_A * dy) < 0.0 else ["RR"]
        return _row(dx, dy, clamped)

    @staticmethod
    def _opposite_probe(dx, dy):
        clamped = [] if (HALF_B * dx + HALF_A * dy) > 0.0 else ["RR"]
        return _row(dx, dy, clamped)

    def test_region_metrics_three_states_partition_the_grid(self):
        offsets, resolution = _PROBE.square_offsets(0.01, 0.005)

        def flaky(dx, dy):
            if dx == 0.0 and dy == 0.0:
                raise RuntimeError("合成：分配器显式失败")
            return self._probe(dx, dy)

        rows = _PROBE.scan_region(flaky, offsets)
        metrics = _PROBE.region_metrics(rows, resolution)
        self.assertEqual(metrics["point_count"], len(offsets))
        self.assertEqual(metrics["allocator_error_count"], 1)
        self.assertEqual(
            metrics["feasible_count"] + metrics["clamped_count"] + metrics["allocator_error_count"],
            metrics["point_count"])
        # 解析对照：原点不参与时，可行点数 = 严格满足半平面的网格点数
        expected = sum(1 for dx, dy in offsets
                       if (dx, dy) != (0.0, 0.0) and (HALF_B * dx + HALF_A * dy) < 0.0)
        self.assertEqual(metrics["feasible_count"], expected)
        self.assertEqual(metrics["area_lower_bound_m2"],
                         metrics["feasible_count"] * resolution ** 2)
        nearest = metrics["nearest_feasible"]
        self.assertIsNotNone(nearest)
        self.assertAlmostEqual(
            nearest["distance_m"], math.hypot(*nearest["offset_m"]), places=12)

    def test_region_metrics_reports_no_nearest_when_nothing_is_feasible(self):
        offsets, resolution = _PROBE.square_offsets(0.01, 0.005)
        rows = _PROBE.scan_region(lambda dx, dy: _row(dx, dy, ["RR"]), offsets)
        metrics = _PROBE.region_metrics(rows, resolution)
        self.assertEqual(metrics["feasible_count"], 0)
        self.assertIsNone(metrics["nearest_feasible"])

    def test_intersection_is_empty_for_opposite_half_planes(self):
        """负向：两个相反半平面的交集必须为空（否则「交集为空」这类结论不可信）。"""
        offsets, resolution = _PROBE.square_offsets(0.01, 0.005)
        left = _PROBE.scan_region(self._probe, offsets)
        right = _PROBE.scan_region(self._opposite_probe, offsets)
        result = _PROBE.intersect_regions({"FL": left, "RR": right}, resolution)
        self.assertEqual(result["intersection_point_count"], 0)
        self.assertEqual(result["intersection_area_lower_bound_m2"], 0.0)

    def test_intersection_is_full_for_identical_regions(self):
        """正例对照：同一区域与自己求交 = 该区域全部可行点（证明交集机器不是恒空）。"""
        offsets, resolution = _PROBE.square_offsets(0.01, 0.005)
        rows = _PROBE.scan_region(self._probe, offsets)
        metrics = _PROBE.region_metrics(rows, resolution)
        result = _PROBE.intersect_regions({"FL": rows, "FR": rows}, resolution)
        self.assertEqual(result["intersection_point_count"], metrics["feasible_count"])
        self.assertGreater(result["intersection_point_count"], 0)
        self.assertEqual(result["pairwise_feasible_point_counts"]["FL+FR"],
                         metrics["feasible_count"])


@unittest.skipIf(_PROBE is None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
class RayScanTests(unittest.TestCase):
    def test_ray_scan_finds_analytic_barrier(self):
        """正例对照：势垒在 0.0073 m（解析）⇒ 首个无截断距离 = 0.0075 m（步长 0.0005）。"""
        barrier = 0.0073
        probe = lambda distance: _row(0.0, 0.0, [] if distance > barrier else ["RR"])
        scan = _PROBE.ray_scan(lambda dx, dy: probe(math.hypot(dx, dy)), (1.0, 0.0), 0.02, 0.0005)
        self.assertEqual(len(scan["samples"]), 41)
        self.assertAlmostEqual(_PROBE.first_clamp_free_distance(scan), 0.0075, places=12)

    def test_ray_scan_reports_none_when_never_free(self):
        scan = _PROBE.ray_scan(lambda dx, dy: _row(dx, dy, ["RR"]), (0.0, 1.0), 0.01, 0.001)
        self.assertIsNone(_PROBE.first_clamp_free_distance(scan))

    def test_ray_scan_rejects_zero_direction(self):
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.ray_scan(lambda dx, dy: _row(dx, dy, []), (0.0, 0.0), 0.01, 0.001)
        self.assertEqual(ctx.exception.code, 1)

    def test_ray_scan_records_allocator_errors_as_errors(self):
        def probe(dx, dy):
            if dx > 0.004:
                raise RuntimeError("合成：分配器显式失败")
            return _row(dx, dy, [])

        scan = _PROBE.ray_scan(probe, (1.0, 0.0), 0.01, 0.001)
        errors = [item for item in scan["samples"] if item.get("allocator_error") is not None]
        self.assertTrue(errors, "分配器显式失败必须被登记，而不是混进「无截断」")
        self.assertIsNotNone(_PROBE.first_clamp_free_distance(scan))

    def test_triangle_centroid_is_the_mean_of_support_feet(self):
        centroid = _PROBE.triangle_centroid([[0.2, 0.14], [0.2, -0.14], [-0.2, -0.14]])
        self.assertAlmostEqual(float(centroid[0]), 0.06666666666666667, places=12)
        self.assertAlmostEqual(float(centroid[1]), -0.04666666666666667, places=12)


@unittest.skipIf(_PROBE is None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
class FailClosedTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)
        self.base = yaml.safe_load(BASELINE.read_text(encoding="utf-8"))

    def _write_baseline(self, mutate, name="baseline.yaml"):
        data = yaml.safe_load(yaml.safe_dump(self.base, allow_unicode=True))
        mutate(data)
        path = self.tmp / name
        path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
        return path

    def test_positive_control_real_baseline_loads(self):
        """正例对照：真基线必须解析成功（否则下面的负向用例证明不了门禁「严格」而非「恒失败」）。"""
        _baseline, legs, contact_geoms, params = _PROBE.load_declarations(str(BASELINE))
        self.assertEqual(sorted(legs), sorted(contact_geoms))
        self.assertIn("allocation", params)

    def test_missing_gait_legs_is_exit_code_2(self):
        path = self._write_baseline(lambda data: data["gait"].pop("legs"))
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.load_declarations(str(path))
        self.assertEqual(ctx.exception.code, 2)

    def test_missing_balance_section_is_exit_code_2(self):
        path = self._write_baseline(lambda data: data.pop("balance", None))
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.load_declarations(str(path))
        self.assertEqual(ctx.exception.code, 2)

    def test_missing_baseline_file_is_exit_code_3(self):
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.load_declarations(str(self.tmp / "nope.yaml"))
        self.assertEqual(ctx.exception.code, 3)

    def test_missing_height_target_is_exit_code_2_and_writes_nothing(self):
        if not MODEL.is_file():
            self.skipTest("场景构建产物不在本机（build/ 为 gitignore 证据区）")
        path = self._write_baseline(lambda data: data["gait"]["verification"].pop("height_target_m"))
        out = self.tmp / "should-not-exist.json"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            code = _PROBE.main(["--baseline", str(path), "--model", str(MODEL), "--json", str(out)])
        self.assertEqual(code, 2)
        self.assertNotIn("正例对照", buffer.getvalue())
        self.assertFalse(out.exists())

    def test_main_returns_usage_code_for_degenerate_grid_and_writes_nothing(self):
        out = self.tmp / "should-not-exist.json"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            code = _PROBE.main(["--baseline", str(BASELINE), "--half-m", "0.0", "--json", str(out)])
        self.assertEqual(code, 1)
        self.assertFalse(out.exists())

    def test_main_on_missing_model_is_exit_code_3(self):
        """引用完整性：模型不在场时显式失败，不得把 Δ=0 当成「一次空跑」。"""
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            code = _PROBE.main(["--baseline", str(BASELINE),
                                "--model", str(self.tmp / "no-model.xml")])
        self.assertEqual(code, 3)
        self.assertNotIn("正例对照", buffer.getvalue())


@unittest.skipUnless(_PROBE is not None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
@unittest.skipUnless(MODEL.is_file(), "场景构建产物不在本机（build/ 为 gitignore 证据区）")
class RuntimeInvariantTests(unittest.TestCase):
    """现场构建产物上的结构不变量（不断言实测数字）。"""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls._tmp.cleanup)
        out = Path(cls._tmp.name) / "probe.json"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            cls.code = _PROBE.main(["--baseline", str(BASELINE), "--model", str(MODEL),
                                   "--json", str(out)])
        cls.stdout = buffer.getvalue()
        cls.report = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else None

    def test_runs_to_completion_with_reference_control(self):
        self.assertEqual(self.code, 0)
        self.assertIsNotNone(self.report)
        self.assertTrue(self.report["simulation"])
        self.assertTrue(self.report["reference_ok"], "四腿正例对照不成立 ⇒ 实验组结论不得据此判读")
        self.assertEqual(self.report["probe"], "go2_com_shift_feasibility")

    def test_four_leg_reference_region_is_fully_feasible(self):
        """正例对照：四腿支撑集在整个网格内无截断（此条失败 ⇒ 先复核声明/探针，再读实验组）。"""
        region = self.report["reference_region"]
        self.assertEqual(region["feasible_count"], region["point_count"])
        self.assertEqual(region["allocator_error_count"], 0)
        self.assertIsNotNone(region["nearest_feasible"])

    def test_phase_invariants(self):
        phases = self.report["phases"]
        self.assertEqual(len(phases), len(self.report["foot_xy_m"]))
        weight = self.report["weight_n"]
        grid_points = self.report["grid"]["points_per_case"]
        for phase in phases:
            support = phase["support_legs"]
            self.assertNotIn(phase["excluded_leg"], support)
            self.assertEqual(len(support), len(phases) - 1)

            region = phase["region"]
            self.assertEqual(region["point_count"], grid_points)
            self.assertEqual(
                region["feasible_count"] + region["clamped_count"] + region["allocator_error_count"],
                region["point_count"])
            self.assertAlmostEqual(region["area_lower_bound_m2"],
                                   region["feasible_count"] * region["resolution_m"] ** 2, places=12)
            if region["feasible_count"] > 0:
                self.assertIsNotNone(region["nearest_feasible"])
            else:
                self.assertIsNone(region["nearest_feasible"])

            # 有截断 ⇔ 期望力旋量未精确兑现；无截断 ⇒ 残差只是数值噪声
            zero = phase["zero_offset"]
            self.assertGreaterEqual(zero["min_normal_over_mg"], 0.0)
            if zero["clamped_legs"]:
                self.assertGreater(zero["residual_force_n"], 0.0)
            else:
                self.assertLessEqual(zero["residual_force_n"], 1.0e-9 * weight)

            # 形心必须确实是三个支撑足 xy 的均值（几何量从报告自身可复核）
            feet = self.report["foot_xy_m"]
            reference_xy = self.report["reference_point_xy_m"]
            mean_x = sum(feet[code][0] for code in support) / len(support)
            mean_y = sum(feet[code][1] for code in support) / len(support)
            self.assertAlmostEqual(
                phase["support_centroid_distance_m"],
                math.hypot(phase["support_centroid_offset_m"][0],
                           phase["support_centroid_offset_m"][1]), places=12)
            self.assertEqual(phase["support_centroid"]["offset_m"],
                             phase["support_centroid_offset_m"])
            self.assertAlmostEqual(phase["support_centroid_offset_m"][0],
                                   mean_x - reference_xy[0], places=9)
            self.assertAlmostEqual(phase["support_centroid_offset_m"][1],
                                   mean_y - reference_xy[1], places=9)

            ray = phase["ray"]
            self.assertEqual(ray["samples"][0]["distance_m"], 0.0)
            self.assertEqual(len(ray["samples"]), 1 + int(round(
                self.report["ray"]["max_distance_m"] / ray["step_m"])))
            first_free = ray["first_clamp_free_distance_m"]
            if first_free is not None:
                self.assertTrue(any(abs(item["distance_m"] - first_free) < 1.0e-12
                                    for item in ray["samples"]))

    def test_intersection_is_consistent_with_pairwise_counts(self):
        intersection = self.report["constant_shift_intersection"]
        self.assertIsNotNone(intersection)
        for phase in self.report["phases"]:
            self.assertLessEqual(intersection["intersection_point_count"],
                                 phase["region"]["feasible_count"])
        for pair, count in intersection["pairwise_feasible_point_counts"].items():
            for code in pair.split("+"):
                phase = [item for item in self.report["phases"] if item["excluded_leg"] == code][0]
                self.assertLessEqual(count, phase["region"]["feasible_count"])
        if intersection["intersection_point_count"] == 0:
            self.assertTrue(any(count == 0
                                for count in intersection["pairwise_feasible_point_counts"].values()),
                            "四相交集为空 ⇒ 至少有一对相位的交集也必须为空（否则交集机器可疑）")


if __name__ == "__main__":
    unittest.main()
