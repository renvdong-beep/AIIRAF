"""契约测试：支撑集可行性只读探针（步骤 02b 决策包）。

覆盖两件事：
1. **fail-closed 形状**：缺 `gait.legs` / 缺 `balance` / 缺 `robot.profile` / Profile 缺
   `spec.model.trunk_body` / 基线文件不存在 ⇒ 各自的退出码，且**不得**用默认值兜底。
2. **正例对照**：四腿支撑集（现状的实测接触判定）必须「无截断 + 残差 ≈ 0」。没有这条对照，
   「三腿支撑集不可兑现」就分不清是机制结果还是探针恒失败。

判据只断**结构不变量**（不硬编码实测数字）：本探针的数值依赖机型声明与场景几何，
把它们写成快照会让声明一改就假失败。现场构建产物（`build/` 是证据区、已 gitignore）
不在本机时，运行类用例整体 `skip` 并注明理由，而不是伪造通过。
"""

import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "probe_go2_support_set_feasibility.py"
BASELINE = ROOT / "config/go2_loopback.yaml"
MODEL = ROOT / "build/scenes/handoff_lab/handoff_lab.xml"


def _load_probe():
    spec = importlib.util.spec_from_file_location("probe_support_set_feasibility", SCRIPT)
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


@unittest.skipIf(_PROBE is None, "探针模块不可导入（缺依赖）: %s" % _LOAD_ERROR)
class SupportSetFeasibilityProbeTests(unittest.TestCase):
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

    # ---------- fail-closed ----------

    def test_positive_control_real_baseline_loads(self):
        """正例对照：真基线必须解析成功（否则下面的负向用例不能证明门禁是「严格」而非「恒失败」）。"""
        _baseline, legs, contact_geoms, params = _PROBE.load_declarations(str(BASELINE))
        self.assertEqual(len(legs), 4)
        self.assertEqual(sorted(legs), sorted(contact_geoms))
        self.assertIn("allocation", params)
        self.assertGreaterEqual(int(params["allocation"]["min_stance_legs"]), 1)

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

    def test_missing_robot_profile_is_exit_code_2(self):
        path = self._write_baseline(lambda data: data.pop("robot", None))
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.resolve_trunk_body(loaded)
        self.assertEqual(ctx.exception.code, 2)

    def test_profile_without_trunk_body_is_exit_code_2(self):
        profile = self.tmp / "profile.yaml"
        profile.write_text(yaml.safe_dump({"spec": {"model": {}}}), encoding="utf-8")
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.resolve_trunk_body(self.base, str(profile))
        self.assertEqual(ctx.exception.code, 2)

    def test_profile_path_not_found_is_exit_code_3(self):
        with self.assertRaises(_PROBE.ProbeError) as ctx:
            _PROBE.resolve_trunk_body(self.base, str(self.tmp / "no-profile.yaml"))
        self.assertEqual(ctx.exception.code, 3)

    def test_main_returns_declaration_code_for_broken_baseline(self):
        """入口层把 ProbeError 映射成退出码（不是 traceback），且不写任何产物。"""
        path = self._write_baseline(lambda data: data["gait"].pop("legs"))
        out = self.tmp / "should-not-exist.json"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            code = _PROBE.main(["--baseline", str(path), "--json", str(out)])
        self.assertEqual(code, 2)
        self.assertFalse(out.exists())

    # ---------- 正例对照 + 结构不变量（依赖现场构建产物） ----------

    def test_four_leg_reference_is_realizable(self):
        self.assertTrue(MODEL.is_file(), "场景构建产物不在本机（build/ 为 gitignore 证据区）")
        out = self.tmp / "probe.json"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = _PROBE.main(["--baseline", str(BASELINE), "--model", str(MODEL),
                                "--json", str(out)])
        self.assertEqual(code, 0)
        self.assertTrue(out.is_file())
        report = json.loads(out.read_text(encoding="utf-8"))

        self.assertTrue(report["simulation"])
        self.assertTrue(report["reference_ok"], "四腿正例对照不成立 ⇒ 不得据此判读实验组")

        cases = {item["case_id"]: item for item in report["cases"]}
        reference = cases["reference_four_leg_measured_contact"]
        self.assertEqual(reference["role"], "positive_control")
        self.assertEqual(reference["clamped_legs"], [])
        self.assertEqual(reference["support_leg_count"], 4)
        weight = report["weight_n"]
        self.assertLessEqual(reference["residual_force_n"], 1.0e-9 * weight)
        self.assertLessEqual(reference["residual_torque_nm"], 1.0e-9 * weight)

        hypotheses = [item for item in report["cases"] if item["role"] == "hypothesis"]
        self.assertEqual(len(hypotheses), len(report["foot_xy_m"]))
        for item in hypotheses:
            self.assertEqual(item["support_leg_count"], len(hypotheses) - 1)
            self.assertNotIn(item["excluded_leg"], item["support_legs"])
            # 不变量：有截断 ⇒ 必然有残差；无截断 ⇒ 精确兑现（残差 = 数值噪声）
            if item["clamped_legs"]:
                self.assertGreater(item["residual_force_over_mg"], 0.0)
                self.assertFalse(item["realizable"])
            else:
                self.assertLessEqual(item["residual_force_n"], 1.0e-9 * weight)
                self.assertTrue(item["realizable"])
            self.assertGreaterEqual(item["min_normal_over_mg"], 0.0)

    def test_main_on_missing_model_is_exit_code_3(self):
        """引用完整性：模型不在场时显式失败，不得退化成「无接触点的一次空跑」。"""
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            code = _PROBE.main(["--baseline", str(BASELINE),
                                "--model", str(self.tmp / "no-model.xml")])
        self.assertNotEqual(code, 0)
        self.assertNotIn("正例对照", buffer.getvalue())


if __name__ == "__main__":
    unittest.main()
