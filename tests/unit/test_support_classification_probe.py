"""只读探针 `scripts/probe_go2_support_classification.py` 的契约测试（步骤 02b）。

覆盖（每条都成对：**正例对照 + 负向**，否则分不清「门禁严格」与「门禁恒失败」）：

1. **正例对照**：合成采样里「声明摆动相 ∧ 实测离地」⇒ 交叉制表必须报出 `declared_swing_free > 0`、
   `self_lock_fraction == 0.0`（证明这个口径**能**检测到真摆动，不是恒真门禁）；
2. 自锁形态：合成采样里「声明摆动相 ∧ 实测接触」⇒ `self_lock_fraction == 1.0`、四腿全无自由摆动采样；
3. 同源门禁：报告登记的声明 sha256 不符 ⇒ 退出码 4（不得据此判读）；
4. 引用完整性：缺 samples 文件 ⇒ 退出码 3；逐采样缺 `contact_n[腿]` ⇒ 退出码 3（**不带 traceback**）；
5. 相位分组解析回归：形如 trot 的「一组两条腿」必须解析成两条腿各自偏移（首跑的缺陷是只取 `legs[0]`）；
6. 权重推导的两条分支：`balance.enabled=false` 时**没有**「实测接触 → 权重」映射，不得照样推出一份表。
"""

import contextlib
import hashlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "src"))

from probe_go2_support_classification import (  # noqa: E402
    EXIT_INCONSISTENT,
    EXIT_OK,
    EXIT_REFERENCE,
    _derived_weight,
    main,
    phase_offsets_from_groups,
)
from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

CONFIG = REPO_ROOT / "config" / "go2_loopback.yaml"


def _load():
    declaration, _ = unitree_go2.load_declaration(CONFIG)
    profile = load_robot_profile(REPO_ROOT / declaration["robot"]["profile"])
    params = gait.load_gait_declaration(declaration, profile.joints)
    return declaration, params


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _phase_groups(params):
    """把解析后的腿按偏移分组（与生产侧写进报告的形状一致）。"""
    groups = {}
    for code, leg in params["legs"].items():
        groups.setdefault(float(leg["phase_offset"]), []).append(code)
    return [
        {"offset": offset, "legs": sorted(codes)} for offset, codes in sorted(groups.items())
    ]


def _samples(params, contact_fn, steps=100, dt=0.01):
    """构造合成采样：`contact_fn(elapsed, code) -> 接触力`（腿的相位用生产原语算）。

    `elapsed` **必须**与探针算的一致（`time_s − samples[0].time_s`）：duty=0.75、周期 0.8 s、
    采样 0.01 s 时每 0.2 s 就有一个采样点正好落在支撑/摆动边界上，两条边界的浮点归属会
    一边算支撑、一边算摆动（实测首跑 300 个摆动采样里 7 个被判成接触）。
    夹具按「与探针同一算式」定相位，**不是**放宽判据（阈值一字未改）。
    """
    onset = 5.0
    out = []
    for index in range(steps):
        time_s = onset + index * dt
        elapsed = time_s - onset  # 与探针口径逐位一致
        out.append(
            {
                "time_s": time_s,
                "base_height_m": float(params["verification"]["height_target_m"]),
                "contact_n": {
                    code: float(contact_fn(elapsed, code)) for code in params["legs"]
                },
            }
        )
    return out


class ProbeSupportClassificationTests(unittest.TestCase):
    def setUp(self):
        self.declaration, self.params = _load()
        self.threshold = float(self.params["verification"]["contact_force_threshold_n"])
        self.duty = float(self.params["duty_factor"])
        self.tmp = Path(tempfile.mkdtemp(prefix="probe-support-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def _write(self, samples, *, sha=None, period=None):
        report = {
            "simulation": True,
            "config": {
                "sha256": sha or _sha256(CONFIG),
                "gait": {
                    "kind": self.params["kind"],
                    "duty_factor": self.duty,
                    "ramp_s": float(self.params["ramp_s"]),
                    "frequency_hz": float(self.params["frequency_hz"]),
                    "phase_groups": _phase_groups(self.params),
                },
            },
        }
        samples_doc = {
            "simulation": True,
            "gait": {"period_s": period or float(self.params["period_s"])},
            "samples": samples,
        }
        (self.tmp / "report.json").write_text(
            json.dumps(report, ensure_ascii=False), encoding="utf-8"
        )
        (self.tmp / "samples.json").write_text(
            json.dumps(samples_doc, ensure_ascii=False), encoding="utf-8"
        )
        return self.tmp / "report.json"

    def _run(self, report, *, samples=None, out="probe.json"):
        argv = [
            "--root",
            str(REPO_ROOT),
            "--config",
            str(CONFIG),
            "--report",
            str(report),
            "--out",
            str(self.tmp / out),
        ]
        if samples is not None:
            argv += ["--samples", str(samples)]
        # 入口会把探测结果 JSON 打到 stdout：测试里必须吞掉，否则全量单测的证据文件被 JSON 噪声淹没。
        with contextlib.redirect_stdout(io.StringIO()):
            return main(argv)

    def _is_declared_swing(self, elapsed, code):
        return not gait.is_stance(self.params, gait.leg_phase(self.params, code, elapsed))

    # ---- 1. 正例对照：真摆动必须被检测到 ---------------------------------------------
    def test_free_swing_is_detected(self):
        ramp = float(self.params["ramp_s"])

        def contact(elapsed, code):
            if elapsed < ramp:
                return self.threshold * 10.0
            return 0.0 if self._is_declared_swing(elapsed, code) else self.threshold * 20.0

        report = self._write(_samples(self.params, contact, steps=400))
        self.assertEqual(self._run(report), EXIT_OK)
        result = json.loads((self.tmp / "probe.json").read_text(encoding="utf-8"))
        self.assertEqual(result["self_lock"]["self_lock_fraction"], 0.0)
        self.assertEqual(result["self_lock"]["legs_with_no_free_swing_sample"], [])
        for code, item in result["per_leg"].items():
            self.assertGreater(item["declared_swing_free_samples"], 0, msg=code)
            self.assertEqual(item["declared_swing_contact_samples"], 0, msg=code)
            self.assertGreater(item["declared_swing_samples"], 0, msg=code)

    # ---- 2. 自锁形态：声明摆动相 ∧ 实测接触 ------------------------------------------
    def test_self_locking_shape_is_reported(self):
        def contact(elapsed, code):
            return self.threshold * 20.0

        report = self._write(_samples(self.params, contact, steps=400))
        self.assertEqual(self._run(report), EXIT_OK)
        result = json.loads((self.tmp / "probe.json").read_text(encoding="utf-8"))
        self.assertEqual(result["self_lock"]["self_lock_fraction"], 1.0)
        self.assertEqual(
            sorted(result["self_lock"]["legs_with_no_free_swing_sample"]),
            sorted(self.params["legs"]),
        )
        self.assertEqual(result["measured_stance_legs_histogram"], {"4": 300})

    # ---- 3. 同源门禁 ----------------------------------------------------------------
    def test_foreign_declaration_sha_is_refused(self):
        def contact(elapsed, code):
            return self.threshold * 20.0

        report = self._write(_samples(self.params, contact, steps=100), sha="0" * 64)
        self.assertEqual(self._run(report), EXIT_INCONSISTENT)

    def test_foreign_period_is_refused(self):
        def contact(elapsed, code):
            return self.threshold * 20.0

        report = self._write(_samples(self.params, contact, steps=100), period=0.5)
        self.assertEqual(self._run(report), EXIT_INCONSISTENT)

    # ---- 4. 引用完整性 / 结构校验 ----------------------------------------------------
    def test_missing_samples_file_is_reference_error(self):
        def contact(elapsed, code):
            return self.threshold * 20.0

        report = self._write(_samples(self.params, contact, steps=100))
        (self.tmp / "samples.json").unlink()
        self.assertEqual(self._run(report), EXIT_REFERENCE)

    def test_sample_without_contact_is_reference_error_without_traceback(self):
        def contact(elapsed, code):
            return self.threshold * 20.0

        samples = _samples(self.params, contact, steps=100)
        del samples[7]["contact_n"]["FL"]
        report = self._write(samples)
        self.assertEqual(self._run(report), EXIT_REFERENCE)

    # ---- 5. 相位分组解析回归（首跑缺陷：只取 legs[0]） --------------------------------
    def test_phase_offsets_expand_every_leg_in_group(self):
        trot_shaped = [
            {"offset": 0.0, "legs": ["FL", "RR"]},
            {"offset": 0.5, "legs": ["FR", "RL"]},
        ]
        self.assertEqual(
            phase_offsets_from_groups(trot_shaped),
            {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5},
        )
        # 负向对照：只取 legs[0] 的旧写法在**同一输入**上必然丢掉两条腿（回归断言）。
        legacy = {item["legs"][0]: item["offset"] for item in trot_shaped}
        self.assertEqual(sorted(legacy), ["FL", "FR"])
        self.assertNotEqual(set(legacy), set(phase_offsets_from_groups(trot_shaped)))

    def test_single_leg_groups_still_parse(self):
        self.assertEqual(
            phase_offsets_from_groups([{"offset": 0.25, "legs": ["FR"]}]),
            {"FR": 0.25},
        )

    # ---- 6. 权重推导的两条分支 ------------------------------------------------------
    def test_weight_mapping_absent_when_balance_disabled(self):
        entry = {"declared_swing_contact_samples": 10, "declared_swing_free_samples": 5}
        disabled = _derived_weight(entry, 15, 0.0, 1.0, False)
        self.assertFalse(disabled["balance_enabled"])
        self.assertIn("无「实测接触 → 权重」映射", disabled["derivation"])
        self.assertNotIn("stance_weight_position", disabled)
        enabled = _derived_weight(entry, 15, 0.0, 1.0, True)
        self.assertTrue(enabled["balance_enabled"])
        self.assertEqual(enabled["declared_swing_contact_samples_weighted_stance"], 10)
        self.assertEqual(enabled["declared_swing_free_samples_weighted_swing"], 5)


if __name__ == "__main__":
    unittest.main()
