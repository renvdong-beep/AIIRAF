"""`stop`(damped_hold) 报告的判读口径：只按**实际存在**的键判定，缺键即显式失败。

要证明的三件事（对应两次实测踩点）：
1. `failure_reason` 空 ⇒ 保持成立（`damped_hold` 真正停稳时不得被判成失败）；
2. `failure_reason` 非空 ⇒ 原样返回该原因（不吞掉、不改写）；
3. 缺判定键 ⇒ `StopReportError`（**不假定成功**）—— 这正是不存在键 `static_entered`
   造成的假失败类的根因（dock 的保持段与 locomote 的失败路径各踩一次）。
"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.unitree.stop_verdict import (  # noqa: E402
    StopReportError,
    static_hold_failure,
)

#: 实测摘录（build/iraf-a6a13/dock-measure.json 的 settle.report 精简）
DAMPED_HOLD_OK = {
    "capability": "stop",
    "stop_mode": "damped_hold",
    "succeeded": True,
    "final_speed_mps": 0.00024619728899357373,
    "failure_reason": "",
}


class StaticHoldVerdictTest(unittest.TestCase):
    def test_empty_reason_means_hold_ok(self):
        self.assertIsNone(static_hold_failure(DAMPED_HOLD_OK))

    def test_none_reason_means_hold_ok(self):
        report = dict(DAMPED_HOLD_OK, failure_reason=None)
        self.assertIsNone(static_hold_failure(report))

    def test_non_empty_reason_is_returned_verbatim(self):
        report = dict(DAMPED_HOLD_OK, failure_reason="末速 0.31 m/s 超过判据 0.05 m/s")
        self.assertEqual(static_hold_failure(report), "末速 0.31 m/s 超过判据 0.05 m/s")

    def test_missing_failure_reason_fails_explicitly(self):
        report = {key: value for key, value in DAMPED_HOLD_OK.items() if key != "failure_reason"}
        with self.assertRaises(StopReportError) as ctx:
            static_hold_failure(report)
        self.assertIn("failure_reason", str(ctx.exception))

    def test_missing_final_speed_fails_explicitly(self):
        # 只看 failure_reason 会漏掉"报告形状变了"这一类；末速是判据量，必须同时存在
        report = {key: value for key, value in DAMPED_HOLD_OK.items() if key != "final_speed_mps"}
        with self.assertRaises(StopReportError) as ctx:
            static_hold_failure(report)
        self.assertIn("final_speed_mps", str(ctx.exception))

    def test_phantom_static_entered_is_not_used(self):
        # 实测踩点：`static_entered` 是 loopback **验收脚本**自己按样本算的，不在报告里。
        # 本口径不得依赖它 —— 缺它也必须照常判定。
        report = dict(DAMPED_HOLD_OK, static_entered=False)
        self.assertIsNone(static_hold_failure(report))

    def test_non_dict_report_fails_explicitly(self):
        for bad in (None, [], "ok"):
            with self.assertRaises(StopReportError):
                static_hold_failure(bad)


if __name__ == "__main__":
    unittest.main()
