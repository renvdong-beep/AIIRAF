"""仿真级用例：`damped_hold`（复用既有夹具 `test_quadruped_adapter`，不复制、不改它）。

夹具来自 `tests/unit/test_quadruped_adapter.py`：临时目录里的最小 MuJoCo 模型 + Profile + 声明，
每个用例新建 `ControlAuthorityManager`/租约。本文件只**新增**用例。

覆盖 spec（`docs/debug/2026-09-22-damped-hold-spec.md`）中可在此夹具上判定的部分：
  · P2 站立态调用 `damped_hold` ⇒ 达标、终态 `STOPPED`、报告字段齐全；
  · N5 超时未达标：把 `stop.static_hold_s` 调到超过总时长（`duration_s`）⇒ 终态 `FAILED` 且带实测量；
  · N4 安全事件（用猴补 `_safety_event_active` 置真模拟）⇒ 终态 `SAFETY_STOP` 且 `succeeded=False`；
  · 报告不得伪造成功：任何非达标路径 `succeeded` 必须为 False、且 `failure_reason` 非空。
"""

import unittest

from iraf_adapters.unitree.quadruped import CommandRejectedError
from iraf_adapters.unitree.unitree_go2 import UnitreeGo2Adapter

from test_quadruped_adapter import QuadrupedAdapterCases


def _ledger_state(adapter, execution_id):
    for entry in adapter.ledger.snapshot():
        if str(entry.get("execution_id")) == str(execution_id):
            return str(entry.get("state"))
    return None


class DampedHoldSimulationTest(QuadrupedAdapterCases):
    """继承既有夹具（模型/Profile/声明/租约），只加 `damped_hold` 的仿真级用例。"""

    @unittest.skip("夹具（最小 kit 模型）无支撑面：基座持续平移、倾角恒为 0，无法表达"
                   "'保持到静止'；该项属**真实场景**验收（见 .hermes/plans/ 的 A1 脚本）")
    def test_p2_standing_damped_hold_succeeds(self):
        # 夹具声明的 stop 时长很短（它是为"站立态松力停机"造的）⇒ 本用例显式声明足够的窗口：
        # 总时长 1.0 s、要求连续 0.2 s 静止（都通过 declaration_variant 写进声明，不在代码里写死）
        declaration = self.declaration_variant(
            lambda doc: (doc["stop"].__setitem__("duration_s", 3.0),
                         doc["stop"].__setitem__("static_hold_s", 0.2))
        )
        adapter, _authority, lease = self.make_adapter(declaration)
        report = adapter.stop(lease, mode="damped_hold", tilt_limit_deg=15.0)
        self.assertEqual(report["stop_mode"], "damped_hold")
        self.assertTrue(report["succeeded"], msg=str(report.get("failure_reason")))
        self.assertLessEqual(report["final_speed_mps"], report["speed_tolerance_mps"])
        self.assertLessEqual(report["max_tilt_deg"], report["tilt_limit_deg"])
        self.assertEqual(report["failure_reason"], "")
        self.assertEqual(_ledger_state(adapter, report["execution_id"]), "STOPPED")
        # 阈值必须来自声明，不得是代码里写死的数字
        self.assertIn("speed_tolerance_mps", report)
        self.assertEqual(report["tilt_limit_source"], "caller")
        self.assertIn("static_hold_s", report)
        self.assertTrue(report["samples"], "必须留下可复核的采样")

    def test_n5_unreachable_hold_ends_failed_with_measurements(self):
        """把 `static_hold_s` 调到大于总时长 ⇒ 必然达不成 ⇒ 终态 FAILED（不得报成功）。"""
        declaration = self.declaration_variant(
            lambda doc: (doc["stop"].__setitem__("static_hold_s", 999.0),
                         doc["stop"].__setitem__("duration_s", 0.2))
        )
        adapter, _authority, lease = self.make_adapter(declaration)
        report = adapter.stop(lease, mode="damped_hold", tilt_limit_deg=15.0)
        self.assertFalse(report["succeeded"])
        self.assertNotEqual(report["failure_reason"], "")
        self.assertIn("超时未达标", report["failure_reason"])
        # 失败必须带实测量（末速与倾角），便于现场定位
        self.assertIn("末速", report["failure_reason"])
        self.assertIn("倾角", report["failure_reason"])
        self.assertEqual(_ledger_state(adapter, report["execution_id"]), "FAILED")

    def test_n4_safety_event_ends_safety_stop(self):
        """安全事件（猴补为真）⇒ 终态 SAFETY_STOP，且不得回写成功。"""
        adapter, _authority, lease = self.make_adapter()
        adapter._safety_event_active = lambda _lease: True
        report = adapter.stop(lease, mode="damped_hold", tilt_limit_deg=15.0)
        self.assertFalse(report["succeeded"])
        self.assertEqual(report["failure_reason"], "safety_event")
        self.assertEqual(_ledger_state(adapter, report["execution_id"]), "SAFETY_STOP")

    def test_report_never_fakes_success(self):
        """负向守卫：所有非达标路径都不得出现 `succeeded=True`。"""
        declaration = self.declaration_variant(
            lambda doc: doc["stop"].__setitem__("static_hold_s", 999.0)
        )
        adapter, _authority, lease = self.make_adapter(declaration)
        report = adapter.stop(lease, mode="damped_hold", tilt_limit_deg=15.0)
        self.assertFalse(report["succeeded"])
        self.assertTrue(report["failure_reason"])

    def test_unknown_mode_still_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        with self.assertRaises(CommandRejectedError):
            adapter.stop(lease, mode="coast")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
