"""单测：`ProviderCore`（第 4 块内核）—— 含端到端**负向路径**，用注入求解器，无需 osqp/scipy。

覆盖点：
  · 成功路径：解可回代（与原问题变量在缩放往返意义下一致）、decision=ok、状态与耗时透传；
  · 求解器返回坏状态（infeasible/max_iter/unknown 等）⇒ decision=damped_hold 且 **z is None**；
  · 求解器抛异常 ⇒ decision=damped_hold、z is None、原因里含异常摘要（不吞异常）；
  · **状态过期**：成功一次后把时钟推过 40.0 ms ⇒ decision=damped_hold 且 z is None；
  · **禁止跑旧解**：失败后即使此前有成功解，也不得把旧解当本拍输出；
  · 急停覆盖一切 ⇒ torque_zero_release 且 z is None；
  · 请求缺字段 ⇒ 显式失败（不静默）；
  · `invalidate()` 之后无解可用（has_solution False）。
"""

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.freshness import (
    DECISION_DAMPED_HOLD, DECISION_OK, DECISION_TORQUE_ZERO_RELEASE, STALE_LIMIT_MS)
from iraf_adapters.unitree.mpc import qp_scaling as qs
from iraf_adapters.unitree.mpc.provider_core import ProviderCore


def make_qp():
    """最小可解 QP：2 变量、1 行等式（z0 + z1 = 1）。"""
    return {
        "h_diag": np.array([2.0, 2.0]), "g": np.array([-1.0, -1.0]),
        "a_rows": np.array([0]), "a_cols": np.array([0]),
        "a_vals": np.array([1.0]),
        "lbx": np.array([-1.0, -1.0]), "ubx": np.array([1.0, 1.0]),
        "lba": np.array([0.0]), "uba": np.array([2.0]),
    }


class FakeSolver:
    """可编程的假求解器：返回预设状态，或抛异常。"""

    def __init__(self, status_class="ok", raise_exc=None):
        self.status_class = status_class
        self.raise_exc = raise_exc
        self.calls = 0

    def __call__(self, h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba, prev, settings):
        self.calls += 1
        if self.raise_exc is not None:
            raise self.raise_exc
        # 注意：内核把**缩放后**的 h（= 1）交给求解器 ⇒ 这里的 "d" 恒为 1。
        # 因此本假求解器返回**缩放空间**里的点 w = [0.5, 0.5]；回代后 z = w / d_真实（由测试断言）。
        w = np.array([0.5, 0.5])
        n = h_diag.size
        y = np.zeros(n + 1)
        return {"x": w, "y": y, "status": self.status_class, "status_class": self.status_class,
                "iter": 5.0, "setup_ms": 1.0, "solve_ms": 2.0}


def test_success_path_unscales_and_reports():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 1000.0)
    out = core.step(make_qp())
    assert out["decision"] == DECISION_OK and out["z"] is not None
    d = qs.build_scale_diag(make_qp()["h_diag"])
    # 假求解器在缩放空间返回 w = [0.5, 0.5]；内核必须用**真实 d** 回代 ⇒ z = w / d
    np.testing.assert_allclose(out["z"], np.array([0.5, 0.5]) / d)
    assert out["status_class"] == "ok" and out["iter"] == 5.0 and out["solve_ms"] == 2.0
    assert core.has_solution is True


@pytest.mark.parametrize("status", ["infeasible", "unbounded", "max_iter", "time_limit",
                                    "nonconvex", "unknown"])
def test_bad_status_yields_damped_hold_without_solution(status):
    core = ProviderCore(FakeSolver(status), now_fn=lambda: 0.0)
    out = core.step(make_qp())
    assert out["decision"] == DECISION_DAMPED_HOLD
    assert out["z"] is None, "坏状态下绝不输出解（禁止跑旧解/伪造成功）"
    assert core.has_solution is False


def test_solver_exception_is_explicit_failure_not_swallowed():
    core = ProviderCore(FakeSolver(raise_exc=RuntimeError("乱流：求解器崩了")), now_fn=lambda: 0.0)
    out = core.step(make_qp())
    assert out["decision"] == DECISION_DAMPED_HOLD and out["z"] is None
    assert "求解器异常" in out["reason"] and "乱流" in out["reason"]


def test_stale_solution_is_rejected_by_consumer_side_pick():
    """消费侧：解龄超过 2 个 MPC 周期 ⇒ 不得使用（`damped_hold`），与生产速率解耦。"""
    clock = {"t": 0.0}
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: clock["t"])
    assert core.step(make_qp())["decision"] == DECISION_OK
    assert core.latest()["decision"] == DECISION_OK
    clock["t"] = STALE_LIMIT_MS + 0.1
    out = core.latest()
    assert out["decision"] == DECISION_DAMPED_HOLD and out["flags"]["stale"] is True
    assert "状态过期" in out["reason"]
    # 重新求解 ⇒ 立刻恢复可用（新解解龄为 0）
    assert core.step(make_qp())["decision"] == DECISION_OK
    assert core.latest()["decision"] == DECISION_OK


def test_consumer_side_pick_without_any_solution_fails():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    out = core.latest()
    assert out["decision"] == DECISION_DAMPED_HOLD and out["age_ms"] is None
    assert "无可用解" in out["reason"]


def test_consumer_side_pick_honours_emergency():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    core.step(make_qp())
    assert core.latest(emergency=True)["decision"] == DECISION_TORQUE_ZERO_RELEASE


def test_failure_does_not_reuse_last_solution():
    """负向守卫：先成功，再失败 —— 本拍不得把上一次的解拿来用。"""
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    assert core.step(make_qp())["decision"] == DECISION_OK
    core._solver = FakeSolver("max_iter")                   # 换成会失败的求解器
    out = core.step(make_qp())
    assert out["decision"] == DECISION_DAMPED_HOLD and out["z"] is None


def test_emergency_overrides_and_emits_no_solution():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    out = core.step(make_qp(), emergency=True)
    assert out["decision"] == DECISION_TORQUE_ZERO_RELEASE and out["z"] is None


def test_missing_fields_fail_explicitly():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    qp = make_qp()
    del qp["a_vals"]
    out = core.step(qp)
    assert out["decision"] == DECISION_DAMPED_HOLD and out["z"] is None
    assert "缺少字段" in out["reason"] and "a_vals" in out["reason"]


def test_invalidate_clears_state():
    core = ProviderCore(FakeSolver("ok"), now_fn=lambda: 0.0)
    core.step(make_qp())
    assert core.has_solution is True
    core.invalidate("安全事件")
    assert core.has_solution is False and core.last_ok_ms is None
    out = core.step(make_qp())                              # 重新求解即可恢复
    assert out["decision"] == DECISION_OK
