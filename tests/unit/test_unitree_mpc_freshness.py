"""单测：`freshness.decide` —— 解新鲜度门禁与显式失败路径（第 4 块策略核心）。

覆盖点（每条都对着一份契约/铁律，不是"测覆盖率"）：
  · 正常：解龄在限内且 QP 状态可用 ⇒ `ok`；
  · 边界：39.9 / 40.0 / 40.1 ms（阈值 = 2 个 MPC 周期，来自 ADR-0009 §6.5）；
  · 状态过期、QP 各类失败状态、未知状态 ⇒ `damped_hold`（**不得继续跑旧解**）；
  · 解龄 `None` / `NaN` / `Inf` / 负值 ⇒ `damped_hold`（不可信即失败）；
  · 急停覆盖一切 ⇒ `torque_zero_release`；
  · **负向守卫**：任何非可用 QP 状态都不可能返回 `ok`（防"伪造成功"，铁律 6.6）；
  · 超预算只记 `overran` 标志，不改变决定（不在门禁里偷偷放宽验收判据）。
"""

import math

import pytest

from iraf_adapters.unitree.mpc.freshness import (
    DECISION_DAMPED_HOLD,
    DECISION_OK,
    DECISION_TORQUE_ZERO_RELEASE,
    MPC_PERIOD_MS,
    STALE_LIMIT_MS,
    decide,
)


def test_constants_are_single_source_of_truth():
    assert MPC_PERIOD_MS == 20.0            # 50 Hz 解耦口径
    assert STALE_LIMIT_MS == 40.0           # 2 个周期


def test_fresh_and_usable_is_ok():
    out = decide(age_ms=5.0, qp_status_class="ok", solve_ms=1.9972)
    assert out["decision"] == DECISION_OK
    assert out["flags"]["stale"] is False and out["flags"]["overran"] is False
    assert out["reason"]                              # 必须有可读原因


@pytest.mark.parametrize("age", [0.0, 19.999, 39.9, 40.0])
def test_boundary_within_limit_is_ok(age):
    assert decide(age_ms=age, qp_status_class="ok")["decision"] == DECISION_OK


def test_boundary_just_over_limit_is_damped_hold():
    out = decide(age_ms=40.1, qp_status_class="ok")
    assert out["decision"] == DECISION_DAMPED_HOLD and out["flags"]["stale"] is True
    assert "状态过期" in out["reason"]


@pytest.mark.parametrize("status", ["infeasible", "unbounded", "max_iter", "time_limit",
                                    "nonconvex", "unknown", "exception"])
def test_bad_qp_status_never_ok(status):
    out = decide(age_ms=1.0, qp_status_class=status)
    assert out["decision"] == DECISION_DAMPED_HOLD
    assert "显式失败" in out["reason"]


@pytest.mark.parametrize("age", [None, float("nan"), float("inf"), -1.0, "abc"])
def test_untrustworthy_age_is_damped_hold(age):
    out = decide(age_ms=age, qp_status_class="ok")
    assert out["decision"] == DECISION_DAMPED_HOLD
    assert out["flags"]["invalid_age"] is True


def test_emergency_overrides_everything():
    out = decide(age_ms=999.0, qp_status_class="infeasible", emergency=True)
    assert out["decision"] == DECISION_TORQUE_ZERO_RELEASE
    assert "急停" in out["reason"]


def test_inaccurate_is_usable_but_flagged():
    out = decide(age_ms=1.0, qp_status_class="ok_inaccurate")
    assert out["decision"] == DECISION_OK and out["flags"]["inaccurate"] is True
    assert "solved inaccurate" in out["reason"]


def test_overrun_is_only_a_flag():
    out = decide(age_ms=1.0, qp_status_class="ok", solve_ms=MPC_PERIOD_MS + 0.001)
    assert out["decision"] == DECISION_OK and out["flags"]["overran"] is True
    assert "超过预算" in out["reason"]


def test_negative_guard_no_bad_status_can_return_ok():
    """负向守卫：遍历所有非可用状态分类，断言绝不可能得到 `ok`（防伪造成功）。"""
    bad = ["infeasible", "unbounded", "max_iter", "time_limit", "nonconvex", "unknown",
           "exception", "", "SOLVED?", None]
    for s in bad:
        out = decide(age_ms=0.5, qp_status_class=s)
        assert out["decision"] != DECISION_OK, "状态 %r 竟被判为可用" % (s,)
        assert out["decision"] == DECISION_DAMPED_HOLD
