"""单测：`mpc.torque_hook`（A6a-④ 第二块）—— MPC 解 → B1 执行器载荷 + 契约 §4 负向路径。

用**假客户端**驱动**真** `ProviderRuntime`（不启子进程），覆盖：
  · 正常拍：解的第一拍足端力 → `Jᵀ·f`（手算对照）→ B1 载荷；权重按本拍接触表逐关节；
  · 更新率：`plan_fn`（384 变量 QP 的构造）只在更新拍被调用；消费拍复用同一个解、不再构造；
  · 契约 §4 四类负向（QP `max_iter` / 子进程超时 / 解龄过期 / 急停）各自的 decision，
    且**绝不复用旧解**（失败当拍 `has_solution is False`）；
  · 响应非法（解长度/数值）与本拍计划不可用 ⇒ 显式失败，原始原因保留；
  · 逐关节权重与腿→关节映射的门禁。
"""

from __future__ import annotations

import numpy as np
import pytest

from iraf_adapters.unitree.mpc import protocol as pr
from iraf_adapters.unitree.mpc.contact import LEG_ORDER
from iraf_adapters.unitree.mpc.freshness import STALE_LIMIT_MS
from iraf_adapters.unitree.mpc.provider_runtime import ProviderRuntime
from iraf_adapters.unitree.mpc.torque_hook import (MpcTorqueHook, MpcUnavailableError,
                                                   position_weight_vector, solution_foot_forces)

HORIZON = 16
NVARS = HORIZON * (12 + 12)          # 384（与 qp_builder 的列序同一事实）
FORCES0 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
FORCES1 = [100.0 + i for i in range(12)]
MASK_STANCE_FRONT = {"FL": 1, "FR": 1, "RL": 0, "RR": 0}
JT_IDENT = {code: np.eye(3) for code in LEG_ORDER}
INDEX_MAP = {"FL": (0, 1, 2), "FR": (3, 4, 5), "RL": (6, 7, 8), "RR": (9, 10, 11)}


def _req():
    n = NVARS
    return pr.build_request(h_diag=[1.0] * n, g=[0.0] * n, a_rows=[0], a_cols=[0], a_vals=[1.0],
                            lbx=[-1.0] * n, ubx=[1.0] * n, lba=[0.0], uba=[0.0])


def _solution(step0=FORCES0, step1=None):
    z = [0.0] * NVARS
    base = HORIZON * 12
    z[base:base + 12] = list(step0)
    if step1 is not None:
        z[base + 12:base + 24] = list(step1)
    return z


class FakeClient:
    """按脚本返回响应（不启子进程）。队列空 ⇒ 返回 ok + `_solution()`。"""

    def __init__(self, z=None):
        self.calls = 0
        self.queue = []
        self._z = _solution() if z is None else z

    def push(self, decision="ok", err=None, status="ok", solve_ms=2.0, z=None):
        self.queue.append({"decision": decision, "err": err, "status": status,
                           "solve_ms": solve_ms, "z": z})

    def call(self, request, timeout_ms=None):
        self.calls += 1
        spec = self.queue.pop(0) if self.queue else {"decision": "ok", "err": None,
                                                     "status": "ok", "solve_ms": 2.0, "z": None}
        z = spec["z"] if spec["z"] is not None else (self._z if spec["decision"] == "ok" else None)
        resp = pr.build_response(spec["decision"], "ok" if spec["decision"] == "ok" else "显式失败",
                                 z=z, status_class=spec["status"], iter_=60.0,
                                 solve_ms=spec["solve_ms"], age_ms=0.0)
        return resp, spec["err"]


def _hook(client, ticks_per_update=1, now_fn=lambda: 0.0, mask=MASK_STANCE_FRONT,
          stance_weight=0.0, swing_weight=1.0, horizon=HORIZON, emergency_fn=None,
          jt=JT_IDENT, plan_factory=None, counter=None):
    runtime = ProviderRuntime(client, ticks_per_update=ticks_per_update, now_fn=now_fn)
    counter = {} if counter is None else counter
    counter.setdefault("count", 0)

    def plan_fn():
        counter["count"] += 1
        if plan_factory is not None:
            return plan_factory()
        return {"request": _req(), "current_mask": mask}

    hook = MpcTorqueHook(runtime, plan_fn, INDEX_MAP, stance_weight, swing_weight,
                         jt if callable(jt) else (lambda: jt), horizon,
                         emergency_fn=emergency_fn)
    return hook


# ---------- 正路径 ----------

def test_ok_payload_uses_first_step_forces_and_declared_weights():
    hook = _hook(FakeClient())
    payload = hook(0)
    assert set(payload) == {"balance_torque_nm", "position_weight"}
    # 单位 Jᵀ ⇒ τ = f（逐腿 3 分量按 LEG_ORDER 落位）
    assert np.allclose(payload["balance_torque_nm"], FORCES0)
    # 支撑腿（FL/FR）取声明 stance_weight=0.0、摆动腿（RL/RR）取 swing_weight=1.0
    assert np.allclose(payload["position_weight"],
                       [0.0] * 6 + [1.0] * 6)
    assert hook.stats["payloads"] == 1 and hook.stats["plans"] == 1


def test_payload_matches_hand_computed_jacobian_product():
    jt = {code: np.eye(3) for code in LEG_ORDER}
    jt["FL"] = np.array([[2.0, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, -1.0]])
    hook = _hook(FakeClient(), jt=jt)
    tau = hook(0)["balance_torque_nm"]
    assert np.allclose(tau[0:3], [2.0, 1.0, -3.0])       # diag(2, 0.5, -1) · [1,2,3]
    assert np.allclose(tau[3:], FORCES0[3:])             # 其余腿单位阵 ⇒ 原样


def test_plan_fn_only_called_on_update_ticks():
    client = FakeClient()
    counter = {}
    hook = _hook(client, ticks_per_update=2, counter=counter)
    for cycle in range(4):
        hook(cycle)
    assert counter["count"] == 2             # 100 Hz 控制、50 Hz 更新
    assert client.calls == 2
    assert hook.stats["plans"] == 2 and hook.stats["payloads"] == 4


def test_consumer_tick_reuses_same_solution_not_a_new_solve():
    client = FakeClient(z=_solution(FORCES0, FORCES1))
    counter = {}
    hook = _hook(client, ticks_per_update=2, counter=counter)
    first = hook(0)["balance_torque_nm"]
    second = hook(1)["balance_torque_nm"]
    assert np.allclose(first, FORCES0) and np.allclose(second, FORCES0)   # 同一个解的第 0 拍
    assert client.calls == 1 and counter["count"] == 1


def test_horizon_step_selects_later_horizon_forces():
    z = _solution(FORCES0, FORCES1)
    assert np.allclose(solution_foot_forces(z, HORIZON, step=0).reshape(-1), FORCES0)
    assert np.allclose(solution_foot_forces(z, HORIZON, step=1).reshape(-1), FORCES1)


@pytest.mark.parametrize("bad_weight", [None, np.zeros(11), np.zeros(13), np.full(12, 1.5),
                                        np.full(12, -0.1), np.array([np.nan] + [0.0] * 11)])
def test_payload_rejects_bad_position_weight(bad_weight):
    from iraf_adapters.unitree.mpc.torque_provider import torque_provider_payload
    with pytest.raises((ValueError, TypeError)):
        torque_provider_payload(np.zeros((4, 3)), JT_IDENT, bad_weight)


# ---------- 契约 §4 负向（四条） ----------

def test_qp_max_iter_raises_damped_hold_and_clears_solution():
    client = FakeClient()
    hook = _hook(client)
    assert hook(0)["balance_torque_nm"] is not None
    client.push(decision="damped_hold", status="max_iter")
    with pytest.raises(MpcUnavailableError) as exc:
        hook(1)
    assert exc.value.decision == "damped_hold"
    assert hook._runtime.has_solution is False and hook._runtime.solution is None
    assert "max_iter" in exc.value.diagnostics["status_class"]


def test_subprocess_timeout_raises_damped_hold():
    client = FakeClient()
    hook = _hook(client)
    hook(0)
    client.push(decision="damped_hold", err="调用超时", status="timeout")
    with pytest.raises(MpcUnavailableError) as exc:
        hook(1)
    assert exc.value.decision == "damped_hold" and "超时" in exc.value.reason
    assert hook.stats["unavailable"] == 1


def test_stale_solution_raises_damped_hold_with_stale_flag():
    clock = {"t": 0.0}
    client = FakeClient()
    hook = _hook(client, ticks_per_update=5, now_fn=lambda: clock["t"])
    hook(0)
    clock["t"] = STALE_LIMIT_MS + 0.1
    with pytest.raises(MpcUnavailableError) as exc:
        hook(1)                       # 非更新拍 ⇒ 消费侧门禁判过期
    assert exc.value.decision == "damped_hold"
    assert "过期" in exc.value.reason
    assert client.calls == 1          # 过期拍不得再去调子进程


def test_emergency_raises_release_without_calling_subprocess():
    client = FakeClient()
    state = {"estop": False}
    hook = _hook(client, emergency_fn=lambda: state["estop"])
    hook(0)
    state["estop"] = True
    with pytest.raises(MpcUnavailableError) as exc:
        hook(1)
    assert exc.value.decision == "torque_zero_release"
    assert client.calls == 1          # 急停拍不再花求解预算
    assert hook._runtime.has_solution is False
    assert hook.stats["payloads"] == 1


# ---------- 其它显式失败 ----------

def test_invalid_solution_shape_maps_to_damped_hold_preserving_reason():
    client = FakeClient()
    hook = _hook(client)
    client.push(decision="ok", status="ok", z=[1.0, 2.0, 3.0])       # 长度 3 ⇒ 非法
    with pytest.raises(MpcUnavailableError) as exc:
        hook(0)
    assert exc.value.decision == "damped_hold"
    assert "解向量长度" in exc.value.reason


def test_non_finite_solution_maps_to_damped_hold():
    client = FakeClient()
    hook = _hook(client)
    z = _solution()
    z[HORIZON * 12] = np.inf
    client.push(decision="ok", status="ok", z=z)
    with pytest.raises(MpcUnavailableError) as exc:
        hook(0)
    assert exc.value.decision == "damped_hold" and "非有限值" in exc.value.reason


def test_plan_failure_is_explicit_and_keeps_cause():
    def boom():
        raise ValueError("状态含非有限值")
    hook = _hook(FakeClient(), plan_factory=boom)
    with pytest.raises(MpcUnavailableError) as exc:
        hook(0)
    assert exc.value.decision == "damped_hold"
    assert "ValueError" in exc.value.reason and "非有限值" in exc.value.reason


def test_plan_missing_key_is_explicit():
    hook = _hook(FakeClient(), plan_factory=lambda: {"request": _req()})
    with pytest.raises(MpcUnavailableError) as exc:
        hook(0)
    assert "current_mask" in exc.value.reason


def test_unavailable_decision_must_be_known():
    with pytest.raises(ValueError):
        MpcUnavailableError("solved", "旧词表不得使用")


def test_summary_is_traceable():
    hook = _hook(FakeClient())
    hook(0)
    s = hook.summary()
    assert s["horizon"] == HORIZON
    assert s["stats"]["payloads"] == 1 and s["stats"]["last_decision"] == "ok"
    assert s["mask"] == [1, 1, 0, 0]
    assert s["runtime"]["updates"] == 1


# ---------- 装配期与纯函数门禁 ----------

def test_position_weight_vector_uses_leg_order_and_covers_all_joints():
    w = position_weight_vector({"RR": 1, "RL": 0, "FR": 1, "FL": 0}, INDEX_MAP, 0.25, 0.75)
    assert np.allclose(w, [0.75, 0.75, 0.75, 0.25, 0.25, 0.25, 0.75, 0.75, 0.75,
                           0.25, 0.25, 0.25])
    flat = position_weight_vector([1, 0, 1, 0], INDEX_MAP, 0.0, 1.0)
    assert np.allclose(flat, [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1])


@pytest.mark.parametrize("bad_mask", [
    {"FL": 1, "FR": 1, "RL": 0},                     # 缺腿
    {"FL": 1, "FR": 1, "RL": 0, "RR": 0, "XX": 1},   # 多腿
    [1, 1, 0],                                       # 长度错
    [1, 1, 0, 2],                                    # 取值非法
])
def test_leg_mask_gates(bad_mask):
    with pytest.raises(ValueError):
        position_weight_vector(bad_mask, INDEX_MAP, 0.0, 1.0)


@pytest.mark.parametrize("bad_map", [
    {"FL": (0, 1, 2), "FR": (3, 4, 5), "RL": (6, 7, 8)},              # 缺腿
    {"FL": (0, 1, 2), "FR": (3, 4, 5), "RL": (6, 7, 8), "RR": (8, 9, 10)},  # 下标重复/不覆盖
    {"FL": (0, 1, 2), "FR": (3, 4, 5), "RL": (6, 7, 8), "RR": (9, 10, 99)},  # 越界
    {"FL": (0, 1), "FR": (3, 4, 5), "RL": (6, 7, 8), "RR": (9, 10, 11)},     # 三元组缺项
])
def test_joint_index_map_gates(bad_map):
    with pytest.raises(ValueError):
        position_weight_vector([1, 1, 1, 1], bad_map, 0.0, 1.0)


@pytest.mark.parametrize("bad_weight", [(0.0, 1.5), (-0.5, 1.0), (0.0, float("nan"))])
def test_weight_range_gates_at_assembly(bad_weight):
    with pytest.raises(ValueError):
        MpcTorqueHook(ProviderRuntime(FakeClient(), ticks_per_update=1),
                      lambda: {"request": _req(), "current_mask": [1, 1, 0, 0]}, INDEX_MAP,
                      bad_weight[0], bad_weight[1], lambda: JT_IDENT, HORIZON)


@pytest.mark.parametrize("args", [(-1, 0), (0, -1), (HORIZON, HORIZON), (HORIZON, -1)])
def test_solution_step_gates(args):
    with pytest.raises(ValueError):
        solution_foot_forces([0.0] * NVARS, args[0], step=args[1])


def test_runtime_must_expose_will_update_and_step():
    class Bare:
        pass
    with pytest.raises(TypeError):
        MpcTorqueHook(Bare(), lambda: {}, INDEX_MAP, 0.0, 1.0, lambda: JT_IDENT, HORIZON)
