"""解新鲜度门禁与显式失败路径（第 4 块 `provider` 的策略核心，纯逻辑、无依赖）。

依据（ADR-0009 §4.3 / §6.5，写死）：
  · MPC 更新周期 **20.0 ms（50 Hz）**，对齐 `control_frequency_hz = 100`（每 2 拍更新一次）；
  · **解龄 > 2 个 MPC 周期（> 40.0 ms）⇒ `damped_hold`**；
  · 移动中 `stop`/失败一律 `damped_hold`（减速到零保持站立）；**急停/安全事件**才用 `torque_zero_release`；
  · **QP 无解、超时或状态过期必须有显式失败路径**，**禁止降级为"继续跑旧解"**；
  · 终态不得被回写为成功（铁律 6.6）。

本模块只做**判定**（返回决定 + 中文原因 + 标志位），不碰控制器；调用方据此执行。
"""

from __future__ import annotations

import math

__all__ = [
    "MPC_PERIOD_MS",
    "STALE_LIMIT_MS",
    "DECISION_OK",
    "DECISION_DAMPED_HOLD",
    "DECISION_TORQUE_ZERO_RELEASE",
    "decide",
]

# 单一事实来源（改这里即改全局；业务代码不得再写死这两个数）
MPC_PERIOD_MS = 20.0                 # ADR-0009 §6.5：50 Hz 解耦口径的预算
STALE_LIMIT_MS = 2.0 * MPC_PERIOD_MS  # = 40.0 ms

DECISION_OK = "ok"
DECISION_DAMPED_HOLD = "damped_hold"
DECISION_TORQUE_ZERO_RELEASE = "torque_zero_release"

# QP 状态分类 → 是否可用（`ok_inaccurate` 可用但必须带标志；其余一律失败）
_USABLE_STATUS = {"ok": True, "ok_inaccurate": True}


def decide(age_ms, qp_status_class, solve_ms=None, emergency=False):
    """决定本拍该怎么做，并给出**中文原因**与标志位。

    参数
    ----
    age_ms          : 解龄（ms）；`None`/`NaN`/负值视为**不可信**（显式失败，不当作新鲜）
    qp_status_class : `osqp_native.classify_status` 的分类字符串
                      （"ok" / "ok_inaccurate" / "infeasible" / "unbounded" / "max_iter" /
                       "time_limit" / "nonconvex" / "unknown"）
    solve_ms        : 本次求解耗时（ms，可选）；> `MPC_PERIOD_MS` 记为超预算（标志，不改变决定）
    emergency       : 是否处于急停/安全事件（**最高优先级**）

    返回
    ----
    dict(decision, reason, flags)  —— `decision ∈ {ok, damped_hold, torque_zero_release}`
    """
    flags = {"overran": False, "inaccurate": False, "invalid_age": False, "stale": False}

    # 1) 急停/安全事件优先级最高（铁律 6.6）
    if emergency:
        return {"decision": DECISION_TORQUE_ZERO_RELEASE,
                "reason": "急停/安全事件：优先级最高，切力矩释放", "flags": flags}

    # 2) 解龄不可信 ⇒ 显式失败（不得当作新鲜继续跑）
    if age_ms is None:
        flags["invalid_age"] = True
        return {"decision": DECISION_DAMPED_HOLD,
                "reason": "无可用解（尚未求解）⇒ 不得继续跑旧解", "flags": flags}
    try:
        age = float(age_ms)
    except (TypeError, ValueError):
        flags["invalid_age"] = True
        return {"decision": DECISION_DAMPED_HOLD,
                "reason": "解龄非数值 ⇒ 不可信，显式失败", "flags": flags}
    if math.isnan(age) or math.isinf(age) or age < 0.0:
        flags["invalid_age"] = True
        return {"decision": DECISION_DAMPED_HOLD,
                "reason": "解龄为 NaN/Inf/负值 ⇒ 时钟或时间戳异常，显式失败", "flags": flags}

    # 3) 状态过期 ⇒ damped_hold
    if age > STALE_LIMIT_MS:
        flags["stale"] = True
        return {"decision": DECISION_DAMPED_HOLD,
                "reason": "解龄 %.4f ms > %.1f ms（2 个 MPC 周期）⇒ 状态过期" % (age, STALE_LIMIT_MS),
                "flags": flags}

    # 4) QP 状态必须可用；未知状态**绝不视为成功**
    cls = str(qp_status_class)
    if cls not in _USABLE_STATUS:
        return {"decision": DECISION_DAMPED_HOLD,
                "reason": "QP 未给出可用解（状态分类 %r）⇒ 显式失败" % cls, "flags": flags}
    if not _USABLE_STATUS[cls]:
        flags["inaccurate"] = True
    if cls == "ok_inaccurate":          # 修正：首版写成 `not _USABLE_STATUS[cls]`（恒为 False）
        flags["inaccurate"] = True      # ⇒ 标志永不置位，被 test_inaccurate_is_usable_but_flagged 抓出

    # 5) 超预算只记标志（解仍然可用；是否可接受由验收判据决定，不在此放宽）
    if solve_ms is not None:
        try:
            sm = float(solve_ms)
            flags["overran"] = bool(sm > MPC_PERIOD_MS)
        except (TypeError, ValueError):
            flags["overran"] = False

    reason = "解可用（解龄 %.4f ms ≤ %.1f ms）" % (age, STALE_LIMIT_MS)
    if cls == "ok_inaccurate":
        reason += "；注意：求解状态为 solved inaccurate"
    if flags["overran"]:
        reason += "；注意：本次求解耗时超过预算 %.1f ms" % MPC_PERIOD_MS
    return {"decision": DECISION_OK, "reason": reason, "flags": flags}
