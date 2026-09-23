"""MPC 参考轨迹的**接触表**（A6a-3b 第②步下半块之一）。

移植清单 §1.1 的落地：接触表的**唯一事实来源是 `iraf_adapters.unitree.gait` 的声明**
（`legs.<code>.phase_offset` + `duty_factor` + `period_s`），本模块**不复制**相位公式，
只负责三件事：

  1. **腿序**：上游 `gait.PHASE_OFFSET = [0.5, 0.0, 0.0, 0.5]` 的索引顺序是 `[FL, FR, RL, RR]`
     （注意 **RL 在 RR 之前**，与我们既有 wave 声明里的书写顺序不同）⇒ 显式定为 `LEG_ORDER`，
     不靠字典迭代顺序；
  2. **采样时刻**：上游 `compute_contact_table` 取 `t = t0 + arange(N)·dt + dt/2`（**半拍前移**），
     `compute_current_mask` 取 `t = t0`（`dt=0, N=1` ⇒ 半拍项为 0，两者自洽）；
  3. **相位/支撑判定**：调用 `gait.leg_phase` 与 `gait.is_stance`，逐条腿求值后按腿序堆叠。

上游语义（`build/research/mpc-repo/src/convex_mpc/gait.py:26-37`）：
    t = t0 + arange(N)·dt + dt/2 ; phases = mod(offset + t/T, 1) ; contact = phases < duty
我们的 `gait.leg_phase` 是 `((elapsed/T) + offset) % 1`、`gait.is_stance` 是 `phase < duty_factor`
⇒ 同一 `elapsed` 下两条式子逐位相同（`mod` 与取模顺序不同但实数结果一致，纯浮点加法顺序一致）。

未验证边界：本模块的**逐位校验**（与研究侧同一 `t0/dt/N` 下比较 `contact_table`）在下一步做；
在此之前不得接进 `provider_runtime`。

已实测的相位结构（本模块单测覆盖，**与直觉不同故记下**）：`duty = 0.6 > 0.5` 且两组相位偏移
相距 0.5 时，两组支撑窗口各长 `duty = 0.6`、在相位环上错开半周期 ⇒ 交集是**两段**
（一段 `[0, duty−0.5)`、一段跨环边界 `[0.5, 0.6)`），总长 `2·(duty − 0.5) = 0.2` 周期。
即：存在**四足同时支撑**窗口，且**不存在腾空相**。「trot 恒为两条腿支撑」只在 `duty ≤ 0.5` 时成立
（`duty = 0.5` 退化为恒两条腿交替）。单测实测重叠占比 `12/64 = 0.1875`（`dt = T/64` 的采样量化
把 0.2 切成 12 格），与分析式一致。
"""

from __future__ import annotations

import numpy as np

from iraf_adapters.unitree import gait as gait_mod

__all__ = ["LEG_ORDER", "MPC_GAIT_KIND", "contact_table", "current_mask"]

#: 上游 `PHASE_OFFSET` 的索引顺序（**RL 在 RR 之前**）。改为别的顺序会让控制律的
#: 足端力与腿的对应关系整体错位，而数值上「看起来仍然自洽」⇒ 显式写死并做入参校验。
LEG_ORDER = ("FL", "FR", "RL", "RR")

#: 本 Provider 只允许对角小跑（③ 的倾角上界与 ② 的跟踪数字都只在 trot 下取过）。
MPC_GAIT_KIND = "trot"


def _check_params(params):
    kind = params.get("kind")
    if kind != MPC_GAIT_KIND:
        raise gait_mod.DeclarationError(
            "MPC 接触表要求 gait.kind = %r，实际 %r（其它步态未验收，不得静默套用）"
            % (MPC_GAIT_KIND, kind)
        )
    legs = params.get("legs")
    if not isinstance(legs, dict):
        raise gait_mod.DeclarationError("gait.legs 必须是映射，实际: %r" % (type(legs).__name__,))
    missing = [code for code in LEG_ORDER if code not in legs]
    if missing:
        raise gait_mod.DeclarationError(
            "gait.legs 缺少 MPC 腿序所需的腿：%s（上游腿序 = %s）"
            % (", ".join(missing), "/".join(LEG_ORDER))
        )
    # trot 的相位结构：恰好两组、每组两条腿、组间半周期（复用既有声明校验的同一判据）。
    groups = params.get("phase_groups")
    if not isinstance(groups, list) or len(groups) != 2 or any(len(g["legs"]) != 2 for g in groups):
        raise gait_mod.DeclarationError(
            "MPC 接触表要求 trot 的相位结构（2 组 × 2 条腿），实际: %r" % (groups,)
        )


def contact_table(params, t0, dt, n, *, half_step=True):
    """`(4, N)` 的接触表：行序 = `LEG_ORDER`，`1` = 支撑、`0` = 摆动（int32，同上游 dtype）。

    `half_step=True` 复刻上游 `compute_contact_table` 的 `+dt/2`；`current_mask(params, t)`
    是它的单点特例。
    """
    _check_params(params)
    t0 = float(t0)
    dt = float(dt)
    n = int(n)
    if n < 1:
        raise ValueError("n 必须 ≥1，实际 %r" % (n,))
    times = t0 + np.arange(n, dtype=float) * dt + (dt / 2.0 if half_step else 0.0)
    rows = []
    for code in LEG_ORDER:
        rows.append([1 if gait_mod.is_stance(params, gait_mod.leg_phase(params, code, t))
                     else 0 for t in times])
    return np.array(rows, dtype=np.int32)


def current_mask(params, t):
    """上游 `compute_current_mask` 等价物：`(4,)` 的当拍接触掩码（腿序同 `LEG_ORDER`）。"""
    return contact_table(params, t, 0.0, 1, half_step=False)[:, 0]
