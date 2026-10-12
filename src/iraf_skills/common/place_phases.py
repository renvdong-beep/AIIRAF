# -*- coding: utf-8 -*-
"""放置相位的**单点声明**（P4 / B1+B2，2026-10-12）。

为什么需要（AGENTS 3：分层；AGENTS 2.8：证据单一来源）
------------------------------------------------------
在引入本模块之前，「放置相位集合」被抄成了**多份字面量**：
  · `scripts/build_piper_baseline.py` 的 plan 组装（above/descend/retreat/transit/via）；
  · `src/iraf_adapters/mujoco/mujoco_backend.py` 的必需解检查、白名单与回放顺序；
  · `scripts/probe_place_feedforward.py` 的局部 `PLACE_PHASES`（只 2~4 项）。
后果是「加一个相位要改 4 处，探针常常漏改」——2026-10-08 就因为探针漏掉 `via` 误判过一次
（探针报 `path:transit>above`，而实际已是 `via>above`）。

本模块把这件事收敛为**一处声明**：相位名、顺序、是否可选、是否需要 CarryGoal、
对应的报告 positions 键名。求解器/构建器/后端/探针都从这里取，禁止再写第二份字面量列表。

口径（逐位不变）
----------------
`via` / `transit` 缺省**不插入**（optional=True 且报告里有对应 key 才回放）；其余相位的
顺序与必需性保持与改动前完全一致 ⇒ 缺省链路逐位不变。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlacePhase:
    """一个放置相位：名字 + 报告的 positions 键 + 可选性 + 是否需要 CarryGoal。

    `name`           —— 相位逻辑名（用于日志/证据相位标签，如 `above` / `via`）。
    `positions_key`  —— 场景报告 `gripper` 段里承载该段关节解的键名
                        （如 `place_above_positions`）。
    `optional`       —— True = 允许缺席（缺席即不插入该相位；`via` / `transit` 为 True）。
    `requires_carry_goal` —— True = 搬运期回放该段时要套 `_carry_goal`（保持夹紧力语义）。
    """

    name: str
    positions_key: str
    optional: bool
    requires_carry_goal: bool


#: 放置相位的**规范顺序**（与后端回放顺序、构建器求解顺序一致）。
#: 顺序即事实：后端按它回放，构建器按它求解，探针按它比对。
PLACE_PHASES = (
    PlacePhase("transit", "place_transit_positions", optional=True, requires_carry_goal=True),
    PlacePhase("via", "place_via_positions", optional=True, requires_carry_goal=True),
    PlacePhase("above", "place_above_positions", optional=False, requires_carry_goal=True),
    PlacePhase("descend", "place_descend_positions", optional=False, requires_carry_goal=True),
    PlacePhase("retreat", "place_retreat_positions", optional=False, requires_carry_goal=False),
)

#: 相位逻辑名，按规范顺序（探针/日志的可读顺序）。
PLACE_PHASE_NAMES = tuple(phase.name for phase in PLACE_PHASES)

#: 报告 `gripper` 段的 positions 键，按规范顺序（白名单/解析层从这里取）。
PLACE_POSITIONS_KEYS = tuple(phase.positions_key for phase in PLACE_PHASES)

#: **必需**的 positions 键（缺席即显式失败）：放置段必须由声明的求解器在构建期解出。
REQUIRED_PLACE_POSITIONS_KEYS = tuple(
    phase.positions_key for phase in PLACE_PHASES if not phase.optional)

#: **可选**的 positions 键（缺席即不插入该相位，逐位不变）。
OPTIONAL_PLACE_POSITIONS_KEYS = tuple(
    phase.positions_key for phase in PLACE_PHASES if phase.optional)

#: 逻辑名 → 相位（供按名字取键的消费方用，避免再写一遍映射）。
PLACE_PHASE_BY_NAME = {phase.name: phase for phase in PLACE_PHASES}

#: positions 键 → 逻辑名（探针从报告读键名时反查相位名用）。
PLACE_PHASE_NAME_BY_KEY = {phase.positions_key: phase.name for phase in PLACE_PHASES}


def place_phase_names(positions_keys):
    """把一组 positions 键按**规范顺序**映射成相位逻辑名。

    用途：探针/证据标签 —— 从报告里读到的键名（可能只含一个子集，如只有 above/descend）
    转成规范顺序的相位名，避免各消费方各写一份排序。
    未知键保留原名（不静默丢弃，便于暴露"报告里有新相位但表里没登记"）。
    """
    present = set(positions_keys)
    ordered = [phase.name for phase in PLACE_PHASES if phase.positions_key in present]
    unknown = [key for key in positions_keys if key not in PLACE_PHASE_NAME_BY_KEY]
    return tuple(ordered) + tuple(unknown)
