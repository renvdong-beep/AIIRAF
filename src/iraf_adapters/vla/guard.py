"""VLA 动作护栏：关节限位、速率限制、夹爪范围、动作维度校验。

只做“过滤/拒绝”，不修改模型、不伪造成功；拒绝时返回原因供上层回退。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class VlaLimits:
    joint_min: tuple[float, ...] = (-2.618, 0.0, -2.967, -1.745, -1.22, -2.094)
    joint_max: tuple[float, ...] = (2.168, 3.14, 0.0, 1.745, 1.22, 2.094)
    max_step_rad: float = 0.20          # 单控制周期最大关节变化
    gripper_min: float = 0.005
    gripper_max: float = 0.045
    expected_dim: int = 7


@dataclass
class GuardResult:
    action: np.ndarray
    accepted: bool
    violations: list[str] = field(default_factory=list)
    clamped: bool = False


class ActionGuard:
    def __init__(self, limits: VlaLimits | None = None):
        self.limits = limits or VlaLimits()
        lo = np.asarray(self.limits.joint_min, dtype=float)
        hi = np.asarray(self.limits.joint_max, dtype=float)
        if lo.shape != (6,) or hi.shape != (6,):
            raise ValueError("joint_min/joint_max must be 6-dim")

    def check(self, action: Sequence[float],
              previous: Sequence[float] | None = None) -> GuardResult:
        a = np.asarray(action, dtype=float).reshape(-1)
        lim = self.limits
        if a.shape[0] != lim.expected_dim:
            return GuardResult(a, False, ["dim=%d expected=%d" % (a.shape[0], lim.expected_dim)])
        if not np.all(np.isfinite(a)):
            return GuardResult(a, False, ["non-finite"])

        lo = np.asarray(lim.joint_min, dtype=float)
        hi = np.asarray(lim.joint_max, dtype=float)
        violations: list[str] = []
        out = a.copy()

        before = out[:6].copy()
        out[:6] = np.clip(out[:6], lo, hi)
        for j in range(6):
            if abs(out[j] - before[j]) > 1e-12:
                violations.append("joint%d out of [%.3f,%.3f]" % (j + 1, lo[j], hi[j]))

        if previous is not None and lim.max_step_rad > 0:
            prev = np.asarray(previous, dtype=float).reshape(-1)
            if prev.shape[0] >= 6:
                delta = out[:6] - prev[:6]
                over = np.abs(delta) > lim.max_step_rad
                if np.any(over):
                    out[:6] = prev[:6] + np.clip(delta, -lim.max_step_rad, lim.max_step_rad)
                    out[:6] = np.clip(out[:6], lo, hi)
                    violations.append("rate-limit")

        grip = float(out[6])
        if grip < lim.gripper_min or grip > lim.gripper_max:
            violations.append("gripper out of [%.3f,%.3f]" % (lim.gripper_min, lim.gripper_max))
            out[6] = float(np.clip(grip, lim.gripper_min, lim.gripper_max))

        return GuardResult(out, not violations, violations, bool(violations))
