"""计划类 Provider：调用端侧运动学 planner，返回**计划**而不是执行结果。

不调用 backend、不申请运动租约、不产生物理动作；输出强制 planned=true, physics=false。
"""
from __future__ import annotations

from iraf_adapters.kinematic.planner import PlanError, plan_pick


class PlanTaskProvider:
    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        try:
            return plan_pick(
                scene_id=str(inputs.get("scene_id") or "ur5-pick-planned"),
                target_id=str(inputs.get("target_id") or "box_01"),
                approach_m=float(inputs.get("approach_m", 0.10)),
                lift_m=float(inputs.get("lift_m", 0.05)),
                duration_ms=int(inputs.get("duration_ms", 3000)),
            )
        except PlanError as exc:
            return {"status": "FAILED", "error_code": exc.code, "reason": exc.detail,
                    "planned": True, "physics": False, "backend": "kinematic"}