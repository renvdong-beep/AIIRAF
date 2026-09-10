"""平台无关的操作 Skill Provider。"""

import math

from .motion import SkillRejected


class PickObjectProvider:
    """只接受 Backend 以真实仿真状态确认的抓取结果。"""

    _CONFIRMATIONS = frozenset({"contact", "constraint", "gripper_state"})

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        target_id = inputs["target_id"]
        pose = inputs["grasp_pose"]
        self._validate_pose(pose)
        if not hasattr(self.backend, "pick_object"):
            raise SkillRejected("Backend 未实现 pick_object，拒绝伪造抓取成功")

        result = self.backend.pick_object(
            target_id=target_id,
            grasp_pose=pose,
            duration_ms=int(inputs.get("duration_ms", 1000)),
            lease=lease,
        )
        if not isinstance(result, dict):
            raise SkillRejected("Backend 未返回可验证的抓取结果")
        if result.get("target_id") != target_id:
            raise SkillRejected("Backend 抓取结果与目标不匹配")
        if result.get("grasped") is not True:
            raise SkillRejected("Backend 未确认目标已抓取")
        confirmation = result.get("confirmation")
        if confirmation not in self._CONFIRMATIONS:
            raise SkillRejected("Backend 抓取确认类型不受信")
        return {
            "skill": "pick_object",
            "accepted": True,
            "target_id": target_id,
            "confirmation": confirmation,
        }

    @staticmethod
    def _validate_pose(pose):
        values = tuple(pose["position"].values()) + tuple(
            pose["orientation"].values()
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise SkillRejected("抓取位姿必须是有限数值")
        orientation = pose["orientation"]
        norm = math.sqrt(
            sum(float(orientation[key]) ** 2 for key in ("x", "y", "z", "w"))
        )
        if not 0.999 <= norm <= 1.001:
            raise SkillRejected("抓取姿态四元数必须归一化")
