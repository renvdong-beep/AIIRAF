"""场景管理类 Skill Provider（开发/调试用）。

为什么单独一个模块：`reset_scene` 改的是**世界状态**而不是机器人动作，
但它同样必须经过 Skill Runtime → Policy → Provider 与资源租约，不能走 shell 旁路
（2026-10-23 使用者口径：复位应该是一个 IRAF 技能，而不是运维脚本）。
"""
from .motion import SkillRejected


class ResetSceneProvider:
    """只接受 Backend 以真实仿真状态确认的复位结果。"""

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        scope = str(inputs.get("scope") or "all").strip().lower()
        target_id = inputs.get("target_id")
        if scope == "target" and not target_id:
            raise SkillRejected("scope=target 必须给 target_id")
        if scope not in ("all", "target"):
            raise SkillRejected("scope 只接受 all / target，实际：%s" % scope)
        if not hasattr(self.backend, "reset_scene"):
            raise SkillRejected("Backend 未实现 reset_scene，拒绝伪造复位成功")

        result = self.backend.reset_scene(scope=scope, target_id=target_id, lease=lease)
        if not isinstance(result, dict) or result.get("status") != "SUCCEEDED":
            raise SkillRejected("场景复位未确认成功：%r" % (result,))
        return result
