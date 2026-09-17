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
        output = {
            "skill": "pick_object",
            "accepted": True,
            "target_id": target_id,
            "confirmation": confirmation,
        }
        evidence = result.get("evidence")
        if evidence is not None:
            if not isinstance(evidence, dict):
                raise SkillRejected("Backend 抓取证据格式无效")
            output["evidence"] = evidence
        return output

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


class CalibrateGraspProvider:
    """通过 Backend 读取模型几何，禁止在 Skill 层猜测标定值。"""

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        if not hasattr(self.backend, "calibrate_grasp"):
            raise SkillRejected("Backend 未实现 calibrate_grasp")
        evidence = self.backend.calibrate_grasp(inputs, lease)
        return {"skill": "calibrate_grasp", "accepted": True, "evidence": evidence}


class VisualPickProvider:
    """只接受 Backend 返回的视觉目标位姿，再进入统一抓取闭环。"""

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        if not hasattr(self.backend, "visual_pick"):
            raise SkillRejected("Backend 未实现 visual_pick，拒绝回退到先验坐标")
        result = self.backend.visual_pick(inputs, lease)
        if not isinstance(result, dict) or result.get("grasped") is not True:
            raise SkillRejected("视觉抓取未通过目标位姿或接触力验收")
        return {"skill": "visual_pick", "accepted": True, "target_id": inputs["target_id"], "evidence": result.get("evidence", {})}


class CalibrateCameraProvider:
    """相机外参标定入口，具体拟合由 Backend/标定工具实现。"""

    def __init__(self, profile, backend):
        self.backend = backend

    def execute(self, inputs, lease):
        if not hasattr(self.backend, "calibrate_camera_to_base"):
            raise SkillRejected("Backend 未实现 calibrate_camera_to_base")
        return self.backend.calibrate_camera_to_base(inputs, lease)


"""显示专用 Provider：复用 visual_pick 的 Backend 闭环，仅放宽时长上限。

设计边界（务必保持）：
- **不新增任何物理行为**：直接委托 backend.visual_pick(...)，
  与 VisualPickProvider 是同一个调用，因此抓取语义完全一致；
- **不放宽验收判据**：本 Provider 只用于"看得见完整过程"的显示场景，
  验收链仍走 visual_pick + profiles/safety/simulation_lab.yaml；
- **仍经完整策略链**：作为普通 skill 由 Runtime 调度，
  运动租约、安全策略、审计记录一律不绕过（AGENTS.md 铁律 2）。
"""

from .motion import SkillRejected


class DisplayPickProvider:
    """与 VisualPickProvider 同源，仅把时长上限交给显示用 skill 声明。"""

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        if not hasattr(self.backend, "visual_pick"):
            raise SkillRejected("Backend 未实现 visual_pick，拒绝回退到先验坐标")
        result = self.backend.visual_pick(inputs, lease)
        if not isinstance(result, dict) or result.get("grasped") is not True:
            raise SkillRejected("视觉抓取未通过目标位姿或接触力验收")
        return {
            "skill": "display_pick",
            "accepted": True,
            "target_id": inputs["target_id"],
            "evidence": result.get("evidence", {}),
        }
