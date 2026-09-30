"""平台无关的操作 Skill Provider。"""

import math

from .motion import SkillRejected


class PickObjectProvider:
    """只接受 Backend 以真实仿真状态确认的抓取结果。"""

    _CONFIRMATIONS = frozenset({"contact", "constraint", "gripper_state"})
    #: 与 `skills/pick_object/pick_object.input.json` 的 `grasp_pose.pose_source` 同口径。
    _POSE_SOURCES = ("world_absolute", "live_target_body")

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

    @classmethod
    def _validate_pose(cls, pose):
        """按**声明的来源**校验抓取位姿（2026-09-30，§11.27）。

        `live_target_body`：坐标由 Backend 在**执行时刻**按目标体当前位姿解析
        （仿真真值 FK；真机应由感知 Provider 提供）⇒ 这里**不校验不存在的数字**，
        只拒绝"两份事实"（同时给坐标会让来源分叉）。缺省 `world_absolute` ⇒ 行为与改动前一致。
        """
        source = pose.get("pose_source", "world_absolute")
        if source not in cls._POSE_SOURCES:
            raise SkillRejected("抓取位姿来源不受支持: %r（可用: %s）"
                                % (source, list(cls._POSE_SOURCES)))
        if source == "live_target_body":
            if "position" in pose or "orientation" in pose:
                raise SkillRejected(
                    "pose_source=live_target_body 时不得同时给出 position/orientation（两份事实必然分叉）"
                )
            return
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


class PlaceObjectProvider:
    """只接受 Backend 以**真实仿真状态**确认的放置结果：夹爪已张开 **且** 载荷实测落在接收体上。

    为什么两条都要（契约 `place_object.output.json`）：`released` 只证明"夹爪开了"，
    载荷可能仍被带着走或掉在别处 ⇒ 必须另有 `payload_in_tray` 的**实测**判据；
    任一条不成立即拒绝报告成功（铁律 1.5：不得返回伪造成功）。
    """

    _CONFIRMATIONS = frozenset({"released"})

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        place_target_id = inputs["place_target_id"]
        payload_id = inputs["payload_id"]
        if not hasattr(self.backend, "place_object"):
            raise SkillRejected("Backend 未实现 place_object，拒绝伪造放置成功")
        result = self.backend.place_object(
            place_target_id=place_target_id,
            payload_id=payload_id,
            duration_ms=int(inputs.get("duration_ms", 1000)),
            lease=lease,
        )
        if not isinstance(result, dict):
            raise SkillRejected("Backend 未返回可验证的放置结果")
        if (result.get("place_target_id") != place_target_id
                or result.get("payload_id") != payload_id):
            raise SkillRejected("Backend 放置结果与请求的接收体/载荷不匹配")
        if result.get("released") is not True:
            raise SkillRejected("Backend 未确认载荷已放下")
        confirmation = result.get("confirmation")
        if confirmation not in self._CONFIRMATIONS:
            raise SkillRejected("Backend 放置确认类型不受信")
        output = {"skill": "place_object", "accepted": True,
                  "place_target_id": place_target_id, "payload_id": payload_id,
                  "confirmation": confirmation}
        evidence = result.get("evidence")
        if evidence is not None:
            if not isinstance(evidence, dict):
                raise SkillRejected("Backend 放置证据格式无效")
            if evidence.get("payload_in_tray") is not True:
                raise SkillRejected("Backend 证据显示载荷不在接收体上，拒绝报告成功")
            output["evidence"] = evidence
        return output
