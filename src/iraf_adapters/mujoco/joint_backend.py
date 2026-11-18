#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""联合世界 Backend：一份 MkModel/MjData 上组合多个机器人适配器，按 resource_id 路由。

设计边界（docs/plans/2026-11-18-joint-scene-composer.md P1）：

- **不新造协议**：仍实现 Capability Backend 的同一批方法（pick_object/stand/...），
  由 `IRAF_BACKEND_ENTRYPOINT` 选择本类；
- **一份植物**：四足适配器是 owner（唯一时间推进者），机械臂适配器以 guest 注入同一 plant；
- **按租约资源路由**：`lease.resource_id` 决定落到哪台机器人；缺资源/缺能力显式失败，
  不做“猜一个”或静默降级；
- 现有单机 provider 不受影响（本类只在联合场景的装配单里启用）。
"""
from __future__ import annotations

from pathlib import Path

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_adapters.unitree.quadruped import CAPABILITIES as QUADRUPED_CAPABILITIES
from iraf_adapters.unitree.unitree_go2 import UnitreeGo2Adapter, repo_root
from iraf_core.profile import load_robot_profile

#: 本类真正实现的能力 = 机械臂（MujocoBackend）+ 四足（UnitreeGo2Adapter）的并集。
#: declared ⊆ implemented 由 factory 的装配期校验执行；这里不做任何“声明即实现”的假设。
IMPLEMENTED_CAPABILITIES = frozenset(
    set(MujocoBackend.IMPLEMENTED_CAPABILITIES) | set(UnitreeGo2Adapter.IMPLEMENTED_CAPABILITIES)
) if hasattr(MujocoBackend, "IMPLEMENTED_CAPABILITIES") else frozenset(
    {"move_joint", "pick_object", "visual_pick", "place_object", "reset_scene", "stop",
     "stand", "locomote", "dock_for_handoff", "accept_payload", "read_state"}
)


class JointBindingError(ValueError):
    """联合世界的资源/能力绑定错误（显式失败，不降级）。"""


class JointMujocoBackend:
    """联合世界 Capability Backend（piper/ur5 + go2 共用一个 MjModel/MjData）。"""

    CAPABILITIES = IMPLEMENTED_CAPABILITIES
    IMPLEMENTED_CAPABILITIES = IMPLEMENTED_CAPABILITIES
    #: 资源别名 → 子后端。composer 生成装配单时写入 `config["resources"]`。
    DEFAULT_RESOURCES = {
        "piper": "arm",
        "ur5": "arm",
        "ur5e": "arm",
        "go2": "quadruped",
        "unitree_go2": "quadruped",
    }

    @classmethod
    def from_config(cls, config, profile, authority):
        root = Path(repo_root())
        go2_cfg = dict(config.get("go2") or {})
        if not go2_cfg:
            raise JointBindingError("联合装配缺少 go2 段（declaration/profile）")
        go2_profile_path = go2_cfg.pop("profile", "profiles/unitree_go2_mujoco.yaml")
        go2_profile = load_robot_profile(root / go2_profile_path)
        go2 = UnitreeGo2Adapter.from_config(go2_cfg, go2_profile, authority)

        arm_cfg = dict(config.get("piper") or {})
        arm_cfg.setdefault("model_path", config.get("model_path"))
        arm_cfg["plant"] = go2.plant  # 机械臂是 guest：共享四足拥有的植物
        arm_profile_path = arm_cfg.pop("profile", "profiles/piper_mujoco.yaml")
        arm_profile = load_robot_profile(root / arm_profile_path)
        arm = MujocoBackend.from_config(arm_cfg, arm_profile, authority)

        return cls(profile=profile, authority=authority, arm=arm, quadruped=go2,
                   resources=dict(config.get("resources") or cls.DEFAULT_RESOURCES),
                   go2_profile=go2_profile, arm_profile=arm_profile)

    def __init__(self, *, profile, authority, arm, quadruped, resources,
                 go2_profile=None, arm_profile=None):
        self.profile = profile
        self.authority = authority
        self._arm = arm
        self._quadruped = quadruped
        self._resources = dict(resources)
        self.model = getattr(quadruped, "model", getattr(arm, "model", None))
        self.data = getattr(quadruped, "data", getattr(arm, "data", None))
        self.plant = getattr(quadruped, "plant", None)
        self._go2_profile = go2_profile
        self._arm_profile = arm_profile

    # ---- 路由 ----
    def _kind(self, lease, resource_id=""):
        rid = str(resource_id or getattr(lease, "resource_id", "") or "").strip()
        if not rid:
            raise JointBindingError("联合世界缺少 resource_id：拒绝猜测目标机器人")
        kind = self._resources.get(rid)
        if not kind:
            raise JointBindingError(
                "联合世界未登记资源 %r（已登记：%s）" % (rid, sorted(self._resources))
            )
        return kind, rid

    def _arm_backend(self):
        return self._arm

    def _quad_backend(self):
        return self._quadruped

    # ---- 机械臂能力（piper / ur5） ----
    def move_joint(self, *args, **kwargs):
        return self._arm.move_joint(*args, **kwargs)

    def pick_object(self, *args, **kwargs):
        return self._arm.pick_object(*args, **kwargs)

    def place_object(self, *args, **kwargs):
        return self._arm.place_object(*args, **kwargs)

    def visual_pick(self, *args, **kwargs):
        return self._arm.visual_pick(*args, **kwargs)

    def reset_scene(self, *args, **kwargs):
        return self._arm.reset_scene(*args, **kwargs)

    # ---- 四足能力（go2） ----
    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        kind, _ = self._kind(lease)
        if kind != "quadruped":
            raise JointBindingError("stand 只能用于四足资源（lease.resource_id=%s）"
                                    % getattr(lease, "resource_id", ""))
        return self._quadruped.stand(lease, targets=targets,
                                     duration_ms=duration_ms, execution_id=execution_id)

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        kind, _ = self._kind(lease)
        if kind != "quadruped":
            raise JointBindingError("locomote 只能用于四足资源")
        return self._quadruped.locomote(velocity, duration_ms, lease,
                                        execution_id=execution_id)

    def dock_for_handoff(self, *, lease, position_tolerance_m, yaw_tolerance_rad,
                         max_final_speed_mps, execution_id=None):
        kind, _ = self._kind(lease)
        if kind != "quadruped":
            raise JointBindingError("dock_for_handoff 只能用于四足资源")
        return self._quadruped.dock_for_handoff(
            lease=lease, position_tolerance_m=position_tolerance_m,
            yaw_tolerance_rad=yaw_tolerance_rad,
            max_final_speed_mps=max_final_speed_mps, execution_id=execution_id)

    def accept_payload(self, *args, **kwargs):
        return self._quadruped.accept_payload(*args, **kwargs)

    def read_state(self, *args, **kwargs):
        return self._quadruped.read_state(*args, **kwargs)

    # ---- 通用 ----
    def stop(self, lease, execution_id=None):
        kind, _ = self._kind(lease)
        target = self._quadruped if kind == "quadruped" else self._arm
        return target.stop(lease, execution_id=execution_id)

    def emergency_stop(self, reason):
        reports = []
        for backend in (self._quadruped, self._arm):
            try:
                reports.append(backend.emergency_stop(reason))
            except Exception as exc:  # noqa: BLE001 - 急停必须尝试两侧，失败要可见
                reports.append({"error": "%s: %s" % (type(exc).__name__, exc)})
        return {"simulation": True, "capability": "emergency_stop",
                "reports": reports}
