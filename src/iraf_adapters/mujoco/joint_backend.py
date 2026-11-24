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

import threading
import time
from contextlib import contextmanager
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
        # 场景注册表的资源 id（joint 场景）
        "joint-piper": "arm",
        "joint-ur5": "arm",
        "joint-go2": "quadruped",
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

        arms = {}
        piper_cfg = dict(config.get("piper") or {})
        ur5_cfg = dict(config.get("ur5") or {})
        for key, cfg, default_profile in (("piper", piper_cfg, "profiles/piper_mujoco.yaml"),
                                          ("ur5", ur5_cfg, "profiles/ur5_mujoco.yaml")):
            if not cfg:
                continue
            cfg.setdefault("model_path", config.get("model_path"))
            cfg["plant"] = go2.plant  # 机械臂是 guest：共享四足拥有的植物
            cfg.setdefault("plant_guest_timeout_factor", 3.0)
            profile_path = cfg.pop("profile", default_profile)
            arms[key] = MujocoBackend.from_config(cfg, load_robot_profile(root / profile_path), authority)
        if not arms:
            raise JointBindingError("联合装配至少需要一台机械臂（piper/ur5）")

        return cls(profile=profile, authority=authority, arms=arms, quadruped=go2,
                   resources=dict(config.get("resources") or cls.DEFAULT_RESOURCES))

    def __init__(self, *, profile, authority, arms, quadruped, resources):
        self.profile = profile
        self.authority = authority
        self._arms = dict(arms)
        self._quadruped = quadruped
        self._resources = dict(resources)
        first = next(iter(self._arms.values()))
        self.model = getattr(quadruped, "model", getattr(first, "model", None))
        self.data = getattr(quadruped, "data", getattr(first, "data", None))
        self.plant = getattr(quadruped, "plant", None)

    def _arm_for(self, lease):
        rid = str(getattr(lease, "resource_id", "") or "")
        if "ur5" in rid and "ur5" in self._arms:
            return self._arms["ur5"]
        if "piper" in rid and "piper" in self._arms:
            return self._arms["piper"]
        # 缺资源名（场景/Profile 名）时：只有一台臂就直接用它；多台必须显式
        if len(self._arms) == 1:
            return next(iter(self._arms.values()))
        raise JointBindingError("联合世界有多台机械臂，但 lease.resource_id=%r 无法区分" % rid)

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
        return next(iter(self._arms.values()))

    def _quad_backend(self):
        return self._quadruped

    # ---- 植物推进：机械臂是 guest，动作期间必须由 owner（四足）推进时间 ----
    @contextmanager
    def _advance_plant(self):
        stop = threading.Event()
        owner = self._quadruped
        plant = self.plant

        def loop():
            while not stop.is_set():
                try:
                    plant.step_once(owner)
                except Exception:
                    break
                time.sleep(0.0005)

        thread = threading.Thread(target=loop, daemon=True, name="joint-plant-stepper")
        thread.start()
        try:
            yield
        finally:
            stop.set()
            thread.join(timeout=1.0)

    # ---- 机械臂能力（piper / ur5） ----
    def move_joint(self, *args, **kwargs):
        lease = kwargs.get("lease") or (args[-1] if args else None)
        with self._advance_plant():
            return self._arm_for(lease).move_joint(*args, **kwargs)

    def pick_object(self, *args, **kwargs):
        lease = kwargs.get("lease") or (args[-1] if args else None)
        with self._advance_plant():
            return self._arm_for(lease).pick_object(*args, **kwargs)

    def pick_random(self, lease):
        with self._advance_plant():
            return self._arm_for(lease).pick_random(lease)

    def place_object(self, *args, **kwargs):
        lease = kwargs.get("lease") or (args[-1] if args else None)
        with self._advance_plant():
            return self._arm_for(lease).place_object(*args, **kwargs)

    def visual_pick(self, *args, **kwargs):
        lease = kwargs.get("lease") or (args[-1] if args else None)
        with self._advance_plant():
            return self._arm_for(lease).visual_pick(*args, **kwargs)

    def reset_scene(self, *args, **kwargs):
        """联合世界复位：直接把共享 MjData 恢复到模型初始 qpos0。"""
        import mujoco
        lease = kwargs.get("lease") or (args[-1] if args else None)
        self.authority.validate(lease)
        lock = self.plant.lock() if callable(getattr(self.plant, "lock", None)) else None
        if lock is None:
            raise RuntimeError("联合世界复位需要 plant.lock")
        with lock:
            if int(getattr(self.model, "nkey", 0) or 0) > 0:
                mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            else:
                self.data.qpos[:] = self.model.qpos0
                self.data.qvel[:] = 0
            self.data.eq_active[:] = 0
            mujoco.mj_forward(self.model, self.data)
        baseline = "keyframe:0" if int(getattr(self.model, "nkey", 0) or 0) > 0 else "initial-snapshot"
        return {"status": "SUCCEEDED", "scope": "all", "baseline": baseline,
                "detail": "联合世界已复位到模型初始状态"}

    # ---- 四足能力（go2） ----
    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        # 能力本身决定目标机器人：stand 只可能落在四足上（resource_id 可能是场景/Profile 名）。
        return self._quadruped.stand(lease, targets=targets,
                                     duration_ms=duration_ms, execution_id=execution_id)

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        return self._quadruped.locomote(velocity, duration_ms, lease,
                                        execution_id=execution_id)

    def dock_for_handoff(self, *, lease, position_tolerance_m, yaw_tolerance_rad,
                         max_final_speed_mps, execution_id=None):
        return self._quadruped.dock_for_handoff(
            lease=lease, position_tolerance_m=position_tolerance_m,
            yaw_tolerance_rad=yaw_tolerance_rad,
            max_final_speed_mps=max_final_speed_mps, execution_id=execution_id)

    def accept_payload(self, *args, **kwargs):
        return self._quadruped.accept_payload(*args, **kwargs)

    def read_state(self, *args, **kwargs):
        return self._quadruped.read_state(*args, **kwargs)

    def runtime_inventory(self):
        """联合世界的运行时清单：机械臂安全/操纵事实 + 资源表 + 四足可读状态。

        远端 runtime 会拿它做 Policy 前置条件与 Health.inventory；缺它时
        Provider 直接被判 IRAF-PROVIDER-INVENTORY-UNAVAILABLE（2026-11-18 实测）。
        """
        inventory = dict(next(iter(self._arms.values())).runtime_inventory())
        inventory["mode"] = "simulation"
        inventory["joint"] = {
            "resources": dict(self._resources),
            "capabilities": sorted(self.IMPLEMENTED_CAPABILITIES),
            "quadruped_state": callable(getattr(self._quadruped, "read_state", None)),
        }
        return inventory

    # ---- 显示/渲染（viewer 依赖；联合世界用四足/机械臂的同一把植物锁） ----
    def display_lock(self):
        arm = next(iter(self._arms.values()))
        if callable(getattr(arm, "display_lock", None)):
            return arm.display_lock()
        return self.plant.lock()

    def render_frames(self, *args, **kwargs):
        for backend in [self._quadruped, *self._arms.values()]:
            fn = getattr(backend, "render_frames", None)
            if callable(fn):
                return fn(*args, **kwargs)
        raise JointBindingError("组合后端没有可用的 render_frames")

    # ---- 通用 ----
    def stop(self, lease, execution_id=None):
        try:
            kind, _ = self._kind(lease)
        except JointBindingError:
            kind = "arm"
        target = self._quadruped if kind == "quadruped" else self._arm_for(lease)
        return target.stop(lease, execution_id=execution_id)

    def emergency_stop(self, reason):
        reports = []
        for backend in [self._quadruped, *self._arms.values()]:
            try:
                reports.append(backend.emergency_stop(reason))
            except Exception as exc:  # noqa: BLE001 - 急停必须尝试两侧，失败要可见
                reports.append({"error": "%s: %s" % (type(exc).__name__, exc)})
        return {"simulation": True, "capability": "emergency_stop",
                "reports": reports}
