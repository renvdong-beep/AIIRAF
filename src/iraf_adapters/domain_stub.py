"""控制域 Provider 桩（双域容器化验证用）—— 2026-10-08。

用途：在 openEuler 24.03 容器里把"控制域"替身化，配合智能域容器验证
`TaskFlow → Skill Runtime → Policy Gateway →（跨域 gRPC）→ Capability Provider` 这条链与四条失败路径。

设计约束（照抄参考实现与框架契约，**不新造接口**）
- 契约：`src/iraf_adapters/factory.py: verify_backend_contract()` 要求
  ① `from_config(config, profile, authority, **kwargs)` 存在；
  ② `profile.capabilities` 里每个能力都有同名可调用方法（`CAPABILITY_METHODS` 为同名映射）。
- 方法签名：照抄 `src/iraf_adapters/unitree/unitree_go2.py: UnitreeGo2Adapter`
  （Go2 profile 的五个能力：stand / stop / locomote / dock_for_handoff / accept_payload）。
- 调用方向：是 **skill 调 backend**（`SkillRuntime` → `skill.invoke(profile, backend, parameters, lease)`）；
  runtime 另会 `hasattr` 探测 `runtime_inventory()` 并在需要时调 `stop(lease)`。
- **本桩不做任何物理计算、不驱动任何执行器**：只返回标记为 simulation 的"事实"，
  并支持注入 `timeout` / `unreachable` 两种故障，供失败路径验收。
- 缺省惰性：只有把 `IRAF_BACKEND_ENTRYPOINT` 指到本类时才生效；不指向 ⇒ 现有链路零影响。

配置（`IRAF_BACKEND_CONFIG` 的 JSON）：
    {"fault": "none"|"timeout"|"unreachable", "latency_ms": 0, "facts": {...自定义事实...}}
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

CONTRACT = "iraf.backend-contract/v1"
BACKEND_NAME = "control_domain_stub"
_ALLOWED_FAULTS = ("none", "timeout", "unreachable")


class BackendUnavailable(RuntimeError):
    """控制域不可达（对应验收例④）。"""


class BackendTimeout(RuntimeError):
    """控制域超时（对应验收例③；必须显式失败，不得重试到成功）。"""


class ControlDomainStubBackend:
    """控制域替身：返回可复现事实 + 可注入故障（不驱动执行器）。"""

    def __init__(self, config: Optional[Dict[str, Any]], profile: Any, authority: Any) -> None:
        cfg = dict(config or {})
        fault = str(cfg.get("fault") or "none")
        if fault not in _ALLOWED_FAULTS:
            raise ValueError("IRAF_BACKEND_CONFIG.fault 只允许 %s（实际 %r）" % (list(_ALLOWED_FAULTS), fault))
        latency_ms = cfg.get("latency_ms", 0)
        if not isinstance(latency_ms, (int, float)) or latency_ms < 0:
            raise ValueError("latency_ms 必须是非负数（实际 %r）" % (latency_ms,))
        self.fault = fault
        self.latency_ms = float(latency_ms)
        self.extra_facts = dict(cfg.get("facts") or {})
        self.profile = profile
        self.authority = authority
        #: 审计账：每次能力调用的能力名/执行 id/墙钟（证据，供报告引用）
        self.calls: list = []
        declared = getattr(profile, "capabilities", None) or []
        self.declared_capabilities = tuple(str(item) for item in declared)

    # ---- 契约要求的装配入口（factory 会校验其存在）----
    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]], profile: Any, authority: Any,
                    plant: Any = None, **kwargs: Any) -> "ControlDomainStubBackend":
        """装配：与参考实现同签名（`plant=` 在桩里忽略——桩不持有植物）。"""
        return cls(config=config, profile=profile, authority=authority)

    # ---- 内部：动作放行（等价于参考实现的 require_motion_allowed，但桩不产生物理动作）----
    def require_motion_allowed(self, capability: str, lease: Any) -> None:
        """fail-closed：能力必须在 Profile 里已声明，且必须带租约（没有租约即拒绝）。"""
        if capability not in self.declared_capabilities:
            raise PermissionError("能力未在 Profile 声明：%s（已声明：%s）"
                                 % (capability, list(self.declared_capabilities)))
        if lease is None:
            raise PermissionError("缺少资源租约（lease）：控制域拒绝无租约的动作请求")

    def _maybe_fault(self) -> None:
        if self.fault == "unreachable":
            raise BackendUnavailable("控制域不可达（注入故障 fault=unreachable）")
        if self.fault == "timeout":
            raise BackendTimeout("控制域超时（注入故障 fault=timeout）")
        if self.latency_ms:
            time.sleep(self.latency_ms / 1000.0)

    def _fact(self, capability: str, observed: Dict[str, Any], execution_id: Any = None) -> Dict[str, Any]:
        """统一出口：记录审计 + 返回**标注为仿真替身**的事实（含时间戳）。"""
        now = time.time()
        self.calls.append({"capability": capability, "execution_id": execution_id, "wall_s": round(now, 6)})
        fact = {"capability": capability, "backend": BACKEND_NAME, "contract": CONTRACT,
                "simulation": True, "timestamp_s": round(now, 6),
                "note": "控制域替身（容器化验证用）：非真机、非实时，不构成板卡证据"}
        fact.update(self.extra_facts)
        fact["observed"] = observed
        return fact

    # ---- 五个能力（签名逐字照抄 UnitreeGo2Adapter）----
    def stand(self, lease: Any, targets: Any = None, duration_ms: Any = None,
              execution_id: Any = None) -> Dict[str, Any]:
        self.require_motion_allowed("stand", lease)
        self._maybe_fault()
        return self._fact("stand", {"targets": targets, "duration_ms": duration_ms,
                                    "final_speed_mps": 0.0}, execution_id)

    def stop(self, lease: Any, execution_id: Any = None, *, mode: Any = None,
             tilt_limit_deg: Any = None) -> Dict[str, Any]:
        self.require_motion_allowed("stop", lease)
        self._maybe_fault()
        return self._fact("stop", {"mode": mode, "tilt_limit_deg": tilt_limit_deg,
                                   "final_speed_mps": 0.0}, execution_id)

    def locomote(self, velocity: Any, duration_ms: Any, lease: Any, execution_id: Any = None,
                 command_provider: Any = None, hold_pose_world: Any = None) -> Dict[str, Any]:
        self.require_motion_allowed("locomote", lease)
        self._maybe_fault()
        return self._fact("locomote", {"velocity": velocity, "duration_ms": duration_ms,
                                       "hold_pose_world": hold_pose_world}, execution_id)

    def dock_for_handoff(self, *, lease: Any, position_tolerance_m: Any, yaw_tolerance_rad: Any,
                         max_final_speed_mps: Any, station: Any = None,
                         execution_id: Any = None) -> Dict[str, Any]:
        self.require_motion_allowed("dock_for_handoff", lease)
        self._maybe_fault()
        # 桩返回"贴合容差"的可复现事实（用于跨域链路的判据判定），不代表真机精度
        return self._fact("dock_for_handoff", {
            "station": station,
            "position_tolerance_m": position_tolerance_m,
            "yaw_tolerance_rad": yaw_tolerance_rad,
            "max_final_speed_mps": max_final_speed_mps,
            "translation_error_m": 0.0,
            "yaw_error_rad": 0.0,
            "final_speed_mps": 0.0,
        }, execution_id)

    def accept_payload(self, payload_id: Any, place_target_id: Any, lease: Any) -> Dict[str, Any]:
        self.require_motion_allowed("accept_payload", lease)
        self._maybe_fault()
        return self._fact("accept_payload", {"payload_id": payload_id,
                                             "place_target_id": place_target_id,
                                             "payload_on_target": True})

    # ---- 供 runtime 探测（可选接口）----
    def runtime_inventory(self) -> Dict[str, Any]:
        return {"backend": BACKEND_NAME, "contract": CONTRACT,
                "declared_capabilities": list(self.declared_capabilities),
                "implemented_capabilities": ["stand", "stop", "locomote",
                                             "dock_for_handoff", "accept_payload"],
                "fault": self.fault, "calls": len(self.calls),
                "simulation": True}
