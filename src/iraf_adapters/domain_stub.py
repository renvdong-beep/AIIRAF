"""控制域 Provider 桩（双域容器化验证用）—— 2026-10-08。

用途：在 openEuler 24.03 容器里把"控制域"替身化，配合智能域容器验证
`TaskFlow → Skill Runtime → Policy Gateway →（跨域 gRPC）→ Capability Provider` 这条链
与四条失败路径（正常 / 策略拒绝 / 取消或超时 / 不可达）。

设计约束（照抄参考实现与框架契约，**不新造接口**）
- 装配契约：`src/iraf_adapters/factory.py: verify_backend_contract()` 要求
  ① `from_config(config, profile, authority, **kwargs)` 存在；
  ② `profile.capabilities` 里每个能力都有同名可调用方法（`CAPABILITY_METHODS` 为同名映射）。
- 方法签名：照抄 `src/iraf_adapters/unitree/unitree_go2.py: UnitreeGo2Adapter`
  （Go2 profile 的五个能力：stand / stop / locomote / dock_for_handoff / accept_payload）。
- **报告形状必须满足 skill 层输出 schema**（第二轮补的关键事实）：
  能力方法返回的不是"随便一个字典"，而是被 `iraf_skills` 的 Provider 用
  `_evidence(report, EVIDENCE_KEYS)` 抽取、再交给 `Draft202012Validator` 校验的**适配器报告**。
  因此本桩按 `skills/{stand,locomote,dock_for_handoff,accept_payload}/*.output.json` 的
  evidence 键逐键返回（缺键会被 Provider 显式拒绝：`适配器报告缺少输出必需键`）。
- 调用方向：是 **skill 调 backend**（`SkillRuntime` → `skill.invoke(profile, backend, parameters, lease)`）；
  runtime 另会 `hasattr` 探测 `runtime_inventory()` 并在取消/停机路径调 `stop(lease)`。
- **本桩不做任何物理计算、不驱动任何执行器**：只返回标记为 simulation 的"事实"，
  并支持注入 `timeout` / `unreachable` 两种故障，供失败路径验收。
- 缺省惰性：只有把 `IRAF_BACKEND_ENTRYPOINT` 指到本类时才生效；不指向 ⇒ 现有链路零影响。

声明面（桩自己就是它的声明来源，全部可用 config 覆盖；不存在隐式魔法值）
    桩是真机的**测试替身**，所以"机型声明"在它这里就是 config 里的键；每个量都记在
    `STUB_DECLARED_*` 常量里并可被 config 同名键覆盖，缺省值来源写在各常量注释中。

配置（`IRAF_BACKEND_CONFIG` 的 JSON）：
    {"fault": "none"|"timeout"|"unreachable", "latency_ms": 0, "facts": {...自定义事实...},
     "trace_file": "/path/to/stub-trace.jsonl",
     "stand_duration_ms": 2000, "substeps_per_control": 4,
     "base_position_m": [0.0, 0.0, 0.279953602548388],
     "dock_acceptance": {"position_tolerance_m": 0.03, "yaw_tolerance_rad": 0.034,
                         "max_final_speed_mps": 0.05}}

`trace_file` 缺省不设置 ⇒ 不落盘、零开销（缺省惰性）。设置后每次能力调用、故障注入与
`stop`（安全停机）都追加一行 JSON（审计账），供跨域验收证明"显式失败后真的调了 backend.stop"。
"""

from __future__ import annotations

import json
import time
import uuid
from typing import Any, Dict, Optional

CONTRACT = "iraf.backend-contract/v1"
BACKEND_NAME = "control_domain_stub"
_ALLOWED_FAULTS = ("none", "timeout", "unreachable")

#: 桩的声明面（真机对应值在各机型声明里；这里是**测试替身自己的声明**，可被 config 覆盖）。
#: 站立时长：真机来自机型基线（`config/go2_loopback.yaml: stand.duration_s`）；
#: 这里取 2000 ms —— 它只决定"桩推进多少控制周期"，不产生任何物理效果。
STUB_DECLARED_DURATION_MS = 2000.0
#: 每个控制周期内的物理子步数（真机来自 MuJoCo 集成步长；桩只用于让报告自洽）。
STUB_DECLARED_SUBSTEPS_PER_CONTROL = 4
#: 站立时的躯干参考高度（m）：取自 Go2 关键帧 home 的实测躯干高度
#: （`build/acceptance/.../height_mean_m = 0.279953602548388`）。桩只是**复述该实测值**，
#: 不是自己算出来的新数字。
STUB_DECLARED_BASE_POSITION_M = (0.0, 0.0, 0.279953602548388)
#: 停靠验收判据缺省（真机来自 `config/go2_joint.yaml: dock_for_handoff.stations.*`）。
STUB_DECLARED_DOCK_ACCEPTANCE = {"position_tolerance_m": 0.03, "yaw_tolerance_rad": 0.034,
                                "max_final_speed_mps": 0.05}

_NOTE = "控制域替身（容器化验证用）：非真机、非实时，不构成板卡证据"


class BackendUnavailable(RuntimeError):
    """控制域不可达（对应验收例④）。"""


class BackendTimeout(RuntimeError):
    """控制域超时（对应验收例③；必须显式失败，不得重试到成功）。"""


class ControlDomainStubBackend:
    """控制域替身：返回可复现事实 + 可注入故障（不驱动任何执行器）。"""

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
        trace_file = cfg.get("trace_file")
        self.trace_path = str(trace_file) if trace_file else None
        self.profile = profile
        self.authority = authority
        self.stand_duration_ms = float(cfg.get("stand_duration_ms", STUB_DECLARED_DURATION_MS))
        self.substeps_per_control = int(cfg.get("substeps_per_control", STUB_DECLARED_SUBSTEPS_PER_CONTROL))
        base = cfg.get("base_position_m", STUB_DECLARED_BASE_POSITION_M)
        self.base_position_m = [float(item) for item in base]
        self.dock_acceptance_values = dict(STUB_DECLARED_DOCK_ACCEPTANCE)
        self.dock_acceptance_values.update(dict(cfg.get("dock_acceptance") or {}))
        #: 审计账：每次能力调用的能力名/执行 id/墙钟（证据，供报告引用）
        self.calls: list = []
        #: 安全停机账：runtime 在取消/停机路径调 `stop(lease)` 时记一笔（跨域验收要证"真的停了"）
        self.stops: list = []
        declared = getattr(profile, "capabilities", None) or []
        self.declared_capabilities = tuple(str(item) for item in declared)

    # ---- 契约要求的装配入口（factory 会校验其存在）----
    @classmethod
    def from_config(cls, config: Optional[Dict[str, Any]], profile: Any, authority: Any,
                    plant: Any = None, **kwargs: Any) -> "ControlDomainStubBackend":
        """装配：与参考实现同签名（`plant=` 在桩里忽略 —— 桩不持有植物）。"""
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
            self._trace({"event": "fault", "kind": "unreachable"})
            raise BackendUnavailable("控制域不可达（注入故障 fault=unreachable）")
        if self.fault == "timeout":
            self._trace({"event": "fault", "kind": "timeout"})
            raise BackendTimeout("控制域超时（注入故障 fault=timeout）")
        if self.latency_ms:
            time.sleep(self.latency_ms / 1000.0)

    def _trace(self, record: Dict[str, Any]) -> None:
        """审计账：只有在 config 显式给了 `trace_file` 时才落盘（缺省零开销）。"""
        if not self.trace_path:
            return
        payload = {"backend": BACKEND_NAME, "wall_s": round(time.time(), 6)}
        payload.update(record)
        with open(self.trace_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, ensure_ascii=False) + "\n")

    @staticmethod
    def _fencing_token(lease: Any) -> int:
        """租约的 fencescing token（真机由控制器持久化；桩只复述 runtime 发下的值）。"""
        token = getattr(lease, "fencing_token", None)
        return int(token) if isinstance(token, int) and token >= 1 else 1

    def _seal(self, capability: str, execution_id: Any, lease: Any = None,
              observed: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """统一出口：审计 + 返回**标注为仿真替身**的事实（含时间戳）。

        保留 `simulation` 与 `observed` 两个顶层键（`scripts/probe_domain_stub_contract.py`
        与 `runtime_inventory` 的既有判据依赖它们），其余键由各能力按输出 schema 补齐。
        """
        now = time.time()
        resolved_id = str(execution_id or "") or str(uuid.uuid4())
        self.calls.append({"capability": capability, "execution_id": resolved_id, "wall_s": round(now, 6)})
        self._trace({"event": "capability", "capability": capability, "execution_id": resolved_id})
        report: Dict[str, Any] = {
            "capability": capability, "backend": BACKEND_NAME, "contract": CONTRACT,
            "simulation": True, "timestamp_s": round(now, 6), "note": _NOTE,
            "execution_id": resolved_id, "fencing_token": self._fencing_token(lease),
            "observed": dict(observed or {}),
        }
        report.update(self.extra_facts)
        return report

    def _joint_targets(self, targets: Any) -> Dict[str, float]:
        """目标位形：显式给出的优先，缺省回落到 Profile 的 `spec.home`（唯一声明来源）。"""
        source = targets if isinstance(targets, dict) and targets else (getattr(self.profile, "home", None) or {})
        return {str(key): float(value) for key, value in source.items()}

    def _final_state(self, joints: Dict[str, float]) -> Dict[str, Any]:
        now = time.time()
        return {"simulation": True, "time_s": round(now, 6),
                "base_position_m": list(self.base_position_m),
                "base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "joint_positions_rad": dict(joints)}

    def _cycles(self, duration_ms: float) -> int:
        frequency = float(getattr(self.profile, "control_frequency_hz", 0) or 0)
        if frequency <= 0:
            raise ValueError("Profile 未声明 control_frequency_hz，无法把时长折算为控制周期数")
        return max(1, int(round(float(duration_ms) / (1000.0 / frequency))))

    # ---- 五个能力（签名逐字照抄 UnitreeGo2Adapter；报告形状对齐 skill 输出 schema）----
    def stand(self, lease: Any, targets: Any = None, duration_ms: Any = None,
              execution_id: Any = None) -> Dict[str, Any]:
        """`stand`：报告键 = `skills/stand/stand.output.json` 的 13 个 evidence 键。"""
        self.require_motion_allowed("stand", lease)
        self._maybe_fault()
        duration = float(duration_ms if duration_ms is not None else self.stand_duration_ms)
        if duration <= 0:
            raise ValueError("stand 的 duration_ms 必须为正（实际 %r）" % (duration,))
        joints_by_target = self._joint_targets(targets)
        report = self._seal("stand", execution_id, lease, {"duration_ms": duration})
        report.update({
            "duration_ms": duration,
            "control_cycles": self._cycles(duration),
            "substeps_per_control": max(1, int(self.substeps_per_control)),
            "ctrl_saturated_samples": 0,
            "target_source": "explicit" if (isinstance(targets, dict) and targets) else "profile_home",
            "torque_limit_source": "model",
            "gravity_feedforward": False,
            "joint_targets_rad": joints_by_target,
            "final_state": self._final_state(joints_by_target),
        })
        return report

    def stop(self, lease: Any, execution_id: Any = None, *, mode: Any = None,
             tilt_limit_deg: Any = None) -> Dict[str, Any]:
        """安全停机。`skills/stop` 的输出只要求 `{skill, accepted}`，故报告不参与 schema。

        但**必须记账**：这是验收例③④"显式失败后确实执行了安全停机"的唯一证据来源。
        """
        self.require_motion_allowed("stop", lease)
        now = time.time()
        record = {"capability": "stop", "execution_id": str(execution_id or ""),
                  "mode": mode, "tilt_limit_deg": tilt_limit_deg, "wall_s": round(now, 6)}
        self.stops.append(record)
        self._trace({"event": "safe_stop", **record})
        self._maybe_fault()
        return self._seal("stop", execution_id, lease,
                          {"mode": mode, "tilt_limit_deg": tilt_limit_deg, "final_speed_mps": 0.0})

    def resolve_velocity(self, velocity: Any) -> Dict[str, float]:
        """速度指令规范化（对应参考实现同名方法）：只做字段校验，不做物理裁剪。"""
        if not isinstance(velocity, dict):
            raise ValueError("velocity 必须是对象（vx_mps/vy_mps/wz_rad_s），实际 %r" % (type(velocity).__name__,))
        missing = [key for key in ("vx_mps", "vy_mps", "wz_rad_s") if key not in velocity]
        if missing:
            raise ValueError("velocity 缺少字段：%s" % missing)
        return {key: float(velocity[key]) for key in ("vx_mps", "vy_mps", "wz_rad_s")}

    def locomote(self, velocity: Any, duration_ms: Any, lease: Any, execution_id: Any = None,
                 command_provider: Any = None, hold_pose_world: Any = None) -> Dict[str, Any]:
        """`locomote`：报告键 = `skills/locomote/locomote.output.json` 的 evidence 键。"""
        self.require_motion_allowed("locomote", lease)
        self._maybe_fault()
        duration = float(duration_ms)
        if duration <= 0:
            raise ValueError("locomote 的 duration_ms 必须为正（实际 %r）" % (duration,))
        report = self._seal("locomote", execution_id, lease,
                            {"velocity": velocity, "duration_ms": duration,
                             "hold_pose_world": hold_pose_world})
        report.update({"duration_ms": duration, "command": {"velocity": velocity},
                       "hold_pose_world": hold_pose_world})
        return report

    def dock_acceptance(self) -> Dict[str, Any]:
        """停靠验收判据（真机来自机型声明 `dock_for_handoff.stations.*`；桩从 config 复述）。"""
        return dict(self.dock_acceptance_values)

    def dock_for_handoff(self, *, lease: Any, position_tolerance_m: Any, yaw_tolerance_rad: Any,
                         max_final_speed_mps: Any, station: Any = None,
                         execution_id: Any = None) -> Dict[str, Any]:
        """`dock_for_handoff`：报告键 = 该 skill 输出 schema 的 8 个 evidence 键。

        桩返回"刚好贴住容差"的可复现事实（0 误差），用于跨域链路的判据判定；
        **不代表真机精度**，也不构成本体已实现停靠的证据。
        """
        self.require_motion_allowed("dock_for_handoff", lease)
        self._maybe_fault()
        if station is None:
            raise ValueError("dock_for_handoff 需要站位名 station（桩不允许隐式默认站位）")
        report = self._seal("dock_for_handoff", execution_id, lease,
                            {"station": station, "position_tolerance_m": float(position_tolerance_m),
                             "yaw_tolerance_rad": float(yaw_tolerance_rad),
                             "max_final_speed_mps": float(max_final_speed_mps),
                             "translation_error_m": 0.0, "yaw_error_rad": 0.0, "final_speed_mps": 0.0})
        report.update({
            "target_frame": str(station),
            "target_frame_world_fixed": True,
            "settled_at_s": 0.0,
            "final_translation_error_m": 0.0,
            "final_yaw_error_deg": 0.0,
            "final_speed_mps": 0.0,
        })
        return report

    def accept_payload(self, payload_id: Any, place_target_id: Any, lease: Any) -> Dict[str, Any]:
        """`accept_payload`：报告键 = 该 skill 输出 schema 的 12 个 evidence 键。

        `payload_on_target=True` 是 Provider 的硬门禁（为假即抛 SkillRejected），
        `confirmation` 只允许字面量 `payload_confirmed`。
        """
        self.require_motion_allowed("accept_payload", lease)
        self._maybe_fault()
        if not str(payload_id or "") or not str(place_target_id or ""):
            raise ValueError("accept_payload 需要 payload_id 与 place_target_id（桩不做隐式默认）")
        report = self._seal("accept_payload", None, lease,
                            {"payload_id": payload_id, "place_target_id": place_target_id})
        report.update({
            "payload_on_target": True,
            "contact_margin_m": 0.0,
            "payload_low_z_m": 0.0,
            "target_top_z_m": 0.0,
            "resting_gap_m": 0.0,
            "contact_geoms": [],
            "payload_center_m": [0.0, 0.0, 0.0],
            "target_center_m": [0.0, 0.0, 0.0],
            "offset_from_target_center_m": 0.0,
            "last_speed_mps": 0.0,
            "runtime_source": BACKEND_NAME,
            "phase_trace": [],
            "confirmation": "payload_confirmed",
        })
        return report

    # ---- 供 runtime 探测（可选接口）----
    def runtime_inventory(self) -> Dict[str, Any]:
        return {"backend": BACKEND_NAME, "contract": CONTRACT,
                "declared_capabilities": list(self.declared_capabilities),
                "implemented_capabilities": ["stand", "stop", "locomote",
                                             "dock_for_handoff", "accept_payload"],
                "fault": self.fault, "calls": len(self.calls),
                "safe_stops": len(self.stops), "simulation": True}
