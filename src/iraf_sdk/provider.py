# -*- coding: utf-8 -*-
"""南向 Provider SDK：让第三方用自己的 MuJoCo/控制器实现 Capability Provider。

定位（与北向 `iraf_sdk.client` / `iraf_sdk.mcp` 分开）：
  · 北向：harness → 边缘 IRAF（任务级）。
  · 南向：边缘 IRAF → 本 Provider（能力级，`provider.proto`）。
本模块只做**契约执行与安全不变量**，不做策略判定、不导入 `iraf_core`：
  · 鉴权：Bearer token（由 transport 传入，不信任请求体）；
  · lease：缺 deadline / 已过期 / fencing token 回退 / 终态重发 一律拒绝；
  · 幂等：同 execution_id + idempotency_key 返回既有结果，不重复执行；
  · 能力：声明 ⊆ 实现，未声明/未实现/未知能力显式失败；
  · 输出：必须是 dict，且 `simulation=true` 由部署声明固定（不得伪造真机能力）。

依赖：`grpcio`/`protobuf`/生成的 `iraf.v1.*` 桩只在构造 gRPC 服务时惰性导入；
纯逻辑测试用系统 Python 即可跑。

最小实现（参考 `docs/provider-sdk-guide.md`）：

    class MyBackend:
        capabilities = {"move_joint", "pick_object"}
        def runtime_inventory(self): return {"mode": "simulation", "safety": {"estop": False}}
        def move_joint(self, positions, duration_ms, lease): ...
        def pick_object(self, target_id, grasp_pose, duration_ms, lease): ...

    kit = ProviderKit(backend=MyBackend(), token=token, provider_id="acme/mujoco")
    kit.serve(bind="0.0.0.0:50052")
"""
from __future__ import annotations

import json
import sqlite3
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional

__all__ = [
    "CAPABILITY_METHODS",
    "MOTION_CAPABILITIES",
    "Lease",
    "LeaseRejected",
    "ProviderKit",
    "PROVIDER_SDK_VERSION",
]

PROVIDER_SDK_VERSION = "0.1.0"

#: 能力词表（必须与边缘 `src/iraf_adapters/factory.py:CAPABILITY_METHODS` 一致）。
CAPABILITY_METHODS = {
    "move_joint": "move_joint",
    "pick_object": "pick_object",
    "visual_pick": "visual_pick",
    "calibrate_grasp": "calibrate_grasp",
    "calibrate_camera_to_base": "calibrate_camera_to_base",
    "stop": "stop",
    "step": "step",
    "stand": "stand",
    "locomote": "locomote",
    "dock_for_handoff": "dock_for_handoff",
    "accept_payload": "accept_payload",
    "place_object": "place_object",
    "reset_scene": "reset_scene",
    "read_state": "read_state",
    "emergency_stop": "emergency_stop",
}

#: 产生物理动作的能力（其余如 read_state/accept_payload/step 不算）。
MOTION_CAPABILITIES = frozenset({
    "move_joint", "pick_object", "place_object", "visual_pick",
    "calibrate_grasp", "calibrate_camera_to_base", "stand", "locomote",
    "dock_for_handoff", "reset_scene", "emergency_stop",
})

_TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELLED", "QUARANTINED"})


class LeaseRejected(RuntimeError):
    """Provider 拒绝一次能力调用（鉴权/租约/幂等/能力契约）。"""

    def __init__(self, code: str, reason: str, *, retryable: bool = False):
        self.code = code
        self.reason = reason
        self.retryable = bool(retryable)
        super().__init__("%s: %s" % (code, reason))


@dataclass(frozen=True)
class Lease:
    """`provider.proto:LeaseEnvelope` 的本地视图（已校验 deadline/fencing）。"""

    execution_id: str
    idempotency_key: str
    correlation_id: str = ""
    resource_id: str = ""
    fencing_token: int = 0
    deadline_unix_ms: int = 0
    caller_subject: str = ""
    controller: str = ""

    def expired(self, now_ms: Optional[float] = None) -> bool:
        if not self.deadline_unix_ms:
            return True
        moment = time.time() * 1000 if now_ms is None else float(now_ms)
        return moment >= self.deadline_unix_ms


class _ExecutionRegistry:
    """进程内幂等 + fencing 单调 + 终态拒绝（可用 SQLite 持久化重放结果）。"""

    def __init__(self, store_path: Optional[str] = None, ttl_s: float = 3600.0):
        self._lock = threading.RLock()
        self._last_token: Dict[str, int] = {}
        self._results: Dict[tuple, dict] = {}
        self._terminal: Dict[str, str] = {}
        self._ttl_s = float(ttl_s)
        self._db = None
        if store_path:
            self._db = sqlite3.connect(store_path, check_same_thread=False, timeout=30)
            self._db.execute(
                "CREATE TABLE IF NOT EXISTS provider_idempotency("
                "execution_id TEXT NOT NULL,idempotency_key TEXT NOT NULL,"
                "result_json TEXT NOT NULL,updated_at_ms INTEGER NOT NULL,"
                "PRIMARY KEY(execution_id,idempotency_key))")

    def lookup(self, lease: Lease) -> Optional[dict]:
        key = (lease.execution_id, lease.idempotency_key)
        if self._db is not None:
            row = self._db.execute(
                "SELECT result_json FROM provider_idempotency "
                "WHERE execution_id=? AND idempotency_key=?", key).fetchone()
            if row:
                return json.loads(row[0])
        with self._lock:
            return self._results.get(key)

    def remember(self, lease: Lease, result: dict) -> None:
        key = (lease.execution_id, lease.idempotency_key)
        payload = json.dumps(result, ensure_ascii=False, sort_keys=True)
        if self._db is not None:
            self._db.execute(
                "INSERT OR REPLACE INTO provider_idempotency VALUES(?,?,?,?)",
                (key[0], key[1], payload, int(time.time() * 1000)))
            self._db.commit()
        with self._lock:
            self._results[key] = result
            if result.get("status") in _TERMINAL_STATES:
                self._terminal[lease.execution_id] = result["status"]

    def terminal_state(self, execution_id: str) -> str:
        with self._lock:
            return self._terminal.get(execution_id, "")

    def mark_terminal(self, execution_id: str, status: str = "CANCELLED") -> None:
        with self._lock:
            self._terminal[execution_id] = status

    def check_fencing(self, resource_id: str, token: int) -> None:
        resource = resource_id or "default"
        with self._lock:
            seen = self._last_token.get(resource, 0)
            if token and token < seen:
                raise LeaseRejected("IRAF-EXECUTION-CONFLICT",
                                    "fencing token 回退：收到 %d < 已见 %d" % (token, seen))
            if token:
                self._last_token[resource] = max(seen, token)

    def current_fencing(self, resource_id: str) -> int:
        with self._lock:
            return int(self._last_token.get(resource_id or "default", 0))


class ProviderKit:
    """把任意后端对象适配成 `CapabilityProviderService`。

    后端契约（duck typing）：
      · `capabilities`：声明能力集合（必填；未声明但实现的能力不会暴露）
      · `runtime_inventory() -> dict`（建议；Health.inventory 与安全前置条件依赖它）
      · 能力方法：与能力同名，接受 `inputs` 的关键字参数，并接受 `lease`（若签名有）
      · 可选 `display_mode` / `window_opened` / `describe()` 供 Health 使用
    """

    def __init__(self, backend, *, token: str, provider_id: str,
                 subject: str = "iraf-runtime", resource_id: str = "",
                 declared_capabilities: Optional[Iterable[str]] = None,
                 simulation: bool = True, ttl_seconds: float = 120.0,
                 idempotency_db: Optional[str] = None):
        if not token:
            raise ValueError("token 必填（缺 token 直接拒绝启动，fail-closed）")
        if not provider_id:
            raise ValueError("provider_id 必填")
        self.backend = backend
        self._token = str(token)
        self.provider_id = str(provider_id)
        self.subject = str(subject)
        self.resource_id = str(resource_id or getattr(backend, "resource_id", "") or "robot")
        self.simulation = bool(simulation)
        self._declared = set(declared_capabilities or getattr(backend, "capabilities", ()) or ())
        self._registry = _ExecutionRegistry(idempotency_db, ttl_s=ttl_seconds)
        self._ttl_seconds = float(ttl_seconds)
        self._implemented = {
            name for name, method in CAPABILITY_METHODS.items()
            if callable(getattr(backend, method, None))
        }
        self._validate_contract()

    # ---------- 契约 ----------
    def _validate_contract(self) -> None:
        unknown = sorted(self._declared - set(CAPABILITY_METHODS))
        if unknown:
            raise ValueError("声明了未登记的能力：%s（先改 provider.proto 契约）" % unknown)
        missing = sorted(self._declared - self._implemented)
        if missing:
            raise ValueError("声明了但后端未实现：%s（拒绝启动，禁止伪造成功）" % missing)

    def capabilities(self) -> list:
        return sorted(self._declared)

    def health(self) -> dict:
        inventory = {}
        getter = getattr(self.backend, "runtime_inventory", None)
        if callable(getter):
            try:
                inventory = getter() or {}
            except Exception as exc:  # noqa: BLE001 - 读不到清单按不可用报，不伪造
                inventory = {"error": "%s: %s" % (type(exc).__name__, exc)}
        display = getattr(self.backend, "display_mode", "") or (
            "offscreen_frames" if hasattr(self.backend, "render_frames") else "unavailable")
        return {
            "ready": True,
            "simulation": self.simulation,
            "backend": "%s:%s" % (type(self.backend).__module__, type(self.backend).__name__),
            "capabilities": self.capabilities(),
            "provider": self.provider_id,
            "fencing_token": self._registry.current_fencing(self.resource_id),
            "display_mode": display,
            "window_opened": bool(getattr(self.backend, "window_opened", False)),
            "detail": str(getattr(self.backend, "detail", "") or ""),
            "inventory": inventory,
        }

    # ---------- 鉴权 ----------
    def authenticate(self, authorization: str) -> None:
        provided = (authorization or "").strip()
        if provided.lower().startswith("bearer "):
            provided = provided[7:].strip()
        if not self._token or provided != self._token:
            raise LeaseRejected("IRAF-UNAUTHENTICATED", "跨机 Provider 调用鉴权失败")

    # ---------- 调用 ----------
    def invoke(self, capability: str, inputs: Mapping, lease: Mapping,
               *, now_ms: Optional[float] = None) -> dict:
        """执行一次能力调用；返回可序列化结果（失败也返回结构化结果，不抛伪成功）。"""
        try:
            lease_obj = self._validate_lease(lease, now_ms=now_ms)
            result = self._invoke(capability, dict(inputs or {}), lease_obj)
            return result
        except LeaseRejected as exc:
            return {
                "capability": capability,
                "accepted": False,
                "outputs": {},
                "error_code": exc.code,
                "error_message": exc.reason,
                "retryable": exc.retryable,
                "fencing_token": self._registry.current_fencing(self.resource_id),
                "provider": self.provider_id,
                "simulation": self.simulation,
                "status": "FAILED",
            }

    def _validate_lease(self, lease: Mapping, *, now_ms: Optional[float]) -> Lease:
        data = dict(lease or {})
        execution_id = str(data.get("execution_id") or "").strip()
        idempotency_key = str(data.get("idempotency_key") or "").strip()
        if not execution_id:
            raise LeaseRejected("IRAF-INPUT-INVALID", "缺 execution_id（服务端必须生成）")
        if not idempotency_key:
            raise LeaseRejected("IRAF-INPUT-INVALID", "缺 idempotency_key（禁止无幂等调用）")
        deadline = int(data.get("deadline_unix_ms") or 0)
        if not deadline:
            raise LeaseRejected("IRAF-INPUT-INVALID", "缺 deadline（禁止无超时调用）")
        obj = Lease(
            execution_id=execution_id,
            idempotency_key=idempotency_key,
            correlation_id=str(data.get("correlation_id") or ""),
            resource_id=str(data.get("resource_id") or self.resource_id),
            fencing_token=int(data.get("fencing_token") or 0),
            deadline_unix_ms=deadline,
            caller_subject=str(data.get("caller_subject") or ""),
            controller=str(data.get("controller") or ""),
        )
        if obj.expired(now_ms):
            raise LeaseRejected("IRAF-DEADLINE-EXCEEDED", "lease deadline 已过期")
        self._registry.check_fencing(obj.resource_id, obj.fencing_token)
        if self._registry.terminal_state(execution_id) and not self._registry.lookup(obj):
            raise LeaseRejected("IRAF-EXECUTION-CONFLICT",
                                "execution 已终态，拒绝新的控制请求")
        return obj

    def _invoke(self, capability: str, inputs: Dict[str, Any], lease: Lease) -> dict:
        cached = self._registry.lookup(lease)
        if cached is not None:
            cached = dict(cached)
            cached["idempotent_replay"] = True
            return cached
        if capability not in CAPABILITY_METHODS:
            raise LeaseRejected("IRAF-CAPABILITY-UNKNOWN", "未登记的能力：%s" % capability)
        if capability not in self._declared:
            raise LeaseRejected("IRAF-POLICY-DENIED",
                                "能力 %s 未在 Provider 声明中（拒绝越权）" % capability)
        method = getattr(self.backend, CAPABILITY_METHODS[capability], None)
        if not callable(method):
            raise LeaseRejected("IRAF-CAPABILITY-NOT-IMPLEMENTED",
                                "后端未实现 %s" % CAPABILITY_METHODS[capability])
        kwargs = self._kwargs_for(method, inputs, lease)
        try:
            outputs = method(**kwargs)
        except LeaseRejected:
            raise
        except Exception as exc:  # noqa: BLE001 - 后端失败必须变成结构化失败
            raise LeaseRejected("IRAF-PROVIDER-ERROR",
                                "%s: %s" % (type(exc).__name__, exc), retryable=False) from exc
        if outputs is None:
            outputs = {}
        if not isinstance(outputs, Mapping):
            raise LeaseRejected("IRAF-PROVIDER-ERROR",
                                "能力 %s 返回值不是对象（禁止伪造成功）" % capability)
        result = {
            "capability": capability,
            "accepted": True,
            "outputs": dict(outputs),
            "error_code": "",
            "error_message": "",
            "fencing_token": lease.fencing_token or self._registry.current_fencing(lease.resource_id),
            "provider": self.provider_id,
            "simulation": self.simulation,
            "status": "SUCCEEDED",
            "execution_id": lease.execution_id,
        }
        self._registry.remember(lease, result)
        return result

    @staticmethod
    def _kwargs_for(method, inputs: Dict[str, Any], lease: Lease) -> Dict[str, Any]:
        """按后端签名过滤 inputs，并只在签名含 lease 时传入（兼容严格签名）。"""
        import inspect

        try:
            signature = inspect.signature(method)
        except (TypeError, ValueError):
            return dict(inputs)
        params = signature.parameters
        kwargs = {k: v for k, v in inputs.items() if k in params}
        if "lease" in params:
            kwargs["lease"] = lease
        return kwargs

    def cancel(self, execution_id: str, reason: str = "") -> dict:
        if not execution_id:
            return {"accepted": False, "status": "FAILED",
                    "error_code": "IRAF-INPUT-INVALID", "reason": "缺 execution_id"}
        method = getattr(self.backend, "cancel", None)
        if callable(method):
            try:
                method(execution_id=execution_id, reason=reason)
            except TypeError:
                method(execution_id)
        self._registry.mark_terminal(execution_id, "CANCELLED")
        return {"accepted": True, "status": "CANCELLED", "execution_id": execution_id}

    # ---------- gRPC 服务 ----------
    def build_servicer(self, provider_pb2, provider_pb2_grpc, skill_pb2):
        """构造 `CapabilityProviderServiceServicer`（桩由调用方注入，避免核心硬依赖）。"""
        kit = self

        class _Servicer(provider_pb2_grpc.CapabilityProviderServiceServicer):
            def _auth(self, context):
                metadata = dict(context.invocation_metadata() or [])
                try:
                    kit.authenticate(metadata.get("authorization", "")
                                     or metadata.get("x-iraf-token", ""))
                except LeaseRejected as exc:
                    import grpc

                    context.abort(grpc.StatusCode.UNAUTHENTICATED, exc.reason)

            def Health(self, request, context):
                self._auth(context)
                data = kit.health()
                return provider_pb2.ProviderHealthResponse(
                    ready=bool(data["ready"]),
                    simulation=bool(data["simulation"]),
                    backend=str(data["backend"]),
                    capabilities=list(data["capabilities"]),
                    fencing_token=int(data["fencing_token"]),
                    display_mode=str(data["display_mode"]),
                    window_opened=bool(data["window_opened"]),
                    detail=str(data["detail"]),
                    inventory=_struct(provider_pb2, data["inventory"]),
                )

            def Invoke(self, request, context):
                self._auth(context)
                lease = request.lease
                deadline_ms = 0
                if lease.deadline.seconds or lease.deadline.nanos:
                    deadline_ms = int(lease.deadline.seconds) * 1000 + int(lease.deadline.nanos) // 1_000_000
                result = kit.invoke(
                    request.capability,
                    _struct_to_dict(request.inputs),
                    {
                        "execution_id": lease.execution_id,
                        "idempotency_key": lease.idempotency_key,
                        "correlation_id": lease.correlation_id,
                        "resource_id": lease.resource_id,
                        "fencing_token": lease.fencing_token,
                        "deadline_unix_ms": deadline_ms,
                        "caller_subject": lease.caller_subject,
                        "controller": lease.controller,
                    },
                )
                response = provider_pb2.CapabilityResult(
                    capability=result["capability"],
                    accepted=bool(result["accepted"]),
                    outputs=_struct(provider_pb2, result["outputs"]),
                    fencing_token=int(result["fencing_token"]),
                    provider=str(result["provider"]),
                    simulation=bool(result["simulation"]),
                )
                if result.get("error_code"):
                    response.error.code = result["error_code"]
                    response.error.message_zh = result.get("error_message", "")
                    response.error.retryable = bool(result.get("retryable", False))
                return response

            def Cancel(self, request, context):
                self._auth(context)
                data = kit.cancel(request.execution_id, request.reason)
                return provider_pb2.CancelExecutionResponse(
                    accepted=bool(data["accepted"]),
                    state=skill_pb2.SKILL_STATE_CANCELLED if data["accepted"] else skill_pb2.SKILL_STATE_FAILED,
                )

        return _Servicer()

    def serve(self, bind: str = "0.0.0.0:50052", *, max_workers: int = 4) -> None:
        """启动 gRPC 服务（阻塞）。依赖 grpcio + 生成的 `iraf.v1` 桩。"""
        try:
            import grpc
            from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2
        except ImportError as exc:  # pragma: no cover - 依赖缺失时给出可操作提示
            raise RuntimeError(
                "Provider SDK 的 gRPC 服务需要 grpcio 与 iraf.v1 桩（PYTHONPATH 含生成目录）"
            ) from exc
        from concurrent import futures

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
        provider_pb2_grpc.add_CapabilityProviderServiceServicer_to_server(
            self.build_servicer(provider_pb2, provider_pb2_grpc, skill_pb2), server)
        server.add_insecure_port(bind)
        server.start()
        server.wait_for_termination()


def _struct(pb2_module, value: Mapping):
    from google.protobuf import struct_pb2

    message = struct_pb2.Struct()
    message.update(dict(value or {}))
    return message


def _struct_to_dict(message) -> dict:
    from google.protobuf import json_format

    return json_format.MessageToDict(message, preserving_proto_field_name=True)
