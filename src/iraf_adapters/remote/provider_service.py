# -*- coding: utf-8 -*-
"""Capability Provider 的**跨机服务端**（x86 仿真侧，2026-10-16）。

分层（与 `iraf_adapters/grpc/server.py` 不是一回事，勿混用）
------------------------------------------------------------
```
板卡（agent-intel 域）                        x86（仿真主机，本模块）
TaskFlow → Skill Runtime → PolicyGateway  ──►  CapabilityProviderService
   （策略/编排/审计都在板上）                      └─► 本地 Capability Provider（MuJoCo）
                                                        └─► 窗口显示在本机桌面
```
`grpc/server.py` 暴露的是**任务级** `SkillRuntimeService`（服务端自己跑完整策略链）；
本模块暴露的是**能力级** `CapabilityProviderService`：调用方已完成策略判定，
这里只把**单个能力**执行出来。**本模块不做策略判定**（不在这一层），
但必须做铁律 2.12 要求的那些事（服务端生成 execution_id、租约、fencing token、幂等、拒绝过期）。

服务端职责（逐条写死，可被拒绝路径用例验证）
------------------------------------------------
1. **生成 execution_id**（调用方没给就由服务端给），并持有该 execution 的本地租约；
2. **拒绝**：缺 deadline ／ deadline 已过 ／ fencing token 回退 ／ 终态执行的再发控制请求；
3. **幂等**：同 `idempotency_key` ⇒ 返回既有结果，不重复执行；
4. **不伪造成功**：能力返回什么就报什么；异常转 `IrafError` 且 `accepted=false`；
5. **如实报告显示**：`Health.window_opened` 只在窗口真的建立时才为 true。

窗口与线程模型（实测约束，见 `docs/debug/2026-10-16-piper-viewer-window-lifecycle.md`）
--------------------------------------------------------------------------------------
MuJoCo 窗口（GLFW）必须在**主线程**创建与 sync；gRPC 服务在子线程。
因此：主线程跑 `viewer_runner.run_live_mirror`（只渲染不推进 + `SnapshotMirror` 并发契约），
能力调用在 gRPC 线程里执行，两者通过 `backend.display_lock()` 互斥 —— 与 `run_request_live`
已验证的模型一致。
"""
from __future__ import annotations

import threading
import time
import uuid
from typing import Any, Dict, Optional

from iraf_core.authority import LeaseConflict
from iraf_core.policy import AuthenticatedContext
from iraf_adapters.factory import CAPABILITY_METHODS, MOTION_CAPABILITIES

#: 拒绝码（显式、可被用例断言）
CODE_UNKNOWN_CAPABILITY = "IRAF-CAPABILITY-UNKNOWN"
CODE_DEADLINE_MISSING = "IRAF-DEADLINE-MISSING"
CODE_DEADLINE_EXCEEDED = "IRAF-DEADLINE-EXCEEDED"
CODE_FENCING_REGRESSION = "IRAF-FENCING-TOKEN-REGRESSION"
CODE_TERMINAL_REUSE = "IRAF-EXECUTION-TERMINAL"
CODE_UNAUTHENTICATED = "IRAF-UNAUTHENTICATED"
CODE_RESOURCE_BUSY = "IRAF-RESOURCE-BUSY"
CODE_NOT_IMPLEMENTED = "IRAF-PROVIDER-NOT-IMPLEMENTED"

#: 控制类能力额外参数（显式表，避免"猜签名"）
_POSITIONAL = {
    "move_joint": ("positions", "duration_ms"),
    "pick_object": ("target_id", "grasp_pose", "duration_ms"),
    "place_object": ("place_target_id", "payload_id", "duration_ms"),
    "accept_payload": ("payload_id", "place_target_id"),
    "visual_pick": ("inputs",),
    "calibrate_grasp": (),
    "calibrate_camera_to_base": ("inputs",),
    "stand": (),
    # 2026-10-19 板上实测：skills/stop 的技能名是 stop，而本表只登记了 emergency_stop
    # ⇒ 语义解析出 stop 后被打成 IRAF-CAPABILITY-UNKNOWN。零参数能力用空元组登记。
    "stop": (),
    "locomote": ("velocity", "duration_ms"),
    "dock_for_handoff": (),
    "read_state": (),
    "emergency_stop": (),
    # 场景复位（2026-10-23）：开发/调试用。**必须持租约** —— 它改的是物理状态，
    # 与运行中的任务并发会互相破坏（不要加进 _NO_LEASE）。
    "reset_scene": ("scope", "target_id"),
}
#: 不需要 lease 的能力（纯查询 / 无动作）
_NO_LEASE = {"step", "runtime_inventory", "read_state"}


def safety_state_from_inventory(inventory) -> Dict[str, Any]:
    """从运行时清单判定被控对象的 safety 状态（**纯函数**，语义只此一处）。

    - `safety.estop == true` 或 `safety.quarantined == true` ⇒ 已隔离；
    - 清单里**没有** `safety` 段 ⇒ 不臆造隔离（真隔离必须由被控对象给出，不靠我们猜）；
    - 判据是**状态驱动**（读被控对象状态），不是"猜异常类型"。
    """
    safety = dict((inventory or {}).get("safety") or {})
    estop = bool(safety.get("estop"))
    quarantined = bool(safety.get("quarantined")) or estop
    return {"quarantined": quarantined,
            "kind": str(safety.get("kind") or ("ESTOP" if estop else "SAFETY_EVENT")),
            "reason": str(safety.get("reason") or ("被控对象急停已触发" if estop else ""))}


class CapabilityRejected(RuntimeError):
    """带显式错误码的拒绝（服务端**先拒绝、再执行**，不做"尽力而为"）。"""

    def __init__(self, code: str, message: str, retryable: bool = False):
        super().__init__(message)
        self.code = code
        self.retryable = retryable


class ExecutionRegistry:
    """服务端执行登记表：execution_id ↔ 本地租约 / 终态 / 幂等结果 / 取消标志。

    为什么要有它：铁律 2.12 要求"旧 token 或终态执行发出的控制请求必须被拒绝"。
    跨机调用方传来的 fencing token 只用于**单调性**判定；
    真正驱动 Provider 的是**服务端本地**租约（`ControlAuthorityManager` 生成）。
    """

    def __init__(self, authority, resource: str, owner_prefix: str = "remote-runtime",
                 ttl_seconds: float = 120.0):
        self._authority = authority
        self._resource = resource
        self._owner_prefix = owner_prefix
        self._ttl = float(ttl_seconds)
        self._lock = threading.Lock()
        self._leases: Dict[str, Any] = {}          # execution_id -> Lease
        self._last_token: Dict[str, int] = {}      # resource -> 已见最大 fencing token
        self._terminal: Dict[str, str] = {}        # execution_id -> 终态名
        self._quarantined: Dict[str, Dict[str, str]] = {}   # resource -> {kind, reason}（锁存）
        self._by_correlation: Dict[str, str] = {}           # correlation_id -> execution_id
        self._idempotent: Dict[str, Any] = {}      # idempotency_key -> 结果摘要
        self._cancel: Dict[str, threading.Event] = {}

    # ---- 查询 ----
    def cancel_event(self, execution_id: str) -> threading.Event:
        with self._lock:
            return self._cancel.setdefault(execution_id, threading.Event())

    def requested_cancel(self, execution_id: str) -> bool:
        return self.cancel_event(execution_id).is_set()

    def cached(self, idempotency_key: str):
        if not idempotency_key:
            return None
        with self._lock:
            return self._idempotent.get(idempotency_key)

    def link_correlation(self, correlation_id: str, execution_id: str) -> None:
        """记录 correlation → execution_id：调用方在**执行中**只知道 correlation_id，
        取消请求只能带它（execution_id 是服务端生成、要等调用返回才拿到）。"""
        if correlation_id:
            with self._lock:
                self._by_correlation[str(correlation_id)] = str(execution_id)

    def resolve_execution_id(self, identifier: str):
        """把 Cancel 收到标识解析成 execution_id（支持 execution_id 或 correlation_id）。"""
        identifier = str(identifier or "")
        if not identifier:
            return None
        with self._lock:
            if identifier in self._terminal or identifier in self._leases:
                return identifier
            return self._by_correlation.get(identifier)

    def quarantine(self, resource: str, kind: str = "SAFETY_EVENT", reason: str = "") -> None:
        """锁存隔离：一旦被控对象报告安全事件，**授权复位前**不再放行（铁律 1.9）。

        锁存是刻意的 —— 不因为"下一次读到的状态是好的"就自动解除。
        """
        with self._lock:
            self._quarantined[str(resource)] = {"kind": str(kind or "SAFETY_EVENT"),
                                                "reason": str(reason or "")}

    def quarantined(self, resource: str):
        with self._lock:
            return self._quarantined.get(str(resource))

    def mark(self, execution_id: str, state: str) -> None:
        with self._lock:
            self._terminal[execution_id] = state

    # ---- 校验 / 取租约 ----
    def resolve(self, envelope, capability: str) -> tuple:
        """校验信封并返回 `(execution_id, lease)`；不合规即 `CapabilityRejected`。"""
        deadline_s = self._deadline_seconds(envelope)
        now = time.time()
        if deadline_s is None:
            raise CapabilityRejected(CODE_DEADLINE_MISSING,
                                     "跨机能力调用必须带 deadline（铁律 2.3：无超时的等待禁止）")
        if deadline_s <= now:
            raise CapabilityRejected(CODE_DEADLINE_EXCEEDED,
                                     "deadline 已过（%.3f ≤ now %.3f）：拒绝执行" % (deadline_s, now))

        execution_id = str(envelope.execution_id or "").strip() or ("exec-" + uuid.uuid4().hex)
        resource = str(envelope.resource_id or "").strip() or self._resource
        with self._lock:
            if envelope.correlation_id:
                self._by_correlation[str(envelope.correlation_id)] = execution_id
        token = int(envelope.fencing_token or 0)

        with self._lock:
            state = self._terminal.get(execution_id)
            if state and capability in MOTION_CAPABILITIES:
                raise CapabilityRejected(
                    CODE_TERMINAL_REUSE,
                    "execution %s 已处于终态 %s：终态执行的再发控制请求必须被拒绝" % (execution_id, state))
            seen = self._last_token.get(resource, 0)
            if token and token < seen:
                raise CapabilityRejected(
                    CODE_FENCING_REGRESSION,
                    "fencing token 回退：收到 %d < 已见 %d（旧 token 必须被拒绝）" % (token, seen))
            if token:
                self._last_token[resource] = max(seen, token)
            lease = self._leases.get(execution_id)

        if capability in _NO_LEASE:
            return execution_id, None
        if lease is None:
            owner = "%s:%s" % (self._owner_prefix, execution_id)
            try:
                lease = self._authority.acquire(resource, owner, ttl_seconds=self._ttl)
            except LeaseConflict as exc:
                raise CapabilityRejected(CODE_RESOURCE_BUSY, str(exc), retryable=True) from exc
            with self._lock:
                self._leases[execution_id] = lease
        else:
            try:
                self._authority.validate(lease)
            except LeaseConflict:
                # 租约过期：重新获取（同一 execution 续租，fencing token 单调上升）
                owner = "%s:%s" % (self._owner_prefix, execution_id)
                lease = self._authority.acquire(resource, owner, ttl_seconds=self._ttl)
                with self._lock:
                    self._leases[execution_id] = lease
        return execution_id, lease

    def release(self, execution_id: str, state: str) -> bool:
        """放开该 execution 的**执行域**租约（单次动作结束即放开，铁律 2.12）。

        返回"是否真的释放了租约"。没有租约时**不写终态**：拒绝路径（deadline / fencing /
        BUSY）本身就没取到租约，若照旧写终态，会把"从未执行的调用"记成终态，
        空的 execution_id 还会污染登记表。
        """
        with self._lock:
            lease = self._leases.pop(execution_id, None)
            if lease is None:
                return False
            self._terminal[execution_id] = state
        try:
            self._authority.release(lease)
        except LeaseConflict:
            pass  # 已过期 / 已被回收：不视为错误
        return True

    def remember(self, idempotency_key: str, summary: Any) -> None:
        if idempotency_key:
            with self._lock:
                self._idempotent[idempotency_key] = summary

    @staticmethod
    def _deadline_seconds(envelope) -> Optional[float]:
        try:
            if not envelope.HasField("deadline"):
                return None
        except ValueError:
            return None
        ts = envelope.deadline
        return float(ts.seconds) + float(ts.nanos) / 1e9


def build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2):
    """构造 gRPC servicer 类（把生成的桩作为参数注入，避免核心模块硬依赖 grpc 代码生成物）。

    为什么不在本模块顶层 import 桩：桩是生成物（`build/generated/python`，见 `scripts/gen_proto_stubs.sh`），
    IRAF 核心与 SDK 的其它部分不该因为"没跑过生成脚本"而 import 失败。
    """

    class CapabilityProviderServicer(provider_pb2_grpc.CapabilityProviderServiceServicer):
        """把 `CapabilityCall` 落到本地 Provider（MuJoCo）上执行。"""

        def __init__(self, runtime, token: str, subject: str = "board-runtime",
                     provider_id: str = "iraf-adapters/mujoco", ttl_seconds: float = 120.0,
                     display_state: Optional[Dict[str, Any]] = None):
            self._runtime = runtime
            self._backend = runtime.backend
            self._token = token or ""
            self._subject = subject
            self._provider_id = provider_id
            self._resources = getattr(runtime, "_resource_hint", None) or "robot-arm"
            self._registry = ExecutionRegistry(runtime.authority, self._resources, ttl_seconds=ttl_seconds)
            self._display_state = display_state if display_state is not None else {}
            self._declared = self._declared_capabilities()

        # ---------- 辅助 ----------
        def _declared_capabilities(self):
            caps = set()
            for name in CAPABILITY_METHODS:
                if hasattr(self._backend, CAPABILITY_METHODS[name]):
                    caps.add(name)
            declared = getattr(self._runtime.profile, "capabilities", None) or {}
            names = set(declared.keys()) if isinstance(declared, dict) else set(declared)
            # declared ⊆ implemented：声明了但没实现的能力不在此列出（装配期已由 factory 拦截）
            return sorted(caps & names) if names else sorted(caps)

        def _auth(self, context, scopes):
            md = dict(context.invocation_metadata() or [])
            provided = md.get("authorization") or md.get("x-iraf-token") or ""
            if provided.lower().startswith("bearer "):
                provided = provided[7:]
            if not self._token or provided != self._token:
                raise CapabilityRejected(CODE_UNAUTHENTICATED, "跨机 provider 调用鉴权失败")
            return AuthenticatedContext(self._subject, frozenset(scopes), "bearer")

        def _error(self, capability, exc) -> "Any":
            code = getattr(exc, "code", "IRAF-PROVIDER-ERROR")
            return skill_pb2.IrafError(code=code, message_zh=str(exc),
                                       retryable=bool(getattr(exc, "retryable", False)))

        def _safety_state(self) -> Dict[str, Any]:
            """被控对象的 safety 状态 —— 跨机安全事件的**唯一来源**（状态驱动，不靠猜异常类型）。

            读不到清单时按"已隔离"处理（fail-closed）：宁可拒绝，也不在状态不明时放行物理动作。
            """
            getter = getattr(self._backend, "runtime_inventory", None)
            if not callable(getter):
                return {"quarantined": True, "kind": "INVENTORY_UNAVAILABLE",
                        "reason": "被控对象未提供运行时清单，状态不明"}
            try:
                inventory = getter() or {}
            except Exception as exc:  # noqa: BLE001 状态读不到 = 不放行
                return {"quarantined": True, "kind": "INVENTORY_UNAVAILABLE",
                        "reason": "读取运行时清单失败：%s: %s" % (type(exc).__name__, exc)}
            return safety_state_from_inventory(inventory)

        def _capability_call(self, capability, inputs: Dict[str, Any], lease):
            method_name = CAPABILITY_METHODS.get(capability)
            if method_name is None:
                raise CapabilityRejected(CODE_UNKNOWN_CAPABILITY,
                                         "未登记的能力 %r（契约词表见 factory.CAPABILITY_METHODS）" % capability)
            method = getattr(self._backend, method_name, None)
            if not callable(method):
                raise CapabilityRejected(CODE_NOT_IMPLEMENTED,
                                         "Provider 未实现 %s，拒绝伪造成功" % method_name)
            names = _POSITIONAL.get(capability)
            if names is None:
                raise CapabilityRejected(CODE_UNKNOWN_CAPABILITY,
                                         "能力 %r 未登记参数形状（请先改 api/ 契约）" % capability)
            args = [inputs.get(key) for key in names]
            if capability == "move_joint" and inputs.get("duration_ms") is not None:
                args[-1] = int(inputs["duration_ms"])
            if capability in ("pick_object", "place_object") and inputs.get("duration_ms") is not None:
                args[-1] = int(inputs["duration_ms"])
            return method(*args) if lease is None else method(*args, lease)

        # ---------- RPC ----------
        def Health(self, request, context):
            import grpc as _grpc
            from google.protobuf.json_format import ParseDict
            try:
                self._auth(context, ("task.read",))
            except CapabilityRejected as exc:
                context.abort(_grpc.StatusCode.UNAUTHENTICATED, str(exc))
            disp = self._display_state or {}
            inventory = {}
            try:
                getter = getattr(self._backend, "runtime_inventory", None)
                if callable(getter):
                    inventory = getter() or {}
            except Exception as exc:  # noqa: BLE001 如实上报清单不可用，不假装有清单
                inventory = {"inventory_error": "%s: %s" % (type(exc).__name__, exc)}
            resp = provider_pb2.ProviderHealthResponse(
                ready=True,
                simulation=True,                      # x86 侧一律仿真（铁律 1.7）
                backend=self._provider_id,
                capabilities=self._declared,
                fencing_token=int(self._registry._last_token.get(self._resources, 0)),
                display_mode=str(disp.get("display_mode") or "unknown"),
                window_opened=bool(disp.get("window_opened")),
                detail=str(disp.get("detail") or ""),
            )
            if inventory:
                ParseDict(inventory, resp.inventory)
            return resp

        def Invoke(self, request, context):
            from google.protobuf.json_format import MessageToDict, ParseDict
            try:
                self._auth(context, ("task.submit",))
            except CapabilityRejected as exc:
                return provider_pb2.CapabilityResult(
                    capability=request.capability, accepted=False, simulation=True,
                    provider=self._provider_id, error=self._error(request.capability, exc),
                    fencing_token=0, occurred_at=_now_ts())
            capability = str(request.capability or "")
            inputs = MessageToDict(request.inputs) if request.HasField("inputs") else {}
            envelope = request.lease
            # 跨机安全事件传导（铁律 1.6/1.9）：被控对象一旦进入安全隔离，服务端拒绝后续调用，
            # 并如实回给调用方 —— 调用方据此在**本侧**也隔离该资源，避免"远端已隔离、板上还在调度"。
            safety_resource = str(envelope.resource_id or "").strip() or self._resources
            live = self._safety_state()
            if live.get("quarantined"):
                self._registry.quarantine(safety_resource, live.get("kind", ""), live.get("reason", ""))
            latched = self._registry.quarantined(safety_resource)
            if latched:
                return provider_pb2.CapabilityResult(
                    capability=capability, accepted=False, simulation=True,
                    provider=self._provider_id, occurred_at=_now_ts(),
                    error=skill_pb2.IrafError(
                        code="IRAF-SAFETY-QUARANTINED",
                        message_zh="被控对象处于安全隔离（%s）：%s"
                                   % (latched.get("kind", ""), latched.get("reason", "")),
                        retryable=False,
                        details={"resource": safety_resource,
                                 "kind": latched.get("kind", ""),
                                 "reason": latched.get("reason", "")}))
            idem = str(envelope.idempotency_key or "")
            cached = self._registry.cached(idem)
            if cached is not None:
                result = provider_pb2.CapabilityResult()
                ParseDict(cached, result)
                result.capability = capability
                return result
            execution_id = str(envelope.execution_id or "")
            try:
                execution_id, lease = self._registry.resolve(envelope, capability)
                outputs = self._capability_call(capability, inputs, lease)
            except CapabilityRejected as exc:
                # 拒绝也可能发生在"租约已取到之后"（能力未实现 / 参数形状未登记）：
                # 同样要放开，否则资源被占到 TTL 到期。
                if not self._registry.release(execution_id, "REJECTED"):
                    if capability in MOTION_CAPABILITIES and execution_id:
                        self._registry.mark(execution_id, "REJECTED")
                return provider_pb2.CapabilityResult(
                    capability=capability, accepted=False, simulation=True,
                    provider=self._provider_id, error=self._error(capability, exc),
                    fencing_token=0, occurred_at=_now_ts())
            except Exception as exc:  # noqa: BLE001 显式转换，不吞错、不伪造成功
                self._registry.release(execution_id, "FAILED")
                return provider_pb2.CapabilityResult(
                    capability=capability, accepted=False, simulation=True,
                    provider=self._provider_id,
                    error=skill_pb2.IrafError(code="IRAF-PROVIDER-ERROR", message_zh=str(exc),
                                              retryable=False),
                    fencing_token=0, occurred_at=_now_ts())
            payload = {}
            if isinstance(outputs, dict):
                payload = outputs
            result = provider_pb2.CapabilityResult(
                capability=capability, accepted=True, simulation=True, provider=self._provider_id,
                fencing_token=int(getattr(lease, "fencing_token", 0) or 0), occurred_at=_now_ts())
            ParseDict(payload, result.outputs)
            # execution_id 由服务端生成时**必须**回传（铁律 2.12），放 outputs 里显式给出
            result.outputs["provider_execution_id"] = execution_id
            self._registry.remember(idem, MessageToDict(result))
            # 执行域租约：本动作已完成 ⇒ 立刻放开（2026-10-26）。不放开会让同一资源在
            # ttl_seconds 内被判 IRAF-RESOURCE-BUSY —— 实测「复位 → 立刻抓」就是这么被卡的：
            # 服务端只有异常路径调 release，成功路径把 robot-arm 占满 120 s。
            # 安全边界不变：下一次动作要重新 acquire，fencing token 单调上升；
            # 终态执行的再发控制请求仍由 resolve() 拒绝。
            self._registry.release(execution_id, "SUCCEEDED")
            return result

        def Cancel(self, request, context):
            import grpc as _grpc
            try:
                self._auth(context, ("task.cancel", "task.submit"))
            except CapabilityRejected as exc:
                context.abort(_grpc.StatusCode.UNAUTHENTICATED, str(exc))
            execution_id = str(request.execution_id or "")
            resolved = self._registry.resolve_execution_id(execution_id)
            if resolved:
                execution_id = resolved
            event = self._registry.cancel_event(execution_id)
            if execution_id in self._registry._terminal:
                return skill_pb2.CancelExecutionResponse(accepted=False, state=skill_pb2.SKILL_STATE_CANCELLED)
            # 不硬停：能力一旦进入 Provider 就没有中途取消点（诚实语义，不假装停住了）
            event.set()
            self._registry.mark(execution_id, "CANCEL_REQUESTED")
            return skill_pb2.CancelExecutionResponse(accepted=True, state=skill_pb2.SKILL_STATE_CANCELLED)

    return CapabilityProviderServicer


def _now_ts():
    from google.protobuf.timestamp_pb2 import Timestamp
    ts = Timestamp()
    ts.GetCurrentTime()
    return ts
