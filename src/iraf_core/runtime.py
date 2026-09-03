"""IRAF SkillRuntime: policy, registry, leases, cancellation and execution records."""
from dataclasses import dataclass, field
import hashlib
import json
import threading
import uuid

from .core import TaskStatus
from .policy import PolicyGateway, PolicyRejected
from .safety import SafetyQuarantine, QuarantineError
from .taskflow import TERMINAL, TaskFlow

@dataclass
class ActiveExecution:
    execution_id: str
    subject: str
    status: TaskStatus = TaskStatus.PENDING
    sequence: int = 0
    reason: str = ""
    lease: object = None
    cancel_reason: str = ""
    stop_error: str = ""
    stop_invoked: bool = False
    resource: str = ""
    cancel_event: threading.Event = field(default_factory=threading.Event)

class SkillRuntime:
    def __init__(self, profile, safety_policy, backend, registry, authority, store, policy=None, resource_id=None, safety=None):
        self.profile = profile
        self.safety_policy = safety_policy
        self.backend = backend
        self.registry = registry
        self.authority = authority
        self.store = store
        self.policy = policy or PolicyGateway()
        self.resource_id = resource_id or profile.name
        self.safety = safety or SafetyQuarantine(store=store)
        self._lock = threading.RLock()
        self._active_lock = threading.RLock()
        self._active = {}

    @staticmethod
    def execution_id_for(subject, idempotency_key):
        if idempotency_key:
            return str(uuid.uuid5(uuid.NAMESPACE_URL, f"iraf:{subject}:{idempotency_key}"))
        return str(uuid.uuid4())

    def execute(self, request, context, execution_id=None, ready_event=None):
        if context is None or not getattr(context, "subject", ""):
            if ready_event is not None: ready_event.set()
            return self._standalone_failure(request.get("correlation_id", ""), "IRAF-UNAUTHENTICATED", "missing authenticated context")
        key = request.get("idempotency_key", "")
        requested_id = execution_id or self.execution_id_for(context.subject, key)
        control = ActiveExecution(requested_id, context.subject)
        with self._active_lock:
            if requested_id in self._active:
                if ready_event is not None: ready_event.set()
                return self._standalone_failure(request.get("correlation_id", ""), "IRAF-EXECUTION-CONFLICT", "execution is already active")
            self._active[requested_id] = control
        if ready_event is not None: ready_event.set()
        digest = self._request_digest(request)
        try:
            with self._lock:
                existing = self.store.find_idempotent(context.subject, key) if key else None
                if existing:
                    if existing[0] == digest: return existing[1]
                    return self._standalone_failure(request.get("correlation_id", ""), "IRAF-IDEMPOTENCY-CONFLICT", "same idempotency key maps to different request")
                result = self._decorate_result(
                    self._execute_once(request, context, control), request
                )
                if key: self.store.save_result(context.subject, key, digest, result)
                return result
        finally:
            with self._active_lock:
                self._active.pop(requested_id, None)

    def _execute_once(self, request, context, control):
        flow = TaskFlow(control.execution_id)
        correlation = request.get("correlation_id", "")
        self.store.append_event(flow.execution_id, flow.sequence, flow.status.value)
        if control.cancel_event.is_set():
            return self._finish_cancel(flow, control, correlation)
        self._transition(flow, control, TaskStatus.VALIDATING)
        skill = self.registry.resolve(request.get("skill"), request.get("skill_version_constraint", ""))
        if skill is None:
            self._transition(flow, control, TaskStatus.FAILED, "Skill not registered or version mismatch")
            return self._result(flow, correlation, "IRAF-SKILL-PROVIDER-UNAVAILABLE", "Skill not registered or version mismatch")
        inventory = self.backend.runtime_inventory() if hasattr(self.backend, "runtime_inventory") else {}
        try:
            decision = self.policy.validate(request, context, skill, self.profile, self.safety_policy, inventory)
        except PolicyRejected as exc:
            self._transition(flow, control, TaskStatus.FAILED, str(exc))
            return self._result(flow, correlation, exc.code, str(exc))
        if control.cancel_event.is_set():
            return self._finish_cancel(flow, control, correlation)
        lease = None
        try:
            resource = request.get("resource_id") or self.resource_id
            control.resource = resource
            self.safety.require_operational(resource)
            controller = request.get("controller", "default")
            lease = self.authority.acquire(resource, controller, ttl_seconds=skill.manifest.timeout_seconds)
            with self._active_lock:
                control.lease = lease
            self._transition(flow, control, TaskStatus.RUNNING)
            if control.cancel_event.is_set():
                self._stop_active(control)
                return self._finish_cancel(flow, control, correlation)
            invoked = skill.invoke(self.profile, self.backend, decision.parameters, lease)
            if control.cancel_event.is_set():
                return self._finish_cancel(flow, control, correlation)
            self._transition(flow, control, TaskStatus.SUCCEEDED)
            result = self._result(flow, correlation)
            result.update({"result": invoked.output, "skill": {"name": skill.manifest.name, "version": skill.manifest.version, "digest": skill.manifest.digest}, "provider": {"name": invoked.provider_name, "type": invoked.provider_type}, "profile": {"name": self.profile.name, "version": self.profile.version, "digest": self.profile.digest}, "safety_policy": {"name": self.safety_policy.name, "version": self.safety_policy.version, "digest": self.safety_policy.digest}, "policy_decision_id": decision.decision_id, "policy_version": decision.gateway_version, "resource_id": resource, "controller": controller})
            return result
        except QuarantineError as exc:
            self._transition(flow, control, TaskStatus.FAILED, str(exc))
            return self._result(flow, correlation, "IRAF-SAFETY-QUARANTINED", str(exc))
        except Exception as exc:
            if control.cancel_event.is_set():
                if not control.stop_error: control.stop_error = str(exc)
                return self._finish_cancel(flow, control, correlation)
            if flow.status in {TaskStatus.VALIDATING, TaskStatus.RUNNING}:
                self._transition(flow, control, TaskStatus.FAILED, str(exc))
            return self._result(flow, correlation, "IRAF-EXECUTION-FAILED", str(exc))
        finally:
            if lease is not None:
                self.authority.release(lease)
            with self._active_lock:
                control.lease = None

    def cancel(self, execution_id, context, reason="cancel requested"):
        if context is None or not getattr(context, "subject", ""):
            return {"accepted": False, "status": "", "error_code": "IRAF-UNAUTHENTICATED", "reason": "missing authenticated context"}
        with self._active_lock:
            control = self._active.get(execution_id)
            if control is None:
                final = self.store.get(execution_id)
                return {"accepted": False, "status": final.get("status", "") if final else "", "error_code": "IRAF-EXECUTION-NOT-ACTIVE", "reason": "execution is not active"}
            roles = set(getattr(context, "roles", ()))
            if control.subject != context.subject and "task.cancel.any" not in roles:
                return {"accepted": False, "status": control.status.value, "error_code": "IRAF-POLICY-DENIED", "reason": "caller does not own execution"}
            if control.status in TERMINAL:
                return {"accepted": False, "status": control.status.value, "error_code": "IRAF-EXECUTION-NOT-ACTIVE", "reason": "execution is terminal"}
            control.cancel_reason = str(reason or "cancel requested")
            control.cancel_event.set()
            lease = control.lease
        if lease is not None:
            self._stop_active(control)
        return {"accepted": not bool(control.stop_error), "status": control.status.value, "error_code": "IRAF-CANCEL-STOP-FAILED" if control.stop_error else "", "reason": control.stop_error or control.cancel_reason}

    def _stop_active(self, control):
        with self._active_lock:
            lease = control.lease
            if lease is None or control.stop_error or control.stop_invoked:
                return
            control.stop_invoked = True
        if not hasattr(self.backend, "stop"):
            control.stop_error = "Backend does not implement stop"
            return
        try:
            self.backend.stop(lease)
        except Exception as exc:
            control.stop_error = str(exc)

    def _finish_cancel(self, flow, control, correlation):
        if control.stop_error and control.resource:
            self.safety.quarantine(control.resource, control.stop_error, kind="CANCEL_STOP_FAILED")
        status = TaskStatus.SAFETY_STOP if control.stop_error else TaskStatus.CANCELLED
        reason = control.stop_error or control.cancel_reason or "cancel requested"
        if flow.status not in TERMINAL:
            self._transition(flow, control, status, reason)
        code = "IRAF-CANCEL-STOP-FAILED" if control.stop_error else "IRAF-CANCELLED"
        return self._result(flow, correlation, code, reason)

    def _transition(self, flow, control, status, reason=""):
        flow.transition(status, reason)
        with self._active_lock:
            control.status = flow.status
            control.sequence = flow.sequence
            control.reason = reason
        self.store.append_event(flow.execution_id, flow.sequence, flow.status.value, reason)

    def handle_safety_event(self, resource, reason, kind="SAFETY_EVENT"):
        return self.safety.quarantine(resource, reason, kind)

    def recover_safety(self, resource, event_id, controller_safe_state_ack, actor):
        return self.safety.recover(resource, event_id, controller_safe_state_ack, actor)

    def get(self, execution_id):
        with self._active_lock:
            control = self._active.get(execution_id)
            if control is not None:
                return {"execution_id": control.execution_id, "status": control.status.value, "sequence": control.sequence, "reason": control.reason, "error_code": ""}
        return self.store.get(execution_id)

    def record_pre_dispatch_failure(self, request, context, error_code, reason, metadata=None):
        """持久化 Provider/adapter 在 Skill 调度前产生的安全失败。"""
        correlation = request.get("correlation_id", "")
        metadata = dict(metadata or {})
        self._validate_execution_metadata(metadata)
        if (
            context is None
            or not getattr(context, "subject", "")
            or getattr(context, "transport", "") not in {"bearer", "local"}
        ):
            return self._standalone_failure(
                correlation, "IRAF-UNAUTHENTICATED", "missing authenticated context"
            )
        if "task.submit" not in set(getattr(context, "roles", ())):
            return self._standalone_failure(
                correlation, "IRAF-POLICY-DENIED", "调用方无任务提交权限"
            )
        key = request.get("idempotency_key", "")
        if not key:
            result = self._decorate_result(
                self._standalone_failure(correlation, error_code, reason), request
            )
            result.update(metadata)
            return result
        digest = self._request_digest(request)
        with self._lock:
            existing = self.store.find_idempotent(context.subject, key)
            if existing:
                if existing[0] == digest:
                    return existing[1]
                return self._standalone_failure(
                    correlation,
                    "IRAF-IDEMPOTENCY-CONFLICT",
                    "same idempotency key maps to different request",
                )
            execution_id = self.execution_id_for(context.subject, key)
            control = ActiveExecution(execution_id, context.subject)
            flow = TaskFlow(execution_id)
            self.store.append_event(execution_id, flow.sequence, flow.status.value)
            self._transition(flow, control, TaskStatus.VALIDATING)
            self._transition(flow, control, TaskStatus.FAILED, reason)
            result = self._decorate_result(
                self._result(flow, correlation, error_code, reason), request
            )
            result.update(metadata)
            self.store.save_result(context.subject, key, digest, result)
            return result

    def annotate_execution(self, execution_id, metadata):
        self._validate_execution_metadata(metadata)
        return self.store.annotate_result(execution_id, metadata)

    @staticmethod
    def _validate_execution_metadata(metadata):
        allowed = {
            "adapter",
            "intent_provider",
            "intent_request_digest",
            "resolved_skill",
        }
        unexpected = sorted(set(metadata) - allowed)
        if unexpected:
            raise ValueError("不允许追加的执行元数据: " + str(unexpected))

    def _decorate_result(self, result, request):
        """为成功和失败结果冻结相同的回放边界元数据。"""
        result.setdefault(
            "requested_skill",
            {
                "name": str(request.get("skill", "")),
                "version_constraint": str(
                    request.get("skill_version_constraint", "")
                ),
            },
        )
        result.setdefault(
            "profile",
            {
                "name": self.profile.name,
                "version": self.profile.version,
                "digest": self.profile.digest,
            },
        )
        result.setdefault(
            "safety_policy",
            {
                "name": self.safety_policy.name,
                "version": self.safety_policy.version,
                "digest": self.safety_policy.digest,
            },
        )
        result.setdefault("resource_id", request.get("resource_id") or self.resource_id)
        result.setdefault("controller", request.get("controller", "default"))
        result.setdefault("simulation", bool(self.profile.simulation))
        return result

    @staticmethod
    def _request_digest(request):
        return hashlib.sha256(
            json.dumps(
                request,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
            ).encode()
        ).hexdigest()

    def _standalone_failure(self, correlation, code, reason):
        flow = TaskFlow(str(uuid.uuid4()))
        flow.transition(TaskStatus.VALIDATING)
        flow.transition(TaskStatus.FAILED, reason)
        return self._result(flow, correlation, code, reason)

    @staticmethod
    def _result(flow, correlation, error_code="", reason=""):
        return {"execution_id": flow.execution_id, "correlation_id": correlation, "status": flow.status.value, "sequence": flow.sequence, "error_code": error_code, "reason": reason}
