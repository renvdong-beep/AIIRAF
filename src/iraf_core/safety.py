"""Fail-closed SafetyEvent quarantine and explicit recovery gate."""
from dataclasses import dataclass
from enum import Enum
import threading
import time
import uuid


class SafetyState(str, Enum):
    OPERATIONAL = "OPERATIONAL"
    QUARANTINED = "QUARANTINED"


class QuarantineError(RuntimeError):
    def __init__(self, resource, event):
        self.resource = resource
        self.event = event
        super().__init__(f"resource quarantined: {resource}: {event.reason}")


class RecoveryRejected(ValueError):
    pass


class SafetyEventReported(RuntimeError):
    """由 **Provider / 适配层**上报的安全事件（跨机场景下来自远端仿真或控制域）。

    为什么契约放在 core（铁律 2.5：core 不得依赖适配层）：
    适配层（如远端 Capability Provider 客户端）抛出本异常，
    `SkillRuntime` 捕获后调 `handle_safety_event()` 把**本侧**资源一并隔离 ——
    否则就会出现"远端已经隔离、板上还在继续调度"的静默不一致（铁律 1.6/1.9）。
    """

    def __init__(self, resource, reason, kind="SAFETY_EVENT"):
        self.resource = str(resource or "")
        self.reason = str(reason or "")
        self.kind = str(kind or "SAFETY_EVENT")
        super().__init__("资源 %s 报告安全事件[%s]：%s" % (self.resource, self.kind, self.reason))


@dataclass(frozen=True)
class SafetyEvent:
    event_id: str
    resource: str
    kind: str
    reason: str
    occurred_at_ns: int


class SafetyQuarantine:
    """Fail-closed gate backed by the execution store when one is provided."""
    def __init__(self, clock_ns=None, store=None):
        self._clock_ns = clock_ns or time.monotonic_ns
        self._store = store
        self._events = {}
        self._lock = threading.RLock()

    @staticmethod
    def _from_record(record):
        return SafetyEvent(record["event_id"], record["resource"], record["kind"], record["reason"], int(record["occurred_at_ns"]))

    def quarantine(self, resource, reason, kind="SAFETY_EVENT"):
        if not resource or not reason:
            raise ValueError("resource and reason are required")
        with self._lock:
            existing = self.event(resource)
            if existing is not None:
                return existing
            event = SafetyEvent(str(uuid.uuid4()), resource, str(kind), str(reason), self._clock_ns())
            if self._store is not None:
                record = self._store.save_safety_event(event)
                event = self._from_record(record)
            self._events[resource] = event
            return event

    def require_operational(self, resource):
        event = self.event(resource)
        if event is not None:
            raise QuarantineError(resource, event)

    def state(self, resource):
        return SafetyState.QUARANTINED if self.event(resource) is not None else SafetyState.OPERATIONAL

    def event(self, resource):
        with self._lock:
            if self._store is not None:
                record = self._store.get_safety_event(resource)
                event = self._from_record(record) if record is not None else None
                if event is None:
                    self._events.pop(resource, None)
                else:
                    self._events[resource] = event
                return event
            return self._events.get(resource)

    def recover(self, resource, event_id, controller_safe_state_ack, actor):
        if not actor:
            raise RecoveryRejected("authorized recovery actor is required")
        if not controller_safe_state_ack:
            raise RecoveryRejected("controller safe-state acknowledgement is required")
        with self._lock:
            event = self.event(resource)
            if event is None:
                raise RecoveryRejected("resource is not quarantined")
            if event.event_id != event_id:
                raise RecoveryRejected("safety event id does not match quarantine")
            if self._store is not None and not self._store.delete_safety_event(resource, event_id, actor=actor, recovered_at_ns=self._clock_ns()):
                raise RecoveryRejected("safety event changed before recovery")
            self._events.pop(resource, None)
            return event
