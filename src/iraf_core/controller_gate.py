"""Fail-closed RTOS motion permit and controller heartbeat gate."""
from dataclasses import dataclass
import threading
import time


class ControllerGateError(RuntimeError):
    """Raised when the RTOS controller cannot authorize motion safely."""


@dataclass(frozen=True)
class MotionPermit:
    resource: str
    fencing_token: int
    expires_at_ns: int


class RtosMotionGate:
    """Validate controller liveness and a fencing-bound, short-lived permit."""

    def __init__(self, permit_provider, heartbeat_timeout_ms=250, clock_ns=None):
        if not callable(permit_provider):
            raise ValueError("permit_provider must be callable")
        timeout_ns = int(heartbeat_timeout_ms) * 1_000_000
        if timeout_ns <= 0:
            raise ValueError("heartbeat_timeout_ms must be positive")
        self._permit_provider = permit_provider
        self._heartbeat_timeout_ns = timeout_ns
        self._clock_ns = clock_ns or time.monotonic_ns
        self._heartbeats = {}
        self._lock = threading.RLock()

    def heartbeat(self, resource, sequence):
        if not resource:
            raise ValueError("resource is required")
        sequence = int(sequence)
        if sequence < 0:
            raise ValueError("heartbeat sequence must be non-negative")
        with self._lock:
            previous = self._heartbeats.get(resource)
            if previous is not None and sequence <= previous[0]:
                raise ControllerGateError("heartbeat sequence is not increasing")
            self._heartbeats[resource] = (sequence, self._clock_ns())

    def require_motion(self, resource, fencing_token):
        now = self._clock_ns()
        with self._lock:
            heartbeat = self._heartbeats.get(resource)
        if heartbeat is None or now - heartbeat[1] > self._heartbeat_timeout_ns:
            raise ControllerGateError("controller heartbeat timeout")
        try:
            raw_permit = self._permit_provider(resource, int(fencing_token))
            permit = self._coerce_permit(raw_permit)
        except ControllerGateError:
            raise
        except Exception as exc:
            raise ControllerGateError(f"motion permit unavailable: {exc}") from exc
        if permit.resource != resource:
            raise ControllerGateError("motion permit resource mismatch")
        if permit.fencing_token != int(fencing_token):
            raise ControllerGateError("motion permit fencing token mismatch")
        if permit.expires_at_ns <= now:
            raise ControllerGateError("motion permit expired")
        return permit

    @staticmethod
    def _coerce_permit(value):
        if isinstance(value, MotionPermit):
            return value
        if not isinstance(value, dict):
            raise ControllerGateError("motion permit must be a mapping")
        required = {"resource", "fencing_token", "expires_at_ns"}
        if set(value) != required:
            raise ControllerGateError("motion permit fields are invalid")
        try:
            return MotionPermit(
                str(value["resource"]),
                int(value["fencing_token"]),
                int(value["expires_at_ns"]),
            )
        except (TypeError, ValueError) as exc:
            raise ControllerGateError("motion permit values are invalid") from exc
