"""Process-local resource leases with monotonic expiry and fencing tokens."""
from dataclasses import dataclass
import time

class LeaseConflict(RuntimeError):
    pass

@dataclass(frozen=True)
class Lease:
    resource: str
    owner: str
    fencing_token: int
    expires_at_ns: int

class ControlAuthorityManager:
    """Single-process lease authority; durable fencing belongs to the RTOS/controller."""
    def __init__(self, clock_ns=None):
        self._leases = {}
        self._next_token = 0
        self._clock_ns = clock_ns or time.monotonic_ns

    def acquire(self, resource, owner, ttl_seconds=30.0):
        if not resource or not owner:
            raise ValueError("resource and owner are required")
        ttl_ns = int(float(ttl_seconds) * 1_000_000_000)
        if ttl_ns <= 0:
            raise ValueError("lease ttl must be positive")
        current = self._clock_ns()
        existing = self._leases.get(resource)
        if existing is not None and existing.expires_at_ns <= current:
            del self._leases[resource]
            existing = None
        if existing is not None:
            raise LeaseConflict(f"resource busy: {resource}")
        self._next_token += 1
        lease = Lease(resource, owner, self._next_token, current + ttl_ns)
        self._leases[resource] = lease
        return lease

    def validate(self, lease):
        current = self._clock_ns()
        if self._leases.get(lease.resource) != lease:
            raise LeaseConflict("invalid fencing token")
        if lease.expires_at_ns <= current:
            del self._leases[lease.resource]
            raise LeaseConflict("lease expired")

    def release(self, lease):
        self.validate(lease)
        del self._leases[lease.resource]
