from dataclasses import dataclass

class LeaseConflict(RuntimeError):
    pass

@dataclass(frozen=True)
class Lease:
    resource: str
    owner: str
    fencing_token: int

class ControlAuthorityManager:
    def __init__(self):
        self._leases = {}
        self._next_token = 0

    def acquire(self, resource, owner):
        if resource in self._leases:
            raise LeaseConflict(f'resource busy: {resource}')
        self._next_token += 1
        lease = Lease(resource, owner, self._next_token)
        self._leases[resource] = lease
        return lease

    def validate(self, lease):
        if self._leases.get(lease.resource) != lease:
            raise LeaseConflict('invalid fencing token')

    def release(self, lease):
        self.validate(lease)
        del self._leases[lease.resource]
