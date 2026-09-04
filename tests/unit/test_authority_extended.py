import unittest

from iraf_core.authority import ControlAuthorityManager, LeaseConflict


class AuthorityExtendedTests(unittest.TestCase):
    def test_conflict_and_release_are_enforced(self):
        authority = ControlAuthorityManager(clock_ns=lambda: 0)
        lease = authority.acquire("arm", "task-a", ttl_seconds=2)
        with self.assertRaises(LeaseConflict):
            authority.acquire("arm", "task-b", ttl_seconds=2)
        authority.release(lease)
        replacement = authority.acquire("arm", "task-b", ttl_seconds=2)
        self.assertGreater(replacement.fencing_token, lease.fencing_token)

    def test_expired_lease_cannot_be_released_or_reused(self):
        now = [0]
        authority = ControlAuthorityManager(clock_ns=lambda: now[0])
        lease = authority.acquire("arm", "task-a", ttl_seconds=1)
        now[0] = 1_000_000_000
        with self.assertRaises(LeaseConflict):
            authority.release(lease)
        replacement = authority.acquire("arm", "task-b", ttl_seconds=1)
        self.assertNotEqual(replacement.owner, lease.owner)


if __name__ == "__main__":
    unittest.main()
