import unittest

from iraf_core.authority import ControlAuthorityManager, LeaseConflict
from iraf_core.registry import _semver_key

class ArchitectureTest(unittest.TestCase):
    def test_lease_expires_using_monotonic_clock(self):
        now = [0]
        authority = ControlAuthorityManager(clock_ns=lambda: now[0])
        lease = authority.acquire("arm", "test", ttl_seconds=1)
        authority.validate(lease)
        now[0] = 1_000_000_000
        with self.assertRaises(LeaseConflict):
            authority.validate(lease)
        replacement = authority.acquire("arm", "replacement", ttl_seconds=1)
        self.assertGreater(replacement.fencing_token, lease.fencing_token)

    def test_skill_versions_use_numeric_semver_order(self):
        self.assertGreater(_semver_key("1.10.0"), _semver_key("1.9.0"))

    def test_canonical_packages_are_importable(self):
        import iraf_adapters
        import iraf_skills
        self.assertIsNotNone(iraf_adapters)
        self.assertIsNotNone(iraf_skills)

if __name__ == "__main__":
    unittest.main()
