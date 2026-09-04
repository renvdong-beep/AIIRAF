import unittest
from iraf_core.provider_router import ProviderCandidate, ProviderRouteError, ProviderRouter


class ProviderRouterTests(unittest.TestCase):
    def setUp(self):
        self.router = ProviderRouter([
            ProviderCandidate("vla", "vla", frozenset({"arm_motion"}), frozenset({"simulation"}), 20),
            ProviderCandidate("classical", "classical", frozenset({"arm_motion"}), frozenset({"simulation", "hardware"}), 10),
        ])

    def test_selects_lowest_priority_matching_provider(self):
        self.assertEqual(self.router.select("arm_motion", "simulation").name, "classical")
        self.assertEqual(self.router.select("arm_motion", "hardware").name, "classical")

    def test_rejects_missing_capability_or_type(self):
        with self.assertRaises(ProviderRouteError):
            self.router.select("navigation", "simulation")
        with self.assertRaises(ProviderRouteError):
            self.router.select("arm_motion", "simulation", "rl")


if __name__ == "__main__":
    unittest.main()