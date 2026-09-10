import unittest

from iraf_core.controller_gate import ControllerGateError, MotionPermit, RtosMotionGate


class ControllerGateTests(unittest.TestCase):
    def setUp(self):
        self.now = [1_000]
        self.calls = []

        def provider(resource, fencing_token):
            self.calls.append((resource, fencing_token))
            return MotionPermit(resource, fencing_token, self.now[0] + 100)

        self.gate = RtosMotionGate(provider, heartbeat_timeout_ms=1, clock_ns=lambda: self.now[0])

    def test_requires_fresh_monotonic_heartbeat_and_matching_permit(self):
        self.gate.heartbeat("arm", 1)
        permit = self.gate.require_motion("arm", 7)

        self.assertEqual(MotionPermit("arm", 7, 1_100), permit)
        self.assertEqual([("arm", 7)], self.calls)

    def test_stale_or_repeated_heartbeat_fails_closed(self):
        self.gate.heartbeat("arm", 1)
        with self.assertRaisesRegex(ControllerGateError, "not increasing"):
            self.gate.heartbeat("arm", 1)
        self.now[0] = 1_001_001
        with self.assertRaisesRegex(ControllerGateError, "timeout"):
            self.gate.require_motion("arm", 7)
        self.assertEqual([], self.calls)

    def test_invalid_permit_is_rejected(self):
        self.gate.heartbeat("arm", 1)
        cases = (
            MotionPermit("other", 7, 1_100),
            MotionPermit("arm", 8, 1_100),
            MotionPermit("arm", 7, 999),
            {"resource": "arm", "fencing_token": 7},
        )
        for permit in cases:
            with self.subTest(permit=permit):
                gate = RtosMotionGate(lambda _resource, _token, permit=permit: permit, 1, lambda: self.now[0])
                with self.assertRaises(ControllerGateError):
                    gate.require_motion("arm", 7)

    def test_provider_failure_is_fail_closed(self):
        self.gate = RtosMotionGate(lambda _resource, _token: (_ for _ in ()).throw(OSError("link down")), 1, lambda: self.now[0])
        self.gate.heartbeat("arm", 1)
        with self.assertRaisesRegex(ControllerGateError, "unavailable"):
            self.gate.require_motion("arm", 7)


if __name__ == "__main__":
    unittest.main()
