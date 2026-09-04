import time
import unittest
from types import SimpleNamespace

from iraf_core.policy import AuthenticatedContext, PolicyGateway, PolicyRejected


class _Skill:
    manifest = SimpleNamespace(name="move_joint", requires=("arm_motion",), verification="development")

    def validate_inputs(self, inputs):
        return None


class PolicyBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.profile = SimpleNamespace(name="arm", version="1.0.0", digest="profile", simulation=True,
                                       verification="development", capabilities=frozenset({"arm_motion"}))
        self.policy = SimpleNamespace(name="lab", version="1.0.0", digest="policy", verification="development",
                                      simulation_only=True, allowed_skills=frozenset({"move_joint"}),
                                      max_duration_ms=1000)
        self.request = {"request_id": "r1", "idempotency_key": "i1", "correlation_id": "c1",
                        "deadline_unix_ms": int(time.time() * 1000) + 10000, "parameters": {},
                        "profile_name": "arm", "profile_version": "1.0.0", "profile_digest": "profile",
                        "safety_policy_name": "lab", "safety_policy_version": "1.0.0", "safety_policy_digest": "policy"}

    def test_rejects_missing_authenticated_context(self):
        with self.assertRaisesRegex(PolicyRejected, "受信传输身份"):
            PolicyGateway().validate(self.request, None, _Skill(), self.profile, self.policy, {})

    def test_rejects_missing_profile_capability(self):
        self.profile = SimpleNamespace(**{**self.profile.__dict__, "capabilities": frozenset()})
        context = AuthenticatedContext("tester", frozenset({"task.submit"}), "local")
        with self.assertRaisesRegex(PolicyRejected, "缺少能力"):
            PolicyGateway().validate(self.request, context, _Skill(), self.profile, self.policy, {})


if __name__ == "__main__":
    unittest.main()
