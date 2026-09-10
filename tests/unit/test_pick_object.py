import math
import time
import unittest
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore


class PickBackend:
    def __init__(self, authority, *, visible=True, result=None):
        self.authority = authority
        self.visible = visible
        self.result = result
        self.calls = []

    def runtime_inventory(self):
        return {
            "safety": {"estop": False},
            "manipulation": {"target_visible": self.visible},
            "mode": "simulation",
        }

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.authority.validate(lease)
        self.calls.append((target_id, grasp_pose, duration_ms))
        return self.result or {
            "target_id": target_id,
            "grasped": True,
            "confirmation": "contact",
            "evidence": {
                "target_body": target_id,
                "left_finger_body": "left_finger",
                "right_finger_body": "right_finger",
                "bilateral_contact": True,
            },
        }

    def stop(self, lease):
        self.authority.validate(lease)


class PickObjectTests(unittest.TestCase):
    def setUp(self):
        self.authority = ControlAuthorityManager()
        self.profile = RobotProfile(
            "piper",
            "1.0.0",
            True,
            100,
            ("joint1",),
            {"joint1": (-1.0, 1.0)},
            frozenset({"pick_object", "stop"}),
            "development",
            "profile-digest",
        )
        self.policy = SafetyPolicy(
            "lab",
            "1.0.0",
            "development",
            True,
            frozenset({"pick_object", "stop"}),
            30000,
            "policy-digest",
        )
        self.context = AuthenticatedContext(
            "tester", frozenset({"task.submit"}), "local"
        )
        self.registry = SkillRegistry().load_directory(
            Path(__file__).resolve().parents[2] / "skills"
        )

    @staticmethod
    def parameters():
        return {
            "target_id": "box_01",
            "grasp_pose": {
                "frame_id": "world",
                "position": {"x": 0.4, "y": 0.0, "z": 0.2},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "duration_ms": 100,
        }

    def request(self, **changes):
        request = {
            "request_id": "pick-request",
            "correlation_id": "pick-correlation",
            "idempotency_key": "pick-key",
            "skill": "pick_object",
            "skill_version_constraint": "1.0.0",
            "parameters": self.parameters(),
            "deadline_unix_ms": int(time.time() * 1000) + 5000,
            "profile_name": "piper",
            "profile_version": "1.0.0",
            "profile_digest": "profile-digest",
            "safety_policy_name": "lab",
            "safety_policy_version": "1.0.0",
            "safety_policy_digest": "policy-digest",
        }
        request.update(changes)
        return request

    def runtime(self, backend, profile=None):
        return SkillRuntime(
            profile or self.profile,
            self.policy,
            backend,
            self.registry,
            self.authority,
            SqliteExecutionStore(":memory:"),
        )

    def test_verified_backend_result_succeeds(self):
        backend = PickBackend(self.authority)
        result = self.runtime(backend).execute(self.request(), self.context)
        self.assertEqual("SUCCEEDED", result["status"])
        self.assertEqual("common_pick_sim", result["provider"]["name"])
        self.assertEqual("contact", result["result"]["confirmation"])
        self.assertTrue(result["result"]["evidence"]["bilateral_contact"])
        self.assertEqual(1, len(backend.calls))

    def test_invisible_target_fails_before_backend(self):
        backend = PickBackend(self.authority, visible=False)
        result = self.runtime(backend).execute(self.request(), self.context)
        self.assertEqual("IRAF-PRECONDITION-FAILED", result["error_code"])
        self.assertEqual([], backend.calls)

    def test_missing_profile_capability_is_rejected(self):
        backend = PickBackend(self.authority)
        profile = RobotProfile(
            "piper",
            "1.0.0",
            True,
            100,
            ("joint1",),
            {"joint1": (-1.0, 1.0)},
            frozenset({"stop"}),
            "development",
            "profile-digest",
        )
        result = self.runtime(backend, profile).execute(self.request(), self.context)
        self.assertEqual("IRAF-SKILL-PROVIDER-UNAVAILABLE", result["error_code"])
        self.assertEqual([], backend.calls)

    def test_backend_without_confirmation_fails_closed(self):
        backend = PickBackend(
            self.authority,
            result={
                "target_id": "box_01",
                "grasped": False,
                "confirmation": "contact",
            },
        )
        result = self.runtime(backend).execute(self.request(), self.context)
        self.assertEqual("FAILED", result["status"])
        self.assertIn("未确认目标已抓取", result["reason"])

    def test_non_finite_pose_fails_closed(self):
        backend = PickBackend(self.authority)
        parameters = self.parameters()
        parameters["grasp_pose"]["position"]["x"] = math.inf
        result = self.runtime(backend).execute(
            self.request(parameters=parameters), self.context
        )
        self.assertEqual("IRAF-EXECUTION-FAILED", result["error_code"])
        self.assertIn("有限数值", result["reason"])
        self.assertEqual([], backend.calls)


if __name__ == "__main__":
    unittest.main()
