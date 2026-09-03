import time
import tempfile
import unittest
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.safety import QuarantineError, RecoveryRejected, SafetyQuarantine, SafetyState
from iraf_core.store import SqliteExecutionStore

class Backend:
    def __init__(self, authority): self.authority = authority; self.calls = []
    def runtime_inventory(self): return {"safety": {"estop": False}, "mode": "simulation"}
    def move_joint(self, positions, duration_ms, lease): self.authority.validate(lease); self.calls.append("move")
    def stop(self, lease): self.authority.validate(lease); self.calls.append("stop")

class SafetyQuarantineTest(unittest.TestCase):
    def test_store_backed_quarantine_is_visible_across_runtime_instances(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "events.db")
            first = SafetyQuarantine(store=SqliteExecutionStore(path))
            second = SafetyQuarantine(store=SqliteExecutionStore(path))
            event = first.quarantine("arm", "controller lost heartbeat", "HEARTBEAT_TIMEOUT")
            self.assertEqual(SafetyState.QUARANTINED, second.state("arm"))
            with self.assertRaises(QuarantineError): second.require_operational("arm")
            self.assertEqual(event, second.event("arm"))
            self.assertEqual(event, second.recover("arm", event.event_id, True, "operator"))
            self.assertEqual(SafetyState.OPERATIONAL, first.state("arm"))

    def test_recovery_requires_matching_event_ack_and_actor(self):
        quarantine = SafetyQuarantine(clock_ns=lambda: 42)
        event = quarantine.quarantine("arm", "emergency stop", "ESTOP")
        self.assertEqual(SafetyState.QUARANTINED, quarantine.state("arm"))
        with self.assertRaises(RecoveryRejected): quarantine.recover("arm", event.event_id, False, "operator")
        with self.assertRaises(RecoveryRejected): quarantine.recover("arm", "wrong", True, "operator")
        self.assertEqual(event, quarantine.recover("arm", event.event_id, True, "operator"))
        self.assertEqual(SafetyState.OPERATIONAL, quarantine.state("arm"))

    def test_runtime_rejects_quarantined_resource_until_recovery(self):
        authority = ControlAuthorityManager()
        profile = RobotProfile("robot", "1.0.0", True, 100, ("j1",), {"j1": (-1, 1)}, frozenset({"move_joint", "stop"}), "development", "profile-digest")
        safety = SafetyPolicy("lab", "1.0.0", "development", True, frozenset({"move_joint", "stop"}), 30000, "safety-digest")
        backend = Backend(authority)
        registry = SkillRegistry().load_directory(Path(__file__).resolve().parents[2] / "skills")
        quarantine = SafetyQuarantine()
        runtime = SkillRuntime(profile, safety, backend, registry, authority, SqliteExecutionStore(":memory:"), safety=quarantine)
        context = AuthenticatedContext("operator", frozenset({"task.submit"}), "local")
        event = runtime.handle_safety_event("robot", "controller reported ESTOP", "ESTOP")
        request = {"request_id": "r", "correlation_id": "c", "idempotency_key": "q", "skill": "move_joint", "skill_version_constraint": "1.0.0", "parameters": {"positions": {"j1": 0.2}, "duration_ms": 10}, "deadline_unix_ms": int(time.time() * 1000) + 5000, "profile_name": "robot", "profile_version": "1.0.0", "profile_digest": "profile-digest", "safety_policy_name": "lab", "safety_policy_version": "1.0.0", "safety_policy_digest": "safety-digest"}
        blocked = runtime.execute(request, context)
        self.assertEqual("IRAF-SAFETY-QUARANTINED", blocked["error_code"])
        self.assertEqual([], backend.calls)
        runtime.recover_safety("robot", event.event_id, True, "operator")
        request["idempotency_key"] = "q-recovered"
        self.assertEqual("SUCCEEDED", runtime.execute(request, context)["status"])
        self.assertEqual(["move"], backend.calls)

if __name__ == "__main__": unittest.main()
