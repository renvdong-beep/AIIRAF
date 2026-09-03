import threading
import time
import unittest
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore

class BlockingBackend:
    def __init__(self, authority, fail_stop=False):
        self.authority = authority
        self.fail_stop = fail_stop
        self.started = threading.Event()
        self.release = threading.Event()
        self.stop_calls = 0
    def runtime_inventory(self): return {"safety": {"estop": False}, "mode": "simulation"}
    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        self.started.set()
        if not self.release.wait(timeout=3): raise TimeoutError("test backend was not released")
    def stop(self, lease):
        self.authority.validate(lease)
        self.stop_calls += 1
        self.release.set()
        if self.fail_stop: raise RuntimeError("controller did not acknowledge safe stop")

class CancellationSafetyTest(unittest.TestCase):
    def build(self, fail_stop=False):
        authority = ControlAuthorityManager()
        profile = RobotProfile("robot", "1.0.0", True, 100, ("j1",), {"j1": (-1, 1)}, frozenset({"move_joint", "stop"}), "development", "profile-digest")
        safety = SafetyPolicy("lab", "1.0.0", "development", True, frozenset({"move_joint", "stop"}), 30000, "safety-digest")
        registry = SkillRegistry().load_directory(Path(__file__).resolve().parents[2] / "skills")
        backend = BlockingBackend(authority, fail_stop)
        runtime = SkillRuntime(profile, safety, backend, registry, authority, SqliteExecutionStore(":memory:"))
        return runtime, backend

    def task(self, key):
        return {"request_id": "r-" + key, "correlation_id": "c-" + key, "idempotency_key": key, "skill": "move_joint", "skill_version_constraint": "1.0.0", "parameters": {"positions": {"j1": 0.2}, "duration_ms": 1000}, "deadline_unix_ms": int(time.time() * 1000) + 5000, "profile_name": "robot", "profile_version": "1.0.0", "profile_digest": "profile-digest", "safety_policy_name": "lab", "safety_policy_version": "1.0.0", "safety_policy_digest": "safety-digest"}

    def run_async(self, runtime, backend, owner, key):
        execution_id = runtime.execution_id_for(owner.subject, key)
        holder = {}
        thread = threading.Thread(target=lambda: holder.setdefault("result", runtime.execute(self.task(key), owner, execution_id=execution_id)), daemon=True)
        thread.start()
        self.assertTrue(backend.started.wait(timeout=1))
        return execution_id, holder, thread

    def test_non_owner_cannot_cancel(self):
        runtime, backend = self.build()
        owner = AuthenticatedContext("owner", frozenset({"task.submit", "task.cancel"}), "local")
        stranger = AuthenticatedContext("stranger", frozenset({"task.cancel"}), "local")
        execution_id, holder, thread = self.run_async(runtime, backend, owner, "owner-check")
        denied = runtime.cancel(execution_id, stranger, "not mine")
        self.assertFalse(denied["accepted"])
        self.assertEqual("IRAF-POLICY-DENIED", denied["error_code"])
        self.assertEqual(0, backend.stop_calls)
        self.assertTrue(runtime.cancel(execution_id, owner, "cleanup")["accepted"])
        thread.join(timeout=2)
        self.assertEqual("CANCELLED", holder["result"]["status"])

    def test_stop_failure_is_safety_stop(self):
        runtime, backend = self.build(fail_stop=True)
        owner = AuthenticatedContext("owner", frozenset({"task.submit", "task.cancel"}), "local")
        execution_id, holder, thread = self.run_async(runtime, backend, owner, "stop-failure")
        response = runtime.cancel(execution_id, owner, "operator requested")
        self.assertFalse(response["accepted"])
        self.assertEqual("IRAF-CANCEL-STOP-FAILED", response["error_code"])
        thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual("SAFETY_STOP", holder["result"]["status"])
        self.assertEqual("IRAF-CANCEL-STOP-FAILED", holder["result"]["error_code"])
        self.assertNotEqual("SUCCEEDED", runtime.get(execution_id)["status"])

if __name__ == "__main__": unittest.main()
