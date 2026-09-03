import time,unittest
from pathlib import Path
from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile,SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
class Backend:
    def __init__(self,authority,simulation=True): self.authority=authority; self.calls=[]; self.simulation=simulation
    def runtime_inventory(self): return {"safety":{"estop":False},"mode":"simulation" if self.simulation else "hardware"}
    def move_joint(self,positions,duration_ms,lease): self.authority.validate(lease); self.calls.append((positions,duration_ms))
    def stop(self,lease): self.authority.validate(lease); self.calls.append(("stop",))
class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.authority=ControlAuthorityManager(); self.profile=RobotProfile("robot","1.0.0",True,100,("j1",),{"j1":(-1,1)},frozenset({"move_joint","stop"}),"development","profile-digest"); self.safety=SafetyPolicy("lab","1.0.0","development",True,frozenset({"move_joint","stop"}),30000,"safety-digest"); self.backend=Backend(self.authority); self.registry=SkillRegistry().load_directory(Path(__file__).resolve().parents[2]/"skills"); self.store=SqliteExecutionStore(":memory:"); self.runtime=SkillRuntime(self.profile,self.safety,self.backend,self.registry,self.authority,self.store); self.context=AuthenticatedContext("test",frozenset({"task.submit"}),"local")
    def task(self,**changes):
        data={"request_id":"r1","correlation_id":"c1","idempotency_key":"k1","skill":"move_joint","skill_version_constraint":"1.0.0","parameters":{"positions":{"j1":0.2},"duration_ms":10},"deadline_unix_ms":int(time.time()*1000)+5000,"profile_name":"robot","profile_version":"1.0.0","profile_digest":"profile-digest","safety_policy_name":"lab","safety_policy_version":"1.0.0","safety_policy_digest":"safety-digest"}; data.update(changes); return data
    def test_manifest_driven_success(self):
        result=self.runtime.execute(self.task(),self.context); self.assertEqual("SUCCEEDED",result["status"]); self.assertEqual("common_motion_sim",result["provider"]["name"]); self.assertTrue(result["skill"]["digest"])
    def test_schema_rejection(self):
        result=self.runtime.execute(self.task(parameters={"positions":{}}),self.context); self.assertEqual("IRAF-INPUT-INVALID",result["error_code"]); self.assertEqual([],self.backend.calls); replay=self.store.get_replay_manifest(result["execution_id"]); self.assertEqual("move_joint",replay["requested_skill"]["name"]); self.assertTrue(replay["simulation"]); self.assertEqual("profile-digest",replay["profile"]["digest"])
    def test_joint_limit_rejection(self):
        result=self.runtime.execute(self.task(parameters={"positions":{"j1":2}}),self.context); self.assertEqual("FAILED",result["status"]); self.assertEqual([],self.backend.calls)
    def test_same_idempotency_returns_snapshot(self):
        request=self.task(); first=self.runtime.execute(request,self.context); second=self.runtime.execute(request,self.context); self.assertEqual(first["execution_id"],second["execution_id"]); self.assertEqual(first,self.runtime.get(first["execution_id"]))
    def test_idempotency_conflict(self):
        self.runtime.execute(self.task(),self.context); result=self.runtime.execute(self.task(parameters={"positions":{"j1":0.3},"duration_ms":10}),self.context); self.assertEqual("IRAF-IDEMPOTENCY-CONFLICT",result["error_code"])
    def test_lease_conflict_is_failure(self):
        held=self.authority.acquire("robot","other")
        try: result=self.runtime.execute(self.task(idempotency_key="lease"),self.context); self.assertEqual("FAILED",result["status"])
        finally: self.authority.release(held)
    def test_unverified_artifacts_rejected_for_hardware(self):
        profile=RobotProfile("robot","1.0.0",False,100,("j1",),{"j1":(-1,1)},frozenset({"move_joint"}),"development","profile-digest"); safety=SafetyPolicy("lab","1.0.0","development",False,frozenset({"move_joint"}),30000,"safety-digest"); runtime=SkillRuntime(profile,safety,Backend(self.authority,False),self.registry,self.authority,SqliteExecutionStore(":memory:")); result=runtime.execute(self.task(idempotency_key="hardware"),self.context); self.assertEqual("IRAF-POLICY-DENIED",result["error_code"])
if __name__=="__main__": unittest.main()
