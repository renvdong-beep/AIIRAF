import time,unittest
from pathlib import Path
from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile,SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
from dispatcher import TaskDispatcher
from bridge import AgentOSBridge
from openai_intent import IntentProviderError
class Backend:
    def __init__(self,a): self.authority=a; self.stopped=False
    def runtime_inventory(self): return {"safety":{"estop":False},"mode":"simulation"}
    def stop(self,lease): self.authority.validate(lease); self.stopped=True
    def move_joint(self,p,d,l): self.authority.validate(l)
class Provider:
    name="fake"; version="1.0.0"; model_id="fake-model"
    def __init__(self,fail=False): self.fail=fail
    def parse(self,text,allowed):
        self.contracts = allowed
        if self.fail: raise IntentProviderError("bad output")
        return {"skill":"stop","parameters":{}}
class BridgeTest(unittest.TestCase):
    def setUp(self):
        a=ControlAuthorityManager(); self.backend=Backend(a); p=RobotProfile("robot","1.0.0",True,100,("j1",),{"j1":(-1,1)},frozenset({"move_joint","stop"}),"development","pd"); s=SafetyPolicy("lab","1.0.0","development",True,frozenset({"move_joint","stop"}),30000,"sd"); r=SkillRegistry().load_directory(Path(__file__).resolve().parents[2]/"skills"); self.dispatcher=TaskDispatcher(SkillRuntime(p,s,self.backend,r,a,SqliteExecutionStore(":memory:"))); self.context=AuthenticatedContext("agentos",frozenset({"task.submit"}),"bearer")
    def request(self): return {"request_id":"r1","correlation_id":"c1","idempotency_key":"k1","text":"停止机器人","deadline_unix_ms":int(time.time()*1000)+5000}
    def test_bridge_only_maps_then_runtime_executes(self):
        provider=Provider()
        result=AgentOSBridge(provider,self.dispatcher).execute_intent(self.request(),self.context); self.assertEqual("SUCCEEDED",result["status"]); self.assertTrue(self.backend.stopped); self.assertEqual("fake-model",result["intent_provider"]["model"])
        self.assertEqual({"j1"}, set(provider.contracts["move_joint"]["inputSchema"]["properties"]["positions"]["properties"]))
    def test_model_failure_never_calls_backend(self):
        result=AgentOSBridge(Provider(True),self.dispatcher).execute_intent(self.request(),self.context); self.assertEqual("IRAF-INTENT-PARSE-FAILED",result["error_code"]); self.assertFalse(self.backend.stopped)
if __name__=="__main__": unittest.main()