import unittest
from concurrent import futures
from datetime import datetime,timezone,timedelta
from pathlib import Path
import grpc
from google.protobuf.timestamp_pb2 import Timestamp
from iraf.v1 import skill_pb2,runtime_pb2,runtime_pb2_grpc
from runtime_grpc import SkillRuntimeServicer,add_execute_get_servicer_to_server
from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile,SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
class Backend:
    def __init__(self,a): self.authority=a; self.calls=[]
    def runtime_inventory(self): return {"safety":{"estop":False},"mode":"simulation"}
    def move_joint(self,p,d,l): self.authority.validate(l); self.calls.append((p,d))
    def stop(self,l): self.authority.validate(l); self.calls.append(("stop",))
class GrpcRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.authority=ControlAuthorityManager(); cls.backend=Backend(cls.authority); cls.profile=RobotProfile("robot","1.0.0",True,100,("j1",),{"j1":(-1,1)},frozenset({"move_joint","stop"}),"development","profile-digest"); cls.safety=SafetyPolicy("lab","1.0.0","development",True,frozenset({"move_joint","stop"}),30000,"safety-digest"); cls.registry=SkillRegistry().load_directory(Path(__file__).resolve().parents[2]/"skills"); cls.runtime=SkillRuntime(cls.profile,cls.safety,cls.backend,cls.registry,cls.authority,SqliteExecutionStore(":memory:")); cls.server=grpc.server(futures.ThreadPoolExecutor(max_workers=4)); add_execute_get_servicer_to_server(SkillRuntimeServicer(cls.runtime,"token","agentos"),cls.server); cls.port=cls.server.add_insecure_port("127.0.0.1:0"); cls.server.start(); cls.channel=grpc.insecure_channel("127.0.0.1:%d"%cls.port); cls.stub=runtime_pb2_grpc.SkillRuntimeServiceStub(cls.channel)
    @classmethod
    def tearDownClass(cls): cls.channel.close(); cls.server.stop(0)
    def request(self):
        req=skill_pb2.ExecuteSkillRequest(request_id="request-1",idempotency_key="grpc-key-1"); req.goal.correlation_id="grpc-correlation-1"; req.goal.skill_name="move_joint"; req.goal.skill_version_constraint="1.0.0"; req.goal.inputs.update({"positions":{"j1":0.2},"duration_ms":10}); req.goal.robot_profile.name="robot"; req.goal.robot_profile.version="1.0.0"; req.goal.robot_profile.digest="profile-digest"; req.goal.safety_policy.name="lab"; req.goal.safety_policy.version="1.0.0"; req.goal.safety_policy.digest="safety-digest"; req.goal.labels["resource_id"]="robot"; req.goal.labels["controller"]="grpc-test"; req.goal.deadline.FromDatetime(datetime.now(timezone.utc)+timedelta(seconds=5)); return req
    def test_execute_and_get(self):
        feedback=list(self.stub.Execute(self.request(),metadata=(("authorization","Bearer token"),))); self.assertEqual(1,len(feedback)); self.assertEqual(skill_pb2.SKILL_STATE_SUCCEEDED,feedback[0].state); self.assertEqual(1,len(self.backend.calls)); snapshot=self.stub.GetExecution(skill_pb2.GetExecutionRequest(execution_id=feedback[0].execution_id),metadata=(("authorization","Bearer token"),)); self.assertEqual(skill_pb2.SKILL_STATE_SUCCEEDED,snapshot.state); self.assertEqual(3,snapshot.last_sequence)
    def test_authentication_required(self):
        with self.assertRaises(grpc.RpcError) as cm: list(self.stub.Execute(self.request()))
        self.assertEqual(grpc.StatusCode.UNAUTHENTICATED,cm.exception.code())
    def test_cancel_not_registered(self):
        with self.assertRaises(grpc.RpcError) as cm: self.stub.Cancel(skill_pb2.CancelExecutionRequest(execution_id="none"),metadata=(("authorization","Bearer token"),))
        self.assertEqual(grpc.StatusCode.UNIMPLEMENTED,cm.exception.code())
if __name__=="__main__": unittest.main()