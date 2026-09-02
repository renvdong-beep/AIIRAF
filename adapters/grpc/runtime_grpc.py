"""IRAF canonical SkillRuntimeService 的 gRPC 适配器。"""
import grpc
from google.protobuf.json_format import MessageToDict
from google.protobuf.struct_pb2 import Struct
from google.protobuf.timestamp_pb2 import Timestamp
from iraf.v1 import skill_pb2, runtime_pb2, runtime_pb2_grpc

_STATUS={"PENDING":"SKILL_STATE_PENDING","VALIDATING":"SKILL_STATE_VALIDATING","RUNNING":"SKILL_STATE_RUNNING","SUCCEEDED":"SKILL_STATE_SUCCEEDED","FAILED":"SKILL_STATE_FAILED","ABORTED":"SKILL_STATE_ABORTED","CANCELLED":"SKILL_STATE_CANCELLED","SAFETY_STOP":"SKILL_STATE_SAFETY_STOP"}

def _state(value): return getattr(skill_pb2,_STATUS.get(value,"SKILL_STATE_UNSPECIFIED"))
def _context_from_grpc(context,token,subject):
    metadata=dict(context.invocation_metadata()); supplied=metadata.get("authorization","")
    if not token or supplied!="Bearer "+token: context.abort(grpc.StatusCode.UNAUTHENTICATED,"需要有效 Bearer 身份")
    from iraf_core.policy import AuthenticatedContext
    return AuthenticatedContext(subject,frozenset({"task.submit","task.read"}),"bearer")
def _timestamp_ms(value): return value.seconds*1000+value.nanos//1000000

def task_dict(request):
    goal=request.goal; labels=dict(goal.labels)
    return {"request_id":request.request_id,"idempotency_key":request.idempotency_key,"correlation_id":goal.correlation_id,"skill":goal.skill_name,"skill_version_constraint":goal.skill_version_constraint,"parameters":MessageToDict(goal.inputs,preserving_proto_field_name=True),"deadline_unix_ms":_timestamp_ms(goal.deadline),"profile_name":goal.robot_profile.name,"profile_version":goal.robot_profile.version,"profile_digest":goal.robot_profile.digest,"safety_policy_name":goal.safety_policy.name,"safety_policy_version":goal.safety_policy.version,"safety_policy_digest":goal.safety_policy.digest,"resource_id":labels.get("resource_id","") or None,"controller":labels.get("controller","agentos")}

def feedback(result):
    message=skill_pb2.SkillFeedback(execution_id=result.get("execution_id",""),sequence=int(result.get("sequence",0)),state=_state(result.get("status","")),progress=1.0 if result.get("status")=="SUCCEEDED" else 0.0,phase="terminal")
    if result.get("result"): message.outputs.update(result["result"])
    if result.get("error_code"):
        message.error.code=result["error_code"]; message.error.message_zh=result.get("reason",""); message.error.retryable=False
    return message

class SkillRuntimeServicer(runtime_pb2_grpc.SkillRuntimeServiceServicer):
    def __init__(self,runtime,token,subject="iraf-grpc"):
        self.runtime=runtime; self.token=token; self.subject=subject
    def Execute(self,request,context):
        auth=self._auth_submit(context)
        try: result=self.runtime.execute(task_dict(request),auth)
        except Exception as exc: context.abort(grpc.StatusCode.INTERNAL,"IRAF Runtime 执行失败: "+str(exc))
        yield feedback(result)
    def GetExecution(self,request,context):
        self._auth_read(context)
        result=self.runtime.get(request.execution_id)
        if not result: context.abort(grpc.StatusCode.NOT_FOUND,"执行记录不存在")
        snapshot=skill_pb2.ExecutionSnapshot(execution_id=result["execution_id"],state=_state(result["status"]),last_sequence=int(result.get("sequence",0)))
        skill=result.get("skill") or {}; provider=result.get("provider") or {}
        snapshot.skill.name=skill.get("name",""); snapshot.skill.version=skill.get("version",""); snapshot.skill.digest=skill.get("digest","")
        snapshot.provider.name=provider.get("name",""); snapshot.provider.version=provider.get("version","")
        if result.get("error_code"): snapshot.error.code=result["error_code"]; snapshot.error.message_zh=result.get("reason","")
        return snapshot
    def Cancel(self,request,context):
        context.abort(grpc.StatusCode.UNIMPLEMENTED,"Cancel 尚未实现，未注册到生产服务")
    def _auth_submit(self,context): return _context_from_grpc(context,self.token,self.subject)
    def _auth_read(self,context): return _context_from_grpc(context,self.token,self.subject)

def add_execute_get_servicer_to_server(servicer,server):
    handlers={
        "Execute":grpc.unary_stream_rpc_method_handler(servicer.Execute,request_deserializer=skill_pb2.ExecuteSkillRequest.FromString,response_serializer=skill_pb2.SkillFeedback.SerializeToString),
        "GetExecution":grpc.unary_unary_rpc_method_handler(servicer.GetExecution,request_deserializer=skill_pb2.GetExecutionRequest.FromString,response_serializer=skill_pb2.ExecutionSnapshot.SerializeToString),
    }
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("iraf.v1.SkillRuntimeService",handlers),))