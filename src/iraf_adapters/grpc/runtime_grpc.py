"""Canonical SkillRuntimeService gRPC adapter."""
import threading
import time
import grpc
from google.protobuf.json_format import MessageToDict
from google.protobuf.timestamp_pb2 import Timestamp
from iraf.v1 import skill_pb2, runtime_pb2_grpc

_STATUS = {"PENDING": "SKILL_STATE_PENDING", "VALIDATING": "SKILL_STATE_VALIDATING", "RUNNING": "SKILL_STATE_RUNNING", "SUCCEEDED": "SKILL_STATE_SUCCEEDED", "FAILED": "SKILL_STATE_FAILED", "ABORTED": "SKILL_STATE_ABORTED", "CANCELLED": "SKILL_STATE_CANCELLED", "SAFETY_STOP": "SKILL_STATE_SAFETY_STOP"}

def _state(value):
    return getattr(skill_pb2, _STATUS.get(value, "SKILL_STATE_UNSPECIFIED"))

def _context_from_grpc(context, token, subject):
    metadata = dict(context.invocation_metadata())
    if not token or metadata.get("authorization", "") != "Bearer " + token:
        context.abort(grpc.StatusCode.UNAUTHENTICATED, "valid Bearer identity required")
    from iraf_core.policy import AuthenticatedContext
    return AuthenticatedContext(subject, frozenset({"task.submit", "task.read", "task.cancel"}), "bearer")

def _timestamp_ms(value):
    return value.seconds * 1000 + value.nanos // 1_000_000

def task_dict(request):
    goal = request.goal
    labels = dict(goal.labels)
    return {"request_id": request.request_id, "idempotency_key": request.idempotency_key, "correlation_id": goal.correlation_id, "skill": goal.skill_name, "skill_version_constraint": goal.skill_version_constraint, "parameters": MessageToDict(goal.inputs, preserving_proto_field_name=True), "deadline_unix_ms": _timestamp_ms(goal.deadline), "profile_name": goal.robot_profile.name, "profile_version": goal.robot_profile.version, "profile_digest": goal.robot_profile.digest, "safety_policy_name": goal.safety_policy.name, "safety_policy_version": goal.safety_policy.version, "safety_policy_digest": goal.safety_policy.digest, "resource_id": labels.get("resource_id", "") or None, "controller": labels.get("controller", "agentos")}

def feedback(result, phase="terminal"):
    status = result.get("status", "")
    message = skill_pb2.SkillFeedback(execution_id=result.get("execution_id", ""), sequence=int(result.get("sequence", 0)), state=_state(status), progress=1.0 if status == "SUCCEEDED" else 0.0, phase=phase)
    message.occurred_at.GetCurrentTime()
    if result.get("result"):
        message.outputs.update(result["result"])
    if result.get("error_code"):
        message.error.code = result["error_code"]
        message.error.message_zh = result.get("reason", "")
        message.error.retryable = False
    return message

class SkillRuntimeServicer(runtime_pb2_grpc.SkillRuntimeServiceServicer):
    def __init__(self, runtime, token, subject="iraf-grpc"):
        self.runtime = runtime
        self.token = token
        self.subject = subject

    def Execute(self, request, context):
        auth = self._auth_submit(context)
        task = task_dict(request)
        execution_id = self.runtime.execution_id_for(auth.subject, task.get("idempotency_key", ""))
        ready = threading.Event()
        done = threading.Event()
        holder = {}
        def run():
            try:
                holder["result"] = self.runtime.execute(task, auth, execution_id=execution_id, ready_event=ready)
            except Exception as exc:
                holder["error"] = exc
            finally:
                done.set()
        threading.Thread(target=run, name="iraf-execute-" + execution_id[:8], daemon=True).start()
        ready.wait(timeout=1.0)
        if done.is_set():
            if "error" in holder: context.abort(grpc.StatusCode.INTERNAL, "IRAF Runtime execution failed: " + str(holder["error"]))
            yield feedback(holder["result"])
            return
        snapshot = self.runtime.get(execution_id) or {"execution_id": execution_id, "status": "PENDING", "sequence": 0}
        yield feedback(snapshot, phase="accepted")
        while not done.wait(timeout=0.05):
            if not context.is_active():
                self.runtime.cancel(execution_id, auth, "gRPC client disconnected")
                return
        if "error" in holder:
            context.abort(grpc.StatusCode.INTERNAL, "IRAF Runtime execution failed: " + str(holder["error"]))
        yield feedback(holder["result"])

    def GetExecution(self, request, context):
        self._auth_read(context)
        result = self.runtime.get(request.execution_id)
        if not result:
            context.abort(grpc.StatusCode.NOT_FOUND, "execution not found")
        snapshot = skill_pb2.ExecutionSnapshot(execution_id=result["execution_id"], state=_state(result["status"]), last_sequence=int(result.get("sequence", 0)))
        skill = result.get("skill") or {}
        provider = result.get("provider") or {}
        snapshot.skill.name = skill.get("name", "")
        snapshot.skill.version = skill.get("version", "")
        snapshot.skill.digest = skill.get("digest", "")
        snapshot.provider.name = provider.get("name", "")
        snapshot.provider.version = provider.get("version", "")
        if result.get("error_code"):
            snapshot.error.code = result["error_code"]
            snapshot.error.message_zh = result.get("reason", "")
        return snapshot

    def Cancel(self, request, context):
        auth = self._auth_cancel(context)
        result = self.runtime.cancel(request.execution_id, auth, request.reason)
        return skill_pb2.CancelExecutionResponse(accepted=bool(result.get("accepted")), state=_state(result.get("status", "")))

    def _auth_submit(self, context): return _context_from_grpc(context, self.token, self.subject)
    def _auth_read(self, context): return _context_from_grpc(context, self.token, self.subject)
    def _auth_cancel(self, context): return _context_from_grpc(context, self.token, self.subject)

def add_execute_get_servicer_to_server(servicer, server):
    handlers = {
        "Execute": grpc.unary_stream_rpc_method_handler(servicer.Execute, request_deserializer=skill_pb2.ExecuteSkillRequest.FromString, response_serializer=skill_pb2.SkillFeedback.SerializeToString),
        "Cancel": grpc.unary_unary_rpc_method_handler(servicer.Cancel, request_deserializer=skill_pb2.CancelExecutionRequest.FromString, response_serializer=skill_pb2.CancelExecutionResponse.SerializeToString),
        "GetExecution": grpc.unary_unary_rpc_method_handler(servicer.GetExecution, request_deserializer=skill_pb2.GetExecutionRequest.FromString, response_serializer=skill_pb2.ExecutionSnapshot.SerializeToString),
    }
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("iraf.v1.SkillRuntimeService", handlers),))
