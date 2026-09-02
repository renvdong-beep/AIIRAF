"""仅用于开发仿真的 HTTP transport adapter；领域执行位于 IRAF SkillRuntime。"""
import argparse,json,os
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from google.protobuf.json_format import MessageToDict,ParseDict
from iraf.v1.skill_pb2 import ExecuteSkillRequest
from v1.bridge_pb2 import IntentRequest
from iraf_core.policy import AuthenticatedContext
from bootstrap import build_runtime_from_env
from dispatcher import TaskDispatcher
from bridge import AgentOSBridge
from openai_intent import IntentProviderConfig,OpenAICompatibleIntentProvider

def timestamp_ms(value): return value.seconds*1000+value.nanos//1000000
def canonical_task(message):
    goal=message.goal; labels=dict(goal.labels)
    return {"request_id":message.request_id,"idempotency_key":message.idempotency_key,"correlation_id":goal.correlation_id,"skill":goal.skill_name,"skill_version_constraint":goal.skill_version_constraint,"parameters":MessageToDict(goal.inputs,preserving_proto_field_name=True),"deadline_unix_ms":timestamp_ms(goal.deadline),"profile_name":goal.robot_profile.name,"profile_version":goal.robot_profile.version,"profile_digest":goal.robot_profile.digest,"safety_policy_name":goal.safety_policy.name,"safety_policy_version":goal.safety_policy.version,"safety_policy_digest":goal.safety_policy.digest,"resource_id":labels.get("resource_id","") or None,"controller":labels.get("controller","agentos")}
def canonical_intent(message): return {"request_id":message.request_id,"idempotency_key":message.idempotency_key,"correlation_id":message.correlation_id,"text":message.text,"resource_id":message.resource_id,"controller":message.controller,"deadline_unix_ms":timestamp_ms(message.deadline)}

class RuntimeHandler(BaseHTTPRequestHandler):
    dispatcher=None; bridge=None; token=""; subject="agentos"
    def _json(self,status,body):
        encoded=json.dumps(body,ensure_ascii=True).encode(); self.send_response(status); self.send_header("Content-Type","application/json"); self.send_header("Content-Length",str(len(encoded))); self.end_headers(); self.wfile.write(encoded)
    def _context(self):
        supplied=self.headers.get("Authorization","")
        if self.token and supplied=="Bearer "+self.token: return AuthenticatedContext(self.subject,frozenset({"task.submit","task.read"}),"bearer")
        return None
    def do_GET(self):
        if self.path=="/health": self._json(200,{"status":"ok","service":"iraf-http-adapter","mode":"development-simulation","intent_enabled":self.bridge is not None}); return
        if self._context() is None: self._json(401,{"error":"unauthorized"}); return
        if self.path.startswith("/v1/tasks/"):
            result=self.dispatcher.get(self.path.rsplit("/",1)[-1]); self._json(200,result) if result else self._json(404,{"error":"task not found"}); return
        self._json(404,{"error":"not found"})
    def do_POST(self):
        context=self._context()
        if context is None: self._json(401,{"error":"unauthorized"}); return
        if self.path not in {"/v1/tasks","/v1/intents"}: self._json(404,{"error":"not found"}); return
        if self.path=="/v1/intents" and self.bridge is None: self._json(503,{"error":"intent provider unavailable"}); return
        try:
            raw=json.loads(self.rfile.read(int(self.headers.get("Content-Length","0"))))
            if self.path=="/v1/tasks": result=self.dispatcher.dispatch(canonical_task(ParseDict(raw,ExecuteSkillRequest(),ignore_unknown_fields=False)),context)
            else: result=self.bridge.execute_intent(canonical_intent(ParseDict(raw,IntentRequest(),ignore_unknown_fields=False)),context)
            self._json(200,result)
        except (ValueError,json.JSONDecodeError) as exc: self._json(400,{"status":"FAILED","error_code":"IRAF-INPUT-INVALID","reason":str(exc)})
        except Exception as exc: self._json(500,{"status":"FAILED","error_code":"IRAF-INTERNAL","reason":str(exc)})
    def log_message(self,fmt,*args): print("[iraf-http] "+fmt%args)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--host",default=os.environ.get("IRAF_RUNTIME_HOST","127.0.0.1")); parser.add_argument("--port",type=int,default=int(os.environ.get("IRAF_RUNTIME_PORT","8765"))); args=parser.parse_args()
    if os.environ.get("IRAF_HTTP_DEVELOPMENT","").lower()!="true": parser.error("HTTP adapter 仅允许显式 development 仿真模式")
    runtime=build_runtime_from_env(); dispatcher=TaskDispatcher(runtime); RuntimeHandler.dispatcher=dispatcher
    endpoint=os.environ.get("IRAF_AGENTOS_MODEL_URL",""); model=os.environ.get("IRAF_AGENTOS_MODEL","")
    if endpoint and model:
        verify=os.environ.get("IRAF_AGENTOS_VERIFY_TLS","true").lower() not in {"0","false","no"}; provider=OpenAICompatibleIntentProvider(IntentProviderConfig(endpoint,model,os.environ.get("IRAF_AGENTOS_TOKEN",""),float(os.environ.get("IRAF_AGENTOS_TIMEOUT_SECONDS","10")),verify)); RuntimeHandler.bridge=AgentOSBridge(provider,dispatcher)
    RuntimeHandler.token=os.environ.get("IRAF_RUNTIME_TOKEN",""); RuntimeHandler.subject=os.environ.get("IRAF_RUNTIME_SUBJECT","agentos"); server=ThreadingHTTPServer((args.host,args.port),RuntimeHandler); print("IRAF development HTTP adapter listening on %s:%s"%(args.host,args.port),flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()
if __name__=="__main__": main()