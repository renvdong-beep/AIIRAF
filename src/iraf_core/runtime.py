"""IRAF SkillRuntime：策略、Registry、资源租约和执行记录的唯一入口。"""
import hashlib,json,threading,uuid
from .core import TaskStatus
from .policy import PolicyGateway,PolicyRejected
from .taskflow import TaskFlow

class SkillRuntime:
    def __init__(self,profile,safety_policy,backend,registry,authority,store,policy=None,resource_id=None):
        self.profile=profile
        self.safety_policy=safety_policy
        self.backend=backend
        self.registry=registry
        self.authority=authority
        self.store=store
        self.policy=policy or PolicyGateway()
        self.resource_id=resource_id or profile.name
        self._lock=threading.RLock()

    def execute(self,request,context):
        if context is None or not getattr(context,"subject",""):
            return self._standalone_failure(request.get("correlation_id",""),"IRAF-UNAUTHENTICATED","缺少受信传输身份")
        digest=hashlib.sha256(json.dumps(request,sort_keys=True,separators=(",",":"),ensure_ascii=True).encode()).hexdigest()
        key=request.get("idempotency_key","")
        with self._lock:
            existing=self.store.find_idempotent(context.subject,key) if key else None
            if existing:
                if existing[0]==digest:
                    return existing[1]
                return self._standalone_failure(request.get("correlation_id",""),"IRAF-IDEMPOTENCY-CONFLICT","同一幂等键对应不同请求")
            result=self._execute_once(request,context)
            if key:
                self.store.save_result(context.subject,key,digest,result)
            return result

    def _execute_once(self,request,context):
        flow=TaskFlow(str(uuid.uuid4()))
        correlation=request.get("correlation_id","")
        self.store.append_event(flow.execution_id,flow.sequence,flow.status.value)
        flow.transition(TaskStatus.VALIDATING)
        self.store.append_event(flow.execution_id,flow.sequence,flow.status.value)
        skill=self.registry.resolve(request.get("skill"),request.get("skill_version_constraint",""))
        if skill is None:
            flow.transition(TaskStatus.FAILED,"Skill 未注册或版本不匹配")
            self.store.append_event(flow.execution_id,flow.sequence,flow.status.value,"Skill 未注册或版本不匹配")
            return self._result(flow,correlation,"IRAF-SKILL-PROVIDER-UNAVAILABLE","Skill 未注册或版本不匹配")
        inventory=self.backend.runtime_inventory() if hasattr(self.backend,"runtime_inventory") else {}
        try:
            decision=self.policy.validate(request,context,skill,self.profile,self.safety_policy,inventory)
        except PolicyRejected as exc:
            flow.transition(TaskStatus.FAILED,str(exc))
            self.store.append_event(flow.execution_id,flow.sequence,flow.status.value,str(exc))
            return self._result(flow,correlation,exc.code,str(exc))
        lease=None
        try:
            resource=request.get("resource_id",self.resource_id)
            controller=request.get("controller","default")
            lease=self.authority.acquire(resource,controller)
            flow.transition(TaskStatus.RUNNING)
            self.store.append_event(flow.execution_id,flow.sequence,flow.status.value)
            invoked=skill.invoke(self.profile,self.backend,decision.parameters,lease)
            flow.transition(TaskStatus.SUCCEEDED)
            self.store.append_event(flow.execution_id,flow.sequence,flow.status.value)
            result=self._result(flow,correlation)
            result.update({"result":invoked.output,"skill":{"name":skill.manifest.name,"version":skill.manifest.version,"digest":skill.manifest.digest},"provider":{"name":invoked.provider_name,"type":invoked.provider_type},"profile":{"name":self.profile.name,"version":self.profile.version,"digest":self.profile.digest},"safety_policy":{"name":self.safety_policy.name,"version":self.safety_policy.version,"digest":self.safety_policy.digest},"policy_decision_id":decision.decision_id,"policy_version":decision.gateway_version,"resource_id":resource,"controller":controller})
            return result
        except Exception as exc:
            if flow.status in {TaskStatus.VALIDATING,TaskStatus.RUNNING}:
                flow.transition(TaskStatus.FAILED,str(exc))
                self.store.append_event(flow.execution_id,flow.sequence,flow.status.value,str(exc))
            return self._result(flow,correlation,"IRAF-EXECUTION-FAILED",str(exc))
        finally:
            if lease is not None:
                self.authority.release(lease)

    def get(self,execution_id):
        return self.store.get(execution_id)

    def _standalone_failure(self,correlation,code,reason):
        flow=TaskFlow(str(uuid.uuid4()))
        flow.transition(TaskStatus.VALIDATING)
        flow.transition(TaskStatus.FAILED,reason)
        return self._result(flow,correlation,code,reason)

    @staticmethod
    def _result(flow,correlation,error_code="",reason=""):
        return {"execution_id":flow.execution_id,"correlation_id":correlation,"status":flow.status.value,"sequence":flow.sequence,"error_code":error_code,"reason":reason}
