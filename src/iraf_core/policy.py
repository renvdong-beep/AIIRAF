"""所有 Provider 调用前唯一的策略准入点。"""
from dataclasses import dataclass
import time,uuid
from .registry import RegistryError
@dataclass(frozen=True)
class AuthenticatedContext:
    subject:str; roles:frozenset; transport:str; tenant:str=""; credential_id:str=""; authenticated_at_ms:int=0
@dataclass(frozen=True)
class PolicyDecision: decision_id:str; gateway_version:str; safety_policy_digest:str; rules:tuple; parameters:dict
class PolicyRejected(ValueError):
    def __init__(self,code,message): super().__init__(message); self.code=code
class PolicyGateway:
    version="1.0.0"
    def validate(self,request,context,skill,profile,safety_policy,runtime_inventory):
        manifest=skill.manifest
        if not context or not context.subject or context.transport not in {"bearer","local"}: raise PolicyRejected("IRAF-UNAUTHENTICATED","缺少受信传输身份")
        if "task.submit" not in context.roles: raise PolicyRejected("IRAF-POLICY-DENIED","调用方无任务提交权限")
        if not request.get("request_id") or not request.get("idempotency_key") or not request.get("correlation_id"): raise PolicyRejected("IRAF-INPUT-INVALID","request_id、correlation_id 和 idempotency_key 必填")
        if int(request.get("deadline_unix_ms",0))<=int(time.time()*1000): raise PolicyRejected("IRAF-DEADLINE-EXCEEDED","任务截止时间已过期")
        parameters=request.get("parameters") or {}
        if not isinstance(parameters,dict): raise PolicyRejected("IRAF-INPUT-INVALID","parameters 必须是对象")
        try: skill.validate_inputs(parameters)
        except RegistryError as exc: raise PolicyRejected("IRAF-INPUT-INVALID",str(exc)) from exc
        if request.get("profile_name")!=profile.name or str(request.get("profile_version",""))!=profile.version or str(request.get("profile_digest",""))!=profile.digest: raise PolicyRejected("IRAF-POLICY-DENIED","RobotProfile 名称、版本或摘要不匹配")
        if request.get("safety_policy_name")!=safety_policy.name or str(request.get("safety_policy_version",""))!=safety_policy.version or str(request.get("safety_policy_digest",""))!=safety_policy.digest: raise PolicyRejected("IRAF-POLICY-DENIED","SafetyPolicy 名称、版本或摘要不匹配")
        if any(value!="verified" for value in (profile.verification,manifest.verification,safety_policy.verification)) and not profile.simulation: raise PolicyRejected("IRAF-POLICY-DENIED","未验证配置只能用于仿真")
        if safety_policy.simulation_only and not profile.simulation: raise PolicyRejected("IRAF-POLICY-DENIED","仿真 SafetyPolicy 不得用于真机")
        if manifest.name not in safety_policy.allowed_skills: raise PolicyRejected("IRAF-POLICY-DENIED","SafetyPolicy 未允许该 Skill")
        missing=set(manifest.requires)-set(profile.capabilities)
        if missing: raise PolicyRejected("IRAF-SKILL-PROVIDER-UNAVAILABLE","RobotProfile 缺少能力: "+str(sorted(missing)))
        duration=int(parameters.get("duration_ms",0)); maximum=min(manifest.timeout_seconds*1000,safety_policy.max_duration_ms)
        if duration>maximum: raise PolicyRejected("IRAF-POLICY-DENIED","动作时长超过策略上限")
        self._check_preconditions(manifest.preconditions,runtime_inventory)
        return PolicyDecision(str(uuid.uuid4()),self.version,safety_policy.digest,("rbac","schema","profile","safety","capability","precondition"),parameters)
    def _check_preconditions(self,expressions,inventory):
        for expression in expressions:
            parts=expression.split()
            if len(parts)!=3 or parts[1] not in {"==","!="}: raise PolicyRejected("IRAF-PRECONDITION-FAILED","不支持的前置条件表达式")
            value=inventory
            for key in parts[0].split("."):
                if not isinstance(value,dict) or key not in value: raise PolicyRejected("IRAF-PRECONDITION-FAILED","缺少运行时状态: "+parts[0])
                value=value[key]
            literal={"true":True,"false":False}.get(parts[2],parts[2].strip('"')); passed=(value==literal) if parts[1]=="==" else (value!=literal)
            if not passed: raise PolicyRejected("IRAF-PRECONDITION-FAILED","前置条件不满足: "+expression)