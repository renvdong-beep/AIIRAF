"""Skill manifest 加载、schema 校验与确定性 Provider 路由。"""
from dataclasses import dataclass
import hashlib,importlib,json,re
from pathlib import Path
import yaml
from jsonschema import Draft202012Validator
from packaging.specifiers import SpecifierSet,InvalidSpecifier
from packaging.version import Version,InvalidVersion

class RegistryError(ValueError): pass
@dataclass(frozen=True)
class ProviderSpec:
    name:str
    type:str
    entrypoint:str
    selector:dict
    priority:int
@dataclass(frozen=True)
class SkillManifest:
    name:str
    version:str
    digest:str
    verification:str
    requires:tuple
    preconditions:tuple
    intent_examples:tuple
    timeout_seconds:int
    cancellation:str
    recovery:tuple
    safety_class:str
    input_schema:dict
    output_schema:dict
    providers:tuple
@dataclass(frozen=True)
class InvocationResult:
    output:dict
    provider_name:str
    provider_type:str

def _specifier(constraint):
    value=constraint.strip()
    if not value:
        return SpecifierSet()
    if value.startswith("^"):
        base=Version(value[1:])
        upper=Version("%d.0.0"%(base.major+1))
        return SpecifierSet(">=%s,<%s"%(base,upper))
    if value.startswith("~"):
        base=Version(value[1:])
        upper=Version("%d.%d.0"%(base.major,base.minor+1))
        return SpecifierSet(">=%s,<%s"%(base,upper))
    if re.fullmatch(r"\d+\.\d+\.\d+",value):
        return SpecifierSet("=="+value)
    try:
        return SpecifierSet(value)
    except InvalidSpecifier as exc:
        raise RegistryError("无效 Skill 版本约束: "+value) from exc

class RegisteredSkill:
    def __init__(self,manifest): self.manifest=manifest
    def validate_inputs(self,inputs):
        errors=sorted(Draft202012Validator(self.manifest.input_schema).iter_errors(inputs),key=lambda e:list(e.path))
        if errors: raise RegistryError("Skill 输入不符合 schema: "+errors[0].message)
    def invoke(self,profile,backend,inputs,lease):
        self.validate_inputs(inputs)
        mode="simulation" if profile.simulation else "hardware"
        candidates=[p for p in self.manifest.providers if not p.selector.get("mode") or p.selector.get("mode")==mode]
        if not candidates: raise RegistryError("没有匹配运行模式的 Provider")
        provider_spec=sorted(candidates,key=lambda p:(p.priority,p.name))[0]
        if provider_spec.type!="python_adapter": raise RegistryError("当前 Runtime 不支持 Provider 类型: "+provider_spec.type)
        module_name,class_name=provider_spec.entrypoint.split(":",1)
        provider_class=getattr(importlib.import_module(module_name),class_name)
        output=provider_class(profile,backend).execute(inputs,lease)
        errors=sorted(Draft202012Validator(self.manifest.output_schema).iter_errors(output),key=lambda e:list(e.path))
        if errors: raise RegistryError("Provider 输出不符合 schema: "+errors[0].message)
        return InvocationResult(output,provider_spec.name,provider_spec.type)

class SkillRegistry:
    def __init__(self): self._skills={}
    def load_directory(self,root):
        for path in sorted(Path(root).glob("*/skill.yaml")): self.register(load_skill_manifest(path))
        if not self._skills: raise RegistryError("未发现 Skill manifest")
        return self
    def register(self,manifest):
        key=(manifest.name,manifest.version)
        if key in self._skills: raise RegistryError("Skill 版本重复: "+str(key))
        self._skills[key]=RegisteredSkill(manifest)
    def resolve(self,name,version_constraint=""):
        try: specifier=_specifier(version_constraint)
        except RegistryError: return None
        candidates=[s for (n,v),s in self._skills.items() if n==name and specifier.contains(Version(v),prereleases=True)]
        return sorted(candidates,key=lambda s:Version(s.manifest.version),reverse=True)[0] if candidates else None
    def names(self): return tuple(sorted({name for name,version in self._skills}))
    def intent_contracts(self):
        return {name:{"inputSchema":self.resolve(name).manifest.input_schema,"examples":self.resolve(name).manifest.intent_examples,"safetyClass":self.resolve(name).manifest.safety_class} for name in self.names()}

def load_skill_manifest(path):
    path=Path(path)
    raw=path.read_bytes()
    data=yaml.safe_load(raw) or {}
    meta=data.get("metadata") or {}
    spec=data.get("spec") or {}
    if data.get("kind")!="Skill" or not meta.get("name") or not meta.get("version"):
        raise RegistryError("无效 Skill manifest: "+str(path))
    try: Version(str(meta["version"]))
    except InvalidVersion as exc: raise RegistryError("Skill 版本不是 SemVer: "+str(meta["version"])) from exc
    required=("inputSchema","outputSchema","requires","preconditions","intentExamples","timeoutSeconds","cancellation","recovery","safetyClass","providers")
    missing=[key for key in required if key not in spec]
    if missing: raise RegistryError("Skill manifest 缺少字段: "+str(missing))
    def schema(name): return json.loads((path.parent/spec[name]).read_text(encoding="utf-8"))
    providers=[]
    for item in spec["providers"]:
        for key in ("name","type","entrypoint"):
            if key not in item: raise RegistryError("Provider 缺少字段: "+key)
        providers.append(ProviderSpec(item["name"],item["type"],item["entrypoint"],item.get("selector") or {},int(item.get("priority",100))))
    manifest=SkillManifest(meta["name"],str(meta["version"]),hashlib.sha256(raw).hexdigest(),str(spec.get("verification","unverified")),tuple(spec["requires"]),tuple(spec["preconditions"]),tuple(spec["intentExamples"]),int(spec["timeoutSeconds"]),str(spec["cancellation"]),tuple(spec["recovery"]),str(spec["safetyClass"]),schema("inputSchema"),schema("outputSchema"),tuple(providers))
    if manifest.timeout_seconds<=0 or not manifest.providers: raise RegistryError("Skill timeout/providers 无效")
    return manifest
