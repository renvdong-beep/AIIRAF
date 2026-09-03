"""从受控部署环境装配 IRAF 核心 Runtime。"""
import json
import os
from iraf_core.authority import ControlAuthorityManager
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
from .factory import load_backend

def build_runtime_from_env():
    required=("IRAF_PROFILE","IRAF_SAFETY_POLICY","IRAF_SKILL_ROOT","IRAF_BACKEND_ENTRYPOINT","IRAF_BACKEND_CONFIG","IRAF_EVENT_STORE")
    missing=[key for key in required if not os.environ.get(key)]
    if missing: raise RuntimeError("缺少 Runtime 配置: "+str(missing))
    profile=load_robot_profile(os.environ["IRAF_PROFILE"])
    safety=load_safety_policy(os.environ["IRAF_SAFETY_POLICY"])
    registry=SkillRegistry().load_directory(os.environ["IRAF_SKILL_ROOT"])
    authority=ControlAuthorityManager()
    backend=load_backend(os.environ["IRAF_BACKEND_ENTRYPOINT"],json.loads(os.environ["IRAF_BACKEND_CONFIG"]),profile,authority)
    store=SqliteExecutionStore(os.environ["IRAF_EVENT_STORE"])
    return SkillRuntime(profile,safety,backend,registry,authority,store)