"""从受控部署环境装配 IRAF 核心 Runtime。"""
import json
import os
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
from .factory import load_backend


def _version_dir() -> Path:
    """版本目录（装配单里相对路径的基准）：IRAF_VERSION_DIR，默认 current 链接。"""
    return Path(os.environ.get("IRAF_VERSION_DIR") or "/opt/iraf-sdk/current")


def resolve_path(value: str) -> str:
    """把装配单里的**相对路径**按版本目录解析成绝对路径。

    实机缺陷（2026-10-22，E300 智控域，0.3.10 装完自检）：
    装配单写 `IRAF_PROFILE=profiles/piper_mujoco.yaml`，而调用方 cwd 不是版本目录 ⇒
    `FileNotFoundError: 'profiles/piper_mujoco.yaml'`，`iraf-agentos --list-skills` 直接失败。
    相对路径依赖 cwd 属隐式假设（AGENTS 6.2）；systemd 单元靠 WorkingDirectory= 表达基准，
    但 CLI / MCP / 探针各自的 cwd 不一定相同，所以基准在这里显式给。
    解析后仍不存在则**原样返回**，让下游报出装配单里的原始值（不伪造、不吞错）。
    """
    if not value:
        return value
    path = Path(value)
    if path.is_absolute():
        return value
    candidate = _version_dir() / path
    return str(candidate) if candidate.exists() else value


def build_runtime_from_env():
    required=("IRAF_PROFILE","IRAF_SAFETY_POLICY","IRAF_SKILL_ROOT","IRAF_BACKEND_ENTRYPOINT","IRAF_BACKEND_CONFIG","IRAF_EVENT_STORE")
    missing=[key for key in required if not os.environ.get(key)]
    if missing: raise RuntimeError("缺少 Runtime 配置: "+str(missing))
    profile=load_robot_profile(resolve_path(os.environ["IRAF_PROFILE"]))
    safety=load_safety_policy(resolve_path(os.environ["IRAF_SAFETY_POLICY"]))
    registry=SkillRegistry().load_directory(resolve_path(os.environ["IRAF_SKILL_ROOT"]))
    authority=ControlAuthorityManager()
    backend=load_backend(os.environ["IRAF_BACKEND_ENTRYPOINT"],json.loads(os.environ["IRAF_BACKEND_CONFIG"]),profile,authority)
    store=SqliteExecutionStore(resolve_path(os.environ["IRAF_EVENT_STORE"]))
    return SkillRuntime(profile,safety,backend,registry,authority,store)