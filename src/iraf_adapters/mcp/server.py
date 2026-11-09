#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IRAF × AgentOS 的**任务级** MCP server（stdio）。

层次（纪律，不是措辞）：
  AgentOS 的 agent / skills 是**平台侧**概念；IRAF 的 Skill Manager 管的是**机器人 Skill**
  （skills/<name>/skill.yaml 契约 + Policy + Capability Provider）。两者不是一回事，**不得互相注册**。
  所以本 server **只暴露任务级入口**，不暴露"逐个技能"的工具：
    · iraf_submit_task(text, deadline_seconds) → 走 IRAF TaskFlow；**技能选择与参数校验由 IRAF
      自己的 Skill Manager 完成**（调用方只给任务语义，不指定 skill、不直调 skill.execute）
    · iraf_get_task(execution_id)              → 读 IRAF 事件库（权威），返回该执行的事实

运行环境（由 `mcp add --env` 注入，或直接继承进程环境）：
  IRAF_ENV_FILE          默认 /etc/iraf/edge-intel.env（profile/策略/skill root/endpoint/event store）
  IRAF_PROVIDER_TOKEN    跨机 Capability Provider 的鉴权 token（必须）
  IRAF_AGENTOS_MODEL_URL / IRAF_AGENTOS_MODEL / IRAF_AGENTOS_TOKEN
                         IRAF 内部**受约束**意图解析用（把任务语义映射到契约内技能）
  IRAF_AGENTOS_KEY_FILE  模型网关 key 的文件路径（默认 /etc/iraf/agentos.key，600）
  PYTHONPATH             需含 <version>/lib 与 <version>/src
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

VERSION_DIR = Path(os.environ.get("IRAF_VERSION_DIR") or "/opt/iraf-sdk/current")


def _load_env_file() -> None:
    """把装配单里的键补进进程环境（不覆盖已存在的键）。"""
    path = Path(os.environ.get("IRAF_ENV_FILE") or "/etc/iraf/edge-intel.env")
    if not path.is_file():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def _load_key_file() -> None:
    if os.environ.get("IRAF_AGENTOS_TOKEN"):
        return
    key_file = Path(os.environ.get("IRAF_AGENTOS_KEY_FILE") or "/etc/iraf/agentos.key")
    if key_file.is_file():
        os.environ["IRAF_AGENTOS_TOKEN"] = key_file.read_text(encoding="utf-8").strip()


_load_env_file()
_load_key_file()
for _extra in (str(VERSION_DIR / "lib"), str(VERSION_DIR / "src")):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)


def _enter_version_dir() -> None:
    """把工作目录切到 <version dir>（装配单里的 profiles/、skills/ 都是**相对**路径）。

    实机踩到（2026-10-21，E300 智控域）：AgentOS 以 stdio 启动本 server 时 cwd 不是版本目录，
    `profiles/piper_mujoco.yaml` 解析失败 ⇒ `iraf_submit_task` 返回
    `[Errno 2] No such file or directory: 'profiles/piper_mujoco.yaml'`。
    systemd 单元用 `WorkingDirectory=<version dir>` 表达同一事实；这里复现它，避免
    "同一条装配单在单元里成立、在 MCP 里不成立"的静默分叉。
    """
    if VERSION_DIR.is_dir():
        os.chdir(VERSION_DIR)


_enter_version_dir()

from mcp.server.fastmcp import FastMCP  # noqa: E402

app = FastMCP("iraf-task")
_STATE = {}


def _principal():
    """取当前调用方身份（http 由中间件绑定；stdio 由部署环境给出）。

    身份只来自受信 transport（AGENTS 1.8）：工具参数里没有 caller/role/tenant，
    也不允许调用方自报。缺身份即失败，不提供“匿名默认用户”。
    """
    from iraf_adapters.mcp import auth

    current = auth.current_principal()
    if current is not None:
        return current
    if "stdio_principal" not in _STATE:
        _STATE["stdio_principal"] = auth.env_principal()
    return _STATE["stdio_principal"]


def _context(principal):
    """按 Principal 构造本次调用的 AuthenticatedContext（不缓存跨调用方）。"""
    from iraf_adapters.agentos.cli import build_context

    transport = "local" if principal.token_id == "local" else "bearer"
    return build_context(principal.subject, principal.scopes, transport)


def _auth_error(exc) -> str:
    return json.dumps({"status": "FAILED", "error_code": exc.code, "reason": exc.reason},
                      ensure_ascii=False)


def _bridge_for_resolved(resolved):
    """on 模式：用注册表解析出的 profile/endpoint 构造 bridge，按 endpoint 缓存。

    装配单（env）在阶段 B 不再写场景绑定；这里在**构建期**临时注入两个键，
    构建完成立即恢复，避免并发请求互相污染。
    """
    import threading

    key = "bridge:" + resolved.host.endpoint
    if key in _STATE:
        return _STATE[key]
    from iraf_adapters.agentos.bridge import AgentOSBridge
    from iraf_adapters.agentos.cli import build_intent_provider
    from iraf_adapters.agentos.dispatcher import TaskDispatcher
    from iraf_adapters.bootstrap import build_runtime_from_env

    provider = build_intent_provider()
    if provider is None:
        raise RuntimeError(
            "IRAF 侧意图 Provider 未配置（需要 IRAF_AGENTOS_MODEL_URL + IRAF_AGENTOS_MODEL）")
    lock = _STATE.setdefault("_env_lock", threading.Lock())
    with lock:
        saved = {k: os.environ.get(k) for k in ("IRAF_PROFILE", "IRAF_BACKEND_CONFIG")}
        try:
            os.environ["IRAF_PROFILE"] = resolved.scene.profile
            # 装配单里保留的是"基线配置"（token_file/timeout/subject 等），这里只合并 endpoint，
            # 不整体覆盖 —— 否则凭据配置丢失，Provider 鉴权直接失败。
            try:
                base = json.loads(saved.get("IRAF_BACKEND_CONFIG") or "{}")
            except Exception:
                base = {}
            base = dict(base) if isinstance(base, dict) else {}
            base["endpoint"] = resolved.host.endpoint
            os.environ["IRAF_BACKEND_CONFIG"] = json.dumps(base)
            dispatcher = TaskDispatcher(build_runtime_from_env())
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
    bridge = AgentOSBridge(provider, dispatcher)
    _STATE[key] = bridge
    return bridge


def _bridge():
    """惰性构建 TaskFlow 入口（bridge = 意图解析 + 任务分派），失败即抛。"""
    if "bridge" in _STATE:
        return _STATE["bridge"]
    from iraf_adapters.agentos.bridge import AgentOSBridge
    from iraf_adapters.agentos.cli import build_intent_provider
    from iraf_adapters.agentos.dispatcher import TaskDispatcher
    from iraf_adapters.bootstrap import build_runtime_from_env

    provider = build_intent_provider()
    if provider is None:
        raise RuntimeError(
            "IRAF 侧意图 Provider 未配置（需要 IRAF_AGENTOS_MODEL_URL + IRAF_AGENTOS_MODEL）")
    dispatcher = TaskDispatcher(build_runtime_from_env())
    bridge = AgentOSBridge(provider, dispatcher)
    _STATE["bridge"] = bridge
    return bridge


@app.tool()
def iraf_enter_robot_mode(scene_id: str = "") -> str:
    """进入「机器人开发模式」（用户说"开始机器人开发模式"时必须调用本工具）。

    零参数、名字即语义：模型不需要在 enum 里选 action，听到这句中文就调它。
    等价于 iraf_robot_mode(action="enter")。返回 mode / 机器人 Skill 清单 / profile /
    安全策略 / 后端端点 / simulation 标记 / next_steps（进入后照做的固定顺序）。
    调用后**只做盘点并报告**，不要为了验证链路先提交一条任务（会弄脏仿真场景）。
    """
    return iraf_robot_mode(action="enter", scene_id=scene_id)


@app.tool()
def iraf_exit_robot_mode() -> str:
    """退出「机器人开发模式」（用户说"结束/退出机器人开发模式"时调用）。等价于 action="exit"。"""
    return iraf_robot_mode(action="exit")


from iraf_adapters.mcp.hints import (  # noqa: E402  (内容与 server 解耦，便于单测)
    FILTERED_EXAMPLE as _FILTERED_EXAMPLE,
    HANDOFF_LAB_EXAMPLE as _HANDOFF_LAB_EXAMPLE,
    WORKED_EXAMPLE as _WORKED_EXAMPLE,
    enter_next_steps as _enter_next_steps,
    next_steps as _next_steps,
)


@app.resource("iraf://examples/pick_box_01")
def _resource_pick_example() -> str:
    """已验证的抓取案例（原话 / 技能 / 预期结果 / 核验方式 / 注意事项）。"""
    return json.dumps(_WORKED_EXAMPLE, ensure_ascii=False, indent=2)


@app.resource("iraf://examples/filtered_request")
def _resource_filtered_example() -> str:
    """被 IRAF 过滤掉的请求长什么样（平台据此回退到自己的处理）。"""
    return json.dumps(_FILTERED_EXAMPLE, ensure_ascii=False, indent=2)


@app.resource("iraf://examples/handoff_lab")
def _resource_handoff_lab_example() -> str:
    """handoff_lab 联合世界（Go2 + Piper + UR5e）演示入口与判据（在 x86 仿真上运行）。"""
    return json.dumps(_HANDOFF_LAB_EXAMPLE, ensure_ascii=False, indent=2)


@app.resource("iraf://skills/catalog")
def _resource_skill_catalog() -> str:
    """当前可用机器人 Skill 与意图示例（路由表；只读）。"""
    facts = _robot_facts()
    return json.dumps({"skills": facts.get("skills") or [],
                       "intents": facts.get("intents") or {},
                       "profile": facts.get("profile"),
                       "simulation": facts.get("simulation")}, ensure_ascii=False, indent=2)


# ------------------------------------------------------------ 场景注册表（A3）---
#: 会话默认场景（按 principal.subject）。与 robot mode 一样是"会话级提示"，不是安全边界。
_SESSION_SCENE: dict = {}
_SCENE_REGISTRY = None


def _registry_mode() -> str:
    """off（默认，旧行为）/ shadow（只解析并审计）/ on（解析 + 与 env 路由一致性校验）。"""
    mode = (os.environ.get("IRAF_SCENE_REGISTRY_MODE") or "off").strip().lower()
    # 未知取值一律回落到 off：宁可退回旧行为，也不要"半个开关"生效
    return mode if mode in ("off", "shadow", "on") else "off"


def _registry():
    global _SCENE_REGISTRY
    if _SCENE_REGISTRY is None:
        from iraf_core.scene_registry import SceneRegistry

        root = os.environ.get("IRAF_SCENE_ROOT") or "/etc/iraf"
        prof = os.environ.get("IRAF_PROFILES_ROOT") or "/opt/iraf-sdk/current/profiles"
        _SCENE_REGISTRY = SceneRegistry(root=root, profiles_root=prof)
    return _SCENE_REGISTRY


def _resolve_scene(principal, scene_id=None, resource_id=None):
    """按档位解析场景。返回 (resolved|None, error_json|None)；off 模式恒为 (None, None)。"""
    from iraf_adapters.mcp import auth
    from iraf_core.scene_registry import E_REGISTRY_INVALID, RegistryError

    mode = _registry_mode()
    if mode == "off":
        return None, None
    reg = _registry()
    try:
        reg.reload_if_changed()
        sid = ((scene_id or "").strip()
               or _SESSION_SCENE.get(getattr(principal, "subject", ""), "")
               or (os.environ.get("IRAF_DEFAULT_SCENE") or ""))
        resolved = reg.resolve(sid, resource_id=(resource_id or None))
    except RegistryError as exc:
        return None, json.dumps({"status": "FAILED", "error_code": exc.code,
                                 "reason": exc.detail}, ensure_ascii=False)
    except Exception as exc:  # 注册表本身坏掉也不能静默放过
        return None, json.dumps({"status": "FAILED", "error_code": E_REGISTRY_INVALID,
                                 "reason": "%s: %s" % (type(exc).__name__, exc)},
                                ensure_ascii=False)
    auth.audit("resolved_scene", principal, mode=mode, scene=resolved.scene.scene_id,
               resource=resolved.resource.id, host=resolved.host.id,
               endpoint=resolved.host.endpoint)
    if mode == "shadow":
        return resolved, None
    # on：注册表与 env 路由必须一致；真正的"注册表决定路由"在阶段 B 落地
    try:
        cfg = json.loads(os.environ.get("IRAF_BACKEND_CONFIG") or "{}")
    except Exception:
        cfg = {}
    env_ep = str(cfg.get("endpoint") or "")
    if env_ep and env_ep != resolved.host.endpoint:
        return None, json.dumps({
            "status": "FAILED", "error_code": "IRAF-REGISTRY-MISMATCH",
            "reason": "env endpoint %s != registry %s（scene=%s）"
                      % (env_ep, resolved.host.endpoint, resolved.scene.scene_id)},
            ensure_ascii=False)
    return resolved, None


@app.tool()
def iraf_robot_mode(action: str = "status", ttl_seconds: int = 1800,
                    scene_id: str = "") -> str:
    """机器人开发模式的开关与状态（语义触发：用户说"开始机器人开发模式"时调用）。

    action: enter = 打开（后续任务默认按机器人任务域处理）；exit = 关闭；status = 只看状态。
    中文触发短语：开始/进入机器人开发模式 → enter；结束/退出机器人开发模式 → exit；
                 机器人环境怎么样 / 有哪些机器人技能 → status。
    返回 mode / active / remaining_s / 机器人 Skill 清单 / profile / 安全策略 / 后端端点 /
    simulation 标记 / hint / **next_steps（进入后照做的固定顺序）**。
    **只读装配事实，不执行任何动作、不触发机器人运动。**

    什么时候用：这句话有歧义（"抓取/放置"既可能是平台动作也可能是机器人任务）时，先进入
    本模式，平台就知道后续请求要交给 IRAF 的 Skill Manager，而不是自己编或先做全链路自检。

    ⚠ **进入模式不等于要做冒烟提交**：下一个动作是把 skills / intents 报给用户、等真实需求。
    不要为了"验证链路"先提交一条任务 —— 仿真场景有状态，一次成功抓取会搬走目标，
    本次会话就没有干净场景可演示了（再抓必报 distance≈0.374435m）。链路验收用装机探针。
    """
    from iraf_adapters.mcp import auth
    from iraf_adapters.mcp import mode as mode_mod

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    act = (action or "status").strip().lower()
    st = mode_mod.state_for(principal.subject)
    auth.audit("tool_call", principal, tool="iraf_robot_mode", action=act)
    _scene_payload: dict = {}
    _resolved = None
    if (scene_id or "").strip() and act != "status":
        _resolved, _scene_err = _resolve_scene(principal, scene_id)
        if _scene_err:
            return _scene_err
        if _resolved is not None:  # off 模式下注册表不介入：忽略 scene_id，保持旧行为
            if act == "enter":
                _SESSION_SCENE[principal.subject] = _resolved.scene.scene_id
            _scene_payload = {"scene_id": _resolved.scene.scene_id,
                              "resource_id": _resolved.resource.id}

    if act == "enter":
        granted = st.enter(int(ttl_seconds))
        extra = {"entered": True, "ttl_s": granted}
    elif act == "exit":
        extra = {"entered": False, "was_active": st.exit()}
    elif act == "status":
        extra = {}
    else:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": "action 只接受 enter / exit / status：%s" % action},
                          ensure_ascii=False)
    payload: dict = {"status": "SUCCEEDED"}
    payload.update(extra)
    payload.update(_scene_payload)
    payload.update(st.snapshot())
    if _resolved is None and _registry_mode() == "on":
        _r, _e = _resolve_scene(principal, "")
        _resolved = _r
    payload.update(_robot_facts(_resolved))
    # 把"进入之后照做什么"直接写进返回：实机证据（2026-10-22）是模型自己设计了
    # "只读自检 + 冒烟提交"两段式，冒烟那次把仿真场景弄脏。规则写在返回值里比写在文档里有效。
    payload["next_steps"] = _enter_next_steps()
    # 自带"正确用法示范"：模型不必自己摸索该怎么调，也减少它去翻源码/重写实现的动机。
    # 注意：examples 是**语义映射说明**，不是验收脚本；purpose 字段已明示"不要求执行"。
    payload["examples"] = {"robot_task": _WORKED_EXAMPLE, "non_robot": _FILTERED_EXAMPLE}
    return json.dumps(payload, ensure_ascii=False)


def _robot_facts(resolved=None) -> dict:
    """把装配单里的事实读出来（只读；失败就给 unverified，不猜）。

    resolved 非空时（on 模式）：profile/endpoint 以注册表解析结果为准，
    因为装配单里已不再写场景绑定（阶段 B）。
    """
    import os as _os
    from pathlib import Path as _Path

    facts = {"skills": [], "profile": None, "safety_policy": None,
             "provider_endpoint": None, "simulation": None, "event_store": None}
    skill_root = _os.environ.get("IRAF_SKILL_ROOT") or ""
    if skill_root:
        try:
            from iraf_core.registry import SkillRegistry

            registry = SkillRegistry().load_directory(skill_root)
            facts["skills"] = list(registry.names())
            # 把每个技能的意图示例一并给出：平台拿到这张"路由表"才能判断"这句话该不该交给
            # IRAF"，而不是靠猜或干脆自己做（2026-10-21 使用者口径：机器人开发模式下优先走
            # IRAF，但 AgentOS 仍要自己判断项目与事务的处理）。
            intents = {}
            triggers = {}
            for name, contract in registry.intent_contracts().items():
                intents[name] = list(contract.get("examples") or [])
                # 技能选择收口（2026-10-23）：把"能不能用一句话触发、需要什么参数/场景"一起给平台，
                # 否则模型只看 examples，会把"抓取"错配到需要 vision_file 的 visual_pick。
                triggers[name] = dict(contract.get("trigger") or {"natural_language": True})
            facts["intents"] = intents
            facts["skill_triggers"] = triggers
        except Exception as exc:
            facts["skills_error"] = str(exc)[:200]
    facts["profile"] = (resolved.scene.profile if resolved is not None
                        else _os.environ.get("IRAF_PROFILE"))
    facts["safety_policy"] = _os.environ.get("IRAF_SAFETY_POLICY")
    facts["event_store"] = _os.environ.get("IRAF_EVENT_STORE")
    if resolved is not None:
        facts["provider_endpoint"] = resolved.host.endpoint
    else:
        try:
            raw = _os.environ.get("IRAF_BACKEND_CONFIG") or "{}"
            cfg = json.loads(raw)
            facts["provider_endpoint"] = cfg.get("endpoint")
        except Exception:
            pass
    policy_path = _os.environ.get("IRAF_SAFETY_POLICY") or ""
    if policy_path and _Path(policy_path).is_file():
        try:
            from iraf_core.profile import load_safety_policy

            policy = load_safety_policy(policy_path)
            facts["simulation"] = bool(policy.simulation_only)
            facts["allowed_skills"] = sorted(policy.allowed_skills)
        except Exception as exc:
            facts["simulation_error"] = str(exc)[:200]
    return facts


@app.prompt()
def robot_routing_policy() -> str:
    """机器人开发模式的分派规则（robot-first）：平台何时交给 IRAF、何时自己处理。"""
    from iraf_adapters.mcp.mode import ROUTING_POLICY

    return ("机器人开发模式的分派规则（robot-first，不是无脑转发）：\n" + ROUTING_POLICY +
            "\n技能清单与意图示例见 iraf_enter_robot_mode() 的返回（skills / intents）。")


@app.prompt()
def robot_dev_mode() -> str:
    """机器人开发模式（宿主可把它当 /命令 用）：进入模式 + 按任务提交，不要自编结果。"""
    return (
        "你在 IRAF 机器人开发模式下工作。请立刻调用 iraf_enter_robot_mode() 进入模式，"
        "然后用户给出的动作类需求一律作为**任务**调用 iraf_submit_task(text=...)，"
        "由 IRAF 的 Skill Manager 选技能；不要在平台侧自己执行或编造 status/execution_id，"
        "需要核对时用 iraf_get_task(execution_id)。"
    )


@app.tool()
def iraf_submit_task(text: str, deadline_seconds: int = 120,
                     scene_id: str = "", resource_id: str = "") -> str:
    """提交一条**任务**给 IRAF（自然语言语义）。技能选择由 IRAF Skill Manager 完成。

    ⚠ 这是机器人动作的**唯一受支持入口**：
      · 抓取/移动/放置/标定/视觉/停止这类请求，必须调用本工具，**不要**用 execute_code、
        terminal、host_command 或自己写脚本去驱动机器人（那样绕过 Skill Manager、策略与证据链）；
      · 不要自行"重新实现"抓取，也不要修改本仓源码去迁就一次失败；失败先按返回里的
        `next_steps` 处理（多数是场景状态或参数问题，不是实现 bug）。

    返回与 IRAF CLI 同构的 JSON：status / execution_id / error_code / reason / evidence /
    resolved_skill / robot_mode / next_steps。失败不会被包装成成功。
    """
    from iraf_adapters.agentos.cli import build_request
    from iraf_adapters.mcp import auth
    from iraf_adapters.mcp import mode as mode_mod

    try:
        principal = _principal()
        auth.require_scope(principal, "task.submit")
    except auth.AuthError as exc:
        return _auth_error(exc)
    _resolved, _scene_err = _resolve_scene(principal, scene_id, resource_id or None)
    if _scene_err:
        return _scene_err
    _lease = None
    if _resolved is not None:
        from iraf_core.leases import LeaseConflict

        try:
            _lease = _world_lease(
                _resolved.scene, _resolved.host, principal.subject,
                ttl_s=max(int(deadline_seconds) + 60, _resolved.scene.lease_ttl_s))
        except LeaseConflict as exc:
            return json.dumps({"status": "FAILED", "error_code": "IRAF-RESOURCE-BUSY",
                               "reason": str(exc), "lease": exc.lease.as_dict()},
                              ensure_ascii=False)
    try:
        bridge = _bridge() if _resolved is None else _bridge_for_resolved(_resolved)
        auth.require_resource(principal, bridge.dispatcher.resource_id)
        context = _context(principal)
    except auth.AuthError as exc:
        if _lease is not None:
            _lease_store().release(_lease.lease_id, "auth-error")
        return _auth_error(exc)
    try:
        request = build_request(text, deadline_seconds=int(deadline_seconds),
                                controller="agentos-mcp-task")
    except ValueError as exc:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": str(exc)}, ensure_ascii=False)
    _reject = _reject_foreign_host()
    if _reject:
        return _reject
    if _resolved is not None:
        _sw = _ensure_scene(_resolved.scene.switch_name or _resolved.scene.scene_id,
                            _resolved.host.endpoint, _resolved.host, _resolved.provider)
        if _sw.startswith(("switch-failed", "switch-error", "bad-endpoint", "no-endpoint")):
            return json.dumps({"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                               "reason": "Provider 启停失败（%s）：%s"
                                         % (_resolved.scene.scene_id, _sw)},
                              ensure_ascii=False)
    else:
        _scene = (os.environ.get("IRAF_LOCAL_SCENE") or "").strip()
        if _scene:
            _ensure_scene(_scene)
    try:
        result = bridge.execute_intent(request, context)
    except Exception:
        if _lease is not None:
            _lease_store().release(_lease.lease_id, "error")
        raise
    if _lease is not None:
        try:
            if isinstance(result, dict) and result.get("execution_id"):
                _lease_store().renew(_lease.lease_id, execution_id=str(result["execution_id"]))
        except Exception:
            pass
        finally:
            _lease_store().release(_lease.lease_id, str((result or {}).get("status") or "done").lower())
    # 把模式一并回传：平台据此知道"现在处理的是机器人任务域"，也便于审计
    # （不在模式里时给一句 hint，但不阻断执行——模式是提示性约定，不是安全边界）。
    if isinstance(result, dict):
        result.setdefault("robot_mode", mode_mod.state_for(principal.subject).name())
        result.setdefault("next_steps", _next_steps(result))
    if isinstance(result, dict) and _resolved is not None:
        result.setdefault("scene_id", _resolved.scene.scene_id)
        result.setdefault("resolved", _resolved.as_dict())
    auth.audit("tool_call", principal, tool="iraf_submit_task",
               execution_id=(result or {}).get("execution_id"),
               status=(result or {}).get("status"))
    return json.dumps(result, ensure_ascii=False)


def _lease_store():
    """租约存储：与事件库同库不同表；MCP 重启后租约仍在。"""
    if "lease_store" in _STATE:
        return _STATE["lease_store"]
    from iraf_core.leases import LeaseStore

    db = os.environ.get("IRAF_EVENT_STORE") or "/var/lib/iraf/iraf-runtime.db"
    raw = (os.environ.get("IRAF_LEASE_ENFORCE") or "1").strip().lower()
    enforce = raw not in ("0", "false", "off", "no")
    _STATE["lease_store"] = LeaseStore(db, enforce=enforce)
    return _STATE["lease_store"]


def _world_lease(scene, host, holder: str, ttl_s: int):
    """获取 host 级世界租约；冲突抛 LeaseConflict（由调用方转成 IRAF-RESOURCE-BUSY）。"""
    return _lease_store().acquire("host:%s" % host.id, "__world__", holder, ttl_s=ttl_s)


def _capacity_state(provider, holder: str) -> tuple:
    """返回 (free_hosts, held_hosts)：按 host 级世界租约统计（D2 的容量判定）。

    单主机时 held_hosts 必然包含全部 host；多主机时可用 free_hosts 判断是否需要
    返回 IRAF-CAPACITY-EXHAUSTED（而不是简单 BUSY）。
    """
    free, held = [], []
    store = _lease_store()
    for h in sorted(provider.hosts, key=lambda x: x.priority):
        lease = store.get_active("host:%s" % h.id, "__world__")
        if lease is None:
            free.append(h)
        else:
            held.append((h, lease))
    return free, held


def _supervisor_url(host=None) -> str:
    """控制面地址：优先用注册表里该 host 的 control_url，其次 IRAF_SUPERVISOR_URL。"""
    host_control_url = getattr(host, "control_url", "") if host is not None else ""
    return (host_control_url or os.environ.get("IRAF_SUPERVISOR_URL") or "").strip().rstrip("/")


def _supervisor_call(path: str, params: dict, timeout: int = 120, host=None) -> dict:
    """调 x86 supervisor 控制面（带 X-Iraf-Token）。失败返回标准错误 JSON。"""
    import urllib.error
    import urllib.parse
    import urllib.request

    base = _supervisor_url(host)
    if not base:
        return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                "reason": "未配置 IRAF_SUPERVISOR_URL"}
    url = base + path
    if params:
        url += "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, method="POST", data=b"")
    token_file = os.environ.get("IRAF_SUPERVISOR_TOKEN_FILE") or "/etc/iraf/supervisor.token"
    try:
        token = open(token_file, encoding="utf-8").read().strip()
    except Exception:
        token = ""
    if token:
        req.add_header("X-Iraf-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        try:
            return json.loads(exc.read().decode() or "{}")
        except Exception:
            return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                    "reason": "supervisor HTTP %s" % exc.code}
    except Exception as exc:
        return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                "reason": "supervisor 不可达：%s: %s" % (type(exc).__name__, exc)}


def _ensure_scene(scene: str, endpoint: str = "", host=None, provider=None) -> str:
    """确保本端点对应的 Provider 在线（幂等）：端口通就不动，不通就切场景。"""
    import json as _json
    import os as _os
    import socket as _sock
    import subprocess as _sp

    if _supervisor_url(host):
        # C4：按 priority 尝试 hosts；failover=next 时才顺延
        hosted = [host] if host is not None else []
        if not hosted:
            # 无 host 上下文（如 iraf_switch_scene）：退回环境变量里的控制面
            r = _supervisor_call("/ensure", {"scene": scene}, timeout=150)
            if r.get("status") == "OK":
                return "switched(supervisor)"
            return "switch-failed:%s" % (r.get("reason") or "supervisor 拒绝")
        failover = str((getattr(provider, "health", {}) or {}).get("failover") or "")
        if failover == "next" and provider is not None:
            hosted += [h for h in sorted(provider.hosts, key=lambda x: x.priority)
                       if host is None or h.id != host.id]
        last = ""
        for h in hosted:
            if not _supervisor_url(h):
                continue
            r = _supervisor_call("/ensure", {"scene": scene}, timeout=150, host=h)
            if r.get("status") == "OK":
                return "switched(supervisor)" if h is host else "switched(supervisor:%s)" % h.id
            last = "%s(%s)" % (r.get("reason") or "supervisor 拒绝", h.id)
        return "switch-failed:%s" % (last or "no-supervisor")
    if not endpoint:
        raw = _os.environ.get("IRAF_BACKEND_CONFIG", "")
        try:
            endpoint = str(_json.loads(raw).get("endpoint") or "")
        except Exception:
            return "no-endpoint"
    host, _, port_s = endpoint.rpartition(":")
    if not host or not port_s.isdigit():
        return "bad-endpoint"
    try:
        with _sock.create_connection((host, int(port_s)), timeout=2):
            return "already-up"
    except Exception:
        pass
    # 阶段 C6：板卡侧的 SSH 启停路径已删除（switch_scene.sh 已退役）。
    # 没有控制面时只能显式失败——不允许回落到任何"板卡自己起进程"的做法。
    return "switch-failed:supervisor-not-configured（板卡不再自管 Provider）"


def _endpoint_host() -> str:
    import json as _json
    import os as _os
    try:
        return str(_json.loads(_os.environ.get("IRAF_BACKEND_CONFIG", "")).get("endpoint") or "")
    except Exception:
        return ""


def _allowed_hosts() -> list:
    """允许的 Provider 主机：env 显式值优先；否则由注册表汇总；都没有则只允许 .121。"""
    env = (os.environ.get("IRAF_PROVIDER_ALLOWED_HOSTS") or "").strip()
    if env:
        return [h.strip() for h in env.split(",") if h.strip()]
    if _registry_mode() != "off":
        try:
            reg = _registry()
            reg.reload_if_changed()
            hosts = sorted({h for p in reg.providers.values() for h in p.allowed_hosts})
            if hosts:
                return hosts
        except Exception:
            pass
    return ["10.203.247.121"]


def _reject_foreign_host() -> str:
    """返回错误 JSON（若 endpoint 主机不在白名单），否则返回空串。"""
    import json as _json
    import os as _os
    endpoint = _endpoint_host()
    host, _, _port = endpoint.rpartition(":")
    allowed = _allowed_hosts()
    if host and host not in allowed:
        return _json.dumps({
            "status": "FAILED",
            "error_code": "IRAF-PROVIDER-HOST-REJECTED",
            "reason": "endpoint 主机 %s 不在允许列表 %s（本系统 Provider 只允许 .121）" % (host, allowed),
            "endpoint": endpoint,
        }, ensure_ascii=False)
    return ""


@app.tool()
def iraf_switch_scene(scene: str) -> str:
    """（deprecated，阶段 B 起）显式切换场景：piper / piper-y05 / ur5 / joint。

    单端点后不需要"切场景"：提交时用 scene_id 即可（注册表解析 + 自动确保 Provider 在线）。
    本工具仅保留用于排障与过渡期兼容。
    """
    import json as _json
    import subprocess as _sp
    from iraf_adapters.mcp import auth as _auth

    allowed = ("piper", "piper-y05", "ur5", "joint")
    try:
        principal = _principal()
        _auth.require_scope(principal, "task.submit")
    except _auth.AuthError as exc:
        return _auth_error(exc)
    if scene not in allowed:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                            "reason": "scene must be one of %s" % ", ".join(allowed)},
                           ensure_ascii=False)
    result = _ensure_scene(scene)
    ok = result in ("already-up", "switched", "switched(supervisor)")
    _auth.audit("tool_call", principal, tool="iraf_switch_scene", scene=scene,
                status="OK" if ok else "FAILED")
    return _json.dumps({"status": "OK" if ok else "FAILED", "scene": scene,
                        "deprecated": True,
                        "use": "iraf_submit_task(text=..., scene_id=...)",
                        "result": result}, ensure_ascii=False)


@app.tool()
def iraf_run_scenario(scenario: str = "nominal", display: str = "window",
                      hold_s: int = 60, scene_id: str = "") -> str:
    """跑 handoff_lab 全量链路（联合世界 9 步：verify→dock→pick→place_in_tray→confirm→
    dock_station_b→confirm_at_b→unload→place_at_b）。内部调官方链路脚本，返回逐步 status。

    这是"全量演示"的唯一入口；不要用 terminal/SSH 自己跑 run_joint_chain.py。
    单次约 3 分钟（仿真本身的时间），期间不要重复调用。
    """
    import json as _json
    import os as _os
    import subprocess as _sp
    from iraf_adapters.mcp import auth as _auth

    allowed_scen = ("nominal", "stand_stop", "fault_sensor_unavailable")
    try:
        principal = _principal()
        _auth.require_scope(principal, "task.submit")
    except _auth.AuthError as exc:
        return _auth_error(exc)
    _resolved, _scene_err = _resolve_scene(principal, scene_id)
    if _scene_err:
        return _scene_err
    if scenario not in allowed_scen:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                            "reason": "scenario 必须是 %s 之一" % ", ".join(allowed_scen)},
                           ensure_ascii=False)
    host = _resolved.host.host if _resolved is not None else _os.environ.get("IRAF_SCENARIO_HOST", "10.203.247.121")
    allow = _allowed_hosts()
    if host not in allow:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-PROVIDER-HOST-REJECTED",
                            "reason": "scenario 主机 %s 不在允许列表 %s" % (host, allow)},
                           ensure_ascii=False)
    mode = "window" if str(display).lower() in ("window", "interactive", "viewer") else "none"
    try:
        hold = max(0, min(int(hold_s), 600))
    except Exception:
        hold = 60
    _lease = None
    if _resolved is not None:
        from iraf_core.leases import LeaseConflict

        try:
            _lease = _world_lease(_resolved.scene, _resolved.host, principal.subject,
                                  ttl_s=max(int(hold) + 300, _resolved.scene.lease_ttl_s))
        except LeaseConflict as exc:
            return _json.dumps({"status": "FAILED", "error_code": "IRAF-RESOURCE-BUSY",
                                "reason": str(exc), "lease": exc.lease.as_dict()},
                               ensure_ascii=False)

    def _rel(reason="done"):
        if _lease is not None:
            try:
                _lease_store().release(_lease.lease_id, reason)
            except Exception:
                pass

    if _supervisor_url():
        r = _supervisor_call("/scenario", {"scenario": scenario, "display": mode,
                                           "hold": hold}, timeout=900)
        ok = r.get("status") == "SUCCEEDED"
        _auth.audit("tool_call", principal, tool="iraf_run_scenario", scene=scenario,
                    status="SUCCEEDED" if ok else "FAILED", via="supervisor")
        if not ok:
            _rel("scenario-failed")
            return _json.dumps({"status": "FAILED",
                                "error_code": r.get("error_code") or "IRAF-SCENARIO-ERROR",
                                "reason": r.get("reason") or r.get("error") or "链路失败",
                                "steps": r.get("steps") or [],
                                "log": r.get("log")}, ensure_ascii=False)
        _rel("scenario-done")
        return _json.dumps({"status": "SUCCEEDED", "scenario": scenario, "world": "joint",
                            "display": mode, "passed": r.get("passed"),
                            "steps": r.get("steps") or [], "rc": r.get("rc"),
                            "wall_s": r.get("wall_s"), "log": r.get("log"),
                            "via": "supervisor",
                            "note": "全量链路在 x86 supervisor 本机执行，板卡不再 SSH"},
                           ensure_ascii=False)

    cmd = "bash /home/coretek/iraf-instance/scripts/run_full_scenario.sh %s %s %d" % (scenario, mode, hold)
    try:
        proc = _sp.run(["ssh", "-n", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
                        "coretek@%s" % host, cmd],
                       capture_output=True, text=True, timeout=420)
    except Exception as exc:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-SCENARIO-ERROR",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)
    payload = {}
    for line in reversed((proc.stdout or "").strip().splitlines()):
        if line.startswith("{"):
            try:
                payload = _json.loads(line); break
            except Exception:
                continue
    ok = bool(payload.get("passed")) and int(payload.get("rc", 1)) == 0
    _auth.audit("tool_call", principal, tool="iraf_run_scenario", scene=scenario,
                status="SUCCEEDED" if ok else "FAILED")
    _rel("scenario-done" if ok else "scenario-failed")
    return _json.dumps({
        "status": "SUCCEEDED" if ok else "FAILED",
        "scenario": scenario, "world": "joint", "display": mode,
        "passed": payload.get("passed"), "steps": payload.get("steps") or [],
        "rc": payload.get("rc"), "error": payload.get("error"),
        "log": "%s:/tmp/iraf-scenario.log" % host,
        "note": "全量链路在 x86 进程内跑（多机器人同一世界），不经单机 Provider",
    }, ensure_ascii=False)


@app.tool()
def iraf_scene_status(scene_id: str = "") -> str:
    """只读：场景 / Provider / 主机 / 注册表状态（排障的唯一入口，不要用 ss/nc 猜）。

    scene_id 为空时返回全部场景；给定则只返回该场景解析结果（含将使用的 host/endpoint）。
    不发起任何网络探测，不在本工具里做健康检查（健康由 Provider 的 Health 接口负责）。
    """
    from iraf_adapters.mcp import auth

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    mode = _registry_mode()
    payload: dict = {"status": "SUCCEEDED", "mode": mode,
                     "allowed_hosts": _allowed_hosts(),
                     "lease_enforce": _lease_store().enforce,
                     "leases": [l.as_dict() for l in _lease_store().list_active()]}
    if mode == "off":
        payload["note"] = "注册表未启用（IRAF_SCENE_REGISTRY_MODE=off），仅报告 env 事实"
        payload["provider_endpoint"] = _endpoint_host() or None
        auth.audit("tool_call", principal, tool="iraf_scene_status", mode=mode)
        return json.dumps(payload, ensure_ascii=False)
    try:
        reg = _registry()
        reg.reload_if_changed()
        if (scene_id or "").strip():
            resolved = reg.resolve(scene_id)
            payload["resolved"] = resolved.as_dict()
        payload.update(reg.status())
    except Exception as exc:
        from iraf_core.scene_registry import RegistryError

        code = exc.code if isinstance(exc, RegistryError) else "IRAF-REGISTRY-INVALID"
        detail = exc.detail if isinstance(exc, RegistryError) else "%s: %s" % (type(exc).__name__, exc)
        auth.audit("tool_call", principal, tool="iraf_scene_status", status="FAILED")
        return json.dumps({"status": "FAILED", "error_code": code, "reason": detail},
                          ensure_ascii=False)
    auth.audit("tool_call", principal, tool="iraf_scene_status", mode=mode)
    return json.dumps(payload, ensure_ascii=False)


@app.tool()
def iraf_lease(action: str = "list", scene_id: str = "", resource_id: str = "__world__",
               lease_id: str = "", ttl_s: int = 0) -> str:
    """显式租约：acquire / renew / release / list（D3）。

    普通任务由 `iraf_submit_task` 自动加租约，本工具只用于：
      · 排障时查看"谁占着 world"（list）；
      · 长任务分步演示时显式持有（acquire/renew/release）。
    acquire 的资源键默认 `__world__`（host 级）；冲突返回 IRAF-RESOURCE-BUSY。
    """
    from iraf_adapters.mcp import auth
    from iraf_core.leases import LeaseConflict

    try:
        principal = _principal()
        need = "task.read" if (action or "list").lower() == "list" else "task.submit"
        auth.require_scope(principal, need)
    except auth.AuthError as exc:
        return _auth_error(exc)
    act = (action or "list").strip().lower()
    store = _lease_store()

    if act == "list":
        return json.dumps({"status": "SUCCEEDED", "enforce": store.enforce,
                           "leases": [l.as_dict() for l in store.list_active()]},
                          ensure_ascii=False)
    if act == "renew":
        if not lease_id:
            return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                               "reason": "renew 需要 lease_id"}, ensure_ascii=False)
        lease = store.renew(lease_id, ttl_s=(ttl_s or None))
        if lease is None:
            return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                               "reason": "租约不存在或已释放：%s" % lease_id},
                              ensure_ascii=False)
        return json.dumps({"status": "SUCCEEDED", "lease": lease.as_dict()},
                          ensure_ascii=False)
    if act == "release":
        if not lease_id:
            return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                               "reason": "release 需要 lease_id"}, ensure_ascii=False)
        ok = store.release(lease_id, "explicit")
        return json.dumps({"status": "SUCCEEDED" if ok else "FAILED",
                           "released": ok, "lease_id": lease_id}, ensure_ascii=False)
    if act != "acquire":
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": "action 只接受 acquire / renew / release / list"},
                          ensure_ascii=False)

    _resolved, _scene_err = _resolve_scene(principal, scene_id)
    if _scene_err:
        return _scene_err
    if _resolved is None:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-SCENE-AMBIGUOUS",
                           "reason": "acquire 需要 scene_id（或默认场景）"}, ensure_ascii=False)
    ttl = int(ttl_s) if int(ttl_s) > 0 else _resolved.scene.lease_ttl_s
    key_scene = ("host:%s" % _resolved.host.id) if resource_id == "__world__" else _resolved.scene.scene_id
    try:
        lease = store.acquire(key_scene, resource_id, principal.subject, ttl_s=ttl)
    except LeaseConflict as exc:
        free, held = _capacity_state(_resolved.provider, principal.subject)
        # 单主机时"该 host 被占"就是 BUSY；CAPACITY 只在多主机且全部被占时才有意义
        same_host_held = any(h.id == _resolved.host.id for h, _l in held)
        code = ("IRAF-RESOURCE-BUSY" if same_host_held or len(_resolved.provider.hosts) == 1
                else "IRAF-CAPACITY-EXHAUSTED")
        return json.dumps({"status": "FAILED", "error_code": code, "reason": str(exc),
                           "lease": exc.lease.as_dict(),
                           "free_hosts": [h.id for h in free]}, ensure_ascii=False)
    auth.audit("tool_call", principal, tool="iraf_lease", action="acquire",
               scene=_resolved.scene.scene_id, resource=resource_id, lease=lease.lease_id)
    return json.dumps({"status": "SUCCEEDED", "lease": lease.as_dict()}, ensure_ascii=False)


@app.tool()
def iraf_get_task(execution_id: str) -> str:
    """按 execution_id 查询 IRAF 事件库里的执行事实（权威来源，不是缓存）。

    只返回**本调用方自己提交**的执行；别人的 execution_id 一律按 NOT_FOUND 处理，
    不泄露存在性（2026-11-03 多租户鉴权）。
    """
    from iraf_adapters.mcp import auth

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    store_path = os.environ.get("IRAF_EVENT_STORE") or "/var/lib/iraf/iraf-runtime.db"
    if not Path(store_path).is_file():
        return json.dumps({"status": "NOT_FOUND", "reason": "事件库不存在: %s" % store_path},
                          ensure_ascii=False)
    from iraf_core.store import SqliteExecutionStore

    result = SqliteExecutionStore(store_path).get_for_subject(execution_id, principal.subject)
    if result is None:
        auth.audit("tool_call", principal, tool="iraf_get_task",
                   execution_id=execution_id, status="NOT_FOUND")
        return json.dumps({"status": "NOT_FOUND", "execution_id": execution_id},
                          ensure_ascii=False)
    auth.audit("tool_call", principal, tool="iraf_get_task",
               execution_id=execution_id, status=result.get("status"))
    return json.dumps(result, ensure_ascii=False)


def main() -> int:
    """按 IRAF_MCP_TRANSPORT 选择传输：stdio（宿主拉起）或 streamable-http（跨机/语音宿主）。

    为什么需要 http：stdio 只能由**本机**宿主作为子进程拉起。语音/语义前端常常不在板上
    （智控域没有 /dev/snd，实测 microphones=0），要跨机连就必须有网络传输。
    端点取值放装配单（`deploy/sdk/env/edge-intel.env.tmpl`），这里只读，不写默认 IP/端口。
    """
    transport = (os.environ.get("IRAF_MCP_TRANSPORT") or "stdio").strip().lower()
    if transport in ("http", "streamable-http", "streamable_http"):
        host = (os.environ.get("IRAF_MCP_HOST") or "127.0.0.1").strip()
        port = int(os.environ.get("IRAF_MCP_PORT") or "8765")
        path = (os.environ.get("IRAF_MCP_PATH") or "/mcp").strip()
        print(f"[iraf-mcp] transport=streamable-http host={host} port={port} path={path}",
              file=sys.stderr, flush=True)
        app.settings.host = host
        app.settings.port = port
        app.settings.streamable_http_path = path
        # MCP SDK 默认开 DNS rebinding 防护：只接受 loopback 的 Host，跨机访问会被
        # 直接判 421 Misdirected Request（实测）。允许的主机名只能由装配单/环境给，
        # 代码里不猜板卡 IP（AGENTS 5.3）。
        allowed = (os.environ.get("IRAF_MCP_ALLOWED_HOSTS") or "").strip()
        hosts = [h.strip() for h in allowed.split(",") if h.strip()] or [
            f"127.0.0.1:{port}", f"localhost:{port}"]
        try:
            from mcp.server.transport_security import TransportSecuritySettings

            app.settings.transport_security = TransportSecuritySettings(
                allowed_hosts=hosts)
            print(f"[iraf-mcp] allowed_hosts={hosts}", file=sys.stderr, flush=True)
        except Exception as exc:  # 老版本 SDK 没有这项：显式提示，不静默
            print(f"[iraf-mcp][警告] 无法设置 allowed_hosts（{exc}）；跨机访问可能被 421 拒绝",
                  file=sys.stderr, flush=True)

        # 北向鉴权（2026-11-03，fail-closed）：HTTP 形态默认必须有 token 注册表。
        # 只有显式 IRAF_MCP_INSECURE=1 且绑定 loopback 才允许无鉴权（本机调试）。
        from iraf_adapters.mcp.auth import AuthError, BearerAuthMiddleware, TokenRegistry

        insecure = (os.environ.get("IRAF_MCP_INSECURE") or "").strip().lower() in {
            "1", "true", "yes"}
        token_file = Path(os.environ.get("IRAF_MCP_TOKEN_FILE")
                          or "/etc/iraf/mcp-tokens.json")
        registry = None
        if token_file.is_file():
            try:
                registry = TokenRegistry.from_file(token_file)
                print(f"[iraf-mcp] auth=bearer token_file={token_file} "
                      f"tokens={len(registry.redacted())}", file=sys.stderr, flush=True)
            except AuthError as exc:
                print(f"[iraf-mcp][错误] {exc}", file=sys.stderr, flush=True)
                return 2
        elif not insecure:
            print("[iraf-mcp][错误] HTTP 形态缺少 token 注册表：%s\n"
                  "  必须先签发 token（python -m iraf_tools.mcp_tokens issue ...）；"
                  "仅本机调试可设 IRAF_MCP_INSECURE=1 并绑定 127.0.0.1。"
                  % token_file, file=sys.stderr, flush=True)
            return 2
        else:
            loopback = host in {"127.0.0.1", "localhost", "::1"}
            if not loopback:
                print("[iraf-mcp][错误] IRAF_MCP_INSECURE=1 只允许绑定 loopback；"
                      "对外必须配置 IRAF_MCP_TOKEN_FILE", file=sys.stderr, flush=True)
                return 2
            print("[iraf-mcp][警告] auth=disabled（IRAF_MCP_INSECURE=1，仅 loopback 调试）",
                  file=sys.stderr, flush=True)

        cert = (os.environ.get("IRAF_MCP_TLS_CERT") or "").strip()
        key = (os.environ.get("IRAF_MCP_TLS_KEY") or "").strip()
        if bool(cert) != bool(key):
            print("[iraf-mcp][错误] IRAF_MCP_TLS_CERT 与 IRAF_MCP_TLS_KEY 必须同时给",
                  file=sys.stderr, flush=True)
            return 2
        if cert and not Path(cert).is_file():
            print(f"[iraf-mcp][错误] TLS 证书不存在：{cert}", file=sys.stderr, flush=True)
            return 2
        if key and not Path(key).is_file():
            print(f"[iraf-mcp][错误] TLS 私钥不存在：{key}", file=sys.stderr, flush=True)
            return 2
        ca = (os.environ.get("IRAF_MCP_TLS_CA") or "").strip()
        require_client = (os.environ.get("IRAF_MCP_TLS_REQUIRE_CLIENT") or "").strip().lower() in {
            "1", "true", "yes"}

        import uvicorn

        starlette_app = app.streamable_http_app()
        if registry is not None:
            starlette_app.add_middleware(BearerAuthMiddleware, registry=registry)
        ssl_kwargs = {}
        if cert:
            import ssl as _ssl

            ssl_kwargs = {"ssl_certfile": cert, "ssl_keyfile": key}
            if ca:
                ssl_kwargs["ssl_ca_certs"] = ca
                ssl_kwargs["ssl_cert_reqs"] = _ssl.CERT_REQUIRED if require_client else _ssl.CERT_NONE
            print(f"[iraf-mcp] tls=on cert={cert} client_cert={'required' if require_client else 'optional'}",
                  file=sys.stderr, flush=True)
        else:
            print("[iraf-mcp][警告] tls=off：对外部署必须由本服务或反向代理终止 TLS",
                  file=sys.stderr, flush=True)
        uvicorn.run(starlette_app, host=host, port=port, log_level="info", **ssl_kwargs)
        return 0
    if transport not in ("stdio", ""):
        print(f"[iraf-mcp][错误] 不认识的 IRAF_MCP_TRANSPORT={transport}"
              "（允许 stdio / http）", file=sys.stderr, flush=True)
        return 2
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
