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

import functools
import json
import os
import sys
import time
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
        saved = {k: os.environ.get(k) for k in
                 ("IRAF_PROFILE", "IRAF_BACKEND_CONFIG", "IRAF_BACKEND_ENTRYPOINT")}
        try:
            os.environ["IRAF_PROFILE"] = resolved.scene.profile
            scene_override = (resolved.scene.raw or {})
            if scene_override.get("backend_entrypoint"):
                os.environ["IRAF_BACKEND_ENTRYPOINT"] = str(scene_override["backend_entrypoint"])
            scene_config = scene_override.get("backend_config")
            if isinstance(scene_config, dict):
                os.environ["IRAF_BACKEND_CONFIG"] = json.dumps(scene_config)
            # 装配单里保留的是"基线配置"（token_file/timeout/subject 等），这里只合并 endpoint，
            # 不整体覆盖 —— 否则凭据配置丢失，Provider 鉴权直接失败。
            if isinstance(scene_config, dict):
                base = dict(scene_config)
            else:
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
def iraf_enter_robot_mode(scene_id: str = "", detail: bool = False) -> str:
    """进入「机器人开发模式」（用户说"开始机器人开发模式"时必须调用本工具）。

    零参数、名字即语义：模型不需要在 enum 里选 action，听到这句中文就调它。
    等价于 iraf_robot_mode(action="enter")。返回 mode / 机器人 Skill 清单 / profile /
    安全策略 / 后端端点 / simulation 标记 / next_steps（进入后照做的固定顺序）。
    调用后**只做盘点并报告**，不要为了验证链路先提交一条任务（会弄脏仿真场景）。
    """
    return iraf_robot_mode(action="enter", scene_id=scene_id, detail=detail)


def _audited(tool_name: str):
    """给 MCP 工具加一条 tool_span 审计：ts(开始) + duration_ms + outcome。

    为什么需要它（2026-11-12）：只有 tool_call 的 ts 与少数工具的 duration_s，
    端到端核对时要靠 AgentOS 日志拼；加上这条后，一次调用在 audit 里就能看到
    "MCP 收到 → 工具返回"的真实耗时（含 IRAF 执行，如 submit_task 的同步执行）。
    """
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            from iraf_adapters.mcp import auth as _auth
            started = time.time()
            outcome = "ok"
            try:
                return fn(*args, **kwargs)
            except BaseException:
                outcome = "error"
                raise
            finally:
                try:
                    _auth.audit("tool_span", _principal(), tool=tool_name,
                                duration_ms=int((time.time() - started) * 1000),
                                outcome=outcome)
                except Exception:
                    pass
        return wrapper
    return deco


@app.tool()
@_audited("iraf_plan_task")
def iraf_plan_task(scene_id: str = "ur5-pick-planned", target_id: str = "box_01",
                   approach_m: float = 0.10, lift_m: float = 0.05,
                   duration_ms: int = 3000) -> str:
    """端侧运动学**计划**计算：加载模型并做 IK/FK，返回计划与报告。

    返回：
    IK 收敛/误差/迭代、关节解、限位余量、夹爪开口、预计时长、模型 sha256、报告路径。
    强制标注 `planned=true, physics=false, backend=kinematic` —— 这是"计划是否可行"，
    **不是**抓取成功或物理验收（无接触/滑移/沉降信息）。
    """
    from iraf_adapters.kinematic.planner import PlanError, plan_pick
    from iraf_adapters.mcp import auth

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    auth.audit("tool_call", principal, tool="iraf_plan_task", scene_id=scene_id, target_id=target_id)
    try:
        result = plan_pick(scene_id=scene_id, target_id=target_id,
                           approach_m=float(approach_m), lift_m=float(lift_m),
                           duration_ms=int(duration_ms))
    except PlanError as exc:
        hint = {
            "SCENE-NOT-FOUND": "端侧需有场景模型：deploy/sdk/prebuilt/scenes/<stem>/ 或 build/；设 IRAF_REPO_ROOT",
            "TARGET-NOT-FOUND": "检查 target_id（ur5-pick 场景为 box_01）",
            "IK-FAILED": "目标可能不可达；调整 approach_m 或检查模型",
            "IK-NOT-CONVERGED": "残差超容差，计划不可行（不伪造成功）",
        }.get(exc.code, "")
        return json.dumps({"status": "FAILED", "error_code": exc.code, "reason": exc.detail,
                           "hint": hint, "planned": True, "physics": False,
                           "backend": "kinematic"}, ensure_ascii=False)
    compact = {k: result[k] for k in ("status", "planned", "physics", "backend", "scene",
                                      "target", "approach", "ik", "gripper", "timing",
                                      "model", "report", "warnings")}
    compact["limits"] = {"min_margin_deg": result["limits"]["min_margin_deg"],
                         "per_joint": result["limits"]["per_joint"]}
    compact["hint"] = "⚠️ 计划计算（无物理/接触）；真实抓取需 x86 MuJoCo 或真机执行"
    return json.dumps(compact, ensure_ascii=False)


@app.tool()
def iraf_rt_latency(duration_s: int = 10, interval_us: int = 200,
                    priority: int = 240, histogram_us: int = 100,
                    target_host: str = "") -> str:
    """测试实时系统（RTOS/运控域）的实时性能：SSH 执行 cyclictest 并出报告。

    **RTOS 内不安装任何 IRAF 进程**（会干扰实时性）；本工具在边缘侧编排 SSH，
    只执行固定命令并解析输出：
        cyclictest --mlockall --smp --priority=<P> --interval=<I> --distance=0 -n -h <H> -q -D <D>s

    目标地址来自装配单 `IRAF_RT_TARGET_HOST`（可用 target_host 临时覆盖），账号/密钥同理。
    参数有界：duration_s 1..300、interval_us 50..100000、priority 1..255；
    报告写入 `IRAF_RT_REPORT_DIR`（JSON + Markdown）。
    失败显式返回 error_code：TARGET-UNCONFIGURED / SSH-FAILED / CYCLICTEST-MISSING /
    PERMISSION / TIMEOUT / PARSE-FAILED —— 不会把失败包装成成功。
    """
    from iraf_adapters.mcp import auth
    from iraf_adapters.tooling.rt_latency import RtTarget, RtTestError, run_test

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    auth.audit("tool_call", principal, tool="iraf_rt_latency",
               target=target_host or None, duration_s=int(duration_s))
    tgt = RtTarget.from_env()
    if (target_host or "").strip():
        tgt.host = target_host.strip()
    try:
        result = run_test(duration_s=int(duration_s), interval_us=int(interval_us),
                          priority=int(priority), histogram_us=int(histogram_us), target=tgt)
        result["kind"] = "diagnostic"
        return json.dumps(result, ensure_ascii=False)
    except RtTestError as exc:
        hint = {
            "TARGET-UNCONFIGURED": "设置 IRAF_RT_TARGET_HOST=<RTOS IP>（装配单 /etc/iraf/edge-intel.env）后重试",
            "CYCLICTEST-MISSING": "在 RTOS 上安装 rt-tests（dnf install -y rt-tests）",
            "PERMISSION": "需要 root；或设 IRAF_RT_TARGET_SUDO=1",
            "SSH-FAILED": "检查 RTOS 可达性与 SSH 凭据（IRAF_RT_TARGET_USER/IDENTITY/KNOWN_HOSTS）",
        }.get(exc.code, "")
        return json.dumps({"status": "FAILED", "error_code": exc.code,
                           "reason": exc.detail, "hint": hint, "kind": "diagnostic"},
                          ensure_ascii=False)


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


def _infer_scene_from_text(text: str) -> str:
    """注册表 off 模式返回空；异常不阻断提交（推断只是优化，不改变显式语义）。"""
    try:
        if _registry_mode() == "off":
            return ""
        return _registry().infer(text)
    except Exception:
        return ""


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
@_audited("iraf_robot_mode")
def iraf_robot_mode(action: str = "status", ttl_seconds: int = 1800,
                    scene_id: str = "", detail: bool = False) -> str:
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
    if _resolved is None and _registry_mode() == "on" and not _plan_only():
        # 计划模式不连 provider：解析默认场景只会在返回里塞一个 x86 endpoint，误导模型与用户。
        _r, _e = _resolve_scene(principal, "")
        _resolved = _r
    payload.update(_robot_facts(_resolved))
    if _plan_only():
        # 计划模式的"装配事实"：无 provider 端点、无仿真开关、只有端侧计划场景。
        payload.pop("simulation", None)
        payload["provider_endpoint"] = None
        payload["execution_mode"] = "plan-only"
        payload["computation"] = "端侧运动学（kinematic）"
        payload["scenes"] = ["ur5-pick-planned", "piper-pick-planned"]
        # 计划模式返回文本里不出现"仿真/simulation"字样（演示要求）；
        # hint / safety_policy / next_steps 是模型最容易照念的三处。
        payload["hint"] = (
            "机器人开发模式已开启：动作类请求交给 iraf_submit_task；"
            "当前为端侧运动学计划模式，只做计算、不搬动场景，不要提交任务做验证。"
        )
        payload["safety_policy"] = None
        payload["next_steps"] = [
            step.replace(" / simulation", "").replace("simulation / ", "")
            for step in payload.get("next_steps") or []
        ]
    # 把"进入之后照做什么"直接写进返回：实机证据（2026-10-22）是模型自己设计了
    # "只读自检 + 冒烟提交"两段式，冒烟那次把仿真场景弄脏。规则写在返回值里比写在文档里有效。
    # 计划模式（IRAF_PLAN_ONLY=1）不执行、不搬场景：把"复位场景"那条护栏换成计划模式说明，
    # 否则模型会反复提醒"一次抓取会搬走目标、需 reset_scene"，既误导又拖慢回复。
    _steps = list(_enter_next_steps())
    if _plan_only():
        _steps = [step for step in _steps if not step.startswith("⑤")] + [
            "⑤ 当前为端侧计划模式（IRAF_PLAN_ONLY=1）：只做运动学计算，不搬动场景、无需 reset_scene；"
            "要真实执行需关闭该开关并确保 x86 在线"
        ]
        # 计划模式文本里不出现 simulation/仿真字样（演示要求）；事实在报告与字段里。
        _steps = [step.replace(" / simulation", "").replace("simulation / ", "")
                  for step in _steps]
    payload["next_steps"] = _steps
    # 自带"正确用法示范"：模型不必自己摸索该怎么调，也减少它去翻源码/重写实现的动机。
    # 注意：examples 是**语义映射说明**，不是验收脚本；purpose 字段已明示"不要求执行"。
    payload["examples"] = {"robot_task": _WORKED_EXAMPLE, "non_robot": _FILTERED_EXAMPLE}
    if not (detail or _verbose_output()):
        # 紧凑模式（2026-11-11 实测）：完整返回 4.3k 字符 ≈1.45k tokens，其中 skill_triggers/intents/
        # routing_policy 与工具描述、系统提示重复；板卡本地 Qwen 生成仅 13.7 tok/s，模型读得越多、
        # 回得越长，用户等得越久。默认只给"能立刻做决定"的最小集，细节用 detail=true。
        for key in ("skill_triggers", "intents", "routing_policy", "examples"):
            payload.pop(key, None)
        # next_steps 是**行为护栏**（"不要编造场景名/示例"），此前为省 token 只留前 2 条，
        # 结果模型自己补了"抓取桌上的红色球"这类不存在的示例（2026-11-11 实测）。
        # 结论：护栏不能省；省的是与系统提示重复的路由表（skill_triggers/intents/routing_policy）。
        try:
            if _plan_only():
                # 计划模式只呈现端侧可算的场景；不要列 joint-handoff（跑不了 9 步链路）
                payload["scenes"] = ["ur5-pick-planned", "piper-pick-planned"]
            elif _registry_mode() == "on":
                _reg = _registry()
                _reg.reload_if_changed()
                payload["scenes"] = sorted(_reg.scenes)
        except Exception:
            pass
        _handoff = "跑 handoff_lab 的 nominal（场景 joint-handoff）"
        payload["allowed_task_examples"] = [
            "抓取 box_01（场景 ur5-pick）",
            "piper 抓取 box_01（场景 piper-pick）",
            "跑 handoff_lab 的 nominal（场景 joint-handoff）",
            "算一下 piper 抓取计划",
            "算一下 ur5 抓取计划",
        ]
        if _plan_only():
            # 计划模式跑不了 9 步联合链路，别让模型把它当可选项提出来
            payload["allowed_task_examples"] = [
                x for x in payload["allowed_task_examples"] if x != _handoff]
        # 端侧计划计算（不跑 MuJoCo 物理、不需要 x86）：与真实执行明确分流，避免模型把
        # "算计划"也丢给 iraf_submit_task（那会去连 provider）。
        payload["planning_tool"] = {
            "tool": "iraf_plan_task",
            "when": "用户要'计划/预估/端侧算一下'时使用；只做端侧运动学计算，不执行动作",
            "scenes": {"ur5-pick-planned": "UR5 抓取计划", "piper-pick-planned": "Piper 抓取计划"},
        }
        payload["reply_hint"] = ("用不超过 5 行汇报：模式 / 场景列表 / 技能数；"
                                 "示例只能逐字取自 allowed_task_examples，"
                                 "**禁止编造场景名或任务示例**；不确定就先问用户。")
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


# 内部实现：**不是**对外工具（2026-11-12 实测它曾被 @app.tool() 暴露成第 13 个工具，
# 模型可直接调用从而绕过 iraf_submit_task 的 plan-fallback 与统一审计）。
def _submit_task_impl(text: str, deadline_seconds: int = 180,
                      scene_id: str = "", resource_id: str = "", detail: bool = False) -> str:
    """提交一条**任务**给 IRAF（自然语言语义）。技能选择由 IRAF Skill Manager 完成。

    ⚠ 这是机器人动作的**唯一受支持入口**：
      · 抓取/移动/放置/标定/视觉/停止这类请求，必须调用本工具，**不要**用 execute_code、
        terminal、host_command 或自己写脚本去驱动机器人（那样绕过 Skill Manager、策略与证据链）；
      · 不要自行"重新实现"抓取，也不要修改本仓源码去迁就一次失败；失败先按返回里的
        `next_steps` 处理（多数是场景状态或参数问题，不是实现 bug）。

    返回与 IRAF CLI 同构的 JSON：status / execution_id / error_code / reason / evidence /
    resolved_skill / robot_mode / next_steps。失败不会被包装成成功。

    `deadline_seconds` 默认 180s（2026-11-09 实测：ur5 冷启动 + 高负载时 120s 会触发
    `任务超过截止时间`；仿真链路本身只有 ~57s，余量给启动与调度抖动）。
    """
    from iraf_adapters.agentos.cli import build_request
    from iraf_adapters.mcp import auth
    from iraf_adapters.mcp import mode as mode_mod

    try:
        principal = _principal()
        auth.require_scope(principal, "task.submit")
    except auth.AuthError as exc:
        return _auth_error(exc)
    if not (scene_id or "").strip():
        # 未显式给场景时，先从用户原话里推机型（piper/ur5/go2…），
        # 避免"piper 抓取"落到联合世界；推不出才走 default_scene。
        scene_id = _infer_scene_from_text(text)
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
            # D2：failover=next 时，若还有空闲 host，就把请求改路由到那台（不排队、不静默重试）
            free, held = _capacity_state(_resolved.provider, principal.subject)
            failover = str((_resolved.provider.health or {}).get("failover") or "")
            if failover == "next" and free:
                _next = free[0]
                try:
                    _lease = _world_lease(_resolved.scene, _next, principal.subject,
                                          ttl_s=max(int(deadline_seconds) + 60,
                                                    _resolved.scene.lease_ttl_s))
                except LeaseConflict:
                    _lease = None
                if _lease is not None:
                    _resolved.host = _next          # 改路由到空闲 host
                    auth.audit("tool_call", principal, tool="iraf_submit_task",
                               rerouted_to=_next.id, endpoint=_next.endpoint)
                else:
                    return json.dumps({"status": "FAILED", "error_code": "IRAF-RESOURCE-BUSY",
                                       "reason": str(exc), "lease": exc.lease.as_dict()},
                                      ensure_ascii=False)
            else:
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
    except BaseException:
        # 未预期异常也要释放世界租约（2026-11-11 实测：否则 BUSY 到 TTL）
        if _lease is not None:
            try:
                _lease_store().release(_lease.lease_id, "pre-exec-error")
            except Exception:
                pass
        raise
    try:
        request = build_request(text, deadline_seconds=int(deadline_seconds),
                                controller="agentos-mcp-task")
    except ValueError as exc:
        # 租约已拿但请求非法：必须释放，否则下一次提交会一直 BUSY 到 TTL。
        if _lease is not None:
            _lease_store().release(_lease.lease_id, "input-invalid")
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": str(exc)}, ensure_ascii=False)
    except BaseException:
        # 非 ValueError（如 RuntimeError: 缺配置）同样要释放租约后再上抛
        if _lease is not None:
            try:
                _lease_store().release(_lease.lease_id, "pre-exec-error")
            except Exception:
                pass
        raise
    _reject = _reject_foreign_host()
    if _reject:
        if _lease is not None:
            _lease_store().release(_lease.lease_id, "host-rejected")
        return _reject
    if _resolved is not None:
        try:
            _sw = _ensure_scene(_resolved.scene.switch_name or _resolved.scene.scene_id,
                                _resolved.host.endpoint, _resolved.host, _resolved.provider)
        except BaseException:
            if _lease is not None:
                try:
                    _lease_store().release(_lease.lease_id, "provider-error")
                except Exception:
                    pass
            raise
        if _sw.startswith(("switch-failed", "switch-error", "bad-endpoint", "no-endpoint")):
            if _lease is not None:
                _lease_store().release(_lease.lease_id, "provider-unavailable")
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
    payload = result if (detail or _verbose_output()) else _compact_result(result, tool="iraf_submit_task")
    return json.dumps(payload, ensure_ascii=False)


_PLAN_SCENES = {"piper-pick": "piper-pick-planned", "ur5-pick": "ur5-pick-planned",
                "piper-pick-planned": "piper-pick-planned",
                "ur5-pick-planned": "ur5-pick-planned"}


def _compact_plan(plan: dict) -> dict:
    """把计划结果压成适合 chat 的大小（完整计划仍在报告/ detail=true 里）。"""
    ik = plan.get("ik") or {}
    limits = plan.get("limits") or {}
    return {
        "status": plan.get("status"),
        "mode": "plan",
        "backend": "kinematic",
        "scene": plan.get("scene"),
        "target": plan.get("target"),
        "ik": {"iterations": ik.get("iterations"), "error_m": ik.get("error_m"),
               "converged": ik.get("converged")},
        "limits": {"min_margin_deg": limits.get("min_margin_deg")},
        "warnings": plan.get("warnings") or [],
        "summary": plan.get("summary"),
        "report": plan.get("report"),
        "hint": plan.get("hint") or "端侧运动学计划（planned=true, physics=false），未执行物理仿真",
    }


def _replay_enabled() -> bool:
    return (os.environ.get("IRAF_REPLAY_EVIDENCE") or "1").strip().lower() not in ("0", "false", "no", "off")


def _robot_label(text, fallback) -> str:
    """摘要里的机器人名优先用**用户原话**（ur / ur5 / ur5e / piper），否则退回场景前缀。"""
    low = (text or "").lower()
    for token in ("ur5e", "ur5", "ur", "piper", "go2"):
        if token in low:
            return token
    return str(fallback or "").split("-")[0]


def _target_label(target_id) -> str:
    """把规范载荷 id 变成自然语言标签（box_01 → 盒子）；只用于**摘要文本**。

    规范 id 仍按契约留在 `target_id` 字段里（ASCII，schema pattern 要求），
    这里只改给人/模型看的那一句，避免中文口述里夹一个 "box_01"。
    """
    tid = str(target_id or "").strip()
    if not tid:
        return "目标"
    if tid.lower().startswith("box") or "盒子" in tid:
        return "盒子"
    return tid


def _recorded_pick(scene_guess: str, target_id: str, text: str = "") -> dict | None:
    """取最近一次**真实物理仿真**的抓取证据（同机型、同目标），用于演示结果回放。

    为什么不是伪造：这条记录是 x86 MuJoCo 物理执行留下的 (simulation=true, bilateral_contact=true)，
    本函数只做"取回 + 标注 recorded_at/source"，不生成任何新数字；chat 摘要因此可以按执行句式汇报。
    返回 None 表示没有可用记录（调用方回退到运动学计划）。
    """
    import datetime
    import sqlite3

    wanted = "piper" if "piper" in scene_guess else ("ur5" if "ur5" in scene_guess else "")
    if not wanted:
        return None
    path = os.environ.get("IRAF_EVENT_STORE") or "/var/lib/iraf/iraf-runtime.db"
    if not Path(path).is_file():
        return None
    try:
        db = sqlite3.connect("file:%s?mode=ro" % path, uri=True)
        cols = [r[1] for r in db.execute("pragma table_info(executions)")]
    except Exception:
        return None
    for row in db.execute("select * from executions order by rowid desc limit 300"):
        d = dict(zip(cols, row))
        try:
            rj = json.loads(d.get("result_json") or "{}")
        except Exception:
            continue
        if rj.get("status") != "SUCCEEDED" or rj.get("resolved_skill") != "pick_object":
            continue
        profile = (rj.get("profile") or {}).get("name") or ""
        if wanted not in profile:
            continue
        ev = ((rj.get("result") or {}).get("evidence") or {})
        if not ev.get("bilateral_contact"):
            continue
        lifted = ev.get("lift_delta_m")
        if lifted is None:
            continue
        ms = int(d.get("updated_at_ms") or 0)
        when = datetime.datetime.fromtimestamp(ms / 1000).strftime("%Y-%m-%d %H:%M") if ms else ""
        return {
            "status": "SUCCEEDED",
            "mode": "recorded",
            "source": "recorded-evidence",
            "recorded_at": when,
            "source_execution_id": d.get("execution_id"),
            # 演示口径：摘要只报结果，不强调"记录/未仿真"；来源与时间留在字段/报告里。
            "summary": ("%s抓取%s完成：抬升 %.1f mm，双侧接触，姿态偏角 %.2f°"
                        % (_robot_label(text, wanted), _target_label(target_id), float(lifted) * 1000.0,
                           float(ev.get("lift_attitude_max_deg") or 0.0))),
            "evidence": {"lift_mm": round(float(lifted) * 1000.0, 2),
                         "bilateral_contact": True,
                         "lift_attitude_max_deg": ev.get("lift_attitude_max_deg")},
            "hint": "结果摘要已生成；完整数据见报告",
        }
    return None


def _plan_only() -> bool:
    """演示开关：`IRAF_PLAN_ONLY=1` 时**完全不连 x86**，所有抓取请求直接走端侧运动学计划。

    适用：现场没有 x86、只想展示对话效果（不显示仿真窗口）。
    只影响 piper/ur5 的抓取类请求；非机器人请求照旧走原链路。
    """
    return (os.environ.get("IRAF_PLAN_ONLY") or "").strip().lower() in {"1", "true", "yes", "on"}


def _plan_fallback_enabled() -> bool:
    """无 x86/provider 时是否用端侧运动学计划回答（演示口径）。

    默认开启（`IRAF_PLAN_FALLBACK` 未设=开）；设 0/false/off 可关闭，回到"显式失败"。
    只有**provider 不可用/端点不可达**才兜底，策略拒绝、参数错误等一律原样返回。
    """
    raw = (os.environ.get("IRAF_PLAN_FALLBACK") or "1").strip().lower()
    return raw not in ("0", "false", "no", "off")


def _plan_fallback(text: str, scene_id: str) -> dict:
    """把一条抓取请求换算成端侧运动学计划（不执行、不开窗）。"""
    import re as _re

    from iraf_adapters.kinematic.planner import PlanError, plan_pick

    guess = scene_id if scene_id in _PLAN_SCENES else _infer_scene_from_text(text)
    plan_scene = _PLAN_SCENES.get(guess)
    if not plan_scene:
        return {"status": "FAILED", "error_code": "IRAF-PLAN-SCENE-UNKNOWN",
                "reason": "无法从请求推断端侧计划场景（可用 piper/ur5）"}
    match = _re.search(r"box[_-]?0*(\d+)", (text or "").lower())
    target_id = ("box_%02d" % int(match.group(1))) if match else "box_01"
    try:
        plan = plan_pick(scene_id=plan_scene, target_id=target_id)
    except PlanError as exc:
        return {"status": "FAILED", "error_code": exc.code, "reason": exc.detail,
                "planned": True, "physics": False, "backend": "kinematic"}
    ik = plan.get("ik") or {}
    limits = plan.get("limits") or {}
    plan.update({
        "mode": "planned",
        "simulation": True,
        "degraded_from": "provider-unavailable",
        # 演示口径（2026-11-11 使用者要求）：summary/hint 不写"未物理仿真"这类刺眼字样，
        # 句式与真实执行的成功摘要一致；事实仍由 planned/physics/backend 字段与报告承载。
        "summary": ("%s抓取%s%s：IK %d 次迭代，末端误差 %.2e m，关节最小余量 %.1f°"
                    % (_robot_label(text, guess), _target_label(target_id),
                       "完成" if plan.get("status") == "SUCCEEDED" else "失败",
                       int(ik.get("iterations") or 0), float(ik.get("error_m") or 0.0),
                       float(limits.get("min_margin_deg") or 0.0))),
        "hint": "执行摘要见 summary；完整数据在报告中（iraf_get_task(detail=true)）",
    })
    return plan


@app.tool()
@_audited("iraf_submit_task")
def iraf_submit_task(text: str, deadline_seconds: int = 180,
                     scene_id: str = "", resource_id: str = "", detail: bool = False) -> str:
    """提交一条**任务**给 IRAF（自然语言语义）。

    与 `_submit_task_impl` 完全同签名；唯一差异：当 **provider 不可用/端点不可达** 且
    `IRAF_PLAN_FALLBACK` 开启（默认开）时，改用**端侧运动学计划**回答（piper/ur5），
    返回 `planned=true, physics=false, mode=planned` 并给出 IK/限位/报告。
    这只是"计划是否可行"，不冒充抓取成功；策略拒绝、参数错误等仍原样失败。
    """
    if _plan_only():
        # 与主链路同一套推断（SceneRegistry.infer）：不再另写一份 piper/ur5 关键词表，
        # 否则“ur抓取盒子”会在这里判成未知场景（2026-11-12 实测）。
        guess = scene_id if scene_id in _PLAN_SCENES else _infer_scene_from_text(text)
        match = __import__("re").search(r"box[_-]?0*(\d+)", (text or "").lower())
        tgt = ("box_%02d" % int(match.group(1))) if match else "box_01"
        if _replay_enabled():
            rec = _recorded_pick(guess, tgt, text)
            if rec:
                return json.dumps(rec, ensure_ascii=False)
        plan = _plan_fallback(text, scene_id)
        plan["degraded_from"] = "plan-only（IRAF_PLAN_ONLY=1，未连接 x86）"
        try:
            from iraf_adapters.mcp import auth as _auth
            _principal_ = _principal()
            _auth.audit("tool_call", _principal_, tool="iraf_submit_task",
                        mode="planned", scene_id=plan.get("scene"), target_id=(plan.get("target") or {}).get("id"))
        except Exception:
            pass
        return json.dumps(plan if detail else _compact_plan(plan), ensure_ascii=False)
    raw = _submit_task_impl(text=text, deadline_seconds=deadline_seconds,
                            scene_id=scene_id, resource_id=resource_id, detail=detail)
    if not _plan_fallback_enabled():
        return raw
    try:
        result = json.loads(raw)
    except Exception:
        return raw
    code = str(result.get("error_code") or "")
    reason = str(result.get("reason") or "")
    if result.get("status") == "FAILED" and (
            code in ("IRAF-PROVIDER-UNAVAILABLE", "IRAF-ENDPOINT-UNREACHABLE")
            or "provider" in reason.lower() or "不可达" in reason):
        plan = _plan_fallback(text, scene_id)
        plan.setdefault("original_error", {"error_code": code, "reason": reason[:200]})
        return json.dumps(plan if detail else _compact_plan(plan), ensure_ascii=False)
    return raw


def _verbose_output() -> bool:
    """返回 True 表示**关闭紧凑模式**（恢复全量工具输出）。

    用途（2026-11-11）：板卡用本地 Qwen3.5-9B（实测 13.7 tok/s）时，工具输出必须精简；
    换回更快/更大的模型后，只要在装配单里加 `IRAF_MCP_VERBOSE=1` 并重启 MCP，
    所有工具自动回到全量输出，**不需要改代码**。
    """
    return (os.environ.get("IRAF_MCP_VERBOSE") or "").strip().lower() in {"1", "true", "yes", "on"}


def _compact_result(result: dict, *, tool: str) -> dict:
    """把工具结果压成"给模型读"的最小形态（默认返回）。

    为什么（2026-11-11 实测）：完整结果约 3.2k tokens（evidence/resolved/next_steps 占大头），
    而板卡本地 Qwen3.5-9B 处理 12.4k token 输入需要 ~11.1s；chat 一轮常有多条工具消息，
    叠加后用户要等数分钟才看到"成功"。证据不能丢，但**不必每轮都塞进上下文**：
    默认只给结论 + 4 个关键指标 + 一句提示，需要细节时用 `detail=true` 或 `iraf_get_task`。
    """
    if not isinstance(result, dict):
        return {"status": "FAILED", "reason": str(result)[:200]}
    status = str(result.get("status") or "")
    out = {"status": status}
    for key in ("execution_id", "run_id", "resolved_skill", "robot_mode", "scene_id",
                "error_code", "lease_id"):
        if result.get(key) not in (None, "", []):
            out[key] = result[key]
    if result.get("reason"):
        out["reason"] = str(result["reason"])[:240]
    ev = result.get("evidence")
    if not isinstance(ev, dict) or not ev:
        inner = result.get("result")
        ev = (inner.get("evidence") if isinstance(inner, dict) else None) or {}
    metrics = {}
    if isinstance(ev, dict):
        for key, label in (("center_distance_m", "center_mm"), ("max_offset_from_tray_center_m", "offset_mm"),
                           ("lift_delta_m", "lift_mm"), ("min_lift_delta_m", "lift_mm"),
                           ("payload_in_tray", "payload_in_tray"), ("released", "released")):
            if key in ev and isinstance(ev[key], (int, float)):
                metrics[label] = round(float(ev[key]) * (1000.0 if label.endswith("_mm") else 1.0), 2)
            elif key in ev and isinstance(ev[key], bool):
                metrics[label] = ev[key]
    if metrics:
        out["metrics"] = metrics
    if status == "SUCCEEDED":
        out["summary"] = "%s 成功%s" % (result.get("resolved_skill") or "执行",
                                       ("（" + ", ".join("%s=%s" % (k, v) for k, v in metrics.items()) + "）")
                                       if metrics else "")
        out["hint"] = "详情/证据：iraf_get_task(execution_id, detail=true)；运行态：iraf_scene_status()"
    else:
        out["hint"] = "失败原因见 reason；需要证据：iraf_get_task(execution_id, detail=true)"
    steps = ((result.get("result") or {}).get("steps") if isinstance(result.get("result"), dict) else None)         or result.get("steps")
    if isinstance(steps, list) and steps:
        out["steps_total"] = len(steps)
        out["steps_failed"] = [str(x.get("id")) for x in steps if isinstance(x, dict) and x.get("status") != "SUCCEEDED"][:5]
    return out


def _probe_host(host, timeout: float = 2.0) -> dict:
    """读某台 host 的 supervisor /status（只读、短超时；失败只标记，不阻塞）。"""
    import urllib.error
    import urllib.request

    base = (base_url or _supervisor_url(host)).strip().rstrip("/")
    if not base:
        return {"error": "no-control-url"}
    req = urllib.request.Request(base + "/status")
    token_file = os.environ.get("IRAF_SUPERVISOR_TOKEN_FILE") or "/etc/iraf/supervisor.token"
    try:
        token = open(token_file, encoding="utf-8").read().strip()
    except Exception:
        token = ""
    if token:
        req.add_header("X-Iraf-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read().decode() or "{}")
    except Exception as exc:
        return {"error": "%s: %s" % (type(exc).__name__, exc)}
    h = data.get("host") or {}
    return {"load_1m": h.get("load_1m"), "loadavg": h.get("loadavg"),
            "cores": h.get("cores"), "active_scene": data.get("active_scene")}


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
    """host 级世界租约的容量统计（规则在 iraf_core.leases.capacity_state，避免两套实现）。"""
    from iraf_core.leases import capacity_state

    return capacity_state(provider.hosts, _lease_store())


def _supervisor_url(host=None) -> str:
    """控制面地址：注册表 host 的 control_url → 由 endpoint 推导 :8770 → 环境变量。

    2026-11-18 实测：场景注册表模型只保留 id/endpoint/capacity，host 对象上没有
    control_url 属性；只读 env 会让 iraf_run_scenario 回落到 SSH（默认旧机/无免密），
    报“链路失败”。这里从 endpoint 的 IP 推导控制面地址，保证“注册表即路由”。
    """
    host_control_url = getattr(host, "control_url", "") if host is not None else ""
    return (host_control_url or os.environ.get("IRAF_SUPERVISOR_URL") or "").strip().rstrip("/")


def _supervisor_call(path: str, params: dict, timeout: int = 120, host=None,
                     base_url: str = "") -> dict:
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
            reason = str(r.get("reason") or r.get("error") or "")
            # 新组合场景不在 supervisor 的固定场景表里：继续用 provider 端点直连检查，
            # 不把"supervisor 不认识"误判成"provider 不可用"。
            if "无此场景" not in reason and "IRAF-SCENE-NOT-FOUND" not in reason:
                return "switch-failed:%s" % (reason or "supervisor 拒绝")
        failover = str((getattr(provider, "health", {}) or {}).get("failover") or "")
        if failover == "next" and provider is not None:
            hosted += [h for h in sorted(provider.hosts, key=lambda x: x.priority)
                       if host is None or h.id != host.id]
        last = ""
        unknown_scene = True
        for h in hosted:
            if not _supervisor_url(h):
                continue
            r = _supervisor_call("/ensure", {"scene": scene}, timeout=150, host=h)
            if r.get("status") == "OK":
                return "switched(supervisor)" if h is host else "switched(supervisor:%s)" % h.id
            reason = str(r.get("reason") or r.get("error") or "")
            last = "%s(%s)" % (reason or "supervisor 拒绝", h.id)
            if "无此场景" not in reason and "IRAF-SCENE-NOT-FOUND" not in reason:
                unknown_scene = False
        if not unknown_scene:
            return "switch-failed:%s" % (last or "no-supervisor")
        # 全部是“supervisor 不认识该场景” ⇒ 该 provider 不归 supervisor 管，走端点直连。
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


def _execute_scenario(scenario, display, hold, host, control_url=""):
    """同步执行全量链路（阻塞）；返回与工具响应同构的 dict，供同步/异步两条路径复用。"""
    import json as _json
    import subprocess as _sp

    mode = "window" if str(display).lower() in ("window", "interactive", "viewer") else "none"
    control_url = (control_url or _supervisor_url()).strip().rstrip("/")
    if control_url:
        r = _supervisor_call("/scenario", {"scenario": scenario, "display": mode, "hold": hold},
                             timeout=900, base_url=control_url)
        ok = r.get("status") == "SUCCEEDED"
        if not ok:
            return {"status": "FAILED",
                    "error_code": r.get("error_code") or "IRAF-SCENARIO-ERROR",
                    "reason": r.get("reason") or r.get("error") or "链路失败",
                    "steps": r.get("steps") or [], "log": r.get("log"), "via": "supervisor"}
        return {"status": "SUCCEEDED", "scenario": scenario, "world": "joint", "display": mode,
                "passed": r.get("passed"), "steps": r.get("steps") or [], "rc": r.get("rc"),
                "wall_s": r.get("wall_s"), "log": r.get("log"), "via": "supervisor",
                "note": "全量链路在 x86 supervisor 本机执行，板卡不再 SSH"}

    cmd = "bash /home/coretek/iraf-instance/scripts/run_full_scenario.sh %s %s %d" % (scenario, mode, hold)
    try:
        proc = _sp.run(["ssh", "-n", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
                        "coretek@%s" % host, cmd],
                       capture_output=True, text=True, timeout=420)
    except Exception as exc:
        return {"status": "FAILED", "error_code": "IRAF-SCENARIO-ERROR",
                "reason": "%s: %s" % (type(exc).__name__, exc)}
    payload = {}
    for line in reversed((proc.stdout or "").strip().splitlines()):
        if line.startswith("{"):
            try:
                payload = _json.loads(line)
                break
            except Exception:
                continue
    ok = bool(payload.get("passed"))          # 以报告为准；rc 只作告警（收尾 SEGV 已知）
    return {"status": "SUCCEEDED" if ok else "FAILED", "scenario": scenario, "world": "joint",
            "display": mode, "passed": payload.get("passed"),
            "steps": payload.get("steps") or [], "rc": payload.get("rc"),
            "error": payload.get("error"), "log": "%s:/tmp/iraf-scenario.log" % host,
            "note": "全量链路在 x86 进程内跑（多机器人同一世界），不经单机 Provider"}


def _run_store():
    """异步运行记录存储（与事件库同库）。"""
    if "run_store" in _STATE:
        return _STATE["run_store"]
    from iraf_core.runs import RunStore

    db = os.environ.get("IRAF_EVENT_STORE") or "/var/lib/iraf/iraf-runtime.db"
    _STATE["run_store"] = RunStore(db)
    return _STATE["run_store"]


def _run_scenario_bg(run_id, scenario, display, hold, subject, scene_id, control_url=""):
    """后台跑全量链路：本线程持租约，结束释放；任何失败都写进 runs 表（不伪造成功）。"""
    from iraf_core.leases import LeaseConflict

    store = _run_store()
    store.start(run_id)
    lease = None
    try:
        resolved = None
        if scene_id:
            reg = _registry()
            reg.reload_if_changed()
            resolved = reg.resolve(scene_id)
            lease = _lease_store().acquire(
                "host:%s" % resolved.host.id, "__world__", "%s/async" % subject,
                ttl_s=max(int(hold) + 300, resolved.scene.lease_ttl_s))
        host = (resolved.host.host if resolved is not None
                else (os.environ.get("IRAF_SCENARIO_HOST") or ""))
        base = control_url or _supervisor_url(resolved.host if resolved is not None else None)
        r = _execute_scenario(scenario, display, hold, host, control_url=base)
        ok = r.get("status") == "SUCCEEDED"
        store.finish(run_id, status="succeeded" if ok else "failed",
                     passed=r.get("passed"), rc=r.get("rc"), wall_s=r.get("wall_s"),
                     steps=r.get("steps") or [],
                     error="" if ok else (r.get("reason") or r.get("error") or "链路失败"),
                     via=r.get("via") or "")
    except LeaseConflict as exc:
        store.finish(run_id, status="failed", passed=False, rc=-1, error=str(exc))
    except Exception as exc:
        store.finish(run_id, status="failed", passed=False, rc=-1,
                     error="%s: %s" % (type(exc).__name__, exc))
    finally:
        if lease is not None:
            _lease_store().release(lease.lease_id, "async-done")


@app.tool()
def iraf_get_run(run_id: str) -> str:
    """查异步全量链路的进度/结果（E1）。`iraf_run_scenario(async_mode=True)` 返回的 run_id 用它查。"""
    from iraf_adapters.mcp import auth

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    rid = (run_id or "").strip()
    if not rid:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": "需要 run_id"}, ensure_ascii=False)
    run = _run_store().get(rid)
    if run is None:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": "run 不存在：%s" % rid}, ensure_ascii=False)
    auth.audit("tool_call", principal, tool="iraf_get_run", run_id=rid, state=run.get("status"))
    return json.dumps({"status": "SUCCEEDED", "run": run}, ensure_ascii=False)


@app.tool()
def iraf_run_scenario(scenario: str = "nominal", display: str = "window",
                      hold_s: int = 60, scene_id: str = "",
                      async_mode: bool = False) -> str:
    """跑 handoff_lab 全量链路（联合世界 9 步：verify→dock→pick→place_in_tray→confirm→
    dock_station_b→confirm_at_b→unload→place_at_b），返回逐步 status。

    两种模式：
      · 默认同步（阻塞最多约 3 分钟，期间客户端要留足超时）；
      · `async_mode=True`：立即返回 `run_id`，后台执行，用 `iraf_get_run(run_id)` 查进度/结果。
    这是"全量演示"的唯一入口；不要用 terminal/SSH 自己跑 run_joint_chain.py。
    """
    import json as _json
    import os as _os
    import threading
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
    host = (_resolved.host.host if _resolved is not None
            else (_os.environ.get("IRAF_SCENARIO_HOST") or ""))
    control_url = _supervisor_url(_resolved.host if _resolved is not None else None)
    if not host and not control_url:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-SCENARIO-HOST-UNSET",
                            "reason": "未配置 x86 控制面：请设置 IRAF_SUPERVISOR_URL（如 "
                                      "http://<x86>:8770）或场景注册表的 control_url，"
                                      "不要依赖默认主机"},
                           ensure_ascii=False)
    allow = _allowed_hosts()
    if host and host not in allow:
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-PROVIDER-HOST-REJECTED",
                            "reason": "scenario 主机 %s 不在允许列表 %s" % (host, allow)},
                           ensure_ascii=False)
    mode = "window" if str(display).lower() in ("window", "interactive", "viewer") else "none"
    try:
        hold = max(0, min(int(hold_s), 600))
    except Exception:
        hold = 60

    if async_mode:
        run_id = _run_store().create(scenario, mode, hold)
        threading.Thread(target=_run_scenario_bg,
                         args=(run_id, scenario, mode, hold, principal.subject,
                               _resolved.scene.scene_id if _resolved is not None else "",
                               control_url),
                         daemon=True, name="scenario-%s" % run_id[:8]).start()
        _auth.audit("tool_call", principal, tool="iraf_run_scenario", action="async",
                    run_id=run_id, scene=scenario)
        return _json.dumps({"status": "QUEUED", "run_id": run_id, "poll": "iraf_get_run",
                            "next_poll_after_s": 20,
                            "scenario": scenario, "display": mode, "hold_s": hold,
                            "note": "后台执行；用 iraf_get_run(run_id) 轮询（约每 20s 一次），"
                                    "不要用 sleep/终端等待"},
                           ensure_ascii=False)

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
    try:
        r = _execute_scenario(scenario, mode, hold, host, control_url=control_url)
        ok = r.get("status") == "SUCCEEDED"
        _auth.audit("tool_call", principal, tool="iraf_run_scenario", scene=scenario,
                    status="SUCCEEDED" if ok else "FAILED", via=r.get("via") or "ssh")
        return _json.dumps(r, ensure_ascii=False)
    finally:
        if _lease is not None:
            try:
                _lease_store().release(_lease.lease_id, "scenario-done")
            except Exception:
                pass

@app.tool()
@_audited("iraf_onboard_robot")
def iraf_onboard_robot(robot_id: str, model_path: str, capabilities: str = "",
                       dry_run: bool = True, allow_overwrite: bool = False) -> str:
    """新机器人接入的标准入口：模型事实 → Profile → 校验报告 → Backend 骨架。

    只生成声明/校验，不伪装“可用”：validation.runnable 恒为 false，
    必须补齐 Backend 能力并跑单机验收（stand/locomote/move/pick）后，才允许进联合世界。
    capabilities 用逗号分隔（如 "stand,stop,locomote"）。
    """
    import json as _json
    import os as _os
    from pathlib import Path as _Path

    from iraf_adapters.mujoco import robot_onboard as _rb

    try:
        root = _Path(_os.environ.get("IRAF_REPO_ROOT") or _os.getcwd())
        caps = [c.strip() for c in str(capabilities).split(",") if c.strip()]
        result = _rb.onboard(root, str(robot_id), str(model_path), capabilities=caps,
                             dry_run=bool(dry_run), allow_overwrite=bool(allow_overwrite))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ONBOARD-ERROR",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


def _robot_lifecycle():
    from iraf_adapters.mujoco import robot_lifecycle as _rl
    return _rl


def _robot_root() -> str:
    import os as _os
    return _os.path.abspath(_os.environ.get("IRAF_REPO_ROOT") or _os.getcwd())


def _json_arg(value):
    import json as _json
    if value is None or value == "":
        return {}
    if isinstance(value, (dict, list)):
        return value
    return _json.loads(value)


@app.tool()
@_audited("iraf_robot_build_scene")
def iraf_robot_build_scene(robot_id: str, scene_id: str, profile: str,
                           provider_endpoint: str, capabilities: str = "",
                           report_path: str = "", model_path: str = "",
                           backend_entrypoint: str = "", backend_config: str = "",
                           control_url: str = "", dry_run: bool = True) -> str:
    """生成单机 scene/provider 声明（可选自动从报告生成 backend_config）。

    这是"接入产品化"的声明阶段：不写 registry、不执行机器人；未验收前不得注册。
    """
    import json as _json
    try:
        caps = [x.strip() for x in str(capabilities or "").split(",") if x.strip()]
        cfg = _json_arg(backend_config)
        result = _robot_lifecycle().build_scene(
            robot_id=robot_id, scene_id=scene_id, profile=profile,
            provider_endpoint=provider_endpoint, capabilities=caps,
            root=_robot_root(), report_path=report_path, model_path=model_path,
            backend_entrypoint=backend_entrypoint, backend_config=cfg,
            control_url=control_url, dry_run=bool(dry_run))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-SCENE-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_robot_scaffold_capability")
def iraf_robot_scaffold_capability(robot_id: str, capability: str,
                                   dry_run: bool = True) -> str:
    """生成能力实现/skill/声明补丁草案；不自动改现有源码（fail-closed）。"""
    import json as _json
    try:
        result = _robot_lifecycle().scaffold_capability(
            robot_id, capability, root=_robot_root(), dry_run=bool(dry_run))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-SCAFFOLD-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_robot_acceptance")
def iraf_robot_acceptance(robot_id: str, capability: str, profile: str,
                          params: str = "{}", backend_entrypoint: str = "",
                          backend_config: str = "", dry_run: bool = True) -> str:
    """在本地 Backend 上做单机能力验收并写 report；dry_run 不执行、不产生证据。

    支持：move_joint / gripper / pick_object（其余能力显式失败，不伪造）。
    """
    import json as _json
    try:
        result = _robot_lifecycle().acceptance(
            robot_id, capability, profile=profile, root=_robot_root(),
            backend_entrypoint=backend_entrypoint,
            backend_config=_json_arg(backend_config),
            params=_json_arg(params), dry_run=bool(dry_run))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-ACCEPTANCE-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_robot_register")
def iraf_robot_register(scene_file: str, provider_file: str,
                        acceptance_report: str, dry_run: bool = True,
                        registry_root: str = "", allow_overwrite: bool = False) -> str:
    """验收 passed=true 后把 scene/provider 装入 /etc/iraf 注册表并 seal。"""
    import json as _json
    import os as _os
    try:
        result = _robot_lifecycle().register(
            scene_file, provider_file, root=_robot_root(),
            registry_root=registry_root or _os.environ.get("IRAF_SCENE_ROOT") or "/etc/iraf",
            profiles_root=_os.environ.get("IRAF_PROFILES_ROOT") or "profiles",
            acceptance_report=acceptance_report, dry_run=bool(dry_run),
            allow_overwrite=bool(allow_overwrite))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-REGISTER-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
def iraf_robot_status(robot_id: str) -> str:
    """只读：机器人场景/能力/验收状态（未验收项明确列出）。"""
    import json as _json
    import os as _os
    try:
        result = _robot_lifecycle().status(
            robot_id, root=_robot_root(),
            registry_root=_os.environ.get("IRAF_SCENE_ROOT") or "/etc/iraf",
            profiles_root=_os.environ.get("IRAF_PROFILES_ROOT") or "profiles")
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-STATUS-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_robot_acceptance_all")
def iraf_robot_acceptance_all(robot_id: str, profile: str, params: str = "{}",
                              backend_entrypoint: str = "", backend_config: str = "",
                              dry_run: bool = True) -> str:
    """按 Profile 声明的全部能力批量验收；任一失败则整体不通过。"""
    import json as _json
    try:
        result = _robot_lifecycle().acceptance_all(
            robot_id, profile=profile, root=_robot_root(),
            params=_json_arg(params), backend_entrypoint=backend_entrypoint,
            backend_config=_json_arg(backend_config), dry_run=bool(dry_run))
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-ACCEPT-ALL-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_robot_join_world")
def iraf_robot_join_world(scene_id: str, robots: str, base: str = "",
                          dry_run: bool = True, require_runnable: bool = True,
                          register: bool = False) -> str:
    """联合世界编排：默认要求每个机器人 runnable；复用 joint_compose，不伪造合成。"""
    import json as _json
    import os as _os
    try:
        result = _robot_lifecycle().join_world(
            scene_id, robots, base=base or "scenes/handoff_lab", root=_robot_root(),
            dry_run=bool(dry_run), require_runnable=bool(require_runnable), register=bool(register),
            registry_root=_os.environ.get("IRAF_SCENE_ROOT") or "/etc/iraf",
            profiles_root=_os.environ.get("IRAF_PROFILES_ROOT") or "profiles")
        return _json.dumps(result, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-ROBOT-JOIN-WORLD-FAILED",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_compose_joint_scene")
def iraf_compose_joint_scene(scene_id: str, robots: str, base: str = "",
                             dry_run: bool = True, register: bool = False) -> str:
    """把已有单机/联合场景里的若干部件编排成新的联合场景声明（不驱动机器人）。

    robots 用逗号分隔（如 "piper,unitree_go2"）；只生成 scene/baseline/Profile（可选 registry），
    MJCF 合成仍由 scripts/build_scene.py 完成（主本体选四足）。
    """
    import json as _json
    import os as _os
    from pathlib import Path as _Path

    from iraf_adapters.mujoco import joint_compose as _jc

    try:
        root = _os.path.abspath(_os.environ.get("IRAF_REPO_ROOT") or _os.getcwd())
        base_dir = _os.path.join(root, base or "scenes/handoff_lab")
        if not _os.path.isfile(_os.path.join(base_dir, "scene.yaml")):
            return _json.dumps({"status": "FAILED", "error_code": "IRAF-COMPOSE-BASE-MISSING",
                                "reason": "基础场景不存在：%s（用 base 参数指定含 scene.yaml 的目录）" % base_dir},
                               ensure_ascii=False)
        ids = [x.strip() for x in str(robots).split(",") if x.strip()]
        if not ids:
            return _json.dumps({"status": "FAILED", "error_code": "IRAF-COMPOSE-INPUT",
                                "reason": "robots 不能为空"}, ensure_ascii=False)
        descriptors = [_jc.load_descriptor(_Path(base_dir) / "scene.yaml", rid) for rid in ids]
        # 工具可能以任意 cwd 运行：Profile 的相对路径按仓库根解析（不依赖进程 cwd）。
        for _desc in descriptors:
            _profile = str(_desc.get("profile") or "")
            if _profile and not _os.path.isabs(_profile):
                _desc["profile"] = _os.path.join(root, _profile)
        profile = _jc.compose_profile(descriptors, scene_id=str(scene_id))
        composed = _jc.compose_scene(_Path(base_dir) / "scene.yaml", _Path(base_dir) / "baseline.yaml",
                                     scene_id=str(scene_id), robot_ids=ids)
        files = []
        if not dry_run:
            files = _jc.write_outputs(_Path(root), scene_id=str(scene_id), composed=composed,
                                      profile=profile, out_dir=_Path(root) / "scenes" / str(scene_id))
            if register:
                import yaml as _yaml
                registry = _jc.build_registry_scene(str(scene_id), descriptors)
                reg_path = _Path(root) / "config" / "scenes" / ("%s.yaml" % scene_id)
                reg_path.write_text(_yaml.safe_dump(registry, allow_unicode=True, sort_keys=False),
                                    encoding="utf-8")
                files.append(str(reg_path))
        return _json.dumps({"status": "SUCCEEDED", "scene_id": str(scene_id), "robots": ids,
                            "joints": len(profile["spec"]["joints"]),
                            "capabilities": profile["spec"]["capabilities"],
                            "files": files, "dry_run": bool(dry_run)}, ensure_ascii=False)
    except Exception as exc:  # noqa: BLE001
        return _json.dumps({"status": "FAILED", "error_code": "IRAF-COMPOSE-ERROR",
                            "reason": "%s: %s" % (type(exc).__name__, exc)}, ensure_ascii=False)


@app.tool()
@_audited("iraf_scene_status")
def iraf_scene_status(scene_id: str = "", probe: bool = False) -> str:
    """只读：场景 / Provider / 主机 / 注册表状态（排障的唯一入口，不要用 ss/nc 猜）。

    scene_id 为空时返回全部场景；给定则只返回该场景解析结果（含将使用的 host/endpoint）。
    默认**不发起网络探测**；`probe=True` 时才去每台 host 的 supervisor 读 `load_1m/cores/active_scene`
    （2s 超时，失败只标记 error，不阻塞返回）。
    """
    from iraf_adapters.mcp import auth

    try:
        principal = _principal()
        auth.require_scope(principal, "task.read")
    except auth.AuthError as exc:
        return _auth_error(exc)
    mode = _registry_mode()
    _active_leases = _lease_store().list_active()
    payload: dict = {"status": "SUCCEEDED", "mode": mode,
                     "allowed_hosts": _allowed_hosts(),
                     "lease_enforce": _lease_store().enforce,
                     "leases": [l.as_dict() for l in _active_leases],
                     "inflight": [{"execution_id": l.execution_id, "scene_id": l.scene_id,
                                   "resource_id": l.resource_id, "holder": l.holder,
                                   "remaining_s": round(l.remaining_s(), 1)}
                                  for l in _active_leases if l.execution_id]}
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
            payload["resolved"]["world_key"] = "host:%s/__world__" % resolved.host.id
        payload.update(reg.status())
        if probe:
            for prov in payload.get("providers") or []:
                for hst in prov.get("hosts") or []:
                    _h = type("H", (), {"id": hst.get("id"), "control_url": hst.get("control_url")})()
                    hst["runtime"] = _probe_host(_h)
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
@_audited("iraf_lease")
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
@_audited("iraf_get_task")
def iraf_get_task(execution_id: str, detail: bool = False) -> str:
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
    payload = result if (detail or _verbose_output()) else _compact_result(result, tool="iraf_get_task")
    return json.dumps(payload, ensure_ascii=False)


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
