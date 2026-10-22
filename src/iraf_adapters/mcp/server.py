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


def _bridge():
    """惰性构建 TaskFlow 入口（bridge = 意图解析 + 任务分派），失败即抛。"""
    if "bridge" in _STATE:
        return _STATE["bridge"], _STATE["context"]
    from iraf_adapters.agentos.bridge import AgentOSBridge
    from iraf_adapters.agentos.cli import (build_context, build_intent_provider,
                                           resolve_subject)
    from iraf_adapters.agentos.dispatcher import TaskDispatcher
    from iraf_adapters.bootstrap import build_runtime_from_env

    provider = build_intent_provider()
    if provider is None:
        raise RuntimeError(
            "IRAF 侧意图 Provider 未配置（需要 IRAF_AGENTOS_MODEL_URL + IRAF_AGENTOS_MODEL）")
    dispatcher = TaskDispatcher(build_runtime_from_env())
    bridge = AgentOSBridge(provider, dispatcher)
    context = build_context(resolve_subject(None))
    _STATE["bridge"] = bridge
    _STATE["context"] = context
    return bridge, context


@app.tool()
def iraf_enter_robot_mode() -> str:
    """进入「机器人开发模式」（用户说"开始机器人开发模式"时必须调用本工具）。

    零参数、名字即语义：模型不需要在 enum 里选 action，听到这句中文就调它。
    等价于 iraf_robot_mode(action="enter")。返回 mode / 机器人 Skill 清单 / profile /
    安全策略 / 后端端点 / simulation 标记 / next_steps（进入后照做的固定顺序）。
    调用后**只做盘点并报告**，不要为了验证链路先提交一条任务（会弄脏仿真场景）。
    """
    return iraf_robot_mode(action="enter")


@app.tool()
def iraf_exit_robot_mode() -> str:
    """退出「机器人开发模式」（用户说"结束/退出机器人开发模式"时调用）。等价于 action="exit"。"""
    return iraf_robot_mode(action="exit")


from iraf_adapters.mcp.hints import (  # noqa: E402  (内容与 server 解耦，便于单测)
    FILTERED_EXAMPLE as _FILTERED_EXAMPLE,
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


@app.resource("iraf://skills/catalog")
def _resource_skill_catalog() -> str:
    """当前可用机器人 Skill 与意图示例（路由表；只读）。"""
    facts = _robot_facts()
    return json.dumps({"skills": facts.get("skills") or [],
                       "intents": facts.get("intents") or {},
                       "profile": facts.get("profile"),
                       "simulation": facts.get("simulation")}, ensure_ascii=False, indent=2)


@app.tool()
def iraf_robot_mode(action: str = "status", ttl_seconds: int = 1800) -> str:
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
    from iraf_adapters.mcp import mode as mode_mod

    act = (action or "status").strip().lower()
    st = mode_mod.state()
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
    payload.update(st.snapshot())
    payload.update(_robot_facts())
    # 把"进入之后照做什么"直接写进返回：实机证据（2026-10-22）是模型自己设计了
    # "只读自检 + 冒烟提交"两段式，冒烟那次把仿真场景弄脏。规则写在返回值里比写在文档里有效。
    payload["next_steps"] = _enter_next_steps()
    # 自带"正确用法示范"：模型不必自己摸索该怎么调，也减少它去翻源码/重写实现的动机。
    # 注意：examples 是**语义映射说明**，不是验收脚本；purpose 字段已明示"不要求执行"。
    payload["examples"] = {"robot_task": _WORKED_EXAMPLE, "non_robot": _FILTERED_EXAMPLE}
    return json.dumps(payload, ensure_ascii=False)


def _robot_facts() -> dict:
    """把装配单里的事实读出来（只读；失败就给 unverified，不猜）。"""
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
            for name, contract in registry.intent_contracts().items():
                intents[name] = list(contract.get("examples") or [])
            facts["intents"] = intents
        except Exception as exc:
            facts["skills_error"] = str(exc)[:200]
    facts["profile"] = _os.environ.get("IRAF_PROFILE")
    facts["safety_policy"] = _os.environ.get("IRAF_SAFETY_POLICY")
    facts["event_store"] = _os.environ.get("IRAF_EVENT_STORE")
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
def iraf_submit_task(text: str, deadline_seconds: int = 120) -> str:
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

    bridge, context = _bridge()
    try:
        request = build_request(text, deadline_seconds=int(deadline_seconds),
                                controller="agentos-mcp-task")
    except ValueError as exc:
        return json.dumps({"status": "FAILED", "error_code": "IRAF-INPUT-INVALID",
                           "reason": str(exc)}, ensure_ascii=False)
    result = bridge.execute_intent(request, context)
    # 把模式一并回传：平台据此知道"现在处理的是机器人任务域"，也便于审计
    # （不在模式里时给一句 hint，但不阻断执行——模式是提示性约定，不是安全边界）。
    from iraf_adapters.mcp import mode as mode_mod

    if isinstance(result, dict):
        result.setdefault("robot_mode", mode_mod.state().name())
        result.setdefault("next_steps", _next_steps(result))
    return json.dumps(result, ensure_ascii=False)


@app.tool()
def iraf_get_task(execution_id: str) -> str:
    """按 execution_id 查询 IRAF 事件库里的执行事实（权威来源，不是缓存）。"""
    store_path = os.environ.get("IRAF_EVENT_STORE") or "/var/lib/iraf/iraf-runtime.db"
    if not Path(store_path).is_file():
        return json.dumps({"status": "NOT_FOUND", "reason": "事件库不存在: %s" % store_path},
                          ensure_ascii=False)
    from iraf_core.store import SqliteExecutionStore

    result = SqliteExecutionStore(store_path).get(execution_id)
    if result is None:
        return json.dumps({"status": "NOT_FOUND", "execution_id": execution_id},
                          ensure_ascii=False)
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
        app.run(transport="streamable-http")
        return 0
    if transport not in ("stdio", ""):
        print(f"[iraf-mcp][错误] 不认识的 IRAF_MCP_TRANSPORT={transport}"
              "（允许 stdio / http）", file=sys.stderr, flush=True)
        return 2
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
