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
def iraf_submit_task(text: str, deadline_seconds: int = 120) -> str:
    """提交一条**任务**给 IRAF（自然语言语义）。技能选择由 IRAF Skill Manager 完成。

    返回与 IRAF CLI 同构的 JSON：status / execution_id / error_code / reason / evidence。
    失败不会被包装成成功；缺 Provider、策略拒绝、精度不达标都会如实返回错误码。
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
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
