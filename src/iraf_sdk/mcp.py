# -*- coding: utf-8 -*-
"""IRAF 任务级 MCP 客户端适配器（上位机 harness / LLM agent 用）。

边界（与 `iraf_sdk.client` 一致，AGENTS 1.2 / 1.8）：
  · 只封装 5 个**任务级**工具：enter/exit/robot_mode/submit/get；
  · 不提供技能级工具（skill.execute/list），也不提供 Provider Invoke；
  · 身份只放在 HTTP `Authorization: Bearer` 头（或 stdio 的部署环境），
    工具参数里没有 caller/role/tenant；
  · 无自动重试，timeout_s 必填；失败返回原样 JSON（用 `iraf_sdk.raise_for_status` 判读）。

依赖：`mcp>=1.26,<2` 为可选依赖（`pip install intewell-iraf[mcp]`）。
本模块顶层只 import 标准库与 `iraf_sdk.errors`，不 import grpc/mujoco/iraf_core。

用法（streamable-http）：
    from iraf_sdk.mcp import McpToolClient

    async with McpToolClient("https://edge:8765/mcp", token=token, timeout_s=120) as client:
        print(await client.enter_robot_mode())
        result = await client.submit_task("抓取 box_01")
        print(await client.get_task(result["execution_id"]))
"""
from __future__ import annotations

import json
from typing import Mapping, Optional, Sequence

from .client import SdkConfigError, SdkTransportError

__all__ = ["McpToolClient", "StdioMcpToolClient"]

_TASK_TOOLS = frozenset({
    "iraf_enter_robot_mode", "iraf_exit_robot_mode", "iraf_robot_mode",
    "iraf_submit_task", "iraf_get_task",
})


def _require_text(value, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SdkConfigError("%s 必须是非空字符串（SDK 不提供隐式默认值）" % field)
    return value.strip()


def _require_timeout(value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise SdkConfigError("timeout_s 必须是 > 0 的秒数（禁止无超时等待）")
    return float(value)


def _decode(result) -> dict:
    """把 CallToolResult 的文本解码为字典；非 JSON/非对象显式失败。"""
    content = getattr(result, "content", None) or []
    text = content[0].text if content and hasattr(content[0], "text") else ""
    try:
        payload = json.loads(text)
    except ValueError as exc:
        raise SdkTransportError("MCP 工具返回不是 JSON（原始内容：%r）" % (text[:200],)) from exc
    if not isinstance(payload, Mapping):
        raise SdkTransportError("MCP 工具返回不是 JSON 对象：%r" % (text[:200],))
    return dict(payload)


class _BaseMcpClient:
    """公共工具调用层；子类只负责建立 (read, write) 传输。"""

    def __init__(self, *, timeout_s):
        self.timeout_s = _require_timeout(timeout_s)
        self._session = None
        self._stack = []

    async def _call(self, name: str, arguments: Optional[Mapping] = None) -> dict:
        if name not in _TASK_TOOLS:
            raise SdkConfigError("不是任务级工具，SDK 拒绝调用：%s" % name)
        if self._session is None:
            raise SdkConfigError("MCP 客户端未连接：请使用 async with")
        return _decode(await self._session.call_tool(name, dict(arguments or {})))

    async def list_tools(self) -> list:
        if self._session is None:
            raise SdkConfigError("MCP 客户端未连接：请使用 async with")
        tools = await self._session.list_tools()
        return sorted(tool.name for tool in tools.tools)

    async def enter_robot_mode(self) -> dict:
        return await self._call("iraf_enter_robot_mode")

    async def exit_robot_mode(self) -> dict:
        return await self._call("iraf_exit_robot_mode")

    async def robot_mode(self, action: str = "status", ttl_seconds: int = 1800) -> dict:
        if action not in {"enter", "exit", "status"}:
            raise SdkConfigError("action 只接受 enter/exit/status：%r" % (action,))
        return await self._call("iraf_robot_mode",
                                {"action": action, "ttl_seconds": int(ttl_seconds)})

    async def submit_task(self, text: str, deadline_seconds: Optional[int] = None) -> dict:
        arguments = {"text": _require_text(text, "text")}
        if deadline_seconds is not None:
            arguments["deadline_seconds"] = int(deadline_seconds)
        return await self._call("iraf_submit_task", arguments)

    async def get_task(self, execution_id: str) -> dict:
        return await self._call("iraf_get_task",
                                {"execution_id": _require_text(execution_id, "execution_id")})

    async def _close(self) -> None:
        for context in reversed(self._stack):
            try:
                await context.__aexit__(None, None, None)
            except Exception:  # noqa: BLE001 - 关闭失败不掩盖原始错误
                pass
        self._stack.clear()
        self._session = None


class McpToolClient(_BaseMcpClient):
    """streamable-http 传输（边缘侧 IRAF MCP，Bearer 鉴权）。"""

    def __init__(self, url: str, token: str, *, timeout_s: float,
                 headers: Optional[Mapping] = None):
        super().__init__(timeout_s=timeout_s)
        self.url = _require_text(url, "url")
        self._token = _require_text(token, "token")
        self._headers = {str(k): str(v) for k, v in (headers or {}).items()}

    async def __aenter__(self):
        try:
            from mcp import ClientSession
            from mcp.client.streamable_http import streamablehttp_client
        except ImportError as exc:
            raise SdkConfigError(
                "MCP 客户端需要可选依赖 mcp>=1.26,<2：pip install 'intewell-iraf[mcp]'"
            ) from exc
        headers = dict(self._headers)
        headers["Authorization"] = "Bearer " + self._token
        transport = streamablehttp_client(self.url, headers=headers, timeout=self.timeout_s)
        read, write, _ = await transport.__aenter__()
        self._stack.append(transport)
        session = ClientSession(read, write)
        self._session = await session.__aenter__()
        self._stack.append(session)
        await self._session.initialize()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self._close()
        return False


class StdioMcpToolClient(_BaseMcpClient):
    """stdio 传输（同机宿主/开发机，进程边界即信任边界）。"""

    def __init__(self, command: str, args: Sequence[str], *, timeout_s: float,
                 env: Optional[Mapping] = None, cwd: Optional[str] = None):
        super().__init__(timeout_s=timeout_s)
        self.command = _require_text(command, "command")
        self.args = [str(item) for item in args]
        self.env = {str(k): str(v) for k, v in (env or {}).items()} or None
        self.cwd = cwd

    async def __aenter__(self):
        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
        except ImportError as exc:
            raise SdkConfigError(
                "MCP 客户端需要可选依赖 mcp>=1.26,<2：pip install 'intewell-iraf[mcp]'"
            ) from exc
        params = StdioServerParameters(command=self.command, args=self.args,
                                       env=self.env, cwd=self.cwd)
        transport = stdio_client(params)
        read, write = await transport.__aenter__()
        self._stack.append(transport)
        session = ClientSession(read, write)
        self._session = await session.__aenter__()
        self._stack.append(session)
        await self._session.initialize()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self._close()
        return False
