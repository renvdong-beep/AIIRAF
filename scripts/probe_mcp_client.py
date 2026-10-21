#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 MCP 客户端直连我们的 iraf-task server，验证两个任务级工具（不经过 AgentOS）。

用法（板上 root）：
  PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
  python3 /opt/iraf-extras/probe_mcp_client.py "抓取 box_01"
"""
import asyncio
import json
import os
import sys
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# server 启动方式与板上登记一致：`python3 -m iraf_adapters.mcp.server`（随包交付的实现）。
# 兼容旧登记：IRAF_MCP_SERVER=<脚本路径> 时改用该路径启动（只做转发，不应再定义工具）。
SERVER = os.environ.get("IRAF_MCP_SERVER") or ""
ARGS = [SERVER] if SERVER else ["-m", "iraf_adapters.mcp.server"]
PYTHON = sys.executable


async def main(text: str) -> int:
    params = StdioServerParameters(
        command=PYTHON,
        args=ARGS,
        env={
            **os.environ,
            "IRAF_PROVIDER_TOKEN": os.environ.get("IRAF_PROVIDER_TOKEN", "iraf-e300-probe-2026"),
            "IRAF_ENV_FILE": os.environ.get("IRAF_ENV_FILE", "/etc/iraf/edge-intel.env"),
            "IRAF_AGENTOS_MODEL_URL": os.environ.get("IRAF_AGENTOS_MODEL_URL",
                                                     "http://192.168.100.89:3000/v1"),
            "IRAF_AGENTOS_MODEL": os.environ.get("IRAF_AGENTOS_MODEL", "deepseek-flash"),
        },
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            print("TOOLS", [t.name for t in tools.tools])

            print("---- iraf_submit_task ----")
            res = await session.call_tool("iraf_submit_task", {"text": text})
            payload = res.content[0].text if res.content else ""
            try:
                data = json.loads(payload)
            except ValueError:
                print("非 JSON 返回:", payload[:300])
                return 2
            print("status=", data.get("status"),
                  "| error_code=", data.get("error_code"),
                  "| skill=", data.get("resolved_skill"),
                  "| execution_id=", data.get("execution_id"))
            if data.get("reason"):
                print("reason=", str(data["reason"])[:300])

            execution_id = data.get("execution_id")
            if execution_id:
                print("---- iraf_get_task ----")
                res2 = await session.call_tool("iraf_get_task", {"execution_id": execution_id})
                payload2 = res2.content[0].text if res2.content else ""
                try:
                    data2 = json.loads(payload2)
                    print("查回 status=", data2.get("status"),
                          "| execution_id=", data2.get("execution_id"))
                except ValueError:
                    print("非 JSON 返回:", payload2[:300])
            return 0 if data.get("status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "抓取 box_01")))
