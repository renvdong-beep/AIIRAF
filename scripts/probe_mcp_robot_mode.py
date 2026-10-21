#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义触发验收：iraf_robot_mode(enter) → 状态 → iraf_submit_task → 回传 robot_mode。

用途：证明"开始机器人开发模式"这条语言触发在**接口层**真的把会话收进机器人任务域，
并且后续任务仍然只走 TaskFlow / Skill Manager（不是平台自己去编）。

用法（开发机，需 mcp 包；本仓库用 build/mcp-venv/bin/python）：
  build/mcp-venv/bin/python scripts/probe_mcp_robot_mode.py \
      --url http://10.203.247.147:8765/mcp --text "抓取 box_01"
"""
import argparse
import asyncio
import json
import sys

from mcp import ClientSession


async def _flow(session, text):
    await session.initialize()
    tools = [t.name for t in (await session.list_tools()).tools]
    print("TOOLS", tools)
    if "iraf_robot_mode" not in tools:
        print("✗ 没有 iraf_robot_mode（板上版本太旧？）")
        return 2

    print("\n---- status（进模式前）----")
    before = json.loads((await session.call_tool("iraf_robot_mode", {"action": "status"})).content[0].text)
    print("mode =", before.get("mode"), "| skills =", len(before.get("skills") or []),
          "| simulation =", before.get("simulation"))
    print("skills:", ", ".join(before.get("skills") or [])[:300])

    print("\n---- enter(ttl=600) ----")
    entered = json.loads((await session.call_tool(
        "iraf_robot_mode", {"action": "enter", "ttl_seconds": 600})).content[0].text)
    print("mode =", entered.get("mode"), "| remaining_s =", entered.get("remaining_s"))
    print("hint =", (entered.get("hint") or "")[:140])
    print("profile =", entered.get("profile"), "| policy =", entered.get("safety_policy"))
    print("provider_endpoint =", entered.get("provider_endpoint"))

    print("\n---- iraf_submit_task ----")
    data = json.loads((await session.call_tool("iraf_submit_task", {"text": text})).content[0].text)
    print("status=", data.get("status"), "| skill=", data.get("resolved_skill"),
          "| robot_mode=", data.get("robot_mode"), "| execution_id=", data.get("execution_id"))
    if data.get("reason"):
        print("reason=", str(data["reason"])[:200])

    print("\n---- exit ----")
    after = json.loads((await session.call_tool("iraf_robot_mode", {"action": "exit"})).content[0].text)
    print("mode =", after.get("mode"))

    ok = (before.get("mode") == "idle" and entered.get("mode") == "robot_dev"
          and data.get("robot_mode") == "robot_dev" and after.get("mode") == "idle")
    print("\n判读：mode 进/出与任务回传一致 =", ok)
    return 0 if ok else 1


async def _via_http(url, text):
    from mcp.client.streamable_http import streamablehttp_client

    async with streamablehttp_client(url) as (read, write, _):
        async with ClientSession(read, write) as session:
            return await _flow(session, text)


async def _via_stdio(text):
    from mcp import StdioServerParameters
    from mcp.client.stdio import stdio_client

    params = StdioServerParameters(command=sys.executable,
                                   args=["-m", "iraf_adapters.mcp.server"], env=None)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            return await _flow(session, text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://10.203.247.147:8765/mcp")
    ap.add_argument("--text", default="抓取 box_01")
    ap.add_argument("--stdio", action="store_true", help="stdio 形态（板上跑，需 PYTHONPATH）")
    args = ap.parse_args()
    if args.stdio:
        return asyncio.run(_via_stdio(args.text))
    return asyncio.run(_via_http(args.url, args.text))


if __name__ == "__main__":
    sys.exit(main())
