#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨机 HTTP 探针：用 streamable-http 连板上的任务级 MCP server（替代 stdio 本地拉起）。

为什么需要它：stdio MCP 只能由**本机**宿主作为子进程拉起；语音/语义前端常常不在板上
（智控域实测 microphones=0、无 /dev/snd），跨机只能走网络传输。

用法（在 x86 开发机，用有 mcp 包的解释器）：
  build/mcp-venv/bin/python /tmp/iraf_mcp_http_probe.py \
      --url http://10.203.247.147:8765/mcp --text "抓取 box_01"

判据：TOOLS 里出现 iraf_submit_task / iraf_get_task；submit 返回 SUCCEEDED 且
get_task 读回同一 execution_id。
"""
import argparse
import asyncio
import json
import sys

from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client


async def main(url: str, text: str) -> int:
    async with streamablehttp_client(url) as (read, write, _):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            names = [t.name for t in tools.tools]
            print("TOOLS", names)

            res = await session.call_tool("iraf_submit_task", {"text": text})
            payload = res.content[0].text if res.content else ""
            data = json.loads(payload)
            print("status=", data.get("status"), "| error_code=", data.get("error_code"),
                  "| skill=", data.get("resolved_skill"),
                  "| execution_id=", data.get("execution_id"))
            if data.get("reason"):
                print("reason=", str(data["reason"])[:300])

            eid = data.get("execution_id")
            if not eid:
                return 2
            res2 = await session.call_tool("iraf_get_task", {"execution_id": eid})
            data2 = json.loads(res2.content[0].text if res2.content else "{}")
            same = data2.get("execution_id") == eid
            print("查回 status=", data2.get("status"), "| execution_id=", data2.get("execution_id"),
                  "| 同一条=", same)
            return 0 if (data.get("status") == "SUCCEEDED" and same) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://10.203.247.147:8765/mcp")
    ap.add_argument("--text", default="抓取 box_01")
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args.url, args.text)))
