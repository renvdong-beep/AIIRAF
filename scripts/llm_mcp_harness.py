#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最小 LLM harness：把 IRAF MCP 的 5 个任务级工具交给模型，由模型决定调用。

用途：证明"LLM → MCP → IRAF"这条链在真实模型上可用，而不是只用手写客户端。
不引入 AgentOS：只用 OpenAI 兼容的 /chat/completions + iraf_sdk.mcp 客户端。

用法：
  PYTHONPATH=src:build/generated/python \
  MCP_URL=http://127.0.0.1:8879/mcp MCP_TOKEN=... NEWAPI_KEY=... \
  python3 scripts/llm_mcp_harness.py \
      --prompt "开始机器人开发模式，只做盘点，不要提交任务" \
      --prompt "把这句话原样交给 IRAF 判断：今天天气怎么样"
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import urllib.request

from iraf_sdk.mcp import McpToolClient

SYSTEM = """你是接入 IRAF 机器人的开发助手。规则：
1. 用户说“开始/进入机器人开发模式”时，调用 iraf_enter_robot_mode。
2. 进入后先只读盘点，把 skills/intents/profile/simulation 如实报告，不要为了验证链路提交任务。
3. 凡可能涉及机器人本体或物理动作的请求，调用 iraf_submit_task(text=用户原话)，由 IRAF 选技能。
4. 返回 IRAF-NOT-ROBOT-TASK 表示与机器人无关，交回你自己处理，不要重试、不要包装成功。
5. 需要核对执行事实时调用 iraf_get_task(execution_id)，不要编造。
6. 不要用 shell/代码去驱动机器人；唯一入口是 iraf_submit_task。"""


def chat(model: str, messages: list, tools: list, *, timeout_s: float = 90.0) -> dict:
    url = os.environ["LLM_BASE_URL"].rstrip("/") + "/chat/completions"
    body = {"model": model, "messages": messages, "tools": tools,
            "tool_choice": "auto", "temperature": 0}
    request = urllib.request.Request(
        url, data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json",
                 "Authorization": "Bearer " + os.environ["LLM_API_KEY"]},
        method="POST")
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return json.loads(response.read().decode("utf-8"))


def openai_tools(mcp_tools: list) -> list:
    return [{"type": "function",
             "function": {"name": tool.name,
                          "description": (tool.description or "")[:1024],
                          "parameters": tool.inputSchema or {"type": "object"}}}
            for tool in mcp_tools]


async def run(prompts: list, model: str) -> int:
    async with McpToolClient(os.environ["MCP_URL"], token=os.environ["MCP_TOKEN"],
                             timeout_s=180) as client:
        tools = openai_tools((await client._session.list_tools()).tools)
        print("MCP_TOOLS", [t["function"]["name"] for t in tools])
        messages = [{"role": "system", "content": SYSTEM}]
        for prompt in prompts:
            print("\nUSER", prompt)
            messages.append({"role": "user", "content": prompt})
            for _ in range(6):
                reply = chat(model, messages, tools)
                message = reply["choices"][0]["message"]
                calls = message.get("tool_calls") or []
                if message.get("content"):
                    print("ASSISTANT", message["content"][:800])
                messages.append({"role": "assistant",
                                 "content": message.get("content") or "",
                                 "tool_calls": calls})
                if not calls:
                    break
                for call in calls:
                    name = call["function"]["name"]
                    args = json.loads(call["function"].get("arguments") or "{}")
                    print("TOOL_CALL", name, json.dumps(args, ensure_ascii=False)[:300])
                    result = await client._call(name, args)
                    print("TOOL_RESULT", json.dumps(result, ensure_ascii=False)[:600])
                    messages.append({"role": "tool", "tool_call_id": call["id"],
                                     "content": json.dumps(result, ensure_ascii=False)})
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prompt", action="append", required=True)
    parser.add_argument("--model", default=os.environ.get("LLM_MODEL", "deepseek-flash"))
    args = parser.parse_args()
    return asyncio.run(run(args.prompt, args.model))


if __name__ == "__main__":
    sys.exit(main())
