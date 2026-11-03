# -*- coding: utf-8 -*-
"""上位机 SDK 的 MCP 客户端适配器：只暴露 5 个任务级工具，且默认不引入 mcp 依赖。"""
import asyncio
import unittest

from iraf_sdk import SdkConfigError
from iraf_sdk.mcp import McpToolClient, StdioMcpToolClient


class McpAdapterTest(unittest.TestCase):
    def test_import_does_not_require_mcp_package(self):
        import sys

        self.assertIn("iraf_sdk.mcp", sys.modules)
        # 顶层导入不应加载可选依赖（纯标准库可安装）
        self.assertNotIn("mcp", sys.modules)

    def test_requires_endpoint_token_timeout(self):
        with self.assertRaises(SdkConfigError):
            McpToolClient("", token="t", timeout_s=30)
        with self.assertRaises(SdkConfigError):
            McpToolClient("http://127.0.0.1:8765/mcp", token="", timeout_s=30)
        with self.assertRaises(SdkConfigError):
            McpToolClient("http://127.0.0.1:8765/mcp", token="t", timeout_s=0)

    def test_stdio_requires_command(self):
        with self.assertRaises(SdkConfigError):
            StdioMcpToolClient("", args=[], timeout_s=30)

    def test_rejects_non_task_tool(self):
        client = McpToolClient("http://127.0.0.1:8765/mcp", token="t", timeout_s=30)

        async def call():
            await client._call("iraf_execute_skill", {})

        with self.assertRaises(SdkConfigError) as caught:
            asyncio.run(call())
        self.assertIn("任务级", str(caught.exception))

    def test_unconnected_call_fails_closed(self):
        client = McpToolClient("http://127.0.0.1:8765/mcp", token="t", timeout_s=30)

        async def call():
            await client.submit_task("抓取 box_01")

        with self.assertRaises(SdkConfigError):
            asyncio.run(call())

    def test_submit_requires_text(self):
        client = McpToolClient("http://127.0.0.1:8765/mcp", token="t", timeout_s=30)

        async def call():
            await client.submit_task("  ")

        with self.assertRaises(SdkConfigError):
            asyncio.run(call())


if __name__ == "__main__":
    unittest.main()
