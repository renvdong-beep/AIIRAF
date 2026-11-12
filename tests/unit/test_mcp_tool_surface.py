# -*- coding: utf-8 -*-
"""MCP 对外工具面：只允许 iraf_* 公共工具（2026-11-12 实测 _submit_task_impl 被误暴露）。"""
import ast
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SERVER = REPO / "src/iraf_adapters/mcp/server.py"


def exported_tools() -> list:
    tree = ast.parse(SERVER.read_text(encoding="utf-8"))
    names = []
    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        for deco in node.decorator_list:
            text = ast.unparse(deco) if hasattr(ast, "unparse") else ""
            if text in ("app.tool()", "app.tool"):
                names.append(node.name)
    return names


class McpToolSurfaceTest(unittest.TestCase):
    def test_every_registered_tool_is_public(self):
        tools = exported_tools()
        self.assertGreaterEqual(len(tools), 5, "工具面扫描为空，AST 逻辑可能失效：%s" % tools)
        bad = [name for name in tools if not name.startswith("iraf_")]
        self.assertEqual([], bad, "内部函数被暴露成 MCP 工具：%s" % bad)

    def test_submit_impl_is_not_a_tool(self):
        self.assertNotIn("_submit_task_impl", exported_tools())

    def test_submit_task_is_exported(self):
        self.assertIn("iraf_submit_task", exported_tools())


if __name__ == "__main__":
    unittest.main()
