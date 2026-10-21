# -*- coding: utf-8 -*-
"""MCP server 的**层次**约束（静态检查，不需要 mcp/grpc）。

2026-10-20 使用者纠正：AgentOS 的 skills 与 IRAF Skill Manager 管的机器人 Skill
不是一个概念，不得互相注册；MCP 只能暴露**任务级**入口，不得暴露"逐个技能"的工具
（那等于让 AgentOS 绕过 Skill Manager 直连 skill.execute）。
这条用例就是防回潮：一旦有人加回 `iraf_execute_skill`/`iraf_list_skills`，CI 立刻红。
"""
import ast
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SERVER = REPO / "src/iraf_adapters/mcp/server.py"
FORWARDER = REPO / "scripts/iraf_mcp_server.py"

#: 允许的 MCP 工具名（任务级）
ALLOWED = {"iraf_submit_task", "iraf_get_task"}
#: 明确禁止的（技能级 / 能力级直连）
FORBIDDEN = {"iraf_execute_skill", "iraf_list_skills", "iraf_run_skill"}


def tool_names() -> set:
    tree = ast.parse(SERVER.read_text(encoding="utf-8"))
    names = set()
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            for dec in node.decorator_list:
                # 形如 @app.tool() 或 @app.tool
                target = dec.func if isinstance(dec, ast.Call) else dec
                if isinstance(target, ast.Attribute) and target.attr == "tool":
                    names.add(node.name)
    return names


class McpLayerTest(unittest.TestCase):
    def test_only_task_level_tools(self):
        names = tool_names()
        self.assertEqual(names, ALLOWED,
                         "MCP 只应暴露任务级工具；实际=%s" % sorted(names))

    def test_no_skill_level_tools(self):
        names = tool_names()
        self.assertFalse(names & FORBIDDEN,
                         "出现了技能级工具（会绕过 Skill Manager）：%s" % sorted(names & FORBIDDEN))

    def test_forwarder_defines_no_tools(self):
        """兼容入口不得自己定义工具（否则又会出现两套实现/两套层次）。"""
        tree = ast.parse(FORWARDER.read_text(encoding="utf-8"))
        defs = [n.name for n in tree.body
                if isinstance(n, ast.FunctionDef) and n.name.startswith("iraf_")]
        self.assertEqual(defs, [], "scripts/iraf_mcp_server.py 不应再定义工具：%s" % defs)

    def test_chdir_to_version_dir(self):
        """启动时必须把 cwd 切到 <version dir>：装配单里的 profiles/、skills/ 是相对路径。

        实机踩到（2026-10-21）：AgentOS 以 stdio 启动本 server 时 cwd 不是版本目录，
        `profiles/piper_mujoco.yaml` 打不开 ⇒ iraf_submit_task 直接失败。
        systemd 单元用 WorkingDirectory=<version dir> 表达同一事实，MCP 入口必须复现它。
        """
        tree = ast.parse(SERVER.read_text(encoding="utf-8"))
        helpers = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
        self.assertIn("_enter_version_dir", helpers,
                      "缺少 _enter_version_dir：MCP 入口未对齐单元的工作目录语义")
        helper = helpers["_enter_version_dir"]
        self.assertTrue(
            any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == "chdir" for n in ast.walk(helper)),
            "_enter_version_dir 里没有 os.chdir 调用")
        called = [n for n in tree.body
                  if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
                  and isinstance(n.value.func, ast.Name)
                  and n.value.func.id == "_enter_version_dir"]
        self.assertTrue(called, "模块顶层没有调用 _enter_version_dir（import 时不会切目录）")


if __name__ == "__main__":
    unittest.main()
