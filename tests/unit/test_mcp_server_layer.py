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

#: 允许的 MCP 工具名（**任务级**：提交 / 查询 / 机器人开发模式开关）
#: iraf_robot_mode 不是"技能级"工具：它只读装配事实并置一个会话级模式标记，
#: 不选技能、不执行动作（2026-10-21 使用者口径：语义歧义用显式模式收进机器人任务域）。
ALLOWED = {"iraf_submit_task", "iraf_get_task", "iraf_robot_mode",
           "iraf_enter_robot_mode", "iraf_exit_robot_mode"}
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

    def test_robot_mode_returns_next_steps(self):
        """进入模式必须回传"照做什么"。

        实机（2026-10-22 dashboard /chat）：模型把"进入模式"自行补成"只读自检 + 冒烟提交"两段，
        冒烟那次把 MuJoCo 场景弄脏（后续必现 distance≈0.374435m）。返回值里缺 next_steps
        是诱因之一，这条用例防止再被删掉。
        """
        tree = ast.parse(SERVER.read_text(encoding="utf-8"))
        funcs = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
        self.assertIn("iraf_robot_mode", funcs, "缺少 iraf_robot_mode")
        keys = set()
        for node in ast.walk(funcs["iraf_robot_mode"]):
            if isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store):
                sl = node.slice
                if isinstance(sl, ast.Constant) and isinstance(sl.value, str):
                    keys.add(sl.value)
        self.assertIn("next_steps", keys,
                      "iraf_robot_mode 的返回里没有 next_steps：平台会自己设计下一步（可能先冒烟提交）")
        self.assertIn("examples", keys)

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


class McpAuthWiringTest(unittest.TestCase):
    """北向鉴权接线（2026-11-03）：HTTP 默认 fail-closed，身份只来自 transport。"""

    def setUp(self):
        self.source = SERVER.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source)
        self.funcs = {n.name: n for n in self.tree.body if isinstance(n, ast.FunctionDef)}

    def test_http_requires_token_registry(self):
        self.assertIn("IRAF_MCP_TOKEN_FILE", self.source)
        self.assertIn("BearerAuthMiddleware", self.source)
        self.assertIn("IRAF_MCP_INSECURE", self.source)
        # 无注册表且非显式 insecure 时必须 return 2（拒绝启动），不能静默放行
        main = self.funcs["main"]
        returns = [n for n in ast.walk(main) if isinstance(n, ast.Return)
                   and isinstance(n.value, ast.Constant) and n.value.value == 2]
        self.assertGreaterEqual(len(returns), 3, "main 对缺 token/非法 TLS 等必须显式失败")

    def test_tools_use_principal_and_scopes(self):
        for name, scope in (("iraf_submit_task", "task.submit"),
                            ("iraf_get_task", "task.read"),
                            ("iraf_robot_mode", "task.read")):
            func = self.funcs[name]
            calls = [n for n in ast.walk(func) if isinstance(n, ast.Call)]
            self.assertTrue(any(isinstance(c.func, ast.Name) and c.func.id == "_principal"
                                for c in calls), "%s 没有取 Principal" % name)
            self.assertTrue(any(isinstance(c.func, ast.Attribute) and c.func.attr == "require_scope"
                                and c.args and isinstance(c.args[-1], ast.Constant)
                                and c.args[-1].value == scope for c in calls),
                            "%s 没有校验 scope=%s" % (name, scope))

    def test_get_task_filters_by_owner(self):
        self.assertIn("get_for_subject", self.source,
                      "iraf_get_task 必须按调用方归属过滤，否则可读别人的执行事实")

    def test_tool_signatures_have_no_identity_params(self):
        for name in ALLOWED:
            func = self.funcs[name]
            params = {a.arg for a in func.args.args}
            self.assertFalse(params & {"subject", "caller", "role", "tenant", "token"},
                             "%s 不应接受身份参数（身份只能来自 transport）：%s"
                             % (name, sorted(params)))

    def test_mode_state_is_per_subject(self):
        self.assertIn("state_for", self.source,
                      "HTTP 多调用方共用一个进程，机器人开发模式必须按 subject 隔离")


if __name__ == "__main__":
    unittest.main()
