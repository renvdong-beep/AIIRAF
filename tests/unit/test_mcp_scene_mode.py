# -*- coding: utf-8 -*-
"""A3 静态约束：MCP 工具接 scene_id + 三档注册表模式（不需要 mcp 依赖）。

为什么用静态检查：server.py 顶层 `import mcp`，CI/开发机上不一定装 FastMCP；
但"工具签名 + 三档开关 + 错误码"属于契约面，必须可断言、防回潮。
运行时行为（shadow 审计、on 一致性校验）由阶段 A6 在板卡上验证。
"""

from __future__ import annotations

import ast
import re
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SERVER = REPO / "src/iraf_adapters/mcp/server.py"
SRC = SERVER.read_text(encoding="utf-8")
REGISTRY = REPO / "src/iraf_core/scene_registry.py"
SRC_REGISTRY = REGISTRY.read_text(encoding="utf-8")


def _func_args(name: str) -> dict:
    tree = ast.parse(SRC)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            args = {}
            defaults = node.args.defaults
            all_args = node.args.args
            for i, a in enumerate(all_args):
                default = None
                idx = i - (len(all_args) - len(defaults))
                if idx >= 0:
                    default = ast.literal_eval(defaults[idx])
                args[a.arg] = default
            return args
    return {}


class SceneIdParamTest(unittest.TestCase):
    """四个入口都要能被显式指定场景（默认空串 = 用会话默认/env 默认）。"""

    def test_robot_mode_has_scene_id(self):
        args = _func_args("iraf_robot_mode")
        self.assertIn("scene_id", args)
        self.assertEqual(args["scene_id"], "")

    def test_enter_robot_mode_forwards_scene_id(self):
        args = _func_args("iraf_enter_robot_mode")
        self.assertIn("scene_id", args)
        self.assertIn('iraf_robot_mode(action="enter", scene_id=scene_id)', SRC)

    def test_submit_task_has_scene_and_resource(self):
        args = _func_args("iraf_submit_task")
        self.assertIn("scene_id", args)
        self.assertIn("resource_id", args)
        self.assertEqual(args["scene_id"], "")

    def test_run_scenario_has_scene_id(self):
        args = _func_args("iraf_run_scenario")
        self.assertIn("scene_id", args)


class RegistryModeTest(unittest.TestCase):
    def test_mode_env_and_default_off(self):
        self.assertIn("IRAF_SCENE_REGISTRY_MODE", SRC)
        # 默认必须是 off：任何"未配置就生效"都是行为回归
        self.assertRegex(SRC, r'IRAF_SCENE_REGISTRY_MODE"\)\s*or\s*"off"')

    def test_three_modes_explicit(self):
        for mode in ('"shadow"', '"on"', '"off"'):
            self.assertIn(mode, SRC, "缺少模式分支：%s" % mode)
        # off 必须在解析前直接返回，不允许"先解析后忽略"
        self.assertRegex(SRC, r'if mode == "off":\s*\n\s*return None, None')

    def test_registry_paths_env_overridable(self):
        self.assertIn("IRAF_SCENE_ROOT", SRC)
        self.assertIn("IRAF_PROFILES_ROOT", SRC)

    def test_shadow_audits_resolved_scene(self):
        self.assertIn('auth.audit("resolved_scene"', SRC)
        for field in ("scene=", "resource=", "host=", "endpoint=", "mode="):
            self.assertIn(field, SRC, "审计缺字段：%s" % field)

    def test_on_mode_requires_env_registry_consistency(self):
        self.assertIn("IRAF-REGISTRY-MISMATCH", SRC)

    def test_session_default_scene_memory(self):
        self.assertIn("_SESSION_SCENE", SRC)
        # 进入模式且给了 scene_id 时必须记住（否则后续 submit 无法沿用）
        self.assertRegex(SRC, r'_SESSION_SCENE\[principal\.subject\]\s*=\s*_resolved\.scene\.scene_id')

    def test_run_scenario_resolves_after_principal(self):
        """回归：A3 首次打补丁时把 _resolve_scene 插到了 principal 赋值之前（NameError）。"""
        body = SRC.split("def iraf_run_scenario")[1]
        i_p = body.find("principal = _principal()")
        i_r = body.find("_resolve_scene(principal, scene_id)")
        self.assertGreaterEqual(i_p, 0, "缺少 principal 赋值")
        self.assertGreater(i_r, i_p, "_resolve_scene 必须在 principal 之后")

    def test_submit_resolves_before_dispatch(self):
        # 解析失败必须早于 bridge/任务构造返回，避免"带病执行"
        idx_resolve = SRC.find("_resolved, _scene_err = _resolve_scene(principal, scene_id, resource_id or None)")
        idx_build = SRC.find("build_request(text, deadline_seconds=int(deadline_seconds)")
        self.assertNotEqual(idx_resolve, -1, "submit 未接入 _resolve_scene")
        self.assertLess(idx_resolve, idx_build, "解析必须在 build_request 之前")


class OffModeSafetyTest(unittest.TestCase):
    """回归：off 模式下 _resolve_scene 返回 None，不能直接解引用（曾导致 robot_mode 报错）。"""

    def test_robot_mode_guards_none(self):
        body = SRC.split("def iraf_robot_mode(")[1].split("@app.tool()")[0]
        self.assertIn("if _resolved is not None", body, "off 模式必须先判空")
        i_guard = body.find("if _resolved is not None")
        i_use = body.find("_resolved.scene.scene_id")
        self.assertGreater(i_use, i_guard, "解引用必须在判空之后")


class RegistryRoutingTest(unittest.TestCase):
    """B1：on 模式下注册表真正决定路由（A 阶段只做一致性校验）。"""

    def test_bridge_for_resolved_exists_and_caches_by_endpoint(self):
        body = SRC.split("def _bridge_for_resolved")[1].split("\ndef ")[0]
        self.assertIn('key = "bridge:" + resolved.host.endpoint', body)
        self.assertIn("_STATE[key]", body)

    def test_build_injects_and_restores_env(self):
        body = SRC.split("def _bridge_for_resolved")[1].split("\ndef ")[0]
        self.assertIn('os.environ["IRAF_PROFILE"] = resolved.scene.profile', body)
        self.assertIn('base["endpoint"] = resolved.host.endpoint', body)
        self.assertIn('json.loads(saved.get("IRAF_BACKEND_CONFIG") or "{}")', body)
        self.assertIn("finally:", body, "构建后必须恢复 env")

    def test_submit_resolves_before_building_bridge(self):
        body = SRC.split("def iraf_submit_task")[1].split("@app.tool()")[0]
        i_resolve = body.find("_resolve_scene(principal, scene_id, resource_id or None)")
        i_bridge = body.find("_bridge_for_resolved(_resolved)")
        self.assertNotEqual(i_resolve, -1)
        self.assertNotEqual(i_bridge, -1)
        self.assertLess(i_resolve, i_bridge, "必须先解析再选 bridge")

    def test_ensure_scene_uses_resolved_endpoint(self):
        self.assertIn("def _ensure_scene(scene: str, endpoint: str = \"\", host=None, provider=None)", SRC)
        self.assertIn("_ensure_scene(_resolved.scene.switch_name or _resolved.scene.scene_id,", SRC)
        self.assertIn("_resolved.host.endpoint, _resolved.host, _resolved.provider", SRC)
        self.assertIn("IRAF-PROVIDER-UNAVAILABLE", SRC, "启停失败必须显式失败")

    def test_robot_facts_prefers_resolved(self):
        body = SRC.split("def _robot_facts(")[1].split("\ndef ")[0]
        self.assertIn("resolved.scene.profile", body)
        self.assertIn("resolved.host.endpoint", body)

    def test_run_scenario_routes_by_resolved_host(self):
        self.assertIn("_resolved.host.host if _resolved is not None", SRC)


class DeadlineDefaultTest(unittest.TestCase):
    """回归：ur5 冷启动 + 高负载时 120s deadline 不够（实测 146.6s 超时）→ 默认 180s。"""

    def test_submit_default_deadline(self):
        args = _func_args("iraf_submit_task")
        self.assertEqual(args["deadline_seconds"], 180)


class AsyncScenarioTest(unittest.TestCase):
    """E1：async_mode 立即返回 run_id；iraf_get_run 只读查询。"""

    def test_run_scenario_has_async_mode(self):
        args = _func_args("iraf_run_scenario")
        self.assertIn("async_mode", args)
        self.assertEqual(args["async_mode"], False)

    def test_async_returns_queued_with_run_id(self):
        body = SRC.split("def iraf_run_scenario(")[1].split("\ndef ")[0]
        self.assertIn('"status": "QUEUED"', body)
        self.assertIn('"run_id": run_id', body)
        self.assertIn("_run_store().create(", body)
        self.assertIn("threading.Thread(", body)

    def test_background_worker_marks_runs(self):
        body = SRC.split("def _run_scenario_bg(")[1].split("\ndef ")[0]
        self.assertIn("store.start(run_id)", body)
        self.assertIn('status="succeeded" if ok else "failed"', body)
        self.assertIn("LeaseConflict", body)

    def test_get_run_tool(self):
        args = _func_args("iraf_get_run")
        self.assertIn("run_id", args)
        body = SRC.split("def iraf_get_run(")[1].split("@app.tool()")[0]
        self.assertIn("_run_store().get(", body)
        self.assertIn("task.read", body)


class HostLoadProbeTest(unittest.TestCase):
    """D4：hosts[].load —— probe 时才联网，默认离线。"""

    def test_scene_status_has_probe_param(self):
        args = _func_args("iraf_scene_status")
        self.assertIn("probe", args)
        self.assertEqual(args["probe"], False)

    def test_probe_helper_exists(self):
        body = SRC.split("def _probe_host(")[1].split("\ndef ")[0]
        self.assertIn('base + "/status"', body)
        self.assertIn("timeout=timeout", body)
        self.assertIn('return {"error"', body)

    def test_probe_merges_runtime(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn("if probe:", body)
        self.assertIn('hst["runtime"] = _probe_host(_h)', body)

    def test_default_offline(self):
        """probe 默认 False；且 off 模式在读取注册表/探测之前就返回。"""
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        i_off = body.find('if mode == "off":')
        i_reg = body.find("reg = _registry()")
        i_probe = body.find("if probe:")
        self.assertGreaterEqual(i_off, 0)
        self.assertGreater(i_reg, i_off, "off 模式必须先返回，不读注册表")
        self.assertGreater(i_probe, i_reg, "probe 只在注册表分支内")


class InflightAndRoutingTest(unittest.TestCase):
    """D4：scene_status 的 inflight/world_key；D2：failover=next 时的改路由。"""

    def test_scene_status_has_inflight(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('"inflight"', body)
        self.assertIn("l.execution_id", body)

    def test_resolved_world_key(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('"world_key"', body)
        self.assertIn('"host:%s/__world__" % resolved.host.id', body)

    def test_submit_reroutes_on_conflict_when_failover(self):
        body = SRC.split("def iraf_submit_task")[1].split("def _ensure_scene")[0]
        self.assertIn('failover == "next" and free', body)
        self.assertIn("_resolved.host = _next", body)
        self.assertIn("rerouted_to", body)

    def test_reroute_only_when_declared(self):
        body = SRC.split("def iraf_submit_task")[1].split("def _ensure_scene")[0]
        i_fail = body.find('failover == "next" and free')
        i_assign = body.find("_resolved.host = _next")
        self.assertGreater(i_assign, i_fail, "先判定声明再改路由")


class LeaseSurfaceTest(unittest.TestCase):
    """D3/D4：iraf_lease 工具 + scene_status 暴露租约 + 容量判定。"""

    def test_lease_tool_signature(self):
        args = _func_args("iraf_lease")
        for k in ("action", "scene_id", "resource_id", "lease_id", "ttl_s"):
            self.assertIn(k, args)
        self.assertEqual(args["action"], "list")
        self.assertEqual(args["resource_id"], "__world__")

    def test_lease_actions(self):
        body = SRC.split("def iraf_lease(")[1].split("@app.tool()")[0]
        for act in ("acquire", "renew", "release", "list"):
            self.assertIn('"%s"' % act, body)
        self.assertIn("LeaseConflict", body)

    def test_scene_status_exposes_leases(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('"leases"', body)
        self.assertIn('"lease_enforce"', body)

    def test_capacity_helper(self):
        # 规则下沉到 iraf_core.leases.capacity_state，server 只做委托
        body = SRC.split("def _capacity_state(")[1].split("\ndef ")[0]
        self.assertIn("capacity_state(provider.hosts, _lease_store())", body)
        self.assertIn("IRAF-CAPACITY-EXHAUSTED", SRC)


class MultiHostTest(unittest.TestCase):
    """C4：host 级 control_url + 按 priority 的 failover。"""

    def test_control_url_preferred_over_env(self):
        # 看实现行（docstring 里也会出现同名 env，不能按首次出现位置判）
        self.assertIn('return (host_control_url or os.environ.get("IRAF_SUPERVISOR_URL") or "")', SRC)

    def test_ensure_passes_host_and_provider(self):
        self.assertIn("_resolved.host.endpoint, _resolved.host, _resolved.provider", SRC)

    def test_failover_only_when_declared(self):
        body = SRC.split("def _ensure_scene(")[1].split("\ndef ")[0]
        self.assertIn('failover == "next"', body)
        self.assertIn("sorted(provider.hosts, key=lambda x: x.priority)", body)

    def test_failover_reports_host(self):
        self.assertIn('"switched(supervisor:%s)" % h.id', SRC)

    def test_control_url_validation(self):
        self.assertIn("control_url 必须是 http(s)://", SRC_REGISTRY)


class PhaseBContractTest(unittest.TestCase):
    """B3：switch_scene 降级；submit 回传 scene/resolved。"""

    def test_switch_scene_marked_deprecated(self):
        body = SRC.split("def iraf_switch_scene(")[1].split("@app.tool()")[0]
        self.assertIn("deprecated", body)
        self.assertIn("iraf_submit_task", body)

    def test_submit_echoes_resolved_scene(self):
        body = SRC.split("def iraf_submit_task")[1].split("def _ensure_scene")[0]
        self.assertIn('result.setdefault("scene_id", _resolved.scene.scene_id)', body)
        self.assertIn('result.setdefault("resolved", _resolved.as_dict())', body)


class AllowedHostsTest(unittest.TestCase):
    """A4：白名单单一来源 = env 显式值 > 注册表汇总 > 默认 .121。"""

    def test_helper_exists(self):
        self.assertIn("def _allowed_hosts()", SRC)

    def test_env_has_priority(self):
        body = SRC.split("def _allowed_hosts()")[1].split("def ")[0]
        i_env = body.find("IRAF_PROVIDER_ALLOWED_HOSTS")
        i_reg = body.find("_registry_mode()")
        self.assertGreaterEqual(i_env, 0, "必须先读 env")
        self.assertGreater(i_reg, i_env, "env 显式值优先于注册表")

    def test_default_is_local_host(self):
        body = SRC.split("def _allowed_hosts()")[1].split("def ")[0]
        self.assertIn('return ["10.203.247.121"]', body)

    def test_reject_uses_helper_not_hardcoded(self):
        body = SRC.split("def _reject_foreign_host()")[1].split("def ")[0]
        self.assertIn("allowed = _allowed_hosts()", body)
        self.assertNotIn('or "10.203.247.121"', body, "不允许在拒绝逻辑里再硬编码一份")


class SceneStatusToolTest(unittest.TestCase):
    """A5：iraf_scene_status 只读、给定 scene_id 时给解析结果。"""

    def test_tool_defined_with_optional_scene(self):
        args = _func_args("iraf_scene_status")
        self.assertIn("scene_id", args)
        self.assertEqual(args["scene_id"], "")

    def test_requires_read_scope(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('auth.require_scope(principal, "task.read")', body)

    def test_reports_mode_and_allowed_hosts(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('"mode": mode', body)
        self.assertIn("_allowed_hosts()", body)

    def test_resolves_scene_when_given(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn("reg.resolve(scene_id)", body)
        self.assertIn("as_dict()", body)

    def test_off_mode_reports_env_facts_only(self):
        body = SRC.split("def iraf_scene_status(")[1].split("@app.tool()")[0]
        self.assertIn('if mode == "off":', body)
        self.assertIn("_endpoint_host()", body)


if __name__ == "__main__":
    unittest.main()
