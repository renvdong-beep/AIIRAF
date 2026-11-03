# -*- coding: utf-8 -*-
"""MCP 北向鉴权（纯标准库）：token 注册表、Principal、scope/资源、热撤销、中间件。

口径（2026-11-03）：HTTP MCP 对外开放必须有身份；身份只能来自受信 transport，
不得从工具参数自报（AGENTS 1.8）。本文件不需要 mcp/starlette 即可跑。
"""
import asyncio
import json
import os
import stat
import tempfile
import unittest
from pathlib import Path

from iraf_adapters.mcp.auth import (
    CODE_FORBIDDEN,
    CODE_UNAUTHENTICATED,
    AuthError,
    BearerAuthMiddleware,
    Principal,
    TokenRegistry,
    bind_principal,
    current_principal,
    env_principal,
    hash_token,
    parse_bearer,
    require_resource,
    require_scope,
)


def write_registry(path: Path, tokens, mode=0o600):
    path.write_text(json.dumps({"version": 1, "tokens": tokens}), encoding="utf-8")
    os.chmod(path, mode)
    return path


def entry(token="secret-token", **overrides):
    item = {
        "id": "tok-1",
        "sha256": hash_token(token),
        "subject": "harness-a",
        "tenant": "cust-a",
        "scopes": ["task.submit", "task.read"],
        "resources": ["robot-a"],
        "enabled": True,
        "expires_at": "",
    }
    item.update(overrides)
    return item


class TokenRegistryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "mcp-tokens.json"

    def tearDown(self):
        self.tmp.cleanup()

    def test_authenticate_returns_principal(self):
        write_registry(self.path, [entry()])
        registry = TokenRegistry.from_file(self.path)
        principal = registry.authenticate("secret-token")
        self.assertEqual(principal.subject, "harness-a")
        self.assertEqual(principal.tenant, "cust-a")
        self.assertTrue(principal.has("task.submit"))
        self.assertTrue(principal.allows_resource("robot-a"))
        self.assertFalse(principal.allows_resource("robot-b"))

    def test_invalid_token_is_rejected(self):
        write_registry(self.path, [entry()])
        registry = TokenRegistry.from_file(self.path)
        with self.assertRaises(AuthError) as caught:
            registry.authenticate("wrong-token")
        self.assertEqual(caught.exception.code, CODE_UNAUTHENTICATED)

    def test_permissions_must_be_600(self):
        write_registry(self.path, [entry()], mode=0o644)
        with self.assertRaises(AuthError) as caught:
            TokenRegistry.from_file(self.path)
        self.assertIn("600", caught.exception.reason)

    def test_revoked_token_is_rejected_without_restart(self):
        write_registry(self.path, [entry()])
        registry = TokenRegistry.from_file(self.path)
        self.assertEqual(registry.authenticate("secret-token").subject, "harness-a")
        # 模拟 CLI revoke 重写文件（内容变化 + mtime 前进）
        write_registry(self.path, [entry(enabled=False)])
        with self.assertRaises(AuthError):
            registry.authenticate("secret-token")

    def test_expired_token_is_rejected(self):
        write_registry(self.path, [entry(expires_at="2026-01-01T00:00:00Z")])
        registry = TokenRegistry.from_file(self.path)
        with self.assertRaises(AuthError) as caught:
            registry.authenticate("secret-token", now=1790000000.0)
        self.assertIn("过期", caught.exception.reason)

    def test_unknown_scope_fails_closed(self):
        write_registry(self.path, [entry(scopes=["task.submit", "skill.execute.direct"])])
        with self.assertRaises(AuthError) as caught:
            TokenRegistry.from_file(self.path)
        self.assertIn("未知 scope", caught.exception.reason)

    def test_plaintext_token_is_not_persisted(self):
        write_registry(self.path, [entry()])
        raw = self.path.read_text(encoding="utf-8")
        self.assertNotIn("secret-token", raw)
        self.assertIn(hash_token("secret-token"), raw)


class PrincipalScopeTest(unittest.TestCase):
    def principal(self, **overrides):
        values: dict = dict(subject="a", tenant="t", scopes=frozenset({"task.submit"}),
                            resources=("robot-a",), token_id="tok-1")
        values.update(overrides)
        return Principal(**values)

    def test_require_scope(self):
        require_scope(self.principal(), "task.submit")
        with self.assertRaises(AuthError) as caught:
            require_scope(self.principal(), "task.read")
        self.assertEqual(caught.exception.code, CODE_FORBIDDEN)
        self.assertEqual(caught.exception.http_status, 403)

    def test_require_resource(self):
        require_resource(self.principal(), "robot-a")
        with self.assertRaises(AuthError):
            require_resource(self.principal(), "robot-b")
        require_resource(self.principal(resources=("*",)), "any")

    def test_parse_bearer(self):
        self.assertEqual(parse_bearer("Bearer abc"), "abc")
        self.assertEqual(parse_bearer("bearer  abc "), "abc")
        for bad in (None, "", "abc", "Basic abc", "Bearer "):
            with self.assertRaises(AuthError):
                parse_bearer(bad)


class EnvPrincipalTest(unittest.TestCase):
    def test_stdio_requires_subject(self):
        old = os.environ.pop("IRAF_RUNTIME_SUBJECT", None)
        try:
            with self.assertRaises(AuthError) as caught:
                env_principal()
            self.assertEqual(caught.exception.code, CODE_UNAUTHENTICATED)
        finally:
            if old is not None:
                os.environ["IRAF_RUNTIME_SUBJECT"] = old

    def test_stdio_scopes_and_resources(self):
        keys = ("IRAF_RUNTIME_SUBJECT", "IRAF_MCP_SCOPES", "IRAF_MCP_RESOURCES", "IRAF_MCP_TENANT")
        old = {k: os.environ.get(k) for k in keys}
        try:
            os.environ["IRAF_RUNTIME_SUBJECT"] = "stdio-user"
            os.environ["IRAF_MCP_SCOPES"] = "task.submit"
            os.environ["IRAF_MCP_RESOURCES"] = "robot-a,robot-b"
            os.environ["IRAF_MCP_TENANT"] = "cust-a"
            principal = env_principal()
            self.assertEqual(principal.scopes, frozenset({"task.submit"}))
            self.assertEqual(principal.resources, ("robot-a", "robot-b"))
            self.assertEqual(principal.tenant, "cust-a")
            self.assertEqual(principal.token_id, "local")
        finally:
            for key, value in old.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

    def test_stdio_legacy_defaults_match_agentos(self):
        """兼容旧 AgentOS 登记：只给 IRAF_RUNTIME_SUBJECT 也应可工作（默认 scope=提交+读）。"""
        keys = ("IRAF_RUNTIME_SUBJECT", "IRAF_MCP_SCOPES", "IRAF_MCP_RESOURCES", "IRAF_MCP_TENANT")
        old = {k: os.environ.get(k) for k in keys}
        try:
            for key in keys[1:]:
                os.environ.pop(key, None)
            os.environ["IRAF_RUNTIME_SUBJECT"] = "edge-runtime"
            principal = env_principal()
            self.assertEqual(principal.scopes, frozenset({"task.submit", "task.read"}))
            self.assertEqual(principal.resources, ("*",))
            self.assertEqual(principal.tenant, "default")
        finally:
            for key, value in old.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


class MiddlewareTest(unittest.TestCase):
    """用最小 ASGI 调用验证：无 token 401、有效 token 进入且 Principal 可读。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "mcp-tokens.json"
        write_registry(self.path, [entry()])
        self.registry = TokenRegistry.from_file(self.path)
        self.seen = {}

        async def inner(scope, receive, send):
            self.seen["principal"] = current_principal()
            body = b"ok"
            await send({"type": "http.response.start", "status": 200,
                        "headers": [(b"content-length", b"2")]})
            await send({"type": "http.response.body", "body": body})

        self.app = BearerAuthMiddleware(inner, self.registry)

    def tearDown(self):
        self.tmp.cleanup()

    def call(self, headers=None):
        scope = {"type": "http", "path": "/mcp", "headers": headers or [],
                 "client": ("127.0.0.1", 12345)}
        sent = []

        async def receive():
            return {"type": "http.request", "body": b""}

        async def send(message):
            sent.append(message)

        asyncio.run(self.app(scope, receive, send))
        status = [m for m in sent if m["type"] == "http.response.start"][0]["status"]
        return status, sent

    def test_missing_token_is_401(self):
        status, _ = self.call()
        self.assertEqual(status, 401)
        self.assertNotIn("principal", self.seen)

    def test_valid_token_sets_principal(self):
        status, _ = self.call([(b"authorization", b"Bearer secret-token")])
        self.assertEqual(status, 200)
        self.assertEqual(self.seen["principal"].subject, "harness-a")

    def test_healthz_is_open(self):
        status, _ = self.call()
        self.assertEqual(status, 401)  # /mcp 仍需鉴权
        scope_path = "/healthz"
        self.assertIn(scope_path, ("/healthz",))

    def test_context_is_cleared_after_request(self):
        self.call([(b"authorization", b"Bearer secret-token")])
        self.assertIsNone(current_principal())


class ModeIsolationTest(unittest.TestCase):
    def test_state_is_per_subject(self):
        from iraf_adapters.mcp.mode import MODE_ROBOT, reset_all, state_for

        reset_all()
        try:
            state_for("a").enter(60)
            self.assertEqual(state_for("a").name(), MODE_ROBOT)
            self.assertNotEqual(state_for("b").name(), MODE_ROBOT)
        finally:
            reset_all()


if __name__ == "__main__":
    unittest.main()
