# -*- coding: utf-8 -*-
"""IRAF × AgentOS 交互式 CLI 的纯逻辑回归测试（2026-10-12）。

锁住四件事（不必起 Runtime、不调模型即可单测）：
1. `build_request` 三个 id 必齐、deadline 在未来、空白文本被拒；
2. 身份解析优先级（显式 > 环境 > 显式失败）与 scope 不含 `skill.execute.direct`；
3. `classify_result` **不把失败当成功**（含 intent 解析失败 / 策略拒绝的分流）；
4. `missing_env` 的空值也算缺失（避免"设了空串即通过"）。
"""
import time
import unittest

from iraf_adapters.agentos.cli import (
    DEFAULT_SCOPES,
    EXIT_EXECUTION_FAILED,
    EXIT_INTENT_PARSE_FAILED,
    EXIT_OK,
    RUNTIME_ENV,
    build_context,
    build_request,
    classify_result,
    missing_env,
    render_result,
    resolve_subject,
)


class BuildRequestTests(unittest.TestCase):
    def test_three_ids_present_and_deadline_future(self):
        request = build_request("让机器狗站起来", deadline_seconds=30)
        for key in ("request_id", "correlation_id", "idempotency_key"):
            self.assertTrue(request[key], key)
        self.assertGreater(request["deadline_unix_ms"], int(time.time() * 1000))
        self.assertEqual(request["text"], "让机器狗站起来")

    def test_blank_text_rejected(self):
        for bad in ("", "   ", None, 123):
            with self.assertRaises((ValueError, TypeError)):
                build_request(bad)

    def test_explicit_request_id_is_reused_for_idempotency(self):
        request = build_request("站立", request_id="fixed-1")
        self.assertEqual(request["request_id"], "fixed-1")
        self.assertEqual(request["idempotency_key"], "fixed-1")


class ContextTests(unittest.TestCase):
    def test_default_scopes_exclude_direct_skill(self):
        context = build_context("agentos-cli")
        self.assertEqual(context.transport, "local")
        self.assertIn("task.submit", context.roles)
        self.assertNotIn("skill.execute.direct", context.roles)
        self.assertEqual(set(DEFAULT_SCOPES), set(context.roles))

    def test_missing_subject_rejected(self):
        with self.assertRaises(ValueError):
            build_context("")

    def test_resolve_subject_priority(self):
        import os
        from unittest.mock import patch
        with patch.dict(os.environ, {"IRAF_RUNTIME_SUBJECT": "env-subject"}, clear=True):
            self.assertEqual(resolve_subject(None), "env-subject")
            self.assertEqual(resolve_subject("explicit"), "explicit")
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(RuntimeError):
                resolve_subject(None)


class ClassifyResultTests(unittest.TestCase):
    def test_success(self):
        code, kind = classify_result({"status": "SUCCEEDED"})
        self.assertEqual((code, kind), (EXIT_OK, "succeeded"))

    def test_intent_parse_failure(self):
        code, kind = classify_result({"status": "FAILED", "error_code": "IRAF-INTENT-PARSE-FAILED"})
        self.assertEqual((code, kind), (EXIT_INTENT_PARSE_FAILED, "intent_parse_failed"))

    def test_denied_is_not_success(self):
        code, _ = classify_result({"status": "FAILED", "error_code": "IRAF-POLICY-DENIED"})
        self.assertEqual(code, EXIT_EXECUTION_FAILED)

    def test_error_code_with_succeeded_status_is_not_success(self):
        """状态写着 SUCCEEDED 但带 error_code ⇒ **不得**判成功（防伪造）。"""
        code, _ = classify_result({"status": "SUCCEEDED", "error_code": "IRAF-EXECUTION-FAILED"})
        self.assertEqual(code, EXIT_EXECUTION_FAILED)

    def test_non_dict_is_failure(self):
        self.assertEqual(classify_result(None)[0], EXIT_EXECUTION_FAILED)


class RenderTests(unittest.TestCase):
    def test_render_shows_code_and_reason(self):
        text = render_result({"status": "FAILED", "error_code": "IRAF-POLICY-DENIED",
                              "reason": "调用方无任务提交权限"})
        self.assertIn("IRAF-POLICY-DENIED", text)
        self.assertIn("调用方无任务提交权限", text)

    def test_render_json_is_verbatim(self):
        import json
        payload = {"status": "SUCCEEDED", "execution_id": "e1"}
        self.assertEqual(json.loads(render_result(payload, as_json=True)), payload)


class MissingEnvTests(unittest.TestCase):
    def test_empty_string_counts_as_missing(self):
        missing = missing_env(("A", "B"), {"A": "", "B": "set"})
        self.assertEqual(missing, ["A"])

    def test_runtime_env_list_is_stable(self):
        self.assertEqual(set(missing_env(RUNTIME_ENV, {})), set(RUNTIME_ENV))


if __name__ == "__main__":
    unittest.main()
