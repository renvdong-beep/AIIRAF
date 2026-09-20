"""IRAF SDK 错误诊断与客户端契约测试（步骤 05）。

被测契约：
  - `src/iraf_sdk/errors.py`：错误码 → 中文诊断映射（来源 = IDL §5 错误码表 + 实现抛出点）；
  - `src/iraf_sdk/client.py`：HTTP / gRPC 公共契约薄封装（无旁路、无隐式默认、无自动重试）；
  - `pyproject.toml`：`iraf_sdk*` 必须被打进 SDK wheel，版本号必须与 `iraf_sdk.__version__` 一致。

设计要点（fail-closed，对应 AGENTS.md 铁律 1.5 / 2.3 / 2.4）：
  1. 未知错误码必须显式标注「未知错误码」，不得静默返回原文、不得按已知码猜测、不得自动重试；
  2. 失败终态缺 `error_code` 字段即契约违反（禁止默认成功）；空字符串表示显式无错误；
  3. 每个已知错误码都必须有中文诊断 + 下一步建议（逐码用例）；
  4. 每个码的 `emitter`（路径:行号）必须真的存在且真的出现该码——防止映射表“凭印象”声明；
  5. 每条负向门禁都配一条正向对照，避免"恒失败"的假门禁。

HTTP 协议用例使用**本机 loopback stub 服务器**：它只证明 SDK 侧的协议与解析逻辑，
**不是**运行时健康检查，也不能作为目标端/真机证据（x86-first 本轮范围）。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_sdk_errors -v`
"""

import http.server
import importlib
import json
import os
import re
import socket
import subprocess
import sys
import threading
import unittest
from pathlib import Path

from iraf_sdk import (
    ERROR_SPECS,
    KNOWN_ERROR_CODES,
    TRANSPORT_FAILED_CODE,
    UNKNOWN_CODE_LABEL,
    Diagnosis,
    GrpcSkillClient,
    HttpSkillClient,
    IrafError,
    SdkConfigError,
    SdkContractError,
    SdkTransportError,
    SdkUnsupportedError,
    __version__,
    canonical_execute_body,
    deadline_after,
    describe,
    identity_of,
    is_retryable,
    raise_for_status,
    spec_for,
)
from iraf_sdk.client import (
    event_to_dict,
    execute_request_from_body,
    result_from_feedback,
    result_from_snapshot,
    status_from_state,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "src"
GENERATED_DIR = REPO_ROOT / "build" / "generated" / "python"
IDL_DOC = REPO_ROOT / "docs" / "iraf-idl-runtime-detailed-design.md"
CATALOG_DOC = REPO_ROOT / "docs" / "iraf-api-contract-catalog.md"
PYPROJECT = REPO_ROOT / "pyproject.toml"

ERROR_CODE_PATTERN = re.compile(r"IRAF-[A-Z][A-Z-]*")
IDL_TABLE_ROW = re.compile(r"^\|\s*`(IRAF-[A-Z-]+)`\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*$")
CHINESE_PATTERN = re.compile(r"[\u4e00-\u9fff]")

PROFILE = {"name": "piper", "version": "1.0.0", "digest": "a" * 64}
SAFETY = {"name": "lab", "version": "1.0.0", "digest": "b" * 64}


def parse_idl_error_table():
    """解析 `docs/iraf-idl-runtime-detailed-design.md` §5 的错误码表（doc 是契约来源）。"""
    rows = {}
    for line in IDL_DOC.read_text(encoding="utf-8").splitlines():
        match = IDL_TABLE_ROW.match(line)
        if match:
            rows[match.group(1)] = {"meaning_zh": match.group(2), "retry_zh": match.group(3)}
    return rows


class IdlErrorTableContractTests(unittest.TestCase):
    """错误码映射必须与 IDL §5 错误码表一致（正反两向都钉住）。"""

    @classmethod
    def setUpClass(cls):
        cls.table = parse_idl_error_table()

    def test_idl_error_table_is_parsed_completely(self):
        """正向对照：解析器本身有效（否则后面的门禁可能是恒真的空循环）。"""
        self.assertGreaterEqual(len(self.table), 9, f"IDL §5 表只解析出 {len(self.table)} 行：{sorted(self.table)}")
        self.assertIn("IRAF-INPUT-INVALID", self.table)
        self.assertIn("IRAF-POLICY-DENIED", self.table)

    def test_every_idl_code_has_a_known_chinese_diagnosis(self):
        for code in sorted(self.table):
            with self.subTest(code=code):
                spec = spec_for(code)
                self.assertIsNotNone(spec, f"IDL 声明的错误码 {code} 在 SDK 映射中缺失")
                diagnosis = describe(code, "原始原因")
                self.assertTrue(diagnosis.known, f"{code} 被当成未知错误码")
                self.assertTrue(CHINESE_PATTERN.search(diagnosis.summary_zh), f"{code} 缺少中文诊断")
                self.assertTrue(CHINESE_PATTERN.search(diagnosis.suggestion_zh), f"{code} 缺少中文下一步建议")
                self.assertNotIn(UNKNOWN_CODE_LABEL, diagnosis.format_zh())

    def test_idl_retry_column_matches_declared_retry_policy(self):
        """doc 的「是否重试」列是事实来源：映射不得自说自话。"""
        for code, row in sorted(self.table.items()):
            with self.subTest(code=code):
                spec = spec_for(code)
                self.assertIsNotNone(spec)
                self.assertTrue(
                    spec.retry_policy_zh.startswith(row["retry_zh"]),
                    f"{code} 的重试说明 {spec.retry_policy_zh!r} 未对齐 IDL 表 {row['retry_zh']!r}",
                )
                self.assertEqual(
                    spec.retryable,
                    row["retry_zh"].startswith("是"),
                    f"{code} 的 retryable={spec.retryable} 与 IDL 表 {row['retry_zh']!r} 不一致",
                )

    def test_api_catalog_code_is_registered(self):
        """`IRAF-STREAM-LAGGED` 只在 API 契约目录声明，也必须能被诊断。"""
        raw = CATALOG_DOC.read_text(encoding="utf-8")
        self.assertIn("IRAF-STREAM-LAGGED", raw)
        spec = spec_for("IRAF-STREAM-LAGGED")
        self.assertIsNotNone(spec)
        self.assertEqual(spec.provenance, "api-catalog-only")

    def test_idl_only_codes_are_marked_as_not_yet_implemented(self):
        """契约已声明但实现未落地：必须显式标注，避免被当成"现在可触发"的能力。"""
        for code in ("IRAF-RESOURCE-BUSY", "IRAF-SAFETY-STOP"):
            with self.subTest(code=code):
                spec = spec_for(code)
                self.assertIsNotNone(spec)
                self.assertEqual(spec.provenance, "idl-only")
                self.assertEqual(spec.emitter, (), f"{code} 标注为 idl-only，却登记了抛出点")


class SourceEmissionContractTests(unittest.TestCase):
    """映射里的每个码/抛出点都要能在仓库里核实（防止凭印象声明契约）。"""

    def test_every_code_emitted_in_source_is_registered(self):
        emitted = set()
        for path in sorted(SRC_DIR.rglob("*")):
            if path.suffix not in {".py", ".proto"} or "__pycache__" in path.parts:
                continue
            emitted |= set(ERROR_CODE_PATTERN.findall(path.read_text(encoding="utf-8", errors="replace")))
        self.assertGreaterEqual(len(emitted), 15, f"源码里只扫到 {len(emitted)} 个错误码，扫描逻辑可能失效")
        missing = sorted(emitted - set(KNOWN_ERROR_CODES))
        self.assertEqual([], missing, f"源码抛出但 SDK 未登记的错误码：{missing}")

    def test_declared_emitters_exist_and_really_emit_the_code(self):
        checked = 0
        for spec in ERROR_SPECS:
            for reference in spec.emitter:
                path_text, _, line_text = reference.partition(":")
                path = REPO_ROOT / path_text
                with self.subTest(code=spec.code, emitter=reference):
                    self.assertTrue(path.is_file(), f"{spec.code} 的抛出点不存在：{path_text}")
                    content = path.read_text(encoding="utf-8", errors="replace")
                    self.assertIn(spec.code, content, f"{spec.code} 未出现在 {path_text} 中")
                    line = int(line_text)
                    self.assertLessEqual(line, len(content.splitlines()), f"{reference} 行号越界")
                checked += 1
        self.assertGreaterEqual(checked, 15, f"只核对了 {checked} 条抛出点，映射表可能被削弱")

    def test_every_known_code_has_unique_and_prefixed_name(self):
        self.assertEqual(len(KNOWN_ERROR_CODES), len(set(KNOWN_ERROR_CODES)), "错误码重复")
        for code in KNOWN_ERROR_CODES:
            with self.subTest(code=code):
                self.assertTrue(code.startswith("IRAF-"), code)
                self.assertEqual(code, code.upper(), code)


class DiagnosisTests(unittest.TestCase):
    """中文诊断的正向与负向用例。"""

    def test_every_known_code_describes_without_unknown_label(self):
        for code in KNOWN_ERROR_CODES:
            with self.subTest(code=code):
                diagnosis = describe(code, "服务端原文")
                self.assertTrue(diagnosis.known)
                self.assertNotIn(UNKNOWN_CODE_LABEL, diagnosis.format_zh())
                self.assertIn("下一步：", diagnosis.format_zh())
                self.assertIn("服务端原文：服务端原文", diagnosis.format_zh())

    def test_unknown_code_is_labeled_and_never_silently_passed_through(self):
        diagnosis = describe("IRAF-NOT-A-REAL-CODE", "server said boom")
        self.assertIsInstance(diagnosis, Diagnosis)
        self.assertFalse(diagnosis.known)
        self.assertEqual(UNKNOWN_CODE_LABEL, diagnosis.summary_zh)
        self.assertIn(UNKNOWN_CODE_LABEL, diagnosis.format_zh())
        # 服务端原文必须保留，但不能冒充诊断结论
        self.assertEqual("server said boom", diagnosis.server_reason)
        self.assertIn("server said boom", diagnosis.format_zh())
        self.assertNotIn("server said boom", diagnosis.summary_zh)
        self.assertFalse(diagnosis.retryable, "未知错误码不得假设可重试")
        self.assertEqual("unregistered", diagnosis.provenance)
        self.assertIn("未登记", diagnosis.format_zh())

    def test_unknown_code_does_not_borrow_a_known_diagnosis(self):
        known_summaries = {spec.summary_zh for spec in ERROR_SPECS}
        diagnosis = describe("IRAF-WHATEVER")
        self.assertNotIn(diagnosis.summary_zh, known_summaries, "未知码不得借用已知码的诊断文本")
        self.assertFalse(diagnosis.known)
        self.assertEqual("unregistered", diagnosis.provenance)
        self.assertIsNone(diagnosis.contract_retryable)

    def test_blank_or_missing_code_is_a_contract_violation(self):
        for value in ("", "   ", None, 17):
            with self.subTest(value=repr(value)):
                with self.assertRaises(SdkContractError):
                    describe(value)

    def test_retryable_set_is_exactly_the_documented_ones(self):
        retryable = {code for code in KNOWN_ERROR_CODES if is_retryable(code)}
        self.assertEqual(
            {"IRAF-SKILL-PROVIDER-UNAVAILABLE", "IRAF-RESOURCE-BUSY", TRANSPORT_FAILED_CODE},
            retryable,
        )

    def test_safety_and_terminal_codes_are_never_retryable(self):
        for code in (
            "IRAF-SAFETY-STOP",
            "IRAF-SAFETY-QUARANTINED",
            "IRAF-CANCEL-STOP-FAILED",
            "IRAF-CANCELLED",
            "IRAF-POLICY-DENIED",
            "IRAF-EXECUTION-FAILED",
            "IRAF-IDEMPOTENCY-CONFLICT",
            "IRAF-UNAUTHENTICATED",
        ):
            with self.subTest(code=code):
                self.assertFalse(is_retryable(code), f"{code} 不得被判为可重试")

    def test_unknown_code_is_not_retryable(self):
        self.assertFalse(is_retryable("IRAF-NOPE"))

    def test_server_retryable_overrides_contract_default_and_is_reported(self):
        diagnosis = describe("IRAF-SKILL-PROVIDER-UNAVAILABLE", "no healthy provider", False)
        self.assertFalse(diagnosis.retryable)
        self.assertIn("契约漂移提示", diagnosis.format_zh())
        self.assertTrue(diagnosis.to_dict()["known"])

    def test_diagnosis_format_contains_code_diagnosis_next_step_reason_and_source(self):
        text = describe("IRAF-POLICY-DENIED", "调用方无任务提交权限").format_zh()
        self.assertIn("[IRAF-POLICY-DENIED]", text)
        self.assertIn("下一步：", text)
        self.assertIn("重试：不可重试", text)
        self.assertIn("服务端原文：调用方无任务提交权限", text)
        self.assertIn("来源：idl+code", text)

    def test_spec_for_unknown_returns_none_instead_of_a_guess(self):
        self.assertIsNone(spec_for("IRAF-NOPE"))
        self.assertIsNone(spec_for(None))


class RaiseForStatusTests(unittest.TestCase):
    """"失败即抛错、成功不误伤" 的判读规则（禁止默认成功）。"""

    def test_success_payload_passes_through_unchanged(self):
        payload = {"status": "SUCCEEDED", "error_code": "", "execution_id": "execution-1", "reason": ""}
        self.assertIs(payload, raise_for_status(payload))

    def test_non_terminal_payload_passes_through(self):
        payload = {"status": "RUNNING", "error_code": "", "execution_id": "execution-1", "reason": ""}
        self.assertIs(payload, raise_for_status(payload))

    def test_empty_error_code_is_explicit_no_error(self):
        """取消成功的真实形状：status=CANCELLED 但 error_code 为空字符串。"""
        payload = {"accepted": True, "status": "CANCELLED", "error_code": "", "reason": "caller cancelled"}
        self.assertIs(payload, raise_for_status(payload))

    def test_failure_payload_raises_iraf_error_with_chinese_diagnosis(self):
        payload = {
            "status": "FAILED",
            "error_code": "IRAF-POLICY-DENIED",
            "reason": "调用方无任务提交权限",
            "execution_id": "execution-1",
        }
        with self.assertRaises(IrafError) as raised:
            raise_for_status(payload)
        error = raised.exception
        self.assertEqual("IRAF-POLICY-DENIED", error.code)
        self.assertFalse(error.retryable)
        self.assertEqual(payload, error.payload)
        text = str(error)
        self.assertIn("IRAF-POLICY-DENIED", text)
        self.assertIn("调用方无任务提交权限", text)
        self.assertIn("下一步：", text)

    def test_unknown_code_failure_is_raised_with_explicit_label(self):
        payload = {"status": "FAILED", "error_code": "IRAF-FROM-THE-FUTURE", "reason": "future"}
        with self.assertRaises(IrafError) as raised:
            raise_for_status(payload)
        self.assertFalse(raised.exception.known)
        self.assertIn(UNKNOWN_CODE_LABEL, str(raised.exception))
        self.assertFalse(raised.exception.retryable)

    def test_missing_error_code_key_is_a_contract_violation_not_a_success(self):
        with self.assertRaises(SdkContractError):
            raise_for_status({"status": "FAILED", "reason": "boom"})

    def test_none_error_code_is_a_contract_violation(self):
        with self.assertRaises(SdkContractError):
            raise_for_status({"status": "FAILED", "error_code": None, "reason": "boom"})

    def test_missing_or_unknown_status_is_a_contract_violation(self):
        with self.assertRaises(SdkContractError):
            raise_for_status({"error_code": ""})
        with self.assertRaises(SdkContractError):
            raise_for_status({"status": "MELTED", "error_code": ""})

    def test_non_mapping_payload_is_a_contract_violation(self):
        for value in (None, [], "ok", 3):
            with self.subTest(value=repr(value)):
                with self.assertRaises(SdkContractError):
                    raise_for_status(value)

    def test_from_payload_rejects_payload_without_error_code(self):
        with self.assertRaises(SdkContractError):
            IrafError.from_payload({"status": "SUCCEEDED", "reason": ""})
        with self.assertRaises(SdkContractError):
            IrafError.from_payload({"status": "FAILED", "error_code": "  "})


class CanonicalRequestBodyTests(unittest.TestCase):
    """请求体必须逐字段对齐 IDL 的 JSON 映射（服务端 ignore_unknown_fields=False）。"""

    def body(self, **overrides):
        kwargs = dict(
            skill="move_joint",
            profile=PROFILE,
            safety_policy=SAFETY,
            parameters={"positions": {"j1": 0.1}, "duration_ms": 500},
            request_id="request-1",
            idempotency_key="idem-1",
            correlation_id="corr-1",
            deadline_s=30,
        )
        kwargs.update(overrides)
        return canonical_execute_body(**kwargs)

    def test_body_field_names_match_proto_json_mapping(self):
        body = self.body()
        self.assertEqual({"requestId", "idempotencyKey", "goal"}, set(body))
        self.assertEqual(
            {
                "correlationId",
                "skillName",
                "skillVersionConstraint",
                "inputs",
                "deadline",
                "robotProfile",
                "safetyPolicy",
                "labels",
            },
            set(body["goal"]),
        )
        self.assertEqual({"name", "version", "digest"}, set(body["goal"]["robotProfile"]))
        self.assertEqual({"name", "version", "digest"}, set(body["goal"]["safetyPolicy"]))
        self.assertEqual({"resource_id", "controller"}, set(body["goal"]["labels"]))
        self.assertEqual({"positions": {"j1": 0.1}, "duration_ms": 500}, body["goal"]["inputs"])
        json.dumps(body)  # 必须可序列化为 JSON

    def test_deadline_is_iso_utc_with_z_suffix_within_budget(self):
        import datetime

        body = self.body(deadline_s=45)
        value = body["goal"]["deadline"]
        self.assertTrue(value.endswith("Z"), value)
        parsed = datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))
        now = datetime.datetime.now(datetime.timezone.utc)
        delta = (parsed - now).total_seconds()
        self.assertGreater(delta, 40)
        self.assertLessEqual(delta, 45.5)

    def test_resource_id_defaults_to_profile_name_and_controller_is_declared(self):
        body = self.body()
        self.assertEqual("piper", body["goal"]["labels"]["resource_id"])
        self.assertEqual("iraf-sdk", body["goal"]["labels"]["controller"])
        explicit = self.body(resource_id="quadruped.base", controller="studio")
        self.assertEqual("quadruped.base", explicit["goal"]["labels"]["resource_id"])
        self.assertEqual("studio", explicit["goal"]["labels"]["controller"])

    def test_ids_default_to_generated_uuid_and_stay_consistent(self):
        body = self.body(request_id=None, idempotency_key=None, correlation_id=None)
        self.assertEqual(body["requestId"], body["idempotencyKey"])
        self.assertEqual(body["requestId"], body["goal"]["correlationId"])
        self.assertTrue(body["requestId"])

    def test_incomplete_identity_is_rejected(self):
        for field in ("name", "version", "digest"):
            with self.subTest(field=field):
                broken = dict(PROFILE)
                broken.pop(field)
                with self.assertRaises(SdkConfigError):
                    self.body(profile=broken)
        with self.assertRaises(SdkConfigError):
            self.body(profile={"name": "piper", "version": "1.0.0", "digest": "  "})

    def test_identity_of_accepts_mapping_and_object(self):
        class Artifact:
            name = "ur5e"
            version = "2.0.0"
            digest = "c" * 64

        self.assertEqual({"name": "ur5e", "version": "2.0.0", "digest": "c" * 64}, identity_of(Artifact(), "profile"))
        self.assertEqual(identity_of(PROFILE, "profile"), PROFILE)

    def test_non_object_parameters_are_rejected(self):
        for value in ("{}", ["positions"], 3):
            with self.subTest(value=repr(value)):
                with self.assertRaises(SdkConfigError):
                    self.body(parameters=value)

    def test_blank_skill_or_deadline_is_rejected(self):
        with self.assertRaises(SdkConfigError):
            self.body(skill="   ")
        for value in (0, -1, None, "30"):
            with self.subTest(deadline_s=repr(value)):
                with self.assertRaises(SdkConfigError):
                    canonical_execute_body(
                        "move_joint", profile=PROFILE, safety_policy=SAFETY, deadline_s=value
                    )

    def test_deadline_after_requires_positive_budget(self):
        with self.assertRaises(SdkConfigError):
            deadline_after(0)
        self.assertTrue(deadline_after(1).endswith("Z"))


class _StubHandler(http.server.BaseHTTPRequestHandler):
    """本机 loopback stub：只证明 SDK 侧协议与解析逻辑，不代表运行时健康。"""

    routes = {}
    captured = []

    def _handle(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        type(self).captured.append(
            {
                "method": self.command,
                "path": self.path,
                "headers": {key.lower(): value for key, value in self.headers.items()},
                "body": raw.decode("utf-8") if raw else "",
            }
        )
        status, body = type(self).routes.get((self.command, self.path), (404, b'{"error":"not found"}'))
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    do_GET = _handle
    do_POST = _handle

    def log_message(self, fmt, *args):  # 保持测试输出干净
        return


class HttpProtocolStubTests(unittest.TestCase):
    """HTTP 客户端协议用例（本机 stub，非目标端证据）。"""

    @classmethod
    def setUpClass(cls):
        cls.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _StubHandler)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5)

    def setUp(self):
        _StubHandler.routes = {}
        _StubHandler.captured = []
        self.client = HttpSkillClient(endpoint=f"http://127.0.0.1:{self.port}", token="token-1", timeout_s=5)

    def submit(self, **overrides):
        kwargs = dict(
            skill="move_joint",
            profile=PROFILE,
            safety_policy=SAFETY,
            parameters={"positions": {"j1": 0.1}, "duration_ms": 500},
            request_id="request-1",
            idempotency_key="idem-1",
            correlation_id="corr-1",
        )
        kwargs.update(overrides)
        return self.client.submit(**kwargs)

    def test_submit_posts_canonical_body_to_tasks_path(self):
        expected = {"status": "SUCCEEDED", "error_code": "", "execution_id": "execution-1", "reason": ""}
        _StubHandler.routes[("POST", "/v1/tasks")] = (200, json.dumps(expected).encode())
        self.assertEqual(expected, self.submit())
        self.assertEqual(1, len(_StubHandler.captured))
        captured = _StubHandler.captured[0]
        self.assertEqual("POST", captured["method"])
        self.assertEqual("/v1/tasks", captured["path"])
        self.assertEqual("Bearer token-1", captured["headers"]["authorization"])
        self.assertEqual("application/json", captured["headers"]["content-type"])
        sent = json.loads(captured["body"])
        expected = canonical_execute_body(
            "move_joint",
            profile=PROFILE,
            safety_policy=SAFETY,
            parameters={"positions": {"j1": 0.1}, "duration_ms": 500},
            request_id="request-1",
            idempotency_key="idem-1",
            correlation_id="corr-1",
            deadline_s=5,
        )
        # deadline 由调用时刻生成，两次构造必然有毫秒级差异：单独断言其形状，其余字段逐字节比对。
        sent_deadline = sent["goal"].pop("deadline")
        expected_deadline = expected["goal"].pop("deadline")
        self.assertTrue(sent_deadline.endswith("Z") and expected_deadline.endswith("Z"))
        self.assertEqual(expected, sent)

    def test_server_failure_payload_is_diagnosed_in_chinese(self):
        payload = {"status": "FAILED", "error_code": "IRAF-INPUT-INVALID", "reason": "positions 缺少必填键", "execution_id": "e"}
        _StubHandler.routes[("POST", "/v1/tasks")] = (200, json.dumps(payload, ensure_ascii=False).encode())
        result = self.submit()
        self.assertEqual(payload, result)
        with self.assertRaises(IrafError) as raised:
            raise_for_status(result)
        self.assertEqual("IRAF-INPUT-INVALID", raised.exception.code)
        self.assertIn("下一步：", str(raised.exception))

    def test_get_execution_uses_tasks_path_and_returns_payload(self):
        payload = {"status": "RUNNING", "error_code": "", "execution_id": "execution-abc", "reason": ""}
        _StubHandler.routes[("GET", "/v1/tasks/execution-abc")] = (200, json.dumps(payload).encode())
        self.assertEqual(payload, self.client.get_execution("execution-abc"))
        self.assertEqual("GET", _StubHandler.captured[0]["method"])
        self.assertEqual("/v1/tasks/execution-abc", _StubHandler.captured[0]["path"])

    def test_http_error_without_error_code_becomes_transport_error(self):
        _StubHandler.routes[("POST", "/v1/tasks")] = (401, b'{"error":"unauthorized"}')
        with self.assertRaises(SdkTransportError) as raised:
            self.submit()
        error = raised.exception
        self.assertEqual(TRANSPORT_FAILED_CODE, error.code)
        self.assertEqual(401, error.http_status)
        self.assertEqual({"error": "unauthorized"}, error.payload)
        text = str(error)
        self.assertIn("未包含 IDL error_code", text)
        self.assertIn("不代表请求未被服务端处理", text)

    def test_non_json_success_body_becomes_transport_error(self):
        _StubHandler.routes[("GET", "/v1/tasks/execution-abc")] = (200, b"<html>not json</html>")
        with self.assertRaises(SdkTransportError):
            self.client.get_execution("execution-abc")

    def test_unreachable_endpoint_becomes_transport_error(self):
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        closed_port = probe.getsockname()[1]
        probe.close()
        client = HttpSkillClient(endpoint=f"http://127.0.0.1:{closed_port}", token="token-1", timeout_s=2)
        with self.assertRaises(SdkTransportError) as raised:
            client.get_execution("execution-abc")
        self.assertEqual(TRANSPORT_FAILED_CODE, raised.exception.code)
        self.assertIn("HTTP 传输失败", str(raised.exception))

    def test_http_adapter_gaps_fail_explicitly_instead_of_bypassing(self):
        with self.assertRaises(SdkUnsupportedError):
            self.client.cancel("execution-1", "operator cancelled")
        with self.assertRaises(SdkUnsupportedError):
            self.client.list_events(execution_id="execution-1")
        with self.assertRaises(SdkUnsupportedError):
            self.client.get_replay_manifest("execution-1")
        self.assertEqual([], _StubHandler.captured, "不支持的路径必须直接拒绝，禁止发出任何旁路请求")


class ClientConfigTests(unittest.TestCase):
    """配置类 fail-closed：缺参数即显式失败（禁止隐式默认值）。"""

    def test_http_client_requires_endpoint_token_and_timeout(self):
        with self.assertRaises(SdkConfigError):
            HttpSkillClient(endpoint="", token="token", timeout_s=5)
        with self.assertRaises(SdkConfigError):
            HttpSkillClient(endpoint="http://127.0.0.1:8765", token="  ", timeout_s=5)
        # timeout_s 无默认值：缺参直接是调用错误（禁止无超时等待）
        with self.assertRaises(TypeError):
            HttpSkillClient(endpoint="http://127.0.0.1:8765", token="token")
        with self.assertRaises(SdkConfigError):
            HttpSkillClient(endpoint="http://127.0.0.1:8765", token="token", timeout_s=0)

    def test_grpc_client_validates_before_importing_grpc(self):
        """参数校验先于任何传输依赖：缺参时即便没有 grpcio 也必须是 SdkConfigError。"""
        with self.assertRaises(SdkConfigError):
            GrpcSkillClient(target="", token="token", timeout_s=5)
        with self.assertRaises(SdkConfigError):
            GrpcSkillClient(target="127.0.0.1:50051", token="", timeout_s=5)
        with self.assertRaises(SdkConfigError):
            GrpcSkillClient(target="127.0.0.1:50051", token="token", timeout_s=-1)

    def test_grpc_event_limit_matches_server_bound(self):
        client = GrpcSkillClient(target="127.0.0.1:50051", token="token", timeout_s=5)
        for value in (0, 1001, "10", True):
            with self.subTest(limit=repr(value)):
                with self.assertRaises(SdkConfigError):
                    client.list_events(limit=value)

    def test_endpoint_trailing_slash_is_normalized(self):
        client = HttpSkillClient(endpoint="http://127.0.0.1:8765/", token="token", timeout_s=5)
        self.assertEqual("http://127.0.0.1:8765", client.endpoint)


class PackagingContractTests(unittest.TestCase):
    """SDK 必须随包发布，且版本号两处一致（声明与产物不脱节）。"""

    def test_pyproject_includes_iraf_sdk_package(self):
        raw = PYPROJECT.read_text(encoding="utf-8")
        self.assertIn("iraf_sdk*", raw, "pyproject 的 packages.find.include 未包含 iraf_sdk*")

    def test_sdk_version_matches_pyproject_version(self):
        raw = PYPROJECT.read_text(encoding="utf-8")
        match = re.search(r'^version\s*=\s*"([^"]+)"', raw, re.MULTILINE)
        self.assertIsNotNone(match, "未找到 [project].version")
        self.assertEqual(match.group(1), __version__)


def _proto_stubs_available():
    """gRPC 消息层用例的前置条件：生成的 stub 可导入（需要 protobuf>=5.29）。

    实测（步骤 05）：/usr/bin/python3 的 protobuf 为 4.25.7，生成的 stub 需要 5.x 的
    `runtime_version`，因此本机默认**跳过**这组用例——跳过原因会原样写入验收输出，
    不伪造通过。把生成目录加入 PYTHONPATH 并升级 protobuf 后同一组用例会自动执行。
    """
    if not (GENERATED_DIR / "iraf" / "v1" / "skill_pb2.py").is_file():
        return False, f"{GENERATED_DIR} 下没有生成的 proto stub"
    added = str(GENERATED_DIR) not in sys.path
    if added:
        sys.path.insert(0, str(GENERATED_DIR))
    try:
        importlib.import_module("iraf.v1.skill_pb2")
    except Exception as exc:  # noqa: BLE001 - 跳过原因需要原样呈现
        if added:
            sys.path.remove(str(GENERATED_DIR))
        return False, f"{type(exc).__name__}: {exc}"
    return True, ""


PROTO_STUBS_OK, PROTO_STUBS_REASON = _proto_stubs_available()


@unittest.skipUnless(
    PROTO_STUBS_OK,
    f"需要生成的 IDL stub 与 protobuf>=5.29；当前不可用（{PROTO_STUBS_REASON}）——不伪造该层证据",
)
class GrpcMessageLayerTests(unittest.TestCase):
    """gRPC 消息层（不含 RPC 通道）：用真实生成的 stub 验证请求构造与反馈解析。

    只覆盖消息层：`grpc` 通道（SkillRuntimeServiceStub 等）需要 grpcio，本机未安装，
    属于**未验证**项，不在本用例中断言"通过"。
    """

    def setUp(self):
        from iraf.v1 import events_pb2, skill_pb2

        self.skill_pb2 = skill_pb2
        self.events_pb2 = events_pb2

    def body(self):
        return canonical_execute_body(
            "move_joint",
            profile=PROFILE,
            safety_policy=SAFETY,
            parameters={"positions": {"j1": 0.1}, "duration_ms": 500},
            request_id="request-1",
            idempotency_key="idem-1",
            correlation_id="corr-1",
            deadline_s=30,
        )

    def test_canonical_body_round_trips_through_generated_request(self):
        request = execute_request_from_body(self.body())
        self.assertEqual("request-1", request.request_id)
        self.assertEqual("idem-1", request.idempotency_key)
        self.assertEqual("corr-1", request.goal.correlation_id)
        self.assertEqual("move_joint", request.goal.skill_name)
        self.assertEqual("piper", request.goal.robot_profile.name)
        self.assertEqual(SAFETY["digest"], request.goal.safety_policy.digest)
        self.assertEqual("piper", dict(request.goal.labels)["resource_id"])
        self.assertEqual(0.1, request.goal.inputs.fields["positions"].struct_value.fields["j1"].number_value)
        self.assertGreater(request.goal.deadline.seconds, 0)

    def test_extra_fields_are_rejected_not_silently_dropped(self):
        broken = self.body()
        broken["unknownTopLevel"] = 1
        with self.assertRaises(Exception):
            execute_request_from_body(broken)

    def test_feedback_maps_to_canonical_result_dict(self):
        feedback = self.skill_pb2.SkillFeedback(execution_id="execution-1", sequence=7, phase="terminal")
        feedback.state = self.skill_pb2.SKILL_STATE_FAILED
        feedback.error.code = "IRAF-POLICY-DENIED"
        feedback.error.message_zh = "调用方无任务提交权限"
        feedback.outputs.update({"detail": "denied"})
        result = result_from_feedback(feedback)
        self.assertEqual("FAILED", result["status"])
        self.assertEqual("IRAF-POLICY-DENIED", result["error_code"])
        self.assertEqual("调用方无任务提交权限", result["reason"])
        self.assertEqual({"detail": "denied"}, result["result"])
        with self.assertRaises(IrafError):
            raise_for_status(result)

    def test_snapshot_and_event_conversions_keep_sequence_and_timestamps(self):
        snapshot = self.skill_pb2.ExecutionSnapshot(execution_id="execution-1", last_sequence=3)
        snapshot.state = self.skill_pb2.SKILL_STATE_RUNNING
        converted = result_from_snapshot(snapshot)
        self.assertEqual({"status": "RUNNING", "sequence": 3, "error_code": ""}, {
            "status": converted["status"],
            "sequence": converted["sequence"],
            "error_code": converted["error_code"],
        })
        event = self.events_pb2.EventRecord(event_id="event-1", execution_id="execution-1", sequence=2, kind="SAFETY")
        event.occurred_at.seconds = 1_700_000_000
        event.occurred_at.nanos = 123_000_000
        self.assertEqual(1_700_000_000_123, event_to_dict(event)["occurred_at_ms"])
        self.assertEqual("SAFETY", event_to_dict(event)["kind"])


class SdkImportPurityTests(unittest.TestCase):
    """SDK 必须保持纯标准库：不 import mujoco/grpc/iraf_core（跨架构、且无旁路）。"""
    PROBE = (
        "import sys, iraf_sdk\n"
        "from iraf_sdk import HttpSkillClient, describe, canonical_execute_body, raise_for_status\n"
        "assert 'mujoco' not in sys.modules, 'mujoco 被导入'\n"
        "assert 'grpc' not in sys.modules, 'grpc 被导入'\n"
        "assert 'iraf_core' not in sys.modules, 'iraf_core 被导入（旁路风险）'\n"
        "assert 'iraf_adapters' not in sys.modules, 'iraf_adapters 被导入（旁路风险）'\n"
        "describe('IRAF-POLICY-DENIED', 'x')\n"
        "canonical_execute_body('move_joint', profile={'name':'p','version':'1','digest':'d'},"
        " safety_policy={'name':'s','version':'1','digest':'d'}, deadline_s=1)\n"
        "raise_for_status({'status':'SUCCEEDED','error_code':''})\n"
        "HttpSkillClient(endpoint='http://127.0.0.1:1', token='t', timeout_s=1)\n"
        "assert 'mujoco' not in sys.modules and 'iraf_core' not in sys.modules\n"
        "print('ok')\n"
    )

    def _run_probe(self, code):
        env = dict(os.environ)
        env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
        return subprocess.run(
            [sys.executable, "-c", code], capture_output=True, text=True, env=env, timeout=120, cwd=str(REPO_ROOT)
        )

    def test_import_and_use_sdk_never_pulls_heavy_or_framework_modules(self):
        completed = self._run_probe(self.PROBE)
        self.assertEqual(0, completed.returncode, f"探针失败：{completed.stderr}")
        self.assertIn("ok", completed.stdout)


if __name__ == "__main__":
    unittest.main()
