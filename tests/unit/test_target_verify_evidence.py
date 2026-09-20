"""步骤 09：verify.sh 的证据契约与退出码测试（含"服务未启动不得视为通过"）。

覆盖：
1. 用法错误 / 缺伴随校验和 / 校验和不符 / bundle 无法解析 → 1 / 2 / 4 / 4
2. **服务未启动**：HTTP 探针指向关闭端口时，证据里 `state=SERVICE_NOT_RUNNING`、
   `verified=false`，且**非演练**退出 5（边界要求：不得当作通过）
3. 演练（--dry-run）如实记录未启动状态但退出 0（演练 ≠ 目标端证据）
4. 本地 HTTP stub 正向对照：探针与解析逻辑能跑通、`verified=true`（stub 级证据，
   不得表述为目标端健康检查通过）
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_uninstall_scope import DEPS_OK, REPO_ROOT, build_sandbox, write_record  # noqa: E402

DEPLOY = REPO_ROOT / "deploy" / "sdk"
VERIFY = DEPLOY / "verify.sh"
#: 关闭端口：本机无服务监听，用于稳定复现"服务未启动"（不用随机端口，避免竞态）。
CLOSED_PORT = 1


class _HealthStub(BaseHTTPRequestHandler):
    """本地 /health stub：只证明探针与解析逻辑（stub 级证据，不代表运行时健康）。"""

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler 接口
        body = json.dumps({"status": "ok", "service": "stub", "simulation": True}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):  # pragma: no cover - 静音测试输出
        pass


def _python() -> str:
    return os.environ.get("IRAF_TEST_PYTHON") or sys.executable


def run_verify(*args: str) -> tuple[int, dict | None, str]:
    env = dict(os.environ)
    env["PYTHON"] = _python()
    proc = subprocess.run(
        ["bash", str(VERIFY), *args], capture_output=True, text=True, env=env, timeout=600
    )
    payload = None
    if proc.stdout.strip():
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError:
            payload = None
    return proc.returncode, payload, proc.stderr


class VerifyCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="iraf-verify-cli-"))

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_help_exits_zero(self) -> None:
        rc, _, stderr = run_verify("--help")
        self.assertEqual(rc, 0, stderr)

    def test_no_arguments_is_usage_error(self) -> None:
        rc, _, _ = run_verify()
        self.assertEqual(rc, 1)

    def test_cannot_fake_success_with_missing_checksum_file(self) -> None:
        fake = self.tmp / "iraf-board-fake.tar.gz"
        fake.write_bytes(b"not a bundle")
        rc, _, stderr = run_verify("--bundle", str(fake), "--dry-run")
        self.assertEqual(rc, 2, stderr)
        self.assertIn("校验和", stderr)

    def test_checksum_mismatch_is_verify_failure(self) -> None:
        fake = self.tmp / "iraf-board-fake.tar.gz"
        fake.write_bytes(b"not a bundle")
        Path(str(fake) + ".sha256").write_text("0" * 64 + "  iraf-board-fake.tar.gz\n", encoding="utf-8")
        rc, _, stderr = run_verify("--bundle", str(fake), "--dry-run")
        self.assertEqual(rc, 4, stderr)
        self.assertIn("SHA-256", stderr)

    def test_unparseable_bundle_with_valid_checksum_is_verify_failure(self) -> None:
        import hashlib

        fake = self.tmp / "iraf-board-fake.tar.gz"
        fake.write_bytes(b"not a bundle at all")
        digest = hashlib.sha256(fake.read_bytes()).hexdigest()
        Path(str(fake) + ".sha256").write_text(f"{digest}  {fake.name}\n", encoding="utf-8")
        rc, _, stderr = run_verify("--bundle", str(fake), "--dry-run")
        self.assertEqual(rc, 4, stderr)


@unittest.skipUnless(DEPS_OK, "缺少 PyYAML/jsonschema：声明层校验无法进行（环境缺依赖）")
class VerifyInstalledTreeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="iraf-verify-tree-"))
        self.sandbox = build_sandbox(self.tmp)
        write_record(self.sandbox)
        self.installed_dir = str(self.sandbox["version_dir"])

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _base_args(self) -> list[str]:
        return [
            "--root",
            str(self.sandbox["root"]),
            "--installed-dir",
            self.installed_dir,
            "--health-url",
            f"http://127.0.0.1:{CLOSED_PORT}/health",
            "--grpc-target",
            f"127.0.0.1:{CLOSED_PORT}",
            "--timeout-s",
            "2",
        ]

    def test_service_not_running_is_reported_and_fails(self) -> None:
        rc, payload, stderr = run_verify(*self._base_args())
        self.assertEqual(rc, 5, stderr)
        self.assertIn("服务未启动", stderr)
        self.assertIsNotNone(payload)
        self.assertFalse(payload["verified"])
        self.assertEqual(payload["service_state"], "SERVICE_NOT_RUNNING")
        self.assertEqual(payload["health"]["state"], "SERVICE_NOT_RUNNING")
        self.assertEqual(payload["grpc"]["state"], "SERVICE_NOT_RUNNING")
        self.assertEqual(payload["evidence_scope"], "target_end_verify")
        self.assertTrue(self.sandbox["record"].is_file(), "自检必须生成安装记录")

    def test_dry_run_records_not_running_without_failing(self) -> None:
        rc, payload, stderr = run_verify(*self._base_args(), "--dry-run")
        self.assertEqual(rc, 0, stderr)
        self.assertIsNotNone(payload)
        self.assertFalse(payload["verified"], "演练不得声明已验证")
        self.assertEqual(payload["service_state"], "SERVICE_NOT_RUNNING")
        self.assertEqual(payload["evidence_scope"], "rehearsal_only")
        self.assertTrue(payload["simulation"])
        self.assertTrue(self.sandbox["record"].is_file())
        record = json.loads(self.sandbox["record"].read_text(encoding="utf-8"))
        self.assertGreaterEqual(len(record["files"]), 4)

    def test_installed_dir_outside_declared_layout_is_refused(self) -> None:
        elsewhere = self.tmp / "elsewhere" / "0.2.0"
        shutil.copytree(self.sandbox["version_dir"], elsewhere)
        args = list(self._base_args())
        args[args.index("--installed-dir") + 1] = str(elsewhere)
        rc, _, stderr = run_verify(*args)
        self.assertEqual(rc, 2, stderr)
        self.assertIn("声明布局不符", stderr)

    def test_health_stub_positive_control_marks_verified(self) -> None:
        server = ThreadingHTTPServer(("127.0.0.1", 0), _HealthStub)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            port = server.server_address[1]
            rc, payload, stderr = run_verify(
                "--root",
                str(self.sandbox["root"]),
                "--installed-dir",
                self.installed_dir,
                "--health-url",
                f"http://127.0.0.1:{port}/health",
                "--grpc-target",
                f"127.0.0.1:{port}",
                "--timeout-s",
                "2",
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
        self.assertEqual(rc, 0, stderr)
        self.assertIsNotNone(payload)
        self.assertTrue(payload["verified"])
        self.assertEqual(payload["health"]["status"], 200)
        self.assertEqual(payload["health"]["body_status"], "ok")
        # stub 只证明解析逻辑：gRPC 侧必须留下"未做协议层判定"的如实说明
        self.assertEqual(payload["grpc"]["probe_kind"], "tcp_connect")
        self.assertIn("不得表述为", payload["grpc"]["limitation"])


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
