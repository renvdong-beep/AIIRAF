"""步骤 09：uninstall.sh 的「只动本 bundle」范围门禁测试。

要点：卸载的唯一依据是安装记录（delivery.install.file_record）。测试用**生产侧实现**
生成记录（不手写，避免测试与实现漂移），并通过 **bash 入口** 调用以同时验证退出码传递
（步骤 08 实测过 `if ! cmd; then rc=$?` 吞退出码的坑）。
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEPLOY = REPO_ROOT / "deploy" / "sdk"
UNINSTALL = DEPLOY / "uninstall.sh"

try:  # 目标端声明校验需要 PyYAML + jsonschema（缺则整类跳过，不伪造通过）
    import jsonschema  # noqa: F401
    import yaml  # noqa: F401

    DEPS_OK = True
except ImportError:  # pragma: no cover - 环境缺依赖
    DEPS_OK = False


def _python() -> str:
    return os.environ.get("IRAF_TEST_PYTHON") or sys.executable


def build_sandbox(root: Path, version: str = "0.2.0") -> dict:
    """构造一个"已安装"沙箱：版本目录 + 激活链接 + env + systemd 单元 + 日志目录。"""
    version_dir = root / "opt" / "iraf-sdk" / version
    (version_dir / "config" / "sdk").mkdir(parents=True)
    for name in ("package_matrix.yaml", "package_matrix.schema.json"):
        shutil.copy2(REPO_ROOT / "config" / "sdk" / name, version_dir / "config" / "sdk" / name)
    (version_dir / "scripts").mkdir()
    (version_dir / "scripts" / "verify.sh").write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    (version_dir / "lib" / "iraf_sdk").mkdir(parents=True)
    (version_dir / "lib" / "iraf_sdk" / "__init__.py").write_text(
        "__version__ = '0.2.0'\n", encoding="utf-8"
    )
    unit = root / "etc" / "systemd" / "system" / "iraf-sdk-board.service"
    unit.parent.mkdir(parents=True)
    unit.write_text("[Unit]\nDescription=测试占位单元\n", encoding="utf-8")
    env_file = root / "etc" / "iraf" / "iraf-sdk.env"
    env_file.parent.mkdir(parents=True)
    env_file.write_text("IRAF_EVENT_STORE=/var/lib/iraf/iraf-events.db\n", encoding="utf-8")
    (root / "var" / "log" / "iraf").mkdir(parents=True)
    (root / "opt" / "iraf-sdk" / "current").symlink_to(version)
    (root / "opt" / "iraf-sdk" / "未登记的邻居.txt").write_text(
        "prefix 下不属于本 bundle 的文件，卸载必须保留\n", encoding="utf-8"
    )
    return {
        "root": root,
        "version_dir": version_dir,
        "unit": unit,
        "env_file": env_file,
        "log_dir": root / "var" / "log" / "iraf",
        "record": version_dir / "iraf-sdk-files.json",
        "neighbour": root / "opt" / "iraf-sdk" / "未登记的邻居.txt",
    }


def write_record(sandbox: dict) -> dict:
    """用生产侧实现写安装记录（与 verify.sh 走的是同一段代码）。"""
    if str(DEPLOY) not in sys.path:
        sys.path.insert(0, str(DEPLOY))
    from lib_manifest import load_matrix  # noqa: PLC0415
    from lib_target_verify import load_verify_declaration, write_install_record  # noqa: PLC0415

    version_dir = sandbox["version_dir"]
    matrix = load_matrix(version_dir, version_dir / "config" / "sdk" / "package_matrix.yaml")
    declaration = load_verify_declaration(matrix)
    record, path = write_install_record(
        root=sandbox["root"],
        version_dir=version_dir,
        install=declaration["install"],
        health=declaration["health"],
        version=version_dir.name,
    )
    assert path == sandbox["record"], (path, sandbox["record"])
    return record


def run_uninstall(sandbox: dict, *args: str) -> tuple[int, dict | None, str]:
    env = dict(os.environ)
    env["PYTHON"] = _python()
    proc = subprocess.run(
        ["bash", str(UNINSTALL), "--root", str(sandbox["root"]), *args],
        capture_output=True,
        text=True,
        env=env,
        timeout=300,
    )
    payload = None
    if proc.stdout.strip():
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError:
            payload = None
    return proc.returncode, payload, proc.stderr


@unittest.skipUnless(DEPS_OK, "缺少 PyYAML/jsonschema：声明层校验无法进行（环境缺依赖，非跳过通过）")
class UninstallScopeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="iraf-uninstall-"))
        self.sandbox = build_sandbox(self.tmp)
        write_record(self.sandbox)

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_record_covers_only_installed_files(self) -> None:
        record = json.loads(self.sandbox["record"].read_text(encoding="utf-8"))
        paths = {item["path"] for item in record["files"]}
        self.assertIn("lib/iraf_sdk/__init__.py", paths)
        self.assertNotIn("未登记的邻居.txt", paths)  # 前缀下的邻居不在版本目录内
        self.assertEqual(record["schema_version"], "iraf.sdk-install-record/v1")
        self.assertEqual(len(record["external"]), 2)

    def test_record_regenerated_twice_stays_consistent(self) -> None:
        """回归（2026-09-20 实测缺陷）：记录被生成**两次**后，上一份记录不得进入 files。

        verify.sh 的后置校验会在 install 之后再写一次记录；写入侧若只清点目录、不排除记录自身，
        第二次就会把上一份记录登记进 files，而比对侧又把它排除 → 卸载报
        「记录内文件缺失 1 个：… - iraf-sdk-files.json」并在**全量树**上整体拒绝。
        只生成一次记录的沙箱测不出来（首轮验收 24 项中失败的正是那 2 项），故此处显式生成两次。
        """
        first = write_record(self.sandbox)
        second = write_record(self.sandbox)
        self.assertNotIn(
            "iraf-sdk-files.json",
            {item["path"] for item in second["files"]},
            "写入侧必须排除记录自身，否则第二次生成会把上一份记录登记进 files",
        )
        self.assertEqual(len(second["files"]), len(first["files"]))
        rc, payload, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 0, stderr)
        self.assertIsNotNone(payload)
        comparison = payload["comparison"]
        self.assertEqual(comparison["record_file_excluded"], "iraf-sdk-files.json")
        self.assertEqual(comparison["missing"], [])
        self.assertEqual(comparison["extras"], [])
        self.assertEqual(
            comparison["file_count_record"], comparison["file_count_actual"]
        )

    def test_dry_run_passes_and_deletes_nothing(self) -> None:
        rc, payload, stderr = run_uninstall(self.sandbox, "--dry-run")
        self.assertEqual(rc, 0, stderr)
        self.assertIsNotNone(payload)
        self.assertEqual(payload["mode"], "rehearsal")
        self.assertEqual(payload["evidence_scope"], "rehearsal_only")
        self.assertTrue(payload["simulation"])
        self.assertTrue(self.sandbox["version_dir"].is_dir())
        self.assertTrue(self.sandbox["env_file"].is_file())
        self.assertEqual(payload["removed"], [])

    def test_extra_file_outside_record_is_refused(self) -> None:
        foreign = self.sandbox["version_dir"] / "extra" / "foreign.txt"
        foreign.parent.mkdir(parents=True)
        foreign.write_text("清单外文件\n", encoding="utf-8")
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 1, stderr)
        self.assertIn("清单外文件", stderr)
        self.assertTrue(foreign.is_file(), "拒绝卸载时不得删除任何文件")
        self.assertTrue(self.sandbox["version_dir"].is_dir())

    def test_tampered_file_is_refused(self) -> None:
        target = self.sandbox["version_dir"] / "lib" / "iraf_sdk" / "__init__.py"
        target.write_text("__version__ = '9.9.9'\n", encoding="utf-8")
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 4, stderr)
        self.assertIn("安装记录不一致", stderr)
        self.assertTrue(self.sandbox["version_dir"].is_dir())

    def test_missing_record_is_refused(self) -> None:
        self.sandbox["record"].unlink()
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 1, stderr)
        self.assertIn("安装记录", stderr)
        self.assertTrue(self.sandbox["version_dir"].is_dir())

    def test_missing_installed_version_is_refused(self) -> None:
        shutil.rmtree(self.sandbox["version_dir"])
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 1, stderr)
        self.assertIn("拒绝卸载", stderr)

    def test_apply_removes_only_recorded_files_and_keeps_data(self) -> None:
        rc, payload, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 0, stderr)
        self.assertIsNotNone(payload)
        self.assertFalse(self.sandbox["version_dir"].exists(), "版本目录必须删除")
        self.assertFalse(self.sandbox["unit"].is_file(), "systemd 单元必须删除")
        self.assertFalse(self.sandbox["env_file"].is_file(), "env 文件必须删除")
        self.assertFalse((self.sandbox["root"] / "opt" / "iraf-sdk" / "current").is_symlink())
        self.assertTrue(self.sandbox["log_dir"].is_dir(), "日志目录属数据，必须保留")
        self.assertTrue(self.sandbox["neighbour"].is_file(), "前缀下的邻居文件必须保留")
        self.assertIn("var/log/iraf", json.dumps(payload["kept"], ensure_ascii=False))
        # 删除项 = systemd 单元 + env 文件 + 激活链接 + 版本目录
        self.assertEqual(len(payload["removed"]), 4)

    def test_second_apply_is_refused(self) -> None:
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 0, stderr)
        rc, _, stderr = run_uninstall(self.sandbox)
        self.assertEqual(rc, 1, stderr)
        self.assertIn("拒绝卸载", stderr)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
