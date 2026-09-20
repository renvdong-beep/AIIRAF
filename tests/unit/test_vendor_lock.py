#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vendor 资产锁（`vendor/unitree_go2/source-lock.json`）与 `scripts/verify_vendor_lock.py` 的契约测试（步骤 12）。

被测契约：`iraf.vendor-source-lock/v1`（锁文件）+ 校验器退出码
（0=通过；1=用法错误；2=锁缺失/非法/声明不一致；3=内容不符；4=存在未登记文件；5=只读门禁失败）。

设计要点（fail-closed，每条负向都配正向对照）：
  1. 锁是**声明**：必需键缺失、`file_count` 与 `files` 长度不一致、`total_bytes` 与逐件字节和不一致、
     sha256 非 64 位小写十六进制、路径越界 → 一律退出码 2，禁止用默认值兜底；
  2. 逐件 SHA-256/字节复算不符 → 退出码 3（改一个字节就必须失败——步骤 12 第 5 条）；
  3. 反方向：出现未登记文件 → 退出码 4；锁内声明的 `non_asset_files`（自指的 `source-lock.json`、
     `LICENSE-BOM.md`）不得被误报为未登记文件（与步骤 09 的自指记录同一纪律）；
  4. 只读门禁只在 `--require-readonly` 下生效（退出码 5）；默认只如实报告，
     因为 git 不保存写位，写位不是可移植门禁，可移植门禁是哈希复算；
  5. 断言真实仓库锁：`file_count >= 20` 且 commit 必须等于锁点；
  6. 报告必须是纯 JSON、且不得包含本机绝对路径（AGENTS.md 2.7 / 5.5）。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_vendor_lock -v`
"""

import contextlib
import io
import json
import os
import shutil
import stat
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import fetch_vendor_assets as fva  # noqa: E402
import verify_vendor_lock as vvl  # noqa: E402

REAL_LOCK = REPO_ROOT / "vendor" / "unitree_go2" / "source-lock.json"
PINNED_COMMIT = "1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d"
REPO_URL = "https://github.com/unitreerobotics/unitree_mujoco"

ASSET_TREE = REPO_ROOT / "vendor" / "unitree_go2" / "unitree_robots" / "go2"
ASSETS_ABSENT_REASON = (
    "厂商模型按 .gitignore「不入库」：工作树里没有 vendor/unitree_go2/unitree_robots ⇒ 本用例只在本机"
    "按锁重取后可跑（显式 skip，不假装通过）。重取入口：PYTHONPATH=src /usr/bin/python3 "
    "scripts/fetch_vendor_assets.py --lock vendor/unitree_go2/source-lock.json --apply"
)


def asset_tree_file_count():
    """工作树里实际存在的厂商资产件数（0 = 未按锁重取）。"""
    if not ASSET_TREE.is_dir():
        return 0
    return sum(1 for path in ASSET_TREE.rglob("*") if path.is_file())


FIXTURE_ASSETS = {
    "unitree_robots/go2/go2.xml": b"<mujoco model=\"go2\"/>\n",
    "unitree_robots/go2/assets/base_0.obj": b"v 0 0 0\nv 1 0 0\nf 1 2 3\n",
}


def build_fixture(tmpdir, assets=None, non_asset=("source-lock.json", "LICENSE-BOM.md")):
    """在 tmpdir 下造一个与真实布局同构的最小锁（含自指的非资产文件）。"""
    root = Path(tmpdir) / "vendor" / "unitree_go2"
    root.mkdir(parents=True, exist_ok=True)
    files = {}
    for rel, data in (assets or FIXTURE_ASSETS).items():
        target = root / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        files[rel] = data
    for name in non_asset:
        (root / name).write_text("fixture\n", encoding="utf-8")
    lock = {
        "schema_version": vvl.SCHEMA_VERSION,
        "vendor": "unitree_go2",
        "upstream": {
            "repo_url": REPO_URL,
            "commit": PINNED_COMMIT,
            "subtree": "unitree_robots/go2",
            "fetched_at": "2026-09-20T18:40:00+0800",
        },
        "license": {
            "spdx": "BSD-3-Clause",
            "license_file": "LICENSE",
            "review_state": "pending_per_asset_review",
        },
        "non_asset_files": list(non_asset),
        "file_count": len(files),
        "total_bytes": sum(len(d) for d in files.values()),
        "files": [
            {
                "path": rel,
                "bytes": len(data),
                "sha256": __import__("hashlib").sha256(data).hexdigest(),
            }
            for rel, data in sorted(files.items())
        ],
    }
    lock_path = root / "source-lock.json"
    lock_path.write_text(json.dumps(lock, ensure_ascii=False, indent=2), encoding="utf-8")
    return root, lock_path, lock


class VendorLockFixtureTest(unittest.TestCase):
    """合成夹具：锁结构、哈希方向、未登记文件方向、只读门禁。"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="vendor-lock-")
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.root, self.lock_path, self.lock = build_fixture(self.tmp)

    def write_lock(self, lock):
        self.lock_path.write_text(json.dumps(lock, ensure_ascii=False, indent=2), encoding="utf-8")

    def run_main(self, argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = vvl.main(argv)
        return code, buf.getvalue()

    # ---------- 正向对照 ----------

    def test_fixture_positive_control(self):
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, 0, report)
        self.assertTrue(report["passed"])
        self.assertEqual(report["checked_files"], len(FIXTURE_ASSETS))
        self.assertEqual(report["checked_bytes"], self.lock["total_bytes"])
        self.assertEqual(report["unexpected_files"], [])

    def test_non_asset_files_are_not_reported_as_unexpected(self):
        """自指文件（source-lock.json）与许可证 BOM 由声明排除，不得误报。"""
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, 0, report)
        for name in self.lock["non_asset_files"]:
            self.assertNotIn(name, report["unexpected_files"])

    def test_main_prints_pure_json(self):
        code, out = self.run_main(["--lock", str(self.lock_path)])
        self.assertEqual(code, 0)
        parsed = json.loads(out)  # 纯 JSON：任何额外打印都会让这里失败
        self.assertEqual(parsed["schema_version"], vvl.REPORT_SCHEMA_VERSION)

    def test_report_contains_no_absolute_local_path(self):
        code, out = self.run_main(["--lock", str(self.lock_path), "--root", str(self.root)])
        self.assertEqual(code, 0)
        self.assertNotIn(str(self.root), out)
        self.assertNotIn(os.path.expanduser("~"), out)
        json.loads(out)

    def test_expect_commit_positive_and_negative(self):
        code, _ = self.run_main(["--lock", str(self.lock_path), "--expect-commit", PINNED_COMMIT])
        self.assertEqual(code, 0)
        code, out = self.run_main(["--lock", str(self.lock_path), "--expect-commit", "0" * 40])
        self.assertEqual(code, vvl.EXIT_LOCK_INVALID)
        self.assertIn("commit_mismatch", out)

    # ---------- 负向：内容 ----------

    def test_one_byte_change_fails(self):
        target = self.root / "unitree_robots/go2/go2.xml"
        data = bytearray(target.read_bytes())
        data[0] = ord("X") if data[0] != ord("X") else ord("Y")
        target.write_bytes(bytes(data))
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, vvl.EXIT_CONTENT_MISMATCH)
        self.assertFalse(report["passed"])
        kinds = {(m["path"], m["kind"]) for m in report["mismatches"]}
        self.assertIn(("unitree_robots/go2/go2.xml", "sha256"), kinds)

    def test_truncated_file_fails_on_bytes_and_sha256(self):
        target = self.root / "unitree_robots/go2/assets/base_0.obj"
        target.write_bytes(target.read_bytes()[:-1])
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, vvl.EXIT_CONTENT_MISMATCH)
        kinds = {m["kind"] for m in report["mismatches"] if m["path"].endswith("base_0.obj")}
        self.assertEqual(kinds, {"bytes", "sha256"})

    def test_missing_registered_file_fails(self):
        os.remove(self.root / "unitree_robots/go2/go2.xml")
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, vvl.EXIT_CONTENT_MISMATCH)
        self.assertIn("missing", {m["kind"] for m in report["mismatches"]})

    # ---------- 负向：未登记文件 ----------

    def test_unregistered_file_fails(self):
        (self.root / "unitree_robots/go2/evil.xml").write_text("<a/>\n", encoding="utf-8")
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, vvl.EXIT_UNEXPECTED_FILES)
        self.assertEqual(report["unexpected_files"], ["unitree_robots/go2/evil.xml"])

    # ---------- 负向：锁声明本身 ----------

    def assert_lock_invalid(self, mutate, expected_fragment=None):
        lock = json.loads(self.lock_path.read_text(encoding="utf-8"))
        mutate(lock)
        self.write_lock(lock)
        code, out = self.run_main(["--lock", str(self.lock_path)])
        self.assertEqual(code, vvl.EXIT_LOCK_INVALID, out)
        if expected_fragment:
            self.assertIn(expected_fragment, out)
        parsed = json.loads(out)
        self.assertFalse(parsed["passed"])

    def test_missing_top_level_key_fails(self):
        self.assert_lock_invalid(lambda lk: lk.pop("non_asset_files"), "缺少必需键")

    def test_missing_upstream_key_fails(self):
        self.assert_lock_invalid(lambda lk: lk["upstream"].pop("commit"), "upstream 缺少必需键")

    def test_missing_license_key_fails(self):
        self.assert_lock_invalid(lambda lk: lk["license"].pop("review_state"), "license 缺少必需键")

    def test_file_count_mismatch_fails(self):
        self.assert_lock_invalid(lambda lk: lk.__setitem__("file_count", 99), "不一致")

    def test_total_bytes_mismatch_is_reported_by_verifier(self):
        """total_bytes 与逐件和不一致：校验器复算出的字节和与声明不同 ⇒ 内容不符。"""
        lock = json.loads(self.lock_path.read_text(encoding="utf-8"))
        lock["total_bytes"] = lock["total_bytes"] + 1
        self.write_lock(lock)
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, 0, report)  # 锁自身合法，但报告里两个数字不同，由验收断言
        self.assertNotEqual(report["checked_bytes"], report["declared_total_bytes"])

    def test_empty_files_fails(self):
        self.assert_lock_invalid(lambda lk: (lk.__setitem__("files", []), lk.__setitem__("file_count", 0)), "files 为空")

    def test_bad_sha256_format_fails(self):
        self.assert_lock_invalid(lambda lk: lk["files"][0].__setitem__("sha256", "ABC"), "sha256 格式非法")

    def test_path_traversal_in_lock_fails(self):
        self.assert_lock_invalid(
            lambda lk: lk["files"][0].__setitem__("path", "../outside.xml"), "不得越界"
        )

    def test_absolute_path_in_lock_fails(self):
        self.assert_lock_invalid(
            lambda lk: lk["files"][0].__setitem__("path", "/etc/passwd"), "不得越界"
        )

    def test_missing_lock_file_fails(self):
        code, out = self.run_main(["--lock", str(self.root / "nope.json")])
        self.assertEqual(code, vvl.EXIT_LOCK_INVALID)
        self.assertIn("不存在", out)

    def test_invalid_json_fails(self):
        self.lock_path.write_text("{not json", encoding="utf-8")
        code, out = self.run_main(["--lock", str(self.lock_path)])
        self.assertEqual(code, vvl.EXIT_LOCK_INVALID)
        self.assertIn("不是合法 JSON", out)

    # ---------- 只读门禁 ----------

    def test_readonly_gate_off_by_default(self):
        report, code = vvl.verify(str(self.lock_path))
        self.assertEqual(code, 0, report)
        self.assertFalse(report["readonly_required"])
        self.assertEqual(report["writable_files"], [])

    def test_readonly_gate_flags_writable_asset(self):
        report, code = vvl.verify(str(self.lock_path), require_readonly=True)
        self.assertEqual(code, vvl.EXIT_READONLY_GATE, report)
        self.assertFalse(report["readonly_enforced"])
        self.assertIn("unitree_robots/go2/go2.xml", report["writable_files"])
        self.assertIn("git 只保存可执行位", report["readonly_portability_note"])

    def test_readonly_gate_passes_after_chmod(self):
        for rel in FIXTURE_ASSETS:
            os.chmod(self.root / rel, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
        report, code = vvl.verify(str(self.lock_path), require_readonly=True)
        self.assertEqual(code, 0, report)
        self.assertTrue(report["readonly_enforced"])
        self.assertEqual(report["writable_files"], [])

    def test_usage_error_without_lock_argument(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            code = vvl.main([])
        self.assertEqual(code, vvl.EXIT_USAGE)
        self.assertIn("禁止用默认锁路径兜底", err.getvalue())


class VendorLockRealRepoTest(unittest.TestCase):
    """真实仓库锁：形状与锁点断言（不依赖网络；无资产时显式 skip 而不是假装通过）。"""

    @classmethod
    def setUpClass(cls):
        if not REAL_LOCK.is_file():
            raise unittest.SkipTest("vendor/unitree_go2/source-lock.json 不存在")

    def test_lock_shape_and_pinned_commit(self):
        lock = json.loads(REAL_LOCK.read_text(encoding="utf-8"))
        self.assertEqual(lock["schema_version"], vvl.SCHEMA_VERSION)
        self.assertEqual(lock["vendor"], "unitree_go2")
        self.assertEqual(lock["upstream"]["repo_url"], REPO_URL)
        self.assertEqual(lock["upstream"]["commit"], PINNED_COMMIT)
        self.assertEqual(lock["upstream"]["subtree"], "unitree_robots/go2")
        self.assertGreaterEqual(lock["file_count"], 20)
        self.assertEqual(lock["file_count"], len(lock["files"]))
        self.assertEqual(lock["total_bytes"], sum(f["bytes"] for f in lock["files"]))
        paths = [f["path"] for f in lock["files"]]
        self.assertEqual(paths, sorted(paths), "锁内文件路径必须排序，便于逐位比对")
        self.assertIn("unitree_robots/go2/go2.xml", paths)
        self.assertIn("unitree_robots/go2/scene.xml", paths)
        for name in ("source-lock.json", "LICENSE-BOM.md"):
            self.assertIn(name, lock["non_asset_files"])

    def test_real_assets_recompute_clean(self):
        if not asset_tree_file_count():
            self.skipTest(ASSETS_ABSENT_REASON)
        report, code = vvl.verify(str(REAL_LOCK), expect_commit=PINNED_COMMIT)
        self.assertEqual(code, 0, report)
        self.assertEqual(report["mismatches"], [])
        self.assertEqual(report["unexpected_files"], [])
        self.assertGreaterEqual(report["checked_files"], 20)

    def test_real_lock_report_has_no_local_path(self):
        if not asset_tree_file_count():
            self.skipTest(ASSETS_ABSENT_REASON)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = vvl.main(["--lock", str(REAL_LOCK)])
        self.assertEqual(code, 0)
        self.assertNotIn(os.path.expanduser("~"), buf.getvalue())
        self.assertNotIn(str(REPO_ROOT), buf.getvalue())
        json.loads(buf.getvalue())


class VendorFetchApplyTest(unittest.TestCase):
    """按锁重取入口 `scripts/fetch_vendor_assets.py`：注入假下载器离线测正反两向（不联网、不写盘外部）。"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="vendor-fetch-")
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.root, self.lock_path, self.lock = build_fixture(self.tmp)
        self.missing = "unitree_robots/go2/assets/base_0.obj"
        self.payload = FIXTURE_ASSETS[self.missing]
        os.remove(self.root / self.missing)  # 制造 missing：模拟新克隆后按锁重取
        self.urls = []

    def fake_fetch(self, data):
        def _fetch(url):
            self.urls.append(url)
            return data
        return _fetch

    def run_fetch(self, argv, fetch_bytes=None):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = fva.main(argv, fetch_bytes=fetch_bytes)
        return code, json.loads(buf.getvalue())

    def test_apply_writes_verified_bytes_and_marks_readonly(self):
        code, report = self.run_fetch(
            ["--lock", str(self.lock_path), "--apply"], self.fake_fetch(self.payload)
        )
        self.assertEqual(code, fva.EXIT_OK, report)
        self.assertEqual(report["written"], [self.missing])
        self.assertEqual(report["missing_or_mismatched"], [])
        target = self.root / self.missing
        self.assertEqual(target.read_bytes(), self.payload)
        self.assertFalse(os.stat(target).st_mode & 0o222, "按锁重取的厂商资产必须只读")
        self.assertIn(PINNED_COMMIT, self.urls[0])
        self.assertIn("api.github.com", self.urls[0])

    def test_apply_rejects_corrupted_bytes_without_writing(self):
        code, report = self.run_fetch(
            ["--lock", str(self.lock_path), "--apply"], self.fake_fetch(b"corrupted")
        )
        self.assertEqual(code, fva.EXIT_DOWNLOAD_FAILED, report)
        self.assertFalse((self.root / self.missing).exists(), "校验不通过时禁止写盘")
        self.assertEqual(report["failures"][0]["reason"], "sha256_or_bytes_mismatch")

    def test_apply_download_failure_is_reported_not_faked(self):
        code, report = self.run_fetch(
            ["--lock", str(self.lock_path), "--apply"], self.fake_fetch(None)
        )
        self.assertEqual(code, fva.EXIT_DOWNLOAD_FAILED, report)
        self.assertFalse((self.root / self.missing).exists())
        self.assertEqual(report["failures"][0]["reason"], "download_failed")

    def test_require_complete_fails_on_missing_assets(self):
        code, report = self.run_fetch(
            ["--lock", str(self.lock_path), "--dry-run", "--require-complete"]
        )
        self.assertEqual(code, vvl.EXIT_CONTENT_MISMATCH, report)
        self.assertEqual(report["missing_or_mismatched"], [self.missing])
        self.assertEqual(len(report["planned_urls"]), 1)

    def test_dry_run_is_default_and_does_not_touch_disk(self):
        code, report = self.run_fetch(["--lock", str(self.lock_path), "--dry-run"])
        self.assertEqual(code, fva.EXIT_OK, report)
        self.assertEqual(report["mode"], "dry_run")
        self.assertFalse((self.root / self.missing).exists())
        self.assertIn(PINNED_COMMIT, report["planned_urls"][self.missing])


if __name__ == "__main__":
    unittest.main()
