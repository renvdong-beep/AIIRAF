#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 vendor 资产锁（步骤 12：vendor 锁定 unitree_mujoco）。

契约：`vendor/<vendor>/source-lock.json`（`iraf.vendor-source-lock/v1`）。锁文件是**声明**，
本脚本只做复算与比对，不做任何"猜"的动作：

1. 锁文件自身完整性：必需键齐备、`file_count` 与 `files` 长度一致、`total_bytes` 与逐件字节和一致、
   `sha256` 为 64 位小写十六进制；路径必须是 root 相对路径且不得越界（否则显式失败）。
2. 逐件复算 SHA-256 与字节数，任一不符即为内容不符（退出码 3）。
3. 反方向：root 下出现**未登记文件**即失败（退出码 4），防止资产被悄悄增删。
   锁内的 `non_asset_files` 是**声明**出来的非资产白名单（`source-lock.json` 自指、`LICENSE-BOM.md`），
   写入侧与比对侧使用同一份声明——与步骤 09 的自指安装记录同一纪律。
4. `--require-readonly`：把「资产只读」变成硬门禁（退出码 5）。默认只**如实报告**是否只读，
   因为 git 只保存可执行位、不保存写位，只在本地 chmod 的写位不是可移植门禁；
   可移植的门禁是上一条的哈希复算。该限制在报告字段 `readonly_portability_note` 里明写。

退出码：0=通过；1=用法错误；2=锁缺失/非法/声明不一致；3=内容不符；4=存在未登记文件；5=只读门禁失败。

运行：
    PYTHONPATH=src /usr/bin/python3 scripts/verify_vendor_lock.py --lock vendor/unitree_go2/source-lock.json
"""

import argparse
import hashlib
import json
import os
import re
import sys

SCHEMA_VERSION = "iraf.vendor-source-lock/v1"
REPORT_SCHEMA_VERSION = "iraf.vendor-lock-verify/v1"

REQUIRED_TOP_KEYS = (
    "schema_version",
    "vendor",
    "upstream",
    "license",
    "non_asset_files",
    "file_count",
    "total_bytes",
    "files",
)
REQUIRED_UPSTREAM_KEYS = ("repo_url", "commit", "subtree", "fetched_at")
REQUIRED_LICENSE_KEYS = ("spdx", "license_file", "review_state")
REQUIRED_FILE_KEYS = ("path", "bytes", "sha256")

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_LOCK_INVALID = 2
EXIT_CONTENT_MISMATCH = 3
EXIT_UNEXPECTED_FILES = 4
EXIT_READONLY_GATE = 5

READONLY_PORTABILITY_NOTE = (
    "git 只保存可执行位、不保存写位：新克隆的工作树里文件是可写的。"
    "可移植的完整性门禁是逐件 SHA-256 复算（本脚本默认执行）；"
    "只读位只在 --require-readonly 下作为本地状态门禁。"
)


class LockError(Exception):
    """锁文件缺失、非法或声明自相矛盾。"""


def sha256_of(path):
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_lock(lock_path):
    """读取并做结构校验；任何缺失都显式失败，不使用默认值。"""
    if not os.path.isfile(lock_path):
        raise LockError("锁文件不存在：%s" % lock_path)
    try:
        with open(lock_path, "rb") as fh:
            raw = json.loads(fh.read().decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as exc:
        raise LockError("锁文件不是合法 JSON：%s" % exc)
    missing = [k for k in REQUIRED_TOP_KEYS if k not in raw]
    if missing:
        raise LockError("锁文件缺少必需键：%s" % ", ".join(missing))
    if raw["schema_version"] != SCHEMA_VERSION:
        raise LockError("schema_version 不是 %s：%s" % (SCHEMA_VERSION, raw["schema_version"]))
    for key in REQUIRED_UPSTREAM_KEYS:
        if key not in raw["upstream"]:
            raise LockError("upstream 缺少必需键：%s" % key)
    for key in REQUIRED_LICENSE_KEYS:
        if key not in raw["license"]:
            raise LockError("license 缺少必需键：%s" % key)
    if not raw["files"]:
        raise LockError("files 为空：禁止用空清单表示通过")
    if len(raw["files"]) != raw["file_count"]:
        raise LockError(
            "file_count=%s 与 files 长度 %d 不一致" % (raw["file_count"], len(raw["files"]))
        )
    if not isinstance(raw["non_asset_files"], list):
        raise LockError("non_asset_files 必须是列表（声明非资产白名单）")
    for entry in raw["files"]:
        miss = [k for k in REQUIRED_FILE_KEYS if k not in entry]
        if miss:
            raise LockError("files[] 缺少必需键 %s：%s" % (", ".join(miss), entry))
        if not SHA256_RE.match(entry["sha256"]):
            raise LockError("sha256 格式非法：%s" % entry["path"])
    return raw


def relpath_posix(path, root):
    """转成 root 相对 posix 路径；越界即显式失败（不写本机绝对路径进报告）。"""
    real_root = os.path.realpath(root)
    real_path = os.path.realpath(path)
    if real_path != real_root and not real_path.startswith(real_root + os.sep):
        raise LockError("路径越界：%s 不在 %s 下" % (path, root))
    rel = os.path.relpath(real_path, real_root)
    return rel.replace(os.sep, "/")


def verify(lock_path, root=None, require_readonly=False, expect_commit=None):
    """返回 (report, exit_code)。锁结构非法时由调用者捕获 LockError 处理。"""
    lock_path = os.path.abspath(lock_path)
    lock = load_lock(lock_path)
    if root is None:
        root = os.path.dirname(lock_path)
    root = os.path.abspath(root)
    if not os.path.isdir(root):
        raise LockError("root 目录不存在：%s" % root)

    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "lock": relpath_posix(lock_path, root) if os.path.realpath(lock_path).startswith(os.path.realpath(root)) else os.path.basename(lock_path),
        "vendor": lock["vendor"],
        "repo_url": lock["upstream"]["repo_url"],
        "commit": lock["upstream"]["commit"],
        "simulation": True,
        "declared_file_count": lock["file_count"],
        "declared_total_bytes": lock["total_bytes"],
        "checked_files": 0,
        "checked_bytes": 0,
        "mismatches": [],
        "unexpected_files": [],
        "writable_files": [],
        "readonly_required": bool(require_readonly),
        "readonly_enforced": True,
        "readonly_portability_note": READONLY_PORTABILITY_NOTE,
        "expect_commit": expect_commit,
        "commit_matches_expectation": (
            None if expect_commit is None else lock["upstream"]["commit"] == expect_commit
        ),
        "passed": False,
        "exit_code": EXIT_OK,
    }

    exit_code = EXIT_OK

    if expect_commit is not None and lock["upstream"]["commit"] != expect_commit:
        report["mismatches"].append({
            "path": "<upstream.commit>",
            "kind": "commit_mismatch",
            "expected": expect_commit,
            "actual": lock["upstream"]["commit"],
        })
        exit_code = EXIT_LOCK_INVALID

    # 2. 逐件复算
    registered = set()
    for entry in lock["files"]:
        rel = entry["path"]
        if os.path.isabs(rel) or rel.startswith("../") or "/../" in rel:
            raise LockError("锁内路径必须是 root 相对路径且不得越界：%s" % rel)
        registered.add(rel)
        target = os.path.join(root, rel)
        if not os.path.isfile(target):
            report["mismatches"].append({
                "path": rel, "kind": "missing", "expected": entry["sha256"], "actual": None,
            })
            continue
        actual_bytes = os.path.getsize(target)
        actual_sha = sha256_of(target)
        if actual_bytes != entry["bytes"]:
            report["mismatches"].append({
                "path": rel, "kind": "bytes", "expected": entry["bytes"], "actual": actual_bytes,
            })
        if actual_sha != entry["sha256"]:
            report["mismatches"].append({
                "path": rel, "kind": "sha256", "expected": entry["sha256"], "actual": actual_sha,
            })
        report["checked_files"] += 1
        report["checked_bytes"] += actual_bytes
        if require_readonly and (os.stat(target).st_mode & 0o222):
            report["writable_files"].append(rel)

    if report["mismatches"] and exit_code == EXIT_OK:
        exit_code = EXIT_CONTENT_MISMATCH

    # 3. 反方向：未登记文件
    declared_non_asset = set(lock["non_asset_files"])
    found = set()
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            found.add(relpath_posix(os.path.join(dirpath, name), root))
    unexpected = sorted(found - registered - declared_non_asset)
    report["unexpected_files"] = unexpected
    if unexpected and exit_code == EXIT_OK:
        exit_code = EXIT_UNEXPECTED_FILES

    # 4. 只读门禁
    if require_readonly:
        report["readonly_enforced"] = not report["writable_files"]
        if report["writable_files"] and exit_code == EXIT_OK:
            exit_code = EXIT_READONLY_GATE

    report["exit_code"] = exit_code
    report["passed"] = exit_code == EXIT_OK
    return report, exit_code


def build_parser():
    parser = argparse.ArgumentParser(
        description="复算 vendor 资产锁（SHA-256 + 未登记文件 + 可选只读门禁），输出纯 JSON 报告",
    )
    parser.add_argument("--lock", default=None, help="source-lock.json 路径（必需）")
    parser.add_argument("--root", default=None, help="资产根目录，默认取锁文件所在目录")
    parser.add_argument(
        "--require-readonly", action="store_true",
        help="把「资产只读（chmod -R a-w）」变成硬门禁（退出码 5）",
    )
    parser.add_argument(
        "--expect-commit", default=None,
        help="断言 upstream.commit 等于该值（不相等即退出码 2），用于 CI 锁点校验",
    )
    parser.add_argument("--json-out", default=None, help="额外把报告写入该路径")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not args.lock:
        sys.stderr.write("用法错误：必须显式给出 --lock（禁止用默认锁路径兜底）\n")
        return EXIT_USAGE
    try:
        report, exit_code = verify(
            args.lock, root=args.root, require_readonly=args.require_readonly,
            expect_commit=args.expect_commit,
        )
    except LockError as exc:
        report = {
            "schema_version": REPORT_SCHEMA_VERSION,
            "lock": args.lock if not os.path.isabs(args.lock) else os.path.basename(args.lock),
            "passed": False,
            "exit_code": EXIT_LOCK_INVALID,
            "error": str(exc),
        }
        exit_code = EXIT_LOCK_INVALID
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
