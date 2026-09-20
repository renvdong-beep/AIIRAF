#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 vendor 资产锁重新拉取厂商资产（步骤 12）。

存在理由：仓库 `.gitignore` 明确「厂商机器人模型：第三方代码，不入库」，改用
`vendor/*/source-lock.json` 锁定来源与哈希、「需要时按锁文件重新拉取」。因此锁文件必须配一个
**可复跑的入口**，否则"按锁文件重新拉取"只是注释里的一句话（AGENTS.md 5.2）。

契约：
  - 路径、字节数、SHA-256、上游 URL、commit **全部来自锁文件**；脚本内不硬编码任何路径或版本；
  - 拉取后做两级校验：`sha1("blob <len>\\0" + content) == 锁内 blob_sha1`（与上游 git 对象逐位一致）
    且 `sha256(content) == 锁内 sha256`；任一不符即失败，**不写盘**；
  - 默认 `--dry-run`：只打印计划（哪些文件缺失/不符、要访问的 URL），不联网、不写盘；
  - `--apply` 才真正下载；下载后按锁内 `read_only` 语义（默认 asset）设为只读；
  - `--require-complete`：把「资产是否齐备」变成硬门禁（退出码 3），用于 CI / 新克隆。

退出码：0=成功（dry-run 表示计划已产出）；1=用法错误；2=锁非法/缺失；3=资产不齐备或校验不符；
        4=下载失败（网络/上游不可达，如实报告，不伪造成功）。

运行：
    PYTHONPATH=src /usr/bin/python3 scripts/fetch_vendor_assets.py --lock vendor/unitree_go2/source-lock.json --dry-run
    PYTHONPATH=src /usr/bin/python3 scripts/fetch_vendor_assets.py --lock vendor/unitree_go2/source-lock.json --apply
"""

import argparse
import hashlib
import http.client
import json
import os
import stat
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import quote

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from verify_vendor_lock import (  # noqa: E402
    EXIT_CONTENT_MISMATCH,
    EXIT_LOCK_INVALID,
    EXIT_OK,
    EXIT_USAGE,
    LockError,
    load_lock,
    sha256_of,
    verify,
)

EXIT_DOWNLOAD_FAILED = 4

API_CONTENT_TEMPLATE = "https://api.github.com/repos/{repo}/contents/{path}?ref={commit}"
API_TIMEOUT_S = 120
MAX_TRIES = 3


def git_blob_sha1(data):
    digest = hashlib.sha1()
    digest.update(b"blob %d\0" % len(data))
    digest.update(data)
    return digest.hexdigest()


def repo_slug(repo_url):
    """`https://github.com/<owner>/<repo>` → `<owner>/<repo>`；非 github URL 显式失败。"""
    prefix = "https://github.com/"
    if not repo_url.startswith(prefix):
        raise LockError("当前只支持 github.com 上游，锁内 repo_url=%s" % repo_url)
    return repo_url[len(prefix):].rstrip("/")


def content_url(repo_url, commit, path):
    return API_CONTENT_TEMPLATE.format(
        repo=repo_slug(repo_url), path=quote(path, safe="/"), commit=commit
    )


def download(url, timeout=API_TIMEOUT_S):
    """只读下载一个 blob（raw 媒体类型），失败返回 None 并打日志。"""
    last = None
    for attempt in range(1, MAX_TRIES + 1):
        try:
            req = urllib.request.Request(url, headers={
                "Accept": "application/vnd.github.raw",
                "User-Agent": "iraf-vendor-lock/1.0",
            })
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                data = resp.read()
            if data:
                return data
            last = "空内容"
        except (urllib.error.URLError, OSError, http.client.HTTPException) as exc:
            last = "%s: %s" % (type(exc).__name__, exc)
        print("  下载重试 %d/%d：%s（%s）" % (attempt, MAX_TRIES, url, last), file=sys.stderr, flush=True)
        time.sleep(2)
    print("  下载失败：%s（%s）" % (url, last), file=sys.stderr, flush=True)
    return None


def plan(lock, dest):
    """逐件判断现状（present / missing / mismatch），并给出应访问的 URL。"""
    repo_url = lock["upstream"]["repo_url"]
    commit = lock["upstream"]["commit"]
    items = []
    for entry in lock["files"]:
        rel = entry["path"]
        target = os.path.join(dest, rel)
        url = content_url(repo_url, commit, rel)
        item = {
            "path": rel, "url": url, "declared_bytes": entry["bytes"],
            "declared_sha256": entry["sha256"], "declared_blob_sha1": entry.get("blob_sha1"),
        }
        if not os.path.isfile(target):
            item["state"] = "missing"
            item["actual_sha256"] = None
        else:
            actual = sha256_of(target)
            item["actual_sha256"] = actual
            item["state"] = "present" if actual == entry["sha256"] else "mismatch"
        items.append(item)
    return items


def apply_items(lock, dest, items, fetch_bytes=None):
    """下载 missing/mismatch 的文件；两级校验通过才写盘。返回 (written, failures)。"""
    fetch_bytes = fetch_bytes or (lambda url: download(url))
    written, failures = [], []
    for item in items:
        if item["state"] == "present":
            continue
        data = fetch_bytes(item["url"])
        if data is None:
            failures.append({"path": item["path"], "reason": "download_failed"})
            continue
        got_blob = git_blob_sha1(data)
        got_sha = hashlib.sha256(data).hexdigest()
        if item["declared_blob_sha1"] and got_blob != item["declared_blob_sha1"]:
            failures.append({
                "path": item["path"], "reason": "blob_sha1_mismatch",
                "expected": item["declared_blob_sha1"], "actual": got_blob,
            })
            continue
        if got_sha != item["declared_sha256"] or len(data) != item["declared_bytes"]:
            failures.append({
                "path": item["path"], "reason": "sha256_or_bytes_mismatch",
                "expected": item["declared_sha256"], "actual": got_sha,
            })
            continue
        target = os.path.join(dest, item["path"])
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "wb") as fh:
            fh.write(data)
        os.chmod(target, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)  # 厂商文件只读
        written.append(item["path"])
    return written, failures


def build_parser():
    parser = argparse.ArgumentParser(description="按 source-lock.json 拉取/校验 vendor 资产（输出纯 JSON 报告）")
    parser.add_argument("--lock", default=None, help="source-lock.json 路径（必需）")
    parser.add_argument("--dest", default=None, help="资产根目录，默认取锁文件所在目录")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="只产出计划，不联网不写盘（默认）")
    mode.add_argument("--apply", action="store_true", help="真正下载并校验后写盘")
    parser.add_argument("--require-complete", action="store_true",
                        help="资产不齐备即退出码 3（用于 CI / 新克隆门禁）")
    parser.add_argument("--json-out", default=None, help="额外把报告写入该路径")
    return parser


def main(argv=None, fetch_bytes=None):
    args = build_parser().parse_args(argv)
    if not args.lock:
        sys.stderr.write("用法错误：必须显式给出 --lock（禁止用默认锁路径兜底）\n")
        return EXIT_USAGE
    lock_path = os.path.abspath(args.lock)
    dest = os.path.abspath(args.dest) if args.dest else os.path.dirname(lock_path)
    mode = "apply" if args.apply else "dry_run"
    try:
        lock = load_lock(lock_path)
    except LockError as exc:
        report = {
            "schema_version": "iraf.vendor-fetch/v1", "mode": mode,
            "passed": False, "exit_code": EXIT_LOCK_INVALID, "error": str(exc),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return EXIT_LOCK_INVALID

    items = plan(lock, dest)
    written, failures = ([], [])
    if args.apply:
        written, failures = apply_items(lock, dest, items, fetch_bytes=fetch_bytes)
        items = plan(lock, dest)  # 写盘后重新观察真实状态，不沿用下载前的判断

    missing = [i["path"] for i in items if i["state"] != "present"]
    exit_code = EXIT_OK
    if failures:
        exit_code = EXIT_DOWNLOAD_FAILED
    elif args.require_complete and missing:
        exit_code = EXIT_CONTENT_MISMATCH

    report = {
        "schema_version": "iraf.vendor-fetch/v1",
        "mode": mode,
        "simulation": True,
        "vendor": lock["vendor"],
        "repo_url": lock["upstream"]["repo_url"],
        "commit": lock["upstream"]["commit"],
        "dest": os.path.basename(dest) or dest,
        "declared_file_count": lock["file_count"],
        "present_count": sum(1 for i in items if i["state"] == "present"),
        "missing_or_mismatched": missing,
        "planned_urls": {i["path"]: i["url"] for i in items if i["state"] != "present"},
        "written": written,
        "failures": failures,
        "exit_code": exit_code,
        "passed": exit_code == EXIT_OK,
    }
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())


# 供测试与调用方复用：写盘后仍以 verify_vendor_lock.verify 为唯一完整性裁决者
__all__ = ["main", "plan", "apply_items", "git_blob_sha1", "content_url", "verify"]
