#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 IRAF 的 MCP 绑定进 AgentOS worker 的共享配置（在板上宿主机执行）。

背景：chat 的 agent 跑在 agentos-worker-* 容器里，容器 home 是
/var/lib/agentos-shared/.intewell 的 bind mount。只配宿主 dashboard 不够；
必须把 mcp_servers.iraf-task 合并进这个共享目录的 config.yaml，并重启 worker。

用法（板上 root）：
  python3 agentos_mcp_bind.py --status
  python3 agentos_mcp_bind.py --apply --url http://10.203.247.147:8765/mcp
  python3 agentos_mcp_bind.py --revert

安全：
  · 默认 --status（只读）；--apply 才写；
  · 写前备份 config.yaml.bak.<时间戳>；原子写；保留其它 mcp_servers；
  · token 从 600 文件读，不打印；缺失时用 iraf_tools.mcp_tokens 签发；
  · 不 import AgentOS，不写容器可写层。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lib_agentos_mcp as lib  # noqa: E402

VERSION_DIR = Path(os.environ.get("IRAF_VERSION_DIR") or "/opt/iraf-sdk/current")
DEFAULT_REGISTRY = Path("/etc/iraf/mcp-tokens.json")
DEFAULT_TOKEN_FILE = Path("/etc/iraf/mcp-agentos.token")
PYTHON_IN_CONTAINER = "/usr/local/bin/python3.11"


def run(cmd, *, timeout=30, check=False):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                          check=check)


def discover_workers(prefix=lib.WORKER_PREFIX):
    result = run(["docker", "ps", "--format", "{{.Names}}"])
    if result.returncode != 0:
        return []
    return sorted(name for name in result.stdout.splitlines()
                  if name.strip().startswith(prefix))


def inspect_worker(name):
    result = run(["docker", "inspect", name])
    if result.returncode != 0:
        return {}
    try:
        payload = json.loads(result.stdout)
    except ValueError:
        return {}
    return payload[0] if payload else {}


def discover_shared_dir(workers):
    for worker in workers:
        path = lib.discover_shared_dir(inspect_worker(worker))
        if path:
            return path
    return ""


def worker_has_mcp(worker):
    result = run(["docker", "exec", worker, PYTHON_IN_CONTAINER, "-c", "import mcp"],
                 timeout=20)
    return result.returncode == 0, (result.stderr or result.stdout).strip()[:300]


def issue_token(registry: Path, token_file: Path, subject: str):
    if token_file.is_file() and token_file.stat().st_size > 0:
        return
    if not registry.is_file():
        raise SystemExit("[agentos-mcp][错误] token 注册表不存在：%s" % registry)
    env = dict(os.environ)
    env["PYTHONPATH"] = "%s/lib:%s/src" % (VERSION_DIR, VERSION_DIR)
    result = subprocess.run(
        [sys.executable, "-m", "iraf_tools.mcp_tokens", "--file", str(registry), "issue",
         "--subject", subject, "--tenant", "default",
         "--scopes", "task.submit,task.read", "--resources", "*", "--expires-days", "90"],
        capture_output=True, text=True, timeout=60, env=env)
    if result.returncode != 0:
        raise SystemExit("[agentos-mcp][错误] 签发 token 失败：%s"
                         % (result.stderr or result.stdout).strip()[:300])
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise SystemExit("[agentos-mcp][错误] 签发 token 没有输出")
    token_file.parent.mkdir(parents=True, exist_ok=True)
    token_file.write_text(lines[-1].strip() + "\n", encoding="utf-8")
    os.chmod(token_file, 0o600)


def read_token(token_file: Path) -> str:
    if not token_file.is_file():
        return ""
    return token_file.read_text(encoding="utf-8").strip()


def config_path(shared_dir: Path) -> Path:
    return shared_dir / "config.yaml"


def shared_owner(shared_dir: Path):
    info = shared_dir.stat()
    return (info.st_uid, info.st_gid)


def show_status(workers, shared_dir, token_file, server_name):
    print("[agentos-mcp] workers=%s" % (workers or "none"))
    print("[agentos-mcp] shared_dir=%s" % (shared_dir or "not-found"))
    print("[agentos-mcp] token_file=%s (%s)" % (
        token_file, "present" if token_file.is_file() else "missing"))
    if shared_dir:
        path = config_path(shared_dir)
        print("[agentos-mcp] config=%s (%s)" % (
            path, "present" if path.is_file() else "missing"))
        if path.is_file():
            try:
                data = lib.load_config(path)
                servers = (data.get("mcp_servers") or {})
                print("[agentos-mcp] mcp_servers=%s" % sorted(servers))
            except Exception as exc:  # noqa: BLE001
                print("[agentos-mcp] config 解析失败：%s" % exc)
    for worker in workers:
        ok, detail = worker_has_mcp(worker)
        print("[agentos-mcp] %s mcp=%s %s" % (worker, "ok" if ok else "MISSING", detail))


def write_server(shared_dir: Path, token: str, url: str, server_name: str, *, revert=False):
    path = config_path(shared_dir)
    data = lib.load_config(path)
    if revert:
        data, changed = lib.remove_server(data, name=server_name)
    else:
        data = lib.merge_server(data, name=server_name, url=url, token=token)
        changed = True
    if not changed:
        print("[agentos-mcp] 无需变更（%s 不存在）" % server_name)
        return False
    if path.is_file():
        backup = path.with_name("config.yaml.bak.%s" % datetime.now(timezone.utc)
                                .strftime("%Y%m%dT%H%M%SZ"))
        shutil.copy2(path, backup)
        os.chmod(backup, 0o600)
        print("[agentos-mcp] 备份：%s" % backup)
    lib.write_config(path, data, owner=shared_owner(shared_dir))
    print("[agentos-mcp] 已写入：%s（server=%s）" % (path, server_name))
    return True


def restart_workers(workers):
    if not workers:
        return
    result = run(["docker", "restart", *workers], timeout=120)
    if result.returncode != 0:
        print("[agentos-mcp][警告] 重启 worker 失败：%s" % (result.stderr or result.stdout)[:300])
    else:
        print("[agentos-mcp] 已重启：%s" % ", ".join(workers))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--status", action="store_true", help="只读状态（默认）")
    parser.add_argument("--apply", action="store_true", help="写入并重启 worker")
    parser.add_argument("--revert", action="store_true", help="只删除 iraf-task 段")
    parser.add_argument("--url", default="", help="MCP URL，例如 http://<board>:8765/mcp")
    parser.add_argument("--server-name", default=lib.DEFAULT_SERVER_NAME)
    parser.add_argument("--token-file", default=str(DEFAULT_TOKEN_FILE))
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--shared-dir", default="", help="覆盖自动发现的共享目录")
    parser.add_argument("--workers", default="", help="逗号分隔的 worker 名；默认自动发现")
    parser.add_argument("--no-restart", action="store_true")
    parser.add_argument("--strict", action="store_true",
                        help="worker 缺 mcp 包时返回非 0（默认只警告）")
    args = parser.parse_args(argv)

    token_file = Path(args.token_file)
    shared_dir = Path(args.shared_dir) if args.shared_dir else None
    workers = ([w.strip() for w in args.workers.split(",") if w.strip()]
               if args.workers else discover_workers())
    if not shared_dir:
        shared_dir = Path(discover_shared_dir(workers)) if workers else None

    if args.status or not (args.apply or args.revert):
        show_status(workers, shared_dir, token_file, args.server_name)
        return 0

    if not workers:
        print("[agentos-mcp] 未发现 agentos-worker-* 容器：跳过（AgentOS 不在本机？）")
        return 0
    if not shared_dir or not shared_dir.is_dir():
        print("[agentos-mcp][错误] 未找到 worker 的共享 home（/home/agentos/.intewell 的 bind 源）")
        return 4

    missing = [w for w in workers if not worker_has_mcp(w)[0]]
    if missing:
        message = ("worker 缺 mcp 客户端包：%s（镜像需内置 mcp>=1.26,<2，"
                   "或用离线 wheelhouse 安装；否则 chat 无法发现 MCP）" % ", ".join(missing))
        print("[agentos-mcp][错误] " + message)
        if args.strict:
            return 3

    if args.revert:
        changed = write_server(shared_dir, "", args.url, args.server_name, revert=True)
        if changed and not args.no_restart:
            restart_workers(workers)
        return 0

    if not args.url:
        print("[agentos-mcp][错误] --apply 需要 --url（例如 http://10.203.247.147:8765/mcp）")
        return 2
    issue_token(Path(args.registry), token_file, subject="agentos-worker")
    token = read_token(token_file)
    if not token:
        print("[agentos-mcp][错误] token 为空：%s" % token_file)
        return 3
    write_server(shared_dir, token, args.url, args.server_name)
    if not args.no_restart:
        restart_workers(workers)
    print("[agentos-mcp] 完成；chat 下一轮应能看到 iraf_* 工具")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
