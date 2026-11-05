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
import http.cookiejar
import json
import os
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lib_agentos_mcp as lib  # noqa: E402

VERSION_DIR = Path(os.environ.get("IRAF_VERSION_DIR") or "/opt/iraf-sdk/current")
DEFAULT_REGISTRY = Path("/etc/iraf/mcp-tokens.json")
DEFAULT_TOKEN_FILE = Path("/etc/iraf/mcp-agentos.token")
DEFAULT_HOST_TOKEN_FILE = Path("/etc/iraf/mcp-host.token")
DEFAULT_ADMIN_FILE = Path("/etc/iraf/agentos-admin")
DEFAULT_DASHBOARD = "http://127.0.0.1:9119"
PYTHON_IN_CONTAINER = "/usr/local/bin/python3.11"


def host_config_path(explicit=""):
    if explicit:
        return Path(explicit)
    home = os.environ.get("INTEWELL_AGENT_HOME") or "/root/.intewell"
    return Path(home) / "config.yaml"


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


def read_admin(path: Path):
    """读 dashboard admin 凭据：支持 `username=.../password=...` 或 `user:pass`。"""
    if not path.is_file():
        return None
    user = password = ""
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" in line:
            key, _, value = line.partition("=")
            if key.strip().lower() in ("username", "user", "admin"):
                user = value.strip()
            elif key.strip().lower() == "password":
                password = value.strip()
        elif ":" in line and not user and not password:
            user, _, password = line.partition(":")
    return (user, password) if user and password else None


def dashboard_api(base, admin_file: Path):
    """登录 dashboard 并返回 opener；无凭据/失败返回 (None, 原因)。"""
    creds = read_admin(admin_file)
    if not creds:
        return None, "缺少 %s（username/password）" % admin_file
    jar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))
    body = json.dumps({"provider": "basic", "username": creds[0],
                       "password": creds[1], "next": "/"}).encode()
    req = urllib.request.Request(base.rstrip("/") + "/auth/password-login",
                                 data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with opener.open(req, timeout=10) as resp:
            if resp.status != 200:
                return None, "登录失败 HTTP %s" % resp.status
    except Exception as exc:  # noqa: BLE001
        return None, "登录失败：%s" % str(exc)[:200]
    return opener, ""


def api_json(opener, base, method, path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base.rstrip("/") + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with opener.open(req, timeout=timeout) as resp:
        raw = resp.read().decode()
        return resp.status, (json.loads(raw) if raw else None)


def bind_host_api(base, admin_file, *, name, url, token, revert=False):
    opener, reason = dashboard_api(base, admin_file)
    if opener is None:
        return False, reason
    try:
        if revert:
            status, _ = api_json(opener, base, "DELETE", "/api/mcp/servers/" + name)
            return status in (200, 204, 404), "HTTP %s" % status
        payload = {"name": name, "url": url, "auth": "header", "bearer_token": token}
        status, body = api_json(opener, base, "POST", "/api/mcp/servers", payload)
        if status == 400 and "already" in json.dumps(body or {}).lower():
            status, body = api_json(opener, base, "PUT", "/api/mcp/servers", {"servers": {name: payload}})
        if status not in (200, 201):
            return False, "登记失败 HTTP %s %s" % (status, str(body)[:200])
        status, tested = api_json(opener, base, "POST",
                                  "/api/mcp/servers/%s/test" % name, {}, timeout=60)
        tools = (tested or {}).get("tools") or []
        if status == 200 and (tested or {}).get("ok"):
            return True, "tools=%d" % len(tools)
        return False, "test 失败 HTTP %s %s" % (status, str(tested)[:200])
    except Exception as exc:  # noqa: BLE001
        return False, "API 调用失败：%s" % str(exc)[:200]


def verify_host_api(base, admin_file, *, name):
    opener, reason = dashboard_api(base, admin_file)
    if opener is None:
        return False, reason
    try:
        status, tested = api_json(opener, base, "POST",
                                  "/api/mcp/servers/%s/test" % name, {}, timeout=60)
        tools = (tested or {}).get("tools") or []
        return bool(status == 200 and (tested or {}).get("ok")), "tools=%d" % len(tools)
    except Exception as exc:  # noqa: BLE001
        return False, "API 调用失败：%s" % str(exc)[:200]


def host_apply(path: Path, token: str, url: str, name: str, *,
               dashboard: str, admin_file: Path, revert=False):
    """优先走 dashboard API；无凭据时降级为 YAML 合并（并明确告警）。"""
    if admin_file.is_file():
        ok, detail = bind_host_api(dashboard, admin_file, name=name, url=url,
                                   token=token, revert=revert)
        if ok:
            print("[agentos-mcp] 宿主 dashboard：%s（%s）" % (
                "已解除" if revert else "已登记", detail))
            return True
        print("[agentos-mcp][警告] dashboard API 失败：%s；降级为写 config.yaml" % detail)
    data = lib.load_config(path)
    if revert:
        data, changed = lib.remove_server(data, name=name)
    else:
        data = lib.merge_server(data, name=name, url=url, token=token)
        changed = True
    if not changed:
        print("[agentos-mcp] 宿主 config 无需变更")
        return True
    if path.is_file():
        backup = path.with_name("config.yaml.bak.%s" % datetime.now(timezone.utc)
                                .strftime("%Y%m%dT%H%M%SZ"))
        shutil.copy2(path, backup)
        os.chmod(backup, 0o600)
        print("[agentos-mcp] 宿主配置备份：%s" % backup)
    lib.write_config(path, data, owner=(0, 0))
    print("[agentos-mcp] 已写入宿主 config：%s（需重启 dashboard/tui_gateway 才生效）" % path)
    return True


def service_health(mcp_url: str):
    base = mcp_url.rstrip("/")
    if base.endswith("/mcp"):
        base = base[:-4]
    health = base + "/healthz"
    try:
        with urllib.request.urlopen(health, timeout=5) as resp:
            health_ok = resp.status == 200
    except Exception:  # noqa: BLE001
        health_ok = False
    # 无 token 必须 401
    try:
        req = urllib.request.Request(
            mcp_url, data=b'{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}',
            method="POST", headers={"Content-Type": "application/json",
                                    "Accept": "application/json, text/event-stream"})
        urllib.request.urlopen(req, timeout=5)
        no_token = 0
    except urllib.error.HTTPError as exc:
        no_token = exc.code
    except Exception:  # noqa: BLE001
        no_token = -1
    return health_ok, no_token


def verify_all(*, url, host_path, host_token_file, worker_token_file,
               admin_file, dashboard, workers, server_name, record_path):
    results = []
    health_ok, no_token = service_health(url)
    results.append(("service", health_ok and no_token == 401,
                    "healthz=%s no_token=%s" % (health_ok, no_token)))
    # host
    if admin_file.is_file():
        ok, detail = verify_host_api(dashboard, admin_file, name=server_name)
        results.append(("host_api", ok, detail))
    else:
        data = lib.load_config(host_path) if host_path.is_file() else {}
        present = server_name in ((data.get("mcp_servers") or {}))
        results.append(("host_config", present, "no admin file; file check only"))
    # workers
    for worker in workers:
        has, detail = worker_has_mcp(worker)
        results.append(("%s.mcp" % worker, has, detail))
        res = run(["docker", "exec", "-e", "HOME=/home/agentos", worker,
                   "intewell-agent-cli", "mcp", "test", server_name], timeout=60)
        text = (res.stdout or "") + (res.stderr or "")
        ok = res.returncode == 0 and "Tools discovered:" in text
        results.append(("%s.tools" % worker, ok, text.strip().splitlines()[-1][:160] if text else ""))
    entries = []
    for item in results:
        name, ok = item[0], bool(item[1])
        detail = item[2] if len(item) > 2 else ""
        entries.append({"check": name, "ok": ok, "detail": str(detail)})
    record = {"time": datetime.now(timezone.utc).isoformat(), "url": url,
              "server": server_name, "results": entries}
    try:
        record_path.parent.mkdir(parents=True, exist_ok=True)
        record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
        os.chmod(record_path, 0o600)
    except OSError as exc:
        print("[agentos-mcp][警告] 写验证记录失败：%s" % exc)
    print("[agentos-mcp] 验证记录：%s" % record_path)
    for item in record["results"]:
        print("  %-24s %s %s" % (item["check"], "OK" if item["ok"] else "FAIL",
                                 item["detail"]))
    return all(item["ok"] for item in record["results"])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--status", action="store_true", help="只读状态（默认）")
    parser.add_argument("--apply", action="store_true", help="写入并重启/热加载")
    parser.add_argument("--revert", action="store_true", help="删除 iraf-task 绑定")
    parser.add_argument("--verify", action="store_true", help="只读验证（服务/host/worker）")
    parser.add_argument("--url", default="", help="MCP URL，例如 http://<board>:8765/mcp")
    parser.add_argument("--server-name", default=lib.DEFAULT_SERVER_NAME)
    parser.add_argument("--token-file", default=str(DEFAULT_TOKEN_FILE))
    parser.add_argument("--host-token-file", default=str(DEFAULT_HOST_TOKEN_FILE))
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--shared-dir", default="", help="覆盖自动发现的 worker 共享目录")
    parser.add_argument("--host-config", default="", help="覆盖宿主 config.yaml")
    parser.add_argument("--dashboard-url", default=DEFAULT_DASHBOARD)
    parser.add_argument("--admin-file", default=str(DEFAULT_ADMIN_FILE))
    parser.add_argument("--workers", default="", help="逗号分隔的 worker 名；默认自动发现")
    parser.add_argument("--no-workers", action="store_true", help="不处理 worker")
    parser.add_argument("--no-host", action="store_true", help="不处理宿主 dashboard")
    parser.add_argument("--no-restart", action="store_true")
    parser.add_argument("--strict", action="store_true",
                        help="worker 缺 mcp 包时返回非 0（默认只警告）")
    args = parser.parse_args(argv)

    token_file = Path(args.token_file)
    host_token_file = Path(args.host_token_file)
    host_path = host_config_path(args.host_config)
    admin_file = Path(args.admin_file)
    shared_dir = Path(args.shared_dir) if args.shared_dir else None
    workers = ([] if args.no_workers else
               ([w.strip() for w in args.workers.split(",") if w.strip()]
                if args.workers else discover_workers()))
    if not shared_dir and workers:
        shared_dir = Path(discover_shared_dir(workers)) if discover_shared_dir(workers) else None
    do_host = not args.no_host
    do_workers = not args.no_workers

    def host_present():
        data = lib.load_config(host_path) if host_path.is_file() else {}
        return args.server_name in ((data.get("mcp_servers") or {}))

    def detect_url():
        if args.url:
            return args.url
        if host_path.is_file():
            servers = (lib.load_config(host_path).get("mcp_servers") or {})
            entry = servers.get(args.server_name) or {}
            if entry.get("url"):
                return str(entry["url"])
        if shared_dir:
            servers = (lib.load_config(config_path(shared_dir)).get("mcp_servers") or {})
            entry = servers.get(args.server_name) or {}
            if entry.get("url"):
                return str(entry["url"])
        return ""

    if args.status or not (args.apply or args.revert or args.verify):
        show_status(workers, shared_dir, token_file, args.server_name)
        if do_host:
            print("[agentos-mcp] host_config=%s (%s) server_present=%s token=%s (%s)" % (
                host_path, "present" if host_path.is_file() else "missing",
                host_present(), host_token_file,
                "present" if host_token_file.is_file() else "missing"))
            print("[agentos-mcp] dashboard_api=%s admin_file=%s" % (
                args.dashboard_url,
                "present" if admin_file.is_file() else "missing"))
        return 0

    if args.revert:
        if do_host:
            host_apply(host_path, "", "", args.server_name,
                       dashboard=args.dashboard_url, admin_file=admin_file, revert=True)
        if do_workers and workers and shared_dir and shared_dir.is_dir():
            changed = write_server(shared_dir, "", args.url, args.server_name, revert=True)
            if changed and not args.no_restart:
                restart_workers(workers)
        return 0

    if args.apply:
        url = detect_url()
        if not url:
            print("[agentos-mcp][错误] --apply 需要 --url（例如 http://10.203.247.147:8765/mcp）")
            return 2
        if do_host:
            issue_token(Path(args.registry), host_token_file, subject="agentos-host")
            host_token = read_token(host_token_file)
            if not host_token:
                print("[agentos-mcp][错误] 宿主 token 为空：%s" % host_token_file)
                return 3
            host_apply(host_path, host_token, url, args.server_name,
                       dashboard=args.dashboard_url, admin_file=admin_file)
        if do_workers:
            if not workers:
                print("[agentos-mcp] 未发现 agentos-worker-* 容器：跳过 worker 绑定")
            elif not shared_dir or not shared_dir.is_dir():
                print("[agentos-mcp][错误] 未找到 worker 共享 home，跳过 worker 绑定")
            else:
                missing = [w for w in workers if not worker_has_mcp(w)[0]]
                if missing:
                    message = ("worker 缺 mcp 客户端包：%s（镜像需内置 mcp>=1.26,<2，"
                               "或用离线 wheelhouse 安装）" % ", ".join(missing))
                    print("[agentos-mcp][错误] " + message)
                    if args.strict:
                        return 3
                issue_token(Path(args.registry), token_file, subject="agentos-worker")
                token = read_token(token_file)
                if not token:
                    print("[agentos-mcp][错误] token 为空：%s" % token_file)
                    return 3
                write_server(shared_dir, token, url, args.server_name)
                if not args.no_restart:
                    restart_workers(workers)
        print("[agentos-mcp] 绑定完成")

    if args.verify:
        url = detect_url()
        if not url:
            print("[agentos-mcp][错误] --verify 需要 --url 或已有配置")
            return 2
        ok = verify_all(url=url, host_path=host_path, host_token_file=host_token_file,
                        worker_token_file=token_file, admin_file=admin_file,
                        dashboard=args.dashboard_url, workers=workers,
                        server_name=args.server_name,
                        record_path=Path("/etc/iraf/agentos-mcp-binding.json"))
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
