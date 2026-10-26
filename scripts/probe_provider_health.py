# -*- coding: utf-8 -*-
"""只读探针：跨机 Provider 的 Health（可达性 / 能力 / 显示事实 / 清单）。

用法：
  PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python \
      scripts/probe_provider_health.py --endpoint 10.203.247.121:50052
退出码：0 可达且 ready / 1 不可达 / 2 缺 token
"""
from __future__ import annotations

import argparse
from pathlib import Path

from iraf_adapters.remote.provider_backend import RemoteProviderBackend, read_token_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="10.203.247.121:50052")
    parser.add_argument("--token-file", default=str(Path.home() / ".iraf/provider.token"))
    args = parser.parse_args()

    token = read_token_file(args.token_file)
    if not token:
        print("[health][错误] 缺 token：%s" % args.token_file)
        return 2
    backend = RemoteProviderBackend(args.endpoint, token, timeout_s=10.0, subject="probe-health")
    try:
        health = backend.health()
    except Exception as exc:  # noqa: BLE001 探针只报告，不掩盖
        print("[health] reachable=false endpoint=%s 原因=%s" % (args.endpoint, exc))
        return 1
    inv = health.get("inventory") or {}
    print("[health] reachable=true ready=%s simulation=%s backend=%s"
          % (health.get("ready"), health.get("simulation"), health.get("backend")))
    print("[health] display_mode=%s window_opened=%s detail=%s"
          % (health.get("display_mode"), health.get("window_opened"), health.get("detail")))
    print("[health] capabilities=%d %s"
          % (len(health.get("capabilities") or []), ",".join(health.get("capabilities") or [])))
    print("[health] inventory 顶层键=%s mode=%s safety=%s"
          % (sorted(inv.keys()), inv.get("mode"), inv.get("safety")))
    return 0 if health.get("ready") else 1


if __name__ == "__main__":
    raise SystemExit(main())
