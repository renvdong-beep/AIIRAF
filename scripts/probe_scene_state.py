# -*- coding: utf-8 -*-
"""探针：看一眼**场景脏不脏**（只读），并按需复位（显式 --reset）。

为什么需要：任务失败时最常见的真因不是模型/参数，而是**世界状态被上一次执行改过**
（目标已被搬走、夹爪还夹着东西、臂停在半路）。判据只有一个 —— 被控对象自己报的
`inventory.manipulation`（声明目标位姿 vs 实测目标位姿），不靠猜。

用法：
  PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python \
      scripts/probe_scene_state.py [--endpoint 10.203.247.121:50052] [--reset]
退出码：0 读到状态 / 1 不可达 / 2 缺 token
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from iraf_adapters.remote.provider_backend import RemoteProviderBackend, read_token_file


def _show(tag: str, inv: dict) -> None:
    manipulation = dict(inv.get("manipulation") or {})
    print("[%s] mode=%s safety=%s" % (tag, inv.get("mode"), inv.get("safety")))
    if not manipulation:
        print("      manipulation=<无>（被控对象未报操作段）")
        return
    print("      manipulation=%s" % json.dumps(manipulation, ensure_ascii=False, sort_keys=True)[:900])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="10.203.247.121:50052")
    parser.add_argument("--token-file", default=str(Path.home() / ".iraf/provider.token"))
    parser.add_argument("--reset", action="store_true", help="读完后复位场景（会改变世界状态，需明确给）")
    args = parser.parse_args()

    token = read_token_file(args.token_file)
    if not token:
        print("[scene][错误] 缺 token：%s" % args.token_file)
        return 2
    backend = RemoteProviderBackend(args.endpoint, token, timeout_s=30.0, subject="probe-scene")
    try:
        before = (backend.health().get("inventory") or {})
    except Exception as exc:  # noqa: BLE001 探针只报告
        print("[scene] 不可达：%s" % exc)
        return 1
    _show("复位前", before)

    if not args.reset:
        print("（只读；要复位请加 --reset）")
        return 0

    out = backend._invoke("reset_scene", {"scope": "all", "target_id": None}, None)
    print("[reset] exec=%s status=%s baseline=%s detail=%s"
          % (out.get("provider_execution_id"), out.get("status"), out.get("baseline"), out.get("detail")))
    after = (backend.health().get("inventory") or {})
    _show("复位后", after)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
