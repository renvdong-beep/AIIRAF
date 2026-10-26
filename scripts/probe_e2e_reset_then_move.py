# -*- coding: utf-8 -*-
"""跨进程探针：真实 gRPC Provider 上「reset_scene → 立刻 move_joint」是否还被判 BUSY。

与 scripts/probe_provider_lease_release.py 的分工：
  - 那个在**进程内**走 servicer.Invoke（不需要端口/MuJoCo），用于回归；
  - 这个走**真实 gRPC + 真实 MuJoCo 后端**，证明跨机链路上一体化成立。

用法（先起 scripts/probe_e2e_provider_up.sh）：
  PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python \
      scripts/probe_e2e_reset_then_move.py --endpoint 127.0.0.1:50099
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

from iraf_adapters.remote.provider_backend import RemoteProviderBackend, read_token_file

RESOURCE = "robot-arm"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="127.0.0.1:50099")
    parser.add_argument("--token-file", default=str(Path.home() / ".iraf/provider.token"))
    parser.add_argument("--joint", default="joint1")
    parser.add_argument("--target", type=float, default=0.05)
    parser.add_argument("--duration-ms", type=int, default=600)
    args = parser.parse_args()

    token = read_token_file(args.token_file)
    if not token:
        print("[探针][错误] 缺 token：%s" % args.token_file)
        return 2
    backend = RemoteProviderBackend(args.endpoint, token, timeout_s=180.0,
                                    resource_id=RESOURCE, subject="probe-e2e")

    # 用 _invoke 直接拿 outputs（含服务端生成的 provider_execution_id），
    # 公开方法 move_joint 会把返回值吞成 None，看不出"服务端真的登记了一次执行"。
    t0 = time.time()
    reset = backend._invoke("reset_scene", {"scope": "all", "target_id": None}, None)
    t1 = time.time()
    print("[1] reset_scene  accepted wall=%.2fs exec=%s status=%s baseline=%s"
          % (t1 - t0, reset.get("provider_execution_id"), reset.get("status"),
             reset.get("baseline")))

    # 关键：**紧接着**下发第二个能力调用（原缺陷在这里被判 IRAF-RESOURCE-BUSY）
    moved = backend._invoke("move_joint",
                            {"positions": {args.joint: args.target},
                             "duration_ms": args.duration_ms}, None)
    t2 = time.time()
    print("[2] move_joint   accepted wall=%.2fs exec=%s"
          % (t2 - t1, moved.get("provider_execution_id")))

    # 再发一次，证明资源每次都被放开（否则第三次会被占住）
    again = backend._invoke("move_joint",
                            {"positions": {args.joint: 0.0},
                             "duration_ms": args.duration_ms}, None)
    t3 = time.time()
    print("[3] move_joint   accepted wall=%.2fs exec=%s（回原位）"
          % (t3 - t2, again.get("provider_execution_id")))

    health = backend.health()
    inv = health.get("inventory") or {}
    print("[4] health       ready=%s capabilities=%d inventory 顶层键=%s"
          % (health.get("ready"), len(health.get("capabilities") or []), sorted(inv.keys())))
    # 收尾复位：探针动过关节，必须把世界还原成干净初始态（演示/复跑口径）
    tail = backend._invoke("reset_scene", {"scope": "all", "target_id": None}, None)
    print("[5] reset_scene  收尾复位 exec=%s baseline=%s"
          % (tail.get("provider_execution_id"), tail.get("baseline")))
    ids = {reset.get("provider_execution_id"), moved.get("provider_execution_id"),
           again.get("provider_execution_id")}
    print("结论: 复位→立刻动 一步到位 = %s（3 次执行互不阻塞，execution 各不相同=%s，总墙钟 %.2fs）"
          % ("是" if len(ids) == 3 else "否", len(ids) == 3, t3 - t0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
