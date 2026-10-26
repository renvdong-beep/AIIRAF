# -*- coding: utf-8 -*-
"""在**已安装的 Provider 进程里**执行一次抓取（动的是桌面上那个 MuJoCo 窗口，不是另起一个世界）。

为什么要这个脚本：
  `scripts/verify_pick.py` 是自建 runtime + 自建 MuJoCo（进程退出窗口就没了，桌面上看不到动作）；
  要"看得见"的抓取，必须让动作发生在**常驻 Provider**里 —— 也就是板上那条链：
  TaskFlow → Skill Runtime → PolicyGateway → RemoteProviderBackend → x86 provider → MuJoCo 窗口。

用法：
  PYTHONPATH=src:build/generated/python build/mcp-venv/bin/python \
      scripts/run_pick_remote.py --endpoint 127.0.0.1:50053 --profile profiles/ur5_mujoco.yaml
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
from iraf_adapters.remote.provider_backend import RemoteProviderBackend, read_token_file

REPO = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="127.0.0.1:50053")
    parser.add_argument("--profile", default="profiles/ur5_mujoco.yaml")
    parser.add_argument("--safety", default="profiles/safety/simulation_lab.yaml")
    parser.add_argument("--target-id", default="box_01")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--resource-id", default="ur5_mujoco-mujoco")
    parser.add_argument("--token-file", default=str(Path.home() / ".iraf/provider.token"))
    parser.add_argument("--store", default="build/acceptance/ur5-pick-via-provider/events.db")
    args = parser.parse_args()

    token = read_token_file(args.token_file)
    if not token:
        print("[pick] 缺 token：%s" % args.token_file)
        return 2

    profile = load_robot_profile(REPO / args.profile)
    safety = load_safety_policy(REPO / args.safety)
    registry = SkillRegistry().load_directory(REPO / "skills")
    authority = ControlAuthorityManager()
    backend = RemoteProviderBackend(args.endpoint, token, timeout_s=180.0,
                                   resource_id=args.resource_id, subject="board-runtime")
    store_path = REPO / args.store
    store_path.parent.mkdir(parents=True, exist_ok=True)
    runtime = SkillRuntime(profile, safety, backend, registry, authority,
                           SqliteExecutionStore(str(store_path)))

    key = "pick-%d" % int(time.time())
    request = {
        "request_id": "req-" + key,
        "correlation_id": "corr-" + key,
        "idempotency_key": key,
        "skill": "pick_object",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": args.target_id,
            # 位姿由后端按**目标体当前位姿**解析：这正是"看得见的那次抓取"该走的口径，
            # 避免把构建期标称坐标当成现场事实（docs/debug 2026-09-24 §11.27）。
            "grasp_pose": {"frame_id": "world", "pose_source": "live_target_body"},
            "duration_ms": int(args.duration_ms),
        },
        "deadline_unix_ms": int(time.time() * 1000) + 180000,
        "resource_id": args.resource_id,
        "controller": "remote-provider",
        "profile_name": profile.name, "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name, "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
    }
    owner = AuthenticatedContext("board", frozenset({"task.submit"}), "local")

    t0 = time.time()
    result = runtime.execute(request, owner)
    wall = time.time() - t0

    print("[pick] status=%s error_code=%s wall=%.2fs" % (result.get("status"), result.get("error_code") or "-", wall))
    if result.get("reason"):
        print("[pick] reason=%s" % result["reason"])
    payload = result.get("result") or {}
    keys = ("lifted", "lift_delta_m", "distance_m", "target_body", "grasp_distance_m", "qpos")
    shown = {k: payload[k] for k in keys if k in payload}
    print("[pick] result=%s" % json.dumps(shown or payload, ensure_ascii=False, default=str)[:1200])
    print("[pick] 证据：%s" % (REPO / args.store))
    return 0 if result.get("status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
