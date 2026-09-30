#!/usr/bin/env python3
"""s06→s07 交接取证复跑（2026-09-30 §11.54）。

背景：s07（放置）起步即报「未夹持载荷」有两种**完全不同的**机制，必须分开量：
  ① s06 自己没抓上（`末端未到达目标抓取位姿`）⇒ 载荷仍在托盘、夹爪停在张开位；
     这时的 s07 拒绝是**正确的连锁**，与交接无关。
  ② s06 抓上了（PICK_RESULT grasped=True）但**步骤交界处**把载荷放掉了。
s06 的抓取残差是**间歇**的（实测 0.0034~0.0082 m，容差 0.005）⇒ 需要连跑到一次成功才有得看。

本脚本：连跑 N 轮，每轮单独落日志，只汇总「交接」相关的三个事实源：
  · 报告里 s06/s07 的 status 与 reason
  · s06 的 `PICK_RESULT`（含 `handoff_state` = **本步返回那一刻**的载荷/指腹间距、夹爪 ctrl/qpos、
    焊缝激活状态、载荷接触体）
  · s07 的 `PLACE_GATE`（= **下一步起步那一刻**的同组量）
两者对账即可判定「谁在什么时候把载荷放掉」。

用法（仓库根目录）：
    PYTHONPATH=src python3 scripts/probe_handoff_handover.py --rounds 3
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
REPORT = REPO / "build" / "acceptance" / "handoff_lab" / "nominal" / "report.json"
DIAG = REPO / "build" / "diagnostics"


def run_round(index, scene, scenario, world):
    log_path = DIAG / ("handoff-round-%d.log" % index)
    env = dict(os.environ)
    env.setdefault("PYTHONPATH", "src")
    env["IRAF_DEBUG_PICK"] = "1"
    env["IRAF_DEBUG_PLACE"] = "1"
    command = [sys.executable, "scripts/scenario.py", "run",
               "--scene", scene, "--scenario", scenario,
               "--world", world, "--display", "none"]
    started = time.monotonic()
    with log_path.open("w") as handle:
        exit_code = subprocess.call(command, cwd=str(REPO), env=env, stdout=handle,
                                    stderr=subprocess.STDOUT)
    return exit_code, log_path, time.monotonic() - started


def collect(log_path):
    """从日志里取**交接相关**的两条调试通路（s06 的返回时刻 / s07 的起步时刻）。"""
    pick_result = None
    place_gate = None
    for line in log_path.read_text(errors="replace").splitlines():
        if line.startswith("PICK_RESULT "):
            payload = json.loads(line.split(" ", 1)[1])
            # 只认 **UR5e** 那次（夹爪通道名带本体前缀）；Piper 的是 s03，与本问题无关。
            channels = list(((payload.get("handoff_state") or {}).get("gripper_ctrl") or {}))
            if any(name.startswith("ur5e_") for name in channels):
                pick_result = payload
        elif line.startswith("PLACE_GATE "):
            place_gate = json.loads(line.split(" ", 1)[1])
    return pick_result, place_gate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--scene", default="scenes/handoff_lab")
    parser.add_argument("--scenario", default="nominal")
    parser.add_argument("--world", default="joint")
    args = parser.parse_args()

    DIAG.mkdir(parents=True, exist_ok=True)
    rounds = []
    for index in range(1, args.rounds + 1):
        print("=" * 78)
        print("[轮 %d/%d] 开始" % (index, args.rounds), flush=True)
        exit_code, log_path, wall = run_round(index, args.scene, args.scenario, args.world)
        report = json.loads(REPORT.read_text()) if REPORT.exists() else {}
        steps = {step["id"]: step for step in report.get("steps", [])}
        pick_result, place_gate = collect(log_path)
        record = {
            "round": index, "exit_code": exit_code, "wall_seconds": round(wall, 1),
            "log": str(log_path.relative_to(REPO)),
            "steps": {step_id: {"status": steps[step_id]["status"],
                                "reason": (steps[step_id].get("reason") or "")[:300]}
                      for step_id in ("s02b_dock_station_b", "s06_unload_at_b",
                                      "s07_place_at_b_table") if step_id in steps},
            "pick_result": pick_result,
            "place_gate": place_gate,
        }
        rounds.append(record)
        print("[轮 %d] exit=%d wall=%.1fs s06=%s s07=%s" % (
            index, exit_code, wall,
            record["steps"].get("s06_unload_at_b", {}).get("status"),
            record["steps"].get("s07_place_at_b_table", {}).get("status")), flush=True)
        if pick_result:
            state = pick_result.get("handoff_state") or {}
            print("  [s06 返回时刻] grasped=%s lift=%.6f 载荷=%s" % (
                pick_result.get("grasped"), pick_result.get("lift_delta_m") or -1.0,
                state.get("payload_xyz_m")), flush=True)
            print("     指腹间距 左=%.9f 右=%.9f 夹爪ctrl=%s 焊缝active=%s 载荷接触体=%s" % (
                state.get("payload_left_gap_m") or float("nan"),
                state.get("payload_right_gap_m") or float("nan"),
                state.get("gripper_ctrl"), state.get("lift_constraint_active"),
                state.get("payload_contact_bodies")), flush=True)
        if place_gate:
            print("  [s07 起步时刻] 指腹间距 左=%.9f 右=%.9f 夹爪ctrl=%s qpos=%s 焊缝active=%s 载荷接触体=%s" % (
                place_gate.get("payload_left_gap_m") or float("nan"),
                place_gate.get("payload_right_gap_m") or float("nan"),
                place_gate.get("gripper_ctrl"), place_gate.get("gripper_qpos"),
                place_gate.get("lift_constraint_active"),
                place_gate.get("payload_contact_bodies")), flush=True)

    out = DIAG / ("handoff-handover-%s.json" % time.strftime("%Y%m%d-%H%M%S"))
    out.write_text(json.dumps({"rounds": rounds}, ensure_ascii=False, indent=2))
    print("=" * 78)
    print("[汇总] 写入 %s" % out.relative_to(REPO))
    successes = sum(1 for item in rounds
                    if item["steps"].get("s06_unload_at_b", {}).get("status") == "SUCCEEDED")
    print("[汇总] s06 成功 %d/%d 轮" % (successes, len(rounds)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
