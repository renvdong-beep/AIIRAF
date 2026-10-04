#!/usr/bin/env python3
"""s06（UR5e 卸载）抓取残差的**逐相位拆解**（2026-09-30 §11.56）。

背景：s06 的"到位"判据是 0.005 m，而实测残差在 **0.0012~0.0083 m** 之间间歇
（同一份产物：0.0011997952751190118 / 0.001243694 / 0.0030730518534785945 / 0.007822 / 0.008228）
⇒ 演示轮会随机落在"抓到"或"没抓到"。要决定改哪个**内部**停止条件（不是放宽 0.005），
必须先知道残差由谁贡献：
  ① 手臂执行/IK 误差（纠偏时解出来的目标 vs 停稳后指腹实测）
  ② 目标漂移（载荷/托盘在"接近 + 下压"窗口里动了多少）
  ③ 测量口径差（指腹 geom 中点 vs body 中点等）
本脚本只读日志（`PICK_TRACE` / `PICK_CORRECTION` / `PICK_SETTLED` / `PICK_RESULT`），
按 `sim_time_s` 排序给出每段的**漂移率**（mm/s）与各量，并落 JSON 供后续对账。

用法（仓库根目录）：
    PYTHONPATH=src python3 scripts/probe_grasp_window_drift.py build/diagnostics/joint-chain-fix1.log ...
    # 不给参数时自动取 build/diagnostics/ 下与联合世界相关的日志
"""
from __future__ import annotations

import argparse
import glob
import json
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DIAG = REPO / "build" / "diagnostics"
DEFAULT_GLOBS = ("joint-chain*.log", "handoff-round-*.log")

PREFIXES = ("PICK_TRACE ", "PICK_CORRECTION ", "PICK_SETTLED ", "PICK_RESULT ")


def _rows(path):
    out = []
    for line in path.read_text(errors="replace").splitlines():
        for prefix in PREFIXES:
            if line.startswith(prefix):
                payload = json.loads(line[len(prefix):])
                payload["_kind"] = prefix.strip()
                out.append(payload)
                break
        else:
            m = re.match(r"^PICK_SETTLED\.samples (\{.*\})$", line.strip())
            if m:
                payload = json.loads(m.group(1))
                payload["_kind"] = "PICK_SETTLED.samples"
                out.append(payload)
    return out


def _is_ur5e(payload):
    """按夹爪通道名判定是不是 UR5e 那次（Piper 的是 joint7/joint8）。"""
    channels = list((payload.get("handoff_state") or {}).get("gripper_ctrl") or {})
    if channels:
        return any(str(name).startswith("ur5e_") for name in channels)
    joint_qpos = payload.get("joint_qpos") or {}
    return any(str(name).endswith(("shoulder_pan_joint", "elbow_joint")) for name in joint_qpos)


def _dist(a, b):
    return sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)) ** 0.5


def _label(path):
    """日志标签：能给仓库相对路径就给（绝对/相对入参都能处理）。"""
    try:
        return str(Path(path).resolve().relative_to(REPO.resolve()))
    except ValueError:
        return str(path)


def analyze(path):
    rows = [item for item in _rows(path) if _is_ur5e(item)]
    traces = [item for item in rows if item["_kind"] == "PICK_TRACE"]
    corrections = [item for item in rows if item["_kind"] == "PICK_CORRECTION"]
    settled = [item for item in rows if item["_kind"] == "PICK_SETTLED"]
    results = [item for item in rows if item["_kind"] == "PICK_RESULT"]
    report = {"log": _label(path), "phases": [], "corrections": [],
              "settled": None, "gate": None}
    print("=" * 78)
    print("[日志] %s" % _label(path))
    if not traces and not results:
        print("  （没有 UR5e 的抓取轨迹：本轮 s06 未执行到 / 未开 IRAF_DEBUG_PICK）")
        return report

    print("  -- 相位轨迹（指腹中点 / 目标点 / 载体）--")
    prev = None
    for item in traces:
        finger = item.get("finger_center_position_m")
        target = item.get("target_position_m")
        carrier = item.get("carrier_z_m")
        phase = item.get("phase")
        sim_time = item.get("sim_time_s")
        distance = item.get("center_distance_m")
        print("   %-16s sim=%-10s 指腹=%s 目标=%s 距离=%.9f 载体z=%s" % (
            phase, sim_time,
            [round(v, 6) for v in finger] if finger else None,
            [round(v, 6) for v in target] if target else None,
            float(distance) if distance is not None else float("nan"),
            round(float(carrier), 6) if carrier is not None else None))
        report["phases"].append({"phase": phase, "sim_time_s": sim_time,
                                 "finger_m": finger, "target_m": target,
                                 "distance_m": distance, "carrier_z_m": carrier})
        prev = item
    # 目标漂移率（相邻相位之间）
    print("  -- 目标漂移（相邻相位之间的位移与速率）--")
    for before, after in zip(traces, traces[1:]):
        if not (before.get("target_position_m") and after.get("target_position_m")):
            continue
        dt = float(after.get("sim_time_s") or 0.0) - float(before.get("sim_time_s") or 0.0)
        move = _dist(after["target_position_m"], before["target_position_m"])
        rate = (move / dt) if dt > 1e-9 else float("nan")
        print("   %-16s → %-16s Δt=%9.3f s  位移=%9.6f m  速率=%9.6f m/s" % (
            before.get("phase"), after.get("phase"), dt, move, rate))
        report.setdefault("drift", []).append(
            {"from": before.get("phase"), "to": after.get("phase"), "dt_s": dt,
             "move_m": move, "rate_m_s": rate})

    print("  -- 运行期纠偏（每次的修正量与各相位残差）--")
    for item in corrections:
        norm = item.get("delta_norm_m") or item.get("correction_norm_m")
        print("   sim=%s applied=%s delta_norm=%s 拒绝原因=%s" % (
            item.get("sim_time_s"), item.get("applied"), norm,
            str(item.get("rejected_reason"))[:80]))
        report["corrections"].append(item)

    if settled:
        item = settled[-1]
        print("  -- 下压停稳复量 --")
        print("   min=%.9f max=%.9f spread=%.9f tolerance=%s within=%s" % (
            float(item.get("min_distance_m", float("nan"))),
            float(item.get("max_distance_m", float("nan"))),
            float(item.get("spread_m", float("nan"))),
            item.get("tolerance_m"), item.get("within_tolerance")))
        report["settled"] = {"min_m": item.get("min_distance_m"), "max_m": item.get("max_distance_m"),
                             "spread_m": item.get("spread_m"),
                             "samples": item.get("samples")}
    if results:
        gate = results[-1]
        report["gate"] = {"grasped": gate.get("grasped"),
                          "center_distance_m": gate.get("center_distance_m"),
                          "lift_delta_m": gate.get("lift_delta_m"),
                          "bilateral_contact": gate.get("bilateral_contact")}
        print("  -- 到位门禁（`_grasp_alignment_evidence`）--")
        print("   center_distance_m=%s（判据 0.005）grasped=%s lifted=%s bilateral=%s" % (
            gate.get("center_distance_m"), gate.get("grasped"), gate.get("lifted"),
            gate.get("bilateral_contact")))
        settled_distance = None
        if report["settled"]:
            settled_distance = report["settled"]["min_m"]
        if settled_distance is not None and gate.get("center_distance_m") is not None:
            print("   停稳复量 min=%.9f → 门禁=%.9f：差值=%.9f m（同一时刻两次测量的口径差）" % (
                float(settled_distance), float(gate["center_distance_m"]),
                float(gate["center_distance_m"]) - float(settled_distance)))
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="*")
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args()
    paths = [Path(item) for item in args.logs]
    if not paths:
        for pattern in DEFAULT_GLOBS:
            paths.extend(sorted(Path(item) for item in glob.glob(str(DIAG / pattern))))
    paths = [item for item in paths if item.is_file()]
    if not paths:
        print("没有可分析的日志")
        return 1
    reports = [analyze(item) for item in paths]
    out = Path(args.json_out) if args.json_out else (DIAG / "grasp-window-drift.json")
    out.write_text(json.dumps({"reports": reports}, ensure_ascii=False, indent=2))
    print("=" * 78)
    print("[汇总] 写入 %s" % out.relative_to(REPO) if out.is_absolute() else out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
