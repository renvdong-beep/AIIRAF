#!/usr/bin/env python3
"""`nominal --world joint` 的**多轮连跑**与分布统计（2026-09-30 §11.40 纪律要求）。

为什么要连跑（本轮教训）：连续两轮得到 **7/8 全绿** 与 **3/5** 两种结果 ⇒ 轮间方差是当前主要障碍，
用单轮数字判"修好了/修坏了"必然误判。本脚本把每轮的关键量落盘并给出分布：

  · 每步状态与失败 `reason`（判读只用 reason，不用判据名——见 §11.25(f-7) 的踩坑）
  · s04 的 `place_correction_lateral_m` / `place_correction_vertical_m` / `place_offset_from_tray_center_m`
  · s06 的到位残差（从 reason 里取 `distance=`，或从 `PICK_SETTLED` 取）
  · 通过率（passed 的步数 / 总步数）与"哪一步最先失败"

用法（从仓库根执行）：
  PYTHONPATH=src python3 scripts/run_nominal_campaign.py --runs 4
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
REPORT = REPO / "build/acceptance/handoff_lab/nominal/report.json"


def _latest_report():
    return json.loads(REPORT.read_text(encoding="utf-8"))


def _extract(log_text):
    """从日志里取 `PICK_SETTLED` / `PICK_CORRECTION`（都与 pickup 的到位/纠偏直接相关）。"""
    settled, correction = [], []
    for line in log_text.splitlines():
        if line.startswith("PICK_SETTLED "):
            try:
                settled.append(json.loads(line.split("PICK_SETTLED ", 1)[1]))
            except json.JSONDecodeError:
                pass
        elif line.startswith("PICK_CORRECTION "):
            try:
                payload = json.loads(line.split("PICK_CORRECTION ", 1)[1])
                correction.append({"delta_norm_m": payload.get("delta_norm_m"),
                                   "live_target_m": payload.get("live_target_m"),
                                   "nominal_grasp_point_m": payload.get("nominal_grasp_point_m")})
            except json.JSONDecodeError:
                pass
    return settled, correction


def _distance_from_reason(reason):
    match = re.search(r"distance=([0-9.]+)m", str(reason or ""))
    return float(match.group(1)) if match else None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=4)
    parser.add_argument("--scene", default="scenes/handoff_lab")
    parser.add_argument("--scenario", default="nominal")
    parser.add_argument("--world", default="joint")
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_path = pathlib.Path(args.output or
                            "build/diagnostics/campaign-%s-%s.json" % (args.scenario, stamp))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rounds = []
    for index in range(1, max(1, args.runs) + 1):
        command = [sys.executable, "scripts/scenario.py", "run", "--scene", args.scene,
                   "--scenario", args.scenario, "--world", args.world, "--display", "none"]
        started = time.time()
        completed = subprocess.run(command, cwd=str(REPO), capture_output=True, text=True,
                                   env={**__import__("os").environ, "IRAF_DEBUG_PICK": "1"})
        elapsed = round(time.time() - started, 3)
        record = {"round": index, "exit_code": int(completed.returncode), "wall_s": elapsed}
        try:
            report = _latest_report()
            steps = [(str(item.get("id")), str(item.get("status")),
                      str(item.get("reason") or ""), item.get("measured") or {})
                     for item in (report.get("steps") or [])]
            record["passed"] = bool(report.get("passed"))
            record["steps_total"] = len(steps)
            record["steps_ok"] = sum(1 for item in steps if item[1] == "SUCCEEDED")
            record["first_failure"] = next(({"id": item[0], "status": item[1],
                                             "reason": item[2][:220]}
                                            for item in steps if item[1] != "SUCCEEDED"), None)
            for item in steps:
                if item[0] == "s04_place_in_tray":
                    record["s04"] = {key: value for key, value in item[3].items()
                                     if key.startswith("place_")}
                if item[0] == "s06_unload_at_b":
                    record["s06_distance_m"] = _distance_from_reason(item[2])
                    record["s06_status"] = item[1]
        except Exception as error:  # noqa: BLE001
            record["report_error"] = "%s: %s" % (type(error).__name__, error)
        settled, correction = _extract(completed.stdout or "")
        if settled:
            record["settled"] = settled[-1]
        if correction:
            record["correction"] = correction
        rounds.append(record)
        out_path.write_text(json.dumps({"runs": rounds}, ensure_ascii=False, indent=2),
                            encoding="utf-8")     # 每轮即时落盘（可中途判读）
        print("[round %d] exit=%d passed=%s steps=%s/%s first_failure=%s" % (
            index, record["exit_code"], record.get("passed"), record.get("steps_ok"),
            record.get("steps_total"),
            (record.get("first_failure") or {}).get("id")), flush=True)

    total = len(rounds)
    green = sum(1 for item in rounds if item.get("passed"))
    failures = [{"id": (item.get("first_failure") or {}).get("id"),
                 "reason": (item.get("first_failure") or {}).get("reason")}
                for item in rounds if item.get("first_failure")]
    lateral = [round(float((item.get("s04") or {}).get("place_correction_lateral_m")), 9)
               for item in rounds if (item.get("s04") or {}).get("place_correction_lateral_m") is not None]
    s06 = [item.get("s06_distance_m") for item in rounds if item.get("s06_distance_m") is not None]
    summary = {"runs": total, "all_green": green, "first_failures": failures,
               "s04_lateral_m": lateral, "s06_distance_m": s06,
               "note": ("单轮结论无效：通过率与残差必须看分布（本脚本把每轮原样落盘）。")}
    out_path.write_text(json.dumps({"summary": summary, "runs": rounds}, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("证据: %s" % out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
