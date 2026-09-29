#!/usr/bin/env python3
"""稳定性探针（2026-09-29 §11.23(48) 收尾）：同配置连跑 N 次，量化"绿"的可信度。

为什么必须（本轮教训）：我在只连续几次绿的情况下就下过"稳定"的结论 —— 随后在**关闭 regrasp 的
基线配置**上同配置连跑出现了一次失败（载荷在抓取段被碰掉到地上）。判据只有一个样本时说"稳定"是
过度概括；本探针把通过率、失败模式与关键量的离散度变成**可验收的数字**。

用法：python3 build/stability_probe.py [次数]（默认 6）
输出：build/stability.json（逐次明细 + 汇总）
"""

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "build/acceptance/handoff_lab/nominal/report.json"
OUT = ROOT / "build/stability.json"
KEYS = ("grasp_center_distance_m", "grasp_lift_delta_m", "place_offset_from_tray_center_m",
        "accept_offset_from_target_center_m", "accept_last_speed_mps", "place_carry_payload_lateral_m")


def main():
    times = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    runs = []
    for index in range(1, times + 1):
        completed = subprocess.run(
            [sys.executable, "scripts/scenario.py", "run", "--scene", "scenes/handoff_lab",
             "--scenario", "nominal", "--world", "joint"],
            cwd=str(ROOT), capture_output=True, text=True,
            env={**__import__("os").environ, "PYTHONPATH": str(ROOT / "src")})
        record = {"run": index, "exit_code": completed.returncode}
        try:
            report = json.loads(REPORT.read_text(encoding="utf-8"))
            record["passed"] = bool(report.get("passed"))
            record["failed_checks"] = len(report.get("failed_checks") or [])
            record["failed_heads"] = [(item or "")[:120] for item in (report.get("failed_checks") or [])[:2]]
            measured = {}
            for step in report.get("steps") or []:
                for key, value in (step.get("measured") or {}).items():
                    if key in KEYS:
                        measured[key] = value
            record["measured"] = measured
        except Exception as error:  # noqa: BLE001 —— 解析失败也要如实记录
            record["parse_error"] = str(error)
        runs.append(record)
        print("run %d/%d exit=%s passed=%s failed=%s" % (
            index, times, record.get("exit_code"), record.get("passed"),
            record.get("failed_checks")), flush=True)

    passed = [item for item in runs if item.get("passed")]
    summary = {"times": times, "passed": len(passed), "failed": times - len(passed)}
    for key in KEYS:
        values = [item["measured"][key] for item in runs
                  if item.get("measured", {}).get(key) is not None]
        if values:
            summary[key] = {"min": min(values), "max": max(values),
                            "spread": round(max(values) - min(values), 9),
                            "n": len(values)}
    OUT.write_text(json.dumps({"summary": summary, "runs": runs}, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    print("\n==== 汇总 ====", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=1), flush=True)
    for item in runs:
        if not item.get("passed"):
            print("失败 run %d: exit=%s %s" % (item["run"], item.get("exit_code"),
                                            (item.get("failed_heads") or [""])[0]), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
