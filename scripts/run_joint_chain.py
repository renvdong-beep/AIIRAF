#!/usr/bin/env python3
"""联合世界 9 步链一键复跑 + 判读（2026-09-30 §11.54）。

用法（仓库根目录）：
    PYTHONPATH=src python3 scripts/run_joint_chain.py

为什么落成脚本（AGENTS.md 5.2）：单次运行 ~3 分钟，判读要同时看三处 ——
① 报告里每步的 status/reason ② 调试通路的 `PICK_RESULT`/`PLACE_GATE`（步驟交接取证）
③ 抓取段停稳量 `PICK_SETTLED`。人工拼输出既慢又容易漏（本会话已因"只看判据名不看 reason"
和"用管道吞退出码"各栽过一次）。这里把三处合成一次判读，并**原样保留退出码**。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
REPORT = REPO / "build" / "acceptance" / "handoff_lab" / "nominal" / "report.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="scenes/handoff_lab")
    parser.add_argument("--scenario", default="nominal")
    parser.add_argument("--world", default="joint")
    parser.add_argument("--log", default="build/diagnostics/joint-chain.log")
    args = parser.parse_args()

    log_path = REPO / args.log
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.setdefault("PYTHONPATH", "src")
    env["IRAF_DEBUG_PICK"] = "1"
    env["IRAF_DEBUG_PLACE"] = "1"
    command = [sys.executable, "scripts/scenario.py", "run",
               "--scene", args.scene, "--scenario", args.scenario,
               "--world", args.world, "--display", "none"]
    print("[run] " + " ".join(command))
    with log_path.open("w") as handle:
        exit_code = subprocess.call(command, cwd=str(REPO), env=env, stdout=handle,
                                    stderr=subprocess.STDOUT)
    print("[exit] %d  (日志 %s)" % (exit_code, log_path))

    if not REPORT.exists():
        print("[报告] 不存在：%s" % REPORT)
        return exit_code
    report = json.loads(REPORT.read_text())
    print("[report] passed=%s" % report.get("passed"))
    for step in report.get("steps", []):
        print("  %-26s %-9s %s" % (step.get("id"), step.get("status"),
                                   (step.get("reason") or "")[:220]))

    lines = log_path.read_text(errors="replace").splitlines()
    print("\n[交接取证]（PICK_RESULT 的 handoff_state = 本步返回那一刻；PLACE_GATE = 下一步起步那一刻）")
    for line in lines:
        if line.startswith("PICK_RESULT") or line.startswith("PLACE_GATE"):
            print("  " + line[:1200])
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
