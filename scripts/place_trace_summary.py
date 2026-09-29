#!/usr/bin/env python3
"""`PLACE_TRACE` 取证判读：从连跑日志里抽取"掉件在哪一段、哪一刻发生"需要的量。

背景（2026-09-29 §11.25(f)）：联合世界放置段有约 1/4 概率掉件（方块从托盘掉到台面），
失败终态**逐位相同**（载荷最低点 −0.000215511 m）⇒ 终态确定、触发不确定。失败轮的
s04/s05 `evidence` 是空的（失败路径不产证据）⇒ 只能用 `IRAF_DEBUG_PLACE=1` 在**执行过程中**
打印的 `PLACE_TRACE` 行做判读。

本脚本只做"取数与对照"，**不下结论**：
  · 逐轮列出 phase 与关键量（载荷中心/最低点、指腹中点、载荷相对夹口偏移、托盘与载荷的
    相对位置、接触对、落稳间隙、末速）；
  · 通过轮与失败轮并排，标注差异最大的量（差值超过阈值才标，避免噪声）；
  · 输出 JSON 供后续引用，stdout 给紧凑表。

用法：
  python3 scripts/place_trace_summary.py build/joint-stability/run_*.log
  python3 scripts/place_trace_summary.py --output build/diagnostics/place-trace-summary.json <logs...>
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

TRACE_PREFIX = "PLACE_TRACE "
#: 关心的量（键名按子串匹配，兼容字段增删）：只打印命中这些子串且非空的键
INTEREST = ("payload", "pad", "tray", "anchor", "gap", "speed", "contact", "offset",
            "lateral", "vertical", "lowest", "min_z", "phase", "step", "iteration",
            "plant_step", "released", "in_tray", "settle")
#: 差异标注阈值（相对差）：超过才值得看
DIFF_REL = 0.02


def _load_runs(paths):
    runs = []
    for path in paths:
        text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
        rows = []
        for line in text.splitlines():
            if not line.startswith(TRACE_PREFIX):
                continue
            try:
                rows.append(json.loads(line[len(TRACE_PREFIX):]))
            except json.JSONDecodeError:
                continue
        passed = None
        match = re.search(r'"passed": (true|false)', text)
        if match:
            passed = match.group(1) == "true"
        failed_step = None
        m = re.search(r'"(s[0-9a-z_]+)"[^}]*"status": "FAILED"', text)
        if m:
            failed_step = m.group(1)
        runs.append({"log": path, "passed": passed, "failed_step": failed_step,
                     "trace_rows": len(rows), "rows": rows})
    return runs


def _flatten(row, prefix=""):
    flat = {}
    for key, value in (row or {}).items():
        name = "%s%s" % (prefix, key)
        if isinstance(value, dict):
            flat.update(_flatten(value, name + "."))
        elif isinstance(value, (list, tuple)):
            flat[name] = value
        else:
            flat[name] = value
    return flat


def _interesting(flat):
    keep = {}
    for key, value in flat.items():
        if value is None or value == "" or value == []:
            continue
        if any(token in key for token in INTEREST):
            keep[key] = value
    return keep


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--output", default="build/diagnostics/place-trace-summary.json")
    args = parser.parse_args()

    runs = _load_runs(args.logs)
    for run in runs:
        print("=== %s | passed=%s failed_step=%s | PLACE_TRACE 行数=%d"
              % (pathlib.Path(run["log"]).name, run["passed"], run["failed_step"],
                 run["trace_rows"]))
        if not run["rows"]:
            print("   （无 trace 行：该轮可能没跑到放置段）")
            continue
        phases = []
        for row in run["rows"]:
            phase = str(row.get("phase") or "?")
            if phase not in phases:
                phases.append(phase)
        print("   phase 序列：%s" % " → ".join(phases))
        # 每个 phase 的首/末行关键量
        by_phase = {}
        for row in run["rows"]:
            by_phase.setdefault(str(row.get("phase") or "?"), []).append(row)
        for phase, items in by_phase.items():
            first = _interesting(_flatten(items[0]))
            last = _interesting(_flatten(items[-1]))
            print("   [%s] n=%d" % (phase, len(items)))
            print("      首: %s" % json.dumps(first, ensure_ascii=False)[:330])
            print("      末: %s" % json.dumps(last, ensure_ascii=False)[:330])
        run["phases"] = {phase: {"first": _interesting(_flatten(items[0])),
                                 "last": _interesting(_flatten(items[-1])),
                                 "n": len(items)}
                         for phase, items in by_phase.items()}

    # 通过轮 vs 失败轮：逐 phase 逐量对照（只看两边都有的标量）
    ok = [run for run in runs if run["passed"]]
    bad = [run for run in runs if run["passed"] is False]
    if ok and bad:
        print("\n=== 通过轮 vs 失败轮（末值对照，|相对差| > %.0f%% 才标注）===" % (DIFF_REL * 100))
        for phase in sorted(bad[0].get("phases") or {}):
            if phase not in (ok[0].get("phases") or {}):
                continue
            good_last = ok[0]["phases"][phase]["last"]
            bad_last = bad[0]["phases"][phase]["last"]
            for key in sorted(set(good_last) & set(bad_last)):
                good_value, bad_value = good_last[key], bad_last[key]
                if not isinstance(good_value, (int, float)) or isinstance(good_value, bool):
                    continue
                if not isinstance(bad_value, (int, float)) or isinstance(bad_value, bool):
                    continue
                base = abs(good_value) if good_value else 1.0
                delta = abs(bad_value - good_value) / base
                if delta > DIFF_REL:
                    print("   [%s] %-42s 通过 %-14r 失败 %-14r Δrel=%.3f"
                          % (phase, key, good_value, bad_value, delta))
    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"runs": runs}, ensure_ascii=False, indent=1) + "\n",
                   encoding="utf-8")
    print("\nWROTE %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
