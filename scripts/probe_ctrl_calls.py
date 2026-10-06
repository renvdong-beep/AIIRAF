#!/usr/bin/env python3
"""比对「hold 调用起点」账：取值分叉是否来自**调用边界落在不同植物步号**（§11.88 量测⑳-A）。

数据来源：`PLANT_CTRL_CALLS`（`IRAF_DEBUG_CTRL_CALLS=1`，**全 run 一行**）=
每次 `_run_control` 调用的 `(植物步号, 仿真钟, 周期数, 通路)`。

为什么需要它：非步态路径的 `desired = q0 + alpha*(q_des - q0)`，`q0` 与 `alpha` 都取
「该次调用开始时」的位形/仿真钟 ⇒ **同一植物步、同一植物状态却算出不同控制量**时，
「调用起点不同」是唯一自洽出口（量测⑱/⑳）。

判读：
  两支的调用起点表**逐项相同** ⇒ 否掉判定 A，取值分叉只能来自植物状态先变（判定 B：转查臂侧）。
  某一项起不同 ⇒ **判定 A 坐实**：hold 调用边界没锚到植物步号 ⇒ 修法＝按步号起调用。

只读 build/diagnostics/*.log，不修改配置、不写仓库文件。

用法：python3 scripts/probe_ctrl_calls.py 'build/diagnostics/calls-round*.log'
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os

NORMAL_YAW = -2.38582468189734
YAW_TOL = 1e-9
PATH_TAG = {"ramp": "斜坡(stand/hold)", "gait": "步态(locomote)", "zero": "零力矩"}


def parse(path: str):
    dock = None
    calls = None
    with open(path, errors="replace") as fh:
        for line in fh:
            if line.startswith("PLANT_CTRL_CALLS "):
                try:
                    calls = json.loads(line.split(" ", 1)[1])
                except ValueError:
                    continue
            elif line.startswith("DOCK_HALT "):
                try:
                    d = json.loads(line[len("DOCK_HALT "):])
                except ValueError:
                    continue
                if str(d.get("station", "")).endswith("b"):
                    dock = d
    return dock, calls


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--around", type=int, default=2, help="分叉点前后各打几项")
    args = ap.parse_args()

    paths = sorted({p for pattern in args.logs for p in globmod.glob(pattern)})
    if not paths:
        print("无数据：参数未匹配到任何日志")
        return 2

    rounds = []
    for p in paths:
        dock, calls = parse(p)
        if dock is None or calls is None:
            continue
        yaw = dock.get("final_yaw_deg")
        rounds.append({
            "file": os.path.basename(p),
            "branch": "常态" if (yaw is not None and abs(yaw - NORMAL_YAW) <= YAW_TOL) else "备选",
            "yaw": yaw,
            "calls": calls,
        })

    if not rounds:
        print("无数据：%d 个日志里没有一行 PLANT_CTRL_CALLS"
              "（需带 IRAF_DEBUG_CTRL_CALLS=1 采集）" % len(paths))
        return 2

    normal = next((r for r in rounds if r["branch"] == "常态"), None)
    anoms = [r for r in rounds if r["branch"] == "备选"]
    print("共 %d 轮：常态 %d / 备选 %d；调用条数 = %s" % (
        len(rounds), len(rounds) - len(anoms), len(anoms),
        sorted({len(r["calls"]) for r in rounds})))
    if normal is None:
        print("无常态参照轮 ⇒ 不构成结论")
        return 3
    if not anoms:
        print("本批无备选轮 ⇒ 不构成结论（需更大批次）")
        return 4

    ref = normal["calls"]
    for r in anoms:
        cur = r["calls"]
        n = min(len(ref), len(cur))
        first = next((i for i in range(n) if ref[i] != cur[i]), None)
        print("\n--- %s（yaw=%r，%d 条调用）" % (r["file"], r["yaw"], len(cur)))
        if len(ref) != len(cur):
            print("    ⚠ 调用条数不同：常态 %d vs 备选 %d" % (len(ref), len(cur)))
        if first is None and len(ref) == len(cur):
            print("    **调用起点表逐项相同** ⇒ 否掉判定 A（调用相位）；"
                  "取值分叉只能是植物状态先变 ⇒ 转判定 B（查臂侧写入）")
            continue
        if first is None:
            continue
        print("    **第一次不同的调用项 = 第 %d 项**（0 起）" % first)
        lo = max(0, first - args.around)
        hi = min(n, first + args.around + 1)
        for i in range(lo, hi):
            mark = "  <<< 分叉" if i == first else ""
            print("      #%-3d 常态=%-42s 备选=%s%s" % (
                i, ref[i], cur[i], mark))
        a, b = ref[first], cur[first]
        print("      通路: %s vs %s；周期数: %s vs %s；植物步号: %s vs %s；仿真钟: %s vs %s" % (
            PATH_TAG.get(a[3], a[3]), PATH_TAG.get(b[3], b[3]), a[2], b[2], a[0], b[0], a[1], b[1]))
        if a[3] != b[3]:
            print("      ⇒ 通路类型不同（斜坡/步态/零力矩）⇒ 差异在**技能分派**，不是相位")
        else:
            print("      ⇒ **判定 A 坐实**：同一通路下调用起点（步号/仿真钟）不同 ⇒ "
                  "hold 调用边界未锚到植物步号")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
