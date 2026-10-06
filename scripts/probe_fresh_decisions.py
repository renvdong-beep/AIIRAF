#!/usr/bin/env python3
"""按轮汇总「MPC 新鲜度决策」的逐步增量（§11.88 量测⑲）。

判定目标：s02b 停靠的离散分支表现为「**同一植物步号、不同控制量**」
（§11.88 量测⑱）⇒ 差异在控制器内部状态。本探针检验最后一个墙钟嫌疑：
`freshness.decide(age_ms=…)` 按**墙钟**判解的新鲜度 ⇒「用解 / 降级(damped_hold) /
释放(release) / 跳过(skip)」的决定依赖挂钟，从而让同一拍算出不同控制量。

判读：
  异常轮的逐步四元组增量 (unavailable, holds, releases, skips) 与常态轮**不同**
    ⇒ 坐实墙钟新鲜度是「取值分叉」的通道 ⇒ 修法：新鲜度判据换确定性口径（步号/周期）。
  **全同** ⇒ 该候选也被否掉，转查驻留 stand 的其它内部状态。

只读 build/diagnostics/fresh-round*.log，不修改任何配置、不写仓库文件（铁律 §5）。
这些日志含巨大的 DOCK_TRACE 行 ⇒ 一律按前缀挑行解析，禁止整文件打印。

用法：
  python3 scripts/probe_fresh_decisions.py
  python3 scripts/probe_fresh_decisions.py --glob 'build/diagnostics/*round*.log'
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os

NORMAL_YAW = -2.38582468189734
YAW_TOL = 1e-9
FRESH_KEYS = ("_mpc_unavailable", "_mpc_holds", "_mpc_releases", "_mpc_skips")


def parse_round(path: str):
    """返回 (dock_b, spans)。只看两类前缀行，避免解析 290 KB 的 DOCK_TRACE。"""
    dock = None
    spans = None
    with open(path, errors="replace") as fh:
        for line in fh:
            if line.startswith("PLANT_STEP_SPANS "):
                try:
                    spans = json.loads(line.split(" ", 1)[1])
                except ValueError:
                    continue
            elif line.startswith("DOCK_HALT "):
                try:
                    d = json.loads(line[len("DOCK_HALT "):])
                except ValueError:
                    continue
                if str(d.get("station", "")).endswith("b"):
                    dock = d
    return dock, spans


def fresh_tuple(step: dict) -> tuple:
    return tuple(int(step.get(k, 0) or 0) for k in FRESH_KEYS)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="build/diagnostics/fresh-round*.log")
    ap.add_argument("--step", default="s03_pick",
                    help="打印逐步四元组的步骤 id（默认 s03_pick：分叉起源步）")
    args = ap.parse_args()

    paths = sorted(globmod.glob(args.glob))
    if not paths:
        print("无数据：%s 未匹配到日志（脚本必须报「无数据」而不是「相同」）" % args.glob)
        return 2

    rounds = []
    for p in paths:
        dock, spans = parse_round(p)
        if dock is None or spans is None:
            print("跳过 %s（缺 DOCK_HALT(b) 或 PLANT_STEP_SPANS）" % os.path.basename(p))
            continue
        yaw = dock.get("final_yaw_deg")
        branch = "常态" if (yaw is not None and abs(yaw - NORMAL_YAW) <= YAW_TOL) else "备选"
        rounds.append({
            "file": os.path.basename(p),
            "yaw": yaw,
            "branch": branch,
            "reached_s": dock.get("reached_s"),
            "iter_total": (dock.get("mpc_hook") or {}).get("iter_total"),
            "spans": spans,
        })

    if not rounds:
        print("无数据：%d 个日志里没有一个含完整两行留证" % len(paths))
        return 2

    print("轮次总览（判据见 DOCK_HALT；常态 yaw 基准 %r）" % NORMAL_YAW)
    print("%-22s %-4s %-22s %-22s %-10s %s" % ("file", "支", "final_yaw_deg", "reached_s", "iter", "s03 四元组"))
    for r in rounds:
        st = next((s for s in r["spans"] if str(s.get("id")) == args.step), None)
        print("%-22s %-4s %-22s %-22s %-10s %s" % (
            r["file"], r["branch"], repr(r["yaw"]), repr(r["reached_s"]),
            r["iter_total"], (fresh_tuple(st) if st else "(缺该步)")))

    normal = next((r for r in rounds if r["branch"] == "常态"), None)
    anoms = [r for r in rounds if r["branch"] == "备选"]
    print("\n常态 %d 轮 / 备选 %d 轮" % (len(rounds) - len(anoms), len(anoms)))
    if normal is None:
        print("无常态参照轮 ⇒ 无法比对（不构成任何结论）")
        return 3
    if not anoms:
        print("本批无备选轮 ⇒ 无法判定新鲜度通道（需更大批次）；"
              "但下面的常态轮逐步四元组可用于确认「计数是否非零」")
        anoms = []

    # 逐步四元组：常态内部是否一致 + 与备选是否不同
    ref = {str(s["id"]): fresh_tuple(s) for s in normal["spans"]}
    same_within_normal = all(
        {str(s["id"]): fresh_tuple(s) for s in r["spans"]} == ref
        for r in rounds if r["branch"] == "常态")
    print("常态轮之间逐步四元组逐位一致? %s" % same_within_normal)

    for r in anoms:
        cur = {str(s["id"]): fresh_tuple(s) for s in r["spans"]}
        diff = [sid for sid in ref if sid in cur and cur[sid] != ref[sid]]
        if not diff:
            print("%s：逐步四元组与常态轮**完全相同** ⇒ 否掉墙钟新鲜度通道" % r["file"])
        else:
            print("%s：**第一个四元组不同的步骤 = %s**" % (r["file"], diff[0]))
            for sid in diff[:4]:
                print("    %-22s 常态=%s 备选=%s" % (sid, ref[sid], cur[sid]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
