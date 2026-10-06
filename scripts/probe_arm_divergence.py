#!/usr/bin/env python3
"""比对臂侧控制写入账（`PLANT_ARM_CTRL`）：定位**第一个取值不同的臂侧拍**（§11.88 量测㉒）。

背景：狗的取值分叉被锁进植物步 `[14750,14755)` 这 5 步内，而**该窗口内狗一次都没写 ctrl**
⇒ 能让植物状态在该窗口变化的只剩另一台本体（Piper）。本脚本按机器人分别比对
`(植物步号, 该拍全部通道取值摘要)` 序列，给出分叉在**臂侧**的首次显形位置。

判读：
  某机器人首个不同项出现在步号 S ⇒ 臂侧从 S 起与常态不同 ⇒ 源头窗口 = (前一项步号, S]。
  两支序列**逐项相同** ⇒ 臂侧也不是源头 ⇒ 回到"谁在哪一拍推进植物"（推进口径）继续查。
  条数不同 ⇒ 臂侧写入拍数本身不同（也是结论，别当噪声）。

只读 build/diagnostics/*.log，不修改配置、不写仓库文件。

用法：python3 scripts/probe_arm_divergence.py 'build/diagnostics/arm-*.log'
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os

NORMAL_YAW = -2.38582468189734
YAW_TOL = 1e-9


def parse(path: str):
    """返回 (dock_b, {robot: [(step, digest), ...]})。"""
    dock = None
    arms: dict = {}
    with open(path, errors="replace") as fh:
        for line in fh:
            if line.startswith("PLANT_ARM_CTRL "):
                try:
                    d = json.loads(line.split(" ", 1)[1])
                except ValueError:
                    continue
                arms[str(d.get("robot"))] = [tuple(x) for x in (d.get("writes") or [])]
            elif line.startswith("DOCK_HALT "):
                try:
                    d = json.loads(line[len("DOCK_HALT "):])
                except ValueError:
                    continue
                if str(d.get("station", "")).endswith("b"):
                    dock = d
    return dock, arms


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--around", type=int, default=3)
    ap.add_argument("--window-step", type=int, default=None,
                    help="只看某个植物步号附近的臂侧写入（例：14750）")
    ap.add_argument("--window", type=int, default=20, help="--window-step 的半径（步）")
    args = ap.parse_args()

    paths = sorted({p for pattern in args.logs for p in globmod.glob(pattern)})
    if not paths:
        print("无数据：参数未匹配到任何日志")
        return 2

    rounds = []
    for p in paths:
        dock, arms = parse(p)
        if dock is None or not arms:
            continue
        yaw = dock.get("final_yaw_deg")
        rounds.append({
            "file": os.path.basename(p),
            "branch": "常态" if (yaw is not None and abs(yaw - NORMAL_YAW) <= YAW_TOL) else "备选",
            "yaw": yaw,
            "arms": arms,
        })

    if not rounds:
        print("无数据：%d 个日志里没有 `PLANT_ARM_CTRL` 行"
              "（需带 IRAF_DEBUG_ARM_CTRL=1 采集）" % len(paths))
        return 2

    normal = next((r for r in rounds if r["branch"] == "常态"), None)
    anoms = [r for r in rounds if r["branch"] == "备选"]
    if args.window_step is not None:
        # 只读窗口模式：看某个植物步号附近"臂侧到底在哪几拍写了什么"。用途：
        # ① 确认臂侧格点形状（是否每 `substeps` 步一次）；② 事后对着分叉步做人工核对。
        if normal is None:
            print("无常态参照轮 ⇒ 不构成结论")
            return 3
        for r in ([normal] + anoms):
            print("\n--- %s 臂侧写入（植物步 %d ± %d）" % (
                r["file"], args.window_step, args.window))
            for rb in sorted(r["arms"]):
                _lo = args.window_step - args.window
                _hi = args.window_step + args.window
                _win = [(s, d) for s, d in r["arms"][rb] if _lo <= s <= _hi]
                print("  [%s] 命中 %d 条：%s" % (rb, len(_win), _win[:16]))
        return 0
    robots = sorted({rb for r in rounds for rb in r["arms"]})
    print("共 %d 轮：常态 %d / 备选 %d；臂侧机器人 = %s；各轮臂侧拍数 = %s" % (
        len(rounds), len(rounds) - len(anoms), len(anoms), robots,
        sorted({len(v) for r in rounds for v in r["arms"].values()})))
    if normal is None:
        print("无常态参照轮 ⇒ 不构成结论")
        return 3
    if not anoms:
        print("本批无备选轮 ⇒ 不构成结论（需更大批次）")
        return 4

    for r in anoms[:3]:
        print("\n--- %s（yaw=%r）" % (r["file"], r["yaw"]))
        for rb in robots:
            ref = normal["arms"].get(rb) or []
            cur = r["arms"].get(rb) or []
            if not ref or not cur:
                print("  [%s] 缺数据：常态 %d 项 / 备选 %d 项" % (rb, len(ref), len(cur)))
                continue
            n = min(len(ref), len(cur))
            first = next((i for i in range(n) if ref[i] != cur[i]), None)
            if len(ref) != len(cur):
                print("  [%s] ⚠ 拍数不同：常态 %d / 备选 %d" % (rb, len(ref), len(cur)))
            if first is None:
                print("  [%s] 前 %d 项**逐项相同**%s" % (
                    rb, n, "" if len(ref) == len(cur) else "（尾部长度不同，见上）"))
                continue
            print("  [%s] **首个不同项 = 第 %d 项**：常态 步号=%s 摘要=%s / 备选 步号=%s 摘要=%s" % (
                rb, first, ref[first][0], ref[first][1], cur[first][0], cur[first][1]))
            lo = max(0, first - args.around)
            hi = min(n, first + args.around + 1)
            for i in range(lo, hi):
                mark = "  <<< 分叉" if i == first else ""
                print("        #%-5d 常态=%-18s 备选=%-18s%s" % (i, ref[i], cur[i], mark))
            nxt = cur[first][0]
            prv = ref[first - 1][0] if first > 0 else None
            print("        ⇒ 源头窗口 = 植物步 (%s, %s]（臂侧从该窗口起与常态不同）" % (prv, nxt))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
