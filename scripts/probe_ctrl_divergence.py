"""停靠偶发分叉定位（2026-10-05 §11.88 量测⑩）。

输入：`IRAF_DEBUG_DOCK=1 IRAF_DEBUG_CTRL_ALIGN=1` 连跑产生的日志（每轮一份）。
输出：各轮的 B 站末态 + 是否非常态；然后取**第一个异常轮**与**任一常态轮**逐拍比对
      `DOCK_HALT.ctrl_align.seq`（每次 ctrl 写入的「控制量取值摘要」），给出**第一次分叉的下标**
      与对应的植物步号 —— 这一拍就是「两条分支从这里分开」的位置。

为什么用它：§11.88 已按顺序否掉「步数 / 控制更新对齐 / MPC 计数」三层（异常轮与常态轮逐位相同），
只剩「控制量的**取值**」这一层；逐拍摘要能直接把分叉点钉到具体一拍。

只读日志，不跑仿真。
用法：python3 scripts/probe_ctrl_divergence.py build/diagnostics/cval-round*.log
"""

from __future__ import annotations

import json
import pathlib
import sys

NORMAL_YAW = -2.38582468189734


def _load(path):
    rows = []
    for line in pathlib.Path(path).read_text(errors="replace").splitlines():
        if line.startswith("DOCK_HALT "):
            try:
                rows.append(json.loads(line[len("DOCK_HALT "):]))
            except json.JSONDecodeError:
                pass
    return rows


def main(argv):
    normal = None
    anomaly = None
    print("%-30s %-22s %-9s %s" % ("log", "B_yaw", "wall_s", "判定"))
    for path in argv:
        rows = _load(path)
        b = next((row for row in rows if str(row.get("station", "")).endswith("b")), None)
        if b is None:
            continue
        yaw = b.get("final_yaw_deg")
        align = b.get("ctrl_align") or {}
        seq = align.get("seq")
        is_anomaly = yaw is not None and abs(yaw - NORMAL_YAW) > 1e-12
        if is_anomaly:
            if anomaly is None:
                anomaly = (path, b, seq)
            note = "异常（%s 拍）" % (len(seq) if seq else 0)
        else:
            if normal is None:
                normal = (path, b, seq)
            note = "常态（%s 拍）" % (len(seq) if seq else 0)
        print("%-30s %-22s %-9s %s" % (pathlib.Path(path).name,
                                       ("%+.15g" % yaw) if yaw is not None else "-",
                                       b.get("owner_wall_s"), note))
    if anomaly is None:
        print("\n本批无异常轮 ⇒ 无法比对分叉（不把「没抓到」当「没有」；再跑一批或加大轮数）")
        return
    if normal is None:
        print("\n只有异常轮、没有可比的常态轮")
        return
    print("\n=== 逐拍控制量摘要比对（常态 %s vs 异常 %s）==="
          % (pathlib.Path(normal[0]).name, pathlib.Path(anomaly[0]).name))
    ns, as_ = normal[2] or [], anomaly[2] or []
    n = min(len(ns), len(as_))
    first_diff = next((i for i in range(n) if ns[i] != as_[i]), None)
    step_first = (anomaly[1].get("ctrl_align") or {}).get("first")
    print("  可比拍数 = %d（常态 %d / 异常 %d）" % (n, len(ns), len(as_)))
    if first_diff is None:
        print("  ⇒ 两轮的全部控制量摘要逐拍相同：差异不在本停靠段的控制量取值（需再查上游/别处）")
    else:
        print("  ⇒ 第一次分叉在第 %d 拍（本段首拍 %s，植物步号 ≈ %s）"
              % (first_diff, step_first,
                 (step_first + 5 * first_diff) if isinstance(step_first, int) else "?"))
        print("     常态摘要 = %s" % ns[first_diff])
        print("     异常摘要 = %s" % as_[first_diff])
        print("     ⇒ 分叉前 %d 拍完全一致（控制量在此之前逐位相同）" % first_diff)


if __name__ == "__main__":
    main(sys.argv[1:] or ["build/diagnostics/cval-round1.log"])
