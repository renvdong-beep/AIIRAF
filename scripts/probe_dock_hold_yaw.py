"""停靠**全时段**误差轨迹 + 冻结延迟判读（2026-10-05 §11.86）。

输入：含 `DOCK_TRACE` / `DOCK_HALT` 的日志（`scripts/probe_dock_hold_trace.sh` 产出）。
判读量（`DOCK_TRACE` 与判据**完全同一条测量路径**：frame_pose/body_pose + pose_error）：
  · 接近段（1 s 抽样）：平移 / 偏航误差轨迹 —— 看有无大过冲；
  · 到位拍（首条 vx==0 的记录）：此刻的误差 = "发零"触发点（实测偏航恒为 −1.92°，贴着控制容差）；
  · 保持窗（0.5 s 抽样，按**仿真钟**）：到位后的回摆与回收；
  · 漂移 = 末态 − 到位拍，及换算出的平均漂移率；预算核算：`控制容差 + 漂移 ≤ 验收`？
`DOCK_HALT` 汇总：**冻结延迟**（zero_command_since_s → frozen_elapsed_s）与末态的相关系数
  —— 用来判"到位后回摆 ≈ 步态速度地板 × 冻结延迟"是否成立（成立才谈得上改站定机制）。

列定义：
  DOCK_TRACE samples = [elapsed(相位钟), dx, dy, yaw_err, cmd_vx, cmd_wz, body_yaw, sim_time]
  DOCK_HALT = {station, halt_declared, trot_period_s, reached_s, zero_command_since_s,
               frozen_elapsed_s, freeze_delay_s, frozen, final_pos_m, final_yaw_deg,
               pass_pos, pass_yaw, final_speed_mps}
只读日志，不跑仿真。
用法：python3 scripts/probe_dock_hold_yaw.py build/diagnostics/dockhold-<标签>-round*.log
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

ACCEPT_POS_M = 0.030
ACCEPT_YAW_DEG = 2.0


def _parse(path, prefix):
    rows = []
    for line in pathlib.Path(path).read_text(errors="replace").splitlines():
        if line.startswith(prefix):
            try:
                rows.append(json.loads(line[len(prefix):]))
            except json.JSONDecodeError:
                pass
    return rows


def _traj(trace):
    out = []
    for row in trace:
        elapsed, dx, dy, yaw, vx = row[0], row[1], row[2], row[3], row[4]
        sim = row[7] if len(row) > 7 else None
        out.append({"elapsed": elapsed, "pos": math.hypot(dx, dy), "yaw": math.degrees(yaw),
                    "vx": vx, "sim": sim})
    return out


def _report(path, trace, station):
    pts = _traj(trace)
    if not pts:
        return
    latch = next((index for index, item in enumerate(pts) if item["vx"] == 0.0), None)
    print("=" * 74)
    print("[%s] station=%s 记录 %d 拍" % (pathlib.Path(path).name, station, len(pts)))

    print("  接近段（按相位钟 1 s 抽样；t=仿真钟, pos=平移误差, yaw=偏航误差）")
    shown = 0.0
    for item in pts:
        if latch is not None and item is pts[latch]:
            break
        if item["elapsed"] + 1e-9 >= shown:
            shown += 1.0
            print("    t=%7.2f s  pos=%9.6f m  yaw=%9.4f°"
                  % (item["sim"] if item["sim"] is not None else item["elapsed"],
                     item["pos"], item["yaw"]))

    if latch is None:
        print("  ⚠ 未发零（整段都在接近；应为超时失败）")
        item = pts[-1]
        print("    末拍 pos=%.6f m yaw=%.4f°" % (item["pos"], item["yaw"]))
        return
    latch_pt = pts[latch]
    print("  到位拍（发零）: 相位钟 %.4f s  仿真钟 %s  pos=%.6f m  yaw=%.4f°"
          % (latch_pt["elapsed"], latch_pt["sim"], latch_pt["pos"], latch_pt["yaw"]))
    print("    到位拍余量：平移 %+0.6f m（验收 %.3f）/ 偏航 %+0.4f°（验收 %.1f）"
          % (ACCEPT_POS_M - latch_pt["pos"], ACCEPT_POS_M,
             ACCEPT_YAW_DEG - abs(latch_pt["yaw"]), ACCEPT_YAW_DEG))

    hold = pts[latch:]
    print("  保持窗（按仿真钟 0.5 s 抽样，共 %d 拍）" % len(hold))
    shown = None
    for item in hold:
        key = None if item["sim"] is None else round(item["sim"] * 2.0) / 2.0
        if key is not None and key != shown:
            shown = key
            print("    t=%7.2f s  pos=%9.6f m  yaw=%9.4f°  |dpos|=%+0.6f m |dyaw|=%+0.4f°"
                  % (item["sim"], item["pos"], item["yaw"],
                     item["pos"] - latch_pt["pos"], abs(item["yaw"]) - abs(latch_pt["yaw"])))

    last = pts[-1]
    span = (last["sim"] - latch_pt["sim"]) if (last["sim"] is not None
                                              and latch_pt["sim"] is not None) else None
    print("  ---- 预算核算 ----")
    print("    末态: pos=%.6f m（验收 %.3f，余量 %+0.6f）/ yaw=%.4f°（验收 %.1f，余量 %+0.4f）"
          % (last["pos"], ACCEPT_POS_M, ACCEPT_POS_M - last["pos"],
             last["yaw"], ACCEPT_YAW_DEG, ACCEPT_YAW_DEG - abs(last["yaw"])))
    print("    漂移 = 末态 − 到位: |dpos|=%+0.6f m  |dyaw|=%+0.4f°（保持窗 %s）"
          % (last["pos"] - latch_pt["pos"], abs(last["yaw"]) - abs(latch_pt["yaw"]),
             ("%.2f s 仿真" % span) if span is not None else "未知"))
    if span and span > 0.0:
        print("    平均漂移率: %.4f mm/s / %.5f °/s"
              % (abs(last["pos"] - latch_pt["pos"]) * 1000.0 / span,
                 abs(abs(last["yaw"]) - abs(latch_pt["yaw"])) / span))
    peak_yaw = max(hold, key=lambda item: abs(item["yaw"]))
    peak_pos = max(hold, key=lambda item: item["pos"])
    print("    保持窗峰值: |yaw| 最大 %.4f°（t=%s）/ pos 最大 %.6f m（t=%s）"
          % (abs(peak_yaw["yaw"]), peak_yaw["sim"], peak_pos["pos"], peak_pos["sim"]))


def _pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx == 0.0 or dy == 0.0:
        return None
    return num / (dx * dy)


def _halt_summary(records):
    if not records:
        print("（无 DOCK_HALT 记录）")
        return
    print("=" * 74)
    print("[冻结延迟 ↔ 末态] %d 个样本" % len(records))
    print("  %-14s %-11s %-12s %-12s %-6s %-6s" % ("station", "延迟/s", "末态pos/m",
                                                     "末态yaw/°", "pos过", "yaw过"))
    for item in records:
        delay = item.get("freeze_delay_s")
        print("  %-14s %-11s %-12.6f %-12.4f %-6s %-6s"
              % (item.get("station"), "None" if delay is None else "%.4f" % delay,
                 item.get("final_pos_m") if item.get("final_pos_m") is not None else float("nan"),
                 item.get("final_yaw_deg") if item.get("final_yaw_deg") is not None else float("nan"),
                 item.get("pass_pos"), item.get("pass_yaw")))
    triples = [(item["freeze_delay_s"], item["final_pos_m"], abs(item["final_yaw_deg"]))
               for item in records
               if item.get("freeze_delay_s") is not None
               and item.get("final_pos_m") is not None
               and item.get("final_yaw_deg") is not None]
    if len(triples) >= 3:
        ds, poss, yaws = zip(*triples)
        print("  延迟 min/max = %.4f / %.4f s" % (min(ds), max(ds)))
        print("  r(延迟, 末态pos)   = %s" % _pearson(list(ds), list(poss)))
        print("  r(延迟, |末态yaw|) = %s" % _pearson(list(ds), list(yaws)))


def main(argv):
    records = []
    for path in argv:
        for row in _parse(path, "DOCK_TRACE "):
            _report(path, row.get("samples") or [], row.get("station"))
        records.extend(_parse(path, "DOCK_HALT "))
    _halt_summary(records)


if __name__ == "__main__":
    main(sys.argv[1:] or ["build/diagnostics/dockhold-run-round1.log"])
