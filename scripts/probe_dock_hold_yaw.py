"""停靠**全时段**误差轨迹判读（2026-10-05 §11.86）。

输入：含 `DOCK_TRACE` 的日志（`scripts/probe_dock_hold_trace.sh` 产出，IRAF_DEBUG_DOCK=1）。
判读量（**与判据完全同一条测量路径**：frame_pose/body_pose + pose_error）：
  · 接近段（1 s 抽样）：平移 / 偏航误差的时间轨迹 —— 看有无大过冲；
  · 到位拍（首条 vx==0 的记录）：此刻的平移 / 偏航误差 = "发零"触发点；
  · 保持窗（0.5 s 抽样，按**仿真时钟**）：从发零到返回的误差增长；
  · 漂移 = 末态 − 到位拍，以及换算出来的平均漂移率；
  · 预算核算：`控制容差 + 漂移 ≤ 验收`？超出多少？

列定义：DOCK_TRACE samples = [elapsed(相位钟), dx, dy, yaw_err, cmd_vx, cmd_wz, body_yaw, sim_time]
只读日志，不跑仿真。
用法：python3 scripts/probe_dock_hold_yaw.py build/diagnostics/dockhold-round*.log
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

ACCEPT_POS_M = 0.030
ACCEPT_YAW_DEG = 2.0


def _parse(path):
    rows = []
    for line in pathlib.Path(path).read_text(errors="replace").splitlines():
        if line.startswith("DOCK_TRACE "):
            try:
                rows.append(json.loads(line[len("DOCK_TRACE "):]))
            except json.JSONDecodeError:
                pass
    return rows


def _traj(trace):
    out = []
    for row in trace:
        elapsed, dx, dy, yaw, vx, _wz, _byaw = row[0], row[1], row[2], row[3], row[4], row[5], row[6]
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


def main(argv):
    for path in argv:
        for row in _parse(path):
            _report(path, row.get("samples") or [], row.get("station"))


if __name__ == "__main__":
    main(sys.argv[1:] or ["build/diagnostics/dockhold-round1.log"])
