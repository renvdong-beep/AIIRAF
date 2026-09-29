#!/usr/bin/env python3
"""汇总某轮日志里的 LIFT 段轨迹（`PICK_LIFT_TRACE`）：载荷 / 指腹 / 腕部姿态随时间。

用法：python3 build/lift_trace_summary.py build/lift_stab_3.log [每 N 行打印一次]

为什么要它（2026-09-29 §11.23(48)）：抓取侧 7/8 通过，唯一失败是"腰部 regrasp 成功后，随后的 LIFT
载荷没跟着升起来"（lift_delta_m 0.000139，而腰部夹持力正常）。要在**一个段**里分清两种机制：
  (a) 载荷相对指腹在 LIFT 段渐增 ⇒ 面夹**滑脱**（改夹持/抬升力矩或抬升路径）；
  (b) 指腹上升而载荷不动、且姿态不变 ⇒ **指令/时序**问题（载荷根本没被带动）。
"""

import json
import sys
from pathlib import Path


def _quat_tilt_deg(q0, q1):
    import math
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def main():
    if len(sys.argv) < 2:
        raise SystemExit("用法：lift_trace_summary.py <日志> [打印间隔]")
    path = Path(sys.argv[1])
    every = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("PICK_LIFT_TRACE "):
            rows.append(json.loads(line.split(" ", 1)[1]))
    if not rows:
        print("没有 PICK_LIFT_TRACE（该轮可能没跑到抬升段，或日志被截断）")
        return 0
    first = rows[0]
    print("step   |载荷−指腹|水平  |载荷−指腹|3D   载荷z      指腹z      载荷倾角 腕部倾角 指腹跨度")
    for index, row in enumerate(rows):
        if index % every:
            continue
        payload = row["payload_pos_m"]
        pad = row["pad_mid_m"]
        dx, dy, dz = (payload[0] - pad[0], payload[1] - pad[1], payload[2] - pad[2])
        lateral = (dx * dx + dy * dy) ** 0.5
        total = (dx * dx + dy * dy + dz * dz) ** 0.5
        tilt = _quat_tilt_deg(first.get("payload_quat_wxyz", [1, 0, 0, 0]),
                              row.get("payload_quat_wxyz", [1, 0, 0, 0]))
        # ⚠ **必须同时看腕部**（§11.23(48)）：只有"载荷在转"分不清是"工具带着转"还是"载荷在夹口里滑"
        # —— 两者修法完全不同（前者改运动/姿态，后者改夹持力与抗转力矩）。
        wrist_tilt = _quat_tilt_deg(first.get("wrist_quat_wxyz", [1, 0, 0, 0]),
                                    row.get("wrist_quat_wxyz", [1, 0, 0, 0]))
        print("%-6s %-18.6f %-18.6f %-11.6f %-11.6f %-8.2f %-8.2f %.6f" % (
            row.get("step"), lateral, total, payload[2], pad[2], tilt, wrist_tilt,
            _norm(row.get("pad_span_m"))))
    last = rows[-1]
    payload, pad = last["payload_pos_m"], last["pad_mid_m"]
    print("\n汇总：样本 %d 段长(step) %s→%s" % (
        len(rows), first.get("step"), last.get("step")))
    print("  水平 |载荷−指腹|：首 %.6f → 末 %.6f（净变 %+.6f）" % (
        _lateral(first), _lateral(last), _lateral(last) - _lateral(first)))
    print("  竖直 载荷−指腹：首 %+.6f → 末 %+.6f" % (
        first["payload_pos_m"][2] - first["pad_mid_m"][2],
        last["payload_pos_m"][2] - last["pad_mid_m"][2]))
    print("  指腹 z 升 %.6f；载荷 z 升 %.6f" % (
        last["pad_mid_m"][2] - first["pad_mid_m"][2],
        last["payload_pos_m"][2] - first["payload_pos_m"][2]))
    print("  载荷倾角 首帧→末帧 %.2f°" % _quat_tilt_deg(
        first.get("payload_quat_wxyz", [1, 0, 0, 0]), last.get("payload_quat_wxyz", [1, 0, 0, 0])))
    return 0


def _norm(vector):
    if not vector:
        return float("nan")
    return (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5


def _lateral(row):
    payload, pad = row["payload_pos_m"], row["pad_mid_m"]
    dx, dy = payload[0] - pad[0], payload[1] - pad[1]
    return (dx * dx + dy * dy) ** 0.5


if __name__ == "__main__":
    raise SystemExit(main())
