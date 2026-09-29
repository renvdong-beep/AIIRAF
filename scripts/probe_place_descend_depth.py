#!/usr/bin/env python3
"""放置段**下降深度**核算：把声明的 descend 目标在联合模型上 FK 出来，与实测末端对齐。

为什么必须（2026-09-29 §11.25）：`PLACE_TRACE` 实测下降段结束时"载荷最低点 − 实测承载面"
= **+0.0205 ~ +0.0222 m**（6 轮一致）⇒ 方块是被**放下 21 mm 落差**、不是被放到承载面上；
落点因此双峰（托盘内 / 台面）。要修就得先分清这 21 mm 里哪部分是：
  · **设计落差**（构建期 place 航点本身就悬空 / clearance 声明给的），还是
  · **执行欠到位**（臂没走到声明的 descend 目标）。

做法（纯几何，不做动力学）：
  1. 取联合报告 `gripper.place_descend_positions`（构建期解出的关节解）⇒ 在**联合模型**上 FK；
  2. 读出该位姿下的**指腹中点**（双侧 pad geom 中点 —— 与后端/判据同一几何口径）；
  3. 对照三处高度：报告 `place_targets` 的**名义承载面**、关键帧下**实测**托盘承载面、
     以及 trace 里实测的指腹中点 z ⇒ 三者相减即把 21 mm 分解。

用法：PYTHONPATH=src python3 scripts/probe_place_descend_depth.py
      [--report build/scenes/handoff_lab/handoff_lab_joint.json] [--model ...]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]


def _geom_id(model, name):
    ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return int(ident) if ident >= 0 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--model", default=None)
    parser.add_argument("--key", type=int, default=0)
    args = parser.parse_args()

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    model_path = args.model or str(REPO / report["output"])
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    gripper = report.get("gripper") or {}
    targets = (report.get("place_targets") or {}).get("targets") or []
    if not targets:
        print("报告里没有 place_targets（本场景未声明接收体）")
        return 2
    target = targets[0]
    nominal_pose = target.get("nominal_pose_m")
    size = target.get("size_m") or [None, None, None]
    nominal_top = (float(nominal_pose[2]) + float(size[2]) / 2.0) if nominal_pose else None

    left = _geom_id(model, gripper.get("left_finger_geom") or "")
    right = _geom_id(model, gripper.get("right_finger_geom") or "")
    if left is None or right is None:
        print("模型里找不到声明的指腹 geom：%r / %r"
              % (gripper.get("left_finger_geom"), gripper.get("right_finger_geom")))
        return 2

    # 实测托盘承载面：关键帧位形下托盘体的几何顶点最高点（承载面 = 该 geom 顶面）
    mujoco.mj_resetDataKeyframe(model, data, args.key)
    mujoco.mj_forward(model, data)
    tray_geom = _geom_id(model, target.get("geom") or "")
    measured_top = None
    if tray_geom is not None:
        data_id = int(model.geom_dataid[tray_geom])
        if data_id >= 0:
            start = int(model.mesh_vertadr[data_id])
            count = int(model.mesh_vertnum[data_id])
            verts = model.mesh_vert[start:start + count]
            rotated = verts @ data.geom_xmat[tray_geom].reshape(3, 3).T
            measured_top = float((rotated[:, 2] + float(data.geom_xpos[tray_geom][2])).max())
        else:
            measured_top = (float(data.geom_xpos[tray_geom][2])
                            + float(model.geom_rbound[tray_geom]))

    rows = {}
    for phase in ("place_above_positions", "place_descend_positions", "place_retreat_positions"):
        positions = {str(k): float(v) for k, v in (gripper.get(phase) or {}).items()}
        if not positions:
            continue
        mujoco.mj_resetDataKeyframe(model, data, args.key)
        for name, value in positions.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                data.qpos[int(model.jnt_qposadr[jid])] = value
        mujoco.mj_forward(model, data)
        pad_mid = (np.asarray(data.geom_xpos[left], dtype=float)
                   + np.asarray(data.geom_xpos[right], dtype=float)) / 2.0
        rows[phase] = {"pad_mid_z_m": float(pad_mid[2]),
                       "pad_mid_xy_m": [float(pad_mid[0]), float(pad_mid[1])],
                       "joints_declared": len(positions)}
        # 载荷最低点 = 指腹中点 z + （实测的 载荷最低点−指腹中点）偏移，从 trace 里取
    print("名义承载面（报告 place_targets）      = %s m" % nominal_top)
    print("实测托盘承载面（关键帧 FK，顶点最高） = %s m" % measured_top)
    if measured_top is not None and nominal_top is not None:
        print("  差（实测 − 名义）                   = %.6f m（托盘比名义低这么多 ⇒ 狗停到位后托盘下沉/漂移）"
              % (measured_top - nominal_top))
    for phase, row in rows.items():
        print("%-26s 指腹中点 z = %.6f m  xy = [%.6f, %.6f]  关节 %d 个"
              % (phase, row["pad_mid_z_m"], row["pad_mid_xy_m"][0], row["pad_mid_xy_m"][1],
                 row["joints_declared"]))
    print()
    print("对照（trace 实测，见 build/joint-stability/*.log 的 after_descend 行）：")
    print("  after_descend 实测：pad_mid_z 0.408648 / 0.408525 / 0.408445 …；"
          "载荷最低点 − 指腹中点 = −0.0437 ~ −0.0460")
    print("  ⇒ 若声明的 descend 指腹中点 z 与实测差 > 1 mm，则是**执行欠到位**；")
    print("     若一致，则 21 mm 落差是**声明/几何本身**的（需要改 place 航点或 clearance）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
