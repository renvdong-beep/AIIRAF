#!/usr/bin/env python3
"""UR5e 在联合世界里的**抓取位形自检**（I3 第一步，纯几何）。

口径：取联合报告 `manipulation.per_robot[ur5e].gripper.grasp_positions`（构建期按本场景解出的关节解），
在**联合模型**上 FK，量「双侧指腹中点」与「声明目标点」的残差，以及腕→指腹轴是否朝下（抓取姿态）。

为什么必须（2026-09-29）：UR5e 的目标是**声明覆盖**给出来的（B 站托盘里的载荷，
`robots[].reference_solver.target_override_world_m`）⇒ 解出来的位形到底对不对，必须用同一模型 FK 自证，
而不是相信"构建期没报错"。判据：残差 ≤ 声明容差（`acceptance.pose_tolerance_m`，本场景 0.005）。

用法：PYTHONPATH=src python3 scripts/probe_ur5e_joint_grasp_check.py
      [--report build/scenes/handoff_lab/handoff_lab_joint.json] [--arm ur5e]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--arm", default="ur5e")
    parser.add_argument("--tolerance", type=float, default=None)
    args = parser.parse_args()

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    per_robot = ((report.get("manipulation") or {}).get("per_robot") or {})
    entry = per_robot.get(args.arm) or {}
    if not entry or entry.get("resolved") is not True:
        print(json.dumps({"error": "per_robot[%s] 未解（resolved != true）" % args.arm,
                          "detail": entry.get("error")}, ensure_ascii=False))
        return 2
    gripper = entry.get("gripper") or {}
    positions = {str(k): float(v) for k, v in (gripper.get("grasp_positions") or {}).items()}
    if not positions:
        print(json.dumps({"error": "per_robot[%s].gripper 缺 grasp_positions" % args.arm},
                         ensure_ascii=False))
        return 2
    model = mujoco.MjModel.from_xml_path(str(REPO / report["output"]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for name, value in positions.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            print(json.dumps({"error": "联合模型里没有关节 %r" % name}, ensure_ascii=False))
            return 2
        data.qpos[int(model.jnt_qposadr[jid])] = value
    mujoco.mj_forward(model, data)

    pads = []
    for key in ("left_finger_geom", "right_finger_geom"):
        ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(gripper.get(key) or ""))
        if ident < 0:
            print(json.dumps({"error": "联合模型里没有指腹 geom %r" % gripper.get(key)},
                             ensure_ascii=False))
            return 2
        pads.append(np.asarray(data.geom_xpos[ident], dtype=float))
    pad_mid = (pads[0] + pads[1]) / 2.0
    span = float(np.linalg.norm(pads[0] - pads[1]))
    wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(gripper.get("wrist_body") or ""))
    axis = (pad_mid - np.asarray(data.xpos[wrist_id], dtype=float)) if wrist_id >= 0 else None
    axis_norm = (axis / np.linalg.norm(axis)) if axis is not None and np.linalg.norm(axis) > 1e-9 else None

    # 声明目标（世界系）：从场景声明读；报告里没有就只报 FK 结果
    target = None
    scene_text = (REPO / "scenes/handoff_lab/scene.yaml").read_text(encoding="utf-8")
    marker = "target_override_world_m: ["
    idx = scene_text.find(marker)
    if idx >= 0:
        end = scene_text.find("]", idx)
        target = [float(v) for v in scene_text[idx + len(marker):end].split(",")]
    tolerance = args.tolerance
    if tolerance is None:
        tolerance = 0.005
    residual = None if target is None else float(np.linalg.norm(pad_mid - np.asarray(target)))
    out = {
        "arm": args.arm,
        "model": report["output"],
        "pad_mid_world_m": [round(float(v), 9) for v in pad_mid],
        "declared_target_world_m": target,
        "residual_m": None if residual is None else round(residual, 9),
        "tolerance_m": tolerance,
        "pass": None if residual is None else bool(residual <= tolerance),
        "pad_span_m": round(span, 6),
        "wrist_to_pad_axis": None if axis_norm is None else [round(float(v), 6) for v in axis_norm],
        "axis_down_dot": None if axis_norm is None else round(float(-axis_norm[2]), 6),
        "note": "指腹中点用双侧 pad geom 中点（与后端/判据同一口径）；姿态只报腕→指腹轴与 −z 的 dot（>0 = 朝下）。",
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
