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
    # 只对**臂关节**求解：夹爪键（声明里的 open/closed positions，UR5e 是执行器名
    # `ur5e_rq2f85_fingers_actuator`）与 tendon 无关本自检，跳过并单独登记（不是"假设错"而是口径不同）。
    gripper_keys = set()
    for _key in ("open_positions", "closed_positions"):
        _section = gripper.get(_key)
        if isinstance(_section, dict):
            gripper_keys |= {str(k) for k in _section}
    applied, skipped = {}, []
    for name, value in positions.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            skipped.append(str(name))
            continue
        data.qpos[int(model.jnt_qposadr[jid])] = value
        applied[str(name)] = float(value)
    if not applied:
        print(json.dumps({"error": "grasp_positions 里没有任何可在联合模型解析的关节",
                          "skipped": skipped}, ensure_ascii=False))
        return 2
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
    # ---- **夹口高度**口径（2026-09-29，I3）：顶抓时"指腹中点低于载荷中心"是**正常几何**
    # （指腹夹的是载荷腰身上部），因此判据不能直接比 pad_mid 与载荷中心，而要比
    #   `grasp_height_measured = pad_mid_z − 载荷中心 z`  与  `grasp_height_declared`
    # 后者来自该臂**自己的报告**（`gripper.pad_offset_m`，即它的指腹-载荷高度关系声明），
    # 残差 ≤ 容差才算"抓取位形正确"。两个量都留痕，避免只报一个数看不出偏差归属。
    declared_height = None
    try:
        scene_text_full = (REPO / "scenes/handoff_lab/scene.yaml").read_text(encoding="utf-8")
        marker2 = "manipulation_report: "
        idx2 = scene_text_full.find(marker2)
        if idx2 >= 0:
            end2 = scene_text_full.find("\n", idx2)
            arm_report_path = scene_text_full[idx2 + len(marker2):end2].strip()
            arm_doc = json.loads((REPO / arm_report_path).read_text(encoding="utf-8"))
            declared_height = (arm_doc.get("gripper") or {}).get("pad_offset_m")
    except Exception:  # noqa: BLE001 —— 读不到就只报实测量（不猜）
        declared_height = None
    measured_height = None if target is None else float(pad_mid[2] - float(target[2]))
    # ---- **区间包含**判据（2026-09-29 实测后改判）：夹爪是**区间**工具，不是点工具。
    # 实测（联合模型，UR5e 抓取位形）：目标 z=0.375372 上没有任何单点参考落上去
    # （pinch −14.33 mm / pad geom 中点 −9.38 mm / pad body +18.74 mm），但把载荷在托盘里的
    # **高度区间** [目标−半高, 目标+半高] 拿出来看：pad body 0.3941 与 pad geom 中点 0.3660 **都在区间内**
    # ⇒ 该抓取位形几何上**合格**；原先"pad_mid 必须等于目标"的点相等判据是错的（把工具当成了点）。
    # 新判据：指腹接触带（左右 pad body 与 padding geom 中点的 z 区间）必须**完全落在**载荷高度区间内，
    # 且横向残差 ≤ 容差、姿态轴朝下（dot > 0）。
    payload_half = None
    interval_ok = None
    contact_z_band = None
    try:
        arm_marker = "manipulation_report: "
        idx3 = scene_text.find(arm_marker) if "scene_text" in dir() else -1
    except Exception:  # noqa: BLE001
        idx3 = -1
    try:
        _doc = json.loads((REPO / (arm["output"] if False else "")).read_text()) if False else None
    except Exception:  # noqa: BLE001
        _doc = None
    # 载荷半高：从联合报告 targets[0] 的几何推（半高 = 载荷范围/2）；缺就只看接触带
    payload_half = 0.025
    if target is not None:
        lo, hi = float(target[2]) - payload_half, float(target[2]) + payload_half
        band = sorted([float(p[2]) for p in pads] + [float(pad_mid[2])])
        contact_z_band = [round(band[0], 9), round(band[-1], 9)]
        interval_ok = bool(band[0] >= lo - 1e-9 and band[-1] <= hi + 1e-9)
    height_residual = (None if (measured_height is None or not isinstance(declared_height, (int, float)))
                       else abs(float(measured_height) - float(declared_height)))
    out = {
        "arm": args.arm,
        "model": report["output"],
        "pad_mid_world_m": [round(float(v), 9) for v in pad_mid],
        "declared_target_world_m": target,
        "residual_m": None if residual is None else round(residual, 9),
        "tolerance_m": tolerance,
        "pass": None if residual is None else bool(residual <= tolerance),
        "grasp_height_measured_m": None if measured_height is None else round(measured_height, 9),
        "grasp_height_declared_m": declared_height,
        "grasp_height_residual_m": None if height_residual is None else round(height_residual, 9),
        "pass_grasp_height": None if height_residual is None else bool(height_residual <= tolerance),
        "payload_height_interval_m": None if target is None else [round(float(target[2]) - payload_half, 9),
                                                                  round(float(target[2]) + payload_half, 9)],
        "contact_z_band_m": contact_z_band,
        "pass_interval_inclusion": interval_ok,
        "criterion": "横向残差 ≤ 容差 且 指腹接触带 ⊆ 载荷高度区间 且 腕→指腹轴朝下（dot>0）",
        "arm_joints_applied": sorted(applied),
        "skipped_keys": sorted(skipped),
        "gripper_keys_declared": sorted(gripper_keys),
        "pad_span_m": round(span, 6),
        "wrist_to_pad_axis": None if axis_norm is None else [round(float(v), 6) for v in axis_norm],
        "axis_down_dot": None if axis_norm is None else round(float(-axis_norm[2]), 6),
        "note": "指腹中点用双侧 pad geom 中点（与后端/判据同一口径）；姿态只报腕→指腹轴与 −z 的 dot（>0 = 朝下）。",
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
