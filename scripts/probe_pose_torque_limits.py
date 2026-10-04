#!/usr/bin/env python3
"""位形处的**力矩/限幅对账**（2026-09-30 §11.65）。

背景（§11.64）：同一个位形**静态保持残余 1.35e-7 rad**，可**回放**却稳定不到 0.36 rad。
本探针回答一个具体问题：**该位形的所需关节力矩是否超出执行器的 force/ctrl 限幅**
（超限 ⇒ 伺服永远到不了，表现为"回放跟不上"，且与"静态保持能过"不矛盾——静态保持口径可能
给的是同一套 ctrl，需要看数字）。

口径：把臂设到目标位形（载体按需摆到站位帧）→ `mj_forward` → 逐关节打印
  · `qfrc_bias`（重力/科氏力矩，= 需要被平衡的力矩）
  · 执行器 `gear`、`forcerange`、`ctrlrange`、`gainprm[0]`（位置增益）
  · 由纯 PD 静差公式推出的**静态残余** `|qfrc_bias| / gainprm[0]` 与其相对 `forcerange` 的比例
只读，不改声明/产物。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_pose_torque_limits.py [位形名]
    位形名 ∈ {pick_grasp, pick_lift, place_above, place_descend}（缺省全测）
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco

REPO = Path(__file__).resolve().parents[1]
JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
JOINT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
PREFIX = "ur5e_"
CARRIER_FRAME = "handoff_station_frame_b"
POSES = (("pick_grasp", "grasp_positions"), ("pick_lift", "lift_positions"),
         ("place_above", "place_above_positions"), ("place_descend", "place_descend_positions"))


def main(argv):
    wanted = argv[1] if len(argv) > 1 else None
    doc = json.loads(JOINT_JSON.read_text())
    gripper = ((doc["manipulation"]["per_robot"]["ur5e"]) or {}).get("gripper") or {}
    model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    data = mujoco.MjData(model)

    # 载体摆到站位帧（与 §11.63 的构建期口径一致）
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, CARRIER_FRAME)
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "unitree_go2_trunk")
    free_q = None
    if site >= 0 and trunk >= 0:
        rid = int(model.body_rootid[trunk])
        jid = int(model.body_jntadr[rid]) if int(model.body_jntnum[rid]) else -1
        if jid >= 0:
            free_q = int(model.jnt_qposadr[jid])
    print("[载体] frame=%s site=%s free_qpos=%s" % (CARRIER_FRAME, site, free_q))

    for label, key in POSES:
        if wanted and wanted != label:
            continue
        section = gripper.get(key) or {}
        data.qpos[:] = 0.0
        if site >= 0 and free_q is not None:
            data.qpos[free_q:free_q + 3] = model.site_pos[site]
            data.qpos[free_q + 3:free_q + 7] = model.site_quat[site]
        for name, value in section.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if jid >= 0:
                data.qpos[int(model.jnt_qposadr[jid])] = float(value)
        mujoco.mj_forward(model, data)
        print("== %s (%s)" % (label, key))
        print("   %-30s %10s %12s %12s %10s %10s" % ("关节/执行器", "qfrc_bias", "forcerange", "ctrlrange", "gainparm0", "|τ|/gain"))
        for name, value in sorted(section.items()):
            if "actuator" in str(name):
                continue
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if jid < 0:
                continue
            aid = next((i for i in range(int(model.nu))
                        if int(model.actuator_trnid[i, 0]) == jid), -1)
            dof = int(model.jnt_dofadr[jid])
            bias = float(data.qfrc_bias[dof])
            if aid < 0:
                print("   %-30s %10.4f %12s %12s %10s %10s" % (name, bias, "-", "-", "-", "-"))
                continue
            gain = float(model.actuator_gainprm[aid, 0])
            fr = model.actuator_forcerange[aid]
            cr = model.actuator_ctrlrange[aid]
            ratio = abs(bias) / gain if gain else float("nan")
            print("   %-30s %10.4f %12s %12s %10.4f %10.4f"
                  % (name, bias, "[%.1f,%.1f]" % (fr[0], fr[1]),
                     "[%.3f,%.3f]" % (cr[0], cr[1]), gain, ratio))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
