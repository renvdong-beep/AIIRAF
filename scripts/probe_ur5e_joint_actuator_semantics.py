#!/usr/bin/env python3
"""UR5e 在**联合模型**里的执行器语义核对（2026-09-30 §11.31，为重力前馈判死）。

背景：给本臂声明 `feedforward_entry: build_reference_feedforward`（共享实现，公式 `增量 = τ_g / kp`）
后，构建期在联合模型上重算**被静态保持门禁拦下**：最大关节误差 0.049653448 rad（限 0.001），
最差 elbow / wrist_1。Piper 侧同一公式成立 ⇒ 差异必在本臂执行器的**语义**上。

本探针只读模型事实（不跑场景、不动控制器），逐关节打印：
  · 执行器类型 / 增益类型（`gaintype`/`gainprm`）/ 偏置（`biastype`/`biasprm`）
  · `gear`（传动比）：公式用的是 kp，而实际出力 = gear·gain·(ctrl − qpos) ⇒ 漏掉 gear 会整比失真
  · `forcerange` / `ctrlrange`：若重力矩超过行程，**任何**前馈都不可能把静差压到 0.001 rad
  · 参考姿态下的重力矩 τ_g（`qfrc_bias` 的对应自由度）与该关节"可补偿的静差上限"
    = |τ_g| / (gear·kp)，以及 forcerange 是否够

用法：PYTHONPATH=src python3 scripts/probe_ur5e_joint_actuator_semantics.py
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
#: MuJoCo 增益/偏置类型的可读名（零空间修正与位置伺服的口径判定要用）。
GAIN_TYPES = {int(mujoco.mjtGain.mjGAIN_FIXED): "fixed",
              int(mujoco.mjtGain.mjGAIN_AFFINE): "affine",
              int(mujoco.mjtGain.mjGAIN_MUSCLE): "muscle",
              int(mujoco.mjtGain.mjGAIN_USER): "user"}
BIAS_TYPES = {int(mujoco.mjtBias.mjBIAS_NONE): "none",
              int(mujoco.mjtBias.mjBIAS_AFFINE): "affine",
              int(mujoco.mjtBias.mjBIAS_MUSCLE): "muscle",
              int(mujoco.mjtBias.mjBIAS_USER): "user"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--arm", default="ur5e")
    parser.add_argument("--phase", default="grasp",
                        choices=("home", "approach", "grasp", "lift"))
    parser.add_argument("--hold-ms", type=int, default=4000)
    parser.add_argument("--passes", type=int, default=4)
    parser.add_argument("--output", default="build/diagnostics/ur5e-actuator-semantics.json")
    args = parser.parse_args()

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    entry = (((report.get("manipulation") or {}).get("per_robot") or {}).get(args.arm) or {})
    gripper = entry.get("gripper") or {}
    name_map = entry.get("name_map") or {}
    model = mujoco.MjModel.from_xml_path(str(REPO / report["output"]))
    data = mujoco.MjData(model)

    def model_name(text):
        return name_map.get(str(text), str(text))

    # 参考姿态（指定相位）摆好，量该位形下的重力矩
    arm_positions = {}
    for key, value in (gripper.get("%s_positions" % args.phase) or {}).items():
        jid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, model_name(key)))
        if jid >= 0:
            arm_positions[str(jid)] = float(value)
    for jid_text, value in arm_positions.items():
        data.qpos[int(model.jnt_qposadr[int(jid_text)])] = value
    mujoco.mj_forward(model, data)

    rows = []
    for jid_text, value in sorted(arm_positions.items(), key=lambda item: int(item[0])):
        joint_id = int(jid_text)
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        # 驱动该关节的执行器：按 actuator_trnid 反查（与构建期同一口径）
        matches = [index for index in range(int(model.nu))
                   if int(model.actuator_trntype[index]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                   and int(model.actuator_trnid[index, 0]) == joint_id]
        if not matches:
            rows.append({"joint": joint_name, "error": "没有直接驱动该关节的执行器"})
            continue
        actuator = matches[0]
        dof = int(model.jnt_dofadr[joint_id])
        torque = float(data.qfrc_bias[dof])
        gear = float(model.actuator_gear[actuator, 0])
        gain = np.asarray(model.actuator_gainprm[actuator], dtype=float)
        bias = np.asarray(model.actuator_biasprm[actuator], dtype=float)
        kp = float(gain[0])
        if int(model.actuator_biastype[actuator]) == int(mujoco.mjtBias.mjBIAS_AFFINE):
            kp = abs(float(bias[1]))          # affine: force = kp*(ctrl − qpos) − kv*qvel
        effective = gear * kp
        forcerange = [float(v) for v in model.actuator_forcerange[actuator]]
        ctrlrange = [float(v) for v in model.actuator_ctrlrange[actuator]]
        rows.append({
            "joint": joint_name,
            "actuator": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator),
            "gaintype": GAIN_TYPES.get(int(model.actuator_gaintype[actuator]), "?"),
            "gainprm": [float(v) for v in gain[:3]],
            "biastype": BIAS_TYPES.get(int(model.actuator_biastype[actuator]), "?"),
            "biasprm": [float(v) for v in bias[:3]],
            "gear": gear,
            "gear_vector": [float(v) for v in model.actuator_gear[actuator]],
            "kp_used_by_formula": kp,
            "kp_effective_with_gear": effective,
            "forcerange": forcerange,
            "ctrlrange": ctrlrange,
            "qpos_rad": round(float(data.qpos[int(model.jnt_qposadr[joint_id])]), 9),
            "qfrc_bias_n_m": round(torque, 9),
            # 公式假设：静差 = τ_g / (gear·kp)；这就是"单靠前馈能压到的下限"
            "static_error_lower_bound_rad": (round(abs(torque) / effective, 9) if effective > 0 else None),
            "required_ctrl_offset_rad": (round(torque / effective, 9) if effective > 0 else None),
            "torque_within_forcerange": (None if forcerange[0] == forcerange[1] == 0
                                         else abs(torque) <= max(abs(forcerange[0]), abs(forcerange[1]))),
            "gear_is_unity": abs(gear - 1.0) < 1e-12,
        })
    worst = max((row.get("static_error_lower_bound_rad") or 0.0) for row in rows)
    # --- 判死"迭代为什么停在 5× 下限"（2026-09-30 §11.32(c)）---
    # 前馈的实际作用是把 `ctrl = 目标角 + offset` 写进通道；若 `ctrl + offset` 越过 `ctrlrange`
    # （或目标角已贴住关节行程），前馈就**物理上写不进去** ⇒ 残余与迭代次数无关。
    headroom = []
    for row in rows:
        if "actuator" not in row:
            continue
        actuator = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, row["actuator"]))
        ctrlrange = [float(v) for v in model.actuator_ctrlrange[actuator]]
        offset = row.get("required_ctrl_offset_rad")
        target = row.get("qpos_rad")
        if offset is None or target is None:
            continue
        commanded = target + offset
        entry = {"joint": row["joint"], "target_rad": target, "offset_rad": offset,
                 "commanded_rad": round(commanded, 9), "ctrlrange": ctrlrange}
        if ctrlrange[0] != ctrlrange[1]:
            entry["commanded_within_ctrlrange"] = bool(ctrlrange[0] <= commanded <= ctrlrange[1])
            entry["headroom_rad"] = round(min(ctrlrange[1] - commanded, commanded - ctrlrange[0]), 9)
        else:
            entry["commanded_within_ctrlrange"] = None
        headroom.append(entry)
    # --- 保持仿真 + 逐关节饱和/夹断判定（2026-09-30 §11.33 下一步）---
    # 口径与 `iraf_core.kinematics.gravity_hold_ctrl` 一致：每轮**替换**补偿 c = τ_g(q)/kp，
    # ctrl = 声明角 + c；非臂通道（腱驱动夹爪）按声明值写 ctrl。跑完给出：
    #   目标 / 末轮补偿 / 静止角 / 残余 / 残余×kp（= 未被补偿掉的力矩）vs forcerange / ctrl+补偿 vs ctrlrange
    arm_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(text))
                 for text in arm_positions]
    rows_by_joint = {row["joint"]: row for row in rows}
    hold = mujoco.MjData(model)
    if int(getattr(model, "nkey", 0) or 0) > 0:
        mujoco.mj_resetDataKeyframe(model, hold, 0)
    for text, value in arm_positions.items():
        hold.qpos[int(model.jnt_qposadr[int(text)])] = value
    hold.qvel[:] = 0.0
    hold.qfrc_applied[:] = 0.0
    finger_hold = {}
    for key, value in (gripper.get("%s_positions" % args.phase) or {}).items():
        name = model_name(key)
        if int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) >= 0:
            finger_hold[name] = float(value)
    mujoco.mj_forward(model, hold)
    steps = max(1, int(round(float(args.hold_ms) / 1000.0 / float(model.opt.timestep))))
    trace, last_compensation = [], {}
    for attempt in range(1, int(args.passes) + 1):
        bias = np.asarray(hold.qfrc_bias, dtype=float)
        for name in arm_names:
            row = rows_by_joint[name]
            joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
            actuator = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, row["actuator"]))
            kp = float(row["kp_used_by_formula"])
            last_compensation[name] = float(bias[int(model.jnt_dofadr[joint_id])]) / kp
            hold.ctrl[actuator] = float(arm_positions[str(joint_id)]) + last_compensation[name]
        for name, value in finger_hold.items():
            actuator = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            hold.ctrl[actuator] = value
        for _ in range(steps):
            mujoco.mj_step(model, hold)
        residual = {}
        for name in arm_names:
            joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
            residual[name] = round(float(hold.qpos[int(model.jnt_qposadr[joint_id])])
                                   - float(arm_positions[str(joint_id)]), 9)
        trace.append({"pass": attempt,
                      "worst_residual_rad": max(abs(v) for v in residual.values()),
                      "residual_rad": residual})
    settled = {}
    for name in arm_names:
        row = rows_by_joint[name]
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        actuator = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, row["actuator"]))
        kp = float(row["kp_used_by_formula"])
        residual = trace[-1]["residual_rad"][name]
        compensation = last_compensation.get(name, 0.0)
        commanded = float(arm_positions[str(joint_id)]) + compensation
        ctrlrange = [float(v) for v in model.actuator_ctrlrange[actuator]]
        forcerange = [float(v) for v in model.actuator_forcerange[actuator]]
        torque_uncompensated = kp * residual
        settled[name] = {
            "target_rad": float(arm_positions[str(joint_id)]),
            "compensation_rad": round(compensation, 9),
            "commanded_rad": round(commanded, 9),
            "settled_rad": round(float(hold.qpos[int(model.jnt_qposadr[joint_id])]), 9),
            "residual_rad": residual,
            "residual_x_kp_n_m": round(torque_uncompensated, 6),
            "qfrc_bias_initial_n_m": row.get("qfrc_bias_n_m"),
            "forcerange": forcerange,
            "ctrlrange": ctrlrange,
            "commanded_outside_ctrlrange": (None if ctrlrange[0] == ctrlrange[1]
                                            else not (ctrlrange[0] <= commanded <= ctrlrange[1])),
            # 判读：残余×kp 就是"没能被伺服补掉的力矩"；它接近 forcerange ⇒ 饱和
            "saturation_ratio": (None if forcerange[0] == forcerange[1] == 0 else
                                 round(abs(torque_uncompensated)
                                       / max(abs(forcerange[0]), abs(forcerange[1])), 6)),
        }
    # 接触取证（2026-09-30 §11.33）：`qfrc_bias` **不含接触力** ⇒ 若该位形上有接触，
    # 前馈永远补不掉那一份（残余×kp 就是接触等效力矩）。这里把接触对与法向力打出来判死。
    contacts = []
    for index in range(int(hold.ncon)):
        contact = hold.contact[index]
        ids = (int(contact.geom1), int(contact.geom2))
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "<未命名#%d>" % gid
                 for gid in ids]
        bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                    int(model.geom_bodyid[gid])) or "<未命名>" for gid in ids]
        if not any(str(body).startswith(str(args.arm)) for body in bodies):
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, hold, index, force)
        contacts.append({"geoms": names, "bodies": bodies,
                         "dist_m": round(float(contact.dist), 9),
                         "normal_force_n": [round(float(v), 6) for v in force[:3]]})
    # 被动力量取证（2026-09-30 §11.33）：`qfrc_bias` = 重力+科氏，**不含** `qfrc_passive`
    # （关节 stiffness/damping 与弹簧）。若残余×kp ≈ qfrc_passive 的量级，则前馈折算口径缺这一项。
    passive = []
    passive_forces = np.asarray(hold.qfrc_passive, dtype=float).copy()
    for name in arm_names:
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        dof = int(model.jnt_dofadr[joint_id])
        passive.append({
            "joint": name,
            "jnt_stiffness": float(model.jnt_stiffness[joint_id]),
            "dof_damping": float(model.dof_damping[dof]),
            "qfrc_passive_n_m": round(float(passive_forces[dof]), 6),
            "qfrc_bias_n_m": round(float(hold.qfrc_bias[dof]), 6),
            "residual_x_kp_n_m": settled.get(name, {}).get("residual_x_kp_n_m"),
        })
    # 约束力取证（2026-09-30 §11.33，最后一项力源）：`qfrc_constraint` = 等式（weld/connect）与
    # 限位约束施加的力，**既不在 qfrc_bias 也不在 qfrc_passive** ⇒ 前馈永远补不掉。
    constraint = []
    constraint_forces = np.asarray(hold.qfrc_constraint, dtype=float).copy()
    for name in arm_names:
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        dof = int(model.jnt_dofadr[joint_id])
        constraint.append({"joint": name,
                           "qfrc_constraint_n_m": round(float(constraint_forces[dof]), 6),
                           "residual_x_kp_n_m": settled.get(name, {}).get("residual_x_kp_n_m")})
    equalities = []
    for index in range(int(model.neq)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, index)
        equalities.append({"name": str(name), "type": int(model.eq_type[index]),
                           "active0": bool(model.eq_active0[index]),
                           "active_now": bool(hold.eq_active[index]) if hasattr(hold, "eq_active") else None})
    result = {"arm": args.arm, "phase": args.phase,
              "constraint_forces": constraint,
              "equalities": equalities,
              "passive_forces": passive,
              "contacts_at_hold": contacts,
              "hold": {"hold_ms": int(args.hold_ms), "passes": int(args.passes),
                       "worst_residual_rad": trace[-1]["worst_residual_rad"],
                       "trace": trace, "settled": settled},
              "report": str(REPO / args.report), "model": report["output"],
              "declared_tolerance_rad": 0.001,
              "ctrl_headroom": headroom,
              "worst_static_error_lower_bound_rad": round(worst, 9),
              "note": ("`static_error_lower_bound_rad = |τ_g| / (gear·kp)`：这是**单靠前馈**能把静差压到的"
                       "下限（不含摩擦/耦合）。若它已大于声明容差，说明该声明不可达；若 `gear ≠ 1`，"
                       "说明前馈公式漏了传动比。"),
              "joints": rows}
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    for row in rows:
        print("%-26s kp=%-9s gear=%-8s forcerange=%-22s τ_g=%-12s 静差下限=%-12s" %
              (row.get("joint"), row.get("kp_used_by_formula"), row.get("gear"),
               row.get("forcerange"), row.get("qfrc_bias_n_m"),
               row.get("static_error_lower_bound_rad")))
    print("最差静差下限 = %.9f rad（声明容差 %.9f）" % (worst, 0.001))
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
