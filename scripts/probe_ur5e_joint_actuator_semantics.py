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

    # 参考姿态（grasp）摆好，量该位形下的重力矩
    arm_positions = {}
    for key, value in (gripper.get("grasp_positions") or {}).items():
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
    result = {"arm": args.arm, "report": str(REPO / args.report), "model": report["output"],
              "declared_tolerance_rad": 0.001,
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
