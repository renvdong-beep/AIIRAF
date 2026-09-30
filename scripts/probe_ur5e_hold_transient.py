#!/usr/bin/env python3
"""UR5e 保持过程的**瞬态判死**：是"4 s 没走完行程"还是"饱和极限环"（2026-09-30 §11.34）。

背景（§11.33）：在**联合模型**上跑声明口径的静态保持（4 s × 4 轮，补偿 c = τ_g(q)/kp）后，
`home` 相位的残余 0.049657459 rad，且逐关节取证实测 **wrist_1 = −28.0000、wrist_2 = +28.0000
恰好等于 forcerange ±28（饱和）**、elbow 执行器力 +118.7636 N·m 而该位形重力只有 −15.538884
⇒ 怀疑"这不是静差，而是根本还没停"（或饱和造成的极限环）。

本探针记录每个仿真步的 qpos/qvel/执行器力，给出判据所需的三个量：
  1. 起始位形（关键帧）与目标（声明位形）的差 —— 看"行程有多大"；
  2. 末 1 s 内 |qvel| 的均值与**前 1 s** 对比 —— 衰减 ⇒ 只是没走完（改 hold_ms）；不衰减 ⇒ 极限环；
  3. 末 1 s 内撞到 forcerange 的样本比例（按关节） —— 饱和是否为常态。

用法：PYTHONPATH=src python3 scripts/probe_ur5e_hold_transient.py [--phase home] [--hold-ms 4000]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--arm", default="ur5e")
    parser.add_argument("--phase", default="home", choices=("home", "approach", "grasp", "lift"))
    parser.add_argument("--hold-ms", type=int, default=4000)
    parser.add_argument("--passes", type=int, default=4)
    parser.add_argument("--output", default="build/diagnostics/ur5e-hold-transient.json")
    args = parser.parse_args()

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    entry = (((report.get("manipulation") or {}).get("per_robot") or {}).get(args.arm) or {})
    gripper = entry.get("gripper") or {}
    name_map = entry.get("name_map") or {}
    model = mujoco.MjModel.from_xml_path(str(REPO / report["output"]))
    data = mujoco.MjData(model)
    if int(getattr(model, "nkey", 0) or 0) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)

    def model_name(text):
        return name_map.get(str(text), str(text))

    section = {model_name(k): v for k, v in (gripper.get("%s_positions" % args.phase) or {}).items()}
    arm_names = [k for k in section
                 if int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, k)) >= 0]
    targets = {k: float(section[k]) for k in arm_names}
    keyframe = {k: float(data.qpos[int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, k)])]) for k in arm_names}
    ff_cfg = ((yaml.safe_load((REPO / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8"))
               or {}).get("gravity_feedforward") or {})
    steps = max(1, int(round(float(args.hold_ms) / 1000.0 / float(model.opt.timestep))))

    actuators, gains, dofs, adrs = {}, {}, {}, {}
    for name in arm_names:
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        actuator = next(index for index in range(int(model.nu))
                        if int(model.actuator_trnid[index, 0]) == joint_id)
        actuators[name] = actuator
        gains[name] = float(model.actuator_gainprm[actuator][0])
        dofs[name] = int(model.jnt_dofadr[joint_id])
        adrs[name] = int(model.jnt_qposadr[joint_id])

    finger_hold = {}
    for key, value in section.items():
        actuator = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, key))
        if actuator >= 0:
            finger_hold[actuator] = float(value)

    # 每步记录（只为臂关节；4 s × 4 轮 = 8000 步，量小）
    trace = []
    for attempt in range(1, int(args.passes) + 1):
        bias = np.asarray(data.qfrc_bias, dtype=float)
        for name in arm_names:
            data.ctrl[actuators[name]] = targets[name] + float(bias[dofs[name]]) / gains[name]
        for actuator, value in finger_hold.items():
            data.ctrl[actuator] = value
        for step in range(steps):
            mujoco.mj_step(model, data)
            trace.append({
                "t": round((attempt - 1) * args.hold_ms / 1000.0
                           + (step + 1) * float(model.opt.timestep), 4),
                "qvel": {name: round(float(data.qvel[dofs[name]]), 9) for name in arm_names},
                "qpos": {name: round(float(data.qpos[adrs[name]]), 9) for name in arm_names},
                "force": {name: round(float(data.actuator_force[actuators[name]]), 6)
                          for name in arm_names},
            })

    window = max(1, int(round(0.5 / float(model.opt.timestep))))
    tail, before = trace[-window:], trace[-2 * window:-window]

    def mean_abs(rows):
        return {name: round(float(np.mean([abs(row["qvel"][name]) for row in rows])), 9)
                for name in arm_names}

    forcerange = {name: [float(v) for v in model.actuator_forcerange[actuators[name]]]
                  for name in arm_names}

    def sat_ratio(rows):
        out = {}
        for name in arm_names:
            limit = max(abs(forcerange[name][0]), abs(forcerange[name][1]))
            if limit == 0:
                out[name] = None
                continue
            hits = sum(1 for row in rows
                       if abs(abs(row["force"][name]) - limit) < 1e-6)
            out[name] = round(hits / float(len(rows)), 6)
        return out

    result = {
        "arm": args.arm, "phase": args.phase, "model": report["output"],
        "hold_ms": int(args.hold_ms), "passes": int(args.passes),
        "timestep_s": float(model.opt.timestep), "samples": len(trace),
        "keyframe_rad": {k: round(v, 9) for k, v in keyframe.items()},
        "target_rad": {k: round(v, 9) for k, v in targets.items()},
        "travel_rad": {k: round(targets[k] - keyframe[k], 9) for k in arm_names},
        "mean_abs_qvel_last_0p5s": mean_abs(tail),
        "mean_abs_qvel_prev_0p5s": mean_abs(before),
        "saturation_ratio_last_0p5s": sat_ratio(tail),
        "forcerange": forcerange,
        "final_qpos_rad": {k: trace[-1]["qpos"][k] for k in arm_names},
        "final_residual_rad": {k: round(trace[-1]["qpos"][k] - targets[k], 9) for k in arm_names},
        "note": ("判读：末 0.5 s 的 |qvel| 均值明显小于前 0.5 s ⇒ 只是没走完行程（改 hold_ms）；"
                 "两者同量级且饱和比高 ⇒ 极限环（该位形不可保持，须换 home 位形）。"),
    }
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("相位 %s：行程(声明−关键帧) = %s" % (args.phase, result["travel_rad"]))
    print("末 0.5 s |qvel| = %s" % result["mean_abs_qvel_last_0p5s"])
    print("前 0.5 s |qvel| = %s" % result["mean_abs_qvel_prev_0p5s"])
    print("末 0.5 s 饱和样本比 = %s" % result["saturation_ratio_last_0p5s"])
    print("末态残余 = %s" % result["final_residual_rad"])
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
