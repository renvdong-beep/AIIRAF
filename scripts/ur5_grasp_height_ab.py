#!/usr/bin/env python3
"""UR5e「夹口高度」的**臂侧 vs 联合侧**同口径对照（I3 归因用，纯几何）。

用法：PYTHONPATH=src python3 scripts/ur5_grasp_height_ab.py
（放在 scripts/ 而不是 build/：它有证据价值，必须进版本库 —— build/ 被 gitignore。）

同一段 FK 逻辑跑两个模型：
  臂侧：build/models/ur5-pick-scene.xml + 该报告 gripper.grasp_positions
  联合侧：build/scenes/handoff_lab/handoff_lab_joint.xml + per_robot[ur5e].gripper.grasp_positions
两边都量 `pad_mid_z − 载荷中心 z`（载荷中心 = 各自报告里 targets[0].position_m），
并与声明的 `pad_offset_m` 对照 ⇒ 一步分离"声明语义问题"还是"联合重解/覆盖问题"。
"""
import json, pathlib
import mujoco, numpy as np
REPO = pathlib.Path(__file__).resolve().parents[1]
ARM_REPORT = REPO / "build/models/ur5-pick-scene.json"
JOINT_REPORT = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"

def measure(model_path, positions, target_xy_z, pad_names):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    applied = 0
    for name, value in positions.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if jid >= 0:
            data.qpos[int(model.jnt_qposadr[jid])] = float(value); applied += 1
    mujoco.mj_forward(model, data)
    pts = []
    for n in pad_names:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(n))
        if gid >= 0: pts.append(np.asarray(data.geom_xpos[gid], dtype=float))
    if len(pts) != 2: return {"error": "指腹 geom 未解析到", "pad_names": pad_names}
    mid = (pts[0] + pts[1]) / 2.0
    return {"pad_mid_z": float(mid[2]), "target_z": float(target_xy_z[2]),
            "grasp_height_m": float(mid[2] - float(target_xy_z[2])),
            "joints_applied": applied, "model": str(pathlib.Path(model_path).name)}

arm = json.loads(ARM_REPORT.read_text(encoding="utf-8"))
g = arm.get("gripper") or {}
arm_pads = [g.get("left_finger_geom"), g.get("right_finger_geom")]
arm_t = (arm.get("targets") or [{}])[0].get("position_m") or [0, 0, 0]
print("臂侧声明 pad_offset_m =", g.get("pad_offset_m"))
print("臂侧:", json.dumps(measure(ARM_REPORT.parent / pathlib.Path(arm["output"]).name, g.get("grasp_positions") or {}, arm_t, arm_pads), ensure_ascii=False))
joint = json.loads(JOINT_REPORT.read_text(encoding="utf-8"))
e = (joint["manipulation"]["per_robot"]["ur5e"].get("gripper")) or {}
print("联合侧声明 pad_offset_m(同源) =", e.get("pad_offset_m"))
print("联合侧:", json.dumps(measure(REPO / joint["output"], e.get("grasp_positions") or {}, [0.45, 0.45, 0.375372], [e.get("left_finger_geom"), e.get("right_finger_geom")]), ensure_ascii=False))
