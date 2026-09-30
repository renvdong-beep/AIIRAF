#!/usr/bin/env python3
"""home 相位交叉核对：同一组关节解在**臂自己模型**与**联合模型**上 FK 的夹持区中点比。

判据（用于判死"是求解器解错了还是我量错了"）：
  · 若同一组关节在两个模型上给出的夹持区中点**只差臂基座位姿**（把基座变换扣掉后一致），
    说明关节解本身没问题、问题在该位姿不可达/不适合本场景；
  · 若差得更多，说明某一侧的 FK 口径被我写错了（探针缺陷），必须先修探针。

用法：PYTHONPATH=src python3 scripts/probe_ur5e_home_crossmodel.py
"""
from __future__ import annotations

import json
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
ARM_MODEL = REPO / "build/models/ur5-pick-scene.xml"
JOINT_REPORT = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"


def pad_mid(model, data, names):
    pts = []
    for name in names:
        ident = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(name)))
        if ident < 0:
            return None, "geom 不存在: %s" % name
        pts.append(np.asarray(data.geom_xpos[ident], dtype=float))
    return np.mean(np.asarray(pts), axis=0), None


def set_arm(model, data, joints):
    for name, value in joints.items():
        jid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name)))
        if jid < 0:
            return "关节不存在: %s" % name
        data.qpos[int(model.jnt_qposadr[jid])] = float(value)
    mujoco.mj_forward(model, data)
    return None


def main():
    out = {}
    arm_report = json.loads((REPO / "build/models/ur5-pick-scene.json").read_text(encoding="utf-8"))
    joint_report = json.loads(JOINT_REPORT.read_text(encoding="utf-8"))
    seg = joint_report["manipulation"]["per_robot"]["ur5e"]
    jg = seg["gripper"]
    arm_pads = list(((arm_report["model"] if "model" in arm_report else {}).get("finger_geoms")
                     or {}).get("pad_boxes") or ())
    if not arm_pads:
        arm_pads = list(jg.get("pad_boxes") or ())
    arm_pads = [str(n) for n in arm_pads]

    arm_model = mujoco.MjModel.from_xml_path(str(ARM_MODEL))
    arm_data = mujoco.MjData(arm_model)
    joint_model = mujoco.MjModel.from_xml_path(str(REPO / joint_report["output"]))
    joint_data = mujoco.MjData(joint_model)

    arm_side_home = {k: float(v) for k, v in (arm_report["reference_poses"]["home_solved"]
                                              ["joint_positions"]).items()}
    joint_side_home = {k.replace("ur5e_", ""): float(v) for k, v in jg["home_positions"].items()
                       if k.startswith("ur5e_") and k.endswith("_joint")}

    cases = {"arm_side_home_solved": arm_side_home, "joint_side_home": joint_side_home}
    for label, joints in cases.items():
        entry = {"joints": joints}
        err = set_arm(arm_model, arm_data, joints)
        if err:
            entry["arm_model_error"] = err
        else:
            mid, err2 = pad_mid(arm_model, arm_data, arm_pads)
            entry["arm_model_pad_mid_m"] = [round(float(v), 9) for v in mid] if mid is not None else err2
        err = set_arm(joint_model, joint_data, {"ur5e_%s" % k: v for k, v in joints.items()})
        if err:
            entry["joint_model_error"] = err
        else:
            mid, err2 = pad_mid(joint_model, joint_data, arm_pads)
            entry["joint_model_pad_mid_m"] = [round(float(v), 9) for v in mid] if mid is not None else err2
        out[label] = entry

    # 联合模型里 UR5e 基座的实际世界位姿（用于把"臂模型系"换算到"世界系"）
    base_body = None
    for index in range(int(joint_model.nbody)):
        name = mujoco.mj_id2name(joint_model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
        if name.startswith("ur5e") and name.endswith("base_link"):
            base_body = index
            break
    if base_body is None:
        for index in range(int(joint_model.nbody)):
            name = mujoco.mj_id2name(joint_model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
            if name.startswith("ur5e"):
                base_body = index
                break
    out["joint_model_arm_base"] = {
        "body": mujoco.mj_id2name(joint_model, mujoco.mjtObj.mjOBJ_BODY, base_body) if base_body is not None else None,
        "xpos": [round(float(v), 9) for v in joint_data.xpos[base_body]] if base_body is not None else None,
    }
    out["stage"] = {"arm_model": str(ARM_MODEL), "joint_model": joint_report["output"],
                    "pad_geoms": arm_pads}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
