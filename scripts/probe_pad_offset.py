"""核对对齐门禁的抓取点换算：pad 中点、pad_offset、接近轴三者的关系。

已知事实：
- 参考姿态求解时，IK 的目标就是"pad1 geom 中点"，且残差 6e-06 m；
- 执行后臂关节与参考值几乎完全一致（difference < 1e-8 rad）；
- 但门禁算出的 center_distance = 0.097 m。

门禁公式（mujoco_backend._grasp_alignment_evidence）：
    midpoint = (left_geom_xpos + right_geom_xpos) / 2
    center = midpoint - axis * pad_offset

若中点与 IK 目标一致，则 center 与 target 的差应等于
`-axis * pad_offset`。delta 的实测模长 0.0974 与
`pad_offset=0.0214` 不匹配，说明还有第二个偏差源。

本脚本在参考 grasp 姿态下把三者都量出来，逐一核对。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def _geom(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--pose", default="build/calibration/ur5-baseline-pose.json")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    pose = json.loads((root / args.pose).read_text(encoding="utf-8"))
    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )

    # 只摆臂、不 step：避免任何动力学把方块推走
    for name in baseline["model"]["arm_joints"]:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(
            pose["grasp"]["joint_positions"][name]
        )
    # 夹爪保持张开（命令 ctrl 并 step 到平衡）：pad 几何在张开态才算得准
    actuator_id = int(
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rq2f85_fingers_actuator"
        )
    )
    data.ctrl[:] = 0.0
    # 先用运动学把夹爪摆到张开（tendon 的静态开度≈0 时 pad 已是最大间隙）
    mujoco.mj_forward(model, data)

    lp = _geom(model, "rq2f85_left_pad1")
    rp = _geom(model, "rq2f85_right_pad1")
    target_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))

    midpoint = (np.asarray(data.geom_xpos[lp], dtype=float)
                + np.asarray(data.geom_xpos[rp], dtype=float)) / 2.0
    target = np.asarray(data.xpos[target_body], dtype=float).copy()

    ref_mid = np.asarray(pose["grasp"]["finger_center_m"], dtype=float)
    axis_ref = np.asarray(pose["approach_direction_world"], dtype=float)
    axis_ref = axis_ref / np.linalg.norm(axis_ref)
    correction = float(pose["finger_height_correction_m"])
    pad_offset = float(pose["finger_height_correction_m"])

    print("IK 目标（finger_center_m）  :", np.round(ref_mid, 6))
    print("实测 pad1 中点            :", np.round(midpoint, 6))
    print("  差（实测-参考）          :", np.round(midpoint - ref_mid, 6),
          " |d|=%.6f" % float(np.linalg.norm(midpoint - ref_mid)))
    print("目标方块中心              :", np.round(target, 6))
    print("  目标 - 参考中点          :", np.round(target - ref_mid, 6),
          " |d|=%.6f" % float(np.linalg.norm(target - ref_mid)))
    print()
    print("height_correction_m = pad_offset =", round(correction, 9))
    print("approach_direction_world       =", np.round(axis_ref, 6))
    print("轴与竖直的夹角 = %.3f deg" % np.degrees(
        np.arccos(np.clip(axis_ref[2], -1, 1))))
    print()
    # 门禁公式复现
    center = midpoint - axis_ref * pad_offset
    delta = center - target
    print("门禁 center = midpoint - axis*pad_offset:", np.round(center, 6))
    print("门禁 delta  = center - target          :", np.round(delta, 6),
          " |d|=%.6f" % float(np.linalg.norm(delta)))
    print()
    # 如果轴取竖直（场景里 pad_offset_axis 的声明值）
    vertical = np.array([0.0, 0.0, 1.0])
    delta_v = (midpoint - vertical * pad_offset) - target
    print("若轴=竖直 [0,0,1]: delta =", np.round(delta_v, 6),
          " |d|=%.6f" % float(np.linalg.norm(delta_v)))
    print()
    print("-- 期望：让 center == target 需要的轴 --")
    need = midpoint - target
    norm = float(np.linalg.norm(need))
    print("midpoint - target =", np.round(need, 6), " |d|=%.6f" % norm)
    print("对应单位轴 =", np.round(need / max(1e-12, norm), 6),
          "（pad_offset 应为 %.6f）" % norm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
