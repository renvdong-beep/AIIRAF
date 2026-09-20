"""诊断 UR5 对齐门禁报 94mm 偏差的原因：pad 参考点与目标中心的关系。

现象：
- 参考姿态求解残差 6e-06 m（IK 正确）；
- 后端执行到 grasp 姿态后，臂关节与参考值几乎完全一致
  （shoulder_pan -0.433164909 vs 参考 -0.433164912）；
- 但对齐门禁报 center_distance = 0.094018 m，远超 5mm 容差。

差 94mm 说明门禁算的"抓取点"与"目标中心"不是同一个物理点。
本脚本在**参考姿态**下把各候选参考点逐一量出来：

- pad 内表面中点（真正的夹持中心）
- pad1 geom 坐标中点、pad body 坐标中点
- pinch site
- 目标方块中心

比对"哪一个减掉 pad_offset 后能对齐目标中心"，即可定位偏差来源。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def _body(model, name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError("缺少 body: " + name)
    return int(body_id)


def _site(model, name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError("缺少 site: " + name)
    return int(site_id)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene", default="build/models/ur5-pick-scene.xml"
    )
    parser.add_argument(
        "--pose", default="build/calibration/ur5-baseline-pose.json"
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    pose = json.loads((root / args.pose).read_text(encoding="utf-8"))
    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )
    arm_names = baseline["model"]["arm_joints"]

    # 摆到参考 grasp 姿态，并把夹爪张开（与门禁执行时的状态一致）
    for name in arm_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(
            pose["grasp"]["joint_positions"][name]
        )
    actuator = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rq2f85_fingers_actuator"
    )
    data.ctrl[int(actuator)] = 0.0
    for _ in range(1200):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    lp1 = _geom(model, "rq2f85_left_pad1")
    rp1 = _geom(model, "rq2f85_right_pad1")
    lp2 = _geom(model, "rq2f85_left_pad2")
    rp2 = _geom(model, "rq2f85_right_pad2")
    left_body = _body(model, "rq2f85_left_pad")
    right_body = _body(model, "rq2f85_right_pad")
    pinch = _site(model, "pinch")
    target_body = _body(model, "box_01")

    pad1_mid = (np.asarray(data.geom_xpos[lp1], dtype=float)
                + np.asarray(data.geom_xpos[rp1], dtype=float)) / 2.0
    pad2_mid = (np.asarray(data.geom_xpos[lp2], dtype=float)
                + np.asarray(data.geom_xpos[rp2], dtype=float)) / 2.0
    body_mid = (np.asarray(data.xpos[left_body], dtype=float)
                + np.asarray(data.xpos[right_body], dtype=float)) / 2.0
    pinch_pos = np.asarray(data.site_xpos[pinch], dtype=float).copy()
    target = np.asarray(data.xpos[target_body], dtype=float).copy()

    # 2F-85 的 pad box 局部 y 是厚度方向；内表面中点 = 中心 ± half_y
    half_y = float(model.geom_size[lp1][1])
    axis = np.asarray(data.geom_xpos[rp1], dtype=float) - np.asarray(
        data.geom_xpos[lp1], dtype=float
    )
    axis = axis / max(1e-12, float(np.linalg.norm(axis)))
    inner_mid = pad1_mid  # 两侧内表面中点即 pad1 中心的中点

    candidates = {
        "pad1_center_mid": pad1_mid,
        "pad2_center_mid": pad2_mid,
        "pad_body_mid": body_mid,
        "pinch_site": pinch_pos,
        "inner_surface_mid": inner_mid,
    }
    print("target (box_01 center):", np.round(target, 6))
    print("pad half_y =", round(half_y, 6), " gap axis =", np.round(axis, 6))
    print()
    for label, point in candidates.items():
        delta = point - target
        print("%-20s %s  |delta|=%.6f  dz=%.6f"
              % (label, np.round(point, 6), float(np.linalg.norm(delta)), delta[2]))

    # pad 内表面到目标表面的最近距离（判断是否真的夹住了方块）
    print()
    for label, geom in (("left_pad1", lp1), ("right_pad1", rp1)):
        d = np.asarray(data.geom_xpos[geom], dtype=float) - target
        print("%-12s 相对目标 %s |d|=%.6f" % (label, np.round(d, 6),
                                             float(np.linalg.norm(d))))

    print("\ncontacts:", int(data.ncon))
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        print("  %s | %s  dist=%.6f"
              % (
                  mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                  mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
                  float(contact.dist),
              ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
