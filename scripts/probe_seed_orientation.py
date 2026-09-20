"""量出"种子位形"与"期望姿态"的关系，解释姿态偏差恰好 90.000 度。

现象：用 reach_forward_down 作种子后，夹爪指向偏差恰好 = 90.000 deg。
这个整数说明存在结构性错位（而不是收敛不足）：
很可能"法兰 → pad 中点"这个定义与 pad 的实际朝向不一致，
或坐标轴取反。

本脚本在若干候选位形下打印：
- 法兰 site 的世界位置与姿态；
- pad 中点的世界位置；
- 法兰 → pad 中点 的向量；
- 左/右 pad 连线向量；
并把它们与候选期望方向对照。
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np


def _site(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name))


def _geom(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def _qadr(model, name):
    return int(
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
    )


def report(model, data, label):
    mujoco.mj_forward(model, data)
    flange_id = _site(model, "attachment_site")
    lp, rp = _geom(model, "rq2f85_left_pad1"), _geom(model, "rq2f85_right_pad1")
    flange = np.asarray(data.site_xpos[flange_id], dtype=float)
    flange_mat = np.asarray(data.site_xmat[flange_id], dtype=float).reshape(3, 3)
    left = np.asarray(data.geom_xpos[lp], dtype=float)
    right = np.asarray(data.geom_xpos[rp], dtype=float)
    mid = (left + right) / 2.0
    direction = mid - flange
    direction = direction / np.linalg.norm(direction)
    axis = right - left
    axis = axis / np.linalg.norm(axis)
    print("\n[%s]" % label)
    print("  flange        :", np.round(flange, 6))
    print("  flange 轴 (site_xmat 三列 = x,y,z):")
    for index, name in enumerate("xyz"):
        print("      %s = %s" % (name, np.round(flange_mat[:, index], 6)))
    print("  pad mid       :", np.round(mid, 6))
    print("  指向 法兰→mid :", np.round(direction, 6),
          " 与 -Z 夹角 %.2f deg"
          % np.degrees(np.arccos(np.clip(-direction[2], -1, 1))))
    print("  开合轴 右-左  :", np.round(axis, 6),
          " 与 +X 夹角 %.2f deg"
          % np.degrees(np.arccos(np.clip(abs(axis[0]), 0, 1))))
    print("  法兰 -Z 方向  :", np.round(-flange_mat[:, 2], 6))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)

    seed = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.570796,
        "elbow_joint": 1.570796,
        "wrist_1_joint": -1.570796,
        "wrist_2_joint": -1.570796,
        "wrist_3_joint": 0.0,
    }
    for name, value in seed.items():
        data.qpos[_qadr(model, name)] = value
    report(model, data, "seed=reach_forward_down")

    # 官方 home
    home = {
        "shoulder_pan_joint": -1.5708,
        "shoulder_lift_joint": -1.5708,
        "elbow_joint": 1.5708,
        "wrist_1_joint": -1.5708,
        "wrist_2_joint": -1.5708,
        "wrist_3_joint": 0.0,
    }
    for name, value in home.items():
        data.qpos[_qadr(model, name)] = value
    report(model, data, "official_home")

    # 全零
    data.qpos[:] = 0.0
    report(model, data, "all_zero")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
