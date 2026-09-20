"""用一个明确的解析思路找"夹爪竖直朝下"的抓取姿态，替代迭代式姿态修正。

为什么放弃迭代姿态修正：
UR5e 的"夹爪竖直朝下 + pad 中点落在指定点"是一个**有唯一解族**的问题，
用零空间迭代去逼近它，反复卡在开合轴 41~45° 的偏差上
（左右 pad 同树、旋转雅可比相减退化，开合轴那一维条件数极差）。

更稳的做法是**直接解**：
1. 夹爪指向 = 法兰 -Z 方向。要求它朝向世界 -Z（竖直向下）；
2. 开合轴 = 世界 +X（由 spread_axis 声明）；
3. 因此法兰的期望旋转矩阵可以直接写出；
4. 用"位置 IK 求出的腕部位置 + 该期望旋转"做一次**逆解的目标姿态**，
   但仍走位置 IK（契约层只支持位置），因此改为：
   把期望旋转对应的**腕部姿态**作为约束，用 6 维误差的阻尼最小二乘直接解，
   这在数值上等价于"求满足位姿的逆解"，条件数远好于零空间投影。

本脚本先做一个可行性验证：给定期望法兰旋转，用阻尼最小二乘同时收敛
位置与姿态，看能否把两项误差都压到阈值内。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def rot_to_vec(current, target):
    cross = np.cross(current, target)
    dot = float(np.clip(np.dot(current, target), -1.0, 1.0))
    sin_angle = float(np.linalg.norm(cross))
    if sin_angle < 1e-12:
        return np.zeros(3)
    angle = float(np.arctan2(sin_angle, dot))
    return cross / sin_angle * angle


def mat_to_vec(current, target):
    """两个旋转矩阵之间的旋转向量。"""
    relative = target @ current.T
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, relative.reshape(-1))
    vec = np.zeros(3)
    mujoco.mju_quat2Vel(vec, quat, 1.0)
    return vec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--iterations", type=int, default=600)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )
    arm = baseline["model"]["arm_joints"]
    joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)) for n in arm
    ]
    qpos_adr = [int(model.jnt_qposadr[j]) for j in joint_ids]
    dof_adr = [int(model.jnt_dofadr[j]) for j in joint_ids]

    # 种子：官方 home（该位形下指向已竖直、开合轴已沿 x）
    seed = baseline["gripper"]["seed"]
    for name, value in seed.items():
        data.qpos[int(model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])] = value

    left_geom = int(mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, baseline["model"]["finger_geoms"]["left"]))
    right_geom = int(mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, baseline["model"]["finger_geoms"]["right"]))
    flange = int(mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE,
        baseline["model"]["bodies"]["flange_site"]))

    # 构造期望法兰旋转：z 轴 = 夹爪指向的反方向，x 轴 = 开合轴
    direction = np.asarray(baseline["grasp"]["approach_direction"], dtype=float)
    direction = direction / np.linalg.norm(direction)
    spread = np.asarray(baseline["grasp"]["spread_axis"], dtype=float)
    spread = spread / np.linalg.norm(spread)
    # 法兰的 -z 指向夹爪外侧（实测：指向 = 法兰 -Z 的反方向 → 指向 = -(-z)= z）
    # 由 probe_seed_orientation：指向 [0,0,-1] 时法兰 z 轴 = [0,0,-1] 的相反
    # 实际观测：法兰 z = [0,-0,-1] 且指向 = [0,-0,-1]，故 指向 = 法兰 z 轴
    z_axis = -direction
    x_axis = spread
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    target_rot = np.column_stack([x_axis, y_axis, z_axis])

    target_pos = np.array(
        [*baseline["grasp"]["finger_center_xy_m"],
         baseline["workbench"]["top_z_m"] + baseline["target"]["half_size_m"]],
        dtype=float,
    )
    print("期望 pad 中点:", np.round(target_pos, 6))
    print("期望法兰旋转轴: x=%s y=%s z=%s"
          % (np.round(x_axis, 4), np.round(y_axis, 4), np.round(z_axis, 4)))

    for iteration in range(int(args.iterations)):
        mujoco.mj_forward(model, data)
        left = np.asarray(data.geom_xpos[left_geom], dtype=float)
        right = np.asarray(data.geom_xpos[right_geom], dtype=float)
        mid = (left + right) / 2.0
        current_rot = np.asarray(data.site_xmat[flange], dtype=float).reshape(3, 3)

        e_pos = target_pos - mid
        e_rot = mat_to_vec(current_rot, target_rot)
        if float(np.linalg.norm(e_pos)) < 1e-6 and float(np.linalg.norm(e_rot)) < 1e-5:
            break

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacGeom(model, data, jacp, jacr, left_geom)
        jacp2 = np.zeros((3, model.nv))
        mujoco.mj_jacGeom(model, data, jacp2, None, right_geom)
        jac_flange_p = np.zeros((3, model.nv))
        jac_flange_r = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac_flange_p, jac_flange_r, flange)

        # pad 中点的平移雅可比；法兰的旋转雅可比
        jac_mid = ((jacp + jacp2) / 2.0)[:, dof_adr]
        jac = np.vstack([jac_mid, jac_flange_r[:, dof_adr]])
        err = np.concatenate([e_pos, e_rot])
        lam = 1e-4
        delta = jac.T @ np.linalg.solve(
            jac @ jac.T + lam * np.eye(6), err
        )
        delta = np.clip(delta, -0.2, 0.2)
        for index, joint_id in enumerate(joint_ids):
            low, high = model.jnt_range[joint_id]
            data.qpos[qpos_adr[index]] = min(
                max(float(data.qpos[qpos_adr[index]]) + float(delta[index]), low),
                high,
            )

    mujoco.mj_forward(model, data)
    left = np.asarray(data.geom_xpos[left_geom], dtype=float)
    right = np.asarray(data.geom_xpos[right_geom], dtype=float)
    mid = (left + right) / 2.0
    current_rot = np.asarray(data.site_xmat[flange], dtype=float).reshape(3, 3)
    direction_now = (right - left)
    direction_now = direction_now / np.linalg.norm(direction_now)
    grip_dir = (mid - np.asarray(data.site_xpos[flange], dtype=float))
    grip_dir = grip_dir / np.linalg.norm(grip_dir)

    print("\n结果（迭代 %d 次）:" % (iteration + 1))
    print("  pad 中点      :", np.round(mid, 6), " 误差 %.3e m"
          % float(np.linalg.norm(target_pos - mid)))
    print("  夹爪指向      :", np.round(grip_dir, 6), " 与 -Z 夹角 %.3f deg"
          % np.degrees(np.arccos(np.clip(-grip_dir[2], -1, 1))))
    print("  开合轴(右-左) :", np.round(direction_now, 6), " 与 +X 夹角 %.3f deg"
          % np.degrees(np.arccos(np.clip(abs(direction_now[0]), 0, 1))))
    print("  两 pad 高度差 : %+.6f m" % (right[2] - left[2]))
    print("  关节角:", {n: round(float(data.qpos[qpos_adr[i]]), 6)
                       for i, n in enumerate(arm)})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
