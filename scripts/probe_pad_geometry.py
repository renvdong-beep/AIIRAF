"""量出 2F-85 两个 pad 的**碰撞几何**与方块的实际间隙，判断下降时会不会撞上。

背景：DESCEND 段把方块顶到 z=1.46m。pad 中点位置正确（z=0.0424），
说明"参考点"没问题，问题在**碰撞体**：
2F-85 的 pad 由 4 个 box geom 组成（pad1/pad2 × 左/右），
每个 box 在 pad body 局部系里都有自己的 pos/size。
"两 pad 中心距 - 2*half_y"这类估算只在几何完全对齐时才成立，
必须直接把 box 的**世界包围盒**量出来并与方块尺寸对照。

本脚本在参考 grasp 姿态下量：
1. 每个 pad box 的世界中心与半轴投影（用于算真实净空）；
2. 左右两侧碰撞体在开合轴上的最近面到方块表面的距离；
3. 方块顶面到 pad1 底面的竖直净空。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def box_world_extents(model, data, geom_id):
    """返回 box geom 在世界系下的中心与沿三个世界轴的半长。"""
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=float)
    centre = np.asarray(data.geom_xpos[geom_id], dtype=float)
    extents = np.abs(rotation) @ half
    return centre, extents


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
    for name in baseline["model"]["arm_joints"]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[jid])] = float(
            pose["grasp"]["joint_positions"][name]
        )
    mujoco.mj_forward(model, data)

    target_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    target = np.asarray(data.xpos[target_body], dtype=float).copy()
    half = float(baseline["target"]["half_size_m"])
    print("目标中心:", np.round(target, 6), " 半边长:", half)
    print("目标 AABB: x[%.4f,%.4f] y[%.4f,%.4f] z[%.4f,%.4f]"
          % (target[0] - half, target[0] + half,
             target[1] - half, target[1] + half,
             target[2] - half, target[2] + half))
    print()

    print("%-22s %-24s %-24s" % ("geom", "world centre", "world half-extents"))
    for name in ("rq2f85_left_pad1", "rq2f85_left_pad2",
                 "rq2f85_right_pad1", "rq2f85_right_pad2"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            print(name, "缺失")
            continue
        centre, extents = box_world_extents(model, data, int(gid))
        print("%-22s %-24s %-24s"
              % (name, np.round(centre, 6), np.round(extents, 6)))

    print("\n-- 与方块 AABB 的净空（负值表示已穿透）--")
    for name in ("rq2f85_left_pad1", "rq2f85_right_pad1"):
        gid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
        centre, extents = box_world_extents(model, data, gid)
        gap_x = abs(centre[0] - target[0]) - extents[0] - half
        gap_y = abs(centre[1] - target[1]) - extents[1] - half
        gap_z = abs(centre[2] - target[2]) - extents[2] - half
        print("%-22s gap_x=%+.6f gap_y=%+.6f gap_z=%+.6f"
              % (name, gap_x, gap_y, gap_z))

    print("\n前向若干步（不开夹爪），观察方块是否被顶走:")
    for step in (50, 200, 600):
        for _ in range(step):
            mujoco.mj_step(model, data)
        pos = np.asarray(data.xpos[target_body], dtype=float)
        print("  第 %4d 步后 box_01 = %s" % (step, np.round(pos, 6)))

    print("\ncontacts:", int(data.ncon))
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        print("  %-22s | %-22s dist=%+.6f" % (n1, n2, float(contact.dist)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
