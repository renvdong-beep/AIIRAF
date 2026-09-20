"""量出参考 grasp 姿态下 pad 与方块的接触几何，确认是否需要 pad_offset。

疑点：BACKEND 的 DESCEND 目标把 **pad1 geom 中心**放到方块中心高度，
而 2F-85 的 pad1/pad2 是两个 box（局部 pos 不同），它们的中心并不是
"夹持中心"。若 pad 内表面到块面的间隙为负，下降时必然撞飞方块。

本脚本在参考 grasp 姿态下（夹爪张开）量：
- 左右 pad1/pad2 的世界 AABB；
- 方块 AABB；
- 各 pad 内表面到最近块面的间隙（负 = 已穿透）；
- pad 内表面在开合轴上的间距 vs 方块边长。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


def box_extents(model, data, geom_id):
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    half = np.asarray(model.geom_size[geom_id], dtype=float)
    centre = np.asarray(data.geom_xpos[geom_id], dtype=float)
    return centre, np.abs(rotation) @ half


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--pose", default="build/calibration/ur5-baseline-pose.json")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    pose = json.loads((root / args.pose).read_text(encoding="utf-8"))

    # 摆到参考 grasp 姿态；夹爪保持张开（用 keyframe 的开度）
    for name, value in pose["grasp"]["joint_positions"].items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)

    # 张开（tendon ctrl=0）需要动力学收敛，但只量几何的话
    # 直接用参考姿态的 qpos 即可（tendon 静态开度在 ctrl=0 时≈0）。
    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    target = np.asarray(data.xpos[body_id], dtype=float).copy()
    print("box_01 中心:", np.round(target, 6))

    pads = {}
    for name in ("rq2f85_left_pad1", "rq2f85_left_pad2",
                 "rq2f85_right_pad1", "rq2f85_right_pad2"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            continue
        centre, extents = box_extents(model, data, int(gid))
        pads[name] = (centre, extents)
        print("%-22s centre=%s extents=%s"
              % (name, np.round(centre, 6), np.round(extents, 6)))

    print("\n-- pad 内表面相对方块表面（沿世界 x = 开合轴）--")
    for name, (centre, extents) in pads.items():
        inner_x = centre[0] - extents[0] if "left" in name else centre[0] + extents[0]
        # 方块在 x 方向的面
        near_face = target[0] + 0.025 if "left" in name else target[0] - 0.025
        gap = abs(inner_x - near_face)
        side = "left" if "left" in name else "right"
        print("  %-22s 内表面 x=%+8.5f  块面 x=%+8.5f  间隙=%+.6f"
              % (name, inner_x, near_face, gap if inner_x > near_face else -gap))

    print("\n-- 竖直方向：pad 底面 vs 方块顶面 --")
    top = target[2] + 0.025
    for name, (centre, extents) in pads.items():
        bottom = centre[2] - extents[2]
        print("  %-22s pad 底面 z=%+8.5f  块顶面 z=%+8.5f  净空=%+.6f"
              % (name, bottom, top, bottom - top))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
