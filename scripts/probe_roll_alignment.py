"""诊断夹爪绕"接近轴"的滚转角：pad 开合轴必须对齐方块的一个面法向。

已知（probe_pad_geometry.py）：
- 目标 AABB  x[-0.575,-0.525] y[-0.159,-0.109] z[0,0.050]
- left_pad1  中心 z=0.0766（比目标高 52mm）
- right_pad1 中心 z=0.0162
- 左右 pad 主要沿 **x** 分开，且高度差 60mm —— 夹爪是**倾斜**的。

原因：IK 只约束了"pad 中点位置"，没有约束**姿态**（滚转）。
夹爪绕接近轴的滚转是自由量，求解器落到了任意一支，
于是 pad 面斜插进方块，把方块顶飞。

本脚本在参考 grasp 姿态下量出：
1. pad 开合轴（右 pad 减左 pad）与目标面法向的夹角 —— 应为 0 或 90° 的整数倍；
2. 上下 pad 的高度差 —— 应为 0（即两 pad 在同一水平面）；
3. 给出"要让开合轴对齐某条世界轴"所需的滚转修正角。

结论用于决定：是给参考姿态求解加姿态约束，还是在配置里显式声明
"抓取方向 / 开合轴"由哪两个世界轴决定。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


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

    lp = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rq2f85_left_pad1"))
    rp = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rq2f85_right_pad1"))
    left = np.asarray(data.geom_xpos[lp], dtype=float)
    right = np.asarray(data.geom_xpos[rp], dtype=float)

    axis = right - left
    spread = float(np.linalg.norm(axis))
    axis_unit = axis / max(1e-12, spread)
    print("左 pad:", np.round(left, 6))
    print("右 pad:", np.round(right, 6))
    print("开合轴（右-左）:", np.round(axis, 6), " 间距=%.6f" % spread)
    print("开合轴单位向量:", np.round(axis_unit, 6))
    print("上下高度差 (right_z - left_z) = %+.6f" % (right[2] - left[2]))
    print()

    # 开合轴与三条世界轴的夹角：正确的平面抓取应为 0/90/180 度之一
    for label, vec in (("+X", [1, 0, 0]), ("+Y", [0, 1, 0]), ("+Z", [0, 0, 1])):
        v = np.asarray(vec, dtype=float)
        cos = float(np.dot(axis_unit, v))
        angle = float(np.degrees(np.arccos(np.clip(abs(cos), 0.0, 1.0))))
        print("开合轴与 %s 的夹角 = %6.2f deg（|cos|=%.4f）" % (label, angle, abs(cos)))

    # 夹爪指向（法兰到 pad 中点）
    flange = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site"))
    fpos = np.asarray(data.site_xpos[flange], dtype=float)
    mid = (left + right) / 2.0
    direction = mid - fpos
    direction = direction / np.linalg.norm(direction)
    print("\n夹爪指向（法兰→pad 中点）:", np.round(direction, 6),
          " 与 -Z 夹角 = %.2f deg"
          % float(np.degrees(np.arccos(np.clip(-direction[2], -1, 1)))))

    # 目标方块的三个面法向（单位四元数 ⇒ 轴对齐）
    print("\n目标面法向（轴对齐）: ±X, ±Y, ±Z")
    print("→ 正确抓取要求开合轴 = ±X 或 ±Y（水平面内），且两 pad 等高")

    # 法向的正交基：给出"理想开合轴"
    print("\n-- 诊断结论 --")
    if abs(right[2] - left[2]) > 1e-3:
        print("两 pad 高度差 %.6f m > 1mm：夹爪相对水平面**倾斜**，" % (right[2] - left[2]))
        print("  DESCEND 时下侧 pad 会先撞到方块侧面/顶面，把方块顶飞。")
    else:
        print("两 pad 等高：夹爪水平，姿态正确。")
    horizontal = float(np.linalg.norm(axis[:3]) ** 2 - axis[2] ** 2)
    if abs(axis_unit[2]) > 0.05:
        print("开合轴 z 分量 = %.4f（应 ≈0）：开合轴不在水平面内。" % axis_unit[2])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
