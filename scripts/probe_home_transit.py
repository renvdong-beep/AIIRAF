"""诊断 HOME 段为什么会扫到方块：零位→HOME 的路径是否穿过方块所在高度。

INITIAL（全零位）实测：pad 中点在 [-0.817, -0.378, 0.063]
HOME 实测：            pad 中点在 [-0.556, -0.134, 0.473]
方块：                 [-0.550, -0.134, 0.025]，顶面 z=0.050，AABB 半宽 25mm

即机械臂从 (y=-0.378) 摆到 (y=-0.134) 的过程中，**一度贴台面 63mm 掠过**，
而方块顶面在 50mm、水平范围 y∈[-0.159,-0.109] —— 完全在扫掠路径上。

本脚本把这条路径离散采样出来，逐点量 pad 与方块的最近距离，
量化"扫掠窗口"有多宽，从而判断应该把起始位形抬高多少才安全。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def _geom(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def _qadr(model, name):
    return int(model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--pose", default="build/calibration/ur5-baseline-pose.json")
    parser.add_argument("--samples", type=int, default=40)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    pose = json.loads((root / args.pose).read_text(encoding="utf-8"))
    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )
    arm = baseline["model"]["arm_joints"]
    half = float(baseline["target"]["half_size_m"])

    lp = _geom(model, "rq2f85_left_pad1")
    rp = _geom(model, "rq2f85_right_pad1")
    target_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))

    start = {name: 0.0 for name in arm}
    end = pose["home"]

    print("零位 → HOME 的直线插值扫掠，逐点量 pad 与方块的净空")
    print("%6s %-26s %-26s %10s" % ("t", "pad1 中点", "方块中心", "水平距离"))
    worst = None
    for index in range(int(args.samples) + 1):
        t = index / float(args.samples)
        for name in arm:
            data.qpos[_qadr(model, name)] = (
                start[name] + (end[name] - start[name]) * t
            )
        mujoco.mj_forward(model, data)
        mid = (np.asarray(data.geom_xpos[lp], float)
               + np.asarray(data.geom_xpos[rp], float)) / 2.0
        target = np.asarray(data.xpos[target_body], float).copy()
        horizontal = float(np.linalg.norm(mid[:2] - target[:2]))
        # 方块 AABB 在水平面上的对角线半径
        clearance = horizontal - half * np.sqrt(2.0)
        # 只在 pad 低于方块顶面时才算"会撞"
        danger = clearance if mid[2] < target[2] + half + 0.01 else float("inf")
        if danger < float("inf") and (worst is None or danger < worst[1]):
            worst = (t, danger, mid.copy(), target.copy())
        print("%6.2f %-26s %-26s %10.4f  %s"
              % (t, np.round(mid, 4), np.round(target, 4), horizontal,
                 "危险" if clearance < 0 else ""))
    if worst:
        print("\n最危险时刻 t=%.2f，pad 中点 %s，水平净空 %.4f m"
              % (worst[0], np.round(worst[2], 4), worst[1]))
    else:
        print("\n全程 pad 都高于方块顶面 + 10mm，无碰撞风险")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
