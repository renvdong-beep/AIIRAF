"""复现后端的 DESCEND 阶段，量出对齐门禁看到的真实几何。

背景：静态核对该姿态时，`midpoint - axis*pad_offset - target = 6e-06 m`
（完全通过门禁），但后端执行时报 0.097 m。两者差异只能来自
**执行过程**，因此本脚本按后端同样的顺序复现：

    HOME_HOLD -> APPROACH -> DESCEND （各段 quintic + settle）

每段结束后打印：臂关节角、pad1 中点、目标中心、门禁 delta。
这样能看出偏差是在哪一段、以什么形式出现的。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml


def _geom(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def _actuator(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))


def quintic(start, end, duration_s, t):
    if t >= duration_s:
        return end
    x = t / duration_s
    return start + (end - start) * (10 * x**3 - 15 * x**4 + 6 * x**5)


def run_phase(model, data, targets, duration_ms, settle_ms, label):
    steps = max(1, int(np.ceil(duration_ms / 1000.0 / model.opt.timestep)))
    current = {}
    for name in targets:
        aid = _actuator(model, name)
        joint_id = int(model.actuator_trnid[aid, 0])
        current[name] = float(data.qpos[int(model.jnt_qposadr[joint_id])])
    for step in range(1, steps + 1):
        elapsed = step * float(model.opt.timestep)
        for name, goal in targets.items():
            data.ctrl[_actuator(model, name)] = quintic(
                current[name], float(goal), duration_ms / 1000.0, elapsed
            )
        mujoco.mj_step(model, data)
    for name, goal in targets.items():
        data.ctrl[_actuator(model, name)] = float(goal)
    for _ in range(max(1, int(settle_ms / 1000.0 / model.opt.timestep))):
        for name, goal in targets.items():
            data.ctrl[_actuator(model, name)] = float(goal)
        mujoco.mj_step(model, data)
    report(model, data, label)


def report(model, data, label):
    mujoco.mj_forward(model, data)
    lp, rp = _geom(model, "rq2f85_left_pad1"), _geom(model, "rq2f85_right_pad1")
    left = np.asarray(data.geom_xpos[lp], float)
    right = np.asarray(data.geom_xpos[rp], float)
    mid = (left + right) / 2.0
    target_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    target = np.asarray(data.xpos[target_body], float).copy()
    print("\n[%s]" % label)
    print("  pad1 中点:", np.round(mid, 6),
          " 左:", np.round(left, 6), " 右:", np.round(right, 6))
    print("  pad 高度差: %+.6f" % (right[2] - left[2]))
    print("  box_01   :", np.round(target, 6))
    print("  中点-目标:", np.round(mid - target, 6),
          " |d|=%.6f" % float(np.linalg.norm(mid - target)))
    print("  臂关节角 :", {
        n: round(float(data.qpos[int(model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])]), 6)
        for n in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
    })
    print("  contacts :", int(data.ncon))
    for index in range(int(data.ncon)):
        c = data.contact[index]
        print("     %s | %s dist=%+.6f"
              % (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                 mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2),
                 float(c.dist)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--pose", default="build/calibration/ur5-baseline-pose.json")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument(
        "--phases", default="HOME_HOLD,APPROACH,DESCEND",
        help="要复现的阶段序列",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    scene = json.loads(
        (root / "build/models/ur5-pick-scene.json").read_text(encoding="utf-8")
    )
    gripper = scene["gripper"]
    duration = args.duration_ms
    segment = max(1, duration // 5)

    report(model, data, "INITIAL")
    for phase in [p.strip() for p in args.phases.split(",") if p.strip()]:
        key = {
            "HOME_HOLD": "home_positions",
            "APPROACH": "approach_positions",
            "DESCEND": "grasp_positions",
        }[phase]
        targets = gripper.get(key)
        if not targets:
            print("缺少", key)
            continue
        run_phase(model, data, targets, segment, segment * 4, phase)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
