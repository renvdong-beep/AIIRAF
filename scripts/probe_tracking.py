"""诊断 DESCEND 段"指令值 vs 实际到位值"的跟踪误差。

观测到的关键事实（后端口径）：
- grasp_positions.shoulder_lift = -0.022
- 执行后 alignment 报的 qpos.shoulder_lift_joint = -0.520
两者差 0.498 rad ≈ 28.5°，远超伺服稳态误差，说明**不是单纯的跟踪问题**。

可能原因：
1. 后端的 `_move_trajectory` 里 `starts` 是按 **actuator_trnid 反查关节** 读的，
   若某执行器的 trnid 指向的不是预期关节，插值起点就会错；
2. DESCEND 段的 settle 时间不够（本脚本用同一条 quintic + settle 复现）；
3. 夹爪在下降时已经碰到方块，接触力矩把臂"顶"回去。

本脚本直接复现后端的三段轨迹（与 mujoco_backend._move_trajectory 同算法），
每段结束打印"指令 vs 实际"，即可区分 1/2/3。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


def quintic(start, end, duration_s, t):
    if t >= duration_s:
        return end
    x = t / duration_s
    return start + (end - start) * (10 * x**3 - 15 * x**4 + 6 * x**5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--duration-ms", type=int, default=12000)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    # 与后端一致：从 keyframe[0] 初始化
    if int(model.nkey) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    scene = json.loads(
        (root / "build/models/ur5-pick-scene.json").read_text(encoding="utf-8")
    )
    gripper = scene["gripper"]
    segment = max(1, args.duration_ms // 5)

    def actuator_of(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))

    def joint_name_of(actuator_id):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)

    print("执行器 → 关节 映射:")
    for name in gripper["grasp_positions"]:
        aid = actuator_of(name)
        print("  %-28s -> %s" % (name, joint_name_of(aid) if aid >= 0 else "缺失"))
    print()

    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))

    for phase, key in (
        ("HOME_HOLD", "home_positions"),
        ("APPROACH", "approach_positions"),
        ("DESCEND", "grasp_positions"),
    ):
        targets = gripper[key]
        names = list(targets)
        aid_list = [actuator_of(n) for n in names]
        starts = []
        for aid in aid_list:
            joint_id = int(model.actuator_trnid[aid, 0])
            starts.append(float(data.qpos[int(model.jnt_qposadr[joint_id])]))

        steps = max(1, int(np.ceil(segment / 1000.0 / model.opt.timestep)))
        for step in range(1, steps + 1):
            elapsed = step * float(model.opt.timestep)
            for index, name in enumerate(names):
                data.ctrl[aid_list[index]] = quintic(
                    starts[index], float(targets[name]), segment / 1000.0, elapsed
                )
            mujoco.mj_step(model, data)
        for index, name in enumerate(names):
            data.ctrl[aid_list[index]] = float(targets[name])
        settle = max(250, min(16000, segment * 4))
        for _ in range(max(1, int(np.ceil(settle / 1000.0 / model.opt.timestep)))):
            for index, name in enumerate(names):
                data.ctrl[aid_list[index]] = float(targets[name])
            mujoco.mj_step(model, data)

        mujoco.mj_forward(model, data)
        print("[%s] settle=%dms" % (phase, settle))
        worst = 0.0
        for index, name in enumerate(names):
            joint_id = int(model.actuator_trnid[aid_list[index], 0])
            actual = float(data.qpos[int(model.jnt_qposadr[joint_id])])
            commanded = float(targets[name])
            error = actual - commanded
            worst = max(worst, abs(error))
            print("   %-28s cmd=%+9.6f act=%+9.6f err=%+9.6f"
                  % (name, commanded, actual, error))
        box = np.asarray(data.xpos[body_id], dtype=float)
        print("   最大跟踪误差 %.6f rad | box_01=%s"
              % (worst, np.round(box, 5)))
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
