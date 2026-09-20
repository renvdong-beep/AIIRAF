"""诊断"方块在抓取开始前就飞走"：从静止状态直接 mj_step，看它是否稳定。

现象：HOME_HOLD 段结束时 box_01 已在 z=5.30m（probe_descent.py），
说明方块**在任何机械臂动作之前或期间**就获得了巨大速度。

候选原因：
1. 目标带 `gravcomp=1`（重力补偿）而**没有约束**，在自由关节下
   gravcomp 会让"重力"与"补偿力"数值上不完全抵消，
   残余加速度在长时间积分下把方块推走；
   更严重的是 gravcomp 作用在**质心**，若质心与几何中心不重合会产生力矩。
2. 方块初始穿透台面（z=0.025、半边长 0.025 ⇒ 底面恰好 z=0），
   求解器为消除穿透给出巨大分离冲量。
3. 与 pad/夹爪有初始接触。

本脚本从 qpos 初始化状态出发，**不给任何 ctrl** 直接积分，
打印前若干步与若干毫秒后的位置/速度，定位是哪一类问题。
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--steps", type=int, default=2000)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    model = mujoco.MjModel.from_xml_path(str(root / args.scene))
    data = mujoco.MjData(model)
    baseline = yaml.safe_load(
        (root / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8")
    )

    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    joint_id = int(model.body_jntadr[body_id])
    qpos_adr = int(model.jnt_qposadr[joint_id])
    dof_adr = int(model.jnt_dofadr[joint_id])

    print("目标 gravcomp:", float(model.body_gravcomp[body_id]))
    print("目标质量     :", float(model.body_mass[body_id]))
    print("目标初始 qpos:", np.round(data.qpos[qpos_adr:qpos_adr + 7], 6))
    print("台面 top_z   :", baseline["workbench"]["top_z_m"])
    print("目标半边长   :", baseline["target"]["half_size_m"])
    print("⇒ 底面 z = %.6f（应为 0）"
          % (data.qpos[qpos_adr + 2] - baseline["target"]["half_size_m"]))
    print()

    # 时间步长：UR5 的 2ms 对 40g 方块偏大
    print("timestep = %.6f s" % float(model.opt.timestep))

    for step in range(1, int(args.steps) + 1):
        mujoco.mj_step(model, data)
        if step in (1, 2, 5, 10, 50, 100, 500, 1000, 2000):
            pos = np.asarray(data.qpos[qpos_adr:qpos_adr + 3], dtype=float)
            vel = np.asarray(data.qvel[dof_adr:dof_adr + 3], dtype=float)
            print("step %5d  pos=%s  vel=%s"
                  % (step, np.round(pos, 6), np.round(vel, 6)))
        if not np.all(np.isfinite(data.qpos)):
            print("step %d 已发散" % step)
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
