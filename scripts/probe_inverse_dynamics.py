"""诊断 `_gravity_hold_ctrl` 的静态力为何会顶到执行器力矩限。

现象：四个参考姿态算出的前馈增量出现 0.075 rad（=150N/2000）与
0.056 rad（=28N/500），恰好等于 UR5e 官方力矩限值；而实际稳态误差
（0.015 rad → 约 30N·m）远小于此。两者矛盾，说明 mj_inverse 的
"静止所需广义力"被污染。

本脚本在同一模型上枚举四种口径，打印臂关节的 qfrc_inverse：
  A) keyframe + 方块在位（正常场景）
  B) keyframe + 方块移到 z=5m（远离机械臂）
  C) keyframe + 关闭全部 geom 接触（contype/conaffinity 置 0）
  D) 无 keyframe（qpos 全零，方块与基座重叠）——历史口径
并打印关节空间重力矩的理论量级作对照。
"""

import json
from pathlib import Path

import mujoco
import numpy as np

root = Path(__file__).resolve().parents[1]
scene_path = root / "build/models/ur5-pick-scene.xml"
scene = json.loads(scene_path.with_suffix(".json").read_text(encoding="utf-8"))
poses = json.loads(
    (root / "build/calibration/ur5-baseline-pose.json").read_text(encoding="utf-8")
)
arm_names = list(scene["arm_joints"])
hold = {name: float(poses["grasp"]["joint_positions"][name]) for name in arm_names}

model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

joint_ids = [
    int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)) for name in arm_names
]
dof_adrs = [int(model.jnt_dofadr[j]) for j in joint_ids]
qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]
gains = []
for name, joint_id in zip(arm_names, joint_ids):
    for index in range(int(model.nu)):
        if int(model.actuator_trnid[index, 0]) == joint_id:
            gains.append(float(model.actuator_gainprm[index][0]))
            break
box_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, scene["target_id"]))
box_qadr = int(model.jnt_qposadr[int(model.body_jntadr[box_body])])

print("检查姿态: grasp,  arm=%s" % arm_names)
print("执行器增益: %s" % dict(zip(arm_names, gains)))
print()


def measure(label, reset_keyframe=True, move_box=None, disable_contacts=False):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.qfrc_applied[:] = 0.0
    if reset_keyframe and int(model.nkey) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    for name in arm_names:
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        data.qpos[int(model.jnt_qposadr[joint_id])] = hold[name]
    if move_box is not None:
        data.qpos[box_qadr:box_qadr + 3] = move_box
    data.qvel[:] = 0.0
    if disable_contacts:
        saved = np.array(model.geom_contype, dtype=int), np.array(
            model.geom_conaffinity, dtype=int
        )
        model.geom_contype[:] = 0
        model.geom_conaffinity[:] = 0
    try:
        mujoco.mj_forward(model, data)
        mujoco.mj_inverse(model, data)
        forces = [float(data.qfrc_inverse[adr]) for adr in dof_adrs]
        ncon = int(data.ncon)
    finally:
        if disable_contacts:
            model.geom_contype[:] = saved[0]
            model.geom_conaffinity[:] = saved[1]
    print("%-46s ncon=%-3d 静态力(N·m) = %s" % (
        label,
        ncon,
        " ".join("%+8.3f" % v for v in forces),
    ))
    print("%-46s 折合 ctrl 增量(rad) = %s" % (
        "",
        " ".join("%+8.4f" % (v / g) for v, g in zip(forces, gains)),
    ))
    return forces


measure("A) keyframe + 方块在位", reset_keyframe=True)
measure("B) keyframe + 方块移到 z=5m", reset_keyframe=True, move_box=[0.0, 0.0, 5.0])
measure("C) keyframe + 关闭全部接触", reset_keyframe=True, disable_contacts=True)
measure("D) 无 keyframe（qpos 全零，方块压基座）", reset_keyframe=False)
measure(
    "E) 无 keyframe + 关闭全部接触",
    reset_keyframe=False,
    disable_contacts=True,
)

# 理论对照：各连杆质量与质心到肩部轴线的水平距离
print()
print("连杆质量与 CoM 到肩部竖直轴的水平距离（用于手算重力矩量级）:")
total = 0.0
for bid in range(1, model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or "?"
    if not name.startswith(("shoulder", "upper_arm", "forearm", "wrist", "rq2f85")):
        continue
    mass = float(model.body_mass[bid])
    com = np.asarray(data.xpos[bid], dtype=float)
    lever = float(np.hypot(com[0], com[1]))
    total += mass * lever * 9.81
    print("  %-26s mass=%6.3f kg  水平力臂=%6.3f m  贡献 %6.2f N·m"
          % (name, mass, lever, mass * lever * 9.81))
print("  合计（对肩部竖直轴）≈ %.1f N·m" % total)
