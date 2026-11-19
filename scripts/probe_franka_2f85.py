#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Franka + 2F-85 功能自检：机械臂关节可动、夹爪可开合。"""
import mujoco
import numpy as np

MODEL = "/home/coretek/AIIRAF/build/models/franka_2f85/franka_2f85.xml"
m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

arm_act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator%d" % i) for i in range(1, 8)]
grip_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "rq2f85_fingers_actuator")
print("arm_actuators", arm_act, "gripper_actuator", grip_act)

q0 = d.qpos.copy()
# 机械臂：小角度目标（关节 0.2 rad）
for idx, aid in zip(range(1, 8), arm_act):
    d.ctrl[aid] = 0.2
for _ in range(500):
    mujoco.mj_step(m, d)
arm_delta = float(np.max(np.abs(d.qpos[:7] - q0[:7])))
print("arm_max_delta_rad", arm_delta)

# 夹爪：开 → 合
grip_joint_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in
                  ("rq2f85_right_driver_joint", "rq2f85_left_driver_joint")]
def grip_positions():
    return [float(d.qpos[m.jnt_qposadr[j]]) for j in grip_joint_ids]

d.ctrl[grip_act] = 0.0
for _ in range(1000):
    mujoco.mj_step(m, d)
open_pos = grip_positions()
d.ctrl[grip_act] = 255.0
for _ in range(1000):
    mujoco.mj_step(m, d)
closed_pos = grip_positions()
print("gripper_open", open_pos)
print("gripper_closed", closed_pos)
print("gripper_travel", [abs(a - b) for a, b in zip(open_pos, closed_pos)])
print("moved", arm_delta > 0.05 and max(abs(a - b) for a, b in zip(open_pos, closed_pos)) > 0.01)
