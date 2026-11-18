#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Franka 的联合世界参考姿态（P4 最小实现）：无夹爪/无抓取载荷，只提供 home。

真实抓取事实需按本体标定（与 ur5 的 I3/I4 同类），本文件只让 franka 作为**可装配本体**
进入联合世界；不声明 pick/place 能力。
"""


def build_reference_poses(root, baseline_doc, target_xy_override_m=None, target_z_override_m=None):
    joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
              "finger_joint1", "finger_joint2"]
    home = {name: 0.0 for name in joints}
    return {
        "home": home,
        "approach": {"joint_positions": dict(home)},
        "grasp": {"joint_positions": dict(home)},
        "lift": {"joint_positions": dict(home)},
        "pad_offset_m": 0.0,
    }
