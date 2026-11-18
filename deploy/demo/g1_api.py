#!/usr/bin/env python3
"""G1 人形演示：前后左右行走（运动学步态，无物理）。

- 基座平移 + 双腿 6 关节正弦摆动、双臂反向摆；头/躯干朝向固定。
- 只做 FK 取 body 世界坐标，不 mj_step。
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(os.environ.get("IRAF_DEMO_G1_MODEL")
             or "/home/coretek/AIIRAF/vendor/unitree_g1/g1_widefoot.xml")

LEG_JOINTS = {
    "L": ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
          "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"],
    "R": ["right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
          "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"],
}
ARM_JOINTS = {
    "L": ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint"],
    "R": ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"],
}
LEG_CHAINS = {
    "L": ["pelvis", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link",
          "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link"],
    "R": ["pelvis", "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link",
          "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link"],
}
ARM_CHAINS = {
    "L": ["torso_link", "left_shoulder_pitch_link", "left_shoulder_roll_link",
          "left_shoulder_yaw_link", "left_elbow_link", "left_wrist_yaw_link"],
    "R": ["torso_link", "right_shoulder_pitch_link", "right_shoulder_roll_link",
          "right_shoulder_yaw_link", "right_elbow_link", "right_wrist_yaw_link"],
}
DIRS = {"forward": (1.0, 0.0), "back": (-1.0, 0.0),
        "left": (0.0, 1.0), "right": (0.0, -1.0)}
LABELS = {"forward": "前进", "back": "后退", "left": "左移", "right": "右移"}
REFERENCE = {"pos": [0.9, 0.0, 0.05], "half": [0.05, 0.05, 0.05], "label": "参考 box"}


class G1Planner:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.adr = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.adr[name] = int(self.model.jnt_qposadr[i])
        self.base_adr = self.adr.get("floating_base_joint", 0)
        q0 = self.model.qpos0
        self.base_z = float(q0[self.base_adr + 2]) if self.base_adr + 2 < len(q0) else 0.75
        if self.base_z < 0.2:
            self.base_z = 0.75

    def _set(self, d, q):
        for name, v in q.items():
            if name in self.adr:
                d.qpos[self.adr[name]] = float(v)

    def build(self, direction="forward", distance=0.5, fps=30):
        if direction not in DIRS:
            raise ValueError("方向只支持 forward/back/left/right")
        distance = max(-2.0, min(2.0, float(distance)))
        dx, dy = DIRS[direction]
        duration = 4.0
        n = int(duration * fps)
        start = np.array([0.0, 0.0, self.base_z])
        delta = np.array([dx * distance, dy * distance, 0.0])

        frames = []
        for i in range(n + 1):
            s = i / n
            t = s * duration
            phase = 2 * math.pi * 1.1 * t
            if t <= 0.6:
                p, gait = 0.0, 0.0
            elif t <= 3.4:
                p = (t - 0.6) / 2.8
                gait = 1.0
            else:
                p, gait = 1.0, 0.0
            e = 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, p)))
            base = start + delta * e
            base[2] = self.base_z + 0.015 * math.sin(2 * phase) * gait

            q = {}
            for side, off in (("L", 0.0), ("R", math.pi)):
                ph = phase + off
                sgn = 1.0 if side == "L" else -1.0
                q[f"{side.lower()}_hip_pitch_joint"] = 0.35 * math.sin(ph) * gait
                q[f"{side.lower()}_hip_roll_joint"] = 0.04 * sgn * gait
                q[f"{side.lower()}_hip_yaw_joint"] = 0.05 * math.sin(ph) * gait
                knee = 0.15 + 0.28 * math.sin(ph + 0.5) * gait
                q[f"{side.lower()}_knee_joint"] = max(0.03, knee)
                q[f"{side.lower()}_ankle_pitch_joint"] = -0.28 * math.sin(ph) * gait
                q[f"{side.lower()}_ankle_roll_joint"] = 0.0
                # 手臂反向摆
                q[f"{side.lower()}_shoulder_pitch_joint"] = -0.25 * math.sin(ph) * gait
                q[f"{side.lower()}_shoulder_roll_joint"] = 0.12 if side == "L" else -0.12
                q[f"{side.lower()}_elbow_joint"] = 0.25

            d = mujoco.MjData(self.model)
            d.qpos[0:3] = base
            d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            self._set(d, q)
            mujoco.mj_forward(self.model, d)

            pos = {}
            for chain in list(LEG_CHAINS.values()) + list(ARM_CHAINS.values()):
                for b in chain:
                    bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
                    if bid >= 0:
                        pos[b] = [round(float(v), 5) for v in d.xpos[bid]]
            frames.append({"t": round(t, 3), "phase": "move" if gait else "stand",
                           "phase_label": "站立" if (gait == 0 and p == 0) else ("移动" if gait else "收步"),
                           "base": [round(float(v), 5) for v in base],
                           "yaw": 0.0, "pos": pos})

        chains = [LEG_CHAINS["L"], LEG_CHAINS["R"], ARM_CHAINS["L"], ARM_CHAINS["R"]]
        bodies = sorted({b for c in chains for b in c})
        return {
            "kind": "humanoid", "robot": "g1",
            "bodies": bodies, "chains": chains, "feet": ["L", "R", "LA", "RA"],
            "reference": REFERENCE, "body_box": {"body": "pelvis", "half": [0.13, 0.10, 0.13]},
            "base_body": "pelvis",
            "head": {"local_offset": [0.0, 0.0, 0.40], "half": [0.075, 0.07, 0.08],
                     "label": "头（固定朝前）"},
            "target": {"id": "参考 box", "pos": REFERENCE["pos"], "half_size": REFERENCE["half"]},
            "start": [0.0, 0.0],
            "end": [round(float(dx * distance), 3), round(float(dy * distance), 3)],
            "frames": frames,
            "phases": [{"name": "stand", "label": "站立", "t0": 0.0, "t1": 0.6},
                       {"name": "move", "label": "行走", "t0": 0.6, "t1": 3.4},
                       {"name": "settle", "label": "收步", "t0": 3.4, "t1": 4.0}],
            "meta": {"command": direction, "command_label": LABELS[direction],
                     "distance_m": round(float(distance), 3), "duration_s": duration,
                     "frames": len(frames), "gait": "walk（运动学示意）",
                     "ik_error_m": 0.0},
        }
