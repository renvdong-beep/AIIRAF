#!/usr/bin/env python3
"""Go2 四足演示：前后左右移动（运动学步态可视化，不含物理）。

- 只做 FK：设定基座位姿 + 12 个腿关节角，mj_forward 取各 body 世界坐标；
- 步态用对角小跑（trot）的关节角正弦，仅作演示可视化，不声明物理步态结论；
- 支持 forward/back/left/right + 距离；场景里固定一个参考 box 用于定位对比。
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(os.environ.get("IRAF_DEMO_GO2_MODEL")
             or "/home/coretek/AIIRAF/vendor/unitree_go2/unitree_robots/go2/scene.xml")
LEGS = ["FL", "FR", "RL", "RR"]
FEET = [leg + "_foot" for leg in LEGS]
CHAINS = [[leg + "_hip", leg + "_thigh", leg + "_calf", leg + "_foot"] for leg in LEGS]
HOME = {f"{leg}_{part}_joint": v for leg in LEGS
        for part, v in (("hip", 0.0), ("thigh", 0.9), ("calf", -1.8))}
BASE_Z = 0.27
DIRS = {"forward": (1.0, 0.0), "back": (-1.0, 0.0),
        "left": (0.0, 1.0), "right": (0.0, -1.0)}
LABELS = {"forward": "前进", "back": "后退", "left": "左移", "right": "右移"}
REFERENCE = {"pos": [0.9, 0.0, 0.05], "half": [0.05, 0.05, 0.05], "label": "参考 box"}


class Go2Planner:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.adr = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.adr[name] = int(self.model.jnt_qposadr[i])

    def _set(self, d, q):
        for name, v in q.items():
            d.qpos[self.adr[name]] = float(v)

    def build(self, direction="forward", distance=0.5, fps=30):
        if direction not in DIRS:
            raise ValueError("方向只支持 forward/back/left/right")
        distance = float(distance)
        # 与 profiles/safety/quadruped_lab.yaml 的工作空间一致：单次平移不得超过 0.5m。
        # 不能静默 clamp，否则 Web 会把越界计划当成合法轨迹播放。
        if abs(distance) > 0.5:
            raise ValueError("Go2 单次移动距离 %.2fm 超过工作空间上限 0.5m（安全策略拒绝）" % distance)
        dx, dy = DIRS[direction]
        # 头/机身朝向固定（朝 +x），前后左右只平移；转的只有腿的步态
        yaw = 0.0
        quat = [1.0, 0.0, 0.0, 0.0]
        duration = 4.0
        n = int(duration * fps)
        base0 = np.array([0.0, 0.0, BASE_Z])
        move_delta = np.array([dx * distance, dy * distance, 0.0])

        frames = []
        for i in range(n + 1):
            s = i / n
            t = s * duration
            phase = 2 * math.pi * 1.6 * t          # 1.6 Hz 步频（示意）
            # 0–0.6s 站立；0.6–3.4s 移动；3.4–4s 收步
            if t <= 0.6:
                p = 0.0
                gait = 0.0
            elif t <= 3.4:
                p = (t - 0.6) / 2.8
                gait = 1.0
            else:
                p = 1.0
                gait = 0.0
            e = 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, p)))
            base = base0 + move_delta * e
            base[2] = BASE_Z + 0.012 * math.sin(2 * phase) * gait

            q = {}
            for leg in LEGS:
                off = 0.0 if leg in ("FL", "RR") else math.pi
                ph = phase + off
                q[f"{leg}_hip_joint"] = 0.03 * math.sin(ph) * gait
                q[f"{leg}_thigh_joint"] = HOME[f"{leg}_thigh_joint"] + 0.22 * math.sin(ph) * gait
                q[f"{leg}_calf_joint"] = HOME[f"{leg}_calf_joint"] + 0.35 * math.sin(ph + 0.6) * gait

            d = mujoco.MjData(self.model)
            d.qpos[0:3] = base
            d.qpos[3:7] = quat
            self._set(d, q)
            mujoco.mj_forward(self.model, d)

            pos = {"base_link": [round(float(v), 5) for v in d.xpos[1]]}
            for leg in LEGS:
                for part in ("hip", "thigh", "calf", "foot"):
                    b = f"{leg}_{part}"
                    bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
                    pos[b] = [round(float(v), 5) for v in d.xpos[bid]]

            label = "站立" if gait == 0 and p == 0 else ("移动" if gait else "收步")
            frames.append({"t": round(t, 3), "phase": "move" if gait else "stand",
                           "phase_label": label, "base": [round(float(v), 5) for v in base],
                           "yaw": round(float(yaw), 5), "pos": pos})

        return {
            "kind": "quadruped", "robot": "go2", "bodies": ["base_link"] + [b for c in CHAINS for b in c],
            "chains": CHAINS, "feet": FEET, "reference": REFERENCE,
            "body_box": {"body": "base_link", "half": [0.20, 0.11, 0.09]},
            "base_body": "base_link",
            "head": {"local_offset": [0.24, 0.0, 0.04], "half": [0.075, 0.06, 0.05],
                     "label": "狗头（固定朝前）"},
            "target": {"id": "参考 box", "pos": REFERENCE["pos"], "half_size": REFERENCE["half"]},
            "start": [0.0, 0.0], "end": [round(float(dx * distance), 3), round(float(dy * distance), 3)],
            "frames": frames,
            "phases": [{"name": "stand", "label": "站立", "t0": 0.0, "t1": 0.6},
                       {"name": "move", "label": "移动", "t0": 0.6, "t1": 3.4},
                       {"name": "settle", "label": "收步", "t0": 3.4, "t1": 4.0}],
            "meta": {"command": direction, "command_label": LABELS[direction],
                     "distance_m": round(float(distance), 3), "duration_s": duration,
                     "frames": len(frames), "gait": "trot（运动学示意）",
                     "ik_error_m": 0.0},
        }
