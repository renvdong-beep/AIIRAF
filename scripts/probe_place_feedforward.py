#!/usr/bin/env python3
"""放置四段重力前馈的**隔离验证**（2026-09-30 §11.61）。

用途：**不改构建器**，直接拿联合模型 + 联合报告里的 `place_*_positions` 调共享前馈实现
（`build_piper_baseline.build_reference_feedforward` 的 `extra_poses` 分支），看它到底算出什么。
把一个"构建器整体行为"的问题隔离成"共享函数本身对不对"——首次实现时 ur5e 前馈整体变空，
只有把这一步单独跑一遍才能分清是共享函数、转发还是对账环节的问题。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_place_feedforward.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

PLACE_PHASES = (("place_transit", "place_transit_positions"),
                ("place_above", "place_above_positions"),
                ("place_descend", "place_descend_positions"),
                ("place_retreat", "place_retreat_positions"))


def main():
    import build_piper_baseline as shared

    model = mujoco.MjModel.from_xml_path(
        str(REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"))
    baseline = yaml.safe_load((REPO / "config/ur5_simulation_baseline.yaml").read_text())
    joint = json.loads((REPO / "build/scenes/handoff_lab/handoff_lab_joint.json").read_text())
    gripper = ((joint["manipulation"]["per_robot"]["ur5e"]) or {}).get("gripper") or {}
    grip_keys = set(gripper.get("open_positions") or {}) | set(gripper.get("closed_positions") or {})
    print("[夹爪通道键] %s" % sorted(grip_keys))

    extra_poses, gripper_positions = {}, {}
    for phase, positions_key in PLACE_PHASES:
        section = gripper.get(positions_key) or {}
        arm_only = {str(k)[len("ur5e_"):]: float(v) for k, v in section.items()
                    if str(k) not in grip_keys}
        if arm_only:
            extra_poses[phase] = {"joint_positions": arm_only}
        grip_only = {str(k): float(v) for k, v in section.items() if str(k) in grip_keys}
        if grip_only:
            gripper_positions[phase] = grip_only
    print("[输入] 相位=%s 臂关节数=%s" % (sorted(extra_poses),
                                          {k: len(v["joint_positions"]) for k, v in extra_poses.items()}))
    feedforward, evidence = shared.build_reference_feedforward(
        model, {}, baseline, "ur5e_", gripper_positions, extra_poses)
    print("[输出] 相位=%s" % sorted(feedforward))
    for phase in sorted(feedforward):
        print("  %-14s %s" % (phase, {k: round(float(v), 6) for k, v in feedforward[phase].items()}))
        ev = evidence.get(phase) or {}
        if ev:
            print("     证据: residual=%s passes=%s" % (ev.get("max_residual_rad"), ev.get("passes")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
