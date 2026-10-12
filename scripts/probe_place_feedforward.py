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

# 相位顺序**不再本地抄一份**（P4 / B2：探针曾因漏掉 `via` 误判一次）：
# 从**报告里实际存在的 positions 键**推导，并与单点声明 `PLACE_PHASES` 自动比对。
from iraf_skills.common.place_phases import (  # noqa: E402
    PLACE_PHASES,
    PLACE_POSITIONS_KEYS,
    PLACE_PHASE_NAME_BY_KEY,
)


def resolve_place_phases(gripper):
    """从报告的 `gripper` 段取**实际存在**的放置 positions 键，按单点声明的规范顺序返回。

    返回 `(phases, warnings)`：`phases` 是 `(相位名, positions_key)` 列表；`warnings` 列出
    报告里出现了但单点声明未登记的键（不静默丢弃 —— 暴露"新相位漏登记"）。
    """
    present = [key for key in PLACE_POSITIONS_KEYS
               if isinstance(gripper.get(key), dict) and gripper.get(key)]
    phases = [(PLACE_PHASE_NAME_BY_KEY[key], key) for key in present]
    unknown = [key for key, value in gripper.items()
               if key.startswith("place_") and key.endswith("_positions")
               and isinstance(value, dict) and value and key not in PLACE_PHASE_NAME_BY_KEY]
    return phases, unknown


def main():
    import build_piper_baseline as shared

    model = mujoco.MjModel.from_xml_path(
        str(REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"))
    baseline = yaml.safe_load((REPO / "config/ur5_simulation_baseline.yaml").read_text())
    joint = json.loads((REPO / "build/scenes/handoff_lab/handoff_lab_joint.json").read_text())
    gripper = ((joint["manipulation"]["per_robot"]["ur5e"]) or {}).get("gripper") or {}
    grip_keys = set(gripper.get("open_positions") or {}) | set(gripper.get("closed_positions") or {})
    place_phases, unknown_place_keys = resolve_place_phases(gripper)
    print("[相位来源] 报告实际相位 = %s（单点声明 PLACE_PHASES = %s）"
          % ([name for name, _ in place_phases], [p.name for p in PLACE_PHASES]))
    if unknown_place_keys:
        print("[相位来源] ⚠ 报告里有未登记的放置 positions 键：%s" % unknown_place_keys)
    print("[夹爪通道键] %s" % sorted(grip_keys))

    extra_poses, gripper_positions = {}, {}
    for phase, positions_key in place_phases:
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
