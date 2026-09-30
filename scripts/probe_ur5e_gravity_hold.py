#!/usr/bin/env python3
"""UR5e 重力前馈"保持测试"的 A/B 直调探针（2026-09-30 §11.33）。

问题：共享实现 `iraf_core.kinematics.gravity_hold_ctrl` 在**联合模型**上重算本臂前馈时，
残余停在 0.049653448 rad（= 逐关节单步下限 |τ_g|/(gear·kp) 的 4.8 倍），4 轮只改善 0.4%。

候选真因（本轮）：`hold_positions` 里的**夹爪通道**在旧实现里只按**关节名**解析，而 2F-85 的夹爪是
**腱驱动执行器**（`rq2f85_fingers_actuator`）⇒ 被静默跳过 ⇒ 保持测试期间它按 **keyframe 的 ctrl**
出力（可能把指腹合拢/顶住），接触反力经腕部污染臂关节残差。

A/B 口径（不改代码即可对照）：
  · A（新）：`hold_positions` 同时给出**夹爪执行器名**=声明值（0.0 = 全张）
  · B（旧）：不给夹爪键 ⇒ 它保持 keyframe ctrl（= 旧实现的静默跳过路径）

判据：A 的最差残余 ≤ 声明容差（0.001 rad）而 B 明显更差 ⇒ 真因判死为"夹爪通道没被写 ctrl"。

用法：PYTHONPATH=src python3 scripts/probe_ur5e_gravity_hold.py
      [--report build/scenes/handoff_lab/handoff_lab_joint.json] [--arm ur5e] [--phase grasp]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco

REPO = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--arm", default="ur5e")
    parser.add_argument("--phase", default="grasp", choices=("home", "approach", "grasp", "lift"))
    parser.add_argument("--output", default="build/diagnostics/ur5e-gravity-hold-ab.json")
    args = parser.parse_args()

    from iraf_core.kinematics import gravity_hold_ctrl

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    entry = (((report.get("manipulation") or {}).get("per_robot") or {}).get(args.arm) or {})
    gripper = entry.get("gripper") or {}
    name_map = entry.get("name_map") or {}
    model = mujoco.MjModel.from_xml_path(str(REPO / report["output"]))

    def model_name(text):
        return name_map.get(str(text), str(text))

    section = gripper.get("%s_positions" % args.phase) or {}
    arm_joints, hold_arm = [], {}
    for key, value in section.items():
        name = model_name(key)
        if int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)) >= 0:
            arm_joints.append(name)
            hold_arm[name] = float(value)
    gripper_keys = {str(k) for k in (gripper.get("open_positions") or {})} | \
                   {str(k) for k in (gripper.get("closed_positions") or {})}
    hold_gripper = {model_name(key): float(section[key]) for key in section if str(key) in gripper_keys}
    if not arm_joints:
        print(json.dumps({"error": "该相位没有臂关节键"}, ensure_ascii=False))
        return 2

    import yaml
    ff_cfg = ((yaml.safe_load((REPO / "config/ur5_simulation_baseline.yaml").read_text(encoding="utf-8"))
               or {}).get("gravity_feedforward") or {})
    common = {"hold_ms": int(ff_cfg["hold_ms"]), "tolerance_rad": float(ff_cfg["tolerance_rad"]),
              "max_passes": int(ff_cfg["max_passes"])}

    runs = {}
    for label, hold in (("A_夹爪执行器名也给", {**hold_arm, **hold_gripper}),
                        ("B_不给夹爪键(旧行为)", dict(hold_arm))):
        try:
            offsets, evidence = gravity_hold_ctrl(model, arm_joints, hold, **common)
            runs[label] = {"ok": True, "offsets": {k: round(float(v), 9) for k, v in offsets.items()},
                           "worst_residual_rad": evidence.get("worst_residual_rad"),
                           "residual_rad": evidence.get("residual_rad"),
                           "passes": evidence.get("passes") or evidence.get("pass_trace")}
        except Exception as error:  # noqa: BLE001 —— 门禁失败本身就是数据
            runs[label] = {"ok": False, "error": "%s: %s" % (type(error).__name__, error)}

    result = {"arm": args.arm, "phase": args.phase, "model": report["output"],
              "arm_joints": arm_joints, "gripper_channels": hold_gripper,
              "declared_tolerance_rad": common["tolerance_rad"],
              "note": ("A 与 B 的差别只有\"夹爪通道有没有写 ctrl\"；B 复现旧实现的静默跳过路径。"),
              "runs": runs}
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    for label, info in runs.items():
        if info.get("ok"):
            print("%-22s 最差残余=%-14s offsets=%s" %
                  (label, info.get("worst_residual_rad"), info.get("offsets")))
        else:
            print("%-22s 失败：%s" % (label, info.get("error")[:160]))
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
