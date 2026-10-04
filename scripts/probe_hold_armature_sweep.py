#!/usr/bin/env python3
"""静态保持残余的**armature 扫描**（2026-09-30 §11.61 ①）。

背景：`probe_place_feedforward.py` 判死——本臂在联合模型里、在**放置位形**上的静态保持地板
≈ 0.031 rad（`gravity_hold_ctrl` 4 轮后残差 0.030678510 / 限 0.001）。本探针回答：
**调 `armature_min_kg_m2` 能不能压下来，以及会不会打坏已经在用的抓取位形。**

口径：直接在联合模型上改臂关节的 `dof_armature`，对**同一批位形**（pick 的 grasp/lift + place 的
above/descend）跑 `iraf_core.kinematics.gravity_hold_ctrl`，逐位形记录最坏残余与是否通过。
不改任何声明/产物，纯只读测量。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_hold_armature_sweep.py [armature 列表，逗号分隔]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from iraf_core.kinematics import gravity_hold_ctrl  # noqa: E402

JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
JOINT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
BASELINE = REPO / "config/ur5_simulation_baseline.yaml"
PREFIX = "ur5e_"
POSES = (("pick_grasp", "grasp_positions"), ("pick_lift", "lift_positions"),
         ("place_above", "place_above_positions"), ("place_descend", "place_descend_positions"))


def main(argv):
    values = [float(v) for v in (argv[1].split(",") if len(argv) > 1 else
                                 ["0.5", "1.0", "2.0", "5.0", "20.0"])]
    baseline = yaml.safe_load(BASELINE.read_text())
    ff = baseline["gravity_feedforward"]
    grip = ((json.loads(JOINT_JSON.read_text())["manipulation"]["per_robot"]["ur5e"]) or {}).get("gripper") or {}
    grip_keys = set(grip.get("open_positions") or {}) | set(grip.get("closed_positions") or {})

    poses = {}
    for label, key in POSES:
        section = grip.get(key) or {}
        arm = {str(k)[len(PREFIX):]: float(v) for k, v in section.items() if str(k) not in grip_keys}
        gripper_here = {str(k): float(v) for k, v in section.items() if str(k) in grip_keys}
        if arm:
            poses[label] = (arm, gripper_here)
    if not poses:
        print("没有可测位形（联合报告缺 place_*_positions？先重建产物）")
        return 1

    base = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    arm_joint_ids = []
    for name in list(next(iter(poses.values()))[0]):
        jid = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_JOINT, PREFIX + name)
        if jid >= 0:
            arm_joint_ids.append(jid)
    print("[臂关节] %d 个；hold_ms=%s tolerance_rad=%s max_passes=%s"
          % (len(arm_joint_ids), ff["hold_ms"], ff["tolerance_rad"], ff["max_passes"]))
    print("[位形] %s" % sorted(poses))
    print("%-8s %-14s %-14s %s" % ("armature", "位形", "最坏残余(rad)", "通过"))
    for value in values:
        model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
        for jid in arm_joint_ids:
            model.dof_armature[int(model.jnt_dofadr[jid])] = float(value)
        for label, (arm, gripper_here) in sorted(poses.items()):
            hold = {PREFIX + str(k): float(v) for k, v in arm.items()}
            hold.update({PREFIX + str(k): float(v) for k, v in gripper_here.items()})
            try:
                _, evidence = gravity_hold_ctrl(
                    model, [PREFIX + str(k) for k in arm], hold,
                    hold_ms=int(ff["hold_ms"]), tolerance_rad=float(ff["tolerance_rad"]),
                    max_passes=int(ff["max_passes"]))
                worst = _worst(evidence)
                print("%-8.3f %-14s %-14.9f %s" % (value, label, worst,
                                                   worst <= float(ff["tolerance_rad"])))
            except Exception as error:  # noqa: BLE001 —— 失败即证据
                msg = str(error).replace("\n", " ")
                worst = None
                if "最大关节误差 " in msg:
                    try:
                        worst = float(msg.split("最大关节误差 ")[1].split()[0])
                    except (IndexError, ValueError):
                        worst = None
                print("%-8.3f %-14s %-14s 拒绝：%s" % (value, label, worst, msg[:80]))
    return 0


def _worst(evidence):
    """从 `gravity_hold_ctrl` 的证据里取最坏残余（兼容 dict / 标量两种键）。"""
    for key in ("residual_rad", "worst_residual_rad", "residuals_rad"):
        value = (evidence or {}).get(key)
        if isinstance(value, dict) and value:
            return max(abs(float(v)) for v in value.values())
        if isinstance(value, (int, float)):
            return abs(float(value))
    return float("nan")


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
