#!/usr/bin/env python3
"""`ctrl` 语义的**度量校验**（2026-09-30 §11.67）。

背景：§11.64 用 `|qpos − ctrl|` 当"臂没跟上多少"，读数从 0.023 涨到 0.5 rad；但 §11.67 发现
「臂按指令速率走、误差却持续增大」在位置伺服上自相矛盾 ⇒ **先验证这个差是否有物理含义**。
本探针在该位形上：设定 `ctrl` = 目标关节角 → `mj_step` 到稳态 → 比较 `qpos` 与 `ctrl`。
  · 若 `qpos ≈ ctrl`（残差 ~1e-3 或更小）⇒ 指标有效，§11.64 的 0.5 rad 是真滞后；
  · 若差得远 ⇒ 指标口径错（执行器类型/gear/单位），§11.64 的结论必须撤回。
只读、不改声明/产物。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_ctrl_semantics.py [位形名] [步数]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco

REPO = Path(__file__).resolve().parents[1]
JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
JOINT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
POSES = (("place_above", "place_above_positions"), ("place_descend", "place_descend_positions"))


def main(argv):
    wanted = argv[1] if len(argv) > 1 else "place_above"
    steps = int(argv[2]) if len(argv) > 2 else 3000
    gripper = ((json.loads(JOINT_JSON.read_text())["manipulation"]["per_robot"]["ur5e"]) or {}).get("gripper") or {}
    model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    data = mujoco.MjData(model)
    for label, key in POSES:
        if label != wanted:
            continue
        section = gripper.get(key) or {}
        data.qpos[:] = 0.0
        pairs = []
        for name, value in section.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if jid < 0:
                continue
            aid = next((i for i in range(int(model.nu))
                        if int(model.actuator_trnid[i, 0]) == jid), -1)
            if aid < 0:
                continue
            if model.actuator_dyntype[aid] != 0 or model.actuator_biastype[aid] not in (1, 2):
                print("   [注意] %s 执行器类型非 position-affine: dyntype=%d biastype=%d"
                      % (name, model.actuator_dyntype[aid], model.actuator_biastype[aid]))
            pairs.append((str(name), jid, aid, float(value)))
            data.qpos[int(model.jnt_qposadr[jid])] = float(value)
            data.ctrl[aid] = float(value)
        mujoco.mj_forward(model, data)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        print("== %s（ctrl = 目标关节角，步进 %d 步后）" % (label, steps))
        print("   %-30s %12s %12s %12s %12s" % ("关节", "target(ctrl)", "qpos(稳态)", "qpos−ctrl", "gear"))
        for name, jid, aid, value in sorted(pairs):
            q = float(data.qpos[int(model.jnt_qposadr[jid])])
            print("   %-30s %12.6f %12.6f %12.6f %12.3f"
                  % (name, value, q, q - value, float(model.actuator_gear[aid, 0])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
