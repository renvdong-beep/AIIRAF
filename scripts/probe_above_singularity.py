#!/usr/bin/env python3
"""`above` 段指令路径上的**可操作度/条件数**（2026-09-30 §11.75）。

动机：`above` 段命令速率不高（wrist_1 峰值 ~0.28 rad/s），静态伺服也很快，可臂就是跟不上
（§11.74：无接触即 −0.455 rad）。经典解释之一是**路径经过奇异位形附近** —— 同样的笛卡尔运动
需要趋于无穷的关节速度，臂自然"跟不上"。

做法：把 `TRACE_CMD` 里的**指令目标**逐拍设进联合模型（只设臂关节），对**指腹中点**求
`mj_jac`（平移 3×6），报告最小奇异值 σ_min 与其倒数（速度放大因子）。σ_min → 0 即奇异附近。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_above_singularity.py [日志] [段号]
    缺省：build/diagnostics/above-campaign-round1.log 段27（= place_above_positions）
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"


def main(argv):
    log = Path(argv[1]) if len(argv) > 1 else REPO / "build/diagnostics/above-campaign-round1.log"
    index = int(argv[2]) if len(argv) > 2 else 27
    rows = [json.loads(line.split(" ", 1)[1]) for line in log.read_text(errors="replace").splitlines()
            if line.startswith("TRACE_CMD ")]
    samples = rows[index]["samples"]
    model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    data = mujoco.MjData(model)
    # 指腹中点（用两侧 pad geom 的中点作为 EE 参考点）
    pad_ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
               for name in ("rq2f85_left_pad1", "rq2f85_right_pad1")]
    arm_joints = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
                  for name in ("ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint",
                               "ur5e_elbow_joint", "ur5e_wrist_1_joint",
                               "ur5e_wrist_2_joint", "ur5e_wrist_3_joint")]
    dof_cols = [int(model.jnt_dofadr[jid]) for jid in arm_joints]
    print("段%d：%d 拍；EE = 两侧 pad geom 中点；只取臂 6 个 dof" % (index, len(samples)))
    print("%-6s %-12s %-12s %-12s" % ("step", "σ_min", "1/σ_min", "σ_max/σ_min"))
    worst = (1e9, None)
    for sample in samples:
        if sample["step"] % 50:
            continue
        for name, joint_id in zip(("ur5e_shoulder_pan", "ur5e_shoulder_lift", "ur5e_elbow",
                                   "ur5e_wrist_1", "ur5e_wrist_2", "ur5e_wrist_3"), arm_joints):
            if name in sample["sample"]:
                value = sample["sample"][name]
                data.qpos[int(model.jnt_qposadr[joint_id])] = float(value[0] - value[1])
        mujoco.mj_forward(model, data)
        point = 0.5 * (np.asarray(data.geom_xpos[pad_ids[0]], dtype=float)
                       + np.asarray(data.geom_xpos[pad_ids[1]], dtype=float))
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacp, jacr, point, int(model.geom_bodyid[pad_ids[0]]))
        jac = jacp[:, dof_cols]
        singular = np.linalg.svd(jac, compute_uv=False)
        smin, smax = float(singular[-1]), float(singular[0])
        if smin < worst[0]:
            worst = (smin, sample["step"])
        if sample["step"] % 100 == 0 or smin < 0.02:
            print("%-6d %-12.6f %-12.3f %-12.1f" % (sample["step"], smin,
                                                    (1.0 / smin if smin > 1e-12 else float("inf")),
                                                    (smax / smin if smin > 1e-12 else float("inf"))))
    print("最坏 σ_min = %.6f @step %s ⇒ 速度放大 1/σ_min = %.2f（越接近奇异越放大）"
          % (worst[0], worst[1], 1.0 / worst[0] if worst[0] > 1e-12 else float("inf")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
