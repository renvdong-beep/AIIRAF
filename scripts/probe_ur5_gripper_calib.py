"""标定与诊断：夹爪到底需要多少 ctrl 才能动、方块该放哪里才能被夹住。

前一轮实测（verify_ur5_gripper_force.py）暴露出两个必须先用数据解决的问题：

1. `ctrl=255` 时 pad 间距几乎不变（开合行程 ~0）。到底是"执行器没动"，
   还是"不动才是开"，必须用 driver 关节角 vs ctrl 的曲线回答。
2. 真正"双指夹住"要求 pad 中心与方块中心对齐。因此必须标定
   `pinch` site → 双侧 pad 中点的偏移，以及"闭合后 pad 内表面间隙"
   对应的**可夹持方块边长**，才能给出正确的方块摆放位置。

本脚本只做标定与诊断，不修改任何模型；输出直接用于下一步的场景生成。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ACTUATOR_NAME = "rq2f85_fingers_actuator"
DRIVER_JOINTS = ("rq2f85_right_driver_joint", "rq2f85_left_driver_joint")
PAD_GEOMS = ("rq2f85_left_pad1", "rq2f85_right_pad1")


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def _qpos_adr(model, name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError("缺少 joint: " + name)
    return int(model.jnt_qposadr[joint_id])


def _site(model, name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError("缺少 site: " + name)
    return int(site_id)


def pad_geometry(model, data, pad_names=PAD_GEOMS):
    """双侧 pad 的几何量：中心、间距、内表面间隙。

    pad 是 box(0.011, 0.004, 0.009375)。两个 pad 的开合方向是**局部 y**，
    因此内表面间隙 = 中心距 - 2 * size_y（而不是用全局 y 分量）。
    """
    left = _geom(model, pad_names[0])
    right = _geom(model, pad_names[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    axis = centres[1] - centres[0]
    distance = float(np.linalg.norm(axis))
    return {
        "left_centre": centres[0],
        "right_centre": centres[1],
        "centre": (centres[0] + centres[1]) / 2.0,
        "centre_distance_m": distance,
        "gap_m": distance - 2.0 * half_y,
        "half_y_m": half_y,
        "other_axes_m": [
            float(model.geom_size[left][0]),
            float(model.geom_size[left][2]),
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument("--settle-steps", type=int, default=2000)
    parser.add_argument(
        "--output", type=Path, default=Path("build/calibration/ur5-gripper-calib.json")
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(Path(args.model).resolve()))
    data = mujoco.MjData(model)
    actuator_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    )
    if actuator_id < 0:
        raise ValueError("缺少 " + ACTUATOR_NAME)
    print("forcerange:", [float(v) for v in model.actuator_forcerange[actuator_id]])
    print("ctrlrange:", [float(v) for v in model.actuator_ctrlrange[actuator_id]])
    print("gainprm:", [round(float(v), 6) for v in model.actuator_gainprm[actuator_id][:3]])
    print("biasprm:", [round(float(v), 6) for v in model.actuator_biasprm[actuator_id][:3]])

    # --- 1) ctrl -> driver 关节角与 pad 间隙（臂不动，只看夹爪）---
    rows = []
    for value in list(range(0, 256, 15)) + [255, 254, 250]:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)
        open_row = pad_geometry(model, data)
        data.ctrl[actuator_id] = float(value)
        for _ in range(int(args.settle_steps)):
            mujoco.mj_step(model, data)
        closed_row = pad_geometry(model, data)
        rows.append(
            {
                "ctrl": float(value),
                "gap_before_m": round(open_row["gap_m"], 6),
                "gap_after_m": round(closed_row["gap_m"], 6),
                "gap_delta_m": round(closed_row["gap_m"] - open_row["gap_m"], 6),
                "driver_right_rad": round(
                    float(data.qpos[_qpos_adr(model, DRIVER_JOINTS[0])]), 6
                ),
                "driver_left_rad": round(
                    float(data.qpos[_qpos_adr(model, DRIVER_JOINTS[1])]), 6
                ),
            }
        )
    print("\nctrl -> gap / driver")
    for row in rows:
        print(
            "  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f) driver_r=%+.4f driver_l=%+.4f"
            % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
               row["gap_delta_m"], row["driver_right_rad"], row["driver_left_rad"])
        )

    # --- 2) 行程上下限对应的 pad 间隙 ---
    gaps = [row["gap_after_m"] for row in rows]
    min_gap = min(gaps)
    max_gap = max(gaps)
    print("\ngap range: %.6f .. %.6f (span %.6f)" % (min_gap, max_gap, max_gap - min_gap))

    # --- 3) pinch site 与 pad 中心的偏移（用于反推方块摆放位置）---
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    open_row = pad_geometry(model, data)
    pinch_pos = np.asarray(
        data.site_xpos[_site(model, "pinch")], dtype=float
    ).copy()
    offset = pinch_pos - open_row["centre"]
    print("\npinch site world:", [round(float(v), 6) for v in pinch_pos])
    print("pad centre  world:", [round(float(v), 6) for v in open_row["centre"]])
    print("pinch - pad_centre:", [round(float(v), 6) for v in offset])

    # --- 4) 闭合状态下的 pad 中心（决定方块该放多高）---
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.ctrl[actuator_id] = 255.0
    for _ in range(int(args.settle_steps)):
        mujoco.mj_step(model, data)
    closed_row = pad_geometry(model, data)
    closed_pinch = np.asarray(
        data.site_xpos[_site(model, "pinch")], dtype=float
    ).copy()
    print("\nclosed pad centre world:", [round(float(v), 6) for v in closed_row["centre"]])
    print("closed pinch world:", [round(float(v), 6) for v in closed_pinch])
    print("closed gap:", round(closed_row["gap_m"], 6))
    print("pad 内表面沿开合轴的半厚（可夹持物半边长上限）:", round(closed_row["half_y_m"], 6))

    report = {
        "schema_version": "iraf.ur5-gripper-calib/v1",
        "actuator": {
            "name": ACTUATOR_NAME,
            "forcerange": [float(v) for v in model.actuator_forcerange[actuator_id]],
            "ctrlrange": [float(v) for v in model.actuator_ctrlrange[actuator_id]],
            "gainprm": [round(float(v), 6) for v in model.actuator_gainprm[actuator_id][:3]],
            "biasprm": [round(float(v), 6) for v in model.actuator_biasprm[actuator_id][:3]],
        },
        "gap_range_m": [round(min_gap, 6), round(max_gap, 6)],
        "gap_span_m": round(max_gap - min_gap, 6),
        "pad_half_y_m": round(closed_row["half_y_m"], 6),
        "pinch_minus_pad_centre_open_m": [round(float(v), 6) for v in offset],
        "open_pad_centre_world_m": [round(float(v), 6) for v in open_row["centre"]],
        "closed_pad_centre_world_m": [
            round(float(v), 6) for v in closed_row["centre"]
        ],
        "ctrl_sweep": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
