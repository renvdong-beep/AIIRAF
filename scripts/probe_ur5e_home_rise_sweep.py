#!/usr/bin/env python3
"""home 抬高量（`grasp.home_rise_m`）扫描：找本联合场景里**可达且不侵入载荷**的值（2026-09-30 §11.28）。

背景：UR5e 基线在自己场景里 home_rise_m=0.30 收敛（残差 1.5e-07 m）；换到联合场景
（目标距基座 0.45 m、载荷在狗背托盘里）后 home 目标不可达，解出的位形夹持区只在 0.420188 m
（目标 0.835372）且与载荷重叠 31.6 mm ⇒ 运行期把载荷扫出托盘。

本探针做两件事（都对同一份基线，只改 home_rise_m）：
1. 直接调用**声明的求解器入口**（`scripts/build_robot_baseline.py: build_reference_poses`），
   打印它返回的每相位残差（approach/grasp/lift 自带；home 看 `home_solved`）与是否收敛；
2. 用**臂模型**独立 FK 每个 rise 下的 home 关节解，量「夹持区中点」与「approach 中点 + rise」
   的差（独立于求解器自报值），以及该位形与载荷的最小间距。

用法：
  PYTHONPATH=src python3 scripts/probe_ur5e_home_rise_sweep.py \
      [--rises 0.05,0.10,0.15,0.20,0.25,0.30] [--payload 0.4465018,0.446055507,0.385109022]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib
import sys

import mujoco
import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
SOLVER = REPO / "scripts/build_robot_baseline.py"
BASELINE = REPO / "config/ur5_simulation_baseline.yaml"
ARM_MODEL = REPO / "build/models/ur5-pick-scene.xml"
#: 本联合场景里 UR5e 的目标（基座系局部 xy + 世界 z），与 scene.yaml 声明一致。
TARGET_XY = [-0.45, 0.0]
TARGET_Z = 0.375372
PAD_BOXES = ["rq2f85_left_pad1", "rq2f85_left_pad2",
             "rq2f85_right_pad1", "rq2f85_right_pad2"]


def _load_module():
    spec = importlib.util.spec_from_file_location("build_robot_baseline", SOLVER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fk_pad_mid(model, data, names):
    points = []
    for name in names:
        ident = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
        if ident < 0:
            return None, "geom 不存在: %s" % name
        points.append(np.asarray(data.geom_xpos[ident], dtype=float))
    return np.mean(np.asarray(points), axis=0), None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rises", default="0.05,0.10,0.15,0.20,0.25,0.30")
    parser.add_argument("--payload", default="0.4465018,0.446055507,0.385109022")
    parser.add_argument("--output", default="build/diagnostics/ur5e-home-rise-sweep.json")
    args = parser.parse_args()

    module = _load_module()
    base_doc = yaml.safe_load(BASELINE.read_text(encoding="utf-8"))
    payload = np.asarray([float(v) for v in args.payload.split(",")], dtype=float)
    arm_model = mujoco.MjModel.from_xml_path(str(ARM_MODEL))
    arm_data = mujoco.MjData(arm_model)
    box_id = int(mujoco.mj_name2id(arm_model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    box_geoms = [i for i in range(int(arm_model.ngeom))
                 if int(arm_model.geom_bodyid[i]) == box_id]

    rows = []
    for rise in [float(v) for v in args.rises.split(",") if v.strip()]:
        doc = json.loads(json.dumps(base_doc))
        doc.setdefault("grasp", {})["home_rise_m"] = rise
        row = {"home_rise_m": rise}
        try:
            reference = module.build_reference_poses(
                REPO, doc, target_xy_override_m=list(TARGET_XY),
                target_z_override_m=float(TARGET_Z))
        except Exception as error:  # noqa: BLE001 —— 求解器内部门禁失败也是数据
            row["solver_error"] = "%s: %s" % (type(error).__name__, error)
            rows.append(row)
            continue
        row["return_keys"] = sorted(reference.keys())
        for phase in ("approach", "grasp", "lift"):
            packed = reference.get(phase) or {}
            row["%s_residual_m" % phase] = packed.get("position_error_m")
            row["%s_target_m" % phase] = packed.get("target_m")
            row["%s_solved_m" % phase] = packed.get("finger_center_m")
        solved = reference.get("home_solved")
        row["home_solved_present"] = isinstance(solved, dict)
        if isinstance(solved, dict):
            row["home_solved_residual_m"] = solved.get("position_error_m")
            row["home_solved_target_m"] = solved.get("target_m")
            row["home_solved_center_m"] = solved.get("finger_center_m")
        # 独立 FK（臂模型）：把 home 关节解摆上去，量夹持区中点
        home_joints = reference.get("home") or {}
        for name, value in home_joints.items():
            jid = int(mujoco.mj_name2id(arm_model, mujoco.mjtObj.mjOBJ_JOINT, str(name)))
            if jid >= 0:
                arm_data.qpos[int(arm_model.jnt_qposadr[jid])] = float(value)
        mujoco.mj_forward(arm_model, arm_data)
        mid, err = _fk_pad_mid(arm_model, arm_data, PAD_BOXES)
        if mid is None:
            row["fk_error"] = err
        else:
            row["fk_home_pad_mid_m"] = [round(float(v), 9) for v in mid]
            approach_center = (reference.get("approach") or {}).get("finger_center_m")
            if approach_center:
                expect = np.asarray(approach_center, dtype=float) + np.array([0.0, 0.0, rise])
                row["expected_home_center_m"] = [round(float(v), 9) for v in expect]
                row["fk_minus_expected_m"] = round(float(np.linalg.norm(mid - expect)), 9)
            # 载荷本地系 = 世界系平移：臂模型里的方块在初始位，故把载荷平移到实测位姿再量间距
            if box_geoms:
                box_pos = np.asarray(arm_data.xpos[box_id], dtype=float)
                shift = payload - box_pos
                worst = float("inf")
                pair = None
                for index in range(int(arm_model.ngeom)):
                    if index in box_geoms:
                        continue
                    name = mujoco.mj_id2name(arm_model, mujoco.mjtObj.mjOBJ_GEOM, index)
                    if not name or name.startswith("rq2f85_") or name.startswith("box"):
                        continue
                    centre = np.asarray(arm_data.geom_xpos[index], dtype=float) + shift
                    gap = float(np.linalg.norm(centre - payload)
                                - float(arm_model.geom_size[index][0]) - 0.025)
                    if gap < worst:
                        worst, pair = gap, name
                row["min_geom_gap_to_payload_m_estimate"] = round(worst, 9)
                row["min_geom_gap_pair"] = pair
        rows.append(row)

    result = {"target_xy_local_m": TARGET_XY, "target_z_m": TARGET_Z,
              "payload_world_m": [float(v) for v in payload],
              "note": ("home 残差看求解器自报（approach/grasp/lift 自带、home 看 home_solved）；"
                       "`fk_minus_expected_m` 是独立 FK 的交叉核对（不依赖求解器自报值）；"
                       "最小间距为**球形近似**（geom_size 半长），仅用于判\"是否压在载荷里\"。"),
              "rows": rows}
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(rows, ensure_ascii=False, indent=2)[:4000])
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
