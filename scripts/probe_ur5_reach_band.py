#!/usr/bin/env python3
"""UR5e 工具点在指定高度的**可达水平环带**（与 Piper 侧同一口径）。

口径来源（必须一致，否则两个站位的"够得到"不可比）：scene.yaml 的
`handoff_station_frame` 注释写明 Piper 的判据是
  「工具点在托盘顶面高度（z ≈ 0.347 m）的可达水平环带 r ∈ [0.002028, 0.594284] m
    （20000 次 FK 采样，臂基座在原点）」且**只按位置可达筛选，未约束工具姿态**
（build/iraf-a6a14/reach_probe.py）。本脚本对 UR5e 完全照搬这三条：
  ① 工具点 = **双侧 pad box 的中点**（与臂侧报告/联合构建器同一几何口径：
     `rq2f85_left_pad1` / `rq2f85_right_pad1`）；
  ② 只按位置筛（高度落在 [h−tol, h+tol] 内），**不约束姿态**；
  ③ 关节采样范围取自**模型自己**的 `jnt_range`（厂商声明，不手写不猜）。
输出 r 区间与方位覆盖率，用于给"第二台臂的停靠站位"提供数字依据。

用法：
  PYTHONPATH=src python3 scripts/probe_ur5_reach_band.py --heights 0.350372 0.375372
  → build/calibration/ur5-reach-band.json
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_MODEL = REPO / "build/models/ur5e_2f85/ur5e_2f85.xml"
PAD_GEOMS = ("rq2f85_left_pad1", "rq2f85_right_pad1")


def _geom_id(model, name):
    ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if ident < 0:
        raise SystemExit("模型里没有声明的指腹 geom: %s" % name)
    return int(ident)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--heights", type=float, nargs="+", default=[0.350372, 0.375372])
    parser.add_argument("--tol", type=float, default=0.01)
    parser.add_argument("--bins", type=int, default=36, help="方位扇区数（10° 一档）")
    parser.add_argument("--seed", type=int, default=20260929)
    parser.add_argument("--output", default="build/calibration/ur5-reach-band.json")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(pathlib.Path(args.model).resolve()))
    data = mujoco.MjData(model)

    # 臂关节 = 除 2F-85 手指/联动之外的 6 个主动关节：按**模型自己的驱动器**取，
    # 避免手写关节名（哪 6 个是"臂"由模型声明：actuator 指向的 hinge 关节）。
    arm_joints = []
    for actuator in range(int(model.nu)):
        joint_id = int(model.actuator_trnid[actuator][0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name and name.endswith("_joint") and not name.startswith("rq2f85"):
            if joint_id not in arm_joints:
                arm_joints.append(joint_id)
    if len(arm_joints) != 6:
        raise SystemExit("从模型声明里取到的臂关节不是 6 个：%s"
                         % [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in arm_joints])
    low = np.array([model.jnt_range[j][0] for j in arm_joints], dtype=float)
    high = np.array([model.jnt_range[j][1] for j in arm_joints], dtype=float)
    unlimited = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                 for j, lo, hi in zip(arm_joints, low, high) if lo == 0.0 and hi == 0.0]
    if unlimited:
        # 厂商声明为"无限制"（range=[0,0]）的关节：按 UR5e 规格 ±2π 采样（显式登记，不静默）
        for index, joint_id in enumerate(arm_joints):
            if low[index] == 0.0 and high[index] == 0.0:
                low[index], high[index] = -2 * np.pi, 2 * np.pi
    pads = [_geom_id(model, name) for name in PAD_GEOMS]

    rng = np.random.default_rng(args.seed)
    results = {
        "schema_version": "iraf.ur5-reach-band/v1",
        "model": str(pathlib.Path(args.model).resolve().relative_to(REPO)),
        "samples": args.samples,
        "arm_joints": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in arm_joints],
        "joint_range_rad": {str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)): [float(lo), float(hi)]
                            for j, lo, hi in zip(arm_joints, low, high)},
        "joint_range_from_model": [str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j))
                                   for j in arm_joints],
        "unlimited_joints_assumed_pm_2pi": unlimited,
        "tool_point": "双侧 pad box 中点（%s / %s）" % PAD_GEOMS,
        "attitude_constrained": False,
        "bands": {},
    }

    # 采样（向量化不了 FK，只能逐次 mj_forward；20000 次在本机约十几秒）
    q = rng.uniform(low, high, size=(args.samples, len(arm_joints)))
    heights = {}
    for h in args.heights:
        heights[float(h)] = {"r_min": None, "r_max": None, "n": 0, "bins": [0] * args.bins}
    for row in q:
        data.qpos[:] = 0.0
        for value, joint_id in zip(row, arm_joints):
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
        mujoco.mj_forward(model, data)
        mid = (np.asarray(data.geom_xpos[pads[0]], dtype=float)
               + np.asarray(data.geom_xpos[pads[1]], dtype=float)) / 2.0
        for h, entry in heights.items():
            if abs(mid[2] - h) > args.tol:
                continue
            radius = float(np.hypot(mid[0], mid[1]))
            if entry["r_min"] is None or radius < entry["r_min"]:
                entry["r_min"] = radius
            if entry["r_max"] is None or radius > entry["r_max"]:
                entry["r_max"] = radius
            entry["n"] += 1
            angle = float(np.arctan2(mid[1], mid[0]))
            bucket = int((angle + np.pi) / (2 * np.pi) * args.bins) % args.bins
            entry["bins"][bucket] += 1
    for h, entry in heights.items():
        covered = sum(1 for count in entry["bins"] if count > 0)
        results["bands"]["z=%.6f" % h] = {
            "z_center_m": h,
            "z_tolerance_m": args.tol,
            "samples_in_band": entry["n"],
            "r_min_m": entry["r_min"],
            "r_max_m": entry["r_max"],
            "bearing_bins_covered": covered,
            "bearing_bins_total": args.bins,
            "bearing_coverage": round(covered / args.bins, 4),
        }

    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    for key, entry in results["bands"].items():
        print("%-16s n=%5d  r ∈ [%s, %s] m  方位覆盖 %d/%d"
              % (key, entry["samples_in_band"],
                 "None" if entry["r_min_m"] is None else "%.6f" % entry["r_min_m"],
                 "None" if entry["r_max_m"] is None else "%.6f" % entry["r_max_m"],
                 entry["bearing_bins_covered"], entry["bearing_bins_total"]))
    print("WROTE %s" % out.relative_to(REPO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
