#!/usr/bin/env python3
"""s07 放置航点的**构建期 FK 对账**（2026-09-30 §11.58）。

用途：把联合报告里 ur5e 的 `place_{transit,above,descend,retreat}_positions` 逐段设进**联合模型**做 FK，
打印**指腹中点**的世界坐标，与声明的接收体（落点垫）顶面对齐情况。
这样可区分「构建期解本身就落在错的地方」与「运行期没到位」——两者修法完全不同。

用法（仓库根）：
    PYTHONPATH=src python3 scripts/probe_ur5e_place_waypoint_fk.py \
        [build/scenes/handoff_lab/handoff_lab_joint.json] [build/scenes/handoff_lab/handoff_lab_joint.xml]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEFAULT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
DEFAULT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
PAD_GEOMS = ("ur5e_rq2f85_left_pad", "ur5e_rq2f85_right_pad")


def main(argv):
    report = Path(argv[1]) if len(argv) > 1 else DEFAULT_JSON
    xml = Path(argv[2]) if len(argv) > 2 else DEFAULT_XML
    doc = json.loads(report.read_text())
    per_robot = (doc.get("manipulation") or {}).get("per_robot") or {}
    gripper = ((per_robot.get("ur5e") or {}).get("gripper")) or {}
    pad_geoms = tuple(str(gripper.get(key) or "")
                      for key in ("left_finger_geom", "right_finger_geom"))
    if not all(pad_geoms):
        pad_geoms = PAD_GEOMS
    targets = (doc.get("place_targets") or {}).get("targets") or []
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)

    def pad_mid(positions):
        data.qpos[:] = 0.0
        for name, value in positions.items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
            if joint_id < 0:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                             str(name).replace("_joint", ""))
            if joint_id >= 0:
                data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
        mujoco.mj_forward(model, data)
        points = []
        for geom_name in pad_geoms:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if geom_id >= 0:
                points.append(np.asarray(data.geom_xpos[geom_id], dtype=float))
        return (points[0] + points[1]) / 2.0 if len(points) == 2 else None

    rec = ((per_robot.get("ur5e") or {}).get("reference_pose_resolution") or {}).get("place_reference") or {}
    print("[指腹 geom] %s" % (list(pad_geoms),))
    print("[接收体] %s" % json.dumps(
        [{"id": t.get("id"), "nominal_pose_m": t.get("nominal_pose_m"), "size_m": t.get("size_m")}
         for t in targets], ensure_ascii=False))
    print("[选择] place_target_id = %s" % rec.get("place_target_id"))
    print("[航点高度口径] %s" % json.dumps(rec.get("transit_height"), ensure_ascii=False))
    print("%-26s %-34s" % ("相位", "指腹中点(世界)"))
    for key in ("place_transit_positions", "place_above_positions",
                "place_descend_positions", "place_retreat_positions"):
        positions = gripper.get(key)
        if not positions:
            print("%-26s %s" % (key, "(缺)"))
            continue
        mid = pad_mid(positions)
        print("%-26s %s" % (key, ("None" if mid is None else np.round(mid, 6).tolist())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
