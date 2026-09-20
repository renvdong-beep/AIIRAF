"""逐毫秒定位 HOME→APPROACH→DESCEND 三个关节空间段里"第一次接触"发生的时间点。

背景（`docs/ur5-adaptation-status.md` + `scripts/probe_tracking.py` 复现）：
- DESCEND 结束时 shoulder_lift 跟踪误差 0.4836 rad，方块被推到 [-0.7094, -0.0527, 0.0509]；
- 但**方块在 APPROACH 段就已经位移**，说明碰撞发生在更早的段里，DESCEND 的
  "跟踪误差"很可能是接触反力的**结果**而不是伺服增益不足的**原因**。

本脚本要直接判定因果方向，而不是靠推断。每个物理步同时记录：
  1) 五个臂执行器的 |实际 - 指令|（跟踪误差）；
  2) 所有涉及 box_01 的 contact（对方 geom 名、穿透深度、接触点）；
  3) 夹持区中点（4 个 pad box geom 中心的均值，与 IK 目标同口径）的实测位置；
  4) 方块质心位置（用于判定位移起始时刻）。

输出：先打印"首次接触 / 首次位移 / 误差首次超阈值"三条时间线的先后顺序，
再打印每个接触对在各段的首次出现，最后把完整证据写入 JSON。

用法：
  python3 scripts/probe_approach_contact.py \
      --scene build/models/ur5-pick-scene.xml \
      --duration-ms 12000 \
      --output build/calibration/ur5-approach-contact.json
"""

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


def quintic(start, end, duration_s, t):
    if t >= duration_s:
        return end
    x = t / duration_s
    return start + (end - start) * (10 * x**3 - 15 * x**4 + 6 * x**5)


def geom_id(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def geom_name(model, gid):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    return name if name else "<geom#%d>" % gid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="build/models/ur5-pick-scene.xml")
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument(
        "--output",
        default="build/calibration/ur5-approach-contact.json",
    )
    parser.add_argument(
        "--error-threshold-rad",
        type=float,
        default=0.05,
        help="跟踪误差首次超阈值的判定门限（rad）",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene_xml = root / args.scene
    scene_json = scene_xml.with_suffix(".json")
    scene = json.loads(scene_json.read_text(encoding="utf-8"))
    gripper = scene["gripper"]

    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    if int(model.nkey) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    box_geom = geom_id(model, "box_01_geom")
    if box_geom < 0:
        raise SystemExit("场景里找不到 box_01_geom")
    box_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, scene["target_id"]))
    workbench_geom = geom_id(model, "workbench")
    pad_names = list(scene["finger_geoms"]["pad_boxes"])
    pad_geoms = [geom_id(model, name) for name in pad_names]
    if any(gid < 0 for gid in pad_geoms):
        raise SystemExit("场景里缺少 pad geom: " + repr(pad_names))

    def actuator_of(name):
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))

    phases = [
        ("HOME_HOLD", "home_positions"),
        ("APPROACH", "approach_positions"),
        ("DESCEND", "grasp_positions"),
    ]

    dt = float(model.opt.timestep)
    segment_ms = max(1, args.duration_ms // 5)
    timeline = []
    first_contact = {}
    box_body_id = box_body
    box_initial = np.asarray(data.xpos[box_body_id], dtype=float).copy()
    first_displacement = None
    first_error = None
    pad_lowest_z = None

    def grip_center():
        return np.mean(
            [np.asarray(data.geom_xpos[gid], dtype=float) for gid in pad_geoms], axis=0
        )

    def sample(phase, t_ms, commanded, actuators):
        """记录一个物理步之后的全部观测。"""
        nonlocal first_displacement, first_error, pad_lowest_z
        errors = {}
        worst = 0.0
        worst_name = None
        for name in commanded:
            aid = actuators[name]
            joint_id = int(model.actuator_trnid[aid, 0])
            actual = float(data.qpos[int(model.jnt_qposadr[joint_id])])
            err = actual - float(commanded[name])
            errors[name] = round(err, 6)
            if abs(err) > abs(worst):
                worst, worst_name = err, name
        box_pos = np.asarray(data.xpos[box_body_id], dtype=float)
        geom_span = [
            float(data.geom_xpos[gid][2]) for gid in pad_geoms
        ]
        pad_lowest_z = min(geom_span)
        contacts = []
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            pair = (int(contact.geom1), int(contact.geom2))
            if box_geom not in pair:
                continue
            other = pair[1] if pair[0] == box_geom else pair[0]
            contacts.append(
                {
                    "other_geom": geom_name(model, other),
                    "dist_m": round(float(contact.dist), 6),
                    "position_m": [round(float(v), 6) for v in contact.pos],
                }
            )
        table_contacts = []
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            pair = (int(contact.geom1), int(contact.geom2))
            if workbench_geom in pair:
                other = pair[1] if pair[0] == workbench_geom else pair[0]
                table_contacts.append(geom_name(model, other))
        record = {
            "phase": phase,
            "t_ms": round(t_ms, 3),
            "worst_error_rad": round(worst, 6),
            "worst_joint": worst_name,
            "errors": errors,
            "grip_center_m": [round(float(v), 6) for v in grip_center()],
            "pad_lowest_z_m": round(pad_lowest_z, 6),
            "box_position_m": [round(float(v), 6) for v in box_pos],
            "box_contacts": contacts,
            "table_contacts": table_contacts,
        }
        timeline.append(record)

        if contacts and phase not in first_contact:
            first_contact[phase] = {
                "t_ms": record["t_ms"],
                "other_geoms": sorted({c["other_geom"] for c in contacts}),
                "worst_error_rad": record["worst_error_rad"],
                "worst_joint": record["worst_joint"],
                "grip_center_m": record["grip_center_m"],
                "box_position_m": record["box_position_m"],
            }
        if first_displacement is None:
            delta = float(np.linalg.norm(box_pos - box_initial))
            if delta > 1e-4:
                first_displacement = {
                    "phase": phase,
                    "t_ms": record["t_ms"],
                    "delta_m": round(delta, 6),
                    "box_position_m": record["box_position_m"],
                }
        if first_error is None and abs(worst) > args.error_threshold_rad:
            first_error = {
                "phase": phase,
                "t_ms": record["t_ms"],
                "worst_error_rad": record["worst_error_rad"],
                "worst_joint": record["worst_joint"],
                "box_contact_count": len(contacts),
            }

    phase_summary = []
    for phase, key in phases:
        targets = gripper[key]
        names = list(targets)
        actuators = {name: actuator_of(name) for name in names}
        if any(aid < 0 for aid in actuators.values()):
            missing = [n for n, a in actuators.items() if a < 0]
            raise SystemExit("执行器缺失: " + repr(missing))
        starts = [
            float(data.qpos[int(model.jnt_qposadr[int(model.actuator_trnid[actuators[n], 0])])])
            for n in names
        ]
        steps = max(1, int(math.ceil(segment_ms / 1000.0 / dt)))
        for step in range(1, steps + 1):
            elapsed = step * dt
            commanded = {
                n: quintic(starts[i], float(targets[n]), segment_ms / 1000.0, elapsed)
                for i, n in enumerate(names)
            }
            for name, value in commanded.items():
                data.ctrl[actuators[name]] = value
            mujoco.mj_step(model, data)
            sample(phase, elapsed * 1000.0, commanded, actuators)
        settle_ms = max(250, min(16000, segment_ms * 4))
        commanded = {n: float(targets[n]) for n in names}
        settle_steps = max(1, int(math.ceil(settle_ms / 1000.0 / dt)))
        for step in range(1, settle_steps + 1):
            for name, value in commanded.items():
                data.ctrl[actuators[name]] = value
            mujoco.mj_step(model, data)
            sample(phase, segment_ms + step * dt * 1000.0, commanded, actuators)
        mujoco.mj_forward(model, data)
        final_box = np.asarray(data.xpos[box_body_id], dtype=float)
        phase_summary.append(
            {
                "phase": phase,
                "settle_ms": settle_ms,
                "final_box_position_m": [round(float(v), 6) for v in final_box],
                "box_displacement_m": round(float(np.linalg.norm(final_box - box_initial)), 6),
                "final_grip_center_m": [round(float(v), 6) for v in grip_center()],
                "final_worst_error_rad": round(
                    max(
                        abs(
                            float(data.qpos[int(model.jnt_qposadr[int(model.actuator_trnid[actuators[n], 0])])])
                            - float(targets[n])
                        )
                        for n in names
                    ),
                    6,
                ),
            }
        )

    print("方块初始位置: %s" % np.round(box_initial, 6))
    print()
    print("三条时间线（谁先发生决定因果方向）:")
    print("  首次接触（按段）:")
    for phase in ("HOME_HOLD", "APPROACH", "DESCEND"):
        if phase in first_contact:
            entry = first_contact[phase]
            print(
                "    %-10s t=%8.3fms  对方 geom=%s  该时刻最差误差=%.6f rad (%s)"
                % (
                    phase,
                    entry["t_ms"],
                    ",".join(entry["other_geoms"]),
                    entry["worst_error_rad"],
                    entry["worst_joint"],
                )
            )
    print("  首次位移(>0.1mm): %s" % json.dumps(first_displacement, ensure_ascii=False))
    print("  误差首超阈值(%.2f rad): %s" % (args.error_threshold_rad, json.dumps(first_error, ensure_ascii=False)))
    print()
    print("各段末态:")
    for entry in phase_summary:
        print(
            "  %-10s settle=%-6d box=%s 位移=%.6fm 夹持区中点=%s 末态最差误差=%.6f rad"
            % (
                entry["phase"],
                entry["settle_ms"],
                entry["final_box_position_m"],
                entry["box_displacement_m"],
                entry["final_grip_center_m"],
                entry["final_worst_error_rad"],
            )
        )

    output = root / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "schema_version": "iraf.ur5-approach-contact/v1",
                "scene": str(scene_xml),
                "duration_ms": args.duration_ms,
                "segment_ms": segment_ms,
                "error_threshold_rad": args.error_threshold_rad,
                "box_initial_position_m": [round(float(v), 6) for v in box_initial],
                "box_gravity_compensation": bool(model.body_gravcomp[box_body_id]),
                "first_contact_by_phase": first_contact,
                "first_displacement": first_displacement,
                "first_error": first_error,
                "phase_summary": phase_summary,
                "timeline": timeline,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print()
    print("证据: %s" % output)


if __name__ == "__main__":
    main()
