#!/usr/bin/env python3
"""UR5e 卸载段"扫掉托盘载荷"的几何取证（I3，2026-09-30）。

症状（`nominal --world joint`，`IRAF_DEBUG_PICK=1`）：s06 的相位序列里，载荷位置
   HOME_HOLD 末：`box_01` z = 0.38286116878166887（**在托盘里**）
   APPROACH 末：`box_01` z = 0.024784489159567647（**已落到台面**，xy = (−0.23301, −0.20866)）
⇒ 载荷是在 **home → approach 这一段**被臂的几何扫出托盘的（不是抓取偏差）。

本探针用**同一份联合模型**回答两件事：
1. 四个相位各自的**指腹中点**（`pad_boxes` 中点）在基座系 FK 下的高度、以及相对载荷的间距；
   由此看出 home 与 approach 在 z 上的**先后顺序**（若 home 在载荷**下方**，那这段运动必然穿过载荷）；
2. 沿"上一层位形 → 本相位"的**关节空间五次插值**逐样本 FK，量**被驱动臂的全部 geom 与载荷
   的最小间距**（`mj_geomDistance`，负值 = 侵入）。给出**首次侵入的样本位置/时刻/几何对**。

口径与纪律：
· 载荷按**运行期实测位姿**摆好（默认取 §11.27 实测值；`--payload` 可覆盖），因为它已经不在初始台面位；
· 臂基座在联合世界里是静止的 ⇒ 只按报告里的关节解 FK 即可，不需要复现狗/托盘状态；
· 输出 JSON（`--output`），不打印结论性措辞：判读留给读者。

用法：
  PYTHONPATH=src python3 scripts/probe_ur5e_unload_sweep.py \
      [--report build/scenes/handoff_lab/handoff_lab_joint.json] [--arm ur5e] \
      [--payload 0.4465018,0.446055507,0.385109022] \
      [--output build/diagnostics/ur5e-unload-sweep.json]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]

#: 逐相位推进的顺序（与后端 `pick_object` 一致：home → approach → grasp → lift）。
PHASE_ORDER = ("home_positions", "approach_positions", "grasp_positions", "lift_positions")
PHASE_LABEL = {
    "home_positions": "home",
    "approach_positions": "approach",
    "grasp_positions": "grasp",
    "lift_positions": "lift",
}


def _joint_ids(model, names):
    return [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))) for name in names]


def _pad_midpoint(model, data, pad_geoms):
    points = []
    for name in pad_geoms:
        ident = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(name)))
        if ident < 0:
            raise SystemExit("夹持区 geom 在联合模型里不存在: %s" % name)
        points.append(np.asarray(data.geom_xpos[ident], dtype=float))
    return np.mean(np.asarray(points), axis=0)


def _payload_geoms(model, box_body):
    """载荷 body 自己的全部 geom id（用于量它与臂的最小间距）。"""
    box_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(box_body)))
    if box_id < 0:
        raise SystemExit("载荷 body 在联合模型里不存在: %s" % box_body)
    return [index for index in range(int(model.ngeom))
            if int(model.geom_bodyid[index]) == box_id]


def _arm_geoms(model, box_geoms, exclude):
    """被驱动臂（前缀）的全部 geom：按宿主 body 名带前缀界定，排除夹爪 pad 自己。"""
    out = []
    for index in range(int(model.ngeom)):
        if index in box_geoms or index in exclude:
            continue
        body = int(model.geom_bodyid[index])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        if name.startswith("ur5e"):
            out.append(index)
    return out


def _min_clearance(model, data, arm_geoms, box_geoms):
    worst = float("inf")
    pair = None
    for index in arm_geoms:
        for box in box_geoms:
            try:
                dist = float(mujoco.mj_geomDistance(model, data, index, box, 10.0, None))
            except Exception:  # noqa: BLE001 —— 老版本无该 API 时显式失败更清晰
                raise SystemExit("当前 mujoco 不支持 mj_geomDistance，无法量间距")
            if dist < worst:
                worst, pair = dist, (index, box)
    return worst, pair


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", default="build/scenes/handoff_lab/handoff_lab_joint.json")
    parser.add_argument("--arm", default="ur5e")
    parser.add_argument("--payload", default="0.4465018,0.446055507,0.385109022",
                        help="载荷**运行期实测**位置（世界系 xyz，逗号分隔）")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--output", default="build/diagnostics/ur5e-unload-sweep.json")
    args = parser.parse_args()

    report = json.loads((REPO / args.report).read_text(encoding="utf-8"))
    entry = (((report.get("manipulation") or {}).get("per_robot") or {}).get(args.arm) or {})
    if entry.get("resolved") is not True:
        print(json.dumps({"error": "per_robot[%s] 未解" % args.arm}, ensure_ascii=False))
        return 2
    gripper = entry.get("gripper") or {}
    name_map = entry.get("name_map") or {}
    target = (entry.get("targets") or [{}])[0]
    box_body = str(target.get("body") or "box_01")

    model = mujoco.MjModel.from_xml_path(str(REPO / report["output"]))
    data = mujoco.MjData(model)

    # 1) 把载荷摆到**运行期实测位姿**（初始台面位与 s06 时刻差 ~0.7 m，不摆等于量错对象）。
    payload = np.asarray([float(v) for v in str(args.payload).split(",")], dtype=float)
    box_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box_body))
    free = [index for index in range(int(model.njnt))
            if int(model.jnt_bodyid[index]) == box_id
            and int(model.jnt_type[index]) == int(mujoco.mjtJoint.mjJNT_FREE)]
    if len(free) == 1:
        adr = int(model.jnt_qposadr[free[0]])
        data.qpos[adr:adr + 3] = payload
        data.qpos[adr + 3:adr + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    pad_geoms = [str(name) for name in (gripper.get("pad_boxes") or ())]
    if not pad_geoms:
        print(json.dumps({"error": "gripper 未声明 pad_boxes，无法定夹持区口径"}, ensure_ascii=False))
        return 2
    # 声明名 → 模型名（name_map）；未映射即原样。
    pad_geoms = [name_map.get(name, name) for name in pad_geoms]
    arm_joints = [str(key) for key in (gripper.get("grasp_positions") or {})
                  if str(key) not in set((gripper.get("open_positions") or {}))
                  and str(key) not in set((gripper.get("closed_positions") or {}))]
    if not arm_joints:
        print(json.dumps({"error": "gripper.grasp_positions 里没有臂关节键"}, ensure_ascii=False))
        return 2
    joint_ids = _joint_ids(model, arm_joints)
    if any(jid < 0 for jid in joint_ids):
        print(json.dumps({"error": "臂关节名在模型里不存在: %s" % arm_joints}, ensure_ascii=False))
        return 2
    adrs = [int(model.jnt_qposadr[jid]) for jid in joint_ids]

    box_geoms = _payload_geoms(model, box_body)
    exclude = set()
    for name in pad_geoms:
        ident = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(name)))
        if ident >= 0:
            exclude.add(ident)
    arm_geoms = _arm_geoms(model, set(box_geoms), exclude)

    def pose_from(positions):
        return np.asarray([float(positions[name]) for name in arm_joints], dtype=float)

    phases = {}
    for key in PHASE_ORDER:
        section = gripper.get(key)
        if not isinstance(section, dict) or not section:
            continue
        missing = [name for name in arm_joints if name not in section]
        if missing:
            phases[PHASE_LABEL[key]] = {"skipped": "该段缺臂关节键: %s" % missing}
            continue
        phases[PHASE_LABEL[key]] = {"joint_positions": [float(v) for v in pose_from(section)]}

    # 2) 逐相位：把臂摆到该位形，量指腹中点与"臂 ↔ 载荷"最小间距。
    order = [PHASE_LABEL[key] for key in PHASE_ORDER if PHASE_LABEL[key] in phases]
    for label in order:
        if "joint_positions" not in phases[label]:
            continue
        qpos = dict(zip(arm_joints, phases[label]["joint_positions"]))
        for name, value in qpos.items():
            data.qpos[int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])] = value
        mujoco.mj_forward(model, data)
        mid = _pad_midpoint(model, data, pad_geoms)
        clearance, pair = _min_clearance(model, data, arm_geoms, box_geoms)
        phases[label].update({
            "pad_mid_m": [round(float(v), 9) for v in mid],
            "pad_minus_payload_m": [round(float(v), 9) for v in (mid - payload)],
            "min_clearance_to_payload_m": round(clearance, 9),
            "min_clearance_pair": ([mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair[0]),
                                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair[1])]
                                   if pair else None),
        })

    # 3) 逐段关节空间五次插值扫描（与后端 `_move_trajectory` 同口径）：找**首次侵入**。
    sweeps = []
    for previous, current in zip(["__initial__"] + order[:-1], order):
        if "joint_positions" not in phases.get(current, {}):
            continue
        if previous == "__initial__":
            start = np.asarray([float(data.qpos[adr]) for adr in adrs], dtype=float)
            # 初始 = 关键帧（模型里臂的静态初值），用一次 `mj_resetData` 取到。
            reset = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, reset, 0)
            start = np.asarray([float(reset.qpos[adr]) for adr in adrs], dtype=float)
        else:
            start = np.asarray(phases[previous]["joint_positions"], dtype=float)
        goal = np.asarray(phases[current]["joint_positions"], dtype=float)
        trace = []
        first = None
        for index in range(args.samples + 1):
            t = index / float(args.samples)
            s = 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5  # 五次多项式（与后端一致）
            sample = start + (goal - start) * s
            for name, value in zip(arm_joints, sample):
                jid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
                data.qpos[int(model.jnt_qposadr[jid])] = float(value)
            mujoco.mj_forward(model, data)
            clearance, pair = _min_clearance(model, data, arm_geoms, box_geoms)
            mid = _pad_midpoint(model, data, pad_geoms)
            trace.append({"t": round(t, 6),
                          "clearance_m": round(clearance, 9),
                          "pad_mid_z_m": round(float(mid[2]), 9)})
            if first is None and clearance < 0.0:
                first = {"t": round(t, 6),
                         "clearance_m": round(clearance, 9),
                         "pad_mid_m": [round(float(v), 9) for v in mid],
                         "pair": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair[0]),
                                  mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, pair[1])]}
        sweeps.append({
            "segment": "%s -> %s" % (previous, current),
            "pad_mid_z_start_m": trace[0]["pad_mid_z_m"] if trace else None,
            "pad_mid_z_end_m": trace[-1]["pad_mid_z_m"] if trace else None,
            "min_clearance_m": round(min(item["clearance_m"] for item in trace), 9),
            "first_penetration": first,
            "pad_mid_z_range_m": [min(item["pad_mid_z_m"] for item in trace),
                                  max(item["pad_mid_z_m"] for item in trace)],
        })

    result = {
        "arm": args.arm,
        "report": str(REPO / args.report),
        "model": report["output"],
        "payload_pose_m": [float(v) for v in payload],
        "payload_body": box_body,
        "arm_joints": arm_joints,
        "pad_geoms": pad_geoms,
        "phases": phases,
        "phase_order": order,
        "sweeps": sweeps,
        "note": ("臂基座在联合世界里静止 ⇒ 只按报告关节解 FK 即可；载荷按运行期实测位姿摆放。"
                 "`min_clearance_m < 0` = 臂几何侵入载荷（夹持区 pad 不计入，它是用来夹的）。"),
    }
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("arm", "phase_order", "sweeps")}, ensure_ascii=False, indent=2))
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
