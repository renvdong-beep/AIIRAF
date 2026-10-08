#!/usr/bin/env python3
"""探针：用**运行期实测的臂关节值**在同一联合模型上做 FK，与运行期自身记录的
`pad_mid_m` / 接触事实对账 —— 判"构建期航点姿态"与"运行期实际姿态"是否是同一个。

为什么需要（2026-10-08）：
  `scripts/probe_place_carrier_dock_offset.py` 用**报告里的 place_*_positions** 做静态 FK，
  得到"臂↔托盘"穿透 −30~−110 mm；而运行期 `PLACE_TRACE` 记录的同名穿透只有 −0.000453 m。
  两者差 30~110 mm ⇒ 要么"臂的姿态不同"，要么"载体位姿不同"，必须先分清。
  本探针取运行期**自己记录的** `arm_joint_positions`（`PLACE_TRACE.arm_joint_positions`，
  由后端在控制路径内实测）设置到模型里，比对该行自报的 `pad_mid_m` —— 若一致，说明
  FK 口径与运行期一致，差异就只可能来自**载体位姿**；若不一致，说明关节名/模型不是同一套。

用法（仓库根；只读日志 + 静态 FK，不推进时间）：
  PYTHONPATH=src python3 scripts/probe_arm_fk_from_log.py build/diagnostics/collide1.log --phase place_descend --limit 3
  PYTHONPATH=src python3 scripts/probe_arm_fk_from_log.py <log> --carrier-z 0.276627   # 顺带报托盘净空
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
JOINT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
CARRIER_ROOT = "base_link"


def _declared_pad_geoms():
    """指腹 geom 名**从报告读**（`gripper.left_finger_geom/right_finger_geom`），不写死。

    2026-10-08 实测教训：写死 `..._pad` 而不是声明里的 `..._pad1` 时，FK 的 pad_mid
    与运行期自报的 pad_mid 差 47 mm（y 向 43.9 mm）⇒ 探针与运行期不是同一口径。
    """
    try:
        doc = json.loads(JOINT_JSON.read_text())
        grip = doc["manipulation"]["per_robot"]["ur5e"]["gripper"]
        left = str(grip.get("left_finger_geom") or "")
        right = str(grip.get("right_finger_geom") or "")
        if left and right:
            return (left, right), False
    except Exception:
        pass
    return ("ur5e_rq2f85_left_pad", "ur5e_rq2f85_right_pad"), True


def _body(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="含 PLACE_TRACE 的日志")
    ap.add_argument("--phase", default="place_descend")
    ap.add_argument("--limit", type=int, default=3)
    ap.add_argument("--carrier-frame", default="handoff_station_frame_b")
    ap.add_argument("--carrier-z", type=float, default=None,
                    help="把载体摆到该 z（+ 站位帧 xy/quat），用于顺带报臂↔载体净空")
    ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--prefix", default="ur5e_",
                    help="联合模型里的臂关节名前缀（运行期日志用的是**无前缀**关节名，见 §11.51）")
    args = ap.parse_args()

    def _resolve(model, name, prefix):
        """运行期日志用无前缀名（shoulder_pan_joint），联合模型是 ur5e_shoulder_pan_joint。"""
        candidates = [str(name)]
        if prefix and not str(name).startswith(prefix):
            candidates.append(prefix + str(name))
        for cand in candidates:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, cand)
            if jid >= 0:
                return int(jid)
        return -1

    rows = []
    with Path(args.log).open(errors="replace") as fh:
        for line in fh:
            if not line.startswith("PLACE_TRACE "):
                continue
            try:
                row = json.loads(line[len("PLACE_TRACE "):])
            except json.JSONDecodeError:
                continue
            aj = row.get("arm_joint_positions") or {}
            if aj and "shoulder_pan_joint" in aj and args.phase in str(row.get("phase", "")):
                rows.append(row)
    if not rows:
        print("未找到 phase 含 %r 且带臂关节（shoulder_pan_joint）的 PLACE_TRACE 行" % args.phase)
        return 2
    print("命中 %d 行，取前 %d 行对账" % (len(rows), args.limit))
    pad_geoms, fallback = _declared_pad_geoms()
    print("指腹 geom（来自声明）= %s%s" % (list(pad_geoms), "  ⚠ 未读到声明，用了兜底名" if fallback else ""))

    model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    data = mujoco.MjData(model)
    if args.carrier_z is not None:
        site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.carrier_frame))
        free_adr = None
        root = _body(model, CARRIER_ROOT)
        for jnt in range(int(model.njnt)):
            if int(model.jnt_bodyid[jnt]) == root and int(model.jnt_type[jnt]) == int(
                    mujoco.mjtJoint.mjJNT_FREE):
                free_adr = int(model.jnt_qposadr[jnt])
        for geom in range(int(model.ngeom)):
            model.geom_margin[geom] = max(float(model.geom_margin[geom]), float(args.margin))

    for row in rows[: args.limit]:
        data.qpos[:] = 0.0
        if args.carrier_z is not None:
            data.qpos[free_adr:free_adr + 3] = model.site_pos[site]
            data.qpos[free_adr:free_adr + 3][2] = float(args.carrier_z)
            data.qpos[free_adr + 3:free_adr + 7] = model.site_quat[site]
        missing = []
        for name, value in (row.get("arm_joint_positions") or {}).items():
            jid = _resolve(model, name, args.prefix)
            if jid < 0:
                missing.append(str(name))
                continue
            data.qpos[int(model.jnt_qposadr[jid])] = float(value)
        mujoco.mj_forward(model, data)
        pads = []
        for g in pad_geoms:
            gid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g))
            if gid < 0:
                print("   ⚠ 声明的指腹 geom %s 不在模型里" % g)
                continue
            pads.append(np.asarray(data.geom_xpos[gid], dtype=float))
        if len(pads) != 2:
            print("   跳过该行（指腹 geom 解析不全）")
            continue
        pad_mid = (pads[0] + pads[1]) / 2.0
        logged = row.get("pad_mid_m")
        print("-- phase=%s step=%s" % (row.get("phase"), row.get("plant_step_index")))
        print("   关节名未解析: %s" % (missing or "无"))
        print("   FK pad_mid   = %s" % [round(float(v), 6) for v in pad_mid])
        print("   日志 pad_mid = %s" % logged)
        if logged:
            d = np.asarray(pad_mid, dtype=float) - np.asarray(logged, dtype=float)
            print("   差           = %s  |d|=%.6f m" % ([round(float(v), 6) for v in d],
                                                       float(np.linalg.norm(d))))
        print("   wrist_1 = %s  wrist_2 = %s" % (
            [round(float(v), 6) for v in data.xpos[_body(model, "ur5e_wrist_1_link")]],
            [round(float(v), 6) for v in data.xpos[_body(model, "ur5e_wrist_2_link")]]))
        tray = _body(model, "tray_01")
        print("   tray_01 = %s" % [round(float(v), 6) for v in data.xpos[tray]])
        if args.carrier_z is not None:
            worst = {}
            for ci in range(int(data.ncon)):
                c = data.contact[ci]
                b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
                n1 = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b1) or b1)
                n2 = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b2) or b2)
                if not (n1.startswith("ur5e") or n2.startswith("ur5e")):
                    continue
                other = n2 if n1.startswith("ur5e") else n1
                if other in ("tray_01", "base_link", "FL_hip"):
                    key = "%s↔%s" % (n1 if n1.startswith("ur5e") else n2, other)
                    worst[key] = min(worst.get(key, 1e9), round(float(c.dist), 6))
            print("   臂↔载体（含 margin %.3f）：%s" % (args.margin, worst or "无（净空 > margin）"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
