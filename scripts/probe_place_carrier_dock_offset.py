#!/usr/bin/env python3
"""探针：构建期"载体摆在标称站位"看不见的臂↔载体净空（s07 腕部撞狗 / §11.92 / §11.93附）。

背景（2026-10-08）：
  运行期实测（build/diagnostics/collide1.log）UR5e 放置时有三类**载体**接触：
    ur5e_wrist_1_link ↔ tray_01    最深 -0.000807 m，n=136，首见 after_above
    ur5e_wrist_1_link ↔ FL_hip     最深 -0.000067 m，n=133，首见 place_descend@step300
    ur5e_wrist_2_link ↔ base_link  最深 -0.000746 m，首见 place_above@step640
  而构建期自检只报 `waypoint_contacts=[]`（航点端点无载体接触）、
  `path_contact_count=192` 且取样全是 arm↔world / arm↔place_pad_b（carrier=False）。

本探针在**联合模型**上做**静态 FK**（只 mj_forward，不 mj_step），回答三件事：
  1. 标称口径（把载体 free joint 直接写成站位帧 site 的 pos/quat，腿关节全 0 —— 与
     `src/iraf_adapters/unitree/scene_builder.py:2067-2091` 的构建期自检**同一口径**）
     下，臂↔载体的**有符号净空**是多少（用 margin 把"接近但未接触"也读出来）；
  2. 把载体按给定的**停靠偏差**（平移 + 偏航，绕站位帧轴）刚性摆放后，净空怎么变
     —— 与运行期的接触对对账（判"标称看不见"这一缺口是否成立）；
  3. 关键 body 的世界位姿（tray_01 / base_link / FL_hip / 臂的 wrist_*）——
     用于核对"载体高度口径"（构建期 qpos z 与运行期站立高度是否同一个）。

用法（仓库根，**不要与批次同时跑**；本探针只 mj_forward，不推进时间）：
  PYTHONPATH=src python3 scripts/probe_place_carrier_dock_offset.py --margin 0.05
  PYTHONPATH=src python3 scripts/probe_place_carrier_dock_offset.py --offset-m 0.028 --offset-dir-deg 80 --yaw-deg 2.45 --margin 0.05
  PYTHONPATH=src python3 scripts/probe_place_carrier_dock_offset.py --scan --margin 0.05
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
JOINT_JSON = REPO / "build/scenes/handoff_lab/handoff_lab_joint.json"
JOINT_XML = REPO / "build/scenes/handoff_lab/handoff_lab_joint.xml"
CARRIER_ROOT = "base_link"
ARM_PREFIX = "ur5e_"
WATCH_BODIES = ("tray_01", "base_link", "FL_hip", "ur5e_wrist_1_link",
                "ur5e_wrist_2_link", "ur5e_rq2f85_left_pad", "place_pad_b")


def _subtree(model, root_id):
    out = set()
    for body in range(int(model.nbody)):
        walk = body
        while walk > 0:
            if walk == root_id:
                out.add(body)
                break
            walk = int(model.body_parentid[walk])
    return out


def _free_qpos_adr(model, body_id):
    for jnt in range(int(model.njnt)):
        if int(model.jnt_bodyid[jnt]) == body_id and int(
                model.jnt_type[jnt]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return int(model.jnt_qposadr[jnt])
    return None


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], dtype=float)


def _body_id(model, name):
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))


def run_case(model, data, args, offset_m, offset_dir_deg, yaw_deg, waypoints, order,
             carrier_bodies, arm_bodies, free_adr, site_id, label):
    base_pos = np.asarray(model.site_pos[site_id], dtype=float).copy()
    base_quat = np.asarray(model.site_quat[site_id], dtype=float).copy()
    # 载体**高度口径**（2026-10-08，隔离验证用）：
    #   `site`（构建期现状）＝ 站位帧 site 的 z 原样写入载体 free joint（实测 z=0 ⇒ 狗"沉到地面以下"）
    #   `model-default`      ＝ 用模型默认位形 `model.qpos0` 的载体 z（Go2 模型默认＝站立姿态，
    #                            实测 0.445），站位帧只提供 xy 与偏航 —— 这才接近运行期"站着"的狗
    #   `explicit`           ＝ `--carrier-z` 直接给值
    if args.carrier_z_mode == "model-default":
        base_pos[2] = float(model.qpos0[free_adr + 2]) if free_adr is not None else base_pos[2]
    elif args.carrier_z_mode == "explicit":
        base_pos[2] = float(args.carrier_z)
    ang = math.radians(offset_dir_deg)
    offset_local = np.array([offset_m * math.cos(ang), offset_m * math.sin(ang), 0.0])
    yaw = math.radians(yaw_deg)
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    delta_q = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
    pos_new = base_pos + offset_local
    quat_new = _quat_mul(delta_q, base_quat)

    def reset():
        data.qpos[:] = 0.0
        if free_adr is not None:
            data.qpos[free_adr:free_adr + 3] = pos_new
            data.qpos[free_adr + 3:free_adr + 7] = quat_new

    def joint_sets(section):
        out = {}
        for name, value in section.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                out[int(jid)] = float(value)
        return out

    sets = {name: joint_sets(waypoints[name]) for name in order}
    rows = []

    def scan(tag, fraction=0.0, frm="", to=""):
        mujoco.mj_forward(model, data)
        for ci in range(int(data.ncon)):
            contact = data.contact[ci]
            b1, b2 = int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])
            arm = b1 if b1 in arm_bodies else (b2 if b2 in arm_bodies else None)
            if arm is None:
                continue
            other = b2 if arm == b1 else b1
            rows.append({"at": tag, "fraction": round(fraction, 3), "from": frm, "to": to,
                         "arm_body": str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, arm) or arm),
                         "other_body": str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other) or other),
                         "carrier": bool(other in carrier_bodies),
                         "dist_m": round(float(contact.dist), 6)})

    poses = {}
    for name in order:
        reset()
        for jid, value in sets[name].items():
            data.qpos[int(model.jnt_qposadr[jid])] = value
        scan("waypoint:%s" % name)
        poses[name] = {b: [round(float(v), 6) for v in data.xpos[_body_id(model, b)]]
                       for b in WATCH_BODIES if _body_id(model, b) >= 0}
    for frm, to in zip(order, order[1:]):
        common = sorted(set(sets[frm]) & set(sets[to]))
        for index in range(1, args.samples):
            frac = index / float(args.samples)
            w = 10 * frac ** 3 - 15 * frac ** 4 + 6 * frac ** 5
            reset()
            for jid in common:
                data.qpos[int(model.jnt_qposadr[jid])] = (
                    sets[frm][jid] + (sets[to][jid] - sets[frm][jid]) * w)
            scan("path:%s>%s" % (frm, to), frac, frm, to)

    carrier_rows = [r for r in rows if r["carrier"]]
    worst = {}
    for r in rows:
        key = (r["arm_body"], r["other_body"])
        worst[key] = min(worst.get(key, 1e9), r["dist_m"])
    print("── 用例 %s：平移 %.6f m（方向 %.1f°）偏航 %.6f°  margin %.4f" %
          (label, offset_m, offset_dir_deg, yaw_deg, args.margin))
    carrier_names = {str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or b)
                     for b in carrier_bodies}
    carrier_min = min((v for k, v in worst.items() if k[1] in carrier_names), default=None)
    print("   【最坏载体净空】%s m（越正越好；判据 ≥ 停靠容差 0.030 + 余量）"
          % ("%.6f" % carrier_min if carrier_min is not None else "n/a"))
    print("   臂↔场景对 %d 组；臂↔**载体**对 %d 组" % (len(worst), len([k for k in worst if any(
        k[1] == str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or b)
        for b in carrier_bodies)])))
    for key, value in sorted(worst.items(), key=lambda kv: kv[1])[:12]:
        print("     %-24s ↔ %-22s net=%.6f m" % (key[0], key[1], value))
    # 载体相关对**全列**（判"标称看不见"缺口的主指标；不受 top-12 截断影响）
    carrier_names = {str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or b)
                     for b in carrier_bodies}
    carrier_pairs = sorted(((k, v) for k, v in worst.items() if k[1] in carrier_names),
                           key=lambda kv: kv[1])
    print("     【载体对】%d 组：" % len(carrier_pairs))
    for key, value in carrier_pairs[:14]:
        print("        %-24s ↔ %-22s net=%.6f m" % (key[0], key[1], value))
    for name in order:
        pose = poses.get(name) or {}
        if "tray_01" in pose:
            print("     [位姿] %-8s tray_01=%s  base_link=%s  wrist_1=%s" %
                  (name, pose.get("tray_01"), pose.get("base_link"), pose.get("ur5e_wrist_1_link")))
    if carrier_rows:
        print("   载体接触明细（前 10）：")
        for r in carrier_rows[:10]:
            print("     %-22s frac=%-5s %-22s ↔ %-20s %.6f" %
                  (r["at"], r["fraction"], r["arm_body"], r["other_body"], r["dist_m"]))
    return {"label": label, "offset_m": offset_m, "offset_dir_deg": offset_dir_deg,
            "yaw_deg": yaw_deg, "margin": args.margin, "pairs": {("%s↔%s" % k): v for k, v in worst.items()},
            "carrier_contacts": carrier_rows, "all_contacts": rows, "poses": poses}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offset-m", type=float, default=0.0)
    ap.add_argument("--offset-dir-deg", type=float, default=0.0, help="平移方向（0=+x，90=+y）")
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--margin", type=float, default=0.05, help="臂与载体 geom 的接触 margin（读有符号净空）")
    ap.add_argument("--carrier-frame", default="handoff_station_frame_b")
    ap.add_argument("--carrier-z-mode", default="site", choices=("site", "model-default", "explicit"),
                    help="载体高度口径：site=站位帧原样（构建期现状）；model-default=模型默认位形高度")
    ap.add_argument("--carrier-z", type=float, default=0.0, help="配合 --carrier-z-mode explicit")
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument("--scan", action="store_true", help="扫平移 0..0.04 m × 偏航 0..2.5°")
    ap.add_argument("--scan-offsets", default="0,0.01,0.02,0.03,0.04", help="--scan 的平移量列表（逗号分隔）")
    ap.add_argument("--scan-dirs", default="80", help="--scan 的平移方向列表（度，逗号分隔）")
    ap.add_argument("--scan-yaws", default="0,2.45", help="--scan 的偏航列表（度，逗号分隔）")
    args = ap.parse_args()

    doc = json.loads(JOINT_JSON.read_text())
    grip = doc["manipulation"]["per_robot"]["ur5e"]["gripper"]
    order = [n for n in ("transit", "above", "descend", "retreat")
             if grip.get("place_%s_positions" % n)]
    if not order:
        print("产物里没有 place_*_positions ⇒ 先跑受控重建")
        return 2
    waypoints = {n: {str(k): float(v) for k, v in grip["place_%s_positions" % n].items()}
                 for n in order}

    model = mujoco.MjModel.from_xml_path(str(JOINT_XML))
    data = mujoco.MjData(model)
    site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.carrier_frame))
    if site_id < 0:
        print("站位帧 %s 不在模型里" % args.carrier_frame)
        return 2
    carrier_bodies = _subtree(model, _body_id(model, CARRIER_ROOT))
    arm_bodies = set()
    for body in range(int(model.nbody)):
        name = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "")
        if name.startswith(ARM_PREFIX):
            arm_bodies.add(body)
    free_adr = _free_qpos_adr(model, _body_id(model, CARRIER_ROOT))
    if args.margin > 0.0:
        for geom in range(int(model.ngeom)):
            body = int(model.geom_bodyid[geom])
            if body in arm_bodies or body in carrier_bodies:
                model.geom_margin[geom] = float(args.margin)

    def _floats(text):
        return [float(v) for v in str(text).split(",") if str(v).strip() != ""]

    cases = ([{"offset_m": o, "offset_dir_deg": d, "yaw_deg": y}
              for o in _floats(args.scan_offsets)
              for d in _floats(args.scan_dirs)
              for y in _floats(args.scan_yaws)]
             if args.scan else
             [{"offset_m": args.offset_m, "offset_dir_deg": args.offset_dir_deg,
               "yaw_deg": args.yaw_deg}])
    report = []
    for case in cases:
        report.append(run_case(model, data, args, case["offset_m"], case["offset_dir_deg"],
                               case["yaw_deg"], waypoints, order, carrier_bodies, arm_bodies,
                               free_adr, site_id,
                               "d%.0f" % case["offset_dir_deg"] if args.scan else "case"))

    outdir = REPO / "build/diagnostics"
    outdir.mkdir(parents=True, exist_ok=True)
    # 文件名必须含**高度口径**：否则多次不同 --carrier-z 的运行会互相覆盖（2026-10-08 踩到）
    _ztag = ("site" if args.carrier_z_mode == "site" else
             "z%.6f" % args.carrier_z if args.carrier_z_mode == "explicit" else "zmodel")
    sink = outdir / ("place-carrier-clearance-%s-%s.json" %
                     ("scan" if args.scan else "m%.6f-y%.4f-d%.1f" %
                      (args.offset_m, args.yaw_deg, args.offset_dir_deg), _ztag))
    sink.write_text(json.dumps({"args": vars(args), "cases": report}, ensure_ascii=False, indent=2))
    print("[留档] %s" % sink)
    return 0


if __name__ == "__main__":
    sys.exit(main())
