#!/usr/bin/env python3
"""联合模型**初始位形干涉探针**：找出"谁与谁穿透/谁在线面以下"。

为什么需要（2026-09-29 实测）：把第二台臂的基座从 (0.45, 0.45) 挪到 (0.45, 0.90) 并改偏航后，
整条场景**炸了**（s01 末速 131.3047796509399 m/s、s02 平移 31.296342 m、托盘承载面 −353646.07 m）。
共享植物里任何一处初始穿透都会通过求解器把整株打飞，因此必须在**构建产物**上直接量：
  · 关键帧位形下每个 body/geom 的最低 z（对照支撑面 z=0）；
  · 初始接触对与穿透深度（mj_forward 后的 `mj_contactForce`/`dist`）；
  · 两两机器人之间的最近 geom 距离（谁挨着谁）。
输出 JSON：build/diagnostics/joint-initial-interference.json

用法：PYTHONPATH=src python3 scripts/probe_joint_initial_interference.py
       [--model build/scenes/handoff_lab/handoff_lab_joint.xml] [--key 0]
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]


def _name(model, kind, ident):
    return mujoco.mj_id2name(model, kind, ident) or "<unnamed>"


def _owner_of_body(model, body_id, cache=None):
    """按 **body 树**把对象归到本体上（前缀只对附加本体有效，不足以分类主模型）。

    为什么必须走树（2026-09-29 实测）：第一版按名字前缀分类，把**工作台/台架/道具**都算进了
    "main"，于是"main vs ur5e"的最近间隙恒定由那块大台面决定 ⇒ 换狗的位置数字**完全不变**、
    探针失去分辨力。正确做法：
      · 附加本体按 `piper_` / `ur5e_` 前缀（`attach` 加的前缀，就是所有权）；
      · 主模型里，挂在 `base_link` 子树下的 = 四足（含狗背托盘 `tray_01`）；
      · 其余挂世界的静态件 = 场景固定件（工作台、臂基座）；道具（有自由关节的）= fixture_prop。
    """
    if cache is not None and body_id in cache:
        return cache[body_id]
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
    owner = None
    for prefix in ("piper_", "ur5e_"):
        if name.startswith(prefix):
            owner = prefix[:-1]
            break
    if owner is None:
        # 向上找根：遇到 base_link ⇒ 四足；直接落在 world ⇒ 场景件
        cursor = body_id
        guard = 0
        while cursor > 0 and guard < 64:
            parent = int(model.body_parentid[cursor])
            if parent == 0:
                owner = "fixture"
                break
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent) or "") == "base_link":
                owner = "dog"
                break
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, cursor) or "") == "base_link":
                owner = "dog"
                break
            cursor = parent
            guard += 1
        if owner is None:
            owner = "fixture"
    if cache is not None:
        cache[body_id] = owner
    return owner


def _geom_z_bounds(model, data, gid):
    """顶点级世界系 z 下界（网格用顶点，非网格用包围球）。"""
    data_id = int(model.geom_dataid[gid])
    if data_id < 0:
        z = float(data.geom_xpos[gid][2])
        return z - float(model.geom_rbound[gid])
    start = int(model.mesh_vertadr[data_id])
    count = int(model.mesh_vertnum[data_id])
    verts = model.mesh_vert[start:start + count]
    rotated = verts @ data.geom_xmat[gid].reshape(3, 3).T
    return float((rotated[:, 2] + float(data.geom_xpos[gid][2])).min())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="build/scenes/handoff_lab/handoff_lab_joint.xml")
    parser.add_argument("--key", type=int, default=0)
    # 把**主本体（狗）**搬到指定位姿再量干涉：用于回答"走到某站位时会不会撞到臂"。
    # 只写自由关节的 qpos 并 mj_forward（不做动力学），因此是纯几何判定（快、确定）。
    parser.add_argument("--main-at", type=float, nargs=2, default=None, metavar=("X", "Y"),
                        help="把主本体的自由关节平移到 (x, y)（z 与姿态保持关键帧值）")
    parser.add_argument("--main-yaw-deg", type=float, default=0.0)
    parser.add_argument("--output", default="build/diagnostics/joint-initial-interference.json")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(REPO / args.model))
    data = mujoco.MjData(model)
    if model.nkey > args.key:
        mujoco.mj_resetDataKeyframe(model, data, args.key)
    if args.main_at is not None:
        # 主本体 = 第 0 个自由关节所属的 body（场景里就是四足）
        free_joints = [j for j in range(int(model.njnt))
                       if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)]
        if not free_joints:
            print(json.dumps({"error": "模型里没有自由关节，无法搬运主本体"}, ensure_ascii=False))
            return 2
        address = int(model.jnt_qposadr[free_joints[0]])
        yaw = math.radians(float(args.main_yaw_deg))
        data.qpos[address + 0] = float(args.main_at[0])
        data.qpos[address + 1] = float(args.main_at[1])
        data.qpos[address + 3] = math.cos(yaw / 2.0)
        data.qpos[address + 6] = math.sin(yaw / 2.0)
    mujoco.mj_forward(model, data)

    # 每个本体的最低**碰撞** geom（visual 不产生接触，但要单独列出以防"埋进台面"）
    per_owner = {}
    _owner_cache = {}
    for gid in range(int(model.ngeom)):
        body_id = int(model.geom_bodyid[gid])
        body_name = _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        owner = _owner_of_body(model, body_id, cache=_owner_cache)
        z_min = _geom_z_bounds(model, data, gid)
        collision = bool(int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid]))
        entry = per_owner.setdefault(owner, {"lowest_collision_z_m": None, "lowest_any_geom_z_m": None,
                                             "lowest_collision_geom": None})
        if entry["lowest_any_geom_z_m"] is None or z_min < entry["lowest_any_geom_z_m"]:
            entry["lowest_any_geom_z_m"] = z_min
        if collision and (entry["lowest_collision_z_m"] is None
                          or z_min < entry["lowest_collision_z_m"]):
            entry["lowest_collision_z_m"] = z_min
            entry["lowest_collision_geom"] = _name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) + \
                " @body " + body_name

    # 初始接触（关键帧位形下 mj_forward 之后的接触对 + 穿透深度）。**这才是权威判据**：
    # 上面的"最近间隙"是包围球近似（在下界方向偏保守，可能虚报"接触"）⇒ 决策必须看接触对。
    contacts = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        owner1 = _owner_of_body(model, body1, cache=_owner_cache)
        owner2 = _owner_of_body(model, body2, cache=_owner_cache)
        contacts.append({
            "geom1": _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)),
            "geom2": _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)),
            "owner1": owner1, "owner2": owner2,
            "dist_m": float(contact.dist),
        })
    deep = sorted([item for item in contacts if item["dist_m"] < -1e-6],
                  key=lambda item: item["dist_m"])[:10]
    # 按本体对归类（权威）：几对、最深多少
    by_pair = {}
    for item in deep:
        key = " vs ".join(sorted([item["owner1"], item["owner2"]]))
        entry = by_pair.setdefault(key, {"count": 0, "deepest_dist_m": 0.0, "example": None})
        entry["count"] += 1
        if item["dist_m"] < entry["deepest_dist_m"]:
            entry["deepest_dist_m"] = item["dist_m"]
            entry["example"] = "%s ↔ %s" % (item["geom1"], item["geom2"])

    # 两两本体之间的最近 geom 距离（只用碰撞体；网格对网格用顶点近似 + 包围球粗筛）
    collision_geoms = [gid for gid in range(int(model.ngeom))
                       if int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid])]
    owners = {}
    for gid in collision_geoms:
        body_id = int(model.geom_bodyid[gid])
        owners.setdefault(_owner_of_body(model, body_id, cache=_owner_cache), []).append(gid)
    pairs = {}
    keys = sorted(owners)
    for i, first in enumerate(keys):
        for second in keys[i + 1:]:
            best = None
            for gid_a in owners[first]:
                for gid_b in owners[second]:
                    delta = np.asarray(data.geom_xpos[gid_a], dtype=float) - \
                        np.asarray(data.geom_xpos[gid_b], dtype=float)
                    surface = float(np.linalg.norm(delta)) - float(model.geom_rbound[gid_a]) \
                        - float(model.geom_rbound[gid_b])
                    if best is None or surface < best[0]:
                        best = (surface, _name(model, mujoco.mjtObj.mjOBJ_GEOM, gid_a),
                                _name(model, mujoco.mjtObj.mjOBJ_GEOM, gid_b))
            if best is not None:
                pairs["%s vs %s" % (first, second)] = {
                    "min_surface_gap_m_bound": best[0], "geom_a": best[1], "geom_b": best[2],
                    "note": "包围球近似下界（用于快速定位，不是精确距离）"}

    result = {
        "model": args.model,
        "keyframe": args.key,
        "nq": int(model.nq), "ngeom": int(model.ngeom),
        "support_plane_z_m": 0.0,
        "per_owner_lowest_z": per_owner,
        "owners_below_support": {owner: entry for owner, entry in per_owner.items()
                                 if (entry["lowest_collision_z_m"] is not None
                                     and entry["lowest_collision_z_m"] < -1e-6)},
        "contacts_at_keyframe": len(contacts),
        "deepest_contacts": deep,
        "penetrating_contacts_by_owner_pair": by_pair,
        "pairwise_min_gaps": pairs,
    }
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print("关键帧位形：nq=%d ngeom=%d 接触对=%d" % (result["nq"], result["ngeom"], len(contacts)))
    for owner, entry in sorted(per_owner.items()):
        print("  %-8s 最低碰撞体 z=%s（%s） | 最低任意 geom z=%.9f"
              % (owner, entry["lowest_collision_z_m"], entry["lowest_collision_geom"],
                 entry["lowest_any_geom_z_m"]))
    for item in deep:
        print("  穿透: %s ↔ %s dist=%.6f" % (item["geom1"], item["geom2"], item["dist_m"]))
    for key, value in sorted(by_pair.items()):
        print("  **穿透(接触对，权威)**: %-24s %d 对，最深 %.6f m（%s）"
              % (key, value["count"], value["deepest_dist_m"], value["example"]))
    for key, value in pairs.items():
        print("  最近间隙(界): %-22s %.4f m（%s ↔ %s）"
              % (key, value["min_surface_gap_m_bound"], value["geom_a"], value["geom_b"]))
    print("WROTE %s" % out.relative_to(REPO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
