#!/usr/bin/env python3
"""UR5e 基座包络探针：给"基座装在台面 z=0 是否会被埋进工作台"一个数字答案。

口径说明（为什么不是精确 AABB）：MuJoCo 的 `MjModel` 只给每个 geom 的**包围球**（`geom_rbound`）
与球心（`geom_xpos`）；精确 AABB 需要自己遍历网格顶点。对"基座会不会低于台面"这个问题，
用"球心下界 = geom_xpos.z − geom_rbound"是**保守下界**（真实网格不会更靠下）⇒ 若它 > 0，
结论确定；若 < 0，只说明"可能"，需再看实际网格。

用法：PYTHONPATH=src python3 build/ur5e_base_aabb_probe.py
输出 JSON 到 stdout（build/ 下，不进 git）。
"""
import json
import pathlib
import sys

import mujoco

REPO = pathlib.Path(__file__).resolve().parents[1]
MODEL = REPO / "build/models/ur5e_2f85/ur5e_2f85.xml"


def _geom_mesh_vertex_z_bounds(model, data, gid):
    """从**网格顶点**算该 geom 的世界系 z 上下界（精确到顶点；碰撞体就是顶点凸包）。

    为什么不用包围球：实测 base 的 mesh geom rbound = 0.119524，包围球下界给到 −0.0884 m，
    而顶点下界才是真值（差 8 cm 量级 ⇒ 不够用来判"基座会不会埋进台面"）。
    顶点是网格局部坐标 ⇒ 用 geom_xmat/geom_xpos 变换到世界系；meshes 通过 geom_dataid 关联。
    """
    data_id = int(model.geom_dataid[gid])
    if data_id < 0:
        # 非网格（球/盒子/胶囊等）：解析式上下界
        z = float(data.geom_xpos[gid][2])
        return z - float(model.geom_rbound[gid]), z + float(model.geom_rbound[gid])
    start = int(model.mesh_vertadr[data_id])
    count = int(model.mesh_vertnum[data_id])
    verts = model.mesh_vert[start:start + count]
    rotated = verts @ data.geom_xmat[gid].reshape(3, 3).T
    zs = rotated[:, 2] + float(data.geom_xpos[gid][2])
    return float(zs.min()), float(zs.max())


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    if base_body < 0:
        print(json.dumps({"error": "模型里没有 body 'base'"}, ensure_ascii=False))
        return 2
    rows = []
    lowest_mesh = None
    lowest_collision = None
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) != base_body:
            continue
        z = float(data.geom_xpos[gid][2])
        is_collision = bool(int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid]))
        z_min, z_max = _geom_mesh_vertex_z_bounds(model, data, gid)
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        rows.append({"geom": name, "type": int(model.geom_type[gid]),
                     "collision": is_collision, "geom_origin_z": z,
                     "z_min_m": z_min, "z_max_m": z_max})
        # 两个下界都要：①**几何**下界决定"基座会不会埋进台面"（visual 也一样占地）；
        #   ②**碰撞体**下界决定"会不会与台面产生接触"。本臂实测两者不同（②为空集）。
        lowest_mesh = z_min if lowest_mesh is None else min(lowest_mesh, z_min)
        if is_collision:
            lowest_collision = z_min if lowest_collision is None else min(lowest_collision, z_min)
    result = {
        "model": str(MODEL.relative_to(REPO)),
        "base_body_id": int(base_body),
        "geoms_in_base": rows,
        # 全部 geom（含 visual）的顶点下界：决定"安装面在哪"
        "base_lowest_mesh_z_m": lowest_mesh,
        # 仅碰撞 geom 的顶点下界：None ⇒ 该 body 没有碰撞体（不可能与工作台产生接触）
        "base_lowest_collision_z_m": lowest_collision,
        "base_has_collision_geom": lowest_collision is not None,
        "mount_plane_z_m": 0.0,
        # 判定按**几何**下界（容差 1e-9 覆盖顶点算出来的 −9.02e-10 这种浮点零）
        "verdict_z0_mount_ok": bool(lowest_mesh is not None and lowest_mesh >= -1e-9),
        "note": "z_min_m 由网格顶点经 geom_xmat/geom_xpos 变换后取最小值（精确）。",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
