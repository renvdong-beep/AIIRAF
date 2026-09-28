"""载荷落位事实的**共享测量**（臂侧 `place_object` 与四足侧 `accept_payload` 共用）。

为什么要共享（AGENTS.md 6.3：新机器人/新 Skill 接入不得复制 Runtime 核心代码）：
"接收体上是否有载荷"这件事在两侧都要量同一批**几何事实**（落位间隙、接触对、中心偏移），
但两侧的判据用途不同：
  · 臂侧 `place_object`：判"我把它放下了"（`payload_in_tray`）；
  · 四足侧 `accept_payload`：**独立复核**"它确实在托盘上"（不复用臂侧证据）。
若各写一份，口径必然漂移（本会话已因"两侧口径不一致"踩过坑）。因此把测量收敛到这里，
只做"从模型与 data 出事实"，**锁、租约、证据组装由各自后端负责**。

判据口径（全是事实，不设力阈值；阈值由场景判据声明）：
  `payload_on_target` = 载荷与接收体 geom **存在接触** 且 载荷最低点**不高于承载面**（不是悬空）。
"""

import numpy as np


def _body_id(mujoco, model, name):
    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(name)))
    if body_id < 0:
        raise ValueError("模型里找不到 body: " + str(name))
    return body_id


def _lowest_point_z(mujoco, model, data, body_id):
    """body 下所有 geom 的最低点 z（按 geom 类型算支撑点；用于判"是否落在承载面上"）。

    这里**不引入任何机型语义**：只按 MuJoCo 的 geom 类型取各自的最低支撑点。
    """
    lowest = None
    for geom_id in range(int(model.ngeom)):
        if int(model.geom_bodyid[geom_id]) != int(body_id):
            continue
        center = np.asarray(data.geom_xpos[geom_id], dtype=float)
        geom_type = int(model.geom_type[geom_id])
        if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            # 网格：用**顶点**求最低点（size 只是 AABB 半长，不能当半径 —— 本会话实测踩过）
            vert_adr = int(model.mesh_vertadr[int(model.geom_dataid[geom_id])])
            vert_num = int(model.mesh_vertnum[int(model.geom_dataid[geom_id])])
            verts = np.asarray(model.mesh_vert[vert_adr:vert_adr + vert_num], dtype=float)
            rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
            z = float((rotation @ verts.T).T[:, 2].min() + center[2])
        else:
            # 其余类型用 size 的竖直半高（盒/球/圆柱/胶囊在 MuJoCo 里 size[2] 即竖直半高）
            half_z = float(model.geom_size[geom_id][2])
            z = float(center[2] - abs(half_z))
        lowest = z if lowest is None else min(lowest, z)
    if lowest is None:
        raise ValueError("body 下没有任何 geom，无法量最低点")
    return lowest


def confirm_payload_on_target(mujoco, model, data, payload_name, target_name):
    """量"载荷是否落在接收体承载面上"的一批事实，并给出整链末速。

    返回（全部为实测值）：
      `payload_on_target`（事实判据）/`payload_low_z_m`/`target_top_z_m`/`resting_gap_m`
      （带符号：≤0 表示接触或压入）/`contact_geoms`/`payload_center_m`/`target_center_m`/
      `offset_from_target_center_m`/`last_speed_mps`（模型里**所有自由关节**的线速度上界）。
    """
    payload_body = _body_id(mujoco, model, payload_name)
    target_body = _body_id(mujoco, model, target_name)
    mujoco.mj_forward(model, data)

    low = _lowest_point_z(mujoco, model, data, payload_body)
    # 承载面：接收体**最高**的 geom 上表面中心（按 geom 姿态算，不假设轴对齐）
    top_z, top_center = None, None
    for geom_id in range(int(model.ngeom)):
        if int(model.geom_bodyid[geom_id]) != target_body:
            continue
        center = np.asarray(data.geom_xpos[geom_id], dtype=float)
        rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
        half = np.asarray(model.geom_size[geom_id], dtype=float)
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_BOX):
            # 盒体：按**世界顶点**求上表面（对任意姿态都正确，不假设轴对齐）
            corners = np.asarray([[sx * half[0], sy * half[1], sz * half[2]]
                                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
            z = float((rotation @ corners.T).T[:, 2].max() + center[2])
        else:
            z = float(center[2] + abs(float(half[2])))
        if top_z is None or z > top_z:
            top_z, top_center = z, center
    if top_z is None or top_center is None:
        raise ValueError("接收体 %s 下没有 geom，无法判承载面" % target_name)

    payload_center = np.asarray(data.xpos[payload_body], dtype=float).copy()
    contacts = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        bodies = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
        if payload_body not in bodies or target_body not in bodies:
            continue
        partner = (int(contact.geom2)
                   if int(model.geom_bodyid[contact.geom1]) == target_body else int(contact.geom1))
        contacts.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, partner)
                        or "#%d" % partner)

    # 整链末速：模型里所有**自由关节**（四足基座与载荷）的线速度上界 —— 不写死任何 body 名
    speed = 0.0
    for jnt in range(int(model.njnt)):
        if int(model.jnt_type[jnt]) != int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        dof = int(model.jnt_dofadr[jnt])
        speed = max(speed, float(np.linalg.norm(np.asarray(data.qvel[dof:dof + 3], dtype=float))))

    offset = float(np.linalg.norm(np.asarray(payload_center[:2], dtype=float)
                                  - np.asarray(top_center[:2], dtype=float)))
    resting_gap = float(low - top_z)
    return {
        "payload_on_target": bool(contacts) and resting_gap <= 0.0,
        "payload_low_z_m": round(low, 9),
        "target_top_z_m": round(float(top_z), 9),
        "resting_gap_m": round(resting_gap, 9),
        "contact_geoms": sorted(set(contacts)),
        "payload_center_m": [round(float(v), 9) for v in payload_center],
        "target_center_m": [round(float(v), 9) for v in top_center],
        "offset_from_target_center_m": round(offset, 9),
        "last_speed_mps": round(speed, 9),
    }
