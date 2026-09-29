"""载荷落位事实的**共享测量**（臂侧 `place_object` 与四足侧 `accept_payload` 共用）。

为什么要共享（AGENTS.md 6.3：新机器人/新 Skill 接入不得复制 Runtime 核心代码）：
"接收体上是否有载荷"这件事在两侧都要量同一批**几何事实**（落位间隙、接触对、中心偏移），
但两侧的判据用途不同：
  · 臂侧 `place_object`：判"我把它放下了"（`payload_in_tray`）；
  · 四足侧 `accept_payload`：**独立复核**"它确实在托盘上"（不复用臂侧证据）。
若各写一份，口径必然漂移（本会话已因"两侧口径不一致"踩过坑）。因此把测量收敛到这里，
只做"从模型与 data 出事实"，**锁、租约、证据组装由各自后端负责**。

判据口径（全是事实，不设力阈值；阈值由场景判据声明）：
  `payload_on_target` = 载荷与接收体 geom **存在接触** 且 载荷最低点**不高于「承载面 + 模型声明的
  接触 margin」**（不是悬空）。
  ⚠ margin 必须从**模型**读（`geom_margin`），不能写死 0：厂商 Go2 模型声明 `margin="0.001"`，
  MuJoCo 在 `dist < margin` 时就把接触纳入约束集 ⇒ 载荷会稳定停在几何表面**上方 ~1 mm**
  （实测 dist +0.000489、每点力 0.10~0.12 N）。写死 `gap ≤ 0` 会让任何带 margin 的模型永远判"悬空"。
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
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            # 盒体：按**世界顶点**求最低点 —— 与承载面（下方同一算法）**同口径**。
            # 为什么必须（2026-09-29 实测）：`center[2] - half_z` 只对**轴对齐**盒成立；载荷随
            # 狗身姿态在托盘上轻微倾斜时（本例 0.72 mm / 0.05 m ≈ 0.83°），该式**高估**最低点
            # ⇒ 明明已接触（`contact_geoms=['box_01_geom']`）却算出 `resting_gap=+0.000722398 m`
            # ⇒ `payload_on_target` 被判 False（s05 误报"未确认落在接收体上"）。
            rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
            corners = np.asarray([[sx * float(model.geom_size[geom_id][0]),
                                   sy * float(model.geom_size[geom_id][1]),
                                   sz * float(model.geom_size[geom_id][2])]
                                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
            z = float((rotation @ corners.T).T[:, 2].min() + center[2])
        elif geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            z = float(center[2] - abs(float(model.geom_size[geom_id][0])))
        else:
            # 其余类型（圆柱/胶囊/椭球）用 size 的竖直半高：**已知的近似**（倾斜时同样会高估），
            # 本场景的载荷与接收体都是盒体，不进这条分支；真要支持需按 geom 轴向算端点。
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

    # **接触 margin（"软垫"）**：MuJoCo 在 `dist < margin` 时就把接触纳入约束集 ⇒ 承载物会在几何
    # 表面**上方 ~margin** 处形成稳定平衡（2026-09-29 实测：4 个接触点、每点 0.1007~0.1174 N、
    # 合计 ≈0.436 N ≈ 载荷重量 0.392 N + 托盘受压，而 `dist=+0.000489`）。
    # 本场景的 margin 来自**厂商 Go2 模型**的 `<default class="go2"><geom margin="0.001">`
    # ⇒ 判"是否落在承载面上"必须用**模型自己的口径**，不能写死 `gap ≤ 0`：
    # 否则任何带 margin 的模型都永远判"悬空"（旧的世界固定托盘恰好无 margin 干扰，掩盖了这条）。
    # 取值只从模型读（不写数字）：载荷与接收体 geom 的 margin 上界。
    margin = 0.0
    for geom_id in range(int(model.ngeom)):
        if int(model.geom_bodyid[geom_id]) in (payload_body, target_body):
            margin = max(margin, float(model.geom_margin[geom_id]))
    offset = float(np.linalg.norm(np.asarray(payload_center[:2], dtype=float)
                                  - np.asarray(top_center[:2], dtype=float)))
    resting_gap = float(low - top_z)
    return {
        "payload_on_target": bool(contacts) and resting_gap <= margin,
        "contact_margin_m": round(float(margin), 9),
        "payload_low_z_m": round(low, 9),
        "target_top_z_m": round(float(top_z), 9),
        "resting_gap_m": round(resting_gap, 9),
        "contact_geoms": sorted(set(contacts)),
        "payload_center_m": [round(float(v), 9) for v in payload_center],
        "target_center_m": [round(float(v), 9) for v in top_center],
        "offset_from_target_center_m": round(offset, 9),
        "last_speed_mps": round(speed, 9),
    }


def accumulate_carry_cadence(sink, previous_step_index, current_step_index):
    """累积"搬运节拍"实测：**一次控制迭代内植物前进了多少步**。

    为什么必须可观测（2026-09-28 §11.23(43)）：联合世界里"谁推进时间"由植物 owner 决定，而臂是
    **guest** ⇒ `_advance_for(0)` 在 guest 上是"等 owner 推进"，owner 可能一次推进很多步。搬运段
    把载荷**焊**在 mocap anchor 上，而 anchor 每个**控制迭代**只跟随一次 ⇒ 若一次迭代内植物前进
    步数过大，载荷会长时间挂在**陈旧 anchor** 上（实测均值 18.01 步/迭代）。这个前提此前是隐式的、
    只靠软约束兜着 ⇒ 现在由声明给出上限并**响亮失败**（不静默劣化）。

    返回：`{"max_plant_steps_per_iteration", "iterations", "last_delta"}`（就地返回同一个 sink）。
    """
    delta = int(current_step_index) - int(previous_step_index)
    sink["iterations"] = int(sink.get("iterations", 0)) + 1
    sink["last_delta"] = delta
    if delta > int(sink.get("max_plant_steps_per_iteration", 0)):
        sink["max_plant_steps_per_iteration"] = delta
    return sink


def check_carry_cadence(sink):
    """按**声明上限**判定节拍是否越界；越界即显式失败（返回说明或 None）。"""
    limit = int(sink.get("limit") or 0)
    measured = int(sink.get("max_plant_steps_per_iteration", 0))
    if limit <= 0:
        raise ValueError("搬运节拍检查缺少声明上限（carry_constraint.max_plant_steps_per_iteration）")
    if measured > limit:
        return ("搬运节拍超出声明上限: 单次控制迭代植物最多前进 %d 步 > 声明 %d 步"
                "（anchor 只在每个控制迭代跟随一次 ⇒ 步数越大，载荷挂陈旧 anchor 越久）"
                % (measured, limit))
    return None


#: "放置点纠偏"的合法模式（声明驱动；实现层不给默认值）
PLACE_POSE_CORRECTION_MODES = ("off", "measure_only", "lateral_only", "full_pose", "touchdown")


def resolve_touchdown_correction(declaration, payload_low_m, bearing_surface_z_m):
    """算**竖向触地纠偏量**（纯函数，便于单测；是否施加由调用方按 mode 决定）。

    为什么另开一个函数（2026-09-29 §11.25(f-4)）：`resolve_place_pose_correction` 的 delta 来自
    "**托盘**实测位姿 − 名义位姿"，而实测把它施加下去**反而更差**（offset 0.051933449 → 0.059755439）
    ⇒ 误差主项不在托盘位姿，而在**方块在夹口里下滑**：`PLACE_TRACE` 实测「载荷最低点 − 指腹中点」
    在放置段内变化 **24.7 mm**（start −0.063954 → after_above −0.039229 → after_descend −0.043797），
    而构建期在抓取位形上 FK 的关系是 −0.032008 ⇒ 运行时方块比名义低 ~12 mm，且**逐轮不同**
    ⇒ 松手高度逐轮不同 ⇒ 落点横向散布也逐轮不同（offset 实测摆动 0.0139~0.0630，判据 0.06）。

    口径：要消掉的量 = **载荷底面与承载面的实测间隙**（不是托盘位姿差），
      `gap_m       = payload_low_m − bearing_surface_z_m`
      `delta_z_m   = −(gap_m − touch_clearance_m)`（负 = 往下走；正 = 需要抬高）

    声明（实现层不写默认值，缺即失败）：
      · `touch_clearance_m`     —— 目标落位间隙（= 声明的触地间隙）；
      · `max_vertical_m`        —— 允许的竖向纠偏上限（超过即**显式拒绝**，不静默截断）；
      · `residual_tolerance_m`  —— 进此容差即视为无需纠偏（`applied=False`）。
    返回 dict（含 mode / applied / delta_z_m / gap_m / touch_clearance_m / max_vertical_m）。
    """
    if not isinstance(declaration, dict) or not declaration:
        raise ValueError(
            "缺少 place_pose_correction 声明（grasp.place_pose_correction）：实现层不给默认值")
    for label, value in (("payload_low_m", payload_low_m), ("bearing_surface_z_m", bearing_surface_z_m)):
        if not isinstance(value, (int, float)) or isinstance(value, bool) \
                or not np.isfinite(float(value)):
            raise ValueError("触地纠偏需要实测的 %s（有限数），实际: %r" % (label, value))
    required = {}
    for key in ("touch_clearance_m", "max_vertical_m", "residual_tolerance_m"):
        value = declaration.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) < 0:
            raise ValueError(
                "place_pose_correction.mode=touchdown 时必须声明非负的 %s（实现层不写默认值），"
                "实际: %r" % (key, value))
        required[key] = float(value)
    payload_low = float(payload_low_m)
    bearing = float(bearing_surface_z_m)
    gap = payload_low - bearing
    delta_z = -(gap - required["touch_clearance_m"])
    residual = abs(gap - required["touch_clearance_m"])
    report = {"mode": "touchdown", "applied": False,
              "gap_m": round(gap, 9), "delta_z_m": round(delta_z, 9),
              "residual_m": round(residual, 9),
              "payload_low_m": round(payload_low, 9), "bearing_surface_z_m": round(bearing, 9),
              **{key: required[key] for key in required}}
    if residual <= required["residual_tolerance_m"]:
        return report
    if abs(delta_z) > required["max_vertical_m"]:
        raise ValueError(
            "触地纠偏需要竖向移动 %.9f m（载荷底面 %.9f − 承载面 %.9f − 触地间隙 %.9f），"
            "超过声明上限 max_vertical_m=%.9f m ⇒ 拒绝放置（不静默截断、不按名义高度照放）"
            % (delta_z, payload_low, bearing, required["touch_clearance_m"],
               required["max_vertical_m"]))
    report["applied"] = True
    return report


def resolve_place_alignment_correction(declaration, payload_center_xy_m, tray_center_xy_m):
    """算**放置横向纠偏量**（纯函数，可单测）：把"载荷中心 − 托盘中心"作为要消掉的量。

    为什么（2026-09-29 §11.25(f-5) 实测）：`PLACE_TRACE` 逐相位给出横向偏移的**来源**——
      start 载荷→夹口 27.4 mm → after_transit **49.0 mm**（搬运段在夹口里**侧滑 21.6 mm**，已知现象）
      → after_descend 载荷→托盘 47.4 mm（释放前就已偏）→ after_retreat **61.8 mm**（释放瞬间又 +13.7）
      ⇒ 最终 61.8 mm = 释放前 47.4（主因搬运侧滑）+ 释放 13.7。只要把释放前纠到近零，
        最终就会落回 13.7 mm 量级 ⇒ 判据 0.06 m 能过（现状 0.061824263/0.065815019 刚好破线）。
    现有 `resolve_place_pose_correction` 的 delta 取自"托盘**位姿**差"，D2 实测施加后更差
    （0.051933449 → 0.059755439）⇒ 误差主项不在托盘位姿，而在"载荷相对托盘的位置"。

    口径：`delta_xy = 托盘中心 − 载荷中心`（把载荷往托盘中心挪），
      `lateral_m = |载荷中心 − 托盘中心|`；进 `lateral_tolerance_m` 即 `applied=False`；
      超过 `max_lateral_m` ⇒ **显式拒绝**（不静默截断、不按名义位置照放）。
    返回 dict：mode/applied/delta_xy_m/lateral_m/lateral_tolerance_m/max_lateral_m。
    """
    if not isinstance(declaration, dict) or not declaration:
        raise ValueError(
            "缺少 place_pose_correction 声明（grasp.place_pose_correction）：实现层不给默认值")
    payload_xy = [float(v) for v in payload_center_xy_m]
    tray_xy = [float(v) for v in tray_center_xy_m]
    if len(payload_xy) != 2 or len(tray_xy) != 2:
        raise ValueError("载荷中心与托盘中心都必须是 2 维（xy），实际 %r / %r"
                         % (payload_center_xy_m, tray_center_xy_m))
    for label, value in (("payload_center_xy_m", payload_xy), ("tray_center_xy_m", tray_xy)):
        for v in value:
            if not np.isfinite(float(v)):
                raise ValueError("横向纠偏需要有限的 %s，实际 %r" % (label, value))
    tolerance = declaration.get("lateral_tolerance_m")
    cap = declaration.get("max_lateral_m")
    for label, value in (("lateral_tolerance_m", tolerance), ("max_lateral_m", cap)):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) < 0:
            raise ValueError(
                "横向纠偏需要声明非负的 %s（实现层不写默认值），实际: %r" % (label, value))
    delta = [tray_xy[0] - payload_xy[0], tray_xy[1] - payload_xy[1]]
    lateral = float(np.hypot(payload_xy[0] - tray_xy[0], payload_xy[1] - tray_xy[1]))
    report = {"mode": "lateral_alignment", "applied": False,
              "delta_xy_m": [round(delta[0], 9), round(delta[1], 9)],
              "lateral_m": round(lateral, 9),
              "lateral_tolerance_m": float(tolerance), "max_lateral_m": float(cap)}
    if lateral <= float(tolerance):
        return report
    if lateral > float(cap):
        raise ValueError(
            "横向纠偏需要移动 %.9f m（载荷中心 %r − 托盘中心 %r），超过声明上限 max_lateral_m=%.9f m "
            "⇒ 拒绝放置（不静默截断、不按名义位置照放）"
            % (lateral, payload_xy, tray_xy, float(cap)))
    report["applied"] = True
    return report


def resolve_place_pose_correction(declaration, nominal_pose_m, live_pose_m):
    """解析"放置点纠偏"声明并算出**实测纠偏量**（纯函数，便于单测；是否施加由调用方按 mode 决定）。

    为什么需要（2026-09-29，§11.23(48)）：托盘随载体运动后，放置偏移 = 停靠误差 + 名义/实测量差，
    而放置四段航点是**构建期按名义停靠位姿**解出的。三轮实测 offset = 0.052625625 / 0.058340468 /
    0.056955541（判据 0.06）⇒ 余量最薄 1.7 mm，属"会随机变红"的验收脆弱点。根治只能把放置点纠到
    **运行期实测位姿**上（不调阈值），而"纠多少"必须先量出来。

    口径（旋转不改变模长 ⇒ 水平量与坐标系无关，故 delta 记世界系即可）：
      · `off`          ⇒ 不测量（`applied=False`，返回里无 `delta_world_m`）；
      · `measure_only` ⇒ 实测并留痕，`applied=False`；
      · `lateral_only` / `full_pose` ⇒ **尚未实现** ⇒ 声明了即**显式失败**（不得静默按名义位姿照放）；
      · 实测水平量超过声明的 `max_lateral_m` ⇒ **显式拒绝**（不静默截断）。
    """
    if not isinstance(declaration, dict) or not declaration:
        raise ValueError(
            "缺少 place_pose_correction 声明（grasp.place_pose_correction）：实现层不给默认值")
    mode = str(declaration.get("mode") or "")
    if mode not in PLACE_POSE_CORRECTION_MODES:
        raise ValueError(
            "place_pose_correction.mode 必须是 %s 之一，实际: %r"
            % (list(PLACE_POSE_CORRECTION_MODES), declaration.get("mode")))
    if mode == "off":
        return {"mode": mode, "applied": False}
    if nominal_pose_m is None:
        raise ValueError(
            "接收体记录缺少 nominal_pose_m（构建期名义停靠位姿）：无法量纠偏量 "
            "⇒ 拒绝而非按名义位姿照放")
    nominal = [float(v) for v in nominal_pose_m]
    live = [float(v) for v in live_pose_m]
    if len(nominal) != 3 or len(live) != 3:
        raise ValueError("nominal_pose_m 与实测位姿都必须是 3 维，实际 %r / %r" % (nominal, live))
    delta = [live[i] - nominal[i] for i in range(3)]
    lateral = float(np.linalg.norm(np.asarray(delta[:2], dtype=float)))
    max_lateral = declaration.get("max_lateral_m")
    if not isinstance(max_lateral, (int, float)) or isinstance(max_lateral, bool) \
            or not float(max_lateral) > 0:
        raise ValueError(
            "place_pose_correction 非 off 时必须声明正的 max_lateral_m（实现层不写默认值），实际: %r"
            % (max_lateral,))
    report = {
        "mode": mode,
        "applied": False,
        "delta_world_m": [round(value, 9) for value in delta],
        "lateral_m": round(lateral, 9),
        "vertical_m": round(delta[2], 9),
        "max_lateral_m": float(max_lateral),
    }
    if lateral > float(max_lateral):
        raise ValueError(
            "接收体实测位姿相对名义位姿偏 %.9f m（水平）> 声明上限 %.9f m ⇒ 拒绝放置"
            "（该量说明载体没有停到位；不静默截断、不按名义位姿照放）"
            % (lateral, float(max_lateral)))
    if mode in ("lateral_only", "full_pose"):
        # 施加所需的 IK 参数**必须声明**（实现层不写默认值）：迭代上限 / 步长 / 允许残差。
        for key in ("ik_iterations", "ik_step", "max_residual_m"):
            value = declaration.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not float(value) > 0:
                raise ValueError(
                    "place_pose_correction.mode=%s 时必须声明正的 %s（实现层不写默认值），实际: %r"
                    % (mode, key, value))
        report["ik_iterations"] = int(declaration["ik_iterations"])
        report["ik_step"] = float(declaration["ik_step"])
        report["max_residual_m"] = float(declaration["max_residual_m"])
    return report
