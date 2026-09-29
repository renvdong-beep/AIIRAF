"""停靠（`dock_for_handoff`）的**纯函数层**：位姿误差 → 接近速度指令。

为什么单独一层（与 `mpc/torque_provider.py` 同取向）：
  · 判据与阈值**全部来自声明**（场景的 `translation_error_max_m` / `yaw_error_max_deg`、
    安全策略的 `max_speed_mps` / `max_yaw_rate_rad_s`）—— 本层不写任何数字默认值；
  · 本层只做「误差 → 指令」的几何与限幅，**不碰模型、不碰锁、不碰时钟** ⇒ 可纯单测，
    并可被任何机型复用（跨机型差异只能出现在调用方传入的容差里）。

调用链（下一步接线）：适配器 `dock_for_handoff` 读目标帧位姿 → 本层算指令 → `locomote` 执行
→ 误差进入容差后保持 → 报告末态误差与末速。失败路径（目标帧缺失/超时/不收敛）由调用方显式处理。
"""

from __future__ import annotations

import math

__all__ = ["pose_error", "approach_command", "stopping_distance_m",
           "resolve_dock_station", "assert_target_is_world_fixed", "DOCK_DEFAULTS_FORBIDDEN"]


class DockDeclarationError(ValueError):
    """停靠参数不合契约（缺项、非法值、非有限值）。"""


#: 语义提醒：本层**不得**出现默认值。缺参数即显式失败，避免"隐式默认值"把漏声明伪装成成功。
DOCK_DEFAULTS_FORBIDDEN = True


def _finite(value, label):
    value = float(value)
    if not math.isfinite(value):
        raise DockDeclarationError("%s 必须有限，实际 %r" % (label, value))
    return value


def _positive(value, label):
    value = _finite(value, label)
    if value <= 0.0:
        raise DockDeclarationError("%s 必须 > 0，实际 %r" % (label, value))
    return value


def pose_error(target_xy, target_yaw_rad, body_xy, body_yaw_rad):
    """目标帧与机身的位姿误差 → `(dx_m, dy_m, yaw_error_rad)`。

    · 平移误差在**世界系**（`target − body`），供调用方决定前进方向；
    · 偏航误差解卷绕到 `(-pi, pi]`（跨越 ±π 时不得给出 2π 级的假误差 ——
      实测踩过：未解卷绕的 yaw 误差会让接近指令整圈打转）；
    · 目标/机身位姿都必须有限（NaN/Inf 一律显式失败，不静默回到 0）。
    """
    tx, ty = _finite(target_xy[0], "target_xy[0]"), _finite(target_xy[1], "target_xy[1]")
    bx, by = _finite(body_xy[0], "body_xy[0]"), _finite(body_xy[1], "body_xy[1]")
    tyaw = _finite(target_yaw_rad, "target_yaw_rad")
    byaw = _finite(body_yaw_rad, "body_yaw_rad")
    yaw_error = (tyaw - byaw + math.pi) % (2.0 * math.pi) - math.pi
    return (tx - bx, ty - by, yaw_error)


def approach_command(dx_m, dy_m, yaw_error_rad, *, gain_s_inv, max_speed_mps, max_yaw_rate_rad_s,
                     position_tolerance_m, yaw_tolerance_rad, body_yaw_rad):
    """位姿误差 → 机身系速度指令 `(vx_mps, vy_mps, wz_rad_s)`（含限幅与"到位即零"）。

    规则（全部来自调用方传入的声明值，本函数不含任何数字默认值）：
      1. **到位判据**：`hypot(dx,dy) <= position_tolerance_m` 且 `|yaw_error| <= yaw_tolerance_rad`
         ⇒ 返回精确 `(0.0, 0.0, 0.0)`（停下；末速判据由调用方在保持窗内验收）；
      2. **世界系误差 → 机身系**：按机身偏航旋转（`vx` 前、`vy` 左），否则侧向误差会被当成前进量；
      3. **比例增益**：`v = gain · 误差`，再按声明的速度/角速度上限**双向限幅**（只能收紧）；
      4. **偏航优先**：角度误差未进入容差时**大幅降速平移**（`vx`/`vy` 乘 `|cos(yaw_error)|` 的下界 0
         之外，用 `min(1, tolerance/|yaw_error|)` 这类量会引入隐含常数，故此处只做"旋转时前进分量按
         `max(0, cos(yaw_error))` 缩放"——几何量，不含经验常数）；
      5. 所有入参必须有限，上限必须 > 0（否则显式失败，不静默).
    """
    dx = _finite(dx_m, "dx_m")
    dy = _finite(dy_m, "dy_m")
    yaw_error = _finite(yaw_error_rad, "yaw_error_rad")
    gain = _positive(gain_s_inv, "gain_s_inv")
    v_cap = _positive(max_speed_mps, "max_speed_mps")
    w_cap = _positive(max_yaw_rate_rad_s, "max_yaw_rate_rad_s")
    p_tol = _positive(position_tolerance_m, "position_tolerance_m")
    y_tol = _positive(yaw_tolerance_rad, "yaw_tolerance_rad")
    yaw_body = _finite(body_yaw_rad, "body_yaw_rad")

    if math.hypot(dx, dy) <= p_tol and abs(yaw_error) <= y_tol:
        return (0.0, 0.0, 0.0)

    cos_yaw, sin_yaw = math.cos(yaw_body), math.sin(yaw_body)
    forward_world = gain * dx
    left_world = gain * dy
    vx = cos_yaw * forward_world + sin_yaw * left_world          # 世界 → 机身（R(θ)ᵀ）
    vy = -sin_yaw * forward_world + cos_yaw * left_world
    # 偏航未到位时按几何投影抑制平移分量（cos<0 的朝向 = 目标在身后 ⇒ 先转不前进）
    swing = max(0.0, math.cos(yaw_error))
    vx *= swing
    vy *= swing
    wz = max(-w_cap, min(w_cap, gain * yaw_error))
    magnitude = math.hypot(vx, vy)
    if magnitude > v_cap:
        scale = v_cap / magnitude
        vx *= scale
        vy *= scale
    return (vx, vy, wz)


def stopping_distance_m(v_measured_mps, braking_lead_s, min_distance_m):
    """**制动提前量**：按实测速度提前多少米发零指令（= `提前量时间 × 速度`，下限 `min_distance_m`）。

    为什么需要它（2026-09-24 停靠实测，docs/debug/2026-09-24-dock-target-self-frame.md §6.3）：
    以"误差进容差即发零"的方式停步态时，机身**带着惯性滑行**：实测到位瞬间 |v| = 0.039549 m/s、
    随后 1.01 s 内又走了 40.6 mm（之后 4 s 只再走 6 mm）⇒ 滑行超调把末态撑到 0.06 m > 判据 0.03 m。
    滑行来源是"站定冻结要等 ≥1 个步态周期"（实测冻结时刻 ≈ 指令归零后 0.33~0.40 s）加上步态在
    零指令附近的**速度地板**。因此把判据从"误差 ≤ 容差"改成"误差 ≤ 制动距离"，
    让滑行把残差带到 ~0。

    参数全部来自调用方（本层无默认值）：`v_measured_mps` 实测机身速度、`braking_lead_s` 提前量时间、
    `min_distance_m` 下限（= 声明的控制容差，保证低速时仍按容差停）。
    """
    v = _finite(v_measured_mps, "v_measured_mps")
    lead = _finite(braking_lead_s, "braking_lead_s")
    floor = _positive(min_distance_m, "min_distance_m")
    if lead < 0.0:
        raise DockDeclarationError("braking_lead_s 必须 ≥ 0，实际 %r" % (braking_lead_s,))
    if v < 0.0:
        raise DockDeclarationError("v_measured_mps 必须 ≥ 0（速度模长），实际 %r" % (v_measured_mps,))
    return max(floor, lead * v)


def resolve_dock_station(section, station=None):
    """把**站名**解析成场景帧名（声明驱动；纯函数，不碰模型）。

    声明形状（机型声明的 `dock_for_handoff` 段）：
      · `target_frame: <帧名>`            —— 默认站位的帧（旧行为的唯一事实来源，保留）
      · `stations: {<站名>: <帧名>, ...}`  —— 可选；有它才能**按站名**选择
      · `default_station: <站名>`          —— 有 stations 时必需，其帧必须 == target_frame

    为什么要有这一层（2026-09-29，双臂轮转演示需要第二个站位）：
      · "走到哪个站"是**场景事实**，且站位数量随演示增长 ⇒ 站名进声明，新站位只改声明；
      · 调用方**只能给站名**（坐标、容差、速度等数字一律来自声明，见
        `skills/dock_for_handoff/dock_for_handoff.input.json` 的输入契约）；
      · 缺声明即显式失败，不猜（`DOCK_DEFAULTS_FORBIDDEN` 同一纪律）。

    返回 `(target_frame, station_name)`；未登记 stations 时 station_name 为 None。
    """
    target_frame = str(section["target_frame"])
    stations = section.get("stations")
    if stations is None:
        if station is not None and str(station) != "":
            raise DockDeclarationError(
                "调用方指定了站位 %r，但声明 dock_for_handoff 没有 stations 登记 ⇒ 无法解析站名"
                "（要么在声明里登记站位，要么不要指定）" % (station,))
        return target_frame, None
    if not isinstance(stations, dict) or not stations:
        raise DockDeclarationError("dock_for_handoff.stations 必须是非空对象（站名 → 场景帧名）")
    default_station = section.get("default_station")
    if not isinstance(default_station, str) or not default_station:
        raise DockDeclarationError(
            "声明了 dock_for_handoff.stations 就必须声明 default_station（缺省值必须显式给出，不猜）")
    if str(default_station) not in stations:
        raise DockDeclarationError(
            "dock_for_handoff.default_station=%r 不在 stations 里：%s"
            % (default_station, sorted(str(key) for key in stations)))
    # 自洽门禁：默认站位的帧必须与 target_frame 逐字相同（同一事实不出现两处）
    if str(stations[str(default_station)]) != target_frame:
        raise DockDeclarationError(
            "dock_for_handoff.target_frame=%r 与 stations[%r]=%r 不一致："
            "target_frame 记的就是默认站位的帧，两处必须一致"
            % (target_frame, default_station, stations[str(default_station)]))
    station_name = str(station) if station else str(default_station)
    if station_name not in stations:
        raise DockDeclarationError(
            "未知站位 %r：声明登记的站位是 %s（站名只能来自声明，不接受坐标）"
            % (station_name, sorted(str(key) for key in stations)))
    return str(stations[station_name]), station_name


def assert_target_is_world_fixed(*, frame_label, frame_kind, frame_body_id, frame_body_label,
                                 robot_body_ids, robot_root_label):
    r"""停靠目标帧必须**世界固定** —— 不得与机器人本体刚性相连。

    为什么必须是硬门禁（2026-09-24 实测，`build/iraf-a6a14/dock_frame_probe.py` 可复跑）：
    本场景 `scene.yaml` 的 `props.tray_01.pose.mount` 把托盘**挂在四足背上**
    （`entity: unitree_go2` / `frame: tray_frame`），于是 `dock_for_handoff` 的
    `target_frame: tray_frame` 与机身是**同一刚体**，停靠量到的是**自指量**：
      · 偏航误差恒为 `0.0`（同一个 `xmat`，`atan2` 差逐位为 0）——判据「偏航 ≤ 2.0°」永远"通过"；
      · 平移误差恒为「帧的本地偏置在机身姿态下的**xy 投影**」：实测 `3.632386e-05 m`
        = \|R·(0, 0, 0.057)\|_{xy}（探针复算 3.634612751e-05，两者相对差 6.129e-04，
        差异只来自探针用的是保持段末态、不是测量那一拍）；
      · 第 0 拍就已经"在位"（报告 `settled_at_s = 0.0`）⇒ 接近过程**从未被执行**，
        61.5 s 的指令恒为零。
    即：这类目标会产出**看起来完美、实际什么都没做**的停靠验收 —— 等价于伪造成功
    （AGENTS.md 1.5/1.6），因此必须在**执行前**显式失败，而不是留下一个好看的数字。

    判定：目标帧的所属 body 落在机器人子树内 ⇒ 失败。世界固定的台面对象（静态 props）
    与自由物体（有自由关节的 props）都不在子树内，正常通过。
    """
    on_robot = {int(item) for item in robot_body_ids}
    body_id = int(frame_body_id)
    if body_id in on_robot:
        raise DockDeclarationError(
            "停靠目标帧 %s（%s，所属 body id=%d %s）**刚性挂在机器人 %s 上**："
            "目标帧与机身同一刚体 ⇒ 位姿误差恒为自指量（偏航恒 0、平移 = 帧本地偏置的投影），"
            "接近过程不会被执行，验收数字无意义。"
            "目标帧必须来自**世界固定**的交接站位（例：台面上的 station frame —— "
            "四足要到得了、机械臂够得着的位置），而不是本体自带的挂载帧。"
            % (frame_label, frame_kind, body_id, frame_body_label, robot_root_label)
        )
    return None
