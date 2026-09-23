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

__all__ = ["pose_error", "approach_command", "DOCK_DEFAULTS_FORBIDDEN"]


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
