"""参考轨迹的**状态相关**部分（A6a-3b 第②步第二块）。

来源：上游 `com_trajectory.py:37-104 generate_traj` 的前半段（语义移植，公式逐行对齐）：
  1. `initial_x_vec = go2.compute_com_x_vec()` → 取 `(x0, y0, z0)` 与 `yaw`；
  2. **位置钳位**：`pos_des_world[x]` 与 `x0` 的距离超过 `max_pos_error` 时按 ±阈值夹回；
     y 同理；z 直接取目标高度；
  3. 时间向量 `t_vec = (np.arange(N) + 1) * dt`；
  4. 期望速度由**机体系**经 `R_z` 旋到世界系：`v_world = R_z @ [vx, vy, 0]`；
  5. `pos_traj_world = p_des + v_world ⊗ t`（广播）；`vel_traj_world = v_world`（视界内常量）；
  6. `rpy_traj_world = [0, 0, yaw + yaw_rate·t]`；`omega_traj_world = [0, 0, yaw_rate]`。

**不含**（下一块）：`compute_x_ref_vec`；接触表按移植清单 §1.1 **复用 `iraf_adapters.unitree.gait`**
（`mpc/contact.py`）。足端参考见本模块 `foot_reference_trajectory`。

未验证边界：与本模块配套的**逐位校验**（对研究侧同批状态比较 `pos/vel/rpy/omega_traj_world`）
仍是下一步；在此之前不得接进 `provider_runtime`。
"""

from __future__ import annotations

import numpy as np

from iraf_adapters.unitree.mpc.contact import LEG_ORDER

__all__ = ["yaw_rotation", "reference_state_trajectory", "foot_reference_trajectory",
           "REQUIRED_TOUCHDOWN_KEYS", "touchdown_parameters"]

#: `mpc_model.touchdown` 必需键（缺项即显式失败；多余键同样失败 ⇒ 拒绝"假声明"）。
REQUIRED_TOUCHDOWN_KEYS = ("nominal_z_m", "swing_factor", "stance_half_factor", "lookahead_factor")


def touchdown_parameters(gait_params, touchdown_declaration):
    """由 **trot 步态声明** + **touchdown 声明** 派生 `foot_reference_trajectory` 的调用参数。

    `t_swing` / `t_stance` 由步态声明派生（`(1−duty)·period` / `duty·period`），
    不在 touchdown 段重复声明；`pred_time` 由三个系数算出。
    返回 `{"nominal_z_m", "swing_time_s", "stance_time_s", "pred_time_s", "T_s"}`。
    """
    if not isinstance(touchdown_declaration, dict):
        raise ValueError("touchdown 声明必须是映射，实际: %r" % (type(touchdown_declaration).__name__,))
    missing = [key for key in REQUIRED_TOUCHDOWN_KEYS if key not in touchdown_declaration]
    if missing:
        raise ValueError("mpc_model.touchdown 缺少必需键: %s" % missing)
    extra = [key for key in touchdown_declaration if key not in REQUIRED_TOUCHDOWN_KEYS]
    if extra:
        raise ValueError("mpc_model.touchdown 含未定义键: %s（多余键视为假声明）" % extra)
    values = {}
    for key in REQUIRED_TOUCHDOWN_KEYS:
        values[key] = float(touchdown_declaration[key])
    for key, value in values.items():
        if not value > 0:
            raise ValueError("mpc_model.touchdown.%s 必须为正数，实际 %r" % (key, value))
    duty = float(gait_params["duty_factor"])
    period = float(gait_params["period_s"])
    swing_time = (1.0 - duty) * period
    stance_time = duty * period
    horizon = values["swing_factor"] * swing_time + values["stance_half_factor"] * stance_time
    return {
        "nominal_z_m": values["nominal_z_m"],
        "swing_time_s": swing_time,
        "stance_time_s": stance_time,
        "T_s": horizon,
        "pred_time_s": values["lookahead_factor"] * horizon,
    }


def foot_reference_trajectory(masks, base_pos_traj, base_vel_body, r_z, yaw_rate_des,
                              hip_offsets, nominal_z_m, pred_time_s):
    """足端参考轨迹（上游 `com_trajectory.py:113-207` 的**逐拍状态机**语义移植）。

    上游行为（逐字复刻，含其"混合坐标系"的既有做法）：
      · 参考足端位置是**相对机身**的向量（上游变量名带 `_world` 但实际减去了
        `p_base_traj_world = current_config.base_pos`），且**逐拍分段常量**；
      · 每条腿维护一个 `mask_previous`（初值 **2**，与 0/1 都不同 ⇒ 第 0 拍必然走跳变分支）：
          跳变到 0（离地）⇒ 记录下一落足点 `r_next_td`，当拍参考置 `[0,0,0]`；
          跳变到 1（触地）⇒ 当拍参考 = 被记录的 `r_next_td`；
          掩码未变 ⇒ 参考 = 上一拍的值（**递推**，故第 0 拍不会取到 `[-1]`）。
      · 落足点（上游 `gait.compute_touchdown_world_for_traj_purpose_only`）：
          `body_pos = [base_pos[0], base_pos[1], 0]`；`hip_pos_world = body_pos + R_z @ hip_offset`
          `T = swing_time + 0.5·stance_time`；`pred_time = T/2`（**由 `touchdown_parameters` 一次算出**，
          本函数只收 `pred_time_s`，不在两处各算一遍）
          `nominal = [hip_pos_world[0], hip_pos_world[1], nominal_z_m]`（z 为**常量**，非机身高度）
          `drift   = [base_vel_body[0]·pred, base_vel_body[1]·pred, 0]`
          `dtheta  = yaw_rate_des · pred`；`r_xy = nominal[:2] − base_pos[:2]`
          `rot     = [−dtheta·r_xy[1], dtheta·r_xy[0], 0]`
          `td = nominal + drift + rot` ⇒ 参考 = `td − base_pos`
        ⚠ 上游把 `dq[0:3]` 传的是**体坐标系**速度（`R_world_to_body @ v_world`）却直接当世界系
          drift 用；`yaw_rate_des_world` 实际赋的是**体坐标系**偏航角速度（其 L104）。
          这是上游既有事实，逐位复刻，不在移植中"顺手修正"。

    参数：`masks` = `(4,N)` 接触表（行序 = `LEG_ORDER`，1=支撑）；`base_pos_traj` = `(3,N)`
    机身参考位置（即 `reference_state_trajectory` 的 `pos_traj_world`）；`base_vel_body` = `(3,)`
    或 `(3,N)`；`r_z` = `(3,3)`（由**当前状态**偏航构造，非轨迹偏航）；`yaw_rate_des` 标量；
    `hip_offsets` = `{腿码: (3,)}`；`nominal_z_m` 与两个时间由调用方给出（本模块不内置数字）。
    返回 `{腿码: (3,N)}`（顺序同 `LEG_ORDER`）。
    """
    masks = np.asarray(masks, dtype=np.int64)
    if masks.ndim != 2 or masks.shape[0] != len(hip_offsets):
        raise ValueError("masks 形状必须为 (腿数, N)，实际 %r" % (masks.shape,))
    n_legs, n = masks.shape
    pos = np.asarray(base_pos_traj, dtype=float)
    if pos.shape != (3, n):
        raise ValueError("base_pos_traj 形状必须为 (3, N)=%r，实际 %r" % ((3, n), pos.shape))
    vel = np.asarray(base_vel_body, dtype=float)
    if vel.ndim == 2:
        if vel.shape != (3, n):
            raise ValueError("base_vel_body 形状必须为 (3,) 或 (3,N)，实际 %r" % (vel.shape,))
    elif vel.shape != (3,):
        raise ValueError("base_vel_body 形状必须为 (3,) 或 (3,N)，实际 %r" % (vel.shape,))
    r_z = np.asarray(r_z, dtype=float)
    if r_z.shape != (3, 3):
        raise ValueError("r_z 形状必须为 (3,3)，实际 %r" % (r_z.shape,))
    pred_time = float(pred_time_s)
    if not pred_time > 0:
        raise ValueError("pred_time_s 必须为正，实际 %r" % (pred_time_s,))

    # 腿序必须由 `contact.LEG_ORDER` 决定（与接触表行序同一事实），不靠字典迭代顺序：
    # 顺序写错会静默把一条腿的落足点安到另一条腿上，而数值上"看起来很合理"。
    missing = [code for code in LEG_ORDER if code not in hip_offsets]
    extra = [code for code in hip_offsets if code not in LEG_ORDER]
    if missing or extra:
        raise ValueError(
            "hip_offsets 必须恰好覆盖 LEG_ORDER=%s（缺 %s／多 %s）"
            % (list(LEG_ORDER), missing, extra)
        )
    codes = list(LEG_ORDER)
    offsets = {}
    for code in codes:
        offset = np.asarray(hip_offsets[code], dtype=float).reshape(-1)
        if offset.size != 3:
            raise ValueError("hip_offsets[%r] 必须是 3 维，实际 %r" % (code, offset.size))
        offsets[code] = offset

    nominal_z = float(nominal_z_m)
    out = {code: np.zeros((3, n), dtype=float) for code in codes}
    next_td = {code: np.zeros(3, dtype=float) for code in codes}
    previous = [2] * n_legs          # 上游初值：与 0/1 都不同 ⇒ 第 0 拍必走跳变分支
    for i in range(n):
        current = [int(v) for v in masks[:, i]]
        for k, code in enumerate(codes):
            base_pos = pos[:, i]
            vel_i = vel if vel.ndim == 1 else vel[:, i]
            if current[k] != previous[k] and current[k] == 0:
                body_pos = np.array([base_pos[0], base_pos[1], 0.0])
                hip_pos_world = body_pos + r_z @ offsets[code]
                nominal = np.array([hip_pos_world[0], hip_pos_world[1], nominal_z])
                drift = np.array([vel_i[0] * pred_time, vel_i[1] * pred_time, 0.0])
                dtheta = float(yaw_rate_des) * pred_time
                r_xy = nominal[:2] - np.asarray([base_pos[0], base_pos[1]])
                rot = np.array([-dtheta * r_xy[1], dtheta * r_xy[0], 0.0])
                next_td[code] = nominal + drift + rot - base_pos
                out[code][:, i] = 0.0
            if current[k] != previous[k] and current[k] == 1:
                out[code][:, i] = next_td[code]
            if current[k] == previous[k]:
                if i == 0:
                    raise ValueError(
                        "接触掩码第 0 拍与初值相同（上游初值 2 保证不会发生）：腿 %s" % code
                    )
                out[code][:, i] = out[code][:, i - 1]
        previous = current
    return out


def yaw_rotation(yaw):
    """`R_z`（上游用 yaw 构造；此处为**当前偏航**，与 `dynamics.yaw_avg_rotation` 的
    "视界平均"是两处不同用途，勿混）。"""
    c, s = np.cos(float(yaw)), np.sin(float(yaw))
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def reference_state_trajectory(initial_x_vec, com_pos_world, yaw, n_steps, time_step,
                               vx_body, vy_body, z_des, yaw_rate, pos_des_world,
                               max_pos_error):
    """返回 `(pos_traj_world, vel_traj_world, rpy_traj_world, omega_traj_world, pos_des_world_out)`。

    参数（全部由调用方给出，本模块不内置任何数字）：
    - `initial_x_vec`：12 维状态（p, rpy, v, ω）；`(x0, y0, z0)` 取前三维
    - `com_pos_world`：当前质心位置（用于钳位判据，通常即 `initial_x_vec[0:3]`）
    - `pos_des_world`：期望质心位置（会被就地钳位后返回副本）
    - `max_pos_error`：钳位阈值（上游 `0.1 m`，由声明给出）
    """
    state = np.asarray(initial_x_vec, dtype=float).reshape(-1)
    if state.size < 12:
        raise ValueError("initial_x_vec 至少 12 维，实际 %d" % state.size)
    com = np.asarray(com_pos_world, dtype=float).reshape(3)
    n = int(n_steps)
    dt = float(time_step)
    if n < 1 or not dt > 0:
        raise ValueError("n_steps 必须 ≥1、time_step 必须为正（n=%r, dt=%r）" % (n_steps, time_step))
    thr = float(max_pos_error)
    if not thr >= 0:
        raise ValueError("max_pos_error 必须非负，实际 %r" % (max_pos_error,))

    x0, y0, _z0 = com
    p_des = np.asarray(pos_des_world, dtype=float).reshape(3).copy()
    # 1) 位置钳位（上游逐轴夹回）
    if p_des[0] - x0 > thr:
        p_des[0] = x0 + thr
    if x0 - p_des[0] > thr:
        p_des[0] = x0 - thr
    if p_des[1] - y0 > thr:
        p_des[1] = y0 + thr
    if y0 - p_des[1] > thr:
        p_des[1] = y0 - thr
    p_des[2] = float(z_des)

    # 2) 时间向量与期望速度
    t_vec = (np.arange(n) + 1) * dt
    R_z = yaw_rotation(yaw)
    v_world = R_z @ np.array([float(vx_body), float(vy_body), 0.0], dtype=float)

    # 3) 四条轨迹
    pos_traj = p_des.reshape(3, 1) + v_world.reshape(3, 1) * t_vec.reshape(1, n)
    vel_traj = np.repeat(v_world.reshape(3, 1), n, axis=1)
    rpy_traj = np.zeros((3, n), dtype=float)
    rpy_traj[2, :] = float(yaw) + float(yaw_rate) * t_vec
    omega_traj = np.zeros((3, n), dtype=float)
    omega_traj[2, :] = float(yaw_rate)
    return pos_traj, vel_traj, rpy_traj, omega_traj, p_des
