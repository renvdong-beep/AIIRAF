"""参考轨迹的**状态相关**部分（A6a-3b 第②步第二块）。

来源：上游 `com_trajectory.py:37-104 generate_traj` 的前半段（语义移植，公式逐行对齐）：
  1. `initial_x_vec = go2.compute_com_x_vec()` → 取 `(x0, y0, z0)` 与 `yaw`；
  2. **位置钳位**：`pos_des_world[x]` 与 `x0` 的距离超过 `max_pos_error` 时按 ±阈值夹回；
     y 同理；z 直接取目标高度；
  3. 时间向量 `t_vec = (np.arange(N) + 1) * dt`；
  4. 期望速度由**机体系**经 `R_z` 旋到世界系：`v_world = R_z @ [vx, vy, 0]`；
  5. `pos_traj_world = p_des + v_world ⊗ t`（广播）；`vel_traj_world = v_world`（视界内常量）；
  6. `rpy_traj_world = [0, 0, yaw + yaw_rate·t]`；`omega_traj_world = [0, 0, yaw_rate]`。

**不含**（下一块）：足端轨迹（Raibert 落足点 + 摆动腿五次多项式）与 `compute_x_ref_vec`；
接触表按移植清单 §1.1 **复用 `iraf_adapters.unitree.gait`**，不在本模块实现。

未验证边界：与本模块配套的**逐位校验**（对研究侧同批状态比较 `pos/vel/rpy/omega_traj_world`）
仍是下一步；在此之前不得接进 `provider_runtime`。
"""

from __future__ import annotations

import numpy as np

__all__ = ["yaw_rotation", "reference_state_trajectory"]


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
