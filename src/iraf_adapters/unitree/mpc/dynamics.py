"""离散动力学（A6a-3b 第②步的第一块）：`Ad` / `Bd` / `gd`。

来源：上游 `com_trajectory.py:272 _discreteDynamics` 的**语义移植**（常量与公式逐行对齐）：
  Ad = I12；Ad[0:3, 6:9] = dt·I3；Ad[3:6, 9:12] = dt·RzT（rpy ← ω 的一阶近似）
  gd[0:3] = ½ g dt²；gd[6:9] = g dt（g = [0,0,-9.81]）
  Bd[i] 的块：p ← f 为 (½dt²/m)·I3（四块）；v ← f 为 (dt/m)·I3（四块）；
              ω ← f 为 dt·(I_inv @ skew(r_i))；rpy ← f 为 ½dt²·(RzT @ I_inv @ skew(r_i))

与上游的差异（**只此一处，且已论证**）：上游 `_continuousDynamics` 还会算 `Ac/Bc/gc`，但那三个数组
在全工程**没有读取方**（调试记录 §15）；删除它们可把 `generate_traj` P50 从 4.7170 降到 2.9958 ms
且被消费的 13 个数组**逐位一致** ⇒ 本模块不实现它们（不是"少实现"，是"去死代码"）。

未做（下一步，见 `docs/debug/2026-09-23-qp-builder-port-plan.md` §2）：与本模块配套的
**逐位校验**——用 `traj_parity` 门禁把 `Ad/Bd/gd` 与研究侧同一批状态下的实测值做 `tobytes()` 比对。
在此之前本模块**不得**被接进 `provider_runtime`（未验证不接入）。
"""

from __future__ import annotations

import numpy as np

NX = 12
GRAVITY = np.array([0.0, 0.0, -9.81], dtype=float)

__all__ = ["skew", "yaw_avg_rotation", "discrete_dynamics"]


def skew(vector):
    """`_skew` 的等价实现（上游 `com_trajectory.py:213`）。"""
    v = np.asarray(vector, dtype=float).reshape(3)
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]], dtype=float)


def yaw_avg_rotation(rpy_traj_world):
    """上游用**整段视界平均偏航**构造 `RzT`（`_discreteDynamics` 内 `yaw_avg`）。

    ⚠ 保持与上游一致（含这个"取平均"的取法）：它对我们已量化为**可忽略**（对 yaw 速率预测的影响
    0.0556%~1.3857%，见调试记录 §26.1），但**不得**顺手改成"当前偏航"——那是改语义，不是保值移植。
    """
    yaw = float(np.average(np.asarray(rpy_traj_world, dtype=float)[2, :]))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s, 0.0],
                     [-s, c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def discrete_dynamics(dt, mass, inertia_com_world, rpy_traj_world, foot_world):
    """返回 `(Ad, Bd, gd)`。

    参数
    ----
    dt               : 离散步长（= GAIT_T/16 = 0.020833 s）
    mass             : 整机质量（由调用方从声明/模型给出，本模块不内置数字）
    inertia_com_world: 质心惯量（3×3，世界系）
    rpy_traj_world   : 形状 (3, N) 的参考姿态（用其**平均偏航**，与上游一致）
    foot_world       : 形状 (4, 3, N) 的足端世界位置，顺序 = (FL, FR, RL, RR)
    """
    rpy = np.asarray(rpy_traj_world, dtype=float)
    feet = np.asarray(foot_world, dtype=float)
    if rpy.ndim != 2 or rpy.shape[0] != 3:
        raise ValueError("rpy_traj_world 形状应为 (3, N)，实际 %s" % (rpy.shape,))
    n = rpy.shape[1]
    if feet.shape != (4, 3, n):
        raise ValueError("foot_world 形状应为 (4, 3, %d)，实际 %s" % (n, feet.shape))
    dt = float(dt)
    m = float(mass)
    if not dt > 0 or not m > 0:
        raise ValueError("dt 与 mass 必须为正（dt=%r, mass=%r）" % (dt, mass))

    RzT = yaw_avg_rotation(rpy)

    # ---- Ad ----
    Ad = np.eye(NX, dtype=float)
    Ad[0:3, 6:9] = dt * np.eye(3)
    Ad[3:6, 9:12] = dt * RzT

    # ---- gd ----
    gd = np.zeros((NX, 1), dtype=float)
    gd[0:3, 0] = 0.5 * GRAVITY * dt * dt
    gd[6:9, 0] = GRAVITY * dt

    # ---- Bd ----
    Bd = np.zeros((n, NX, NX), dtype=float)
    I_inv = np.linalg.inv(np.asarray(inertia_com_world, dtype=float))
    Bp = (0.5 * dt * dt / m) * np.eye(3)
    Bv = (dt / m) * np.eye(3)
    for i in range(n):
        W = [I_inv @ skew(feet[leg, :, i]) for leg in range(4)]
        Bi = Bd[i]
        for leg in range(4):
            Bi[0:3, 3 * leg:3 * leg + 3] = Bp
            Bi[6:9, 3 * leg:3 * leg + 3] = Bv
            Bi[9:12, 3 * leg:3 * leg + 3] = dt * W[leg]
            Bi[3:6, 3 * leg:3 * leg + 3] = 0.5 * dt * dt * (RzT @ W[leg])
    return Ad, Bd, gd
