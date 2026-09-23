"""MuJoCo 状态 → MPC 输入（A6a-④ ②b 第一块）：口径见
`docs/debug/2026-09-23-a6a4-state-bridge-facts.md`（每个量的唯一来源 + 实测互证数字）。

上游语义（`build/research/mpc-repo/src/convex_mpc/go2_robot_data.py:74-97,176-192,210-211`，**原样复刻**）：
  `x = [pos_com_world(3), rpy(3), vel_com_world(3), omega_world(3)]`
  · `pos_com_world` / `vel_com_world` 取**质心**量（不是 base 原点）；
  · `rpy` = ZYX（pinocchio `matrixToRpy`），且 **yaw 逐拍解卷绕**（有状态：`yaw_prev` / `yaw_cont`）；
  · `omega_world = R_body_to_world @ ω_body`，`ω_body` 是**体坐标**角速度（MuJoCo 自由关节 `qvel[3:6]`）。

本模块是**纯逻辑 + 注入 MuJoCo**：不碰控制器、不碰锁、不写任何数字（质量/惯量从模型读）。
两处质量口径**必须**区分（见文档 §3）：机器人本体 = **trunk 子树**质量，
`sum(model.body_mass)` 会把挂在 `world` 下的场景物体算进来（实测差 0.04 kg）。
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "quat_wxyz_to_xyzw", "quat_to_rotation", "matrix_to_rpy", "ComStateTracker",
    "com_state_vector", "trunk_subtree_bodies", "subtree_mass_inertia", "hand_built_inertia",
    "robot_subtree_mass_kg",
    "STATE_DIM",
]

#: 状态维（与 `qp_builder.STATE_DIM` 同一事实：p(3) rpy(3) v(3) ω(3)）。
STATE_DIM = 12


def quat_wxyz_to_xyzw(quat_wxyz):
    """MuJoCo 的 `[w, x, y, z]` → 上游/pinocchio 的 `[x, y, z, w]`。

    写错**不会报错**（两者都是 4 个数），只会静默全错 ⇒ 本函数是唯一转换点，单测钉死。
    """
    q = np.asarray(quat_wxyz, dtype=float).reshape(-1)
    if q.size != 4:
        raise ValueError("四元数必须是 4 维，实际 %d" % q.size)
    out = np.array([q[1], q[2], q[3], q[0]], dtype=float)
    norm = float(np.linalg.norm(out))
    if not norm > 0.0:
        raise ValueError("四元数范数为 0：姿态未定义")
    return out / norm


def quat_to_rotation(quat_wxyz):
    """`[w,x,y,z]` 四元数 → 旋转矩阵（**body → world**，与 MuJoCo `xmat` 同向）。"""
    w, x, y, z = (float(v) for v in np.asarray(quat_wxyz, dtype=float).reshape(4))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not norm > 0.0:
        raise ValueError("四元数范数为 0：姿态未定义")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def matrix_to_rpy(rotation):
    """ZYX 欧拉角 `(roll, pitch, yaw)` —— 上游 `pin.rpy.matrixToRpy` 的语义。

    `roll = atan2(R21, R22)`、`pitch = asin(−R20)`、`yaw = atan2(R10, R00)`；
    pitch 在 ±π/2 处钳位（万向节奇异点不返回 NaN，与 `atan2/asin` 的实测行为一致）。
    """
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3):
        raise ValueError("旋转矩阵必须是 3×3，实际 %r" % (r.shape,))
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)


class ComStateTracker:
    """把**逐拍**姿态转成 12 维 MPC 状态（含上游的 **yaw 解卷绕**：有状态，跨拍保存）。

    为什么必须有状态：`yaw` 的原始测量落在 (−π, π]，跨越 ±π 时会**跳变 2π**；
    上游用「最小增量累加」把它连续化（`yaw_delta = (yaw_meas − yaw_prev + π) mod 2π − π`）。
    漏掉这一步，参考轨迹与 QP 状态在 ±π 处突跳 ⇒ 解不可用（本模块不允许"看起来能跑"）。
    """

    def __init__(self):
        self._yaw_prev_meas = 0.0          # 首次调用前的占位值（不会被读到：见 unwrap 的初始化分支）
        self._yaw_cont = None              # None ⇒ 还没调用过；否则为连续 yaw
        self.stats = {"calls": 0, "unwraps": 0}

    @property
    def yaw(self):
        """当前连续 yaw（未调用过 ⇒ None）。"""
        return None if self._yaw_cont is None else float(self._yaw_cont)

    def unwrap(self, yaw_meas):
        """连续化 yaw（首次调用直接以测量值初始化，与上游一致）。"""
        value = float(yaw_meas)
        self.stats["calls"] += 1
        if self._yaw_cont is None:         # 首次：以测量值初始化（上游 `_yaw_unwrap_initialized`）
            self._yaw_cont = value
            self._yaw_prev_meas = value
            return value
        delta = (value - self._yaw_prev_meas + math.pi) % (2.0 * math.pi) - math.pi
        self._yaw_cont += delta
        if abs(delta) > math.pi / 2.0:      # |Δ|>90° 是"卷绕发生"的迹象，计数供验收查看
            self.stats["unwraps"] += 1
        self._yaw_prev_meas = value
        return float(self._yaw_cont)

    def state_vector(self, pos_com_world, quat_wxyz, v_com_world, omega_body,
                     rotation=None):
        """返回 12 维状态 `[p, rpy, v, ω_world]`（yaw 走本 tracker）。

        `omega_body` 是**体坐标**角速度（MuJoCo 自由关节 `qvel[3:6]`）；
        `rotation` 可选传入（避免重复算四元数→矩阵），不传则内部由 `quat_wxyz` 算。
        """
        pos = np.asarray(pos_com_world, dtype=float).reshape(3)
        vel = np.asarray(v_com_world, dtype=float).reshape(3)
        omega = np.asarray(omega_body, dtype=float).reshape(3)
        r = quat_to_rotation(quat_wxyz) if rotation is None else np.asarray(rotation, dtype=float)
        if r.shape != (3, 3):
            raise ValueError("rotation 必须是 3×3，实际 %r" % (r.shape,))
        roll, pitch, yaw_meas = matrix_to_rpy(r)
        state = np.concatenate([pos, np.array([roll, pitch, self.unwrap(yaw_meas)]), vel,
                                r @ omega])
        out = state.reshape(STATE_DIM)
        if not np.all(np.isfinite(out)):
            raise ValueError("状态向量含非有限值：不得把坏状态喂给 MPC")
        return out


def com_state_vector(pos_com_world, quat_wxyz, v_com_world, omega_body, tracker,
                     rotation=None):
    """便捷函数（等价于 `ComStateTracker(...).state_vector(...)`；tracker 由调用方持有）。"""
    if not isinstance(tracker, ComStateTracker):
        raise TypeError("tracker 必须是 ComStateTracker（yaw 解卷绕必须有状态、跨拍保存）")
    return tracker.state_vector(pos_com_world, quat_wxyz, v_com_world, omega_body,
                                rotation=rotation)


def trunk_subtree_bodies(model, root_body):
    """`root_body` 的**整棵子树**（含自身）的 body 下标列表 —— 不含挂在 `world` 下的场景物体。

    ⚠ 踩过的坑：MuJoCo 里**世界体自指**（`body_parentid[0] == 0`）。若照抄"父→子"映射再朴素递归，
    从 root=0 走会**无限循环**（0 是自己的孩子）—— 本函数跳过自指边并用 `seen` 兜底环，
    使 `root=0` 退化为"整场景子树"（可枚举、可判据），而不是挂死。
    """
    root = int(root_body)
    if not 0 <= root < int(model.nbody):
        raise ValueError("root_body 越界：%r（nbody=%d）" % (root_body, model.nbody))
    children = {}
    for body in range(int(model.nbody)):
        parent = int(model.body_parentid[body])
        if parent == body:                      # 世界体自指 ⇒ 不是一条边
            continue
        children.setdefault(parent, []).append(body)
    out, seen, stack = [], set(), [root]
    while stack:
        node = stack.pop()
        if node in seen:                        # 环兜底（模型异常时也不无限扩张）
            continue
        seen.add(node)
        out.append(node)
        stack.extend(children.get(node, ()))
    return sorted(out)


def robot_subtree_mass_kg(model, root_body):
    """整机质量（kg）= 机器人**子树**内 body 的质量和（`root_body` = 机器人根，如躯干）。

    为什么不能用 `sum(model.body_mass)`（2026-09-24 实测，nbody=20 的场景模型）：

        sum(body_mass)          = 15.596408000 kg  ⇒ mg = 153.000762 N
        躯干子树内 body 质量     = 15.556408000 kg  ⇒ mg = 152.608362 N
        差 = 0.040000000 kg（台面上的自由道具 box_01）
        ⇒ mg 偏大 0.392400 N（0.2571%）

    口径规则（两类都能自洽解释）：
      · **挂载在机器人上的物体算机器人的质量** —— 本场景的托盘 `tray_01`（0.35 kg）是躯干子节点，
        就在子树内，应当计入；
      · **台面上带自由关节的道具不算** —— `box_01` 悬在 `world` 下，不属于机器人。
    这条质量进 **balance 路径的重力前馈**（`_leg_balance` 的 `mass`），口径错了就整体偏移支撑力；
    场景里换一个更重的道具时，旧口径的偏差会按道具质量线性放大。
    """
    bodies = trunk_subtree_bodies(model, root_body)
    return float(sum(float(model.body_mass[int(index)]) for index in bodies))


def subtree_mass_inertia(model, data, mujoco, root_body):
    """trunk 子树的 `(mass, com_world(3), inertia(3,3))`，惯量为**世界系、绕该子树质心**。

    唯一来源（实测互证，见文档 §1/§2）：`crb[root]` 的 `(Ixx,Iyy,Izz,Ixy,Ixz,Iyz)` + `[9]` 质量、
    `subtree_com[root]` 质心。**刻意不用** `sum(model.body_mass)`（会把场景游离物体算进来）。
    """
    root = int(root_body)
    bodies = trunk_subtree_bodies(model, root)
    crb = np.asarray(data.crb[root], dtype=float).reshape(-1)
    if crb.size != 10:
        raise ValueError("crb[%d] 必须是 10 个数的紧凑形式，实际 %d" % (root, crb.size))
    mass = float(crb[9])
    if not mass > 0.0:
        raise ValueError("trunk 子树质量必须为正，实际 %r（root_body=%d）" % (mass, root))
    com = np.asarray(data.subtree_com[root], dtype=float).reshape(3).copy()
    inertia = crb_to_matrix(crb)
    if not (np.all(np.isfinite(com)) and np.all(np.isfinite(inertia))):
        raise ValueError("子树质心/惯量含非有限值")
    eigenvalues = np.linalg.eigvalsh(inertia)
    if np.any(eigenvalues <= 0.0):
        raise ValueError("质心惯量不是正定（特征值 %r）：模型或状态不可用" % (eigenvalues.tolist(),))
    return mass, com, inertia, bodies


def crb_to_matrix(crb_entry):
    """`crb`/`cinert` 的 10 元紧凑形式 → 3×3（布局实测：`[0:6] = (Ixx,Iyy,Izz,Ixy,Ixz,Iyz)`）。"""
    c = np.asarray(crb_entry, dtype=float).reshape(-1)
    if c.size < 6:
        raise ValueError("紧凑惯量至少要 6 个数，实际 %d" % c.size)
    return np.array([[c[0], c[3], c[4]],
                     [c[3], c[1], c[5]],
                     [c[4], c[5], c[2]]], dtype=float)


def hand_built_inertia(model, data, root_body):
    """**独立算路**（验收用）：不读 `crb`，逐体按 `body_inertia`/`body_iquat`/`xmat`/`xipos` 装。

    与 `subtree_mass_inertia` 互为对照（实测最大差 1.110e-16）⇒ 生产只走前者、本函数作 oracle。
    返回 `(mass, com_world, inertia)`。
    """
    bodies = trunk_subtree_bodies(model, root_body)
    mass = float(sum(float(model.body_mass[i]) for i in bodies))
    if not mass > 0.0:
        raise ValueError("trunk 子树质量必须为正，实际 %r" % (mass,))
    com = np.zeros(3)
    for i in bodies:
        com += float(model.body_mass[i]) * np.asarray(data.xipos[i], dtype=float)
    com /= mass
    inertia = np.zeros((3, 3))
    for i in bodies:
        mi = float(model.body_mass[i])
        if mi == 0.0:
            continue
        rot = np.asarray(data.xmat[i], dtype=float).reshape(3, 3)          # body → world
        quat = np.asarray(model.body_iquat[i], dtype=float).reshape(4)     # 主轴在体坐标下的姿态
        principal = quat_to_rotation(quat)                                 # 主轴 → body
        local = principal @ np.diag(np.asarray(model.body_inertia[i], dtype=float)) @ principal.T
        r = np.asarray(data.xipos[i], dtype=float) - com
        inertia += rot @ local @ rot.T + mi * ((r @ r) * np.eye(3) - np.outer(r, r))
    return mass, com, inertia
