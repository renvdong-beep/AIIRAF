"""MPC 足端力 → 关节力矩，并产出 B1 契约载荷（A6a-④ 的第一块）。

接入契约见 `docs/debug/2026-09-23-qp-builder-port-plan.md` §1.2（带 file:line）：
`torque_provider(cycle_index, info)` 可返回 `{"balance_torque_nm", "position_weight"}`，
框架侧随后执行 `τ = position_weight ⊙ τ_pd + w_bal·τ_bal` 并按**模型** `ctrlrange` 截断。

本模块只做**纯映射**，不碰模型、不碰锁、不写任何数字：
  · `foot_forces`：MPC 解里的 12 维接触力（按 `contact.LEG_ORDER` × (x,y,z)）；
  · `jacobian_transpose`：由调用方从**既有**平衡/步态路径取（本模块不重造雅可比）；
  · 输出关节力矩按 `LEG_ORDER` × (hip, thigh, calf) 排列，与 `profile` 的关节顺序由调用方对齐。

未验证边界：本模块尚未接进 `locomote()`（A6a-④ 后续块）；`position_weight` 的取值必须来自声明，
本模块只做形状/取值域校验，绝不提供默认值。
"""

from __future__ import annotations

import numpy as np

from iraf_adapters.unitree.mpc.contact import LEG_ORDER

__all__ = ["JOINT_TORQUE_LAYOUT", "foot_forces_matrix", "joint_torques", "torque_provider_payload"]

#: 输出关节力矩的分量名（与 `LEG_ORDER` 组合使用，供调用方对齐 Profile 关节序）。
JOINT_TORQUE_LAYOUT = ("hip", "thigh", "calf")


def foot_forces_matrix(foot_forces):
    """把 MPC 解里的 `{腿码: (3,)}` 或 `(4, 3)`/`(12,)` 归一为 `(4, 3)`（行序 `LEG_ORDER`）。"""
    if isinstance(foot_forces, dict):
        missing = [code for code in LEG_ORDER if code not in foot_forces]
        extra = [code for code in foot_forces if code not in LEG_ORDER]
        if missing or extra:
            raise ValueError("foot_forces 必须恰好覆盖 LEG_ORDER=%s（缺 %s／多 %s）"
                             % (list(LEG_ORDER), missing, extra))
        rows = []
        for code in LEG_ORDER:
            value = np.asarray(foot_forces[code], dtype=float).reshape(-1)
            if value.size != 3:
                raise ValueError("foot_forces[%r] 必须是 3 维，实际 %d" % (code, value.size))
            rows.append(value)
        out = np.vstack(rows)
    else:
        value = np.asarray(foot_forces, dtype=float)
        if value.shape == (12,):
            value = value.reshape(4, 3)
        if value.shape != (4, 3):
            raise ValueError("foot_forces 形状必须为 (4,3)/(12,)/字典，实际 %r" % (value.shape,))
        out = value.copy()
    if not np.all(np.isfinite(out)):
        raise ValueError("foot_forces 含非有限值")
    return out


def joint_torques(foot_forces, jacobian_transpose):
    """`τ_腿 = Jᵀ · f_腿`，返回 `(12,)`（行序 `LEG_ORDER` × (hip, thigh, calf)）。

    `jacobian_transpose`：`{腿码: (3, 3)}`（该腿足端力 → 该腿 3 个关节力矩）或 `(4, 3, 3)`。
    每个腿码都必须给出、形状必须为 3×3（缺项即显式失败，不用零矩阵兜底）。

    ⚠ **符号语义（2026-09-23 实测判定，勿凭直觉）**：本函数是"力 → 力矩"的**纯映射**：
    输入 `foot_forces` 是**地面作用在足端的力**（世界系，+z 向上支撑）时，本函数给出的
    是 `+Jᵀf`；而**执行器需要输出的支撑力矩是它的相反数**（`−Jᵀf`）。
    依据（`build/iraf-a6a4/jt_sign_probe.py`，同一关键帧 + 每腿 mg/4 世界系 +z，推进 0.02 s）：
      · `τ = −Jᵀf` ⇒ 机身高度 Δh = **+0.001714 m**、四腿法向合力 **116.025 N**（撑住）；
      · `τ = +Jᵀf` ⇒ Δh = −0.000563 m、法向合力 **0.0 N**（把足端卸掉、机身下沉）。
    这也与已验证的平衡路径一致：`balance.leg_joint_torques` 实现的是 `-jacobian.T.dot(force)`
    （`src/iraf_adapters/unitree/balance.py:734`）。⇒ 本模块的**载荷构造函数**取负号（见下）。
    """
    forces = foot_forces_matrix(foot_forces)
    if isinstance(jacobian_transpose, dict):
        missing = [code for code in LEG_ORDER if code not in jacobian_transpose]
        extra = [code for code in jacobian_transpose if code not in LEG_ORDER]
        if missing or extra:
            raise ValueError("jacobian_transpose 必须恰好覆盖 LEG_ORDER（缺 %s／多 %s）"
                             % (missing, extra))
        mats = [np.asarray(jacobian_transpose[code], dtype=float) for code in LEG_ORDER]
    else:
        mats = list(np.asarray(jacobian_transpose, dtype=float))
    if len(mats) != 4:
        raise ValueError("jacobian_transpose 必须给出 4 条腿的矩阵，实际 %d" % len(mats))
    out = np.zeros(12, dtype=float)
    for leg, code in enumerate(LEG_ORDER):
        matrix = mats[leg]
        if matrix.shape != (3, 3):
            raise ValueError("jacobian_transpose[%s] 形状必须为 (3,3)，实际 %r" % (code, matrix.shape))
        if not np.all(np.isfinite(matrix)):
            raise ValueError("jacobian_transpose[%s] 含非有限值" % code)
        out[3 * leg:3 * leg + 3] = matrix @ forces[leg]
    return out


def torque_provider_payload(foot_forces, jacobian_transpose, position_weight):
    """构造 `torque_provider` 的 B1 返回值（契约 §1.2）：`{balance_torque_nm, position_weight}`。

    `position_weight` 由调用方从**声明**给出（长度 12，取值 [0, 1]；本模块不给默认值）。

    `balance_torque_nm` = **执行器支撑力矩** = `−Jᵀ·f`（f 为地面作用在足端的力，世界系 +z 向上）；
    符号依据见 `joint_torques` 的实测说明（探针 `build/iraf-a6a4/jt_sign_probe.py`）。
    写成 `−joint_torques(...)` 而不是改 `joint_torques` 本身：纯映射保持"力→力矩"的单一语义，
    执行器符号只在**构造载荷**这一处显式翻转（便于单测分别钉住两侧）。
    """
    if position_weight is None:
        raise ValueError("position_weight 必须显式给出（取值只能来自声明，本模块不给默认值）")
    weight = np.asarray(position_weight, dtype=float).reshape(-1)
    if weight.size != 12:
        raise ValueError("position_weight 长度必须为 12，实际 %d" % weight.size)
    if not np.all(np.isfinite(weight)):
        raise ValueError("position_weight 含非有限值")
    if np.any(weight < 0.0) or np.any(weight > 1.0):
        raise ValueError("position_weight 必须落在 [0, 1]，实际 min=%r max=%r"
                         % (float(weight.min()), float(weight.max())))
    torques = -joint_torques(foot_forces, jacobian_transpose)
    return {"balance_torque_nm": torques, "position_weight": weight.copy()}
