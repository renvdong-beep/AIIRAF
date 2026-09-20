r"""机器人无关的数值逆运动学契约。

设计依据（AGENTS.md 铁律 1「契约先于实现」与铁律 3「硬件差异只进 adapters/ 与 profiles/」）：

- 本模块只依赖 MuJoCo 的通用运动学量（mj_forward / mj_jacBody / mj_jacGeom）
  与 numpy，**不含任何机器人专有名称**（不出现 joint1..joint6、link6、link7 等字面量，
  也不通过 f"joint{i}" 之类的生成式隐式耦合）。
- 关节名、末端 body 名、末端 geom 名全部由调用方以参数传入；
  机器人差异声明在 profiles/*.yaml，由脚本读出后注入。
- 库内部不做静默兜底：无解、退化、未知名称一律显式抛错（铁律 5）。

算法与 Piper 既有实现（scripts/solve_piper_ik.py、scripts/build_piper_baseline.py
的 solve_finger_center_pose）逐位一致，仅把名称与几何来源参数化：

- 阻尼最小二乘：delta = pinv(J[:, dof_adr]) @ error，可选 step 系数；
- 关节限位由 model.jnt_limited / model.jnt_range 提供，逐关节裁剪；
- 每轮迭代后调用 mj_normalizeQuat，保证含自由关节的模型不发散。
"""

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import mujoco
import numpy as np


class KinematicsError(ValueError):
    """IK 契约层的显式失败。"""


@dataclass(frozen=True)
class IkResult:
    """一次 IK 求解的可审计结果。"""

    joint_positions: dict
    solved_position_m: list
    target_position_m: list
    position_error_m: float
    iterations: int
    target_kind: str
    extra: dict = field(default_factory=dict)


def _require_id(model, objtype, name, label):
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise KinematicsError(f"MJCF 缺少 {label}: {name}")
    return int(obj_id)


def joint_ids(model, names: Sequence[str]) -> list:
    """把关节名解析为关节 id，缺失即显式失败。"""
    return [_require_id(model, mujoco.mjtObj.mjOBJ_JOINT, name, "关节") for name in names]


def geom_id(model, name: str) -> int:
    return _require_id(model, mujoco.mjtObj.mjOBJ_GEOM, name, "geom")


def body_id(model, name: str) -> int:
    return _require_id(model, mujoco.mjtObj.mjOBJ_BODY, name, "body")


def _clip_to_limits(model, joint_id, value):
    """按模型自带的关节限位裁剪；无限位关节原样返回。"""
    if model.jnt_limited[joint_id]:
        low, high = model.jnt_range[joint_id]
        return float(np.clip(value, low, high))
    return float(value)


def _jacobian_for_points(model, data, points):
    """把若干几何点的平移雅可比取平均，得到\"虚拟刚体位姿\"的雅可比。

    双指中点目标（多指夹爪）用该函数即可复用同一套求解器；
    单点目标传入长度为 1 的序列，结果与 mj_jacBody/mj_jacGeom 完全一致。

    数学依据：几何点位置 p_i(q) 的雅可比为 ∂p_i/∂q；
    中点 m = (Σ p_i)/n 故 ∂m/∂q = (Σ ∂p_i/∂q)/n，即雅可比的算术平均。
    """
    if not points:
        raise KinematicsError("至少需要一个目标点")
    total = None
    for point in points:
        jac = np.zeros((3, model.nv))
        rotation = np.zeros((3, model.nv))
        if point["kind"] == "geom":
            mujoco.mj_jacGeom(model, data, jac, rotation, point["id"])
        elif point["kind"] == "body":
            mujoco.mj_jacBody(model, data, jac, rotation, point["id"])
        else:
            raise KinematicsError("未知的雅可比目标类型: " + str(point["kind"]))
        total = jac if total is None else total + jac
    return total / float(len(points))


def solve_position_ik(
    model,
    data,
    target,
    arm_joints,
    points,
    iterations=800,
    step=1.0,
    tolerance_m=1e-5,
):
    """位置型阻尼最小二乘 IK。

    参数
    ----
    model, data    : 已加载 MJCF 与 mjData；data.qpos 会作为初值并被就地更新。
    arm_joints     : 参与求解的关节 id 序列（顺序即 qpos 分量的顺序）。
    points         : [{"kind": "geom"|"body", "id": int}, ...]；
                     求解目标是这些点在**世界系**位置的算术平均。
    iterations     : 迭代上限；耗尽仍未收敛时返回实际残差（不伪造收敛）。
    step           : 更新步长系数，1.0 与既有 Piper 求解器一致。
    tolerance_m    : 残差阈值，达到即提前退出。

    返回 IkResult；残差是否可接受由调用方门禁判定，本函数不做静默兜底。
    """
    target = np.asarray(target, dtype=float)
    if target.shape != (3,):
        raise KinematicsError("IK 目标必须是 3 个数值")
    arm_joints = list(arm_joints)
    if not arm_joints:
        raise KinematicsError("IK 至少需要一个臂关节")
    qpos_adr = [int(model.jnt_qposadr[joint]) for joint in arm_joints]
    dof_adr = [int(model.jnt_dofadr[joint]) for joint in arm_joints]

    distance = float("inf")
    used = 0
    for index in range(max(1, int(iterations))):
        mujoco.mj_forward(model, data)
        current = np.mean(
            [
                data.geom_xpos[point["id"]]
                if point["kind"] == "geom"
                else data.xpos[point["id"]]
                for point in points
            ],
            axis=0,
        )
        error = target - current
        distance = float(np.linalg.norm(error))
        used = index + 1
        if distance <= float(tolerance_m):
            break
        jacobian = _jacobian_for_points(model, data, points)
        delta = np.linalg.pinv(jacobian[:, dof_adr]) @ error * float(step)
        for joint, adr, value in zip(arm_joints, qpos_adr, delta):
            data.qpos[adr] = _clip_to_limits(
                model, joint, float(data.qpos[adr]) + float(value)
            )
        mujoco.mj_normalizeQuat(model, data.qpos)

    mujoco.mj_forward(model, data)
    solved = np.mean(
        [
            data.geom_xpos[point["id"]]
            if point["kind"] == "geom"
            else data.xpos[point["id"]]
            for point in points
        ],
        axis=0,
    )
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) for joint in arm_joints
    ]
    if any(name is None for name in names):
        raise KinematicsError("臂关节名称解析失败, 无法回写关节解")
    return IkResult(
        joint_positions={
            str(name): float(data.qpos[adr]) for name, adr in zip(names, qpos_adr)
        },
        solved_position_m=[round(float(value), 9) for value in solved],
        target_position_m=[round(float(value), 9) for value in target],
        position_error_m=float(np.linalg.norm(target - solved)),
        iterations=used,
        target_kind="mean_of_points",
    )


def solve_body_position_ik(
    model, data, target, arm_joints, body_name, **solver
):
    """以单个 body 原点为目标的便捷封装（等价于既有 solve_piper_ik 路径）。"""
    points = [{"kind": "body", "id": body_id(model, body_name)}]
    result = solve_position_ik(model, data, target, arm_joints, points, **solver)
    return result


def solve_geom_position_ik(
    model, data, target, arm_joints, geom_name, **solver
):
    """以单个 geom 中心为目标的便捷封装。"""
    points = [{"kind": "geom", "id": geom_id(model, geom_name)}]
    result = solve_position_ik(model, data, target, arm_joints, points, **solver)
    return result


def solve_point_midpoint_ik(
    model, data, target, arm_joints, geom_names: Iterable[str], **solver
):
    """以多个 geom 中心的算术平均为目标的封装（双指指尖中点即为 2 个 geom）。"""
    names = list(geom_names)
    if not names:
        raise KinematicsError("geom_names 不能为空")
    points = [{"kind": "geom", "id": geom_id(model, name)} for name in names]
    result = solve_position_ik(model, data, target, arm_joints, points, **solver)
    return result


def lowest_mesh_point_z(model, data, geom_ids):
    """返回若干 geom 网格顶点在世界系下的最低 z。

    用于「指尖不得扎进工作台」这类几何门禁；
    网格为空时退化为 geom 中心减最大半尺寸，不做静默跳过。
    """
    lowest = float("inf")
    for geom in geom_ids:
        if int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_BOX):
            # box 用旋转后的半尺寸投影，避免"中心减最大半尺寸"高估边界；
            # UR5e 的 pad 与 Piper 的方块都会走这一支。
            half = np.asarray(model.geom_size[geom], dtype=float)
            rotation = np.asarray(data.geom_xmat[geom], dtype=float).reshape(3, 3)
            extent_z = float(np.abs(rotation[2, :]) @ half)
            lowest = min(lowest, float(data.geom_xpos[geom][2]) - extent_z)
            continue
        mesh = int(model.geom_dataid[geom])
        count = int(model.mesh_vertnum[mesh]) if mesh >= 0 else 0
        if count > 0:
            start = int(model.mesh_vertadr[mesh])
            verts = np.asarray(model.mesh_vert[start : start + count], dtype=float)
            rotation = np.asarray(data.geom_xmat[geom], dtype=float).reshape(3, 3)
            world = np.asarray(data.geom_xpos[geom], dtype=float) + verts @ rotation.T
            lowest = min(lowest, float(world[:, 2].min()))
        else:
            center = np.asarray(data.geom_xpos[geom], dtype=float)
            half = float(np.abs(np.asarray(model.geom_size[geom], dtype=float)).max())
            lowest = min(lowest, float(center[2]) - half)
    return lowest

# ---------------------------------------------------------------------------
# 位姿型（6 维）IK 与伺服前馈
#
# 与上面的位置型 IK 同一设计原则：只依赖 MuJoCo 通用量，关节名/末端 geom/
# 法兰 site 全部由调用方注入，不含任何机器人专有名称。
#
# 为什么需要（实测）：6 自由度通用臂上，同一个"夹持区中点位置"对应无穷多组关节角。
# 只约束位置时求解器会落到"夹爪横着伸出去"的病态解（实测开合轴
# [0.749,0.143,-0.647]、两指高度差 60.5mm），下降阶段把目标顶飞。
# 因此必须同时约束工具指向与开合轴。
# ---------------------------------------------------------------------------


def tool_pose_from_axes(pointing_axis, spread_axis):
    """由"工具指向 + 开合轴"构造法兰的目标旋转矩阵。

    约定（与夹持区定义一致，见 `lowest_mesh_point_z` 的调用方）：
    - 法兰 **+z** = 工具指向（法兰 → 夹持区中点）；
    - 法兰 **+x** = 开合轴（左指 → 右指）。

    两轴必须近似正交：不正交意味着调用方给的姿态期望自相矛盾，
    此时静默正交化会得到一个"看起来合理但谁也不满足"的目标，
    因此这里显式失败。
    """
    pointing = np.asarray(pointing_axis, dtype=float).reshape(-1)
    spread = np.asarray(spread_axis, dtype=float).reshape(-1)
    if pointing.shape != (3,) or spread.shape != (3,):
        raise KinematicsError("工具指向与开合轴都必须是 3 个数值")
    for label, vector in (("工具指向", pointing), ("开合轴", spread)):
        norm = float(np.linalg.norm(vector))
        if norm < 1e-12:
            raise KinematicsError(label + " 不能为零向量")
    pointing = pointing / float(np.linalg.norm(pointing))
    spread = spread / float(np.linalg.norm(spread))
    dot = float(np.dot(pointing, spread))
    if abs(dot) > 1e-6:
        raise KinematicsError(
            "工具指向与开合轴必须正交: dot=%.9f" % dot
        )
    y_axis = np.cross(pointing, spread)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < 1e-12:
        raise KinematicsError("工具指向与开合轴共线，无法构造正交基")
    y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, pointing)
    return np.column_stack([x_axis, y_axis, pointing])


def _rotation_vector(current_rotation, target_rotation):
    """两个旋转矩阵之间的旋转向量（轴 × 角）。"""
    relative = np.asarray(target_rotation, dtype=float) @ np.asarray(
        current_rotation, dtype=float
    ).T
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, relative.reshape(-1))
    vector = np.zeros(3)
    # 注意 mju_quat2Vel 返回"轴 × 半角"，乘 2 才是完整旋转角。
    mujoco.mju_quat2Vel(vector, quat, 2.0)
    return vector


def solve_pose_ik(
    model,
    data,
    target_position,
    target_rotation,
    arm_joints,
    points,
    flange_site,
    iterations=600,
    tolerance_m=1e-5,
    orientation_tolerance=1e-4,
    damping=1e-4,
    step_limit=0.2,
):
    """位姿型（位置 + 姿态）阻尼最小二乘 IK。

    位置误差取 `points` 的世界系算术平均（与 `solve_position_ik` 完全同一口径，
    因此"夹具几何"的定义只需在调用方声明一次），
    姿态误差取法兰 site 的旋转向量，两者拼成 6 维误差后做阻尼最小二乘。

    行为与既有实现逐位一致（迁移自 UR5e 基线求解器）：
    - 迭代上限耗尽时**返回当前迭代状态**而非历史最优——残差是否可接受
      由调用方门禁判定，本函数不伪造收敛；
    - `extra["best_position_error_m"]` 保留历史最优残差与对应迭代数，
      供调用方在"未收敛"时判断是路径问题还是迭代不足。

    返回 IkResult；姿态残差放在 `extra` 中（rotation_error_rad / 各轴偏差角）。
    """
    target_position = np.asarray(target_position, dtype=float).reshape(-1)
    if target_position.shape != (3,):
        raise KinematicsError("IK 目标位置必须是 3 个数值")
    target_rotation = np.asarray(target_rotation, dtype=float)
    if target_rotation.shape != (3, 3):
        raise KinematicsError("IK 目标旋转必须是 3x3 矩阵")
    arm_joints = list(arm_joints)
    if not arm_joints:
        raise KinematicsError("IK 至少需要一个臂关节")
    if not points:
        raise KinematicsError("IK 至少需要一个位置目标点")
    qpos_adr = [int(model.jnt_qposadr[joint]) for joint in arm_joints]
    dof_adr = [int(model.jnt_dofadr[joint]) for joint in arm_joints]
    flange_site = int(flange_site)

    best = None
    position_error = float("inf")
    rotation_error = float("inf")
    used = 0
    for index in range(max(1, int(iterations))):
        mujoco.mj_forward(model, data)
        current = np.mean(
            [
                np.asarray(
                    data.geom_xpos[point["id"]]
                    if point["kind"] == "geom"
                    else data.xpos[point["id"]],
                    dtype=float,
                )
                for point in points
            ],
            axis=0,
        )
        current_rotation = np.asarray(
            data.site_xmat[flange_site], dtype=float
        ).reshape(3, 3)
        e_pos = target_position - current
        e_rot = _rotation_vector(current_rotation, target_rotation)
        position_error = float(np.linalg.norm(e_pos))
        rotation_error = float(np.linalg.norm(e_rot))
        used = index + 1
        if best is None or position_error < best[0]:
            best = (position_error, index + 1)

        if position_error <= float(tolerance_m) and rotation_error <= float(
            orientation_tolerance
        ):
            break

        jac_position = _jacobian_for_points(model, data, points)[:, dof_adr]
        jac_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, None, jac_rotation, flange_site)
        jac_rotation = jac_rotation[:, dof_adr]
        jacobian = np.vstack([jac_position, jac_rotation])
        error = np.concatenate([e_pos, e_rot])
        lam = float(damping)
        if lam <= 0:
            raise KinematicsError("阻尼系数必须为正数")
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + lam * np.eye(6), error
        )
        limit = float(step_limit)
        if limit <= 0:
            raise KinematicsError("步长上限必须为正数")
        delta = np.clip(delta, -limit, limit)
        for joint, adr, value in zip(arm_joints, qpos_adr, delta):
            data.qpos[adr] = _clip_to_limits(
                model, joint, float(data.qpos[adr]) + float(value)
            )
        mujoco.mj_normalizeQuat(model, data.qpos)

    mujoco.mj_forward(model, data)
    solved = np.mean(
        [
            np.asarray(
                data.geom_xpos[point["id"]]
                if point["kind"] == "geom"
                else data.xpos[point["id"]],
                dtype=float,
            )
            for point in points
        ],
        axis=0,
    )
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        for joint in arm_joints
    ]
    if any(name is None for name in names):
        raise KinematicsError("臂关节名称解析失败, 无法回写关节解")
    site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, flange_site)
    return IkResult(
        joint_positions={
            str(name): float(data.qpos[adr]) for name, adr in zip(names, qpos_adr)
        },
        solved_position_m=[round(float(value), 9) for value in solved],
        target_position_m=[round(float(value), 9) for value in target_position],
        position_error_m=float(np.linalg.norm(target_position - solved)),
        iterations=used,
        target_kind="mean_of_points_with_orientation",
        extra={
            "flange_site": site_name,
            "rotation_error_rad": float(rotation_error),
            "best_position_error_m": float(best[0]) if best else float("inf"),
            "best_iteration": int(best[1]) if best else 0,
            "target_rotation": [
                [round(float(value), 9) for value in row] for row in target_rotation
            ],
        },
    )


def balance_tip_clearance(
    model,
    data,
    solve,
    target_position,
    offset_direction,
    tip_geoms,
    floor_z,
    clearance_m,
    iterations=8,
    tolerance_m=1e-4,
):
    """沿指定方向抬高目标，直到指尖最低点离开台面达到要求的间隙。

    为什么需要：夹具几何决定了"抓取点"与"最低点"往往不是同一个位置
    （2F-85 的 pad 是上下两个 box，最低面比夹持区中心低约 19mm），
    照搬别机型的常量会让指尖扎进台面。

    `solve(target) -> IkResult` 由调用方注入（通常是 `solve_pose_ik` 的偏函数），
    因此本函数与具体求解器、具体机型都无关。

    返回 (最后一次 IkResult, 高度修正量, trace, cleared)。
    未收敛时 `cleared=False` 且**不抛错**：是否接受由调用方的门禁决定，
    避免把策略判断下沉进核心库。
    """
    offset = np.asarray(offset_direction, dtype=float).reshape(-1)
    if offset.shape != (3,):
        raise KinematicsError("配平方向必须是 3 个数值")
    norm = float(np.linalg.norm(offset))
    if norm < 1e-12:
        raise KinematicsError("配平方向不能为零向量")
    offset = offset / norm
    base = np.asarray(target_position, dtype=float).reshape(-1)
    if base.shape != (3,):
        raise KinematicsError("配平目标位置必须是 3 个数值")

    correction = 0.0
    trace = []
    result = None
    cleared = False
    for index in range(max(1, int(iterations))):
        result = solve(base + offset * correction)
        tip_z = lowest_mesh_point_z(model, data, tip_geoms)
        deficit = (float(floor_z) + float(clearance_m)) - float(tip_z)
        trace.append(
            {
                "iteration": index,
                "correction_m": round(float(correction), 9),
                "tip_z_m": round(float(tip_z), 9),
                "deficit_m": round(float(deficit), 9),
                "position_error_m": round(float(result.position_error_m), 9),
                "rotation_error_rad": float(
                    (result.extra or {}).get("rotation_error_rad", float("nan"))
                ),
            }
        )
        if deficit <= float(tolerance_m):
            cleared = True
            break
        correction += float(deficit)
    return result, correction, trace, cleared


def gravity_hold_ctrl(
    model,
    arm_joints,
    hold_positions,
    hold_ms=4000,
    tolerance_rad=1e-3,
    use_keyframe=True,
):
    """求每个臂关节"抵消重力所需的 ctrl 增量"（伺服前馈）。

    纯 PD 执行器（Menagerie 的 UR5e 官方模型即如此，没有真机控制器自带的重力
    补偿）在重力矩不为零的位形下必然停在 `ctrl - τ_g / gain`：
    实测 shoulder_lift 稳态误差约 0.015 rad ≈ 末端 15mm，超过抓取容差。
    真机的位置伺服内部已做该补偿，因此这属于**执行器建模差异**而非真机特性。

    重力矩取 `qfrc_bias`（qvel=0），**不要用 `mj_inverse`**：
    MuJoCo 的逆动力学按执行器力限截断结果（本模型实测稳定返回
    shoulder_lift -150.000 N·m / wrist +28.000 N·m，恰好等于官方力矩限值，
    且符号相反），而该位形真实需求约 30 N·m。

    自证方式：把增量加到目标 ctrl 上做一次静态保持仿真，要求稳态关节误差
    ≤ tolerance_rad；判据在物理上验证，不依赖对增益取值的假设。

    返回 ({关节名: ctrl 增量}, 证据字典)。自建 MjData，不修改调用方状态。
    """
    arm_joints = list(arm_joints)
    if not arm_joints:
        raise KinematicsError("重力前馈至少需要一个臂关节")
    data = mujoco.MjData(model)
    if use_keyframe and int(getattr(model, "nkey", 0) or 0) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)

    joint_ids = []
    actuator_ids = []
    gains = []
    for name in arm_joints:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if joint_id < 0:
            raise KinematicsError("缺少臂关节: " + str(name))
        actuator_id = -1
        for index in range(int(model.nu)):
            if int(model.actuator_trnid[index, 0]) == int(joint_id):
                actuator_id = index
                break
        if actuator_id < 0:
            raise KinematicsError("臂关节没有对应的执行器: " + str(name))
        gain = float(model.actuator_gainprm[actuator_id][0])
        if gain <= 0:
            raise KinematicsError(
                "执行器 %s 的 gainprm[0]=%.6f 非正，无法折算前馈量" % (name, gain)
            )
        joint_ids.append(int(joint_id))
        actuator_ids.append(int(actuator_id))
        gains.append(gain)

    # 夹爪等其它自由度：若调用方给出了关节名键，按 keyframe 或声明值固定，
    # 避免把它们的重力算进臂关节补偿。
    for name, value in (hold_positions or {}).items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)

    qpos_adr = [int(model.jnt_qposadr[joint]) for joint in joint_ids]
    for index, name in enumerate(arm_joints):
        if name not in hold_positions:
            raise KinematicsError("缺少臂关节目标位形: " + str(name))
        data.qpos[qpos_adr[index]] = float(hold_positions[name])
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.qfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)

    bias = np.asarray(data.qfrc_bias, dtype=float)
    dof_adr = [int(model.jnt_dofadr[joint]) for joint in joint_ids]
    compensation = {
        str(name): float(bias[dof_adr[index]] / gains[index])
        for index, name in enumerate(arm_joints)
    }
    gravity_torque = {
        str(name): round(float(bias[dof_adr[index]]), 6)
        for index, name in enumerate(arm_joints)
    }

    for index, name in enumerate(arm_joints):
        data.ctrl[actuator_ids[index]] = float(hold_positions[name]) + float(
            compensation[str(name)]
        )
    steps = max(1, int(round(float(hold_ms) / 1000.0 / float(model.opt.timestep))))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    residual = {
        str(name): round(
            float(data.qpos[qpos_adr[index]]) - float(hold_positions[name]), 9
        )
        for index, name in enumerate(arm_joints)
    }
    worst = max(abs(value) for value in residual.values())
    if worst > float(tolerance_rad):
        raise KinematicsError(
            "重力前馈验证未通过: 静态保持 %dms 后最大关节误差 %.9f rad（限 %.9f rad）"
            "，残余=%s" % (hold_ms, worst, tolerance_rad, residual)
        )
    return (
        {name: round(float(value), 9) for name, value in compensation.items()},
        {
            "method": "qfrc_bias_plus_static_hold",
            "gravity_torque_nm": gravity_torque,
            "actuator_gain": {
                str(name): round(gains[index], 3)
                for index, name in enumerate(arm_joints)
            },
            "hold_ms": int(hold_ms),
            "residual_joint_rad": residual,
            "worst_residual_rad": round(float(worst), 9),
            "tolerance_rad": float(tolerance_rad),
        },
    )
