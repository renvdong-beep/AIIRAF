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
