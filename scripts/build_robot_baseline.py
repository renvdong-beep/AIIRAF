"""按**基线配置声明**求解参考关节姿态、生成受控场景并校验（与机型无关）。

本模块由 `scripts/build_baseline.py` 按基线的 `build.baseline_module` 分派；
接入新机型**不需要新增本模块的代码**：关节名、指腹 geom、夹持区、工具指向、
开合轴、后退方向、限位、种子位形全部来自声明。原文件名
`build_ur5_baseline.py` 保留为薄包装（兼容既有命令与文档）。

历史上第一个使用者是 UR5e，因此部分注释保留了该机型的实测结论
（例如"工具指向写成后退方向会导致不可达位姿"），结论本身与机型无关。

与 `build_piper_baseline.py` 的关系：

- **IK 算法复用同一份契约** `iraf_core.kinematics.solve_position_ik`，
  只是把"双指末端"从 Piper 的指腹 mesh 换成 2F-85 的 pad box；
- 校验逻辑（指尖离台间隙、张开时不碰任何物体）与 Piper 侧口径一致，
  保证"对等验收"。

**为什么必须做姿态约束（本脚本与 Piper 侧最大的差异）：**

Piper 的 IK 在固定菱形位形下 joint4/joint6 接近零，夹爪自然竖直，
"只约束 pad 中点位置"刚好落在正确分支上。

UR5e 是 6 自由度通用臂，同一位置有**无穷多组关节角**。
实测（probe_roll_alignment.py）只约束位置时，求解器落到：
    开合轴 = [0.749, 0.143, -0.647]   （z 分量 -0.65，斜插向下）
    两 pad 高度差 = -60.5mm
    夹爪指向与 -Z 夹角 = 71.3°
即夹爪几乎横着伸出去。DESCEND 时下侧 pad 先撞到方块侧面，
把方块顶到 z=1.46m（probe_descent.py 实测），对齐门禁必然失败。

因此求解分两步：
1. **位置收敛**：走契约层 `solve_position_ik`（与 Piper 同一算法）；
2. **姿态修正**：在"位置误差为零"的**零空间**里，用阻尼最小二乘把
   "夹爪指向"和"开合轴"拉到期望方向，然后做一轮短位置复苏。

零空间修正不改变末端位置（一阶近似），因此不会破坏第 1 步的结果；
同时完全不侵入契约层实现（Piper 侧 4 条验收链不受影响）。
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

from build_robot_pick_scene import build_scene
from iraf_core.kinematics import (
    balance_tip_clearance,
    solve_position_ik,
    gravity_hold_ctrl,
    lowest_mesh_point_z,
    solve_pose_ik,
)

DEFAULT_BASELINE = "config/ur5_simulation_baseline.yaml"  # 兼容旧命令；新调用方应显式传 --baseline


def load_baseline(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("基线配置格式无效: " + str(path))
    return data


def _resolve(root, value):
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = (root / candidate).resolve()
    return candidate


def _sha256(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _geom_id(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("MJCF 缺少 geom: " + name)
    return int(geom_id)


def _body_id(model, name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError("MJCF 缺少 body: " + name)
    return int(body_id)


def _require_flange_site(model_cfg):
    """法兰 site 必须显式声明，**不提供机型默认值**。

    历史实现缺省指向 UR5 官方的 `attachment_site`：换机型后要么静默用了
    同名的其它 site，要么报一个与配置无关的"缺少 site"。按铁律 6.2
    （禁止隐式默认值）改为必填。
    """
    declared = (model_cfg.get("bodies") or {}).get("flange_site")
    if not declared:
        raise ValueError(
            "基线缺少 model.bodies.flange_site（法兰 site 名用于姿态求解与工具轴判定）"
        )
    return str(declared)


def _site_id(model, name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise ValueError("MJCF 缺少 site: " + name)
    return int(site_id)


def _joint_adr(model, name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError("MJCF 缺少关节: " + name)
    return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])





def _contact_pairs(model, data):
    pairs = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        pairs.append(
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or "?",
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or "?",
                round(float(contact.dist), 6),
            )
        )
    return pairs


class GraspPoseSolver:
    """以"夹持区中点 + 工具指向 + 开合轴"为目标的位姿型 IK 适配层。

    算法本体在 `iraf_core.kinematics.solve_pose_ik`（与位置型 IK 同源、与机型无关）；
    这里只做三件**UR5e 相关的接线**：
      1. 注入声明：臂关节 id、夹持区 geom 列表、法兰 site；
      2. 构造法兰目标旋转：z 轴 = 工具指向（**后退方向的反向**）、x 轴 = 开合轴；
      3. 求解后按与 IK 同一口径量出夹持区中点与两轴，供门禁与证据使用。

    历史坑：把 `approach_direction`（后退方向，竖直向上）直接当作法兰 z 轴，
    会解出"夹爪朝上"的另一支解 —— 夹持区被抬到法兰上方 134mm，抓取点落到
    台面以下的不可达位姿。详见 docs/debug/2026-09-20-ur5-tool-axis-flip.md。
    """

    def __init__(self, model, data, arm_joint_ids, pad_geoms, flange_site,
                 pointing_direction_world, spread_axis_world):
        self.model = model
        self.data = data
        self.arm = [int(joint) for joint in arm_joint_ids]
        if not self.arm:
            raise ValueError("IK 至少需要一个臂关节")
        self.points = [{"kind": "geom", "id": int(geom)} for geom in pad_geoms]
        if len(self.points) < 2:
            raise ValueError(
                "夹持区至少需要 2 个 geom（每侧一个代表面），实际 %d 个" % len(self.points)
            )
        self.flange_site = int(flange_site)
        self.pointing = _unit(pointing_direction_world, "工具指向")
        self.spread_axis = _unit(spread_axis_world, "开合轴")
        # 目标旋转由**模型实测的局部轴**推出，而不是假设"法兰 x 轴 = 开合轴、
        # z 轴 = 工具指向"。做法：在种子位形下量出
        #   v_point_local  = 法兰 → 夹持区中点 在法兰局部系中的方向
        #   v_spread_local = 左指 → 右指     在法兰局部系中的方向
        # 再把这组局部基映射到期望的世界方向：R = W · Lᵀ。
        # 为什么必须这么做：不同夹爪的安装朝向不同 —— 实测第三台机型（demo3）
        # 的指沿法兰 **y** 分离，写死"x 轴 = 开合轴"会让姿态约束永远无法满足
        # （实测偏差 86.5°，且求解器会收敛到一个姿态完全错误的解）。
        mujoco.mj_forward(self.model, self.data)
        site_rot = np.asarray(
            self.data.site_xmat[self.flange_site], dtype=float
        ).reshape(3, 3)
        _, measured_pointing, measured_spread = self.measure()
        v_point_local = _unit(site_rot.T @ measured_pointing, "工具指向（局部）")
        v_spread_local = _unit(site_rot.T @ measured_spread, "开合轴（局部）")
        if abs(float(np.dot(v_point_local, v_spread_local))) > 1e-3:
            raise ValueError(
                "模型实测的工具指向与开合轴在法兰局部系中不垂直: dot=%.6f"
                % float(np.dot(v_point_local, v_spread_local))
            )
        local_basis = np.column_stack([
            v_spread_local,
            np.cross(v_point_local, v_spread_local),
            v_point_local,
        ])
        world_basis = np.column_stack([
            self.spread_axis,
            np.cross(self.pointing, self.spread_axis),
            self.pointing,
        ])
        self.target_rot = world_basis @ local_basis.T
        # 正交性校验（构造正确性自证）：不正交说明期望方向自相矛盾。
        if not np.allclose(self.target_rot @ self.target_rot.T, np.eye(3), atol=1e-6):
            raise ValueError("由实测局部轴构造的目标旋转不正交，请检查开合轴声明")

    def solve(self, target, solver_cfg, orientation_iterations=None):
        """求解并返回 (IkResult, 姿态残差字典)。"""
        iterations = int(
            orientation_iterations
            if orientation_iterations is not None
            else solver_cfg.get("pose_iterations", 600)
        )
        result = solve_pose_ik(
            self.model,
            self.data,
            target,
            self.target_rot,
            self.arm,
            self.points,
            self.flange_site,
            iterations=iterations,
            tolerance_m=float(solver_cfg.get("tolerance_m", 1e-5)),
        )
        _, direction, axis = self.measure()
        return result, {
            "gripper_direction_deg": float(np.degrees(np.arccos(
                np.clip(float(np.dot(direction, self.pointing)), -1.0, 1.0)
            ))),
            "spread_axis_deg": float(np.degrees(np.arccos(
                np.clip(abs(float(np.dot(axis, self.spread_axis))), 0.0, 1.0)
            ))),
        }

    def measure(self):
        """量出夹持区中点、工具指向、开合轴（与求解口径一致）。

        开合轴取"前一半 geom 均值 → 后一半 geom 均值"的方向：
        4 个 pad box 时即"左两点均值 → 右两点均值"，与雅可比构造同源。
        """
        mujoco.mj_forward(self.model, self.data)
        positions = [
            np.asarray(self.data.geom_xpos[point["id"]], dtype=float)
            for point in self.points
        ]
        mid = sum(positions) / float(len(positions))
        half = len(positions) // 2
        left = sum(positions[:half]) / float(half)
        right = sum(positions[half:]) / float(len(positions) - half)
        flange = np.asarray(self.data.site_xpos[self.flange_site], dtype=float)
        direction = mid - flange
        norm = float(np.linalg.norm(direction))
        direction = direction / norm if norm > 1e-12 else self.pointing.copy()
        axis = right - left
        norm = float(np.linalg.norm(axis))
        axis = axis / norm if norm > 1e-12 else self.spread_axis.copy()
        return mid, direction, axis


def _unit(vector, label):
    """归一化并拒绝零向量（配置写错即显式失败，不静默兜底）。"""
    array = np.asarray(vector, dtype=float).reshape(-1)
    if array.shape != (3,):
        raise ValueError(label + " 必须是 3 个数值")
    norm = float(np.linalg.norm(array))
    if norm < 1e-12:
        raise ValueError(label + " 不能为零向量")
    return array / norm


def _seed_positions(model, baseline, arm_names):
    """IK 种子位形：优先用基线声明的 gripper.seed（构型自带的"向下"位形）。

    为什么需要种子：求解是局部迭代，6 维位姿误差的收敛半径有限。
    从 UR5e 全零位（手臂**竖直向上**）出发时，姿态偏差 112°，
    迭代走不出这个距离（实测收敛失败）；
    而从"肩部已下压"的位形出发，姿态偏差只剩十几度，能顺利收敛。

    种子只影响**收敛到哪一支解**，不影响最终精度（精度由位置残差门禁保证）。
    没有声明种子时回退全零，并保持显式失败（不静默兜底）。
    """
    declared = (baseline.get("gripper") or {}).get("seed")
    if declared:
        seed = {}
        for name in arm_names:
            if name not in declared:
                raise ValueError("gripper.seed 缺少关节: " + name)
            seed[name] = float(declared[name])
        return seed
    return {name: 0.0 for name in arm_names}




def build_reference_poses(root, baseline, target_id=None, target_xy_override_m=None,
                          target_z_override_m=None):
    """求解 home/approach/grasp/lift 四个参考关节姿态。

    `target_xy_override_m` / `target_z_override_m`（可选，2026-09-29 补）：**抓取点**相对**臂基座系**
    的 xy 与世界 z。为什么需要（与 Piper 侧同一理由）：场景的 `reference_solver` 契约要求
    `entry(root, baseline_doc, target_xy_override_m, target_z_override_m)` —— 联合世界把目标换算到
    臂基座系后重解，避免把"臂自己场景"的位姿照搬过去（照搬实测造成 0.367696068 m 抓取残差）。
    缺省 None ⇒ 行为与改动前**逐位一致**。
    """
    model_cfg = baseline["model"]
    source = _resolve(root, model_cfg["source"])
    if not source.is_file():
        raise FileNotFoundError(
            "UR5 模型不存在（请先跑 scripts/assemble_ur5e_2f85.py）: " + str(source)
        )
    arm_names = list(model_cfg["arm_joints"])
    target_cfg = baseline.get("target") or {}
    workbench = baseline.get("workbench") or {}
    grasp_cfg = baseline.get("grasp") or {}
    half_size = float(target_cfg.get("half_size_m", 0.025))
    top_z = float(workbench.get("top_z_m", 0.0))

    # 探测场景必须与最终验收模型同构；且必须写在源模型同目录
    # （meshdir 是相对路径，放 /tmp 会报 base_0.obj 找不到）。
    probe_scene = source.parent / (source.stem + "-probe-scene.xml")
    try:
        build_scene(
            source,
            probe_scene,
            target_id=target_id or target_cfg.get("id", "box_01"),
            half_size=half_size,
            config=baseline,
        )
        model = mujoco.MjModel.from_xml_path(str(probe_scene))
    finally:
        for leftover in (probe_scene, probe_scene.with_suffix(".json")):
            if leftover.is_file():
                leftover.unlink()
    data = mujoco.MjData(model)

    left_geom = _geom_id(model, model_cfg["finger_geoms"]["left"])
    right_geom = _geom_id(model, model_cfg["finger_geoms"]["right"])
    # 夹持区 = 完整 pad box 列表（每侧两个）。缺省回退到"左右代表接触面"
    # 两点，此时中点是 pad1 中点，即改动前的旧行为（Piper 侧口径）。
    pad_names = list(model_cfg["finger_geoms"].get("pad_boxes") or (
        model_cfg["finger_geoms"]["left"], model_cfg["finger_geoms"]["right"]
    ))
    pad_geoms = [_geom_id(model, name) for name in pad_names]
    flange_site = _site_id(
        model, _require_flange_site(model_cfg)
    )
    wrist_body = _body_id(model, model_cfg["bodies"]["wrist"])

    # 姿态期望：工具指向（法兰 → 夹持区中点，本场景为竖直**向下**）与开合轴
    # （世界某水平轴）。
    #
    # **`grasp.approach_direction` 是"后退方向"**（抓取点 → 预抓取点），本场景为
    # 竖直向上；工具指向必须取它的**反向**。两者符号相反，混用会让夹爪背对目标、
    # 法兰 z 轴朝上，抓取点落到台面以下而变成不可达位姿（历史坑，见
    # `docs/debug/2026-09-18-ur5-tool-axis-flip.md`）。
    retreat_direction = np.asarray(
        grasp_cfg.get("approach_direction") or [0.0, 0.0, 1.0], dtype=float
    )
    retreat_direction = retreat_direction / float(np.linalg.norm(retreat_direction))
    pointing_direction = -retreat_direction
    spread_axis = np.asarray(grasp_cfg.get("spread_axis") or [1.0, 0.0, 0.0], dtype=float)
    spread_axis = spread_axis / float(np.linalg.norm(spread_axis))
    # 开合轴必须与接近轴垂直，否则夹爪无法同时满足两者（显式失败而非静默降级）
    if abs(float(np.dot(retreat_direction, spread_axis))) > 1e-6:
        raise ValueError(
            "grasp.spread_axis 必须与 approach_direction 垂直: dot=%.9f"
            % float(np.dot(retreat_direction, spread_axis))
        )

    arm_joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        for name in arm_names
    ]
    if any(joint < 0 for joint in arm_joint_ids):
        raise ValueError("臂关节名在模型中不存在: " + str(arm_names))
    solver = GraspPoseSolver(
        model, data, arm_joint_ids, pad_geoms, flange_site,
        pointing_direction, spread_axis,
    )

    finger_xy = (list(target_xy_override_m) if target_xy_override_m is not None
                 else grasp_cfg.get("finger_center_xy_m"))
    if not finger_xy or len(finger_xy) != 2:
        raise ValueError("基线配置缺少 grasp.finger_center_xy_m")
    grasp_target = [
        float(finger_xy[0]), float(finger_xy[1]),
        (float(target_z_override_m) if target_z_override_m is not None else top_z + half_size),
    ]

    solver_cfg = grasp_cfg.get("solver") or {}
    offset = float(grasp_cfg.get("pregrasp_offset_m", 0.16))
    tip_clearance = float(grasp_cfg.get("tip_clearance_m", 0.003))
    base_target = np.asarray(grasp_target, dtype=float)

    # 从基线声明的种子位形出发；姿态修正负责选到"夹爪朝下"的分支
    for name, value in _seed_positions(model, baseline, arm_names).items():
        qpos_adr, _ = _joint_adr(model, name)
        data.qpos[qpos_adr] = float(value)
    mujoco.mj_forward(model, data)

    # --- 指尖离台间隙配平 ---
    # 姿态约束会让 pad 相对抓取点的偏移与 Piper 不同，
    # 因此必须用"实测 pad 最低点"迭代配平高度，而不能照搬 Piper 的常量。
    #
    # **必须扫全部 4 个 pad box**：pad2 在每个 pad 的局部 -z 侧，是整个
    # 夹爪最低的碰撞体。只扫 pad1 会漏掉它，结果是 pad2 扎进台面
    # （历史实测 pad2 底面 z=-0.00313，比台面低 3.1mm）。
    # 配平算法本体在 core（与机型无关）：沿后退方向抬高目标，直到指尖最低点
    # 离开台面达到要求间隙。这里只负责注入"实测指尖最低点"的几何来源。
    # **必须扫全部 pad box**：pad2 在每个 pad 的局部 -z 侧，是整个夹爪最低的
    # 碰撞体；只扫 pad1 会漏掉它（历史实测 pad2 底面比台面低 3.1mm）。
    per_iteration_orientation = []

    def solve_for_clearance(target):
        result, residual = solver.solve(target, solver_cfg)
        per_iteration_orientation.append(residual)
        return result

    # **配平的支撑面必须取"目标自身所在平面"**（2026-09-29 修，与 Piper 侧同源缺陷）：
    # 原先写死 `top_z` ⇒ 一旦调用方用 `target_z_override_m` 把目标搬到别的高度（联合世界里臂被抬升后
    # 目标相对臂基座可能落在台面之下），配平会把目标**拉回台面附近**（Piper 侧实测指腹中点比方块中心
    # 高 +0.17892，应为 pad_offset）。缺省无覆盖时 `grasp_target[2] - half_size == top_z` ⇒ **逐位不变**。
    support_z = float(grasp_target[2]) - half_size
    grasp_result, height_correction, clearance_trace, cleared = balance_tip_clearance(
        model,
        data,
        solve_for_clearance,
        base_target,
        retreat_direction,
        pad_geoms,
        support_z,
        tip_clearance,
        iterations=int(grasp_cfg.get("clearance_iterations", 8)),
    )
    if not cleared:
        last = clearance_trace[-1] if clearance_trace else {}
        raise ValueError(
            "指尖离台间隙配平未收敛（%d 次迭代后仍差 %.6f m）: tip_z=%.9f "
            "required=%.9f；配平轨迹=%s"
            % (
                len(clearance_trace),
                float(last.get("deficit_m", float("nan"))),
                float(last.get("tip_z_m", float("nan"))),
                support_z + tip_clearance,
                clearance_trace,
            )
        )
    # 姿态残差逐轮附回轨迹：若姿态在配平过程中漂移，说明位置与姿态在互相拉扯，
    # 此时"抬高度"这种单变量修正必然不收敛 —— 必须留证才能区分。
    for entry, residual in zip(clearance_trace, per_iteration_orientation):
        entry["orientation_deg"] = {
            str(key): round(float(value), 6)
            for key, value in (residual or {}).items()
        }
    tip_z = float(clearance_trace[-1]["tip_z_m"])
    orientation_residual = per_iteration_orientation[-1] if per_iteration_orientation else {}
    grasp_target_corrected = base_target + retreat_direction * height_correction


    def pack(result):
        return {
            "joint_positions": {
                name: float(result.joint_positions[name]) for name in arm_names
            },
            "finger_center_m": [float(v) for v in result.solved_position_m],
            "target_m": [float(v) for v in result.target_position_m],
            "position_error_m": float(result.position_error_m),
            "iterations": int(result.iterations),
        }

    grasp = pack(grasp_result)

    pregrasp_direction = np.asarray(
        grasp_cfg.get("pregrasp_direction") or retreat_direction, dtype=float
    )
    pregrasp_direction = pregrasp_direction / float(np.linalg.norm(pregrasp_direction))
    approach_target = grasp_target_corrected + pregrasp_direction * offset
    if float(approach_target[2]) <= top_z:
        raise ValueError(
            "预抓取位置未离开工作台: approach_z=%.9f workbench_top_z=%.9f"
            % (float(approach_target[2]), top_z)
        )
    approach_result, _ = solver.solve(approach_target, solver_cfg)
    approach = pack(approach_result)

    lift_offset = float(grasp_cfg.get("lift_offset_m", 0.08))
    lift_target = grasp_target_corrected + pregrasp_direction * lift_offset
    lift_result, _ = solver.solve(lift_target, solver_cfg)
    lift = pack(lift_result)

    # --- HOME 落在预抓取位正上方，保证 HOME→APPROACH 也近似竖直 ---
    home_rise = float(grasp_cfg.get("home_rise_m", 0.0) or 0.0)
    if home_rise <= 0:
        raise ValueError("grasp.home_rise_m 必须为正（否则 HOME→APPROACH 会横扫台面）")
    home_target = approach_target + np.array([0.0, 0.0, home_rise], dtype=float)
    home_result, _ = solver.solve(home_target, solver_cfg)
    home = pack(home_result)

    # --- 姿态达标校验（在 grasp 姿态上复核）---
    # 注意要重新解一次 grasp：上面最后一次 solve 落在 home 姿态上，
    # 直接 measure() 会量到 home 的姿态（初版就犯了这个错，
    # 表现为"姿态偏差 90°"这种与物理不符的整数）。
    _, _ = solver.solve(grasp_target_corrected, solver_cfg)
    _, direction_now, axis_now = solver.measure()
    direction_err = float(np.degrees(np.arccos(
        np.clip(float(np.dot(direction_now, pointing_direction)), -1.0, 1.0)
    )))
    axis_err = float(np.degrees(np.arccos(
        np.clip(abs(float(np.dot(axis_now, spread_axis))), 0.0, 1.0)
    )))
    limit = float(grasp_cfg.get("max_orientation_error_deg", 3.0))
    if direction_err > limit or axis_err > limit:
        raise ValueError(
            "参考姿态未满足姿态约束: 夹爪指向偏差=%.3f deg 开合轴偏差=%.3f deg "
            "limit=%.3f deg" % (direction_err, axis_err, limit)
        )
    pad_height_diff = abs(float(
        np.abs(data.geom_xpos[right_geom][2] - data.geom_xpos[left_geom][2])
    ))
    if pad_height_diff > 0.002:
        raise ValueError(
            "左右 pad 高度差 %.6f m 超过 2mm：夹爪倾斜，DESCEND 会把目标顶飞"
            % pad_height_diff
        )

    # --- 夹持区中心一致性门禁（本次改动新增）---
    # 复核"求解出来的夹持区中点"与"指令的目标点"确实是同一个物理点。
    # 这是本次 34mm 下沉事故的直接防复发门禁：
    # 旧实现把 pad1 中点当夹持中心，两者相差一个固定的竖直偏移
    # （实测 pad1 底面比方块顶面低 34.4mm），但残差门禁只比较
    # "求解器自报的误差"，无法发现"参考点定义本身就错了"。
    grip_mid, _, _ = solver.measure()
    grip_center_error = float(np.linalg.norm(grip_mid - grasp_target_corrected))
    if grip_center_error > float(grasp_cfg.get("tolerance_m", 1e-5)) * 10.0:
        raise ValueError(
            "夹持区中心与抓取目标不一致: |measured - target|=%.9f m "
            "（夹持区应由 finger_geoms.pad_boxes 的全部 %d 个 pad box 定义）"
            % (grip_center_error, len(pad_geoms))
        )

    # --- 工具指向门禁：夹持区必须落在法兰**沿工具指向**的一侧 ---
    # 这是"符号搞反"事故的直接防复发判据。历史事故中夹持区跑到了法兰上方
    # 134mm（即 along < 0），而姿态偏差仍报 0.008° —— 因为优化目标本身就是反的，
    # 姿态门禁无法自证目标正确。加上这一条后，符号再反会立即显式失败。
    # 注意 `.copy()`：MuJoCo 的 site_xpos 是内部缓冲区视图，不拷贝的话
    # 后续 `_gravity_hold_ctrl` 改写 qpos 时这里的"留证值"会被一起改掉。
    flange_position = np.asarray(data.site_xpos[flange_site], dtype=float).copy()
    grip_along_tool_axis = float(
        np.dot(np.asarray(grip_mid, dtype=float) - flange_position, pointing_direction)
    )
    if grip_along_tool_axis <= 0.0:
        raise ValueError(
            "夹持区不在工具指向一侧: (夹持区-法兰)·工具指向=%.6f m（应 > 0）。"
            "检查 grasp.approach_direction（后退方向）是否被误当作工具指向。"
            % grip_along_tool_axis
        )
    # 法兰在抓取位姿下必须留在台面之上：若工具指向反了，法兰会被压到台面以下。
    if float(flange_position[2]) <= top_z:
        raise ValueError(
            "抓取位姿下法兰位于台面以下: flange_z=%.6f m workbench_top_z=%.6f m"
            % (float(flange_position[2]), top_z)
        )
    # 每个 pad box 相对夹持区中心的高度分布：pad2 必然在中心之下，
    # 这个量决定 tip_clearance 的余量，必须留证以便换夹爪时快速核对。
    pad_zdeltas = sorted(
        round(float(data.geom_xpos[geom][2]) - float(grip_mid[2]), 6)
        for geom in pad_geoms
    )

    wrist = data.xpos[wrist_body].copy()
    axis = np.asarray(grasp["finger_center_m"], dtype=float) - np.asarray(
        wrist, dtype=float
    )
    axis = axis / float(np.linalg.norm(axis))

    # --- 重力前馈（逐姿态）---
    # 四个参考姿态各自"抵消重力矩所需的 ctrl 增量"，见 `_gravity_hold_ctrl`。
    # 官方执行器是纯 PD（无真机的重力前馈），不补前馈时 shoulder_lift
    # 稳态误差约 0.015 rad，折算到末端约 15mm > 5mm 验收容差。
    # **四个姿态都要算**：重力矩随位形变化（实测 shoulder_lift 28~33 N·m），
    # 只算 home 再全段复用会让 DESCEND 段残留数毫米偏差。
    # ⚠ 前馈参数必须**声明**（2026-09-29 修）：本调用原先没传 hold_ms/tolerance_rad/max_passes，
    # 靠函数默认值；而 `max_passes` 早已改成必填（实现层不写默认值）⇒ 通用/UR5e 这条路从那以后
    # 一直是坏的（`KinematicsError: 重力前馈必须声明 max_passes`），因为没有验收覆盖而未暴露。
    # 与 Piper 侧同一契约：三个数都从基线 `gravity_feedforward` 段读，缺声明即失败。
    ff_cfg = baseline.get("gravity_feedforward") or {}
    hold_ms = ff_cfg.get("hold_ms")
    tolerance_rad = ff_cfg.get("tolerance_rad")
    max_passes = ff_cfg.get("max_passes")
    if not isinstance(hold_ms, int) or isinstance(hold_ms, bool) or hold_ms <= 0:
        raise ValueError("基线必须声明 gravity_feedforward.hold_ms（正整数），实际 %r" % (hold_ms,))
    if not isinstance(tolerance_rad, (int, float)) or isinstance(tolerance_rad, bool) or tolerance_rad <= 0:
        raise ValueError("基线必须声明 gravity_feedforward.tolerance_rad（正数），实际 %r" % (tolerance_rad,))
    if not isinstance(max_passes, int) or isinstance(max_passes, bool) or max_passes <= 0:
        raise ValueError("基线必须声明 gravity_feedforward.max_passes（正整数），实际 %r" % (max_passes,))
    feedforward = {}
    feedforward_evidence = {}
    for name, pose in (
        ("home", home),
        ("approach", approach),
        ("grasp", grasp),
        ("lift", lift),
    ):
        offsets, evidence = gravity_hold_ctrl(
            model, arm_names, dict(pose["joint_positions"]),
            hold_ms=int(hold_ms), tolerance_rad=float(tolerance_rad), max_passes=int(max_passes),
        )
        feedforward[name] = offsets
        feedforward_evidence[name] = evidence

    return {
        "schema_version": "iraf.robot-reference-pose/v1",
        "source": str(source),
        "arm_joints": arm_names,
        "gripper_axis_world": [round(float(v), 9) for v in axis],
        # 工具指向（法兰 → 夹持区中点）与后退方向（抓取点 → 预抓取点）互为反向，
        # 两者都留证：只写其中一个正是"符号搞反"事故的成因。
        # `approach_direction_world` 沿用旧字段名（语义 = 后退方向），
        # 因为 scripts/build_robot_pick_scene.py 用它推导 pad_offset_axis。
        "tool_pointing_direction_world": [round(float(v), 9) for v in pointing_direction],
        "approach_direction_world": [round(float(v), 9) for v in retreat_direction],
        "retreat_direction_world": [round(float(v), 9) for v in retreat_direction],
        "pregrasp_direction_world": [round(float(v), 9) for v in pregrasp_direction],
        "spread_axis_world": [round(float(v), 9) for v in spread_axis],
        "grip_along_tool_axis_m": round(grip_along_tool_axis, 9),
        "flange_position_m": [round(float(v), 9) for v in flange_position],
        "orientation_error_deg": {
            "gripper_direction": round(direction_err, 6),
            "spread_axis": round(axis_err, 6),
            "limit": limit,
        },
        "pad_height_diff_m": round(pad_height_diff, 9),
        # 夹持区定义留证：换了夹爪或改了 pad_boxes 后，这里的名单与
        # 高度分布应与实际模型一致（不一致说明配置漂了）。
        "grip_region": {
            "geom_names": [str(name) for name in pad_names],
            "center_error_m": round(grip_center_error, 9),
            "pad_z_delta_from_center_m": pad_zdeltas,
        },
        "pregrasp_offset_m": offset,
        "tip_clearance_m": tip_clearance,
        "finger_tip_z_m": round(float(tip_z), 9),
        "finger_height_correction_m": round(float(height_correction), 9),
        "clearance_trace": clearance_trace,
        # 重力前馈：逐姿态的 ctrl 增量（关节名 → 增量），供运行期加到 ctrl 上。
        "gravity_feedforward": feedforward,
        "gravity_feedforward_evidence": feedforward_evidence,
        "grasp_point_m": [round(float(v), 9) for v in base_target],
        "finger_center_m": grasp["finger_center_m"],
        "target_z_m": top_z + half_size,
        "lift_offset_m": lift_offset,
        "home_rise_m": home_rise,
        "home": home["joint_positions"],
        "home_solved": home,
        "approach": approach,
        "grasp": grasp,
        "lift": lift,
    }


def validate_grasp_pose(scene_path, baseline, reference):
    """在最终场景上校验参考抓取姿态：pad 不碰台、张开时手指不碰任何物体。"""
    model_cfg = baseline["model"]
    arm_names = list(model_cfg["arm_joints"])
    finger_names = (
        model_cfg["finger_geoms"]["left"],
        model_cfg["finger_geoms"]["right"],
    )
    gripper_cfg = baseline.get("gripper") or {}
    open_positions = dict(gripper_cfg.get("open") or {})
    workbench = baseline.get("workbench") or {}
    grasp_cfg = baseline.get("grasp") or {}
    top_z = float(workbench.get("top_z_m", 0.0))
    tip_clearance = float(grasp_cfg.get("tip_clearance_m", 0.003))

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    # **臂关节必须早于任何 mj_step 全部写好。**
    # 历史 bug：夹爪的 actuator 分支会 mj_step 1500 次（3 秒）来让开合动作
    # 稳定，而臂关节是在**那之前**写的；动态步进期间 ctrl 全为 0，
    # 臂会被拉向零位，于是"参考姿态"被破坏，量到的指尖高度比配平阶段
    # 低 19mm（实测 tip_z=-0.0013 而配平阶段已达标）。
    # 正确做法：先把臂关节与夹爪关节全部写好，再统一做一次静态求解。
    for name in arm_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(
            reference["grasp"]["joint_positions"][name]
        )
    # 开合值可能是执行器名（2F-85 是 tendon ctrl）而不是关节名。
    # **这里不能再用"给 ctrl 再步进"的方式来设定开度**：那条路径会
    # 引入动力学，而本函数的目的是"在给定臂关节位形下静态量几何"。
    # 改为直接按标定的"执行器值 → driver 关节角"映射写 qpos，
    # 使 pad 张开到位且不扰动臂（映射关系见 profiles/ur5_mujoco.yaml
    # 的 gripper.open_positions 注释与 probe_ur5_gripper_dynamic.py）。
    for name, value in open_positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
            continue
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, str(name)
        )
        if actuator_id < 0:
            raise ValueError("夹爪开合项既不是关节也不是执行器: " + str(name))
        joint_id = int(model.actuator_trnid[int(actuator_id), 0])
        if joint_id < 0:
            raise ValueError(
                "执行器未绑定关节，无法静态设定开度: " + str(name)
            )
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)


    geom_ids = tuple(_geom_id(model, name) for name in finger_names)
    # 离台间隙与"是否碰场景"都要按**全部 pad box** 判定：pad2 是整爪最低点，
    # 只看每侧代表接触面（pad1）会漏判 pad2 扎台（历史实测低 3.1mm）。
    pad_names = list(model_cfg["finger_geoms"].get("pad_boxes") or finger_names)
    pad_geom_ids = tuple(_geom_id(model, name) for name in pad_names)
    tip_z = lowest_mesh_point_z(model, data, pad_geom_ids)
    pairs = _contact_pairs(model, data)
    pad_name_set = set(pad_names)
    touching = [
        pair for pair in pairs
        if pair[0] in pad_name_set or pair[1] in pad_name_set
    ]
    if tip_z < top_z + tip_clearance - 1e-4:
        raise ValueError(
            "参考抓取姿态指尖扎入工作台: tip_z=%.9f required=%.9f"
            % (tip_z, top_z + tip_clearance)
        )
    if touching:
        raise ValueError(
            "参考抓取姿态下手指与场景发生接触: "
            + ", ".join("%s|%s(%s)" % pair for pair in touching)
        )
    return {
        "finger_tip_z_m": round(float(tip_z), 9),
        "tip_clearance_m": tip_clearance,
        "grip_region_geoms": pad_names,
        "contact_count": len(pairs),
        "finger_contacts": [],
    }


def _grasp_axes(grasp_cfg):
    """从基线声明取出（工具指向, 开合轴）。

    语义：`approach_direction` 是**后退方向**（抓取点 → 预抓取点），
    工具指向取其反向；开合轴必须与之垂直，否则显式失败。
    """
    retreat = np.asarray(
        grasp_cfg.get("approach_direction") or [0.0, 0.0, 1.0], dtype=float
    )
    retreat = retreat / float(np.linalg.norm(retreat))
    pointing = -retreat
    spread = np.asarray(grasp_cfg.get("spread_axis") or [1.0, 0.0, 0.0], dtype=float)
    spread = spread / float(np.linalg.norm(spread))
    if abs(float(np.dot(retreat, spread))) > 1e-6:
        raise ValueError("grasp.spread_axis 必须与 approach_direction 垂直")
    return retreat, pointing, spread


def _euler_deg_to_matrix(euler_deg):
    """ZYX 外旋欧拉角 → 旋转矩阵（与场景生成器的姿态口径一致）。"""
    rx, ry, rz = (math.radians(float(value)) for value in euler_deg)
    rotation_x = np.array(
        [[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]]
    )
    rotation_y = np.array(
        [[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]]
    )
    rotation_z = np.array(
        [[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]]
    )
    return rotation_z @ rotation_y @ rotation_x


def derive_grasp_for_target(baseline, target_id):
    """按 `targets[]` 中该目标的位姿推导抓取参数，返回 (新基线, mode)。

    为什么必须做：多目标场景里各目标的位置与朝向不同，沿用固定抓取点会让参考姿态
    与目标错位（实测：box_green / box_blue 因此抓取失败）。推导内容：

    - 抓取点水平位置 = 该目标 `pos_m` 的 x/y（z 仍由台面高度 + 半边长推出）；
    - 接近方向 = 目标**顶面法向**（仅当倾斜角 ≤ `grasp.max_tilt_deg`；
      超过则回退竖直并把 mode 标为 `fallback_vertical`，不做静默通过）；
    - 预抓取方向固定为**竖直**：若 APPROACH 也沿倾斜法向偏移，
      APPROACH 与 DESCEND 的 joint1 目标不一致，下降阶段的微调会被夹爪摩擦锁死
      （实测 joint1 只走到目标的 23%）；抬升仍沿竖直只考验摩擦；
    - 夹爪 yaw 对齐目标的**面内主轴**：否则张开的手指会撞到棱角并把目标推走
      （实测 50mm 方块在 25° yaw 下被推开 57mm）。

    该行为由 `grasp.derive_from_target: true` 开启（缺省关闭，单目标场景行为不变）。
    """
    targets = baseline.get("targets") or []
    entry = next((item for item in targets if str(item.get("id")) == str(target_id)), None)
    if entry is None:
        raise ValueError("targets 中没有目标: " + str(target_id))
    if not entry.get("pos_m"):
        raise ValueError("targets[%s] 缺少 pos_m，无法推导抓取点" % target_id)

    per_target = json.loads(json.dumps(baseline))
    grasp_cfg = dict(per_target.get("grasp") or {})
    per_target["grasp"] = grasp_cfg
    position = [float(value) for value in entry["pos_m"]]
    grasp_cfg["finger_center_xy_m"] = [position[0], position[1]]

    euler = [float(value) for value in (entry.get("euler_deg") or [0.0, 0.0, 0.0])]
    max_tilt = float(grasp_cfg.get("max_tilt_deg", 30.0))
    if max(abs(euler[0]), abs(euler[1])) <= max_tilt:
        rotation = _euler_deg_to_matrix(euler)
        normal = rotation[:, 2]
        if float(normal[2]) < 0.0:
            normal = -normal
        grasp_cfg["approach_direction"] = [
            round(float(value), 9) for value in normal
        ]
        grasp_cfg["pregrasp_direction"] = [0.0, 0.0, 1.0]
        grasp_cfg["yaw_deg"] = round(
            float(math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))), 6
        )
        mode = "pose_adaptive"
    else:
        mode = "fallback_vertical"
    return per_target, mode


def raised_home_pose(root, baseline, target_id, reference, scene_builder=None):
    """求"从 APPROACH 沿预抓取方向再抬高"的 HOME 关节姿态。

    多目标场景里若沿用零姿态 HOME，从零姿态到 APPROACH 的直线运动会横扫台面、
    把干扰目标撞飞（实测顶到 0.25m 高空）。把 HOME 放在 APPROACH 上方后，
    HOME → APPROACH 近似竖直下降。由 `grasp.raised_home: true` 开启。

    `scene_builder` 由调用方注入：探测场景必须与**最终场景同一个生成器**产出，
    否则几何名可能对不上（实测：Piper 的指腹 geom 由 Piper 生成器写入，
    用通用生成器建探测场景会报"MJCF 缺少 geom: piper_left_finger"）。
    """
    model_cfg = baseline["model"]
    source = _resolve(root, model_cfg["source"])
    arm_names = list(model_cfg["arm_joints"])
    target_cfg = baseline.get("target") or {}
    grasp_cfg = baseline.get("grasp") or {}
    solver_cfg = grasp_cfg.get("solver") or {}

    probe_scene = source.parent / (source.stem + "-home-probe.xml")
    builder = scene_builder or build_scene
    try:
        builder(
            source,
            probe_scene,
            target_id=target_id or target_cfg.get("id", "box_01"),
            half_size=float(target_cfg.get("half_size_m", 0.025)),
            config=baseline,
        )
        model = mujoco.MjModel.from_xml_path(str(probe_scene))
    finally:
        for leftover in (probe_scene, probe_scene.with_suffix(".json")):
            if leftover.is_file():
                leftover.unlink()
    data = mujoco.MjData(model)

    pad_names = list(model_cfg["finger_geoms"].get("pad_boxes") or (
        model_cfg["finger_geoms"]["left"], model_cfg["finger_geoms"]["right"]
    ))
    pad_geoms = [_geom_id(model, name) for name in pad_names]
    arm_joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        for name in arm_names
    ]

    # 先摆到 APPROACH 关节姿态，让迭代从相近构型出发
    for name, value in reference["approach"]["joint_positions"].items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)

    direction = np.asarray(
        grasp_cfg.get("pregrasp_direction")
        or grasp_cfg.get("approach_direction")
        or [0.0, 0.0, 1.0],
        dtype=float,
    )
    direction = _unit(direction, "预抓取方向")
    approach_center = np.asarray(reference["approach"]["finger_center_m"], dtype=float)
    home_target = approach_center + direction * float(grasp_cfg.get("home_rise_m", 0.12))

    # 用**位置型** IK：HOME 只需要落在 APPROACH 正上方，构型由 APPROACH 播种保持，
    # 因此无需姿态约束 —— 也就不需要 flange site（Piper 的模型没有可用的工具 site）。
    result = solve_position_ik(
        model,
        data,
        home_target,
        arm_joint_ids,
        [{"kind": "geom", "id": int(geom)} for geom in pad_geoms],
        iterations=int(solver_cfg.get("iterations", 800)),
        step=float(solver_cfg.get("step", 0.5)),
        tolerance_m=float(solver_cfg.get("tolerance_m", 1e-5)),
    )
    if float(result.position_error_m) > 1e-4:
        raise ValueError(
            "HOME 抬高姿态求解残差过大: %.9f m" % result.position_error_m
        )
    # 打包残差证据一并导出（2026-09-30 §11.28）：联合模式的构建门禁要求**每个相位**都能判定
    # IK 是否收敛，而 `home` 的扁平关节解本身不带残差 ⇒ 挂到 `reference["home_solved"]`
    # （joint 侧 `_joint_reference_resolution` 直接读它；缺它即 fail-closed，不静默放行）。
    reference["home_solved"] = {
        "joint_positions": {name: float(value) for name, value in result.joint_positions.items()},
        "finger_center_m": [float(v) for v in result.solved_position_m],
        "target_m": [float(v) for v in result.target_position_m],
        "position_error_m": float(result.position_error_m),
        "iterations": int(getattr(result, "iterations", 0) or 0),
        "source": "raised_home_pose",
    }
    reference["home_source"] = "raised_above_approach"
    return {name: float(value) for name, value in result.joint_positions.items()}


def build_reference_feedforward(model, reference, baseline, prefix="", gripper_positions=None):
    """重力前馈重算入口（**共享实现**；契约与 Piper 侧同名函数完全一致）（2026-09-30 §11.31）。

    为什么共享而不是复制一份：`增量 = τ_g / kp` 只依赖**模型 + 声明参数**，与机型无关；复制一份
    必然口径漂移（本会话已因"同一逻辑两处各写一份"踩过多次）。因此本入口把调用**转发**到
    `scripts/build_piper_baseline.py` 的通用实现，两边的差异只体现在各自的 `baseline` 文档里。

    触发场景：场景的 `robots[].reference_solver.feedforward_entry` 声明本入口 ⇒ 联合构建器在
    **运行期同款模型**上重算前馈（执行器增益与臂自己场景不同 ⇒ 直接继承会按增益比例失真，
    表现为"指令位形到了、停稳位形差几毫米"的静差；实测本臂联合世界里该静差 5.862 mm）。
    """
    import importlib
    import sys
    scripts_dir = str(Path(__file__).resolve().parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    shared = importlib.import_module("build_piper_baseline")
    return shared.build_reference_feedforward(model, reference, baseline, prefix, gripper_positions)


def build_place_reference_poses(root, baseline, target_local_m, payload_half_m, grip_height_m,
                                clearance_m, touch_clearance_m, transit_local_m=None,
                                seed_positions=None):
    """放置四段关节解入口（**共享实现转发**；契约与 Piper 侧同名函数完全一致）（2026-09-30 §11.49）。

    为什么共享而不是复制：放置段的几何/判据逻辑（above/descend/release/retreat、夹口高度、
    触地间隙、净间隙）只依赖**声明 + 模型**，与机型无关；复制一份必然口径漂移
    （本会话已因"同一逻辑两处各写一份"踩过多次，见 §11.31 的同法处置）。
    本入口把调用转发到 `scripts/build_piper_baseline.py` 的通用实现，差异只在各自的 `baseline` 文档。
    """
    import importlib
    import sys
    scripts_dir = str(Path(__file__).resolve().parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    shared = importlib.import_module("build_piper_baseline")
    # **注入本机型自己的场景生成器**（§11.51）：共享实现内部要用它来建探测场景并解析资产；
    # 缺省（不注入）时它会用 Piper 的生成器 ⇒ 按 Piper 资产名找网格 ⇒ 本机型必然失败。
    return shared.build_place_reference_poses(root, baseline, target_local_m, payload_half_m,
                                              grip_height_m, clearance_m, touch_clearance_m,
                                              transit_local_m, seed_positions,
                                              scene_builder=build_scene)


def build(root, baseline_path, scene_path, calibration_path=None, target_id=None):
    """校验模型来源、求解参考姿态、生成受控场景并校验。

    `target_id` 用于多目标基线：只影响"哪个目标承载搬运约束/参考姿态"，
    缺省取基线声明的 target.id（单目标场景行为不变）。
    """
    root = Path(root).resolve()
    baseline = load_baseline(_resolve(root, baseline_path))
    output = _resolve(root, scene_path)

    grasp_cfg = baseline.get("grasp") or {}
    target_mode = None
    if grasp_cfg.get("derive_from_target"):
        # 多目标：按该目标的位姿推导抓取参数（缺 target_id 即显式失败）
        if not target_id:
            raise ValueError(
                "grasp.derive_from_target=true 时必须指定 target_id（多目标基线）"
            )
        baseline, target_mode = derive_grasp_for_target(baseline, target_id)

    reference = build_reference_poses(root, baseline, target_id=target_id)
    if (baseline.get("grasp") or {}).get("raised_home"):
        # 多目标：HOME 换成"接近轴上方抬高"的解，避免 HOME→APPROACH 横扫台面
        reference["home"] = raised_home_pose(root, baseline, target_id, reference)
        reference["home_hold_mode"] = "raised_above_approach"
    if target_mode:
        reference["target_mode"] = target_mode
    acceptance = baseline.get("acceptance") or {}
    tolerance = float(acceptance.get("pose_tolerance_m", 0.005))
    factor = float(acceptance.get("solver_error_factor", 0.1))
    error = float(reference["grasp"]["position_error_m"])
    if error > tolerance * factor:
        raise ValueError(
            "参考抓取姿态残差超出门禁容差: error=%.9fm limit=%.9fm"
            % (error, tolerance * factor)
        )

    target_cfg = baseline.get("target") or {}
    scene = build_scene(
        _resolve(root, baseline["model"]["source"]),
        output,
        target_id=target_id or target_cfg.get("id", "box_01"),
        half_size=float(target_cfg.get("half_size_m", 0.025)),
        config=baseline,
        reference=reference,
    )
    scene["grasp_pose_validation"] = validate_grasp_pose(output, baseline, reference)

    if calibration_path is not None:
        pose_path = _resolve(root, calibration_path)
        pose_path.parent.mkdir(parents=True, exist_ok=True)
        pose_path.write_text(
            json.dumps(reference, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        reference["pose_evidence"] = str(pose_path)
    scene["model_source"] = {
        "source": str(_resolve(root, baseline["model"]["source"])),
        "sha256": _sha256(_resolve(root, baseline["model"]["source"])),
    }
    scene["reference_poses"] = reference

    # **必须重写 report**：`build_scene` 内部会在 reference 附加之前
    # 先落一次盘，那一版不含 home/approach/grasp，后端拿它执行会报
    # "夹爪配置缺少字段: ['open_positions']"（实测）。
    output.with_suffix(".json").write_text(
        json.dumps(scene, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return scene


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=Path(DEFAULT_BASELINE))
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/ur5-pick-scene.xml")
    )
    parser.add_argument(
        "--pose-evidence",
        type=Path,
        default=Path("build/calibration/ur5-baseline-pose.json"),
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    scene = build(
        args.root, args.baseline, args.scene, calibration_path=args.pose_evidence
    )
    print(json.dumps(scene, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
