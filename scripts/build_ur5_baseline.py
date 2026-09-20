"""按 UR5 基线配置求解参考关节姿态、生成受控场景并校验。

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
from pathlib import Path

import mujoco
import numpy as np
import yaml

from build_robot_pick_scene import build_scene
from iraf_core.kinematics import solve_position_ik

DEFAULT_BASELINE = "config/ur5_simulation_baseline.yaml"


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


def _finger_tip_z(model, data, geom_ids):
    """指腹 geom 在世界系下的最低点（box 与 mesh 都要支持）。"""
    lowest = float("inf")
    for geom_id in geom_ids:
        geom_type = int(model.geom_type[geom_id])
        rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
        if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
            half = np.asarray(model.geom_size[geom_id], dtype=float)
            extent_z = float(np.abs(rotation[2, :]) @ half)
            lowest = min(lowest, float(data.geom_xpos[geom_id][2]) - extent_z)
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id >= 0 and int(model.mesh_vertnum[mesh_id]) > 0:
            count = int(model.mesh_vertnum[mesh_id])
            start = int(model.mesh_vertadr[mesh_id])
            verts = np.asarray(model.mesh_vert[start:start + count], dtype=float)
            world = (
                np.asarray(data.geom_xpos[geom_id], dtype=float) + verts @ rotation.T
            )
            lowest = min(lowest, float(world[:, 2].min()))
        else:
            centre = np.asarray(data.geom_xpos[geom_id], dtype=float)
            half = float(
                np.abs(np.asarray(model.geom_size[geom_id], dtype=float)).max()
            )
            lowest = min(lowest, float(centre[2]) - half)
    return lowest


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


class OrientedGraspSolver:
    """带姿态约束的抓取逆解：直接对 6 维位姿误差做阻尼最小二乘。

    **抓取点 = 四个 pad box 的中点**（由 `finger_geoms.pad_boxes` 声明），
    不是任一 pad1 geom 的中心。原因：2F-85 每侧 pad 是上下两个 box
    （pad1 在局部 +z、pad2 在 -z），pad1 的中点并不是夹持中心。
    用 pad1 中心当目标时整套 pad 相对方块下沉约 34mm，pad2 扎进台面，
    DESCEND 段下侧 pad 先撞方块侧面（probe_pad_vs_block.py 实测）。

    **为什么不用"位置 IK + 零空间姿态修正"（实测走不通）：**
    - 夹爪指向不能用"pad 中点 - 法兰"的位置差雅可比修正：
      两者同挂腕部末端树，位置差的偏导数几乎完全抵消
      （实测 jac 范数 ~1e-16），该维完全推不动，
      表现为指向偏差恒为 90.000° 不收敛；
    - 换成旋转雅可比后指向可收敛，但开合轴仍卡在 41~45°：
      左右 pad 的旋转雅可比几乎相同，任何"两指相减"的构造都退化，
      开合轴那一维条件数极差。

    **改用的方法**：把姿态期望写成法兰的**目标旋转矩阵**（z 轴取夹爪
    指向的反方向、x 轴取开合轴），然后对
        e = [夹持区中点位置误差(3)；法兰旋转误差(3)]
    做阻尼最小二乘。位置雅可比取四个 pad box 的**平均**，
    姿态雅可比取法兰的旋转雅可比，6 行 × N 列，条件数正常。
    实测 11 次迭代即收敛到位置 4.5e-07 m、指向 0.000°、开合轴 0.000°，
    两 pad 高度差 0.000000 m（probe_wrist_solution.py）。

    期望旋转由基线的两条**构型无关物理量**声明：
    - `grasp.approach_direction`：预抓取/抬升的**后退方向**（抓取点 → 预抓取点）；
    - `grasp.spread_axis`：开合轴（左 pad → 右 pad），须与之垂直。

    本类接收的参数是**工具指向**（法兰 → 夹持区中点），即后退方向的**反向**。
    两者语义相反、不可互相替代：把后退方向当工具指向会让夹爪"背对"目标，
    法兰 z 轴朝上、夹持区落在法兰上方 134mm，抓取点变成台面以下的不可达位姿
    （实测见 `docs/debug/2026-09-20-ur5-tool-axis-flip.md`）。
    """

    def __init__(self, model, data, arm_names, pad_geoms,
                 flange_site, pointing_direction_world, spread_axis_world):
        self.model = model
        self.data = data
        self.arm = list(arm_names)
        self.joint_ids = [
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
            for name in self.arm
        ]
        self.qpos_adr = [int(model.jnt_qposadr[j]) for j in self.joint_ids]
        self.dof_adr = [int(model.jnt_dofadr[j]) for j in self.joint_ids]
        #: 四个 pad box 的 geom id，顺序为 [左 pad1, 左 pad2, 右 pad1, 右 pad2]。
        self.pad_geoms = [int(geom) for geom in pad_geoms]
        if len(self.pad_geoms) != 4:
            raise ValueError(
                "夹持区定义必须是 4 个 pad box，实际 %d 个" % len(self.pad_geoms)
            )
        self.flange_site = int(flange_site)
        self.pointing = self._unit(pointing_direction_world)
        self.spread_axis = self._unit(spread_axis_world)
        if abs(float(np.dot(self.pointing, self.spread_axis))) > 1e-6:
            raise ValueError("spread_axis 必须与工具指向垂直")
        self.target_rot = self._target_rotation()

    @staticmethod
    def _unit(vector):
        arr = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(arr))
        if norm < 1e-12:
            raise ValueError("期望方向不能为零向量")
        return arr / norm

    def _target_rotation(self):
        """法兰的目标旋转矩阵。

        实测约定（probe_seed_orientation.py + probe_tool_axis.py）：
            - 夹爪指向（法兰 → 夹持区中点）= 法兰 **+z** 轴；
            - 开合轴（左 pad → 右 pad）= 法兰 +x 轴；
            - 官方 home 位形下法兰 z 轴 = [0,0,-1]，夹持区确实在法兰下方 0.134m。

        因此目标旋转取 z 轴 = **工具指向**（= 后退方向的反向）、x 轴 = 开合轴。

        历史坑（本次修复）：早期把 `approach_direction`（后退方向，竖直向上）
        直接当作法兰 z 轴，得到"夹爪朝上"的解 —— 夹持区被抬到法兰上方 134mm，
        要让 pad 落到方块中心就必须把法兰压到 z=-0.109m（台面以下），
        实际执行时 wrist_2_link 的碰撞体先压在方块顶面上，
        表现为 shoulder_lift 跟踪误差 0.484 rad、末端距目标 0.314m。
        """
        z_axis = self.pointing
        x_axis = self.spread_axis
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / float(np.linalg.norm(y_axis))
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack([x_axis, y_axis, z_axis])

    def measure(self):
        """量出夹持区中点、夹爪指向、开合轴。

        夹持区中点 = 4 个 pad box 中心的中点。
        开合轴取"左两点均值 → 右两点均值"的方向，即与
        `build_ur5_baseline` 里构造雅可比时用的同一条轴，
        保证"量的轴"与"优化的轴"是同一个物理量。
        """
        mujoco.mj_forward(self.model, self.data)
        positions = [
            np.asarray(self.data.geom_xpos[geom], dtype=float)
            for geom in self.pad_geoms
        ]
        mid = sum(positions) / float(len(positions))
        left = (positions[0] + positions[1]) / 2.0
        right = (positions[2] + positions[3]) / 2.0
        flange = np.asarray(self.data.site_xpos[self.flange_site], dtype=float)
        direction = mid - flange
        dn = float(np.linalg.norm(direction))
        direction = direction / dn if dn > 1e-12 else self.pointing.copy()
        axis = right - left
        an = float(np.linalg.norm(axis))
        axis = axis / an if an > 1e-12 else self.spread_axis.copy()
        return mid, direction, axis


    @staticmethod
    def _rotation_vector(current, target):
        """两个旋转矩阵之间的旋转向量（轴×角）。"""
        relative = target @ current.T
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.asarray(relative, dtype=float).reshape(-1))
        vector = np.zeros(3)
        # 注意 muJoCo 的 quat2Vel 返回"轴×半角"，乘 2 得到完整旋转角
        mujoco.mju_quat2Vel(vector, quat, 2.0)
        return vector

    def solve(self, target, solver_cfg, orientation_iterations=None):
        """解出满足"4 点夹持区中点 = target"且姿态达标的关节角。

        返回 (IkResult 兼容对象, 姿态误差字典)。为保持与既有调用方
        （`pack`）的兼容，返回对象只暴露契约层的 4 个字段。
        """
        target = np.asarray(target, dtype=float)
        iterations = int(
            orientation_iterations
            if orientation_iterations is not None
            else solver_cfg.get("pose_iterations", 600)
        )
        position_tolerance = float(solver_cfg.get("tolerance_m", 1e-5))
        best = None
        for iteration in range(iterations):
            mid, _, _ = self.measure()
            current_rot = np.asarray(
                self.data.site_xmat[self.flange_site], dtype=float
            ).reshape(3, 3)
            e_pos = target - mid
            e_rot = self._rotation_vector(current_rot, self.target_rot)
            pos_norm = float(np.linalg.norm(e_pos))
            rot_norm = float(np.linalg.norm(e_rot))
            if best is None or pos_norm < best[0]:
                best = (
                    pos_norm,
                    {name: float(self.data.qpos[adr])
                     for name, adr in zip(self.arm, self.qpos_adr)},
                    iteration + 1,
                )
            if pos_norm < position_tolerance and rot_norm < 1e-4:
                break

            jac_flange_r = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(
                self.model, self.data, None, jac_flange_r, self.flange_site
            )
            # 位置雅可比 = 4 个 pad box 的算术平均（对应"4 点中点"的导数）。
            # 必须与 measure() 取中点的方式完全一致：只对 pad1 求平均
            # 会让"优化的位置"与"量的位置"不是同一个点，残差门禁失去意义。
            jac_mid = np.zeros((3, self.model.nv))
            for geom in self.pad_geoms:
                jac_pad = np.zeros((3, self.model.nv))
                mujoco.mj_jacGeom(self.model, self.data, jac_pad, None, geom)
                jac_mid += jac_pad
            jac_mid = (jac_mid / float(len(self.pad_geoms)))[:, self.dof_adr]
            jac = np.vstack([jac_mid, jac_flange_r[:, self.dof_adr]])
            err = np.concatenate([e_pos, e_rot])
            lam = 1e-4
            delta = jac.T @ np.linalg.solve(
                jac @ jac.T + lam * np.eye(6), err
            )
            delta = np.clip(delta, -0.2, 0.2)
            for index, joint_id in enumerate(self.joint_ids):
                low, high = self.model.jnt_range[joint_id]
                value = float(self.data.qpos[self.qpos_adr[index]]) + float(delta[index])
                self.data.qpos[self.qpos_adr[index]] = min(max(value, low), high)

        mid, direction, axis = self.measure()
        solved = SimpleIkResult(
            joint_positions={
                name: float(self.data.qpos[adr])
                for name, adr in zip(self.arm, self.qpos_adr)
            },
            solved_position_m=mid,
            target_position_m=target,
            position_error_m=float(np.linalg.norm(target - mid)),
            iterations=int(best[2]) if best else 0,
        )
        errors = {
            "gripper_direction_deg": float(np.degrees(np.arccos(
                np.clip(float(np.dot(direction, self.pointing)), -1.0, 1.0)
            ))),
            "spread_axis_deg": float(np.degrees(np.arccos(
                np.clip(abs(float(np.dot(axis, self.spread_axis))), 0.0, 1.0)
            ))),
        }
        return solved, errors


class SimpleIkResult:
    """契约层 `IkResult` 的最小兼容视图（只含调用方用到的字段）。"""

    def __init__(self, joint_positions, solved_position_m, target_position_m,
                 position_error_m, iterations):
        self.joint_positions = joint_positions
        self.solved_position_m = solved_position_m
        self.target_position_m = target_position_m
        self.position_error_m = position_error_m
        self.iterations = iterations


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


def _gravity_hold_ctrl(model, arm_names, hold_positions, hold_ms=4000,
                       tolerance_rad=1e-3):
    """求每个臂关节"抵消重力所需的 ctrl 增量"（重力前馈）。

    **为什么需要**：UR5e 官方执行器是
    `<general gainprm=2000 biasprm=[0,-2000,-400]>`，即
        force = 2000*(ctrl - qpos) - 400*qvel
    这是一个**纯 PD**，没有真实 UR 控制器里的重力/惯量前馈。于是重力矩不为零的
    位形下必须靠稳态位置误差平衡，且该误差是标定不掉的：
        Δq = τ_gravity / gain
    实测 shoulder_lift 的 Δq ≈ 0.014~0.017 rad，折算到末端约 15mm，
    而抓取验收容差是 5mm，因此 DESCEND 后对齐门禁必然失败
    （实测 `末端未到达目标抓取位姿: distance=0.016639m tolerance=0.005m`）。
    注意这是**仿真建模简化**而非真机特性：真机 UR5e 的位置伺服内部已做重力补偿。

    **重力矩取 `qfrc_bias`（qvel=0），不要用 `mj_inverse`**：
    MuJoCo 的逆动力学按执行器力限**截断**结果 —— 本模型实测稳定返回
    shoulder_lift/elbow = -150.000 N·m、wrist_1 = +28.000 N·m，
    恰好等于 UR5e 官方力矩限值，而手算与仿真实测都表明该位形只需约 30 N·m。
    截断值连符号都是错的（作为前馈会把臂推向反方向）。
    `qfrc_bias` 在 qvel=0 时就是"保持静止所需的广义力"，无截断、无接触污染。

    **做法**：τ = qfrc_bias[臂关节自由度]，Δctrl = τ / gainprm[0]，
    其中 gainprm[0] 由模型读出（UR5e 肩/肘 2000、腕 500），不写死。

    **验证方式**：把 Δctrl 加到目标 ctrl 上做一次静态保持仿真，
    要求稳态关节误差 ≤ tolerance_rad。判据与最终验收同源（都在物理上验证），
    不依赖对增益取值的假设。

    返回 ({关节名: ctrl 增量}, 证据字典)。
    """
    actuator_gain = {}
    for name in arm_names:
        # **执行器名可能与关节名不同**：UR5e 的 actuator 是 shoulder_pan /
        # shoulder_lift / ... （不带 _joint 后缀），关节名是
        # shoulder_pan_joint / ...。直接按关节名查 actuator 会报
        # "缺少臂执行器: shoulder_pan_joint"（实测）。
        # 因此这里统一走"关节名 → 驱动它的执行器"反查，Piper 那种同名
        # 构型同样适用。
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError("缺少臂关节: " + name)
        actuator_id = -1
        for index in range(int(model.nu)):
            if int(model.actuator_trnid[index, 0]) == int(joint_id):
                actuator_id = index
                break
        if actuator_id < 0:
            raise ValueError("臂关节没有对应的执行器: " + name)
        # 一阶增益取 gainprm[0]（位置反馈系数）。在 ctrl 与关节角同量纲时
        # 它就是"ctrl 增量 → 力增量"的斜率；UR5e 官方为 2000。
        gain = float(model.actuator_gainprm[actuator_id][0])
        if gain <= 0:
            raise ValueError(
                "执行器 %s 的 gainprm[0]=%.6f 非正，无法推算前馈量"
                % (name, gain)
            )
        actuator_gain[name] = gain

    joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        for name in arm_names
    ]
    dof_adrs = [int(model.jnt_dofadr[j]) for j in joint_ids]
    qpos_adrs = [int(model.jnt_qposadr[j]) for j in joint_ids]

    # 用自己的 MjData：不污染调用方状态，也避免"留证数组是内部缓冲区视图"
    # 这类隐蔽错误（调用方的 data 可能停在求解器的中间位形上）。
    data = mujoco.MjData(model)
    # 场景状态（方块位置、夹爪开度）必须与运行一致：直接用 keyframe 初始化。
    if int(model.nkey) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    # 固定夹爪关节到位形里的开度（若有），避免把夹爪自由度算进补偿
    for name, value in (hold_positions or {}).items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    for index, name in enumerate(arm_names):
        data.qpos[qpos_adrs[index]] = float(hold_positions[name])
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.qfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)
    # qvel=0 时 qfrc_bias 即"保持该位形静止所需的广义力"（含重力、科氏=0）。
    # 不包含接触力，也不受 actuator forcerange 截断影响。
    bias = np.asarray(data.qfrc_bias, dtype=float).copy()
    compensation = {
        name: float(bias[dof_adrs[index]] / actuator_gain[name])
        for index, name in enumerate(arm_names)
    }
    gravity_torque = {
        name: round(float(bias[dof_adrs[index]]), 6)
        for index, name in enumerate(arm_names)
    }

    # --- 验证：施加前馈后做静态保持仿真，量稳态关节误差 ---
    for index, name in enumerate(arm_names):
        joint_id = joint_ids[index]
        actuator_id = -1
        for candidate in range(int(model.nu)):
            if int(model.actuator_trnid[candidate, 0]) == int(joint_id):
                actuator_id = candidate
                break
        data.ctrl[actuator_id] = float(hold_positions[name]) + compensation[name]
    steps = max(1, int(round(hold_ms / 1000.0 / float(model.opt.timestep))))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    residual_rad = {
        name: round(float(data.qpos[qpos_adrs[index]]) - float(hold_positions[name]), 9)
        for index, name in enumerate(arm_names)
    }
    worst = max(abs(value) for value in residual_rad.values())
    if worst > tolerance_rad:
        raise ValueError(
            "重力前馈验证未通过: 静态保持 %dms 后最大关节误差 %.9f rad（限 %.9f rad）。"
            "残余误差=%s" % (hold_ms, worst, tolerance_rad, residual_rad)
        )
    return (
        {name: round(float(value), 9) for name, value in compensation.items()},
        {
            "method": "qfrc_bias_plus_static_hold",
            "gravity_torque_nm": gravity_torque,
            "actuator_gain": {k: round(v, 3) for k, v in actuator_gain.items()},
            "hold_ms": hold_ms,
            "residual_joint_rad": residual_rad,
            "worst_residual_rad": round(worst, 9),
            "tolerance_rad": tolerance_rad,
        },
    )


def build_reference_poses(root, baseline, target_id=None):
    """求解 home/approach/grasp/lift 四个参考关节姿态。"""
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
        model, (model_cfg.get("bodies") or {}).get("flange_site", "attachment_site")
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

    solver = OrientedGraspSolver(
        model, data, arm_names, pad_geoms, flange_site,
        pointing_direction, spread_axis,
    )

    finger_xy = grasp_cfg.get("finger_center_xy_m")
    if not finger_xy or len(finger_xy) != 2:
        raise ValueError("基线配置缺少 grasp.finger_center_xy_m")
    grasp_target = [float(finger_xy[0]), float(finger_xy[1]), top_z + half_size]

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
    height_correction = 0.0
    tip_z = None
    orientation_residual = None
    clearance_trace = []
    cleared = False
    for iteration in range(int(grasp_cfg.get("clearance_iterations", 8))):
        target = base_target + retreat_direction * height_correction
        grasp_result, orientation_residual = solver.solve(target, solver_cfg)
        tip_z = _finger_tip_z(model, data, pad_geoms)
        deficit = (top_z + tip_clearance) - tip_z
        # 留证：配平过程的每一步都记录下来。之前只断言"未收敛"而不留轨迹，
        # 导致无法区分"配平没跑够"与"配平跑够了但被后续步骤破坏"。
        clearance_trace.append({
            "iteration": iteration,
            "correction_m": round(float(height_correction), 9),
            "tip_z_m": round(float(tip_z), 9),
            "deficit_m": round(float(deficit), 9),
            "position_error_m": round(float(grasp_result.position_error_m), 9),
            # 姿态残差：若姿态在配平过程中漂移，说明位置与姿态在互相拉扯，
            # 此时"抬高度"这种单变量修正必然不收敛。
            # 注意 `orientation_residual` 是字典（键为 gripper_direction_deg /
            # spread_axis_deg），不是二元组。
            "orientation_deg": {
                str(key): round(float(value), 6)
                for key, value in (orientation_residual or {}).items()
            },
        })
        if deficit <= 1e-4:
            cleared = True
            break
        height_correction += float(deficit)
    if not cleared:
        raise ValueError(
            "指尖离台间隙配平未收敛（%d 次迭代后仍差 %.6f m）: tip_z=%.9f "
            "required=%.9f；配平轨迹=%s"
            % (
                len(clearance_trace), float(deficit), float(tip_z),
                top_z + tip_clearance, clearance_trace,
            )
        )
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
    feedforward = {}
    feedforward_evidence = {}
    for name, pose in (
        ("home", home),
        ("approach", approach),
        ("grasp", grasp),
        ("lift", lift),
    ):
        offsets, evidence = _gravity_hold_ctrl(
            model, arm_names, dict(pose["joint_positions"])
        )
        feedforward[name] = offsets
        feedforward_evidence[name] = evidence

    return {
        "schema_version": "iraf.ur5-reference-pose/v1",
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
    tip_z = _finger_tip_z(model, data, pad_geom_ids)
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


def build(root, baseline_path, scene_path, calibration_path=None):
    """校验模型来源、求解参考姿态、生成受控场景并校验。"""
    root = Path(root).resolve()
    baseline = load_baseline(_resolve(root, baseline_path))
    output = _resolve(root, scene_path)

    reference = build_reference_poses(root, baseline)
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
        target_id=target_cfg.get("id", "box_01"),
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
