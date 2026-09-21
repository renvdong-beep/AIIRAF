"""Unitree Go2 四足适配器（步骤 16）：厂家细节只留在这里与声明里。

职责
----
1. **关节↔执行器映射只解析一处**（`resolve_joint_bindings`）：`go2.xml` 的关节名
   `FL_hip_joint` 与执行器名 `FL_hip` 不同（Piper 恰好同名，掩盖了这个问题），
   状态读取、力矩读取、控制写入全部走同一份绑定，避免「部分路径解析、部分路径假设」。
2. **力矩上限只从模型读**（`control.torque_limit_source: model` 由契约层强制），
   本文件不出现任何力矩数字；Profile 的关节限位只允许**收紧**模型 `jnt_range`。
3. 能力实现：`stand` / `stop` / `read_state` / `emergency_stop`。
   `locomote` **未实现** —— 首期没有步态控制器（决策 4.B 的四足范围），速度指令必须
   显式拒绝（`UnsupportedCapabilityError`），不得返回伪造成功。

数字来源（禁止在代码里写默认值）
--------------------------------
- `config/go2_loopback.yaml`：模型路径、关键帧、控制频率/增益、前馈开关、传感器名、时长。
- `profiles/unitree_go2_mujoco.yaml`：关节身份、限位、`spec.home` 站立参考位形。

诚实边界
--------
- 全部结论属于**仿真**（`simulation: true`）；真机/目标端验收 DEFERRED（板卡不在场）。
- `stand` 只证明「声明位形下站得住」；`stop` 是松力停机（力矩型电机松力即失能，
  与验收路径 `loopback.stop.mode=torque_zero_release` 同语义）。
"""

from pathlib import Path

import math
import yaml
import threading

import numpy as np

from iraf_adapters.unitree import balance as balance_module
from iraf_adapters.unitree import gait
from iraf_adapters.unitree.loopback import quat_tilt_deg
from iraf_adapters.unitree.quadruped import (
    CommandRejectedError,
    DeclarationError,
    ModelUnavailableError,
    QuadrupedAdapter,
    UnsupportedCapabilityError,
    gravity_bias_torque,
    pd_torque,
    substeps_per_control,
    validate_state,
)

#: 适配器从声明里**必须**读到的键（点号路径）。缺键 = 干净的中文失败，不是运行到一半 KeyError。
REQUIRED_KEYS = (
    "simulation",
    "robot.id",
    "robot.profile",
    "model.file",
    "initial.keyframe",
    "control.frequency_hz",
    "control.kp_nm_per_rad",
    "control.kd_nm_s_per_rad",
    "control.gravity_feedforward",
    "control.torque_limit_source",
    "stand.pose_source",
    "stand.ramp_s",
    "stand.duration_s",
    "stop.mode",
    "stop.duration_s",
    "state.sensors.quat",
    "state.sensors.gyro",
    "state.sensors.acc",
    "state.sensors.torque_pattern",
)

SUPPORTED_POSE_SOURCES = ("profile_home", "explicit")
SUPPORTED_STOP_MODES = ("torque_zero_release",)
#: Profile 的关节限位与模型 `jnt_range` 的比对容差（Profile 里的限位是四舍五入后的实测值）。
LIMIT_TOLERANCE_RAD = 1.0e-4


def repo_root():
    return Path(__file__).resolve().parents[3]


def _dig(document, dotted, label):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            raise DeclarationError("%s 缺少声明键: %s" % (label, dotted))
        node = node[part]
    return node


def load_declaration(config):
    """把 `config`（路径 / 含 declaration 的映射 / 声明本身）统一解析成 (declaration, root)。"""
    root = repo_root()
    if isinstance(config, (str, Path)):
        path = Path(config)
        path = path if path.is_absolute() else root / path
        if not path.is_file():
            raise DeclarationError("声明文件不存在: %s" % path)
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as exc:
            raise DeclarationError("声明不是合法 YAML: %s (%s)" % (path, exc))
    elif isinstance(config, dict):
        if "declaration" in config:
            return load_declaration(config["declaration"])
        if "model" in config and "control" in config:
            document = dict(config)
            if document.get("root"):
                root = Path(str(document["root"]))
        else:
            raise DeclarationError(
                "适配器配置必须是声明路径，或含 declaration 键，或本身就是声明对象"
            )
    else:
        raise DeclarationError("适配器配置类型非法: %r" % (type(config).__name__,))
    if not isinstance(document, dict):
        raise DeclarationError("声明顶层必须是对象")
    missing = []
    for key in REQUIRED_KEYS:
        node = document
        for part in key.split("."):
            if not isinstance(node, dict) or part not in node:
                missing.append(key)
                break
            node = node[part]
    if missing:
        raise DeclarationError(
            "声明缺少必需键（禁止默认值兜底）: %s" % missing
        )
    return document, root


def _sensor_id(model, mujoco, name, label):
    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, str(name))
    if sensor < 0:
        raise ModelUnavailableError("%s 引用的传感器不存在: %s" % (label, name))
    return int(sensor)


def _sensor_values(model, data, sensor_id):
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return np.asarray(data.sensordata[address:address + dimension], dtype=float).copy()


def resolve_joint_bindings(model, mujoco, joints):
    """关节 -> (joint_id, qpos_adr, dof_adr, actuator_id)。**唯一**的命名解析点。

    语义与验收路径 `loopback.resolve_binding` 一致，并由单测逐项交叉比对（避免两份事实漂移）：
    腱驱动/耦合关节没有直接执行器时必须显式失败，不得静默跳过。
    """
    binding = {}
    for name in joints:
        name = str(name)
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ModelUnavailableError("关节 %s 不在被测模型里" % name)
        actuators = [
            index
            for index in range(model.nu)
            if int(model.actuator_trntype[index]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[index, 0]) == int(joint_id)
        ]
        if not actuators:
            raise ModelUnavailableError(
                "关节 %s 没有直接关节执行器（腱驱动/耦合关节必须显式处理，不得静默跳过）" % name
            )
        if len(actuators) > 1:
            raise DeclarationError("关节 %s 绑定多个执行器: %s" % (name, actuators))
        binding[name] = {
            "joint_id": int(joint_id),
            "qpos_adr": int(model.jnt_qposadr[joint_id]),
            "dof_adr": int(model.jnt_dofadr[joint_id]),
            "actuator_id": int(actuators[0]),
        }
    return binding


class UnitreeGo2Adapter(QuadrupedAdapter):
    """Go2 仿真后端：MuJoCo 场景（步骤 13 产物）+ 声明驱动的 PD 控制。"""

    #: 已实现能力。`locomote` 不在其中：首期无步态控制器，必须显式拒绝。
    IMPLEMENTED_CAPABILITIES = frozenset(("emergency_stop", "read_state", "stand", "stop"))

    def __init__(self, declaration, profile, authority, *, root, mujoco, model, data, bindings):
        # 物理步进与显示渲染互斥（显示层通过 display_lock() 取同一把锁做快照，避免撕裂）
        self._lock = threading.Lock()
        self._display_renderer_cache = None
        # 步态资源惰性解析（步骤 02）：声明与几何都只在首次调用步态时解析/实测，
        # 未使用步态的路径（stand/stop）不因步态声明问题而失败。
        self._gait_params = None
        self._leg_geometry_cache = None
        # 力矩级平衡器（步骤 02b）：声明同样惰性解析；`_balance_stats` 记录"要求 vs 兑现"
        # （无有效支撑腿的周期数、被截断的腿、最近一次力旋量），供报告与调试定位。
        self._balance_params = None
        self._balance_stats = None
        super().__init__(declaration, profile, authority)
        self.root = Path(root)
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.bindings = bindings
        self.joint_order = [str(item) for item in profile.joints]
        self.control_hz = float(_dig(declaration, "control.frequency_hz", "声明"))
        self.kp = float(_dig(declaration, "control.kp_nm_per_rad", "声明"))
        self.kd = float(_dig(declaration, "control.kd_nm_s_per_rad", "声明"))
        self.gravity_feedforward = bool(_dig(declaration, "control.gravity_feedforward", "声明"))
        self.substeps = substeps_per_control(float(model.opt.timestep), self.control_hz)
        # 停机复位的对照量：初始（关键帧）基座高度。只作为**实测差值**的基准，
        # 不在报告里写「塌没塌」这类需要额外阈值判断的结论（阈值必须来自声明）。
        self.initial_base_z = float(data.qpos[2])
        self.qpos_adr = [bindings[name]["qpos_adr"] for name in self.joint_order]
        self.dof_adr = [bindings[name]["dof_adr"] for name in self.joint_order]
        self.actuator_ids = [bindings[name]["actuator_id"] for name in self.joint_order]
        # 力矩上限的唯一来源：模型 ctrlrange（声明层只允许取值 model，由契约层强制）。
        self.torque_lower = np.array(
            [float(model.actuator_ctrlrange[index][0]) for index in self.actuator_ids], dtype=float
        )
        self.torque_upper = np.array(
            [float(model.actuator_ctrlrange[index][1]) for index in self.actuator_ids], dtype=float
        )
        sensors = _dig(declaration, "state.sensors", "声明")
        self.sensors = {
            "quat": _sensor_id(model, mujoco, sensors["quat"], "state.sensors.quat"),
            "gyro": _sensor_id(model, mujoco, sensors["gyro"], "state.sensors.gyro"),
            "acc": _sensor_id(model, mujoco, sensors["acc"], "state.sensors.acc"),
        }
        self.torque_sensors = []
        for joint in self.joint_order:
            actuator_id = self.bindings[joint]["actuator_id"]
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if actuator_name is None:
                raise ModelUnavailableError("执行器 %d 没有名字：无法按声明解析力矩传感器" % actuator_id)
            name = str(sensors["torque_pattern"]).format(actuator=actuator_name)
            self.torque_sensors.append(_sensor_id(model, mujoco, name, "state.sensors.torque_pattern"))

    # ---- 装配 ----
    @classmethod
    def from_config(cls, config, profile, authority):
        """装配：声明 -> 模型 -> 绑定 -> 能力。任一步不可用都显式失败，不返回半成品。"""
        declaration, root = load_declaration(config)
        robot_id = str(_dig(declaration, "robot.id", "声明"))
        profile_name = str(getattr(profile, "name", ""))
        if robot_id != profile_name:
            raise DeclarationError(
                "声明 robot.id=%s 与 Profile metadata.name=%s 不一致：身份解析必须唯一"
                % (robot_id, profile_name)
            )
        if not bool(getattr(profile, "simulation", False)):
            raise DeclarationError("Profile 必须声明 simulation: true（仿真结论不得冒充真机）")

        pose_source = str(_dig(declaration, "stand.pose_source", "声明"))
        if pose_source not in SUPPORTED_POSE_SOURCES:
            raise DeclarationError(
                "stand.pose_source 只支持 %s，实际: %s" % (list(SUPPORTED_POSE_SOURCES), pose_source)
            )
        stop_mode = str(_dig(declaration, "stop.mode", "声明"))
        if stop_mode not in SUPPORTED_STOP_MODES:
            raise DeclarationError(
                "stop.mode 只支持 %s，实际: %s" % (list(SUPPORTED_STOP_MODES), stop_mode)
            )

        # 双向校验（声明即事实）：声明 render 段 ⇒ 类必须实现 render_frames，且相机在模型里真实存在。
        render_section = declaration.get("render")
        if render_section is None:
            raise DeclarationError(
                "声明缺少 render 段（S1 交互与场景观看的分辨率/相机名必须显式声明，禁止实现层默认值）"
            )
        for key in ("camera", "width_px", "height_px", "render_hz"):
            if render_section.get(key) in (None, ""):
                raise DeclarationError("render.%s 必须显式声明" % key)
        if not callable(getattr(cls, "render_frames", None)):
            raise DeclarationError("声明了 render 段但 %s 未实现 render_frames" % cls.__name__)

        model_rel = str(_dig(declaration, "model.file", "声明"))
        model_path = Path(model_rel)
        model_path = model_path if model_path.is_absolute() else Path(root) / model_path
        if not model_path.is_file():
            raise ModelUnavailableError(
                "被测模型不存在: %s（先跑声明里的 model.builder 生成场景）" % model_path
            )
        import mujoco

        try:
            model = mujoco.MjModel.from_xml_path(str(model_path))
        except Exception as exc:  # mujoco 抛的是多种异常，统一显式失败
            raise ModelUnavailableError("模型编译失败（%s）: %s" % (model_path, exc))

        keyframe_name = str(_dig(declaration, "initial.keyframe", "声明"))
        keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        if keyframe_id < 0:
            raise ModelUnavailableError(
                "initial.keyframe=%s 不在模型里（实测 nkey=%d）" % (keyframe_name, model.nkey)
            )
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
        mujoco.mj_forward(model, data)

        bindings = resolve_joint_bindings(model, mujoco, [str(item) for item in profile.joints])
        cls._assert_limits_tighten_only(model, mujoco, profile, bindings)
        adapter = cls(
            declaration,
            profile,
            authority,
            root=root,
            mujoco=mujoco,
            model=model,
            data=data,
            bindings=bindings,
        )
        adapter.model_path = model_path
        adapter.keyframe_name = keyframe_name
        adapter.keyframe_id = int(keyframe_id)
        return adapter

    @staticmethod
    def _assert_limits_tighten_only(model, mujoco, profile, bindings):
        """Profile 的关节限位只能**收紧**模型 jnt_range（调用参数只能收紧，铁律 1.3）。"""
        for joint, binding in bindings.items():
            joint_id = binding["joint_id"]
            if not bool(model.jnt_limited[joint_id]):
                raise DeclarationError("模型未对关节 %s 声明限位，静态安全上限缺失" % joint)
            model_lower = float(model.jnt_range[joint_id][0])
            model_upper = float(model.jnt_range[joint_id][1])
            declared_lower, declared_upper = profile.joint_limits[joint]
            if declared_lower < model_lower - LIMIT_TOLERANCE_RAD or (
                declared_upper > model_upper + LIMIT_TOLERANCE_RAD
            ):
                raise DeclarationError(
                    "Profile 对关节 %s 声明限位 [%r, %r] 宽于模型 jnt_range [%r, %r]："
                    "调用参数只能收紧"
                    % (joint, declared_lower, declared_upper, model_lower, model_upper)
                )

    # ---- 能力实现 ----
    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        self.require_motion_allowed("stand", lease)
        if execution_id is not None:
            # 同一执行内续跑：必须仍是同一 fencing token 且执行未终态（终态拒绝）。
            self.require_active_execution(execution_id, lease, capability="stand")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("stand", lease)

        source = str(_dig(self.declaration, "stand.pose_source", "声明"))
        if targets is None and source != "profile_home":
            raise DeclarationError(
                "stand.pose_source=%s：目标位形必须由调用方显式给出（不猜默认值）" % source
            )
        resolved, target_source = self.resolve_targets(targets)
        ramp_s = float(_dig(self.declaration, "stand.ramp_s", "声明"))
        seconds = self.resolve_duration_ms(
            duration_ms, _dig(self.declaration, "stand.duration_s", "声明")
        )
        cycles, saturated = self._run_control(resolved, seconds, ramp_s)
        report = {
            "simulation": True,
            "capability": "stand",
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "control_source_owner": str(getattr(lease, "owner", "")),
            "target_source": target_source,
            "duration_ms": seconds * 1000.0,
            "ramp_s": ramp_s,
            "control_cycles": cycles,
            "substeps_per_control": self.substeps,
            "ctrl_saturated_samples": int(saturated),
            "torque_limit_source": "model",
            "gravity_feedforward": bool(self.gravity_feedforward),
            "joint_targets_rad": dict(resolved),
            "final_state": self.read_state(),
        }
        self.ledger.finish(active_id, "SUCCEEDED")
        return report

    def stop(self, lease, execution_id=None):
        """松力停机：控制量归零并推进 `stop.duration_s`。

        停机是**安全方向**的动作，因此不被安全闭锁阻塞（闭锁只拦重新调度运动），
        但它也不会解除闭锁；仍需要控制权租约（有别的控制源在驱动时不得单方面停机）。
        """
        self.require_capability("stop")
        self.require_lease(lease, "stop")
        if execution_id is not None:
            self.require_active_execution(execution_id, lease, capability="stop")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("stop", lease)
        seconds = self.resolve_duration_ms(
            None, _dig(self.declaration, "stop.duration_s", "声明")
        )
        cycles, saturated = self._run_control(None, seconds, 0.0, zero_torque=True)
        report = {
            "simulation": True,
            "capability": "stop",
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "mode": str(_dig(self.declaration, "stop.mode", "声明")),
            "duration_ms": seconds * 1000.0,
            "control_cycles": cycles,
            "ctrl_saturated_samples": int(saturated),
            "base_height_drop_m": float(self.initial_base_z - float(self.data.qpos[2])),
            "final_tilt_deg": float(quat_tilt_deg(np.asarray(self.data.qpos[3:7], dtype=float))),
            "final_state": self.read_state(),
        }
        self.ledger.finish(active_id, "STOPPED")
        return report

    # ---- 步态（步骤 02：参数化 trot 原地踏步的控制器验收路径）----
    def _gait_parameters(self):
        """惰性解析并缓存步态声明。

        为什么不在 `from_config` 里校验：本方法依赖**机型身份**（Profile 的关节清单）与
        被测模型的几何，而装配期的声明校验只覆盖通用键。把步态声明的完整校验放在调用点，
        既保证「缺声明/相位不自洽 → 显式失败」，又不会让**没有步态声明的本体**无法装配
        （stand/stop 与步态无关）。失败仍是显式失败（`DeclarationError`），不是默认值。
        """
        if self._gait_params is None:
            self._gait_params = gait.load_gait_declaration(self.declaration, self.joint_order)
        return self._gait_params

    def _leg_geometry(self, params):
        """惰性实测腿部几何（大腿/小腿长、中立足端位置）；实测值来自被测模型，不写死约定。"""
        if self._leg_geometry_cache is None:
            self._leg_geometry_cache = gait.measure_leg_geometry(
                self.model, self.data, self.mujoco, params
            )
        return self._leg_geometry_cache

    def _body_frame_velocity(self, trunk_body):
        """机身速度（世界 → 躯干系）。阻尼偏移必须在机身系里算，否则「前后左右」会随姿态混叠。"""
        rotation = np.asarray(self.data.xmat[int(trunk_body)], dtype=float).reshape(3, 3)
        return rotation.T.dot(np.asarray(self.data.qvel[0:3], dtype=float))

    def _body_frame_omega(self, trunk_body):
        """机身角速度（世界 → 躯干系）。只做平动阻尼时滚转/俯仰无阻尼（实测会失稳）。"""
        rotation = np.asarray(self.data.xmat[int(trunk_body)], dtype=float).reshape(3, 3)
        return rotation.T.dot(np.asarray(self.data.qvel[3:6], dtype=float))

    def _stabilization_offsets(self, params, geometry, trunk_body):
        """各腿当前的阻尼足端偏移（证据用；实现与 target_provider 调用同一个函数）。"""
        velocity = self._body_frame_velocity(trunk_body)
        omega = self._body_frame_omega(trunk_body)
        return {
            code: list(gait.stabilization_offset(params, velocity, omega, item["trunk_rel_m"]))
            for code, item in geometry.items()
        }

    def _balance_parameters(self):
        """惰性解析并缓存 `balance` 段（步骤 02b）。

        与 `_gait_parameters` 同一理由：依赖机型身份与模型的完整校验放在**调用点**，
        未使用平衡器的路径（stand/stop/纯位置级步态）不因它失败。一旦调用仍是显式失败
        （`DeclarationError`），不是默认值兜底。
        """
        if self._balance_params is None:
            self._balance_params = balance_module.load_balance_declaration(self.declaration)
        return self._balance_params

    def robot_mass_kg(self):
        """整机质量（kg）：从被测模型读出（禁止在声明里写第二份质量）。"""
        return float(np.sum(np.asarray(self.model.body_mass, dtype=float)))

    def gravity_mps2(self):
        """重力加速度绝对值（m/s²）：从被测模型的 `opt.gravity` 读出（不写第二份数字）。"""
        value = abs(float(self.model.opt.gravity[2]))
        if not (value > 0.0):
            raise ModelUnavailableError(
                "模型 opt.gravity[2]=%r 不是可用的重力值：平衡器不成立" % (self.model.opt.gravity[2],)
            )
        return value

    def _attitude_terms(self):
        """当前躯干滚转/俯仰（rad，不含偏航）与四元数：与样本里的口径同一套公式。"""
        quat = np.asarray(self.data.qpos[3:7], dtype=float).copy()
        w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        r20 = 2.0 * (x * z - w * y)
        r21 = 2.0 * (y * z + w * x)
        r22 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(r21, r22)
        pitch = math.asin(max(-1.0, min(1.0, -r20)))
        return quat, roll, pitch

    def _foot_jacobian(self, dof_addresses):
        """返回 `jacobian(point_world, foot_body)` 的 3×3 平动雅可比（列 = 该腿三个关节自由度）。"""
        def compute(point_world, foot_body):
            jacp = np.zeros((3, int(self.model.nv)), dtype=float)
            jacr = np.zeros((3, int(self.model.nv)), dtype=float)
            self.mujoco.mj_jac(
                self.model, self.data, jacp, jacr,
                np.asarray(point_world, dtype=float).reshape(3), int(foot_body),
            )
            return np.asarray(jacp[:, list(dof_addresses)], dtype=float)

        return compute

    def _balance_provider(self, params, geometry, trunk_body, balance_params):
        """构造力矩级平衡器的**逐控制周期**回调：`(cycle_index, info) -> 12 维附加力矩`。

        支撑腿的判定用**实测接触力**（`contact_force_threshold_n`），不用声明的相位——实测
        「命令抬的腿 ≠ 物理离地的腿」（静态复现：命令抬 FL 0.08 m 时机身翻 17.765°，
        最终 RR 离地 0 N、FL 仍承载 44.46 N）⇒ 按相位写成的支撑集与实测恒对不上。
        没有足够支撑腿时**不施加**平衡力矩（并计数），超过声明的看门狗上限即停止施加。
        """
        threshold = float(params["verification"]["contact_force_threshold_n"])
        target_height = float(params["verification"]["height_target_m"])
        min_stance = int(balance_params["allocation"]["min_stance_legs"])
        watchdog_limit = int(balance_params["watchdog"]["max_consecutive_no_stance_cycles"])
        mass = self.robot_mass_kg()
        gravity = self.gravity_mps2()
        # B1：支撑/摆动两套位置权重（都来自声明，缺键在 `load_balance_declaration` 就失败了）。
        stance_weight = float(balance_params["stance_weight_position"])
        swing_weight = float(balance_params["weight_position"])

        def _no_stance_fallback():
            """支撑集不足 / 看门狗触发：力矩级动作整段放弃，位置权重回落 `weight_position`。

            为什么回落而不是"强制全位置控制"：`weight_position` 是旧实现里**唯一存在**的全局权重，
            回落到它，\"无有效支撑腿\"时的行为才与既有实现一致（不引入新的隐式默认值）。
            """
            stats = self._balance_stats
            stats["position_weight"]["fallback_cycles"] = (
                int(stats["position_weight"]["fallback_cycles"]) + 1
            )
            return {
                "balance_torque_nm": np.zeros(len(self.joint_order), dtype=float),
                "position_weight": np.full(len(self.joint_order), swing_weight, dtype=float),
            }

        def torque_provider(cycle_index, info):
            contact = self._leg_contact_forces(params, geometry)
            stance = [code for code in sorted(geometry) if float(contact[code]) >= threshold]
            stats = self._balance_stats
            stats["cycles"] = int(stats["cycles"]) + 1
            if len(stance) < min_stance:
                stats["no_stance_cycles"] = int(stats["no_stance_cycles"]) + 1
                stats["consecutive_no_stance"] = int(stats["consecutive_no_stance"]) + 1
                if int(stats["consecutive_no_stance"]) > watchdog_limit:
                    stats["watchdog_triggered"] = True
                    stats["disabled_cycles"] = int(stats["disabled_cycles"]) + 1
                return _no_stance_fallback()
            stats["consecutive_no_stance"] = 0
            if stats["watchdog_triggered"]:
                # 看门狗一旦触发即保持关闭：不得在“站不住”的状态下继续施加力矩级动作
                # （否则平衡器自己成为第二个不受监控的控制源）。
                stats["disabled_cycles"] = int(stats["disabled_cycles"]) + 1
                return _no_stance_fallback()
            stance_points = {
                code: np.asarray(
                    self.data.geom_xpos[int(geometry[code]["contact_geom"])], dtype=float
                ).copy()
                for code in stance
            }
            center = np.asarray(self.data.xpos[int(trunk_body)], dtype=float).copy()
            quat, roll, pitch = self._attitude_terms()
            wrench = balance_module.desired_wrench(
                balance_params,
                mass_kg=mass,
                gravity_mps2=gravity,
                height_m=float(self.data.qpos[2]),
                height_target_m=target_height,
                vertical_velocity_mps=float(self.data.qvel[2]),
                roll_rad=roll,
                pitch_rad=pitch,
                horizontal_velocity_world_mps=np.asarray(self.data.qvel[0:2], dtype=float),
                angular_velocity_world_rad_s=np.asarray(self.data.qvel[3:6], dtype=float),
            )
            allocation = balance_module.allocate_foot_forces(
                balance_params, wrench, stance_points, center
            )
            torques = {joint: 0.0 for joint in self.joint_order}
            for code in stance:
                joints = [geometry[code]["joints"][key] for key in ("hip_joint", "thigh_joint", "calf_joint")]
                dof_addresses = [self.bindings[joint]["dof_adr"] for joint in joints]
                jacobian = self._foot_jacobian(dof_addresses)(
                    stance_points[code], geometry[code]["foot_body"]
                )
                leg = balance_module.leg_joint_torques(
                    allocation["forces"][code], jacobian, joints
                )
                torques.update(leg)
            stats["stance_legs_histogram"][len(stance)] = (
                int(stats["stance_legs_histogram"].get(len(stance), 0)) + 1
            )
            stats["clamped_legs"] = sorted(
                set(stats["clamped_legs"]) | set(allocation["clamped_legs"])
            )
            stats["last_wrench"] = {
                "force_n": [float(value) for value in wrench["force_n"]],
                "torque_nm": [float(value) for value in wrench["torque_nm"]],
                "components": wrench["components"],
                "residual": allocation["residual"],
            }
            stats["last_stance_legs"] = list(stance)
            stats["max_abs_torque_nm"] = max(
                float(stats["max_abs_torque_nm"]),
                max((abs(float(value)) for value in torques.values()), default=0.0),
            )
            # B1：逐关节位置权重 —— 支撑腿关节取 `stance_weight_position`（0 ⇒ 力控），
            # 其余（摆动腿）取 `weight_position`。支撑集来自本周期**实测接触力**，
            # 与声明相位无关（实测「命令抬的腿 ≠ 物理离地的腿」）。
            stance_joints = {
                geometry[code]["joints"][key]
                for code in stance
                for key in ("hip_joint", "thigh_joint", "calf_joint")
            }
            position_weight = np.array(
                [
                    stance_weight if joint in stance_joints else swing_weight
                    for joint in self.joint_order
                ],
                dtype=float,
            )
            zeroed = int(np.count_nonzero(position_weight == 0.0))
            stats["position_weight"] = {
                "stance": stance_weight,
                "swing": swing_weight,
                "zeroed_joints_last_cycle": zeroed,
                "max_zeroed_joints": max(int(stats["position_weight"]["max_zeroed_joints"]), zeroed),
                "force_control_cycles": int(stats["position_weight"]["force_control_cycles"]) + 1,
                "fallback_cycles": int(stats["position_weight"]["fallback_cycles"]),
            }
            return {
                "balance_torque_nm": np.array(
                    [torques[joint] for joint in self.joint_order], dtype=float
                ),
                "position_weight": position_weight,
            }

        return torque_provider

    @staticmethod
    def _new_balance_stats():
        return {
            "cycles": 0,
            "no_stance_cycles": 0,
            "consecutive_no_stance": 0,
            "disabled_cycles": 0,
            "watchdog_triggered": False,
            "stance_legs_histogram": {},
            "clamped_legs": [],
            "last_stance_legs": [],
            "last_wrench": None,
            "max_abs_torque_nm": 0.0,
            # B1 逐关节权重的实测统计（"声明说支撑腿走力控"要有**兑现**证据，而不是只看开关）。
            "position_weight": {
                "stance": None,
                "swing": None,
                "zeroed_joints_last_cycle": 0,
                "max_zeroed_joints": 0,
                "force_control_cycles": 0,
                "fallback_cycles": 0,
            },
        }

    def _balance_summary(self, balance_params):
        """报告里的 `balance` 段：声明值 + 实测统计（要求 vs 兑现）。"""
        stats = self._balance_stats or self._new_balance_stats()
        return {
            "enabled": bool(balance_params["enabled"]),
            "weight_position": float(balance_params["weight_position"]),
            "stance_weight_position": float(balance_params["stance_weight_position"]),
            "weight_balance": float(balance_params["weight_balance"]),
            "include_gravity_support": bool(balance_params["include_gravity_support"]),
            "attitude": dict(balance_params["attitude"]),
            "height": dict(balance_params["height"]),
            "velocity": dict(balance_params["velocity"]),
            "allocation": dict(balance_params["allocation"]),
            "watchdog": dict(balance_params["watchdog"]),
            "robot_mass_kg": self.robot_mass_kg(),
            "gravity_mps2": self.gravity_mps2(),
            "stats": {
                "cycles": int(stats["cycles"]),
                "no_stance_cycles": int(stats["no_stance_cycles"]),
                "watchdog_triggered": bool(stats["watchdog_triggered"]),
                "disabled_cycles": int(stats["disabled_cycles"]),
                "position_weight": dict(stats["position_weight"]),
                "stance_legs_histogram": {
                    str(key): int(value) for key, value in stats["stance_legs_histogram"].items()
                },
                "clamped_legs": list(stats["clamped_legs"]),
                "max_abs_torque_nm": float(stats["max_abs_torque_nm"]),
                "last_wrench": stats["last_wrench"],
            },
        }

    def balance_hold(self, lease, duration_ms=None, execution_id=None):
        """静态抗扰保持（步骤 02b 的验收入口）：站立位形 + 声明的水平冲量 → 恢复判定。

        与 `stand` 的差别只有一处：本路径在位置级 PD 之上叠加**力矩级平衡器**，用于证明
        「静态抗扰」这条能力（`stand` 保持逐位一致，不受本路径影响）。冲量大小/时长/
        判据阈值全部来自 `balance.verification.static_disturbance`，本方法不写任何数字。
        """
        self.estop.assert_clear()
        self.require_lease(lease, "locomote")
        if execution_id is not None:
            self.require_active_execution(execution_id, lease, capability="locomote")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("locomote", lease)

        balance_params = self._balance_parameters()
        verification = balance_params["verification"]["static_disturbance"]
        params = self._gait_parameters()
        geometry = self._leg_geometry(params)
        trunk_body = gait.trunk_body_id(self.model, self.mujoco, params)
        seconds = self.resolve_duration_ms(
            duration_ms, float(verification["duration_s"]) + float(verification["hold_s"])
        )
        home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
        desired = np.array([home[joint] for joint in self.joint_order], dtype=float)
        # 目标位形的过渡斜坡复用 `stand.ramp_s`（同一事实不写第二份数字）：初始关键帧位形与
        # 目标一致时等价于无过渡（实测关键帧 home 与 Profile spec.home 逐关节相同）。
        ramp_s = float(_dig(self.declaration, "stand.ramp_s", "声明"))
        q0 = np.asarray(self.data.qpos[self.qpos_adr], dtype=float).copy()
        self._balance_stats = self._new_balance_stats()
        torque_provider = (
            self._balance_provider(params, geometry, trunk_body, balance_params)
            if balance_params["enabled"]
            else None
        )
        disturbance = {
            "force_n": float(verification["force_n"]),
            "axes": list(verification["axes"]),
            "duration_s": float(verification["duration_s"]),
        }
        samples = []
        onset = float(self.data.time)
        base_body = int(trunk_body)
        axes = {"x": 0, "y": 1}
        applied = np.zeros(6, dtype=float)
        for axis in disturbance["axes"]:
            applied[axes[axis]] = disturbance["force_n"]

        def target_provider(cycle_index, now):
            elapsed = now - onset
            active = 1.0 if elapsed < disturbance["duration_s"] else 0.0
            self.data.xfrc_applied[base_body] = applied * active
            alpha = 1.0 if ramp_s <= 0.0 else min(1.0, max(elapsed, 0.0) / ramp_s)
            return q0 + alpha * (desired - q0)

        def sample_callback(cycle_index, info):
            quat = np.asarray(self.data.qpos[3:7], dtype=float).copy()
            _quat, roll, pitch = self._attitude_terms()
            samples.append(
                {
                    "time_s": float(self.data.time),
                    "elapsed_s": float(self.data.time - onset),
                    "disturbance_active": bool(self.data.time - onset < disturbance["duration_s"]),
                    "base_height_m": float(self.data.qpos[2]),
                    "base_position_xy_m": [float(self.data.qpos[0]), float(self.data.qpos[1])],
                    "base_linear_speed_mps": float(np.linalg.norm(self.data.qvel[0:3])),
                    "tilt_deg": float(quat_tilt_deg(quat)),
                    "roll_deg": float(np.degrees(roll)),
                    "pitch_deg": float(np.degrees(pitch)),
                    "ctrl_saturated": int(np.count_nonzero(info["saturated"])),
                    "tracking_error_rad": float(np.max(np.abs(info["desired"] - np.asarray(
                        self.data.qpos[self.qpos_adr], dtype=float)))),
                }
            )

        cycles, saturated = self._run_control(
            None,
            seconds,
            0.0,
            target_provider=target_provider,
            torque_provider=torque_provider,
            sample_callback=sample_callback,
        )
        self.data.xfrc_applied[base_body] = np.zeros(6, dtype=float)
        report = {
            "simulation": True,
            "capability": "locomote",
            "path": "balance_hold",
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "control_source_owner": str(getattr(lease, "owner", "")),
            "declared_duration_s": float(seconds),
            "duration_ms": seconds * 1000.0,
            "control_cycles": cycles,
            "ctrl_saturated_samples": int(saturated),
            "disturbance": disturbance,
            "height_target_m": float(params["verification"]["height_target_m"]),
            "balance": self._balance_summary(balance_params),
            "samples": samples,
        }
        self.ledger.finish(active_id, "SUCCEEDED")
        return report

    def _leg_contact_forces(self, params, geometry):
        """按声明的接触几何量出每条腿的法向接触力（N）。只统计足端与外部（地面等）的接触。"""
        forces = {code: 0.0 for code in params["legs"]}
        geom_to_leg = {int(item["contact_geom"]): code for code, item in geometry.items()}
        result = np.zeros(6, dtype=float)
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            for geom in (int(contact.geom1), int(contact.geom2)):
                code = geom_to_leg.get(geom)
                if code is None:
                    continue
                self.mujoco.mj_contactForce(self.model, self.data, index, result)
                forces[code] += abs(float(result[0]))
        return forces

    def gait_in_place(self, lease, duration_ms=None, execution_id=None):
        """参数化步态原地踏步：位移目标恒为 0，只做支撑/摆动切换（步骤 02）。

        步态类型由声明决定（`gait.kind` = `trot` 或 `wave`）：本方法不写第二条实现——
        相位与轨迹由 `iraf_adapters.unitree.gait` 消费声明后给出，本方法只负责
        守卫顺序、租约/执行记录、以及把目标角交给**既有**的 PD + 重力前馈。

        与能力面的关系：本方法**不声明能力**（Profile 的 capabilities 仍只有 stand/stop；
        步态能力经技能层验收后回填属步骤 03）。台账按规范能力名 `locomote` 登记，
        以便审计链上「一条移动执行的来源」唯一。守卫顺序与 `stand` 同：急停闭锁 → 有效租约，
        再把目标角交给**既有**的 PD + 重力前馈（不新写第二套控制律）。
        """
        self.estop.assert_clear()
        self.require_lease(lease, "locomote")
        if execution_id is not None:
            self.require_active_execution(execution_id, lease, capability="locomote")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("locomote", lease)

        params = self._gait_parameters()
        geometry = self._leg_geometry(params)
        # 重心转移方向的实测复核（wave 必需）：声明方向与「抬腿后支撑三角形最紧边内法线」不符
        # 时显式失败（DeclarationError）。这条门禁放在调用点（需要被测模型的实测足迹）。
        sway_report = gait.sway_direction_report(params, geometry)
        seconds = self.resolve_duration_ms(duration_ms, params["verification"]["duration_s"])
        home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
        limits = {
            joint: self.profile.joint_limits[joint] for joint in self.joint_order
        }
        samples = []
        onset = float(self.data.time)
        trunk_body = gait.trunk_body_id(self.model, self.mujoco, params)
        # 力矩级平衡器（步骤 02b）：声明 enabled=false 时不传 torque_provider，
        # 步态路径与步骤 02 的实现**逐位一致**（可作对照实验的对照组）。
        balance_params = self._balance_parameters()
        self._balance_stats = self._new_balance_stats()
        torque_provider = (
            self._balance_provider(params, geometry, trunk_body, balance_params)
            if balance_params["enabled"]
            else None
        )

        def target_provider(cycle_index, now):
            elapsed = now - onset
            amplitude = gait.amplitude_at(params, elapsed)
            velocity = self._body_frame_velocity(trunk_body)
            omega = self._body_frame_omega(trunk_body)
            targets = gait.gait_joint_targets(
                params, geometry, home, limits, elapsed, amplitude, velocity, omega
            )
            return np.array([targets[joint] for joint in self.joint_order], dtype=float)

        def sample_callback(cycle_index, info):
            q = np.asarray(self.data.qpos[self.qpos_adr], dtype=float)
            quat = np.asarray(self.data.qpos[3:7], dtype=float)
            w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
            # 姿态分解用旋转矩阵的第三行（偏航不计入 tilt；roll/pitch 分开量，便于定位失稳方向）。
            r20 = 2.0 * (x * z - w * y)
            r21 = 2.0 * (y * z + w * x)
            r22 = 1.0 - 2.0 * (x * x + y * y)
            samples.append(
                {
                    "time_s": float(self.data.time),
                    "base_height_m": float(self.data.qpos[2]),
                    "base_position_xy_m": [float(self.data.qpos[0]), float(self.data.qpos[1])],
                    "base_linear_speed_mps": float(np.linalg.norm(self.data.qvel[0:3])),
                    "tilt_deg": float(quat_tilt_deg(quat)),
                    "roll_deg": float(np.degrees(np.arctan2(r21, r22))),
                    "pitch_deg": float(np.degrees(np.arcsin(max(-1.0, min(1.0, -r20))))),
                    "stab_offset_m": self._stabilization_offsets(params, geometry, trunk_body),
                    "contact_n": self._leg_contact_forces(params, geometry),
                    "ctrl_saturated": int(np.count_nonzero(info["saturated"])),
                    "tracking_error_rad": float(np.max(np.abs(info["desired"] - q))),
                }
            )

        cycles, saturated = self._run_control(
            None,
            seconds,
            float(params["ramp_s"]),
            target_provider=target_provider,
            torque_provider=torque_provider,
            sample_callback=sample_callback,
        )
        report = {
            "simulation": True,
            "capability": "locomote",
            "path": "gait_in_place",
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "control_source_owner": str(getattr(lease, "owner", "")),
            "declared_duration_s": float(params["verification"]["duration_s"]),
            "duration_ms": seconds * 1000.0,
            "control_cycles": cycles,
            "substeps_per_control": self.substeps,
            "ctrl_saturated_samples": int(saturated),
            "balance": self._balance_summary(balance_params),
            "gait": {
                "kind": params["kind"],
                "frequency_hz": params["frequency_hz"],
                "period_s": params["period_s"],
                "step_height_m": params["step_height_m"],
                "duty_factor": params["duty_factor"],
                "swing_profile": params["swing_profile"],
                "ramp_s": params["ramp_s"],
                "phase_groups": params["phase_groups"],
                # 逐相位重心转移（wave 必需；trot 为 null 而不是缺键——缺键会被下游读成"未登记"）
                "sway": None
                if params["sway"] is None
                else {
                    "amplitude_m": params["sway"]["amplitude_m"],
                    "axis": list(params["sway"]["axis"]),
                    "smooth_s": params["sway"]["smooth_s"],
                    "ramp_s": params["sway"]["ramp_s"],
                    "direction_tolerance_deg": params["sway"]["direction_tolerance_deg"],
                    "directions": {
                        code: [float(item[0]), float(item[1])]
                        for code, item in params["sway"]["directions"].items()
                    },
                },
                "sway_direction_check": sway_report,
                "legs": {
                    code: {
                        "hip_joint": item["hip_joint"],
                        "thigh_joint": item["thigh_joint"],
                        "calf_joint": item["calf_joint"],
                        "contact_geom": item["contact_geom"],
                        "phase_offset": item["phase_offset"],
                    }
                    for code, item in params["legs"].items()
                },
            },
            "geometry": {
                code: {
                    "l1_m": item["l1_m"],
                    "l2_m": item["l2_m"],
                    "neutral_x_m": item["neutral_x_m"],
                    "neutral_z_m": item["neutral_z_m"],
                    "foot_body": int(item["foot_body"]),
                    "contact_geom": int(item["contact_geom"]),
                }
                for code, item in geometry.items()
            },
            "samples": samples,
        }
        self.ledger.finish(active_id, "SUCCEEDED")
        return report

    def trot_in_place(self, lease, duration_ms=None, execution_id=None):
        """旧名薄包装（步骤 02 新增时的名字）：语义与 `gait_in_place` 完全相同。

        保留原因：既有调用点（`scripts/view_go2_gait.py` 等显示入口）不改名也能继续工作；
        步态类型仍由声明给出，`trot_in_place` 不是「第二条实现」。
        """
        return self.gait_in_place(lease, duration_ms=duration_ms, execution_id=execution_id)

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        """速度指令：首期无步态控制器，显式拒绝（不伪造「指令已生效」）。

        顺序刻意分成两步：先按通用契约校验指令**形状**（未知字段/非有限数值），
        再以"能力未实现"拒绝——这样"乱下指令"与"不会做"在诊断上可区分。
        契约层（`quadruped.verify_capabilities`）已保证 `locomote` 不会被写进能力声明，
        这里再做一层运行期拒绝，防止调用方绕过能力声明直接调方法。
        """
        resolved = self.resolve_velocity(velocity)
        raise UnsupportedCapabilityError(
            "能力 locomote 未在本后端实现（Go2 首期无步态控制器，速度指令须由步态 Provider 提供）："
            "显式拒绝，不返回伪造成功。已校验的指令: %r" % (resolved,)
        )

    # ---- 显示面（S1 交互 / 场景观看）：参数一律来自声明的 render 段 ----
    def display_lock(self):
        """返回物理步进使用的互斥锁：显示层只在快照拷贝瞬间持有它（并发契约见 SnapshotMirror）。"""
        return self._lock

    def render_settings(self):
        """返回 (camera, width_px, height_px, render_hz)，全部来自声明。"""
        render = self.declaration.get("render") or {}
        return (
            str(render.get("camera")),
            int(render.get("width_px")),
            int(render.get("height_px")),
            float(render.get("render_hz")),
        )

    def _display_renderer(self):
        """惰性创建离屏渲染器（分辨率与相机名来自声明，不写死）。"""
        with self._lock:
            if self._display_renderer_cache is None:
                camera, width, height, _hz = self.render_settings()
                camera_id = self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, camera
                )
                if camera_id < 0:
                    raise DeclarationError(
                        "声明 render.camera=%r 在模型中不存在（可用相机见模型 ncam）："
                        "显示参数必须与生成场景一致" % camera
                    )
                self._display_renderer_cache = self.mujoco.Renderer(
                    self.model, height=height, width=width
                )
                self._display_renderer_cache._iraf_camera_id = camera_id
            return self._display_renderer_cache

    def render_frames(self, count=1):
        """渲染指定数量的 RGB 帧（与臂侧同契约：内部自行加锁，调用方不持锁）。"""
        count = int(count)
        if count < 1:
            raise ValueError("render_frames 的 count 必须为正数")
        renderer = self._display_renderer()
        camera_id = getattr(renderer, "_iraf_camera_id", -1)
        frames = []
        for _ in range(count):
            with self._lock:
                renderer.update_scene(self.data, camera=camera_id)
                frames.append(np.array(renderer.render()))
        return frames

    def read_state(self):
        data = self.data
        state = {
            "simulation": True,
            "time_s": float(data.time),
            "base_position_m": [float(v) for v in data.qpos[0:3]],
            "base_quaternion_wxyz": [float(v) for v in data.qpos[3:7]],
            "base_linear_velocity_mps": [float(v) for v in data.qvel[0:3]],
            "base_angular_velocity_rad_s": [float(v) for v in data.qvel[3:6]],
            "joint_positions_rad": {
                joint: float(data.qpos[self.bindings[joint]["qpos_adr"]])
                for joint in self.joint_order
            },
            "joint_velocities_rad_s": {
                joint: float(data.qvel[self.bindings[joint]["dof_adr"]])
                for joint in self.joint_order
            },
            "joint_torque_nm": {
                joint: float(_sensor_values(self.model, data, sensor).ravel()[0])
                for joint, sensor in zip(self.joint_order, self.torque_sensors)
            },
            "imu": {
                "quaternion_wxyz": [float(v) for v in _sensor_values(self.model, data, self.sensors["quat"])],
                "angular_velocity_rad_s": [float(v) for v in _sensor_values(self.model, data, self.sensors["gyro"])],
                "linear_acceleration_mps2": [float(v) for v in _sensor_values(self.model, data, self.sensors["acc"])],
            },
            "emergency_stop": self.estop.snapshot(),
            "control_source": dict(self._control_source),
        }
        validate_state(state)
        return state

    def _apply_zero_torque(self):
        self.data.ctrl[self.actuator_ids] = 0.0
        self.mujoco.mj_forward(self.model, self.data)
        return True

    # ---- 内部 ----
    def _run_control(self, target, seconds, ramp_s, zero_torque=False, target_provider=None,
                     torque_provider=None, sample_callback=None):
        """按控制频率跑一段控制：PD（或零力矩）+ 按声明子步推进物理。

        默认行为（`target_provider is None` 且 `torque_provider is None`）与步骤 15/16 的实现
        **逐位一致**：目标位形 = 初始位形到 `target` 的声明斜坡。步态路径（步骤 02）通过
        `target_provider(cycle_index, now) -> 目标向量` 提供每周期参考轨迹，
        走的是**同一段** PD + 重力前馈 + ctrlrange 截断 + 子步推进（不新写第二套控制律）；
        `torque_provider(cycle_index, info)`（步骤 02b）是**执行器级**钩子，两种返回形式：
        ① `ndarray`（旧契约）：全局权重混合 `τ = w_pos·τ_pd + w_bal·τ_bal`；
        ② `{"balance_torque_nm": …, "position_weight": …}`（B1，逐关节权重）：
        `τ = position_weight ⊙ τ_pd + w_bal·τ_bal`（支撑腿权重 0 ⇒ 该腿走力矩级力控）。
        两种形式都先按声明权重混合、再按模型 ctrlrange 截断并重算饱和标记
        （力矩上限仍只来自模型，本文件不写任何力矩数字）。
        `sample_callback(cycle_index, info)` 只在物理步进之后被调用（量的是步进后的状态）。
        """
        cycles = int(round(float(seconds) * self.control_hz))
        if cycles < 1:
            raise CommandRejectedError(
                "控制周期数不足 1（时长 %g s × %g Hz）：不得静默跳过" % (seconds, self.control_hz)
            )
        q0 = np.asarray(self.data.qpos[self.qpos_adr], dtype=float).copy()
        q_des = (
            np.zeros(len(self.joint_order), dtype=float)
            if target is None
            else np.array([float(target[joint]) for joint in self.joint_order], dtype=float)
        )
        start = float(self.data.time)
        saturated_total = 0
        # 混合权重只在**用到力矩通路时**才从声明解析（无平衡器声明的本体不会因缺段失败）。
        balance_weights = None
        if torque_provider is not None:
            balance_params = self._balance_parameters()
            balance_weights = (
                float(balance_params["weight_position"]),
                float(balance_params["weight_balance"]),
            )
        for cycle_index in range(cycles):
            q = np.asarray(self.data.qpos[self.qpos_adr], dtype=float)
            dq = np.asarray(self.data.qvel[self.dof_adr], dtype=float)
            if zero_torque:
                ctrl = np.zeros_like(q)
                saturated = np.zeros_like(q, dtype=bool)
                desired = np.array(q_des, dtype=float)
            else:
                now = float(self.data.time)
                if target_provider is not None:
                    desired = np.asarray(target_provider(cycle_index, now), dtype=float)
                else:
                    alpha = 1.0 if float(ramp_s) <= 0.0 else min(1.0, max(now - start, 0.0) / float(ramp_s))
                    desired = q0 + alpha * (q_des - q0)
                tau_ff = (
                    gravity_bias_torque(self.model, self.data, self.mujoco, self.dof_adr)
                    if self.gravity_feedforward
                    else np.zeros_like(q)
                )
                ctrl, saturated = pd_torque(
                    q, dq, desired, self.kp, self.kd, tau_ff, self.torque_lower, self.torque_upper
                )
                if torque_provider is not None:
                    returned = torque_provider(
                        cycle_index,
                        {"desired": desired, "ctrl": ctrl, "saturated": saturated},
                    )
                    if isinstance(returned, dict):
                        # B1（ADR-0008 决策 1d，2026-09-21）：**逐关节**位置权重 —— 支撑腿关节取声明
                        # `balance.stance_weight_position`（0 ⇒ 该腿完全走力矩级力控，τ_pd 含重力前馈
                        # 一并乘 0），摆动腿取 `balance.weight_position`。权重向量由 provider 给出：
                        # 支撑集是它按**实测接触力**判定的，本方法不做第二份相位/接触推断。
                        for key in ("balance_torque_nm", "position_weight"):
                            if key not in returned:
                                raise DeclarationError(
                                    "平衡器返回的映射缺少键 %r（B1 逐关节权重契约）" % key
                                )
                        extra = np.asarray(returned["balance_torque_nm"], dtype=float)
                        position_weight = np.asarray(returned["position_weight"], dtype=float)
                        for label, vector in (
                            ("balance_torque_nm", extra),
                            ("position_weight", position_weight),
                        ):
                            if vector.shape != ctrl.shape:
                                raise DeclarationError(
                                    "平衡器返回的 %s 形状 %r 与关节数 %d 不符"
                                    % (label, vector.shape, len(self.joint_order))
                                )
                        if np.any(position_weight < 0.0) or np.any(position_weight > 1.0):
                            raise DeclarationError(
                                "平衡器返回的 position_weight 越界（必须落在 [0,1]：>1 是第二份增益、"
                                "<0 会把 PD 变成正反馈）"
                            )
                        ctrl = position_weight * ctrl + balance_weights[1] * extra
                    else:
                        extra = np.asarray(returned, dtype=float)
                        if extra.shape != ctrl.shape:
                            raise DeclarationError(
                                "平衡器返回的力矩向量形状 %r 与关节数 %d 不符"
                                % (extra.shape, len(self.joint_order))
                            )
                        ctrl = (
                            balance_weights[0] * ctrl + balance_weights[1] * extra
                        )
                    saturated = np.logical_or(ctrl < self.torque_lower, ctrl > self.torque_upper)
                    ctrl = np.clip(ctrl, self.torque_lower, self.torque_upper)
            saturated_total += int(np.count_nonzero(saturated))
            self.data.ctrl[self.actuator_ids] = ctrl
            for _ in range(self.substeps):
                with self._lock:
                    self.mujoco.mj_step(self.model, self.data)
            if sample_callback is not None:
                sample_callback(
                    cycle_index,
                    {"desired": np.asarray(desired, dtype=float), "ctrl": ctrl, "saturated": saturated},
                )
        return cycles, saturated_total

    def describe(self):
        summary = super().describe()
        summary.update(
            {
                "kind": "unitree_go2_mujoco",
                "model": {
                    "path": str(self.model_path),
                    "nq": int(self.model.nq),
                    "nv": int(self.model.nv),
                    "nu": int(self.model.nu),
                    "timestep_s": float(self.model.opt.timestep),
                    "keyframe": self.keyframe_name,
                },
                "joints": list(self.joint_order),
                "joint_actuator_binding": {
                    joint: {
                        "actuator_id": int(binding["actuator_id"]),
                        "actuator_name": self.mujoco.mj_id2name(
                            self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, binding["actuator_id"]
                        ),
                    }
                    for joint, binding in self.bindings.items()
                },
                "torque_limits_nm": {
                    joint: [
                        float(self.torque_lower[index]),
                        float(self.torque_upper[index]),
                    ]
                    for index, joint in enumerate(self.joint_order)
                },
                "control": {
                    "frequency_hz": self.control_hz,
                    "kp_nm_per_rad": self.kp,
                    "kd_nm_s_per_rad": self.kd,
                    "gravity_feedforward": bool(self.gravity_feedforward),
                    "substeps_per_control": int(self.substeps),
                    "torque_limit_source": "model",
                },
            }
        )
        return summary
