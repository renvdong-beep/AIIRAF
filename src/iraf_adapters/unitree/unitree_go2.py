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

import yaml
import numpy as np

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
    def _run_control(self, target, seconds, ramp_s, zero_torque=False):
        """按控制频率跑一段控制：PD（或零力矩）+ 按声明子步推进物理。"""
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
        for _ in range(cycles):
            q = np.asarray(self.data.qpos[self.qpos_adr], dtype=float)
            dq = np.asarray(self.data.qvel[self.dof_adr], dtype=float)
            if zero_torque:
                ctrl = np.zeros_like(q)
                saturated = np.zeros_like(q, dtype=bool)
            else:
                now = float(self.data.time)
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
            saturated_total += int(np.count_nonzero(saturated))
            self.data.ctrl[self.actuator_ids] = ctrl
            for _ in range(self.substeps):
                self.mujoco.mj_step(self.model, self.data)
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
