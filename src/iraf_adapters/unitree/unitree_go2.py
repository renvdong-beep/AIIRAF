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
import sys

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
    pd_torque_raw,
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
SUPPORTED_STOP_MODES = ("torque_zero_release", "damped_hold")
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

    def stop(self, lease, execution_id=None, *, mode=None, tilt_limit_deg=None):
        """停机分派：`mode=None` / `torque_zero_release` ⇒ 既有失能停机（行为逐位不变）；
        `damped_hold` ⇒ 受控停止（**尚未实现**，见 `.hermes/plans/2026-09-23-damped-hold-code.md`）。

        `mode` 由上层 `iraf_skills.quadruped.resolve_stop_mode` 决定，**调用方不得自选**
        （`profiles/safety/quadruped_lab.yaml` 的 `stop_modes.*.applies_to` 已规定适用路径）。
        `damped_hold` 落地前不得进入 `SUPPORTED_STOP_MODES`（声明 ⊆ 实现，不虚报能力）。
        """
        if mode is None or mode == "torque_zero_release":
            return self._stop_torque_zero_release(lease, execution_id)
        if mode == "damped_hold":
            return self.damped_hold(lease, execution_id=execution_id,
                                    tilt_limit_deg=tilt_limit_deg)
        raise CommandRejectedError(
            "不支持的 stop 模式 %r（支持：%s）" % (mode, SUPPORTED_STOP_MODES))

    def _stop_torque_zero_release(self, lease, execution_id=None):
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

    # ---- 受控停止（damped_hold：ADR-0008 决策 1；契约见 docs/debug/2026-09-22-damped-hold-spec.md）----
    def damped_hold(self, lease, execution_id=None, *, tilt_limit_deg=None):
        """受控停止：减速到零并**保持站立**（不松力、不退出控制）。

        判定与阈值**全部来自声明**：`stop.duration_s` / `stop.speed_tolerance_mps` /
        `stop.static_hold_s` / `stop.final_window_s`（本适配器的 declaration，即 `config/go2_loopback.yaml`）。

        ⚠ **倾角上限不在这里读**：`config/go2_loopback.yaml` 明写"`quadruped_limits` 只在
        `profiles/safety/quadruped_lab.yaml` 声明一次（避免同一事实两处）" ⇒ 倾角上限由**调用方**
        （技能/策略层，经 `iraf_skills.quadruped.load_movement_limits`）以 `tilt_limit_deg` 传入；
        不传时适配器**只测量并上报** `max_tilt_deg`，不自行判定（不得偷偷内置一个数字）。
        终态（ledger 只允许写一次，见 quadruped.TERMINAL_STATES）：
          · 达标 ⇒ `STOPPED`；超时未达标 ⇒ `FAILED`（带实测量，不得回写成功）；
          · 运行中出现安全事件 ⇒ 松力并写 `SAFETY_STOP`。
        """
        self.require_capability("stop")
        self.require_lease(lease, "stop")
        if execution_id is not None:
            self.require_active_execution(execution_id, lease, capability="stop")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("stop", lease)

        budget_s = float(_dig(self.declaration, "stop.duration_s", "声明"))
        tol_speed = float(_dig(self.declaration, "stop.speed_tolerance_mps", "声明"))
        hold_need_s = float(_dig(self.declaration, "stop.static_hold_s", "声明"))
        final_win_s = float(_dig(self.declaration, "stop.final_window_s", "声明"))
        # 倾角上限**由调用方传入**（`profiles/safety/quadruped_lab.yaml` 是唯一事实来源；
        # `config/go2_loopback.yaml` 明写避免同一事实两处）⇒ 不传则只测量上报、不自行判定。
        tilt_limit = None if tilt_limit_deg is None else float(tilt_limit_deg)
        ramp_s = float(_dig(self.declaration, "stand.ramp_s", "声明"))

        home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
        balance_params = self._balance_parameters()
        # 步态/几何只在**平衡器启用时**才需要解析（避免对无步态声明的本体提要求；
        # 也是"惰性解析"的既有风格：缺段的本体不该因未用到的通路失败）。
        torque_provider = None
        if balance_params["enabled"]:
            params = self._gait_parameters()
            geometry = self._leg_geometry(params)
            trunk_body = gait.trunk_body_id(self.model, self.mujoco, params)
            torque_provider = self._balance_provider(params, geometry, trunk_body,
                                                     balance_params)

        samples = []

        def sample_callback(cycle_index, info):
            self._append_stop_sample(samples, info)

        # 目标位形是**站立 home**（`target=None` 会被 `_run_control` 解成 0 rad 位形，绝不能用）
        cycles, saturated = self._run_control(
            home, budget_s, ramp_s, zero_torque=False,
            torque_provider=torque_provider, sample_callback=sample_callback,
        )

        speeds = [s["base_linear_speed_mps"] for s in samples]
        tilts = [s["tilt_deg"] for s in samples]
        n_final = max(1, int(round(final_win_s * self.control_hz)))
        final_speed = float(np.mean(speeds[-n_final:])) if speeds else float("nan")
        max_tilt = float(max(tilts)) if tilts else float("inf")
        hold_ok = self._continuous_below(speeds, tol_speed, hold_need_s)
        tilt_ok = True if tilt_limit is None else bool(max_tilt <= tilt_limit)
        safety_event = self._safety_event_active(lease)
        ok = bool(hold_ok and tilt_ok)

        report = {
            "simulation": True,
            "capability": "stop",
            "stop_mode": "damped_hold",
            "mode": str(_dig(self.declaration, "stop.mode", "声明")),
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "duration_ms": budget_s * 1000.0,
            "control_cycles": cycles,
            "ctrl_saturated_samples": int(saturated),
            "succeeded": bool(ok and not safety_event),
            "final_speed_mps": final_speed,
            "max_tilt_deg": max_tilt,
            "speed_tolerance_mps": tol_speed,
            "static_hold_s": hold_need_s,
            "final_window_s": final_win_s,
            "tilt_limit_deg": tilt_limit,
            "tilt_limit_source": ("caller" if tilt_limit is not None else "not_provided"),
            "height_start_m": float(samples[0]["base_height_m"]) if samples else None,
            "height_end_m": float(samples[-1]["base_height_m"]) if samples else None,
            "failure_reason": (
                "safety_event" if safety_event else
                "" if ok else
                "超时未达标：末速 %.6f m/s（限 %.6f）｜倾角 %.4f°（限 %s）"
                % (final_speed, tol_speed, max_tilt,
                   "未提供" if tilt_limit is None else "%.1f°" % tilt_limit)
            ),
            "samples": samples,
            "final_state": self.read_state(),
        }
        if safety_event:
            self.ledger.finish(active_id, "SAFETY_STOP", report["failure_reason"])
        elif ok:
            self.ledger.finish(active_id, "STOPPED")
        else:
            self.ledger.finish(active_id, "FAILED", report["failure_reason"])
        return report

    def _append_stop_sample(self, samples, info):
        """采样（字段与 `balance_hold` 一致，便于复用既有复核脚本）。"""
        quat = np.asarray(self.data.qpos[3:7], dtype=float).copy()
        _quat, roll, pitch = self._attitude_terms()
        samples.append({
            "time_s": float(self.data.time),
            "base_height_m": float(self.data.qpos[2]),
            "base_position_xy_m": [float(self.data.qpos[0]), float(self.data.qpos[1])],
            "base_linear_speed_mps": float(np.linalg.norm(
                np.asarray(self.data.qvel, dtype=float)[0:3])),
            "tilt_deg": float(quat_tilt_deg(quat)),
            "roll_deg": float(np.degrees(roll)),
            "pitch_deg": float(np.degrees(pitch)),
            "ctrl_saturated": int(np.count_nonzero(info["saturated"])),
            "tracking_error_rad": float(np.max(np.abs(
                info["desired"] - np.asarray(self.data.qpos[self.qpos_adr], dtype=float)))),
            "ctrl_nonzero": bool(np.any(np.asarray(info["ctrl"], dtype=float) != 0.0)),
        })

    def _continuous_below(self, values, limit, seconds):
        """是否存在**连续** `seconds` 秒的"值 < limit"窗口（样本数按 `self.control_hz` 换算）。"""
        need = max(1, int(round(float(seconds) * float(self.control_hz))))
        run = 0
        for value in values:
            run = run + 1 if float(value) < float(limit) else 0
            if run >= need:
                return True
        return False

    def _safety_event_active(self, lease):
        """安全事件是否正在生效（急停闭锁 / 租约被安全抢占）。

        只读既有状态，不新造语义：查紧急停止闭锁的常见状态接口；都取不到时按"无事件"处理，
        并在测试里用真实接口覆盖（若实际 API 不同，改这里而不改判定语义）。
        """
        estop = getattr(self, "estop", None)
        for name in ("is_triggered", "triggered", "is_latched", "latched", "active"):
            attr = getattr(estop, name, None)
            if attr is not None:
                try:
                    return bool(attr() if callable(attr) else attr)
                except Exception:  # noqa: BLE001
                    return False
        return False

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

    def _feet_trunk_positions(self, geometry, trunk_body):
        """各腿**足端接触几何**在**躯干系**的位置 `{腿: [x, y, z]}`（证据用）。

        落足点规划的验收需要「足端实际落在哪」这个量（步骤 03/04）：计划落点在 `gait` 模块里
        是躯干系坐标 ⇒ 实测也必须换到躯干系才可比（世界系坐标会随机身漂移，量不出落点误差）。
        用接触几何（与接触力判定同一几何）而不是 foot body：两者的位置差是固定偏移，
        接触判定用的就是这个球心。
        """
        rotation = np.asarray(self.data.xmat[int(trunk_body)], dtype=float).reshape(3, 3)
        origin = np.asarray(self.data.xpos[int(trunk_body)], dtype=float)
        return {
            code: [
                float(item)
                for item in rotation.T.dot(
                    np.asarray(self.data.geom_xpos[int(geom["contact_geom"])], dtype=float)
                    - origin
                )
            ]
            for code, geom in geometry.items()
        }

    def _torque_diagnostics(self, q, dq, info, mode):
        """逐关节的**命令（未截断）**或**实际（截断后）**力矩（证据用，步骤 08）。

        为什么必须能取到"未截断"这一档：采样只拿到 `info["ctrl"]`（已按模型 ctrlrange 截断），
        它只能说明"饱和了"，说不出"缺多少" —— 而缺口大小正是判断「目标是否超出执行器能力」与
        「PD 增益/重力前馈是否失配」的关键量。未截断值由与 `_run_control` **同一个**
        `pd_torque_raw` 算出（不存在第二份公式）。
        """
        if mode == "applied":
            return [float(value) for value in np.asarray(info["ctrl"], dtype=float)]
        if mode != "raw":
            raise DeclarationError("力矩诊断模式只支持 raw / applied，实际: %r" % (mode,))
        tau_ff = (
            gravity_bias_torque(self.model, self.data, self.mujoco, self.dof_adr)
            if self.gravity_feedforward
            else np.zeros_like(np.asarray(q, dtype=float))
        )
        raw = pd_torque_raw(
            np.asarray(q, dtype=float), np.asarray(dq, dtype=float),
            np.asarray(info["desired"], dtype=float), self.kp, self.kd, tau_ff,
        )
        return [float(value) for value in np.asarray(raw, dtype=float)]

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

    def _balance_provider(self, params, geometry, trunk_body, balance_params,
                          declared_phase_provider=None):
        """构造力矩级平衡器的**逐控制周期**回调：`(cycle_index, info) -> 12 维附加力矩`。

        支撑腿的判定口径来自声明 `balance.stance_classification`（决策 1e ⇒ **1g**）—— 该段是
        「控制模式 → 判定口径」的映射，本方法只判定**事实**「本次运行是否存在声明相位」，
        不写死任何路径名：

        - 有声明相位（`declared_phase_provider is not None`，步态时钟激活）⇒ 取声明里
          `gait_clock_active` 给出的口径（生产声明 = `declared_and_contact`：**声明相位 ∧ 实测接触**。
          摆动窗口内的腿一律不算支撑腿（不参与 mg 分摊、不承受力控、位置权重回落 `weight_position`）
          ⇒ 它的轨迹才抬得起来。理由（实测，调试记录 §15）：`contact_only` 把「该抬但还没抬」的腿继续
          当支撑腿并给它 ~mg/4 法向力压在台面上，而支撑腿位置权重为 0 ⇒ 轨迹也抬不动它 ⇒ 自锁）。
          相位用的 `elapsed` 与步态目标生成**同一个起点**：本回调首次被调用时的仿真时刻
          （`_run_control` 在步进前回调，故与调用方的 `onset` 相等）。
        - 无声明相位（静态保持等纯位置路径）⇒ 取声明里 `gait_clock_inactive` 给出的口径
          （生产声明 = `contact_only`：只用**实测接触力**，物理接触即支撑）。此时没有摆动窗口，
          声明支撑集 = **全部腿**；1f 的兜底仍然生效（任何腿离地/异常接触持续超过声明上限即显式报错），
          组合游程门禁（决策 1g）亦生效。

        没有足够支撑腿时**不施加**平衡力矩（并计数），超过声明的看门狗上限即停止施加。
        """
        threshold = float(params["verification"]["contact_force_threshold_n"])
        target_height = float(params["verification"]["height_target_m"])
        min_stance = int(balance_params["allocation"]["min_stance_legs"])
        watchdog_limit = int(balance_params["watchdog"]["max_consecutive_no_stance_cycles"])
        # 翻倒阈值只读声明（`gait.verification.fall_base_height_m`，与步态判据用的**同一个**键），
        # 本处不写任何常数；启动窗口 = 看门狗上限 + 1 个周期（同样是声明的派生量）。
        fall_height = float(params["verification"]["fall_base_height_m"])
        startup_window = watchdog_limit + 1
        mass = self.robot_mass_kg()
        gravity = self.gravity_mps2()
        # B1：支撑/摆动两套位置权重（都来自声明，缺键在 `load_balance_declaration` 就失败了）。
        stance_weight = float(balance_params["stance_weight_position"])
        swing_weight = float(balance_params["weight_position"])
        # 支撑集判定口径（决策 1e ⇒ 1g）：**按控制模式**取 —— 控制模式由「是否存在声明相位」这一事实
        # 决定（不是路径名），映射本身住在声明里；取值合法性已在 `load_balance_declaration` 校验过。
        classification_modes = dict(balance_params["stance_classification"])
        declared_phase_source = (
            "gait_clock" if declared_phase_provider is not None else "no_declared_phase"
        )
        stance_classification = str(
            classification_modes[
                "gait_clock_active"
                if declared_phase_provider is not None
                else "gait_clock_inactive"
            ]
        )
        # 生效口径与「由哪条事实选出」写进证据：**生产侧**写入、报告侧只透传（不在报告侧重算，
        # 否则「声明里写 A、实际跑 B」会看不出来）。
        if isinstance(self._balance_stats, dict):
            self._balance_stats["stance_classification"] = stance_classification
            self._balance_stats["declared_phase_source"] = declared_phase_source
        # 决策 1f 的 fail-closed 兜底阈值（全来自声明）：① 摆动窗口内「异常接触力」阈值；
        # ②/③/④ 连续游程上限（两类各一 + 决策 1g 的组合游程），超过即 `stance_integrity_error` 显式报错。
        integrity = balance_params["stance_integrity"]
        swing_force_threshold = float(integrity["swing_contact_force_n"])
        # 「连续不一致」游程计数：不放进 `_balance_stats`（它是过程量，不是证据），
        # 但在越限时把各自的**最大游程**写进证据。
        declared_stance_gap_run = 0
        swing_abnormal_run = 0
        inconsistent_run = 0
        balance_onset = None

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
            nonlocal balance_onset, declared_stance_gap_run, swing_abnormal_run, inconsistent_run
            contact = self._leg_contact_forces(params, geometry)
            measured = [code for code in sorted(geometry) if float(contact[code]) >= threshold]
            stats = self._balance_stats
            stats["cycles"] = int(stats["cycles"]) + 1
            # 相位与步态目标生成共用起点（首次回调时的仿真时刻）。两种口径都算相位：口径只决定
            # 「相位是否参与支撑集筛选」，`declared` 本身是**只读观测量**（第十七轮起用于逐周期取证）。
            if balance_onset is None:
                balance_onset = float(self.data.time)
            elapsed = float(self.data.time) - balance_onset
            if declared_phase_provider is None:
                # 无声明相位（静态保持/纯位置路径）：没有摆动窗口 ⇒ 声明支撑集 = **全部腿**
                # （退化情形，见 `balance.py` 模块 docstring）。这不等于「跳过兜底」：
                # 此时 1f/1g 的兜底只可能报「声明支撑窗内不接触」（悬空/打滑），照样显式报错。
                declared = {code: True for code in sorted(geometry)}
            else:
                declared = {
                    code: bool(value)
                    for code, value in declared_phase_provider(elapsed).items()
                }
            if stance_classification == "declared_and_contact":
                stance = [code for code in measured if declared[code]]
            else:
                stance = list(measured)
            # ---- 决策 1f 的 fail-closed 兜底（2026-09-21 19:05 授权落地，只增证据）----
            # **两种控制模式都跑**（决策 1g：口径按控制模式区分，但兜底不随口径消失）。
            # 声明摆动却仍接触的腿：正是自锁的观测对象（不参与分摊、不承受力控）。
            swing_in_contact = [code for code in measured if not declared[code]]
            if swing_in_contact:
                stats["declared_swing_in_contact_cycles"] = (
                    int(stats["declared_swing_in_contact_cycles"]) + 1
                )
            stats["declared_swing_in_contact_samples"] = (
                int(stats["declared_swing_in_contact_samples"]) + len(swing_in_contact)
            )
            # ② 声明支撑窗内实测不接触（悬空/打滑）：**不得当作摆动腿** ⇒ 逐周期计数 + 连续游程。
            gap_legs = [
                code
                for code in sorted(geometry)
                if declared[code] and float(contact[code]) < threshold
            ]
            if gap_legs:
                stats["declared_stance_without_contact_cycles"] = (
                    int(stats["declared_stance_without_contact_cycles"]) + 1
                )
                stats["declared_stance_without_contact_samples"] = (
                    int(stats["declared_stance_without_contact_samples"]) + len(gap_legs)
                )
                declared_stance_gap_run += 1
                stats["max_declared_stance_gap_run"] = max(
                    int(stats["max_declared_stance_gap_run"]), declared_stance_gap_run
                )
                stats["last_declared_stance_without_contact"] = list(gap_legs)
            else:
                declared_stance_gap_run = 0
            # ① 摆动窗口内**异常**接触力（> 声明阈值）：记录峰值，并在连续越限时显式报错。
            abnormal = [
                code
                for code in swing_in_contact
                if float(contact[code]) > swing_force_threshold
            ]
            if abnormal:
                stats["swing_abnormal_contact_cycles"] = (
                    int(stats["swing_abnormal_contact_cycles"]) + 1
                )
                stats["swing_abnormal_contact_samples"] = (
                    int(stats["swing_abnormal_contact_samples"]) + len(abnormal)
                )
                swing_abnormal_run += 1
                stats["max_swing_abnormal_run"] = max(
                    int(stats["max_swing_abnormal_run"]), swing_abnormal_run
                )
            else:
                swing_abnormal_run = 0
            for code in swing_in_contact:
                stats["swing_max_contact_n"] = max(
                    float(stats["swing_max_contact_n"]), float(contact[code])
                )
            # ④ 决策 1g 的**组合**游程：本周期至少有一条腿声明与实测不符（两类中任意一类）。
            # 单类游程会被两类交替重置（gap → swing → gap …），本计数不会 ⇒ 它拦的是
            # 「持续处于不一致状态」这种单类门禁看不见的形态。
            inconsistent_run = inconsistent_run + 1 if (gap_legs or abnormal) else 0
            if gap_legs or abnormal:
                stats["inconsistent_cycles"] = int(stats["inconsistent_cycles"]) + 1
                stats["max_inconsistent_run"] = max(
                    int(stats["max_inconsistent_run"]), inconsistent_run
                )
            # 显式报错：越限时给出中文原因（不是只记数），非越限时严格 `None`（= 未发生）。
            # **粘性**写入：游程越限是「发生过」的事实，报告不得因为末周期恰好恢复正常就遗忘。
            # 报告侧（`verify_go2_gait_in_place._balance_segment`）按 **max 游程**复算 `overdue`，
            # 粘性写入才与它等价；非粘性写法只在「末周期仍越限」时偶然一致（潜在恒假字段）。
            integrity_error = balance_module.evaluate_stance_integrity(
                integrity,
                declared_stance_gap_cycles=declared_stance_gap_run,
                swing_abnormal_contact_cycles=swing_abnormal_run,
                inconsistent_run_cycles=inconsistent_run,
            )
            if integrity_error is not None and stats["stance_integrity_error"] is None:
                stats["stance_integrity_error"] = integrity_error
                # 归因证据：**首次**越限的控制周期（与看门狗 `watchdog_trigger_cycle` 同一口径）。
                stats["stance_integrity_error_cycle"] = int(stats["cycles"])
            # ---- 翻倒窗口 / 启动窗口归因（第十七轮：只增证据，不参与任何判据）----
            # 分母口径必须显式：全采样窗口把翻倒之后在空中/翻滚的周期也算进去（§18.4 同族问题）。
            falling = float(self.data.qpos[2]) < fall_height
            if falling:
                if stats["fall_cycle"] is None:
                    stats["fall_cycle"] = int(stats["cycles"])
            else:
                stats["pre_fall_cycles"] = int(stats["pre_fall_cycles"]) + 1
            # 「本周期会不会走力控」在这里就已确定：`len(stance) >= min_stance` 且未被看门狗 latch。
            # （latch 只可能在 `len(stance) < min_stance` 的分支里被置起，故此处读到的值即进入分支后的值。）
            use_force_control = bool(len(stance) >= min_stance) and not bool(
                stats["watchdog_triggered"]
            )
            if not falling:
                key = (
                    "force_control_cycles_pre_fall"
                    if use_force_control
                    else "fallback_cycles_pre_fall"
                )
                stats[key] = int(stats[key]) + 1
            stats["stance_legs_histogram_all_cycles"][len(stance)] = (
                int(stats["stance_legs_histogram_all_cycles"].get(len(stance), 0)) + 1
            )
            if int(stats["cycles"]) <= startup_window:
                stats["startup_trace"].append(
                    {
                        "cycle": int(stats["cycles"]),
                        "stance_legs": len(stance),
                        "measured_contact": list(measured),
                        "declared_stance": [code for code in sorted(geometry) if declared[code]],
                        "stance": list(stance),
                        "use_force_control": use_force_control,
                        "height_m": float(self.data.qpos[2]),
                    }
                )
            if len(stance) < min_stance:
                stats["no_stance_cycles"] = int(stats["no_stance_cycles"]) + 1
                stats["consecutive_no_stance"] = int(stats["consecutive_no_stance"]) + 1
                stats["max_consecutive_no_stance"] = max(
                    int(stats["max_consecutive_no_stance"]),
                    int(stats["consecutive_no_stance"]),
                )
                if int(stats["consecutive_no_stance"]) > watchdog_limit:
                    if not stats["watchdog_triggered"]:
                        # 归因证据：看门狗**首次**触发的控制周期（从 1 起数）。
                        stats["watchdog_trigger_cycle"] = int(stats["cycles"])
                    stats["watchdog_triggered"] = True
                    stats["disabled_cycles"] = int(stats["disabled_cycles"]) + 1
                return _no_stance_fallback()
            stats["consecutive_no_stance"] = 0
            if stats["watchdog_triggered"]:
                # 看门狗一旦触发即保持关闭：不得在“站不住”的状态下继续施加力矩级动作
                # （否则平衡器自己成为第二个不受监控的控制源）。
                stats["disabled_cycles"] = int(stats["disabled_cycles"]) + 1
                return _no_stance_fallback()
            if stats["first_stance_cycle"] is None:
                # 归因证据：**首次真正进入力控**的控制周期（从 1 起数）。用于把「启动瞬态凑不齐支撑集」
                # 与「结构性凑不齐」分开 —— 只看 `no_stance_cycles` 总量无法区分这两者。
                stats["first_stance_cycle"] = int(stats["cycles"])
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
            # 其余（摆动腿）取 `weight_position`。支撑集口径由声明
            # `balance.stance_classification` 决定（`contact_only` = 只按实测接触；
            # `declared_and_contact` = 声明相位 ∧ 实测接触），本处不做第二份推断。
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
            # 看门狗归因（只增证据、不参与判据）：触发发生在**第几个控制周期**、以及首次凑齐支撑集的周期。
            # `None` 的语义是「未发生」（未触发 / 从未凑齐），不是「未测量」；与 `watchdog_triggered`
            # 的自洽关系由验收入口 `_balance_segment` 强制（触发 ⇔ 周期非 None）。
            "watchdog_trigger_cycle": None,
            "first_stance_cycle": None,
            "max_consecutive_no_stance": 0,
            "stance_legs_histogram": {},
            # 翻倒窗口归因（2026-09-21 第十七轮，**只增证据、不参与判据、不引入阈值**）：
            # `stance_legs_histogram` 只在**真正进入力控**的分支里累加，因此它的读法是「力控周期的支撑集
            # 直方图」；而「B1 的参与率」需要一个**分母口径明确**的窗口 —— 全采样窗口会把翻倒之后
            # 在空中/翻滚的周期也算进分母（§18.4 同族口径问题）。这里同时给出：
            # · `stance_legs_histogram_all_cycles` = **全部周期**的支撑集直方图（口径与上面那个不同）；
            # · `fall_cycle` / `pre_fall_cycles` = 按 `gait.verification.fall_base_height_m` 切出的翻倒前窗口；
            # · `force_control_cycles_pre_fall` / `fallback_cycles_pre_fall` = 该窗口内的兑现/退化周期数；
            # · `startup_trace` = 启动窗口（`watchdog.max_consecutive_no_stance_cycles + 1` 个周期，
            #   边界直接来自声明）的**逐周期**取证，用来回答「为什么启动段连续几个周期凑不齐支撑集」。
            # `fall_cycle` 的 `None` 语义 = 「未发生（未翻倒）」，不是「未测量」。
            "fall_cycle": None,
            "pre_fall_cycles": 0,
            "force_control_cycles_pre_fall": 0,
            "fallback_cycles_pre_fall": 0,
            "stance_legs_histogram_all_cycles": {},
            "startup_trace": [],
            "clamped_legs": [],
            "last_stance_legs": [],
            "last_wrench": None,
            "max_abs_torque_nm": 0.0,
            # 支撑集判定口径（决策 1e ⇒ 1g）与自锁的观测计数：判定「摆动腿到底抬没抬起来」的证据。
            # `stance_classification` = **本次运行实际生效**的口径（由声明里对应控制模式的键选出，
            # 由 provider 在构造时写入）；`declared_phase_source` = 选出它的事实依据。声明原文
            # （控制模式 → 口径的映射）在 `_balance_summary` 的顶层字段里，两者并列才可审计。
            "stance_classification": None,
            "declared_phase_source": None,
            "declared_swing_in_contact_cycles": 0,
            "declared_swing_in_contact_samples": 0,
            # 决策 1f 的 fail-closed 兜底（2026-09-21 19:05 授权落地，只增证据、不参与既有判据）：
            # ① 摆动窗口内**异常**接触力（> `stance_integrity.swing_contact_force_n`）；
            # ② 声明支撑窗内实测不接触（悬空/打滑，**不得当作摆动腿**）。
            # `stance_integrity_error` 的 `None` 语义 = 「未越限（未发生）」，不是「未测量」；
            # 非 `None` 时是**中文错误说明**（显式报错，不是只置一个布尔），且为**粘性**（一旦发生过
            # 就不再回到 `None`，与报告侧按 max 游程复算 `overdue` 等价）。
            "declared_stance_without_contact_cycles": 0,
            "declared_stance_without_contact_samples": 0,
            "max_declared_stance_gap_run": 0,
            "last_declared_stance_without_contact": [],
            "swing_abnormal_contact_cycles": 0,
            "swing_abnormal_contact_samples": 0,
            "max_swing_abnormal_run": 0,
            "swing_max_contact_n": 0.0,
            # 决策 1g 的**组合**游程（两类交替也会持续累加）：`inconsistent_cycles` = 逐周期计数，
            # `max_inconsistent_run` = 最长连续游程；`stance_integrity_error_cycle` 的 `None` = 未越限。
            "inconsistent_cycles": 0,
            "max_inconsistent_run": 0,
            "stance_integrity_error": None,
            "stance_integrity_error_cycle": None,
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
            # 决策 1g：口径按**控制模式**给出 ⇒ 报告里必须同时给出「声明原文（映射）」与
            # 「本次运行实际生效的口径 + 由哪条事实选出」。`stance_classification` 保持"生效口径"
            # 这一语义（`None` = 本次运行**未产生统计**（平衡器关闭/未跑），与 `fall_cycle` 的
            # `None` 同一约定：是「未发生」，不是「未测量」）；声明原文在
            # `stance_classification_modes`，两者并列才看得出「声明里写 A、实际跑 B」。
            "stance_classification": (
                None
                if stats["stance_classification"] is None
                else str(stats["stance_classification"])
            ),
            "stance_classification_modes": dict(balance_params["stance_classification"]),
            "declared_phase_source": (
                None
                if stats["declared_phase_source"] is None
                else str(stats["declared_phase_source"])
            ),
            # 决策 1f 的兜底阈值原样透传（判据/证据里的数字必须能在声明里逐字找到）。
            "stance_integrity": dict(balance_params["stance_integrity"]),
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
                "watchdog_trigger_cycle": (
                    None
                    if stats["watchdog_trigger_cycle"] is None
                    else int(stats["watchdog_trigger_cycle"])
                ),
                "first_stance_cycle": (
                    None if stats["first_stance_cycle"] is None else int(stats["first_stance_cycle"])
                ),
                "max_consecutive_no_stance": int(stats["max_consecutive_no_stance"]),
                "disabled_cycles": int(stats["disabled_cycles"]),
                # 翻倒窗口归因（口径显式）：`fall_cycle` 的 `None` = 未翻倒（不是未测量）。
                "fall_cycle": (
                    None if stats["fall_cycle"] is None else int(stats["fall_cycle"])
                ),
                "pre_fall_cycles": int(stats["pre_fall_cycles"]),
                "force_control_cycles_pre_fall": int(stats["force_control_cycles_pre_fall"]),
                "fallback_cycles_pre_fall": int(stats["fallback_cycles_pre_fall"]),
                "position_weight": dict(stats["position_weight"]),
                "stance_legs_histogram": {
                    str(key): int(value) for key, value in stats["stance_legs_histogram"].items()
                },
                # 与上一键**口径不同**：这个是全部周期（含翻倒后）的支撑集直方图，上面那个只含力控周期。
                "stance_legs_histogram_all_cycles": {
                    str(key): int(value)
                    for key, value in stats["stance_legs_histogram_all_cycles"].items()
                },
                "startup_trace": [dict(item) for item in stats["startup_trace"]],
                "clamped_legs": list(stats["clamped_legs"]),
                "declared_swing_in_contact_cycles": int(
                    stats["declared_swing_in_contact_cycles"]
                ),
                "declared_swing_in_contact_samples": int(
                    stats["declared_swing_in_contact_samples"]
                ),
                # 决策 1f 兜底证据（`stance_integrity_error` 的非 `None` = 显式报错，原样透传中文原因）。
                "declared_stance_without_contact_cycles": int(
                    stats["declared_stance_without_contact_cycles"]
                ),
                "declared_stance_without_contact_samples": int(
                    stats["declared_stance_without_contact_samples"]
                ),
                "max_declared_stance_gap_run": int(stats["max_declared_stance_gap_run"]),
                "last_declared_stance_without_contact": list(
                    stats["last_declared_stance_without_contact"]
                ),
                "swing_abnormal_contact_cycles": int(stats["swing_abnormal_contact_cycles"]),
                "swing_abnormal_contact_samples": int(
                    stats["swing_abnormal_contact_samples"]
                ),
                "max_swing_abnormal_run": int(stats["max_swing_abnormal_run"]),
                "swing_max_contact_n": float(stats["swing_max_contact_n"]),
                # 决策 1g 的组合游程（两类交替也持续累加）+ 首次越限周期（`None` = 从未越限）。
                "inconsistent_cycles": int(stats["inconsistent_cycles"]),
                "max_inconsistent_run": int(stats["max_inconsistent_run"]),
                "stance_integrity_error": (
                    None if stats["stance_integrity_error"] is None else str(stats["stance_integrity_error"])
                ),
                "stance_integrity_error_cycle": (
                    None
                    if stats["stance_integrity_error_cycle"] is None
                    else int(stats["stance_integrity_error_cycle"])
                ),
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
        # 决策 1g：本路径**没有声明相位**（静态保持，无步态足端轨迹/无摆动窗口）⇒ 不传
        # `declared_phase_provider`，支撑集口径由声明里 `gait_clock_inactive` 那一项决定
        # （生产声明 = `contact_only`：物理接触即支撑）。这与「有声明相位」是**两条不同的控制模式**，
        # 不是同一口径的兜底 —— 把步态相位套到静态保持上正是 1f 引入的回归（实测 116.04° 翻倒）。
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
        # 决策 1g：本路径**有声明相位**（步态时钟激活）⇒ 支撑集口径由声明里 `gait_clock_active`
        # 那一项决定（生产声明 = `declared_and_contact`）。这里只提供「声明相位」这一**事实**，
        # 不决定口径；相位与步态目标生成共用 `_balance_provider` 的 `elapsed`（首次回调时的仿真
        # 时刻，与下方 `onset` 相等）⇒ 与步骤 02 的相位口径逐位一致。
        def declared_phase(elapsed):
            return {
                code: bool(gait.is_stance(params, gait.leg_phase(params, code, elapsed)))
                for code in sorted(geometry)
            }

        torque_provider = (
            self._balance_provider(params, geometry, trunk_body, balance_params,
                                   declared_phase_provider=declared_phase)
            if balance_params["enabled"]
            else None
        )

        # 低频位置项（契约 docs/debug/2026-09-23-go2-gait-drift.md §9）：维护机身相对**起点**的
        # 水平位移的滑动平均（窗口来自声明 `gait.stabilization.position_window_s`，必须 > 1 个
        # 步态周期 ⇒ 不含逐相位 sway）。仅当声明开启且增益 > 0 时该项才实际生效
        # （`gait.position_offset` 内判 0），因此关闭档与既有实现**逐位一致**。
        from collections import deque

        position_window_s = float(params["stabilization"]["position_window_s"])
        drift_samples = deque()
        drift_origin = {"xy": None}

        def mean_drift_body(trunk):
            xy = np.asarray(self.data.qpos[0:2], dtype=float).copy()
            if drift_origin["xy"] is None:
                drift_origin["xy"] = xy          # 起点即"原点"（首次调用时确定，不写数字）
            rotation = np.asarray(self.data.xmat[int(trunk)], dtype=float).reshape(3, 3)
            delta_world = xy - drift_origin["xy"]
            delta_body = rotation.T.dot(np.array([delta_world[0], delta_world[1], 0.0]))
            now = float(self.data.time)
            drift_samples.append((now, float(delta_body[0]), float(delta_body[1])))
            while drift_samples and (now - drift_samples[0][0]) > position_window_s:
                drift_samples.popleft()
            if not drift_samples:
                return (0.0, 0.0)
            count = float(len(drift_samples))
            return (sum(item[1] for item in drift_samples) / count,
                    sum(item[2] for item in drift_samples) / count)

        def target_provider(cycle_index, now):
            elapsed = now - onset
            amplitude = gait.amplitude_at(params, elapsed)
            velocity = self._body_frame_velocity(trunk_body)
            omega = self._body_frame_omega(trunk_body)
            targets = gait.gait_joint_targets(
                params, geometry, home, limits, elapsed, amplitude, velocity, omega,
                body_mean_drift_xy=mean_drift_body(trunk_body),
            )
            return np.array([targets[joint] for joint in self.joint_order], dtype=float)

        def sample_callback(cycle_index, info):
            q = np.asarray(self.data.qpos[self.qpos_adr], dtype=float)
            dq = np.asarray(self.data.qvel[self.dof_adr], dtype=float)
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
                    # 足端（接触几何）在躯干系的位置：落足点验收用它量「实际落到哪」，
                    # 与计划落点（躯干系）同坐标系可比（步骤 03/04 的判据来源）。
                    "foot_trunk_m": self._feet_trunk_positions(geometry, trunk_body),
                    # 力矩诊断（步骤 08）：命令（**未截断**）/ 实际（截断后）/ 逐关节误差。
                    # 采样只能拿到截断后的 `info["ctrl"]`，只它无法回答"缺多少"。
                    "torque_raw_nm": self._torque_diagnostics(q, dq, info, "raw"),
                    "torque_applied_nm": self._torque_diagnostics(q, dq, info, "applied"),
                    "joint_error_rad": [
                        float(value) for value in
                        (np.asarray(info["desired"], dtype=float) - q)
                    ],
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
        """速度指令 → MPC Provider（独立进程）→ 足端力 → 关节支撑力矩（A6a-④ ⑤）。

        契约：`docs/debug/2026-09-23-locomote-provider-integration-spec.md` §2/§4/§6。
        数据流（每拍）：`plan_fn()` 组 QP 请求（状态→参考→接触表→动力学→装配→协议编码）
        → `ProviderRuntime.step()`（50 Hz 更新、100 Hz 消费，契约 §3）
          ├─ 可用解 ⇒ 解的第一拍足端力 → `τ = −Jᵀ·f`（**实测符号**：`build/iraf-a6a4/jt_sign_probe.py`
          │   τ=−Jᵀf ⇒ Δh=+0.001714 m、四腿法向合力 116.025 N；τ=+Jᵀf ⇒ 把足端卸掉）
          │   → B1 载荷（逐关节 `position_weight`：支撑腿取声明 `stance_weight_position`、
          │   摆动腿取 `weight_position`）⇒ `_run_control` 按**既有**混合与 ctrlrange 截断执行（本文件不改）
          └─ 不可用/超时/过期/急停 ⇒ `MpcUnavailableError` ⇒ 按契约 §4：
             `damped_hold`（移动中停住）或 `torque_zero_release`（急停 ⇒ 松力 + SAFETY_STOP）

        与能力面的关系：本方法**不声明能力**（Profile 的 `capabilities` 仍是 [stand, stop]）；
        能力回填要等本机判据（`config/go2_locomote.yaml` 的 ② 硬判据）与 aarch64 板复测都过（铁律 6.8）。
        初始条件：场景构建期已按声明做 `initial_alignment`（关键帧即落在支撑面上，无 18.372 mm 穿透）。
        """
        from iraf_adapters.unitree.mpc import deployment as mpc_deployment
        from iraf_adapters.unitree.mpc import plan as mpc_plan
        from iraf_adapters.unitree.mpc import process_client as mpc_client
        from iraf_adapters.unitree.mpc import state_bridge as mpc_state
        from iraf_adapters.unitree.mpc import torque_hook as mpc_hook
        from iraf_adapters.unitree.mpc import torque_provider as mpc_torque_provider
        from iraf_adapters.unitree.mpc.contact import LEG_ORDER as MPC_LEG_ORDER
        from iraf_adapters.unitree.mpc.gait_trot import merge_trot_declaration
        from iraf_adapters.unitree.mpc.provider_runtime import ProviderRuntime
        from iraf_adapters.unitree.mpc.torque_hook import MpcUnavailableError

        self.estop.assert_clear()
        self.require_lease(lease, "locomote")
        if execution_id is not None:
            self.require_active_execution(execution_id, lease, capability="locomote")
            active_id = str(execution_id)
        else:
            active_id = self.begin_execution("locomote", lease)
        resolved = self.resolve_velocity(velocity)          # 形状/字段/有限性（未知字段一律拒绝）

        section = self.declaration.get("locomote")
        if not isinstance(section, dict):
            raise DeclarationError("声明缺少 locomote 段（Provider 配置与默认时长必须来自声明）")
        missing = [key for key in ("provider_config", "duration_seconds",
                                   "stance_position_weight", "swing_position_weight",
                                   "velocity_ramp_s", "leg_position_gain_scale")
                   if key not in section]
        if missing:
            raise DeclarationError("声明缺少 locomote 的键: %s" % missing)
        provider_config = Path(str(section["provider_config"]))
        if not provider_config.is_absolute():
            provider_config = repo_root() / provider_config
        if not provider_config.is_file():
            raise DeclarationError("locomote.provider_config 不存在: %s" % provider_config)
        provider_doc = yaml.safe_load(provider_config.read_text(encoding="utf-8"))
        deployment = mpc_deployment.load_deployment(provider_doc["mpc_provider"], self.control_hz)
        mpc_model = dict(provider_doc["mpc_model"])
        # QP 水平/偏航参考的来源（见 `mpc_model.horizontal_reference_from` 声明注释）：
        # 按通道声明，取值只允许 command / measured；缺键、缺子键、非法取值一律显式失败
        # （不在实现层给默认值 —— 默认值会让「声明漏了」伪装成成功）。
        reference_from = mpc_model.get("horizontal_reference_from")
        if not isinstance(reference_from, dict):
            raise DeclarationError("mpc_model.horizontal_reference_from 必须是映射，实际: %r"
                                   % (reference_from,))
        for channel in ("translation", "yaw"):
            if channel not in reference_from:
                raise DeclarationError(
                    "mpc_model.horizontal_reference_from 缺少子键: %s（白名单: translation/yaw）"
                    % channel)
            value = str(reference_from[channel])
            if value not in ("command", "measured"):
                raise DeclarationError(
                    "mpc_model.horizontal_reference_from.%s 只支持 ['command', 'measured']，实际: %r"
                    % (channel, reference_from[channel]))
        unknown = [key for key in sorted(reference_from) if key not in ("translation", "yaw")]
        if unknown:
            raise DeclarationError(
                "mpc_model.horizontal_reference_from 含未知子键 %s（白名单: translation/yaw）" % unknown)
        translation_from = str(reference_from["translation"])
        yaw_from = str(reference_from["yaw"])

        # trot 步态声明：与定位/接触表/落足点共用同一份（移植清单 §1.1：不得造第二份事实来源）
        trot = gait.load_gait_declaration(
            merge_trot_declaration(self.declaration, provider_doc["mpc_gait"]), self.profile.joints
        )

        geometry = self._leg_geometry(trot)
        trunk_body = gait.trunk_body_id(self.model, self.mujoco, trot)
        home = {joint: float(self.profile.home[joint]) for joint in self.joint_order}
        limits = {joint: self.profile.joint_limits[joint] for joint in self.joint_order}
        # 位置级权重**取自本路径自己的声明**（2026-09-23 实测修正，见 config 里 `locomote` 段注释）：
        # 不用 `balance.stance_weight_position`（=0）——那个 0 会把支撑腿的位置环整段关掉，
        # 而 MPC 路径没有平衡器那层兜底 ⇒ 腿构型没人守 ⇒ 倾角级联（实测 46.02°）。
        stance_weight = float(section["stance_position_weight"])
        swing_weight = float(section["swing_position_weight"])
        for label, value in (("stance_position_weight", stance_weight),
                             ("swing_position_weight", swing_weight)):
            if not 0.0 <= value <= 1.0:
                raise DeclarationError("locomote.%s 必须落在 [0,1]，实际 %r" % (label, value))
        # B1 权重的支撑集口径 = **声明相位 ∧ 实测接触**（与平衡路径 `declared_and_contact` 同一规则；
        # 2026-09-21 调试记录 §15/§22 的实测：只按声明会把「该抬未抬」的腿当支撑腿 ⇒ 给它零位置权重
        # 又不给它力控 ⇒ 被压在地面/翻倒）。阈值同样来自步态声明的 verification 段，不写数字。
        contact_threshold = float(trot["verification"]["contact_force_threshold_n"])
        # 速度指令斜坡（见 config 里 `locomote.velocity_ramp_s` 的实测依据）：阶跃指令会在第一拍
        # 让 QP 产生巨大的反向水平力（实测 Σfx=+74.803 N vs 匀速态 +0.495 N），故按斜坡给进。
        velocity_ramp_s = float(section["velocity_ramp_s"])
        if velocity_ramp_s < 0.0:
            raise DeclarationError("locomote.velocity_ramp_s 必须 ≥ 0，实际 %r" % velocity_ramp_s)
        # 本路径 PD 增益缩放（见下方 `saved_gains` 处的实测依据）：必须 > 0，1.0 = 与 stand 同档。
        leg_position_gain_scale = float(section["leg_position_gain_scale"])
        if not leg_position_gain_scale > 0.0:
            raise DeclarationError(
                "locomote.leg_position_gain_scale 必须 > 0，实际 %r" % (leg_position_gain_scale,))
        # `resolve_duration_ms(duration_ms, declared_seconds)` **返回秒**，且缺省值也是秒
        # （quadruped.py:608）；因此这里不再换算、声明键也用 `duration_seconds`。
        seconds = self.resolve_duration_ms(duration_ms, float(section["duration_seconds"]))
        horizon = int(mpc_model["horizon"])
        z_des = float(mpc_model["stand_height_m"])

        mass, _com0, inertia0, _bodies = mpc_state.subtree_mass_inertia(
            self.model, self.data, self.mujoco, trunk_body
        )
        tracker = mpc_state.ComStateTracker()
        # 髋关节相对躯干的偏移（躯干系）—— QP 用它把足端力映射到质心/机身。
        # ⚠ 必须是**当拍实测**，不能只算一次：早先实现在进入循环前算一次就冻结，
        # 机器人一动、腿一摆，QP 的接触几何就与实物不一致 ⇒ 系统性力误差（实测四腿切向力
        # 顶在摩擦锥角点：forward (+19.17,+19.17,47.93)、mu·fz = 19.17）。
        # 每拍重算的成本是 4 次 `mj_name2id`（装配期可缓存 joint_id）+ 一次旋转乘法，
        # 相对 50 Hz 的 QP 求解可忽略。
        hip_joint_bodies = {}
        for code in sorted(geometry):
            joint_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT,
                                              geometry[code]["joints"]["hip_joint"])
            hip_joint_bodies[code] = int(self.model.jnt_bodyid[joint_id])

        def hip_offsets_now():
            rotation = np.asarray(self.data.xmat[int(trunk_body)], dtype=float).reshape(3, 3)
            trunk_pos = np.asarray(self.data.xpos[int(trunk_body)], dtype=float)
            return {code: rotation.T.dot(np.asarray(self.data.xpos[body], dtype=float) - trunk_pos)
                    for code, body in hip_joint_bodies.items()}

        state_holder = {"pos_des_world": None, "t0": None}
        samples = []
        client = mpc_client.MpcProcessClient(deployment.worker_command(sys.executable),
                                             timeout_ms=deployment.call_timeout_ms)
        runtime = ProviderRuntime(client, ticks_per_update=deployment.ticks_per_update)
        index_map = {
            code: tuple(self.joint_order.index(geometry[code]["joints"][key])
                        for key in ("hip_joint", "thigh_joint", "calf_joint"))
            for code in sorted(geometry)
        }
        onset = float(self.data.time)

        def state_vector_now():
            rotation = np.asarray(self.data.xmat[int(trunk_body)], dtype=float).reshape(3, 3)
            return mpc_state.com_state_vector(
                np.asarray(self.data.subtree_com[int(trunk_body)], dtype=float),
                np.asarray(self.data.qpos[3:7], dtype=float),
                np.asarray(self.data.qvel[0:3], dtype=float),
                np.asarray(self.data.qvel[3:6], dtype=float), tracker, rotation=rotation,
            )

        def ramped_command(elapsed):
            """指令斜坡（**单一来源**）：QP 参考与步态支撑足退让项必须消费同一份值。

            依据（2026-09-23 实测）：阶跃指令会让 QP 第 0 拍给出 Σfx=+74.803 N（匀速态只需
            +0.495 N），这段反向冲量造成恒定后漂。`velocity_ramp_s` 来自 `locomote` 段声明。
            """
            ramp = (1.0 if velocity_ramp_s <= 0.0
                    else min(1.0, max(float(elapsed), 0.0) / velocity_ramp_s))
            return (ramp * resolved["vx_mps"], ramp * resolved["vy_mps"],
                    ramp * resolved["wz_rad_s"])

        def plan_fn():
            """本拍 QP 请求（只在更新拍被调用；同一份接触表用于 QP 与 B1 权重口径）。"""
            state_now = state_vector_now()
            elapsed = float(self.data.time) - onset
            vx_cmd, vy_cmd, wz_cmd = ramped_command(elapsed)
            if state_holder["pos_des_world"] is None:
                state_holder["pos_des_world"] = np.array([state_now[0], state_now[1], z_des])
            mass_now, _c, inertia_now, _b = mpc_state.subtree_mass_inertia(
                self.model, self.data, self.mujoco, trunk_body
            )
            body_velocity = self._body_frame_velocity(trunk_body)
            body_omega = self._body_frame_omega(trunk_body)
            # 平移通道（位置 + 速度）按声明取参考：`measured` ⇒ 该通道零误差（交给足端退让），
            # `command` ⇒ QP 用接触力把机身加速到指令速度。偏航通道独立声明（见配置注释的实测：
            # 两者都放开给 `measured` 时后退 4 s 偏航 +30.51°，偏航必须留在 MPC 闭环里）。
            if translation_from == "measured":
                vx_ref, vy_ref = float(body_velocity[0]), float(body_velocity[1])
                pos_des = np.array([state_now[0], state_now[1], z_des], dtype=float)
            else:
                vx_ref, vy_ref = vx_cmd, vy_cmd
                pos_des = state_holder["pos_des_world"]
            wz_ref = float(body_omega[2]) if yaw_from == "measured" else wz_cmd
            request, context = mpc_plan.build_mpc_request(
                mpc_model, trot, com_state=state_now, mass=mass_now,
                inertia_com_world=inertia_now, hip_offsets=hip_offsets_now(),
                body_velocity_body=body_velocity,
                pos_des_world=pos_des,
                command={"vx_body": vx_ref, "vy_body": vy_ref,
                         "yaw_rate": wz_ref, "z_des": z_des},
                t0=float(self.data.time) - onset, return_context=True,
            )
            # 参考位置的**钳位结果**必须跨拍保留（上游既有语义：`pos_des_world` 是有状态量）
            state_holder["pos_des_world"] = np.array(context["pos_des_world_out"], dtype=float)
            # MPC **参考姿态**（`x_ref` **行序** p(3)→rpy(3)→v(3)→ω(3)、列=视界 ⇒ rpy 在第 0 列取 [3:6, 0]）：
            # 用于判定「俯仰偏置是 QP 自己要的，还是 plant 到不了」。⚠ 该布局坑在 reference.py 里被
            # 明文警告过（(12,N) 不是 (N,12)），此处按行序取，避免静默错位。
            try:
                x_ref = np.asarray(context["x_ref"], dtype=float)
                state_holder["ref_rpy_deg"] = [float(np.degrees(v)) for v in x_ref[3:6, 0]]
            except (KeyError, IndexError, TypeError, ValueError):
                state_holder["ref_rpy_deg"] = None
            # B1 权重的支撑集 = 声明相位（QP 接触表第一列，与 QP 同一份表）**∧ 实测接触**
            declared = {code: bool(context["contact_table"][index, 0])
                        for index, code in enumerate(MPC_LEG_ORDER)}
            measured_forces = self._leg_contact_forces(trot, geometry)
            mask = {code: bool(declared[code] and measured_forces[code] >= contact_threshold)
                    for code in declared}
            state_holder["mask"] = mask
            return {"request": request, "current_mask": mask}

        def jacobians_now():
            out = {}
            for code in sorted(geometry):
                joints = [geometry[code]["joints"][key]
                          for key in ("hip_joint", "thigh_joint", "calf_joint")]
                dofs = [self.bindings[joint]["dof_adr"] for joint in joints]
                foot_world = np.asarray(self.data.xpos[int(geometry[code]["foot_body"])],
                                        dtype=float)
                # ⚠ 契约要 **Jᵀ**（`joint_torques` 直接做 `M @ f`），`_foot_jacobian` 给的是 J
                # ⇒ 必须经 `jacobian_transpose_from` 转置。漏转置只会把**水平**方向搞反
                # （实测依据见 `mpc/torque_provider.jacobian_transpose_from` 的 docstring：
                # x/y 轴 thigh/hip 反号、z 轴同号 ⇒ 竖直"看着正常"）。
                out[code] = mpc_torque_provider.jacobian_transpose_from(
                    self._foot_jacobian(dofs)(foot_world, geometry[code]["foot_body"]),
                    label="jacobian[%s]" % code,
                )
            return out

        hook = mpc_hook.MpcTorqueHook(
            runtime, plan_fn, index_map, stance_weight, swing_weight, jacobians_now, horizon,
            emergency_fn=lambda: bool(self.estop.snapshot().get("latched")),
            timeout_ms=deployment.call_timeout_ms,
        )
        # 热身（实测需要）：OSQP 冷启要付 setup 成本（实测首拍 **291.7 ms** > 声明的
        # `call_timeout_ms=200 ms` ⇒ 被拒且被 kill 重启 ⇒ 新进程又冷 ⇒ 无限超时循环，
        # 实测 client `calls=4/timeouts=4/restarts=4`）；热态只要 ~26 ms（solve_ms 0.42~0.52、iter 10）。
        # ⚠ 因此热身**必须用比控制回路更宽的预算**：`call_timeout_ms` 是"控制拍预算"（超了就该停），
        # 而冷启动 setup 是**一次性运行成本**，不属于控制拍。本预算与显示路径的租约 TTL 余量同性质：
        # 属**运行时安全余量**（不是声明事实），故写在这里并记录在报告里，而不是塞进控制声明。
        warmup_timeout_ms = max(float(deployment.call_timeout_ms), 2000.0)
        warmup = {"attempts": [], "warm": False, "timeout_ms": warmup_timeout_ms}
        for _ in range(3):
            warm_response, warm_error = client.call(plan_fn()["request"],
                                                    timeout_ms=warmup_timeout_ms)
            warmup["attempts"].append({"error": warm_error,
                                       "solve_ms": warm_response.get("solve_ms"),
                                       "status_class": warm_response.get("status_class")})
            if warm_error is None and warm_response.get("decision") == "ok":
                warmup["warm"] = True
                break

        # 世界系锚定（`gait.walk.anchor: measured_pose`）所需的**每腿触地世界位**：
        # 触地那一拍记下「当前机身位姿下的中立足端」的世界坐标，支撑相内保持不变
        # （足端在世界系不动 ⇒ 不打滑；摆动相把目标插值回当前机身位姿下的中立位）。
        walk_anchors = {code: {"stance": None, "foot_world": None} for code in geometry}
        walk_duty = float(trot["duty_factor"])

        def walk_pose_measured(elapsed):
            """实测机身位姿 + 各腿触地世界位（仅 `measured_pose` 档需要）。"""
            state_now = state_vector_now()
            body_x = float(state_now[0])
            body_y = float(state_now[1])
            yaw = float(state_now[5])
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            pose = {}
            for code in geometry:
                rx = float(geometry[code]["trunk_rel_m"][0])
                ry = float(geometry[code]["trunk_rel_m"][1])
                neutral_x = body_x + cos_yaw * rx - sin_yaw * ry
                neutral_y = body_y + sin_yaw * rx + cos_yaw * ry
                entry = walk_anchors[code]
                stance = (gait.leg_phase(trot, code, elapsed) % 1.0) < walk_duty
                if stance and entry["stance"] is not True:
                    # 触地：锚点 = 这一拍的中立足端世界位（此后支撑相内不再动）。
                    entry["foot_world"] = (neutral_x, neutral_y)
                entry["stance"] = stance
                pose[code] = {"body_xy_m": (body_x, body_y), "body_yaw_rad": yaw,
                              "stance_foot_world_m": entry["foot_world"]}
            return pose

        def target_provider(cycle_index, now):
            """摆动/支撑的关节形状目标（MPC 只提供接触力矩；形状仍由既有目标生成给出）。

            `walk_command_mps_rad_s` 是与 QP **同一份**斜坡指令；`walk_pose` 是实测机身位姿
            （`anchor: measured_pose` 档用：漂移/速度指令都由它换算，见 `gait.walk_foot_offset_m`）。
            """
            elapsed = now - onset
            state_holder["elapsed_s"] = elapsed          # 采样对齐步态相位用（同一拍）
            vx_cmd, vy_cmd, wz_cmd = ramped_command(elapsed)
            # 退让项的步幅增益（`gait.walk.stride_scale`）：只放大**足端退让**的给进速率，
            # QP 的参考仍用未放大的指令（它跟踪的是真实期望速度，不许被放大）。
            walk_scale = float((trot.get("walk") or {}).get("stride_scale") or 1.0)
            targets = gait.gait_joint_targets(
                trot, geometry, home, limits, elapsed, gait.amplitude_at(trot, elapsed),
                self._body_frame_velocity(trunk_body), self._body_frame_omega(trunk_body),
                walk_command_mps_rad_s=(walk_scale * vx_cmd, walk_scale * vy_cmd,
                                        walk_scale * wz_cmd),
                walk_pose=(walk_pose_measured(elapsed)
                           if (trot.get("walk") or {}).get("anchor") == "measured_pose" else None),
            )
            return np.array([targets[joint] for joint in self.joint_order], dtype=float)

        def sample_callback(cycle_index, info):
            quat = np.asarray(self.data.qpos[3:7], dtype=float)
            w, qx, qy, qz = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
            yaw_deg = float(np.degrees(np.arctan2(2.0 * (w * qz + qx * qy),
                                                  1.0 - 2.0 * (qy * qy + qz * qz))))
            samples.append({
                "time_s": float(self.data.time),
                "base_position_xy_m": [float(self.data.qpos[0]), float(self.data.qpos[1])],
                "base_height_m": float(self.data.qpos[2]),
                "base_linear_speed_mps": float(np.linalg.norm(self.data.qvel[0:3])),
                # 偏航与偏航角速度：转向工况的判据量（与位移同一次采样，不另开测量路径）
                "base_yaw_deg": yaw_deg,
                "base_yaw_rate_rad_s": float(self._body_frame_omega(trunk_body)[2]),
                "tilt_deg": float(quat_tilt_deg(quat)),
                # 俯仰/滚转分开记 + 逐腿实测接触法向力 + 步态已运行时间：
                # 2026-09-23 整流效应分析用（竖直抬放 + 支撑交替 + 机身姿态 ⇒ 每周期净冲量）。
                "body_roll_deg": float(np.degrees(np.arctan2(
                    2.0 * (w * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))),
                "body_pitch_deg": float(np.degrees(np.arcsin(
                    max(-1.0, min(1.0, 2.0 * (w * qy - qz * qx)))))),
                "leg_force_n": {code: float(value) for code, value
                                in self._leg_contact_forces(trot, geometry).items()},
                "gait_elapsed_s": float(state_holder.get("elapsed_s") or -1.0),
                "mpc_ref_rpy_deg": state_holder.get("ref_rpy_deg"),
                # 诊断量：本拍 MPC 载荷经 ctrlrange 截断后的执行力矩峰值 + 本拍支撑集
                # （用于定位"从第几拍开始失控"，不参与任何判据）
                "max_abs_ctrl_nm": float(np.max(np.abs(np.asarray(info["ctrl"], dtype=float)))),

                # 位置环（混合前，含重力前馈）与 MPC 载荷的**拆分**（诊断）：
                # 只有拿到这两项才能回答「QP 的命令有多少被位置环抵消」（§1.8 的 95% 抵消）。
                "max_abs_ctrl_position_nm": float(np.max(np.abs(
                    np.asarray(info.get("ctrl_position_nm", info["ctrl"]), dtype=float)))),
                "ctrl_position_sum_abs_nm": float(np.sum(np.abs(
                    np.asarray(info.get("ctrl_position_nm", info["ctrl"]), dtype=float)))),
                "ctrl_sum_abs_nm": float(np.sum(np.abs(np.asarray(info["ctrl"], dtype=float)))),
                # 逐关节向量（末拍可读；用于看清**哪个关节**被位置环占满）：
                "ctrl_nm": [float(v) for v in np.asarray(info["ctrl"], dtype=float)],
                "ctrl_position_nm": [float(v) for v in np.asarray(
                    info.get("ctrl_position_nm", info["ctrl"]), dtype=float)],
                # 位置环的**跟踪误差**（rad）：τ_pd 的 kp·Δq 项直接由它决定。
                # 静态站立时髋力矩需求 ~23.7 N·m ⇒ kp=150 反推 Δq ≈ 0.158 rad ≈ 9°；
                # 到底是「目标本来就偏」还是「追不上」，只有把 Δq 打出来才能判。
                "track_err_rad": [float(v) for v in
                                  (np.asarray(info["desired"], dtype=float)
                                   - np.asarray(info.get("q", info["desired"]), dtype=float))],
                "max_track_err_rad": float(np.max(np.abs(
                    np.asarray(info["desired"], dtype=float)
                    - np.asarray(info.get("q", info["desired"]), dtype=float)))),
                "stance_legs": sorted(code for code, flag
                                      in (state_holder.get("mask") or {}).items() if flag),
                # 饱和前的 MPC 原始载荷峰值 / 足端力峰值 / 四腿法向力合计（诊断，不参与判据）
                "mpc_payload_max_nm": hook.stats.get("last_payload_max_nm"),
                "mpc_forces_max_n": hook.stats.get("last_forces_max_n"),
                "mpc_forces_sum_z_n": hook.stats.get("last_forces_sum_z_n"),
                "mpc_solve_ms": hook.stats.get("last_solve_ms"),
                "mpc_status_class": hook.stats.get("last_status_class"),
                "declared_velocity": {"vx_mps": resolved["vx_mps"], "vy_mps": resolved["vy_mps"],
                                      "wz_rad_s": resolved["wz_rad_s"]},
                "ctrl_saturated": int(np.count_nonzero(info["saturated"])),
            })

        failure = None
        terminal = "SUCCEEDED"
        cycles = 0
        saturated = 0
        # 本路径的 PD 增益缩放（声明键 `locomote.leg_position_gain_scale`）：
        # `control.kp_nm_per_rad = 150 / kd = 4` 是按**静态站立**指标选出来的（见声明注释的扫描）；
        # 在 trot 3 Hz 摆动目标下实测跟踪误差 Δq 达 0.10~0.26 rad ⇒ `kp·Δq` 直接撞到每关节力矩上限
        # （逐关节实测：髋/大腿 ±23.7、小腿 −45.4 全部饱和）⇒ B1 的 MPC 载荷被截断吃掉
        # （零指令 hold：|τ总|max = |τ位置|max = 45.43 N·m，而 MPC 载荷只有 6~16 N·m）。
        # 只在本执行内有效（`finally` 恢复），不影响 stand/stop 两条既有路径。
        saved_gains = (np.array(self.kp, dtype=float, copy=True),
                       np.array(self.kd, dtype=float, copy=True))
        self.kp = saved_gains[0] * leg_position_gain_scale
        self.kd = saved_gains[1] * leg_position_gain_scale
        try:
            with client:
                cycles, saturated = self._run_control(
                    None, seconds, float(trot["ramp_s"]), target_provider=target_provider,
                    torque_provider=hook, sample_callback=sample_callback,
                )
        except MpcUnavailableError as exc:
            # 契约 §4：不可用/超时/过期 ⇒ 移动中停住（damped_hold）；急停 ⇒ 松力 + SAFETY_STOP
            failure = {"decision": exc.decision, "reason": exc.reason,
                       "diagnostics": dict(exc.diagnostics)}
            if exc.decision == "torque_zero_release":
                self._apply_zero_torque()
                terminal = "SAFETY_STOP"
            else:
                try:
                    hold = self.damped_hold(lease, tilt_limit_deg=None)
                    terminal = "STOPPED" if hold.get("static_entered") else "FAILED"
                except Exception as hold_exc:  # noqa: BLE001 —— 停机失败必须显式落到 FAILED
                    failure["damped_hold_error"] = "%s: %s" % (type(hold_exc).__name__, hold_exc)
                    terminal = "FAILED"
            self.ledger.finish(active_id, terminal, failure["reason"])
        else:
            self.ledger.finish(active_id, terminal)
        finally:
            # 恢复 stand/stop 路径的增益（本路径的缩放只在本执行内有效）
            self.kp, self.kd = saved_gains

        report = {
            "simulation": True,
            "capability": "locomote",
            "path": "mpc_provider",
            "execution_id": active_id,
            "fencing_token": int(lease.fencing_token),
            "control_source_owner": str(getattr(lease, "owner", "")),
            "command": dict(resolved),
            "duration_ms": seconds * 1000.0,
            "control_cycles": int(cycles),
            "ctrl_saturated_samples": int(saturated),
            "deployment": deployment.as_dict(),
            "mpc_model": {"horizon": horizon, "gait_hz": mpc_model["gait_hz"],
                          "z_des_m": z_des,
                          "horizontal_reference_from": dict(reference_from)},
            "position_weight": {"stance": stance_weight, "swing": swing_weight,
                                "leg_position_gain_scale": leg_position_gain_scale},
            "warmup": warmup,
            "provider": hook.summary(),
            "client": dict(client.stats),
            "failure": failure,
            "samples": samples,
        }
        return report

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
                ctrl_position_nm = ctrl.copy()          # 零力矩档：位置环输出也是零
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
                # 混合**前**的位置环输出（含重力前馈）：诊断用。B1 混合是
                # `ctrl = position_weight ⊙ τ_pd + w_bal ⊙ τ_mpc`，光看混合后的总量无法回答
                # 「QP 的命令有多少被位置环抵消」（实测零指令下 QP 下令水平合力 +37.2 N，
                # 机身只得到 1~2 N ⇒ 95% 以上被抵消，见 docs/debug 的 zero-command-drift §1.8）。
                ctrl_position_nm = ctrl.copy()
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
                    {"desired": np.asarray(desired, dtype=float), "ctrl": ctrl, "saturated": saturated,
                     "ctrl_position_nm": np.asarray(ctrl_position_nm, dtype=float),
                     "q": np.asarray(q, dtype=float), "dq": np.asarray(dq, dtype=float)},
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
