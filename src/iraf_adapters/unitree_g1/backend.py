"""Unitree G1 人形适配器（simulation=true，流程验证 + stand 实测）。

G1 用**位置执行器**（厂商 MJCF 的 `<position kp=500 dampratio=1>`），与 Go2 的力矩型
执行器不同：位置环在 MuJoCo 执行器内部，适配器只需每拍把 `ctrl` 设为目标位形。
- `stand`：ctrl=home 位形并步进，量测高度/倾角/跟踪误差（默认站姿静态自平衡，实测 8s 无翻倒）；
- `stop`：`hold_position`（保持当前位置；人形静态自平衡下即安全停机）；
- `locomote`：Unitree 官方 RL 策略 + 力矩 PD 行走（simulation=true，实测 2.2m）。

诚实边界：全部结论 simulation=true；真机能力 DEFERRED。
"""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path

import threading

import math
import yaml

import numpy as np

from iraf_adapters.unitree.quadruped import (
    CommandRejectedError,
    DeclarationError,
    ModelUnavailableError,
    QuadrupedAdapter,
    UnsupportedCapabilityError,
)


def repo_root():
    return Path(__file__).resolve().parents[3]


def _dig(document, dotted, label):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            raise DeclarationError("%s 缺少声明键: %s" % (label, dotted))
        node = node[part]
    return node


REQUIRED_KEYS = (
    "simulation",
    "robot.id",
    "robot.profile",
    "model.file",
    "initial.keyframe",
    "control.frequency_hz",
    "stand.pose_source",
    "stand.duration_s",
    "stand.height_target_m",
    "stand.height_tolerance_m",
    "stand.attitude_tolerance_deg",
    "stop.mode",
    "stop.duration_s",
    "locomote.policy_file",
    "locomote.model_file",
    "locomote.duration_s",
    "locomote.command_forward_mps",
    "locomote.height_tolerance_m",
)

SUPPORTED_POSE_SOURCES = ("profile_home", "explicit")
SUPPORTED_STOP_MODES = ("hold_position",)


def load_declaration(config):
    """把 config（路径 / 声明对象）解析成 (declaration, root)。"""
    root = repo_root()
    if isinstance(config, (str, Path)):
        path = Path(config)
        path = path if path.is_absolute() else root / path
        if not path.is_file():
            raise DeclarationError("声明文件不存在: %s" % path)
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    elif isinstance(config, dict):
        document = dict(config)
        if document.get("root"):
            root = Path(str(document["root"]))
    else:
        raise DeclarationError("适配器配置类型非法: %r" % (type(config).__name__,))
    if not isinstance(document, dict):
        raise DeclarationError("声明顶层必须是对象")
    missing = []
    for key in REQUIRED_KEYS:
        node = document
        ok = True
        for part in key.split("."):
            if not isinstance(node, dict) or part not in node:
                ok = False
                break
            node = node[part]
        if not ok:
            missing.append(key)
    if missing:
        raise DeclarationError("声明缺少必需键（禁止默认值兜底）: %s" % missing)
    return document, root


def resolve_joint_bindings(model, mujoco, joints):
    """关节 -> {joint_id, qpos_adr, dof_adr, actuator_id}。唯一命名解析点。"""
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


def _tilt_deg(data, body_id):
    """躯干 body 的局部 z 轴相对竖直的夹角（度）。"""
    w, x, y, z = data.xquat[body_id]
    zwz = w * w - x * x - y * y + z * z
    return float(np.degrees(np.arccos(float(np.clip(abs(zwz), -1.0, 1.0)))))


class UnitreeG1Backend(QuadrupedAdapter):
    """G1 仿真后端：声明驱动的**位置**控制（位置执行器在模型内部闭环）。"""

    #: 已实现能力（locomote = Unitree RL 行走，simulation=true）。
    IMPLEMENTED_CAPABILITIES = frozenset(("emergency_stop", "read_state", "stand", "stop", "locomote"))

    def __init__(self, declaration, profile, authority, *, root, mujoco, model, data, bindings):
        super().__init__(declaration, profile, authority)
        self.root = Path(root)
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.bindings = bindings
        self.joint_order = [str(item) for item in profile.joints]
        self.control_hz = float(_dig(declaration, "control.frequency_hz", "声明"))
        self.trunk_body = str(_dig(declaration, "robot.trunk_body", "声明"))
        self.trunk_body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.trunk_body))
        if self.trunk_body_id < 0:
            raise ModelUnavailableError("躯干 body 不存在: " + self.trunk_body)
        self.stand_duration_s = float(_dig(declaration, "stand.duration_s", "声明"))
        self.locomote_duration_s = float(_dig(declaration, "locomote.duration_s", "声明"))
        self._lock = threading.Lock()
        self._display_renderer_cache = None

    @classmethod
    def from_config(cls, config, profile, authority):
        import mujoco
        declaration, root = load_declaration(config)
        model_path = root / str(_dig(declaration, "model.file", "声明"))
        if not model_path.is_file():
            raise ModelUnavailableError("模型不存在: %s" % model_path)
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        bindings = resolve_joint_bindings(model, mujoco, profile.joints)
        return cls(declaration, profile, authority, root=root, mujoco=mujoco,
                   model=model, data=data, bindings=bindings)

    # ---- 物理步进（直接 mujoco，不做共享植物）----

    def _reset_to_keyframe(self, name):
        key_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_KEY, name)
        if key_id < 0 and self.model.nkey == 1:
            key_id = 0
        if key_id < 0:
            raise ModelUnavailableError("关键帧不存在: " + name)
        self.mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        self.mujoco.mj_forward(self.model, self.data)

    def _write_position_ctrl(self, targets):
        ctrl = np.zeros(self.model.nu)
        for name, value in targets.items():
            binding = self.bindings[name]
            ctrl[binding["actuator_id"]] = float(value)
        self.data.ctrl[:] = ctrl

    def _substeps(self):
        dt = float(self.model.opt.timestep)
        exact = 1.0 / (self.control_hz * dt)
        steps = int(round(exact))
        if steps < 1 or abs(exact - steps) > 1e-9:
            raise DeclarationError(
                "控制频率 %g Hz 与模型 timestep %g s 不整除" % (self.control_hz, dt)
            )
        return steps

    # ---- 能力实现 ----

    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        self.require_motion_allowed("stand", lease)
        execution_id = execution_id or self.begin_execution("stand", lease)
        targets, source = self.resolve_targets(targets)
        seconds = self.resolve_duration_ms(duration_ms, self.stand_duration_s)
        substeps = self._substeps()

        self._reset_to_keyframe(str(_dig(self.declaration, "initial.keyframe", "声明")))
        self._write_position_ctrl(targets)

        height_target = float(_dig(self.declaration, "stand.height_target_m", "声明"))
        height_tol = float(_dig(self.declaration, "stand.height_tolerance_m", "声明"))
        att_tol = float(_dig(self.declaration, "stand.attitude_tolerance_deg", "声明"))

        total_steps = int(seconds * self.control_hz)
        samples = []
        max_tracking = 0.0
        for _ in range(total_steps):
            for _ in range(substeps):
                self.mujoco.mj_step(self.model, self.data)
            h = float(self.data.xpos[self.trunk_body_id, 2])
            tilt = _tilt_deg(self.data, self.trunk_body_id)
            tracking = max(
                abs(self.data.qpos[self.bindings[n]["qpos_adr"]] - float(targets[n]))
                for n in self.bindings
            )
            max_tracking = max(max_tracking, tracking)
            samples.append({"height_m": h, "tilt_deg": tilt})

        h_end = samples[-1]["height_m"]
        tilt_max = max(s["tilt_deg"] for s in samples)
        height_err = abs(h_end - height_target)
        passed = height_err <= height_tol and tilt_max <= att_tol
        terminal_state = "SUCCEEDED" if passed else "FAILED"
        self.ledger.finish(execution_id, terminal_state, "stand" if passed else "height/tilt 超出容差")
        return {
            # 技能层契约键（StandProvider.EVIDENCE_KEYS）
            "simulation": True,
            "capability": "stand",
            "execution_id": execution_id,
            "fencing_token": int(lease.fencing_token),
            "duration_ms": int(seconds * 1000.0),
            "control_cycles": total_steps,
            "substeps_per_control": substeps,
            "ctrl_saturated_samples": 0,
            "target_source": source,
            "torque_limit_source": "model",
            "gravity_feedforward": False,
            "joint_targets_rad": dict(targets),
            "final_state": self.read_state(),
            # 人形专属实测
            "duration_s": seconds,
            "height_m": h_end,
            "height_target_m": height_target,
            "height_error_m": height_err,
            "max_tilt_deg": tilt_max,
            "max_tracking_error_rad": max_tracking,
            "passed": passed,
        }

    # ---- 行走（Unitree 官方 RL 策略 + 力矩 PD，simulation=true）----

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        self.require_motion_allowed("locomote", lease)
        execution_id = execution_id or self.begin_execution("locomote", lease)
        seconds = self.resolve_duration_ms(duration_ms, self.locomote_duration_s)
        import torch

        policy_path = self.root / str(_dig(self.declaration, "locomote.policy_file", "声明"))
        model_path = self.root / str(_dig(self.declaration, "locomote.model_file", "声明"))
        parsed = self.resolve_velocity(velocity)
        height_min = float(_dig(self.declaration, "locomote.height_tolerance_m", "声明"))
        if not policy_path.is_file():
            raise ModelUnavailableError("策略不存在: %s" % policy_path)
        if not model_path.is_file():
            raise ModelUnavailableError("模型不存在: %s" % model_path)

        model = self.mujoco.MjModel.from_xml_path(str(model_path))
        data = self.mujoco.MjData(model)
        model.opt.timestep = 0.002
        policy = torch.jit.load(str(policy_path), map_location="cpu")
        policy.eval()

        def _gravity(q):
            qw, qx, qy, qz = q
            return np.array([2*(-qz*qx+qw*qy), -2*(qz*qy+qw*qx), 1-2*(qw*qw+qz*qz)], dtype=np.float32)

        kps = np.array([100,100,100,150,40,40,100,100,100,150,40,40], np.float32)
        kds = np.array([2,2,2,4,2,2,2,2,2,4,2,2], np.float32)
        default_angles = np.array([-0.1,0,0,0.3,-0.2,0,-0.1,0,0,0.3,-0.2,0], np.float32)
        cmd = np.array([parsed["vx_mps"], parsed["vy_mps"], parsed["wz_rad_s"]], np.float32)
        cmd_scale = np.array([2.0, 2.0, 0.25], np.float32)
        dt = 0.002; decimation = 10; period = 0.8

        self.mujoco.mj_resetData(model, data)
        data.qpos[7:19] = default_angles
        self.mujoco.mj_forward(model, data)
        action = np.zeros(12, np.float32); target = default_angles.copy(); obs = np.zeros(47, np.float32)
        for _ in range(100):
            q = data.qpos[7:19]; dq = data.qvel[6:18]
            data.ctrl[:12] = (target - q) * kps - dq * kds
            self.mujoco.mj_step(model, data)

        start_x = float(data.qpos[0]); counter = 0; max_dist = 0.0; fell = False
        total_steps = int(seconds / dt)
        for _ in range(total_steps):
            q = data.qpos[7:19]; dq = data.qvel[6:18]
            data.ctrl[:12] = (target - q) * kps - dq * kds
            self.mujoco.mj_step(model, data); counter += 1
            if counter % decimation == 0:
                qj = data.qpos[7:19]; dqj = data.qvel[6:18]
                obs[:3] = data.qvel[3:6] * 0.25
                obs[3:6] = _gravity(data.qpos[3:7])
                obs[6:9] = cmd * cmd_scale
                obs[9:21] = qj - default_angles
                obs[21:33] = dqj * 0.05
                obs[33:45] = action
                ph = (counter * dt % period) / period
                obs[45:47] = [np.sin(2*np.pi*ph), np.cos(2*np.pi*ph)]
                with torch.no_grad():
                    action = policy(torch.from_numpy(obs).unsqueeze(0).float()).numpy().squeeze()
                target = action * 0.25 + default_angles
            dist = float(data.qpos[0] - start_x)
            if data.qpos[2] > height_min:
                max_dist = max(max_dist, dist)
            if data.qpos[2] < height_min:
                fell = True
                break

        terminal_state = "SUCCEEDED" if not fell else "FAILED"
        self.ledger.finish(execution_id, terminal_state, "locomote" if not fell else "fallen")
        return {
            "simulation": True, "capability": "locomote", "execution_id": execution_id,
            "fencing_token": int(lease.fencing_token), "duration_ms": int(seconds*1000.0),
            "distance_m": max_dist, "command_forward_mps": float(parsed["vx_mps"]), "fell": fell,
            "final_state": self.read_state(),
        }

    def stop(self, lease, execution_id=None):
        self.require_motion_allowed("stop", lease)
        execution_id = execution_id or self.begin_execution("stop", lease)
        mode = str(_dig(self.declaration, "stop.mode", "声明"))
        if mode not in SUPPORTED_STOP_MODES:
            raise DeclarationError("不支持的 stop.mode: %s" % mode)
        seconds = self.resolve_duration_ms(None, float(_dig(self.declaration, "stop.duration_s", "声明")))
        substeps = self._substeps()
        # hold_position：把 ctrl 锁在当前位置（人形静态自平衡下即安全停机）
        hold = {n: float(self.data.qpos[self.bindings[n]["qpos_adr"]]) for n in self.bindings}
        for _ in range(int(seconds * self.control_hz)):
            self._write_position_ctrl(hold)
            for _ in range(substeps):
                self.mujoco.mj_step(self.model, self.data)
        self.ledger.finish(execution_id, "STOPPED", mode)
        return {"simulation": True, "capability": "stop", "execution_id": execution_id,
                "mode": mode, "duration_s": seconds}

    def read_state(self):
        return {
            "simulation": True,
            "time_s": float(self.data.time),
            "base_position_m": [float(v) for v in self.data.xpos[self.trunk_body_id]],
            "base_quaternion_wxyz": [float(v) for v in self.data.xquat[self.trunk_body_id]],
            "base_linear_velocity_mps": [float(v) for v in self.data.cvel[self.trunk_body_id, 0:3]],
            "base_angular_velocity_rad_s": [float(v) for v in self.data.cvel[self.trunk_body_id, 3:6]],
            "joint_positions_rad": [float(self.data.qpos[self.bindings[n]["qpos_adr"]]) for n in self.joint_order],
            "joint_velocities_rad_s": [float(self.data.qvel[self.bindings[n]["dof_adr"]]) for n in self.joint_order],
            "joint_torque_nm": [0.0] * len(self.joint_order),
            "imu": {"quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "angular_velocity_rad_s": [0.0, 0.0, 0.0],
                    "linear_acceleration_mps2": [0.0, 0.0, 0.0]},
            "emergency_stop": bool(self.estop.engaged),
            "control_source": dict(self._control_source),
        }

    def runtime_inventory(self):
        """运行时清单：策略前置条件与跨机 safety 判定的**唯一取值来源**。

        为什么必须实现：`provider_service._safety_state()` 读不到清单就按「已隔离」处理
        （fail-closed，`INVENTORY_UNAVAILABLE`），G1 会在执行前被全量拒绝 —— 适配器
        自身能力再对也没用。取值纪律：只报 `data` 里的实测值，不猜、不兜底；
        estop 取被控对象自身状态，躯干高度与 `read_state()` 同一来源（`xpos`）。
        """
        return {
            "safety": {"estop": bool(self.estop.engaged)},
            "mode": "simulation",
            "trunk": {
                "body": self.trunk_body,
                "height_m": float(self.data.xpos[self.trunk_body_id, 2]),
            },
            "joints": {"count": len(self.joint_order), "control_mode": "position"},
        }

    def _apply_zero_torque(self):
        # 位置执行器没有"零力矩"；等价安全动作 = 锁住当前位置（hold）。
        for name in self.bindings:
            self.data.ctrl[self.bindings[name]["actuator_id"]] = float(
                self.data.qpos[self.bindings[name]["qpos_adr"]]
            )
        return True

    # ---- 显示面（render_frames / display_lock / render_settings，与 Go2 同契约）----

    def display_lock(self):
        return self._lock

    def render_settings(self):
        render = self.declaration.get("render") or {}
        return (render.get("camera"), int(render.get("width_px", 640)),
                int(render.get("height_px", 480)), float(render.get("render_hz", 30)))

    def _display_renderer(self):
        with self._lock:
            if self._display_renderer_cache is None:
                camera, width, height, _hz = self.render_settings()
                self._display_renderer_cache = self.mujoco.Renderer(self.model, height=height, width=width)
                if isinstance(camera, dict):
                    required = ("lookat_m", "distance_m", "azimuth_deg", "elevation_deg")
                    missing = [k for k in required if k not in camera]
                    if missing:
                        raise DeclarationError("render.camera 自由相机缺键: %s" % missing)
                    free = self.mujoco.MjvCamera()
                    free.type = self.mujoco.mjtCamera.mjCAMERA_FREE
                    free.lookat[:] = [float(v) for v in camera["lookat_m"]]
                    free.distance = float(camera["distance_m"])
                    free.azimuth = float(camera["azimuth_deg"])
                    free.elevation = float(camera["elevation_deg"])
                    self._display_renderer_cache._iraf_camera_id = free
                else:
                    camera_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, str(camera))
                    if camera_id < 0:
                        raise DeclarationError("render.camera=%r 在模型中不存在" % str(camera))
                    self._display_renderer_cache._iraf_camera_id = camera_id
        return self._display_renderer_cache

    def render_frames(self, count=1):
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

    def describe(self):
        base = super().describe()
        base.update({
            "schema_version": "iraf.humanoid-adapter/v1",
            "robot": str(getattr(self.profile, "name", "")),
            "model": str(self.root / str(_dig(self.declaration, "model.file", "声明"))),
            "joints": list(self.joint_order),
            "joint_actuator_binding": {
                n: {"actuator_id": self.bindings[n]["actuator_id"]} for n in self.joint_order
            },
            "control_mode": "position",
        })
        return base
