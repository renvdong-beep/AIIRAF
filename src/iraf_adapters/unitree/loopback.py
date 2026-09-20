"""Go2 MuJoCo loopback 实现层（步骤 15）：站立 / 停止 / 状态读取。

分层（与 `scene_builder.py` / `scene_sensor_evidence.py` 一致）：
本模块是**实现层**（可导入、可逐项断言），CLI 入口是 `scripts/verify_go2_loopback.py`
（只解析参数、映射退出码）。

纪律
----
- 所有数字来自声明：`config/go2_loopback.yaml`（本步骤）与
  `profiles/unitree_go2_mujoco.yaml`（关节身份 + `spec.home` 站立位形）。
  实现层另有**必需键元组** `REQUIRED_KEYS`（缺键 -> 退出码 2），模块内不含任何机型数字默认值。
- 关节名 ≠ 执行器名（`go2.xml`：关节 `FL_hip_joint` / 执行器 `FL_hip`），
  映射只在 `resolve_binding()` 一处解析，所有读写都走它。
- 判据与**独立计算的模型量**比对（陀螺仪 vs `mj_objectVelocity`、力矩传感器 vs 施加的 ctrl、
  高度/姿态/速度 vs 声明阈值），不是与硬编码常数比。

诚实边界
--------
- 本模块的全部结论都属于**仿真**（报告带 `simulation: true`）；
  目标端/真机验收不在本模块职责内（板卡不在场 → DEFERRED）。
- 站立 ≠ 步态/导航/停靠能力；`stop.mode=torque_zero_release` 是"松力停机"，
  力矩型执行器松力后失能躺倒是预期结果（报告里显式给 `collapsed` 字段）。
"""

import datetime
import hashlib
import json
import math
from pathlib import Path

import numpy as np

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_MODEL = 4
EXIT_FAILED = 5

REPORT_SCHEMA_VERSION = "iraf.quadruped-loopback/v1"
REPORT_KIND = "go2_loopback"

SUPPORTED_POSE_SOURCES = ("profile_home", "explicit")
SUPPORTED_STOP_MODES = ("torque_zero_release",)
SUPPORTED_TORQUE_LIMIT_SOURCES = ("model",)

# 必需键（点号路径）。声明层 schema 之外的第二道门禁：缺键必须是**干净的退出码 2**，
# 而不是运行到一半抛 KeyError（步骤 14 的教训）。
REQUIRED_KEYS = (
    "schema_version",
    "simulation",
    "robot.id",
    "robot.profile",
    "model.file",
    "model.builder",
    "initial.keyframe",
    "control.frequency_hz",
    "control.kp_nm_per_rad",
    "control.kd_nm_s_per_rad",
    "control.gravity_feedforward",
    "control.torque_limit_source",
    "stand.pose_source",
    "stand.ramp_s",
    "stand.duration_s",
    "stand.hold_s",
    "stand.height_target_m",
    "stand.height_tolerance_m",
    "stand.height_std_max_m",
    "stand.attitude_tolerance_deg",
    "stand.speed_tolerance_mps",
    "stand.max_tracking_error_rad",
    "stop.mode",
    "stop.duration_s",
    "stop.speed_tolerance_mps",
    "stop.static_hold_s",
    "stop.final_window_s",
    "sample.frequency_hz",
    "state.sensors.quat",
    "state.sensors.gyro",
    "state.sensors.acc",
    "state.sensors.torque_pattern",
    "state.quat_norm_tolerance",
    "state.gyro_max_abs_error_rad_s",
    "state.torque_max_abs_error_nm",
    "report.path",
)


class LoopbackError(ValueError):
    """带退出码的实现层错误（入口层按 `code` 映射退出码）。"""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


def _fail(code, message):
    raise LoopbackError(code, message)


def repo_root():
    return Path(__file__).resolve().parents[3]


def _repo_relative(value, label):
    """写盘路径必须是仓库相对路径（绝对路径 / 越界路径显式失败）。"""
    text = str(value)
    path = Path(text)
    if path.is_absolute() or text.startswith("~"):
        _fail(EXIT_DECLARATION, "%s 必须是仓库相对路径，实际是: %s" % (label, text))
    if ".." in path.parts:
        _fail(EXIT_DECLARATION, "%s 不得越出仓库根: %s" % (label, text))
    return path


def _load_yaml(path, label):
    import yaml

    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - 解析失败即声明非法
        _fail(EXIT_DECLARATION, "%s 无法解析: %s（%s）" % (label, path, exc))
    if not isinstance(document, dict) or not document:
        _fail(EXIT_DECLARATION, "%s 必须是映射: %s" % (label, path))
    return document


def _get(document, dotted, label):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            _fail(EXIT_DECLARATION, "%s 缺少必需键 %s" % (label, dotted))
        node = node[part]
    return node


def _number(value, label, *, positive=True, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(EXIT_DECLARATION, "%s 必须是数字，实际是 %r" % (label, value))
    number = float(value)
    if math.isnan(number) or math.isinf(number):
        _fail(EXIT_DECLARATION, "%s 必须是有限数字，实际是 %r" % (label, value))
    if positive and number < 0.0:
        _fail(EXIT_DECLARATION, "%s 必须为非负数，实际是 %r" % (label, value))
    if positive and not allow_zero and number == 0.0:
        _fail(EXIT_DECLARATION, "%s 必须为正数，实际是 %r" % (label, value))
    return number


def validate_declaration(document, joints, label="config/go2_loopback.yaml"):
    """校验声明（必需键 + 取值 + 自洽性）。缺键/非法值 -> 退出码 2。"""
    for key in REQUIRED_KEYS:
        _get(document, key, label)
    if str(document.get("schema_version")) != "iraf.quadruped-loopback/v1":
        _fail(EXIT_DECLARATION, "%s.schema_version 必须是 iraf.quadruped-loopback/v1，实际 %r"
              % (label, document.get("schema_version")))
    if document.get("simulation") is not True:
        _fail(EXIT_DECLARATION, "%s.simulation 必须显式为 true（AGENTS.md 1.7）" % label)

    joints = [str(item) for item in joints]
    if not joints:
        _fail(EXIT_DECLARATION, "Profile 的 spec.joints 为空：无法确定受控关节")

    control_hz = _number(_get(document, "control.frequency_hz", label), "control.frequency_hz")
    sample_hz = _number(_get(document, "sample.frequency_hz", label), "sample.frequency_hz")
    if sample_hz > control_hz:
        _fail(EXIT_DECLARATION, "sample.frequency_hz（%g）不得高于 control.frequency_hz（%g）"
              % (sample_hz, control_hz))
    ratio = control_hz / sample_hz
    if abs(ratio - round(ratio)) > 1e-9:
        _fail(EXIT_DECLARATION, "control.frequency_hz 必须是 sample.frequency_hz 的整数倍（%g / %g）"
              % (control_hz, sample_hz))

    _number(_get(document, "control.kp_nm_per_rad", label), "control.kp_nm_per_rad")
    _number(_get(document, "control.kd_nm_s_per_rad", label), "control.kd_nm_s_per_rad", allow_zero=True)
    if _get(document, "control.gravity_feedforward", label) not in (True, False):
        _fail(EXIT_DECLARATION, "control.gravity_feedforward 必须是布尔值")
    limit_source = str(_get(document, "control.torque_limit_source", label))
    if limit_source not in SUPPORTED_TORQUE_LIMIT_SOURCES:
        _fail(EXIT_DECLARATION, "control.torque_limit_source 只支持 %s（实际 %r）："
              "另写一套限幅等于第二份安全边界" % (list(SUPPORTED_TORQUE_LIMIT_SOURCES), limit_source))

    pose_source = str(_get(document, "stand.pose_source", label))
    if pose_source not in SUPPORTED_POSE_SOURCES:
        _fail(EXIT_DECLARATION, "stand.pose_source 只支持 %s（实际 %r）"
              % (list(SUPPORTED_POSE_SOURCES), pose_source))
    if pose_source == "explicit":
        pose = _get(document, "stand.pose_rad", label)
        if not isinstance(pose, dict):
            _fail(EXIT_DECLARATION, "stand.pose_source=explicit 时 stand.pose_rad 必须是映射")
        missing = sorted(set(joints) - set(str(key) for key in pose))
        if missing:
            _fail(EXIT_DECLARATION, "stand.pose_rad 缺少关节: %s" % missing)

    ramp_s = _number(_get(document, "stand.ramp_s", label), "stand.ramp_s", allow_zero=True)
    duration_s = _number(_get(document, "stand.duration_s", label), "stand.duration_s")
    hold_s = _number(_get(document, "stand.hold_s", label), "stand.hold_s")
    if duration_s + 1e-12 < ramp_s + hold_s:
        _fail(EXIT_DECLARATION,
              "stand.duration_s（%g）必须 ≥ stand.ramp_s + stand.hold_s（%g）：评估窗口装不下要求的保持时长"
              % (duration_s, ramp_s + hold_s))

    _number(_get(document, "stand.height_target_m", label), "stand.height_target_m", allow_zero=True)
    _number(_get(document, "stand.height_tolerance_m", label), "stand.height_tolerance_m")
    _number(_get(document, "stand.height_std_max_m", label), "stand.height_std_max_m")
    _number(_get(document, "stand.attitude_tolerance_deg", label), "stand.attitude_tolerance_deg")
    _number(_get(document, "stand.speed_tolerance_mps", label), "stand.speed_tolerance_mps")
    _number(_get(document, "stand.max_tracking_error_rad", label), "stand.max_tracking_error_rad")

    mode = str(_get(document, "stop.mode", label))
    if mode not in SUPPORTED_STOP_MODES:
        _fail(EXIT_DECLARATION, "stop.mode 只支持 %s（实际 %r）"
              % (list(SUPPORTED_STOP_MODES), mode))
    stop_duration = _number(_get(document, "stop.duration_s", label), "stop.duration_s")
    stop_static = _number(_get(document, "stop.static_hold_s", label), "stop.static_hold_s")
    stop_final = _number(_get(document, "stop.final_window_s", label), "stop.final_window_s")
    if stop_static > stop_duration:
        _fail(EXIT_DECLARATION, "stop.static_hold_s（%g）不得大于 stop.duration_s（%g）"
              % (stop_static, stop_duration))
    if stop_final > stop_duration:
        _fail(EXIT_DECLARATION, "stop.final_window_s（%g）不得大于 stop.duration_s（%g）"
              % (stop_final, stop_duration))
    _number(_get(document, "stop.speed_tolerance_mps", label), "stop.speed_tolerance_mps")

    _number(_get(document, "state.quat_norm_tolerance", label), "state.quat_norm_tolerance")
    _number(_get(document, "state.gyro_max_abs_error_rad_s", label), "state.gyro_max_abs_error_rad_s")
    _number(_get(document, "state.torque_max_abs_error_nm", label), "state.torque_max_abs_error_nm")

    pattern = str(_get(document, "state.sensors.torque_pattern", label))
    if "{actuator}" not in pattern:
        _fail(EXIT_DECLARATION, "state.sensors.torque_pattern 必须含 {actuator} 占位符，实际 %r" % pattern)

    _repo_relative(_get(document, "model.file", label), "model.file")
    _repo_relative(_get(document, "report.path", label), "report.path")
    return document


def resolve_profile(root, robot_id, relative_path):
    """按声明路径读 Profile，并用**核心校验器**确认它真的是一份合法 Profile。"""
    from iraf_core.profile import ProfileError, load_robot_profile

    path = Path(root) / _repo_relative(relative_path, "robot.profile")
    if not path.is_file():
        _fail(EXIT_REFERENCE, "robot.profile 引用的文件不存在: %s" % relative_path)
    document = _load_yaml(path, "本体 Profile")
    name = str((document.get("metadata") or {}).get("name") or "")
    if name != str(robot_id):
        _fail(EXIT_DECLARATION, "robot.id（%r）与 Profile 名称（%r）不一致：身份必须来自声明" % (robot_id, name))
    try:
        profile = load_robot_profile(path)
    except ProfileError as exc:
        _fail(EXIT_DECLARATION, "Profile 未通过核心校验: %s" % exc)
    return path, document, profile


def resolve_pose(document, profile_document, joints, label="config/go2_loopback.yaml"):
    """解析站立参考位形：`profile_home` 走 Profile 的 `spec.home`，`explicit` 走配置里的映射。"""
    source = str(document["stand"]["pose_source"])
    spec = profile_document.get("spec") or {}
    if source == "profile_home":
        raw = spec.get("home")
        if not isinstance(raw, dict) or not raw:
            _fail(EXIT_REFERENCE,
                  "stand.pose_source=profile_home，但 Profile 未声明 spec.home："
                  "不得回退到模型默认位形（那是隐式默认值）")
        missing = sorted(set(str(item) for item in joints) - set(str(key) for key in raw))
        if missing:
            _fail(EXIT_DECLARATION, "Profile 的 spec.home 缺少关节: %s" % missing)
        pose = {str(key): float(value) for key, value in raw.items()}
    else:
        pose = {str(key): float(value) for key, value in document["stand"]["pose_rad"].items()}
    for name in joints:
        value = float(pose[str(name)])
        if math.isnan(value) or math.isinf(value):
            _fail(EXIT_DECLARATION, "站立位形 %s 不是有限数字: %r" % (name, pose[str(name)]))
    return pose


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_binding(model, mujoco, joints):
    """关节 -> (joint_id, qpos_adr, dof_adr, actuator_id)。只在**这一处**解析命名差异。"""
    binding = {}
    for name in joints:
        name = str(name)
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            _fail(EXIT_REFERENCE, "关节 %s 不在被测模型里" % name)
        actuators = [
            index
            for index in range(model.nu)
            if int(model.actuator_trntype[index]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[index, 0]) == int(joint_id)
        ]
        if not actuators:
            _fail(EXIT_REFERENCE, "关节 %s 没有直接关节执行器（腱驱动/耦合关节必须显式处理，不得静默跳过）" % name)
        if len(actuators) > 1:
            _fail(EXIT_DECLARATION, "关节 %s 绑定多个执行器: %s" % (name, actuators))
        binding[name] = {
            "joint_id": int(joint_id),
            "qpos_adr": int(model.jnt_qposadr[joint_id]),
            "dof_adr": int(model.jnt_dofadr[joint_id]),
            "actuator_id": int(actuators[0]),
        }
    return binding


def compute_ctrl(q, dq, q_des, kp, kd, tau_ff, lower, upper):
    """力矩型 PD + 前馈，并按模型 ctrlrange 截断。纯函数，便于逐项断言。"""
    ctrl = kp * (q_des - q) - kd * dq + tau_ff
    saturated = np.logical_or(ctrl < lower, ctrl > upper)
    return np.clip(ctrl, lower, upper), saturated


def bias_torque(model, data, mujoco, dofs):
    """重力前馈：把 qvel 置零后取 `qfrc_bias`（与臂侧后端同一套做法）。

    注意 `qfrc_bias` 是**自由浮动基**下的偏置（不含地面约束反力），因此它只消除稳态下垂，
    站立载荷由 PD 承担；报告里不把它表述为"支撑力矩"。
    """
    qvel = data.qvel.copy()
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    tau = np.array([data.qfrc_bias[int(dof)] for dof in dofs], dtype=float)
    data.qvel[:] = qvel
    mujoco.mj_forward(model, data)
    return tau


def _sensor_id(model, mujoco, name, label):
    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, str(name))
    if sensor < 0:
        _fail(EXIT_REFERENCE, "%s 引用的传感器不存在: %s" % (label, name))
    return int(sensor)


def _sensor_values(model, data, sensor_id):
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return np.asarray(data.sensordata[address:address + dimension], dtype=float).copy()


def quat_tilt_deg(quat):
    """躯干**倾斜角**（度，相对竖直）：只取横滚/俯仰，不含偏航。

    对 wxyz 四元数，机体 z 轴与世界 z 的夹角满足 R[2,2] = 1 − 2(x²+y²)，
    因此 tilt = 2·asin(√(x²+y²))。偏航（绕 z）不影响本判据 —— "站得直不直"与"朝向哪"是两件事。
    """
    x = float(quat[1])
    y = float(quat[2])
    sine = min(1.0, max(0.0, math.hypot(x, y)))
    return math.degrees(2.0 * math.asin(sine))


def static_onset(times, speeds, tolerance, hold_s):
    """返回最早的 t*：从 t* 起速度连续 < tolerance 至少 hold_s。找不到返回 None。"""
    count = len(times)
    for start in range(count):
        deadline = times[start] + hold_s
        ok = True
        reached = False
        for index in range(start, count):
            if speeds[index] >= tolerance:
                ok = False
                break
            if times[index] >= deadline:
                reached = True
                break
        if ok and reached:
            return float(times[start])
    return None


def _longest_compliant_seconds(times, mask):
    """mask 中连续 True 段的**时长**最大值（用采样时刻差累计）。"""
    best = 0.0
    start = None
    for index, flag in enumerate(mask):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            best = max(best, times[index] - times[start])
            start = None
    if start is not None:
        best = max(best, times[-1] - times[start])
    return float(best)


class _Checks:
    def __init__(self):
        self.items = []

    def add(self, name, value, expectation, passed, detail):
        self.items.append(
            {
                "name": name,
                "value": value,
                "expectation": expectation,
                "passed": bool(passed),
                "detail": detail,
            }
        )
        return bool(passed)


def simulate(model, data, mujoco, binding, joints, config, pose):
    """按声明跑 stand -> stop 两相位，按 sample 频率采样。返回采样数组字典。"""
    control_hz = float(config["control"]["frequency_hz"])
    sample_hz = float(config["sample"]["frequency_hz"])
    kp = float(config["control"]["kp_nm_per_rad"])
    kd = float(config["control"]["kd_nm_s_per_rad"])
    feedforward = bool(config["control"]["gravity_feedforward"])

    timestep = float(model.opt.timestep)
    substeps_float = 1.0 / (control_hz * timestep)
    substeps = int(round(substeps_float))
    if substeps < 1 or abs(substeps_float - substeps) > 1e-9:
        _fail(EXIT_DECLARATION,
              "control.frequency_hz（%g）与模型 timestep（%g s）不整除：一个控制周期 %g 个物理步"
              % (control_hz, timestep, substeps_float))
    sample_every = int(round(control_hz / sample_hz))

    order = [str(item) for item in joints]
    qpos_adr = np.array([binding[name]["qpos_adr"] for name in order], dtype=int)
    dof_adr = np.array([binding[name]["dof_adr"] for name in order], dtype=int)
    act_id = np.array([binding[name]["actuator_id"] for name in order], dtype=int)
    lower = np.array([float(model.actuator_ctrlrange[index][0]) for index in act_id], dtype=float)
    upper = np.array([float(model.actuator_ctrlrange[index][1]) for index in act_id], dtype=float)

    q_target_full = np.array([float(pose[name]) for name in order], dtype=float)
    q_initial = np.asarray(data.qpos[qpos_adr], dtype=float).copy()

    sensors = config["state"]["sensors"]
    quat_sensor = _sensor_id(model, mujoco, sensors["quat"], "state.sensors.quat")
    gyro_sensor = _sensor_id(model, mujoco, sensors["gyro"], "state.sensors.gyro")
    acc_sensor = _sensor_id(model, mujoco, sensors["acc"], "state.sensors.acc")
    torque_sensors = []
    for name, item in ((n, binding[n]) for n in order):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, item["actuator_id"])
        if actuator_name is None:
            _fail(EXIT_REFERENCE, "执行器 %d 没有名字：无法按声明解析力矩传感器" % item["actuator_id"])
        torque_sensors.append(
            _sensor_id(model, mujoco, str(sensors["torque_pattern"]).format(actuator=actuator_name),
                       "state.sensors.torque_pattern")
        )

    imu_site_name = None
    profile_document = config.get("_profile_document") or {}
    imu_site_name = ((profile_document.get("spec") or {}).get("model") or {}).get("imu_site")
    imu_site = -1
    if imu_site_name:
        imu_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, str(imu_site_name))
        if imu_site < 0:
            _fail(EXIT_REFERENCE, "Profile 的 spec.model.imu_site=%s 不在被测模型里" % imu_site_name)

    samples = {
        key: []
        for key in (
            "time_s", "phase", "base_pos", "base_quat", "base_linvel", "base_angvel",
            "joint_pos", "joint_vel", "ctrl", "torque", "quat", "gyro", "acc", "gyro_model",
        )
    }
    saturated_total = 0
    phases = (
        ("stand", float(config["stand"]["duration_s"]), float(config["stand"]["ramp_s"])),
        ("stop", float(config["stop"]["duration_s"]), None),
    )
    for phase_name, duration_s, ramp_s in phases:
        control_steps = int(round(duration_s * control_hz))
        for step in range(control_steps):
            now = float(data.time)
            q = np.asarray(data.qpos[qpos_adr], dtype=float)
            dq = np.asarray(data.qvel[dof_adr], dtype=float)
            tau_ff = np.zeros_like(q)
            if phase_name == "stand":
                if ramp_s and ramp_s > 0.0:
                    alpha = min(1.0, now / ramp_s)
                else:
                    alpha = 1.0
                desired = q_initial + alpha * (q_target_full - q_initial)
                if feedforward:
                    tau_ff = bias_torque(model, data, mujoco, dof_adr)
            else:
                desired = q_target_full
            if phase_name == "stop":
                ctrl = np.zeros_like(q)
                saturated = np.zeros_like(q, dtype=bool)
            else:
                ctrl, saturated = compute_ctrl(q, dq, desired, kp, kd, tau_ff, lower, upper)
            saturated_total += int(np.count_nonzero(saturated))
            data.ctrl[act_id] = ctrl
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            if (step + 1) % sample_every:
                continue
            samples["time_s"].append(float(data.time))
            samples["phase"].append(phase_name)
            samples["base_pos"].append(np.asarray(data.qpos[0:3], dtype=float).copy())
            samples["base_quat"].append(np.asarray(data.qpos[3:7], dtype=float).copy())
            samples["base_linvel"].append(np.asarray(data.qvel[0:3], dtype=float).copy())
            samples["base_angvel"].append(np.asarray(data.qvel[3:6], dtype=float).copy())
            samples["joint_pos"].append(q.copy())
            samples["joint_vel"].append(dq.copy())
            samples["ctrl"].append(np.asarray(data.ctrl[act_id], dtype=float).copy())
            samples["torque"].append(
                np.array(
                    [float(_sensor_values(model, data, sid).ravel()[0]) for sid in torque_sensors],
                    dtype=float,
                )
            )
            samples["quat"].append(_sensor_values(model, data, quat_sensor))
            samples["gyro"].append(_sensor_values(model, data, gyro_sensor))
            samples["acc"].append(_sensor_values(model, data, acc_sensor))
            if imu_site >= 0:
                rotation = np.asarray(data.site_xmat[imu_site], dtype=float).reshape(3, 3).copy()
                velocity = np.zeros(6, dtype=float)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, imu_site, velocity, 0)
                samples["gyro_model"].append(rotation.T @ velocity[0:3])
            else:
                samples["gyro_model"].append(np.zeros(3, dtype=float))
    return samples, {
        "ctrlrange_nm": {str(name): [float(lower[index]), float(upper[index])] for index, name in enumerate(order)},
        "saturated_samples": saturated_total,
        "substeps_per_control": substeps,
    }


def assess(samples, config, joints, binding_info):
    """判据：全部阈值来自声明；与独立计算的模型量比对。"""
    times = np.asarray(samples["time_s"], dtype=float)
    phases = np.asarray(samples["phase"], dtype=str)
    base_z = np.asarray(samples["base_pos"], dtype=float)[:, 2]
    quats = np.asarray(samples["base_quat"], dtype=float)
    speeds = np.linalg.norm(np.asarray(samples["base_linvel"], dtype=float), axis=1)
    joint_pos = np.asarray(samples["joint_pos"], dtype=float)
    joint_vel = np.asarray(samples["joint_vel"], dtype=float)
    ctrl = np.asarray(samples["ctrl"], dtype=float)
    torque = np.asarray(samples["torque"], dtype=float)
    quat_sensor = np.asarray(samples["quat"], dtype=float)
    gyro = np.asarray(samples["gyro"], dtype=float)
    gyro_model = np.asarray(samples["gyro_model"], dtype=float)

    stand_cfg = config["stand"]
    stop_cfg = config["stop"]
    state_cfg = config["state"]

    stand_mask = phases == "stand"
    window_mask = stand_mask & (times >= float(stand_cfg["ramp_s"]))
    checks = _Checks()

    if not window_mask.any():
        _fail(EXIT_DECLARATION, "评估窗口为空：check stand.duration_s / stand.ramp_s / sample.frequency_hz")

    window_times = times[window_mask]
    height = base_z[window_mask]
    attitude = np.array([quat_tilt_deg(item) for item in quats[window_mask]], dtype=float)
    window_speed = speeds[window_mask]

    target = float(stand_cfg["height_target_m"])
    tolerance = float(stand_cfg["height_tolerance_m"])
    std_max = float(stand_cfg["height_std_max_m"])
    attitude_max = float(stand_cfg["attitude_tolerance_deg"])
    speed_max = float(stand_cfg["speed_tolerance_mps"])

    compliant = (
        (np.abs(height - target) <= tolerance)
        & (attitude <= attitude_max)
        & (window_speed <= speed_max)
    )
    hold_seconds = _longest_compliant_seconds(window_times, compliant)

    height_mean = float(height.mean())
    height_std = float(height.std())
    checks.add("stand.height_mean_within_tolerance", round(height_mean, 9),
               "|%.6f - %.6f| <= %g" % (height_mean, target, tolerance),
               abs(height_mean - target) <= tolerance + 1e-12,
               "稳定段躯干高度均值必须在声明容差内（声明目标 = 厂商关键帧基座 z）")
    checks.add("stand.height_std", round(height_std, 9), "<= %g" % std_max,
               height_std <= std_max + 1e-12, "稳定段高度标准差上限（把振荡与稳稳站住分开）")
    checks.add("stand.max_attitude_error_deg", round(float(attitude.max()), 9), "<= %g" % attitude_max,
               float(attitude.max()) <= attitude_max + 1e-12, "躯干相对竖直的最大倾斜角（不含偏航）")
    checks.add("stand.hold_seconds", round(hold_seconds, 9), ">= %g" % float(stand_cfg["hold_s"]),
               hold_seconds + 1e-9 >= float(stand_cfg["hold_s"]),
               "全部判据连续达标的时长（高度/姿态/速度同时满足）")
    tracking = float(np.abs(joint_pos[window_mask] - np.array(
        [float(config["_pose"][str(name)]) for name in joints], dtype=float)).max())
    checks.add("stand.max_tracking_error_rad", round(tracking, 9),
               "<= %g" % float(stand_cfg["max_tracking_error_rad"]),
               tracking <= float(stand_cfg["max_tracking_error_rad"]) + 1e-12,
               "站立窗口内 max|q − q_des|（PD 稳态下垂的量化）")

    stop_mask = phases == "stop"
    if not stop_mask.any():
        _fail(EXIT_DECLARATION, "stop 相位没有采样：check stop.duration_s / sample.frequency_hz")
    stop_times = times[stop_mask]
    stop_speed = speeds[stop_mask]
    stop_height = base_z[stop_mask]
    stop_tolerance = float(stop_cfg["speed_tolerance_mps"])
    onset = static_onset(stop_times, stop_speed, stop_tolerance, float(stop_cfg["static_hold_s"]))
    final_window = float(stop_cfg["final_window_s"])
    final_mask = stop_times >= (stop_times[-1] - final_window)
    final_speed = float(stop_speed[final_mask].mean()) if final_mask.any() else float(stop_speed[-1])
    final_height = float(stop_height[final_mask].mean()) if final_mask.any() else float(stop_height[-1])
    checks.add("stop.static_entered", None if onset is None else round(onset, 9),
               "存在连续 %g s 速度 < %g m/s 的静止段" % (float(stop_cfg["static_hold_s"]), stop_tolerance),
               onset is not None,
               "力矩型执行器松力后必须进入静止（速度衰减判定）")

    quat_norm_error = float(np.abs(np.linalg.norm(quat_sensor, axis=1) - 1.0).max())
    checks.add("state.quat_norm", round(quat_norm_error, 12),
               "<= %g" % float(state_cfg["quat_norm_tolerance"]),
               quat_norm_error <= float(state_cfg["quat_norm_tolerance"]),
               "imu_quat 读数必须是单位四元数")
    gyro_error = float(np.abs(gyro - gyro_model).max())
    checks.add("state.gyro_matches_model_angular_velocity", round(gyro_error, 12),
               "<= %g rad/s" % float(state_cfg["gyro_max_abs_error_rad_s"]),
               gyro_error <= float(state_cfg["gyro_max_abs_error_rad_s"]),
               "陀螺仪读数 vs 独立计算（mj_objectVelocity）的传感器系角速度：抓「挂错 site」")
    torque_error = float(np.abs(torque - ctrl).max())
    checks.add("state.torque_matches_applied_ctrl", round(torque_error, 12),
               "<= %g N·m" % float(state_cfg["torque_max_abs_error_nm"]),
               torque_error <= float(state_cfg["torque_max_abs_error_nm"]),
               "jointactuatorfrc 读数 vs 施加的 ctrl×gear（gear=1）：抓「力矩传感器挂错执行器」")
    overshoot = float((ctrl - np.array([binding_info["ctrlrange_nm"][str(name)] for name in joints], dtype=float)[:, 1]).max())
    undershoot = float((np.array([binding_info["ctrlrange_nm"][str(name)] for name in joints], dtype=float)[:, 0] - ctrl).max())
    checks.add("control.within_model_ctrlrange", [round(overshoot, 9), round(undershoot, 9)],
               "both <= 0", overshoot <= 1e-9 and undershoot <= 1e-9,
               "控制量必须落在模型 ctrlrange 内（限幅来源 = model）")

    failed = [item["name"] for item in checks.items if not item["passed"]]
    stand = {
        "window_seconds": float(window_times[-1] - window_times[0]) if len(window_times) > 1 else 0.0,
        "samples": int(window_mask.sum()),
        "height_target_m": target,
        "height_tolerance_m": tolerance,
        "height_mean_m": height_mean,
        "height_std_m": height_std,
        "height_min_m": float(height.min()),
        "height_max_m": float(height.max()),
        "max_attitude_error_deg": float(attitude.max()),
        "max_speed_mps": float(window_speed.max()),
        "hold_seconds": hold_seconds,
        "hold_seconds_required": float(stand_cfg["hold_s"]),
        "max_tracking_error_rad": tracking,
        "ctrl_saturated_samples": int(binding_info["saturated_samples"]),
    }
    stop = {
        "mode": str(stop_cfg["mode"]),
        "duration_s": float(stop_cfg["duration_s"]),
        "speed_tolerance_mps": stop_tolerance,
        "static_hold_s": float(stop_cfg["static_hold_s"]),
        "static_entered": onset is not None,
        "seconds_to_static": None if onset is None else float(onset - stop_times[0]),
        "final_speed_mps": final_speed,
        "final_height_m": final_height,
        "min_speed_mps": float(stop_speed.min()),
        "collapsed": bool(final_height < target - tolerance),
        "samples": int(stop_mask.sum()),
    }
    state = {
        "frequency_hz": float(config["sample"]["frequency_hz"]),
        "samples": int(len(times)),
        "joint_velocity_max_rad_s": float(np.abs(joint_vel).max()),
        "imu": {
            "quat_norm_max_error": quat_norm_error,
            "gyro_max_abs_error_rad_s": gyro_error,
            "gyro_max_abs_rad_s": float(np.abs(gyro).max()),
            "acc_mean_m_s2": np.asarray(np.asarray(samples["acc"], dtype=float).mean(axis=0)).tolist(),
            "acc_norm_mean_m_s2": float(np.linalg.norm(np.asarray(samples["acc"], dtype=float), axis=1).mean()),
        },
        "torque": {
            "max_abs_nm": float(np.abs(torque).max()),
            "sensor_vs_ctrl_max_abs_error_nm": torque_error,
            "ctrlrange_nm": binding_info["ctrlrange_nm"],
        },
        "substeps_per_control": int(binding_info["substeps_per_control"]),
    }
    return {
        "stand": stand,
        "stop": stop,
        "state": state,
        "checks": checks.items,
        "failed_checks": failed,
    }


def run_loopback(config_path, root=None, report_path=None):
    """主流程：读声明 -> 编译模型 -> loopback -> 报告。返回 (report, exit_code)。"""
    import mujoco

    root = Path(root) if root else repo_root()
    config_path = Path(config_path)
    if not config_path.is_file():
        _fail(EXIT_USAGE, "配置文件不存在: %s" % config_path)
    document = _load_yaml(config_path, "loopback 声明")

    robot_id = str(document.get("robot", {}).get("id") or "")
    profile_rel = document.get("robot", {}).get("profile")
    if not robot_id or not isinstance(profile_rel, str):
        _fail(EXIT_DECLARATION, "robot.id / robot.profile 必须显式声明（身份只能来自声明）")
    profile_path, profile_document, profile = resolve_profile(root, robot_id, profile_rel)
    joints = [str(item) for item in profile.joints]
    validate_declaration(document, joints)

    model_path = Path(root) / _repo_relative(document["model"]["file"], "model.file")
    if not model_path.is_file():
        _fail(EXIT_REFERENCE,
              "被测模型不存在: %s（先跑 %s --scene scenes/handoff_lab --robot %s 生成）"
              % (document["model"]["file"], document["model"]["builder"], robot_id))
    builder_rel = str(document["model"]["builder"])
    if not (Path(root) / _repo_relative(builder_rel, "model.builder")).is_file():
        _fail(EXIT_REFERENCE, "model.builder 引用的构建器不存在: %s" % builder_rel)

    pose = resolve_pose(document, profile_document, joints)
    document["_pose"] = pose
    document["_profile_document"] = profile_document

    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception as exc:
        _fail(EXIT_MODEL, "模型编译失败（%s）: %s" % (model_path, exc))
    keyframe_name = str(document["initial"]["keyframe"])
    keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
    if keyframe_id < 0:
        _fail(EXIT_REFERENCE, "initial.keyframe=%s 不在模型里（实测 nkey=%d）" % (keyframe_name, model.nkey))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    mujoco.mj_forward(model, data)

    binding = resolve_binding(model, mujoco, joints)
    samples, binding_info = simulate(model, data, mujoco, binding, joints, document, pose)
    assessment = assess(samples, document, joints, binding_info)

    report_path = Path(report_path) if report_path else Path(root) / _repo_relative(
        document["report"]["path"], "report.path")
    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "kind": REPORT_KIND,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "robot": {
            "id": robot_id,
            "profile": str(_repo_relative(profile_rel, "robot.profile")),
            "profile_sha256": sha256_file(profile_path),
            "joints": joints,
            "pose_source": str(document["stand"]["pose_source"]),
        },
        "model": {
            "path": str(_repo_relative(document["model"]["file"], "model.file")),
            "sha256": sha256_file(model_path),
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "timestep_s": float(model.opt.timestep),
            "keyframe": keyframe_name,
            "keyframe_id": int(keyframe_id),
        },
        "config": {
            "path": str(config_path),
            "sha256": sha256_file(config_path),
            "controller": {
                "frequency_hz": float(document["control"]["frequency_hz"]),
                "kp_nm_per_rad": float(document["control"]["kp_nm_per_rad"]),
                "kd_nm_s_per_rad": float(document["control"]["kd_nm_s_per_rad"]),
                "gravity_feedforward": bool(document["control"]["gravity_feedforward"]),
                "torque_limit_source": str(document["control"]["torque_limit_source"]),
            },
            "stand_pose_rad": {str(name): float(pose[str(name)]) for name in joints},
        },
        "stand": assessment["stand"],
        "stop": assessment["stop"],
        "state": assessment["state"],
        "checks": assessment["checks"],
        "failed_checks": assessment["failed_checks"],
        "not_proved": [
            "站立 ≠ 步态/导航/停靠能力：本报告只证明声明位形下能站住、能进入静止、状态可读。",
            "stop.mode=torque_zero_release 是松力停机：力矩型执行器松力后失能躺倒（见 stop.collapsed），不代表「站立保持式停机」。",
            "重力前馈取自由浮动基的 qfrc_bias，不含地面约束反力，因此不是支撑力矩；站立载荷由 PD 承担。",
            "全部结论属于仿真（simulation=true）；目标端/真机验收 DEFERRED（板卡不在场），本报告不含任何真机证据。",
        ],
    }
    report["passed"] = not report["failed_checks"]
    report["exit_code"] = EXIT_OK if report["passed"] else EXIT_FAILED
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report, report["exit_code"]
