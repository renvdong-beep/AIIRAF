"""力矩级反馈平衡器（战役 iraf-24h-2 步骤 02b；ADR-0008 决策 1c）。

为什么需要它（实测事实，不重复量）
--------------------------------
位置级（关节空间）足端目标偏移这一族执行器已被两轮独立实测证否：把步高压到 0.001 m
（四足基本不离地、支撑相 1.0、support_legs min=4）后，幅度 0 的对照完全稳定
（z_mean 0.279955 m、倾角 0.091°、漂移 0.00649 m、饱和 0），而幅度 **5 mm** 即翻倒
（z_mean −2.989653 m、倾角 172.139°、饱和 2145）；且「常量重心平移」的可行域为**空集**
（对角腿对所需平移夹角 180.00°、两相位余量之和上确界 +0.000000000 m）。
⇒ 机身高/姿态只能靠**力矩级**反馈动作（本模块），不能靠目标偏移。
证据：`docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md` §8~§10。

本模块的分层（与参考实现一致）
------------------------------
- **只做纯函数计算**，全部数字来自声明 `balance` 段（缺键/越界即 `DeclarationError`）：
  1. `load_balance_declaration`：解析并校验声明；
  2. `desired_wrench`：实测机身高度/姿态/速度 → 期望机身力旋量（平衡器的“要什么”）；
  3. `allocate_foot_forces`：力旋量 → 各**支撑腿**足端力（含逐腿截断，记 `clamped`）；
  4. `leg_leg_torques`：足端力 → 该腿关节力矩（`τ = Jᵀf`，J 由调用方从被测模型实测）。
- **不 import mujoco**：质量/重力/接触力/雅可比全部由适配器（`unitree_go2.py`）读出后传入
  ⇒ 本模块的判据可在无仿真环境的单测里逐项断言（纯 numpy）。

关键设计取舍（写清楚，避免后来者误读）
------------------------------------
- **支撑/摆动两套位置权重**（B1，ADR-0008 决策 1d）：`weight_position` 作用于**非支撑（摆动）**
  关节；`stance_weight_position`（B1 取 **0**）作用于**支撑**关节 ⇒ 支撑腿只受 `τ_bal` 驱动，
  `τ_pd`（含 PD + 重力前馈）被整体乘 0；摆动腿仍走位置级 PD + 重力前馈。哪个关节属于支撑，
  由**实测接触力**判定（不做相位推断，见 `_balance_provider` 的 docstring）。
- **含重力支撑前馈**（`include_gravity_support: true`，B1）：支撑腿关掉位置级分量后不再有
  PD 平衡点，mg 支撑**必须**由力矩级提供（`desired_wrench` 的 `F_z = include_gravity_support·mg
  + 高度纠正`，再经足端力分配摊到各支撑腿）；`false` 只适用于"位置级承重 + 力矩级纠正"的旧结构
  （实测：站立载荷已由 PD 承担，此时再叠加 mg 会让升力翻倍）。
- 力旋量的参考点在**躯干体心**（调用方传入实测位置），因此足端力分配解的是
  「合力 = F、合力矩 = τ」的最小范数解（`lstsq`），腿数 ≥3 时才有唯一的最小范数解。
- 逐腿截断是**诚实截断**：被截断的腿在其中记录 `clamped`，并集汇成 `clamped_legs`
  ⇒ 允许下游把「平衡器要求了但没兑现」与「本来就没要求」区分开。

诚实边界：本模块的结论一律属于**仿真**（`simulation=true`）；真机/目标端验收在本战役中
DEFERRED（板卡不在场）。力矩上限**不由本模块**施加：最终控制量仍在适配器里按模型
`ctrlrange` 截断（声明 `control.torque_limit_source: model` 是唯一来源）。
"""

import math

import numpy as np

from iraf_adapters.unitree.quadruped import DeclarationError

#: `balance` 段必需键（缺任一即显式失败，禁止实现层默认值）。
REQUIRED_BALANCE_KEYS = (
    "enabled",
    # 位置级 PD 权重：`weight_position` 作用于**非支撑（摆动）**关节，
    # `stance_weight_position` 作用于**支撑**关节（B1：取 0 ⇒ 该腿走力矩级力控）。
    # 两者都必须显式声明：支撑/摆动是两套语义，合成一个全局权重就无法表达 B1。
    "weight_position",
    "stance_weight_position",
    "weight_balance",
    "include_gravity_support",
    "attitude",
    "height",
    "velocity",
    "allocation",
    "watchdog",
    "verification",
)

#: `balance.attitude` 必需键（滚转/俯仰 PD 与力矩上限）。
REQUIRED_ATTITUDE_KEYS = ("kp_nm_per_rad", "kd_nm_s_per_rad", "max_torque_nm")

#: `balance.height` 必需键（高度保持 PD 与力上限）。
REQUIRED_HEIGHT_KEYS = ("kp_n_per_m", "kd_n_s_per_m", "max_force_n")

#: `balance.velocity` 必需键（机身平动/转动阻尼）。
REQUIRED_VELOCITY_KEYS = (
    "linear_gain_ns_per_m",
    "max_force_n",
    "angular_gain_nms_per_rad",
    "max_torque_nm",
)

#: `balance.allocation` 必需键（力分配的腿数与逐腿上限）。
REQUIRED_ALLOCATION_KEYS = (
    "min_stance_legs",
    "max_normal_force_n",
    "normal_force_floor_n",
    "max_horizontal_force_n",
)

#: `balance.watchdog` 必需键（无有效支撑腿时的退出条件）。
REQUIRED_WATCHDOG_KEYS = ("max_consecutive_no_stance_cycles",)

#: `balance.verification` 必需键（静态抗扰 + 隔离实验的判据；数字全在声明里）。
REQUIRED_VERIFICATION_KEYS = ("report", "static_disturbance", "isolation")

#: `balance.verification.static_disturbance` 必需键。
REQUIRED_DISTURBANCE_KEYS = (
    "force_n",
    "axes",
    "duration_s",
    "hold_s",
    "max_tilt_deg",
    "max_height_error_m",
    "max_drift_m",
    "max_saturated_samples",
)

#: `balance.verification.isolation` 必需键。
REQUIRED_ISOLATION_KEYS = (
    "step_height_m",
    "amplitudes_m",
    "duration_s",
    "no_fall_amplitude_m",
    "fall_base_height_m",
    "max_tilt_deg",
)


def _require_section(section, label):
    if not isinstance(section, dict):
        raise DeclarationError("%s 段缺失或不是对象（禁止默认值兜底）" % label)
    return section


def _require_keys(section, keys, label):
    missing = [key for key in keys if key not in section]
    if missing:
        raise DeclarationError("%s 缺少必需键（禁止默认值兜底）: %s" % (label, missing))
    return section


def _number(section, key, label, *, positive=False, non_negative=False, maximum=None):
    value = section[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DeclarationError("%s.%s 必须是数值，实际: %r" % (label, key, value))
    value = float(value)
    if not math.isfinite(value):
        raise DeclarationError("%s.%s 必须是有限数值，实际: %r" % (label, key, value))
    if positive and value <= 0.0:
        raise DeclarationError("%s.%s 必须为正数，实际: %r" % (label, key, value))
    if non_negative and value < 0.0:
        raise DeclarationError("%s.%s 不得为负，实际: %r" % (label, key, value))
    if maximum is not None and value > float(maximum):
        raise DeclarationError(
            "%s.%s 不得大于 %r（上界由实现层强制，防止把判据写成恒真）：实际 %r"
            % (label, key, maximum, value)
        )
    return value


def _integer(section, key, label, *, minimum=1):
    value = section[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise DeclarationError("%s.%s 必须是整数，实际: %r" % (label, key, value))
    if value < int(minimum):
        raise DeclarationError("%s.%s 不得小于 %d，实际: %r" % (label, key, minimum, value))
    return int(value)


def _boolean(section, key, label):
    value = section[key]
    if not isinstance(value, bool):
        raise DeclarationError("%s.%s 必须是布尔值，实际: %r" % (label, key, value))
    return bool(value)


def load_balance_declaration(declaration):
    """解析并校验 `balance` 段；缺键/类型错/越界一律显式失败（不做默认值兜底）。

    返回的字典**只含数值与结构**，不含任何"实现层默认值"：调用方（适配器/验收入口）拿到的
    每个数字都能在 `config/go2_loopback.yaml` 里逐字找到。
    """
    if not isinstance(declaration, dict):
        raise DeclarationError("声明顶层必须是对象")
    section = _require_section(declaration.get("balance"), "balance")
    _require_keys(section, REQUIRED_BALANCE_KEYS, "balance")

    enabled = _boolean(section, "enabled", "balance")
    # 权重是「τ_pd 的乘子」，语义上必须落在 [0,1]：>1 是**第二份增益**（增益已由 control.*
    # 声明），<0 会把 PD 变成正反馈。上界由实现层强制（防止把声明写成放大器）。
    weight_position = _number(
        section, "weight_position", "balance", non_negative=True, maximum=1.0
    )
    stance_weight_position = _number(
        section, "stance_weight_position", "balance", non_negative=True, maximum=1.0
    )
    weight_balance = _number(
        section, "weight_balance", "balance", non_negative=True, maximum=1.0
    )
    if weight_position == 0.0 and weight_balance == 0.0:
        raise DeclarationError(
            "balance.weight_position 与 balance.weight_balance 不得同时为 0"
            "（总控制量会恒为零：这不是“关闭平衡器”，而是丢掉了位置控制）"
        )
    # B1（ADR-0008 决策 1d）：支撑腿走力矩级力控 ⇒ 其 τ_pd 权重为 0，此时**唯一**的控制量
    # 来自力矩级平衡项；若该项也为 0，支撑腿会拿到零力矩 ⇒ 整机直接塌陷（不是“关闭”，是失能）。
    # 因此这一对必须显式校验（缺了它就是一条「声明自相矛盾却照跑」的旁路）。
    if stance_weight_position == 0.0 and weight_balance == 0.0:
        raise DeclarationError(
            "balance.stance_weight_position 与 balance.weight_balance 不得同时为 0"
            "（支撑腿的力矩级力控已关掉位置级分量 ⇒ 两者同时为 0 时支撑腿控制量恒为零，"
            "整机将直接塌陷）"
        )
    include_gravity_support = _boolean(section, "include_gravity_support", "balance")

    attitude = _require_section(section.get("attitude"), "balance.attitude")
    _require_keys(attitude, REQUIRED_ATTITUDE_KEYS, "balance.attitude")
    height = _require_section(section.get("height"), "balance.height")
    _require_keys(height, REQUIRED_HEIGHT_KEYS, "balance.height")
    velocity = _require_section(section.get("velocity"), "balance.velocity")
    _require_keys(velocity, REQUIRED_VELOCITY_KEYS, "balance.velocity")
    allocation = _require_section(section.get("allocation"), "balance.allocation")
    _require_keys(allocation, REQUIRED_ALLOCATION_KEYS, "balance.allocation")
    watchdog = _require_section(section.get("watchdog"), "balance.watchdog")
    _require_keys(watchdog, REQUIRED_WATCHDOG_KEYS, "balance.watchdog")
    verification = _require_section(section.get("verification"), "balance.verification")
    _require_keys(verification, REQUIRED_VERIFICATION_KEYS, "balance.verification")
    disturbance = _require_section(
        verification.get("static_disturbance"), "balance.verification.static_disturbance"
    )
    _require_keys(
        disturbance, REQUIRED_DISTURBANCE_KEYS, "balance.verification.static_disturbance"
    )
    isolation = _require_section(verification.get("isolation"), "balance.verification.isolation")
    _require_keys(isolation, REQUIRED_ISOLATION_KEYS, "balance.verification.isolation")

    axes = disturbance["axes"]
    if not isinstance(axes, list) or not axes or not all(item in ("x", "y") for item in axes):
        raise DeclarationError(
            "balance.verification.static_disturbance.axes 必须是 ['x']/['y']/['x','y'] 之一，"
            "实际: %r" % (axes,)
        )
    amplitudes = isolation["amplitudes_m"]
    if not isinstance(amplitudes, list) or not amplitudes:
        raise DeclarationError("balance.verification.isolation.amplitudes_m 必须是非空列表")
    amplitudes = [float(item) for item in amplitudes]
    if [item for item in amplitudes if item < 0.0]:
        raise DeclarationError("balance.verification.isolation.amplitudes_m 不得含负值")
    no_fall = float(isolation["no_fall_amplitude_m"])
    if no_fall not in amplitudes:
        raise DeclarationError(
            "balance.verification.isolation.no_fall_amplitude_m=%r 必须出现在 amplitudes_m 里"
            "（否则“这一档不许翻倒”的判据无从对应）" % no_fall
        )

    min_stance_legs = _integer(allocation, "min_stance_legs", "balance.allocation", minimum=2)
    if min_stance_legs > 4:
        raise DeclarationError(
            "balance.allocation.min_stance_legs=%d > 4：四足不可能满足该条件（门禁必须可满足）"
            % min_stance_legs
        )
    max_normal_force = _number(
        allocation, "max_normal_force_n", "balance.allocation", positive=True
    )
    # 法向力是**纠正量**（不含 mg 支撑），因此下界允许为负（负 = 把机身往下压）。
    # 上界 0 由实现层强制：正值下界会把负向纠正整段截掉，实测表现为「力矩残差 == 目标力矩」
    # （姿态力矩完全没兑现）——那正是本轮首跑把静态抗倾覆做差的原因。
    normal_floor = _number(
        allocation, "normal_force_floor_n", "balance.allocation", maximum=0.0
    )
    if normal_floor < -max_normal_force:
        raise DeclarationError(
            "balance.allocation.normal_force_floor_n=%r 的绝对值不得大于 max_normal_force_n=%r"
            "（否则下界恒不起作用 = 恒真门禁）" % (normal_floor, max_normal_force)
        )

    return {
        "enabled": enabled,
        "weight_position": weight_position,
        "stance_weight_position": stance_weight_position,
        "weight_balance": weight_balance,
        "include_gravity_support": include_gravity_support,
        "attitude": {
            "kp_nm_per_rad": _number(
                attitude, "kp_nm_per_rad", "balance.attitude", non_negative=True
            ),
            "kd_nm_s_per_rad": _number(
                attitude, "kd_nm_s_per_rad", "balance.attitude", non_negative=True
            ),
            "max_torque_nm": _number(
                attitude, "max_torque_nm", "balance.attitude", positive=True
            ),
        },
        "height": {
            "kp_n_per_m": _number(height, "kp_n_per_m", "balance.height", non_negative=True),
            "kd_n_s_per_m": _number(height, "kd_n_s_per_m", "balance.height", non_negative=True),
            "max_force_n": _number(height, "max_force_n", "balance.height", non_negative=True),
        },
        "velocity": {
            "linear_gain_ns_per_m": _number(
                velocity, "linear_gain_ns_per_m", "balance.velocity", non_negative=True
            ),
            "max_force_n": _number(
                velocity, "max_force_n", "balance.velocity", non_negative=True
            ),
            "angular_gain_nms_per_rad": _number(
                velocity, "angular_gain_nms_per_rad", "balance.velocity", non_negative=True
            ),
            "max_torque_nm": _number(
                velocity, "max_torque_nm", "balance.velocity", non_negative=True
            ),
        },
        "allocation": {
            "min_stance_legs": min_stance_legs,
            "max_normal_force_n": max_normal_force,
            "normal_force_floor_n": normal_floor,
            "max_horizontal_force_n": _number(
                allocation, "max_horizontal_force_n", "balance.allocation", positive=True
            ),
        },
        "watchdog": {
            "max_consecutive_no_stance_cycles": _integer(
                watchdog,
                "max_consecutive_no_stance_cycles",
                "balance.watchdog",
                minimum=1,
            ),
        },
        "verification": {
            "report": str(verification["report"]),
            "static_disturbance": {
                "force_n": _number(
                    disturbance, "force_n", "balance.verification.static_disturbance", positive=True
                ),
                "axes": [str(item) for item in axes],
                "duration_s": _number(
                    disturbance,
                    "duration_s",
                    "balance.verification.static_disturbance",
                    positive=True,
                ),
                "hold_s": _number(
                    disturbance, "hold_s", "balance.verification.static_disturbance", positive=True
                ),
                "max_tilt_deg": _number(
                    disturbance,
                    "max_tilt_deg",
                    "balance.verification.static_disturbance",
                    positive=True,
                    maximum=45.0,
                ),
                "max_height_error_m": _number(
                    disturbance,
                    "max_height_error_m",
                    "balance.verification.static_disturbance",
                    positive=True,
                ),
                "max_drift_m": _number(
                    disturbance,
                    "max_drift_m",
                    "balance.verification.static_disturbance",
                    positive=True,
                ),
                "max_saturated_samples": _integer(
                    disturbance,
                    "max_saturated_samples",
                    "balance.verification.static_disturbance",
                    minimum=0,
                ),
            },
            "isolation": {
                "step_height_m": _number(
                    isolation, "step_height_m", "balance.verification.isolation", positive=True
                ),
                "amplitudes_m": amplitudes,
                "duration_s": _number(
                    isolation, "duration_s", "balance.verification.isolation", positive=True
                ),
                "no_fall_amplitude_m": no_fall,
                "fall_base_height_m": _number(
                    isolation,
                    "fall_base_height_m",
                    "balance.verification.isolation",
                    positive=True,
                ),
                "max_tilt_deg": _number(
                    isolation,
                    "max_tilt_deg",
                    "balance.verification.isolation",
                    positive=True,
                    maximum=45.0,
                ),
            },
        },
    }


def _clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def desired_wrench(
    params,
    *,
    mass_kg,
    gravity_mps2,
    height_m,
    height_target_m,
    vertical_velocity_mps,
    roll_rad,
    pitch_rad,
    horizontal_velocity_world_mps,
    angular_velocity_world_rad_s,
):
    """实测机身状态 → 期望**机身力旋量** `(force_n, torque_nm)`（世界系，参考点=躯干体心）。

    各项与符号（每一项都能在报告里被单独复核）：
    - `F_z = include_gravity_support·mg + kp_h·(h_t − h) − kd_h·v_z`，纠正量按 `height.max_force_n` 截断。
      `include_gravity_support=false` 时**不含 mg**：站立载荷已由位置级 PD 承担，再加一份会让升力翻倍。
    - `F_xy = −kv·v_xy`（纯速度阻尼，不含位置项）：逐相位重心转移是**有意**的机身位移，
      加位置项会与它对抗；阻尼项只抑制“整段滑走”（stick-slip）。
    - `τ_x = −kp_a·roll − kd_a·ω_x`、`τ_y = −kp_a·pitch − kd_a·ω_y`（回正 + 阻尼）；
      `τ_z = −kd_yaw·ω_z`（偏航只阻尼，不做位置回正：原地踏步的偏航目标由步态给）。
    """
    attitude = params["attitude"]
    height = params["height"]
    velocity = params["velocity"]
    mass_kg = float(mass_kg)
    gravity_mps2 = float(gravity_mps2)
    if not (math.isfinite(mass_kg) and mass_kg > 0.0):
        raise DeclarationError("整机质量必须是正有限值（来自被测模型），实际: %r" % (mass_kg,))
    if not (math.isfinite(gravity_mps2) and gravity_mps2 != 0.0):
        raise DeclarationError("重力必须是非零有限值（来自被测模型），实际: %r" % (gravity_mps2,))

    v_xy = np.asarray(horizontal_velocity_world_mps, dtype=float)
    if v_xy.shape != (2,):
        raise DeclarationError("水平速度必须是 2 维（世界系 x/y），实际形状: %r" % (v_xy.shape,))
    omega = np.asarray(angular_velocity_world_rad_s, dtype=float)
    if omega.shape != (3,):
        raise DeclarationError("角速度必须是 3 维（世界系），实际形状: %r" % (omega.shape,))

    correction = float(height["kp_n_per_m"]) * (float(height_target_m) - float(height_m)) - float(
        height["kd_n_s_per_m"]
    ) * float(vertical_velocity_mps)
    correction = _clamp(correction, height["max_force_n"])
    support = mass_kg * gravity_mps2 if params["include_gravity_support"] else 0.0
    force_z = support + correction

    damping = -float(velocity["linear_gain_ns_per_m"]) * v_xy
    damping_norm = float(np.linalg.norm(damping))
    if damping_norm > float(velocity["max_force_n"]) > 0.0:
        damping = damping * (float(velocity["max_force_n"]) / damping_norm)

    torque = np.array(
        [
            -float(attitude["kp_nm_per_rad"]) * float(roll_rad)
            - float(attitude["kd_nm_s_per_rad"]) * float(omega[0]),
            -float(attitude["kp_nm_per_rad"]) * float(pitch_rad)
            - float(attitude["kd_nm_s_per_rad"]) * float(omega[1]),
            -float(velocity["angular_gain_nms_per_rad"]) * float(omega[2]),
        ],
        dtype=float,
    )
    tilt_limit = float(attitude["max_torque_nm"])
    yaw_limit = float(velocity["max_torque_nm"])
    torque = np.array(
        [
            _clamp(torque[0], tilt_limit),
            _clamp(torque[1], tilt_limit),
            _clamp(torque[2], yaw_limit),
        ],
        dtype=float,
    )
    return {
        "force_n": np.array([float(damping[0]), float(damping[1]), float(force_z)], dtype=float),
        "torque_nm": torque,
        "components": {
            "gravity_support_n": float(support),
            "height_correction_n": float(correction),
            "horizontal_damping_n": [float(damping[0]), float(damping[1])],
            "attitude_torque_nm": [float(torque[0]), float(torque[1])],
            "yaw_damping_torque_nm": float(torque[2]),
        },
    }


def _skew(radius):
    x, y, z = (float(radius[0]), float(radius[1]), float(radius[2]))
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def wrench_matrix(radii):
    """支撑腿足端相对体心的位置 → 力旋量映射矩阵 `G`（6 × 3N），用于单测逐项断言。"""
    radii = [np.asarray(item, dtype=float).reshape(3) for item in radii]
    blocks_top = [np.eye(3, dtype=float) for _ in radii]
    blocks_bottom = [_skew(item) for item in radii]
    return np.hstack([np.vstack([blocks_top[i], blocks_bottom[i]]) for i in range(len(radii))])


def allocate_foot_forces(params, wrench, stance_points, center_world):
    """期望力旋量 → 各支撑腿足端力（世界系），逐腿截断并如实记录。

    `stance_points`：`{腿代号: 足端世界坐标}`（只含**实测接触**的腿）。
    `center_world`：力旋量参考点（躯干体心）的世界坐标。
    返回 `{"forces": {code: array(3)}, "clamped_legs": [...], "residual": {...}}`。
    """
    allocation = params["allocation"]
    codes = sorted(stance_points)
    if len(codes) < int(allocation["min_stance_legs"]):
        raise DeclarationError(
            "支撑腿数 %d < 声明的 min_stance_legs=%d：平衡器不得在少于声明的支撑腿上分配力"
            % (len(codes), int(allocation["min_stance_legs"]))
        )
    center = np.asarray(center_world, dtype=float).reshape(3)
    radii = [np.asarray(stance_points[code], dtype=float).reshape(3) - center for code in codes]
    matrix = wrench_matrix(radii)
    target = np.concatenate(
        [
            np.asarray(wrench["force_n"], dtype=float).reshape(3),
            np.asarray(wrench["torque_nm"], dtype=float).reshape(3),
        ]
    )
    try:
        solution, _residuals, rank, _singular = np.linalg.lstsq(matrix, target, rcond=None)
    except np.linalg.LinAlgError as exc:  # 退化足形（共线）时不许静默给零
        raise DeclarationError("足端力分配不可解（足形退化）: %s" % exc)
    if rank < 6:
        raise DeclarationError(
            "力旋量映射矩阵秩 %d < 6（支撑腿共线或重合）：显式失败而不是返回近似解" % rank
        )

    max_normal = float(allocation["max_normal_force_n"])
    floor_normal = float(allocation["normal_force_floor_n"])
    max_horizontal = float(allocation["max_horizontal_force_n"])
    forces = {}
    clamped = []
    for index, code in enumerate(codes):
        raw = np.asarray(solution[3 * index:3 * index + 3], dtype=float)
        force = raw.copy()
        force[0] = _clamp(force[0], max_horizontal)
        force[1] = _clamp(force[1], max_horizontal)
        force[2] = max(floor_normal, min(max_normal, float(raw[2])))
        if not np.allclose(force, raw, rtol=0.0, atol=1.0e-12):
            clamped.append(code)
        forces[code] = force
    achieved = np.zeros(6, dtype=float)
    for index, code in enumerate(codes):
        achieved[:3] += forces[code]
        achieved[3:] += np.cross(radii[index], forces[code])
    return {
        "forces": forces,
        "clamped_legs": clamped,
        "residual": {
            "force_n": [float(target[i] - achieved[i]) for i in range(3)],
            "torque_nm": [float(target[3 + i] - achieved[3 + i]) for i in range(3)],
        },
    }


def leg_joint_torques(foot_force, jacobian, joints):
    """单腿**地面反力** → 该腿关节力矩 `τ = −Jᵀf`（`J` 为 3×3 平动雅可比，列顺序 = `joints`）。

    符号约定（**实测踩过，勿改回去**）：`foot_force` 是**地面作用在足端上的力**（向上 +z 为正），
    即机器人希望从地面获得的力。`Jᵀf` 给出的是「执行器把力 `f` 施加到**足端**」所需的关节力矩，
    而要让地面以 `f` 顶住足端，机构必须对足端施加 `−f` ⇒ 关节力矩要取负号。
    首跑漏了这个负号的后果（实测）：高度纠正与水平阻尼全部变成**正反馈**
    （静态抗扰 max_tilt 1.696° → 4.072°，随后 161.68° 翻倒、机身 −17.313 m）。
    """
    jacobian = np.asarray(jacobian, dtype=float)
    if jacobian.shape != (3, 3):
        raise DeclarationError(
            "腿部雅可比必须是 3×3（三关节 × 三方向），实际形状: %r" % (jacobian.shape,)
        )
    torque = -jacobian.T.dot(np.asarray(foot_force, dtype=float).reshape(3))
    return {str(joint): float(torque[index]) for index, joint in enumerate(joints)}
