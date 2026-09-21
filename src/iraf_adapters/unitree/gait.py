"""参数化步态生成器（步骤 02）：相位 → 足端轨迹 → 关节角（解析 IK）。

支持 `gait.kind` = `trot`（对角小跑，动态）与 `wave`（准静态四相位，首期步态）；
两者的差异**全部**在声明里（相位偏移、占空比、步高/步频），代码路径共用。

本模块只做三件事，全部**声明驱动**：

1. **解析并校验 `gait` 段**（`load_gait_declaration`）：步态类型、步频、步高、占空比、
   摆动轨迹形状、幅度斜坡、每条腿的关节绑定与接触几何、相位偏移，以及验收判据。
   缺键、越界、相位不自洽（对角配对不成立）一律显式失败（`DeclarationError`），
   不做任何默认值兜底（铁律 1.3 / 5.3）。
2. **几何实测**（`measure_leg_geometry`）：大腿/小腿长度、中立足端位置、关节轴方向
   全部**从被测模型量出来**，而不是把某个约定写死进代码。实测值与文档约定不符即失败
   （例如本模块的平面 IK 只在髋关节轴为 ±x、膝/踝轴为 ±y 时成立）。
3. **相位 → 关节角**（`foot_offset` / `leg_ik` / `gait_joint_targets`）与**验收判定**
   （`assess_gait`）：目标角必须在 Profile 声明的关节限位内，否则显式失败；验收判据
   与阈值全部来自声明（倾角上限直接消费安全策略里的移动边界，不写第二份数字）。

分层与边界
----------
- 本模块**不含任何控制律**：PD + 重力前馈由 `quadruped.pd_torque` /
  `quadruped.gravity_bias_torque` 提供，适配器（Provider）是唯一调用点。
- 本模块不接触安全策略文件读取（那是 `iraf_skills.quadruped` 的职责）：倾角上限由调用方
  以参数传入，保证「同一事实只有一处声明」。
- 结论一律属于**仿真**（`simulation: true`）；真机/目标端验收在本战役中 DEFERRED。

实测依据（本机 `/usr/bin/python3` + 场景产物 `build/scenes/handoff_lab/handoff_lab.xml`）
----------------------------------------------------------------------------------------
- 髋关节轴 `(±1,0,0)`、大腿/小腿关节轴 `(0,±1,0)`；大腿长 = 小腿长 = 0.213 m（`body_pos`）。
- 中立位形（关键帧 `home`：thigh=0.9, calf=-1.8）下足端相对大腿关节锚点为
  `(0, 0, -0.26480585)` m，即四条腿在同一平面内、无侧向偏移 ⇒ 平面 IK 成立。
- 足端接触几何**挂在 `*_calf` body 上**（厂商命名 `FL`/`FR`/`RL`/`RR`，球半径 0.022 m），
  因此接触几何名按声明给出，不按 body 名猜测。
"""

import math

import numpy as np

from iraf_adapters.unitree.quadruped import (
    CommandRejectedError,
    DeclarationError,
    ModelUnavailableError,
)

#: 已实现的步态类型（其余类型声明即失败，不做近似）：
#:
#: - `trot`：对角小跑——两条对角腿同时摆动，同一时刻只有一组（两条腿）支撑，是**动态**步态，
#:   需要机身平衡器才能稳定；
#: - `wave`：准静态四相位步态——一次只抬一条腿，任意时刻至少 `n-1` 条腿支撑，
#:   重心始终落在支撑多边形内 ⇒ **静态稳定**，不需要平衡器（首期步态，ADR-0008 决策 1）。
GAIT_KINDS = ("trot", "wave")

#: wave 的相位偏移等间隔（四相位：0 / 0.25 / 0.5 / 0.75）。
WAVE_PHASE_SPACING = 0.25

#: 已实现的摆动相抬脚轨迹形状。`sine` = 半个正弦：`h·sin(πv)`，落地/离地时刻高度为 0。
SWING_PROFILES = ("sine", "cosine")

#: `gait` 段必需键（缺任一即显式失败）。
REQUIRED_GAIT_KEYS = (
    "kind",
    "frequency_hz",
    "step_height_m",
    "duty_factor",
    "swing_profile",
    "ramp_s",
    "stabilization",
    "legs",
    "verification",
)

#: `gait.stabilization` 必需键（机身阻尼参数；缺键即失败，不允许实现层默认值）。
REQUIRED_STABILIZATION_KEYS = ("enabled", "linear_damping_s", "max_linear_offset_m")

#: 每条腿必需键。
REQUIRED_LEG_KEYS = ("hip_joint", "thigh_joint", "calf_joint", "contact_geom", "phase_offset")

#: `gait.verification` 必需键（验收判据与报告路径都在声明里）。
REQUIRED_VERIFICATION_KEYS = (
    "report",
    "duration_s",
    "height_target_m",
    "height_mean_tolerance_m",
    "height_std_max_m",
    "fall_base_height_m",
    "max_displacement_m",
    "contact_force_threshold_n",
    "min_swing_fraction",
    "min_clear_swing_cycles_per_leg",
    "duty_tolerance",
    "diagonal_profile_tolerance",
    "max_tracking_error_rad",
    "profile_bins",
)

#: 相位环的判定容差：两条腿的 `phase_offset` 相差小于它即视为「同相」。
PHASE_GROUP_TOLERANCE = 1.0e-9

#: 几何自检容差：足端相对大腿锚点的侧向偏移（平面 IK 前提）。
LEG_PLANE_TOLERANCE_M = 1.0e-6


class _Checks:
    """判据收集器（与 loopback 的同类实现同形：value / expectation / passed / detail）。"""

    def __init__(self):
        self.items = []

    def add(self, name, value, expectation, passed, detail):
        self.items.append(
            {
                "name": str(name),
                "value": value,
                "expectation": str(expectation),
                "passed": bool(passed),
                "detail": str(detail),
            }
        )
        return bool(passed)


def _positive(value, label, *, allow_zero=False):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise DeclarationError("%s 必须是数字，实际: %r" % (label, value))
    if number < 0.0 or (number == 0.0 and not allow_zero):
        raise DeclarationError(
            "%s 必须%s，实际: %r" % (label, "非负" if allow_zero else "为正数", value)
        )
    return number


def load_gait_declaration(declaration, profile_joints):
    """解析并校验 `gait` 段；返回规范化后的参数字典。缺键/越界/不自洽即显式失败。"""
    section = (declaration or {}).get("gait")
    if not isinstance(section, dict):
        raise DeclarationError(
            "声明缺少 gait 段（步态参数必须显式声明，禁止实现层默认值兜底）"
        )
    missing = [key for key in REQUIRED_GAIT_KEYS if key not in section]
    if missing:
        raise DeclarationError("gait 段缺少必需键: %s" % missing)

    kind = str(section["kind"])
    if kind not in GAIT_KINDS:
        raise DeclarationError(
            "gait.kind 只支持 %s（其余步态类型未实现，不做近似），实际: %r"
            % (list(GAIT_KINDS), kind)
        )

    frequency_hz = _positive(section["frequency_hz"], "gait.frequency_hz")
    step_height_m = _positive(section["step_height_m"], "gait.step_height_m")
    ramp_s = _positive(section["ramp_s"], "gait.ramp_s", allow_zero=True)

    try:
        duty_factor = float(section["duty_factor"])
    except (TypeError, ValueError):
        raise DeclarationError("gait.duty_factor 必须是数字，实际: %r" % (section["duty_factor"],))

    swing_profile = str(section["swing_profile"])
    if swing_profile not in SWING_PROFILES:
        raise DeclarationError(
            "gait.swing_profile 只支持 %s，实际: %r" % (list(SWING_PROFILES), swing_profile)
        )

    stabilization = section["stabilization"]
    if not isinstance(stabilization, dict):
        raise DeclarationError("gait.stabilization 必须是映射（机身阻尼参数必须显式声明）")
    s_missing = [key for key in REQUIRED_STABILIZATION_KEYS if key not in stabilization]
    if s_missing:
        raise DeclarationError("gait.stabilization 缺少必需键: %s" % s_missing)

    legs_section = section["legs"]
    if not isinstance(legs_section, dict) or not legs_section:
        raise DeclarationError("gait.legs 必须是非空映射（腿部身份只能来自声明）")
    if len(legs_section) != 4:
        raise DeclarationError(
            "gait.legs 必须恰好声明 4 条腿（对角小跑的步态定义要求四足），实际: %d" % len(legs_section)
        )

    # 占空比 = 一个周期里处于支撑相的比例。下限由**步态定义**决定（不是调参量，更不是实现层默认值）：
    # - trot：一次两条对角腿摆动，支撑相少于半个周期就会出现「少于两条腿着地」乃至腾空相
    #   （四足在该时刻无支撑 ⇒ 必然塌落）；
    # - wave：一次只抬一条腿，静态稳定要求任意时刻至少 n−1 条腿支撑 ⇒ duty ≥ (n−1)/n（四足 = 0.75）。
    # 因此把「一次只抬一条腿」这条不可违反的几何约束写成声明门禁：duty 更小意味着两条腿会同时摆动。
    n_legs = len(legs_section)
    if kind == "wave":
        duty_lower = float(n_legs - 1) / float(n_legs)
        duty_reason = (
            "wave（准静态）要求任意时刻至少 %d 条腿支撑（一次只抬一条腿），"
            "占空比不得小于 (n−1)/n = %.6f" % (n_legs - 1, duty_lower)
        )
    else:
        duty_lower = 0.5
        duty_reason = "对角小跑要求支撑相不少于半个周期（<0.5 会出现无支撑的腾空相）"
    if not (duty_lower <= duty_factor < 1.0):
        raise DeclarationError(
            "gait.duty_factor 必须落在 [%.6f, 1.0)，实际: %r（%s）"
            % (duty_lower, duty_factor, duty_reason)
        )

    known_joints = set(str(item) for item in (profile_joints or ()))
    if not known_joints:
        raise DeclarationError("Profile 未声明任何关节：无法校验步态的关节绑定")

    legs = {}
    for code, leg in legs_section.items():
        code = str(code)
        if not isinstance(leg, dict):
            raise DeclarationError("gait.legs.%s 必须是映射" % code)
        leg_missing = [key for key in REQUIRED_LEG_KEYS if key not in leg]
        if leg_missing:
            raise DeclarationError("gait.legs.%s 缺少必需键: %s" % (code, leg_missing))
        bound = {key: str(leg[key]) for key in ("hip_joint", "thigh_joint", "calf_joint")}
        for key, joint in bound.items():
            if joint not in known_joints:
                raise DeclarationError(
                    "gait.legs.%s.%s=%s 不在 Profile 关节清单内（关节身份只能来自 Profile）"
                    % (code, key, joint)
                )
        if len(set(bound.values())) != 3:
            raise DeclarationError("gait.legs.%s 的 hip/thigh/calf 关节必须互不相同" % code)
        offset = float(leg["phase_offset"])
        if not (0.0 <= offset < 1.0):
            raise DeclarationError(
                "gait.legs.%s.phase_offset 必须落在 [0, 1)，实际: %r" % (code, leg["phase_offset"])
            )
        legs[code] = {
            "hip_joint": bound["hip_joint"],
            "thigh_joint": bound["thigh_joint"],
            "calf_joint": bound["calf_joint"],
            "contact_geom": str(leg["contact_geom"]),
            "phase_offset": offset,
        }

    all_joints = []
    for leg in legs.values():
        all_joints.extend([leg["hip_joint"], leg["thigh_joint"], leg["calf_joint"]])
    if len(set(all_joints)) != len(all_joints):
        raise DeclarationError("gait.legs 的关节绑定出现重复：同一关节不得属于两条腿")

    # 相位结构自洽门禁（按步态类型分派，都是**声明不可违反**的几何约束）：
    # - trot：恰好两组、每组两条腿、组间相位差半个周期（对角配对）；
    # - wave：每条腿一个独立相位、四相位等间隔 0.25 ⇒ 任意时刻恰好一条腿处于摆动相。
    groups = {}
    for code, leg in legs.items():
        groups.setdefault(round(leg["phase_offset"], 9), []).append(code)
    reference = sorted(groups)
    if kind == "trot":
        if len(reference) != 2:
            raise DeclarationError(
                "对角小跑要求相位偏移恰好分成两组，实际 %d 组: %s" % (len(reference), reference)
            )
        delta = abs((reference[1] - reference[0]) % 1.0)
        if abs(delta - 0.5) > 1.0e-6:
            raise DeclarationError(
                "对角小跑要求两组相位偏移相差半个周期（0.5），实际相差: %r" % delta
            )
        for offset in reference:
            members = groups[offset]
            if len(members) != 2:
                raise DeclarationError(
                    "相位偏移 %r 的腿数量必须为 2（对角配对），实际: %s" % (offset, sorted(members))
                )
    else:
        if len(reference) != n_legs:
            raise DeclarationError(
                "wave（准静态）要求每条腿各有独立的相位偏移（一次只抬一条腿），"
                "实际只有 %d 个相位偏移: %s" % (len(reference), reference)
            )
        for offset in reference:
            if len(groups[offset]) != 1:
                raise DeclarationError(
                    "wave 的每个相位偏移只能绑定 1 条腿，相位偏移 %r 上绑定了: %s"
                    % (offset, sorted(groups[offset]))
                )
        spacing = [
            (reference[(index + 1) % n_legs] - reference[index]) % 1.0
            for index in range(n_legs)
        ]
        if max(abs(value - WAVE_PHASE_SPACING) for value in spacing) > 1.0e-9:
            raise DeclarationError(
                "wave 要求相位偏移等间隔 %r（四相位 0/0.25/0.5/0.75），实际相邻间隔: %s"
                % (WAVE_PHASE_SPACING, [round(value, 9) for value in spacing])
            )

    verification = section["verification"]
    if not isinstance(verification, dict):
        raise DeclarationError("gait.verification 必须是映射（验收判据只能来自声明）")
    v_missing = [key for key in REQUIRED_VERIFICATION_KEYS if key not in verification]
    if v_missing:
        raise DeclarationError("gait.verification 缺少必需键: %s" % v_missing)
    bins = verification["profile_bins"]
    if int(bins) < 4 or int(bins) % 2 != 0:
        raise DeclarationError(
            "gait.verification.profile_bins 必须是 ≥4 的偶数（相位环要能对折比较半周期），实际: %r"
            % (bins,)
        )
    duty_tolerance = float(verification["duty_tolerance"])
    if not (0.0 <= duty_tolerance < 1.0):
        raise DeclarationError(
            "gait.verification.duty_tolerance 必须落在 [0, 1)，实际: %r"
            "（它是「稳态支撑相比例 vs 声明占空比」的对称容差，不是占空比本身）"
            % (verification["duty_tolerance"],)
        )

    return {
        "kind": kind,
        "frequency_hz": frequency_hz,
        "period_s": 1.0 / frequency_hz,
        "step_height_m": step_height_m,
        "duty_factor": duty_factor,
        "swing_profile": swing_profile,
        "ramp_s": ramp_s,
        "stabilization": {
            "enabled": bool(stabilization["enabled"]),
            "linear_damping_s": _positive(
                stabilization["linear_damping_s"],
                "gait.stabilization.linear_damping_s",
                allow_zero=True,
            ),
            "max_linear_offset_m": _positive(
                stabilization["max_linear_offset_m"],
                "gait.stabilization.max_linear_offset_m",
            ),
        },
        "legs": legs,
        # 相位组（按相位偏移升序）：trot 得到 2 组、每组 2 条腿，wave 得到 4 组、每组 1 条腿。
        # 判据（`assess_gait`）按该结构逐组比较，不再假定「只有两组」。
        "phase_groups": [
            {"offset": float(offset), "legs": sorted(groups[offset])} for offset in reference
        ],
        "verification": {
            "report": str(verification["report"]),
            "duration_s": _positive(verification["duration_s"], "gait.verification.duration_s"),
            "height_target_m": float(verification["height_target_m"]),
            "height_mean_tolerance_m": _positive(
                verification["height_mean_tolerance_m"], "gait.verification.height_mean_tolerance_m"
            ),
            "height_std_max_m": _positive(
                verification["height_std_max_m"], "gait.verification.height_std_max_m"
            ),
            "fall_base_height_m": float(verification["fall_base_height_m"]),
            "max_displacement_m": _positive(
                verification["max_displacement_m"], "gait.verification.max_displacement_m"
            ),
            "contact_force_threshold_n": _positive(
                verification["contact_force_threshold_n"],
                "gait.verification.contact_force_threshold_n",
            ),
            "min_swing_fraction": float(verification["min_swing_fraction"]),
            "min_clear_swing_cycles_per_leg": int(verification["min_clear_swing_cycles_per_leg"]),
            "duty_tolerance": float(verification["duty_tolerance"]),
            "diagonal_profile_tolerance": float(verification["diagonal_profile_tolerance"]),
            "max_tracking_error_rad": _positive(
                verification["max_tracking_error_rad"], "gait.verification.max_tracking_error_rad"
            ),
            "profile_bins": int(bins),
        },
    }


# ---- 几何：全部从被测模型实测，不写死约定 ----


def _body_of_joint(model, mujoco, joint_id):
    body = int(model.jnt_bodyid[joint_id])
    if body < 0:
        raise ModelUnavailableError("关节 %d 不属于任何 body" % joint_id)
    return body


def _sole_child_body(model, calf_body, code):
    """小腿 body 的**唯一无关节子 body** = 足端 body（厂商 MJCF 的 `*_foot`）。"""
    children = [index for index in range(model.nbody) if int(model.body_parentid[index]) == calf_body]
    jointed = set(int(model.jnt_bodyid[index]) for index in range(model.njnt))
    sole = [index for index in children if index not in jointed]
    if len(sole) != 1:
        raise ModelUnavailableError(
            "腿 %s 的小腿 body（id=%d）下有 %d 个无关节子 body，无法唯一确定足端"
            % (code, calf_body, len(sole))
        )
    return sole[0]


def trunk_body_id(model, mujoco, params):
    """躯干 body 的 id（由声明里的髋关节反推：髋关节 body 的父节点）。

    用途：把机身世界速度变换到**躯干坐标系**（阻尼偏移必须是机身系量），
    不假设躯干 body 的名字（名字是厂家事实，来自 Profile 的 model.trunk_body 只用于挂载点）。
    """
    codes = sorted(params["legs"])
    if not codes:
        raise DeclarationError("gait.legs 为空：无法确定躯干坐标系")
    hip_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, params["legs"][codes[0]]["hip_joint"]
    )
    if hip_joint < 0:
        raise ModelUnavailableError(
            "腿 %s 的髋关节 %s 不在被测模型里：无法确定躯干坐标系"
            % (codes[0], params["legs"][codes[0]]["hip_joint"])
        )
    hip_body = int(model.jnt_bodyid[hip_joint])
    trunk = int(model.body_parentid[hip_body])
    if trunk < 0:
        raise ModelUnavailableError("髋关节 body 没有父节点：无法确定躯干坐标系")
    return trunk


def measure_leg_geometry(model, data, mujoco, params):
    """实测每条腿的几何：L1/L2、中立足端位置（躯干系 x-z）、关节轴方向校验。

    失败一律显式（模型结构变了就必须重测，不能沿用旧约定）。
    """
    geometry = {}
    for code, leg in params["legs"].items():
        ids = {}
        for key in ("hip_joint", "thigh_joint", "calf_joint"):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, leg[key])
            if joint_id < 0:
                raise ModelUnavailableError("腿 %s 的关节 %s 不在被测模型里" % (code, leg[key]))
            ids[key] = int(joint_id)

        # 平面 IK 的前提：髋关节轴 ±x、大腿/小腿关节轴 ±y（实测后校验，不假设）。
        hip_axis = np.asarray(model.jnt_axis[ids["hip_joint"]], dtype=float)
        if abs(abs(hip_axis[0]) - 1.0) > 1.0e-6 or abs(hip_axis[1]) > 1.0e-6 or abs(hip_axis[2]) > 1.0e-6:
            raise ModelUnavailableError(
                "腿 %s 的髋关节轴为 %s：平面 IK 只在髋轴为 ±x 时成立" % (code, hip_axis)
            )
        for key in ("thigh_joint", "calf_joint"):
            axis = np.asarray(model.jnt_axis[ids[key]], dtype=float)
            if abs(axis[1]) < 1.0 - 1.0e-6 or abs(axis[0]) > 1.0e-6 or abs(axis[2]) > 1.0e-6:
                raise ModelUnavailableError(
                    "腿 %s 的 %s 轴为 %s：平面 IK 只在膝/踝轴为 ±y 时成立" % (code, key, axis)
                )

        thigh_body = _body_of_joint(model, mujoco, ids["thigh_joint"])
        calf_body = _body_of_joint(model, mujoco, ids["calf_joint"])
        if int(model.body_parentid[calf_body]) != thigh_body:
            raise ModelUnavailableError(
                "腿 %s 的小腿 body（%d）不是大腿 body（%d）的子节点" % (code, calf_body, thigh_body)
            )
        l1 = float(np.linalg.norm(model.body_pos[calf_body]))
        foot_body = _sole_child_body(model, calf_body, code)
        l2 = float(np.linalg.norm(model.body_pos[foot_body]))
        if l1 <= 0.0 or l2 <= 0.0:
            raise ModelUnavailableError("腿 %s 的连杆长度为 0（L1=%r, L2=%r）" % (code, l1, l2))

        contact_geom = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, leg["contact_geom"]
        )
        if contact_geom < 0:
            raise ModelUnavailableError("腿 %s 的接触几何 %s 不在模型里" % (code, leg["contact_geom"]))

        # 中立足端位置：相对大腿关节锚点，投影到**躯干**坐标系的 x-z 平面。
        hip_body = _body_of_joint(model, mujoco, ids["hip_joint"])
        trunk_body = int(model.body_parentid[hip_body])
        rotation = np.asarray(data.xmat[trunk_body], dtype=float).reshape(3, 3)
        anchor = np.asarray(data.xanchor[ids["thigh_joint"]], dtype=float)
        foot_pos = np.asarray(data.xpos[foot_body], dtype=float)
        rel = rotation.T.dot(foot_pos - anchor)
        rel_trunk = rotation.T.dot(foot_pos - np.asarray(data.xpos[trunk_body], dtype=float))
        if abs(float(rel[1])) > LEG_PLANE_TOLERANCE_M:
            raise ModelUnavailableError(
                "腿 %s 的足端在躯干系内有侧向偏移 %.3e m：平面 IK 前提不成立"
                % (code, float(rel[1]))
            )
        if float(rel[2]) >= 0.0:
            raise ModelUnavailableError(
                "腿 %s 的中立足端不在大腿锚点下方（z=%r）：腿部几何与声明不符"
                % (code, float(rel[2]))
            )
        geometry[code] = {
            "l1_m": l1,
            "l2_m": l2,
            "neutral_x_m": float(rel[0]),
            "neutral_z_m": float(rel[2]),
            # 足端相对**躯干原点**的三维位置（机身系）：姿态阻尼要用它算 ω × r。
            "trunk_rel_m": [
                float(rel_trunk[0]),
                float(rel_trunk[1]),
                float(rel_trunk[2]),
            ],
            "foot_body": int(foot_body),
            "contact_geom": int(contact_geom),
            "joints": {
                "hip_joint": leg["hip_joint"],
                "thigh_joint": leg["thigh_joint"],
                "calf_joint": leg["calf_joint"],
            },
        }
    return geometry


# ---- 相位 -> 足端偏移 -> 关节角 ----


def foot_offset(phase, params, amplitude=1.0):
    """相位 → 足端相对中立位置的偏移 `(dx_m, dz_m)`（+z 为抬脚）。

    原地踏步：**位移目标恒为 0**（`dx = 0`），只做支撑/摆动切换；
    支撑相足端停在中立位置，摆动相按声明形状抬起 `step_height_m × amplitude`。
    """
    duty = float(params["duty_factor"])
    height = float(params["step_height_m"]) * float(amplitude)
    u = float(phase) % 1.0
    if u < duty:
        return 0.0, 0.0
    progress = (u - duty) / (1.0 - duty)
    shape = str(params["swing_profile"])
    if shape == "sine":
        # 半个正弦：离地/落地高度为 0，但两端速度非零（落地速度 ≈ h·π/((1−duty)·T)）。
        return 0.0, height * math.sin(math.pi * progress)
    if shape == "cosine":
        # 1−cos 型：两端高度与**速度**都为 0（落地无冲击），代价是峰值速度更高。
        return 0.0, 0.5 * height * (1.0 - math.cos(2.0 * math.pi * progress))
    raise DeclarationError("未实现的摆动轨迹形状: %r" % shape)


def leg_ik(px_m, pz_m, l1_m, l2_m):
    """平面二连杆逆解（膝向后弯，与实测中立位形同一支）。

    约定：足端相对大腿关节锚点的位移 `(px, pz)`（`pz < 0` 表示在下方），
    大腿角 q1、小腿角 q2 绕 +y 轴旋转，连杆方向为 `(-sin q, -cos q)`。
    实测校验：`leg_ik(0, -0.26480585, 0.213, 0.213) = (0.9, -1.8)`（关键帧 `home`）。
    """
    px = float(px_m)
    pz = float(pz_m)
    l1 = float(l1_m)
    l2 = float(l2_m)
    if l1 <= 0.0 or l2 <= 0.0:
        raise DeclarationError("连杆长度必须为正数: L1=%r, L2=%r" % (l1_m, l2_m))
    radius = math.hypot(px, pz)
    reach_min = abs(l1 - l2)
    reach_max = l1 + l2
    if radius < 1e-12:
        raise CommandRejectedError("足端目标与大腿锚点重合：逆解无定义")
    if radius > reach_max - 1e-9 or radius < reach_min + 1e-9:
        raise CommandRejectedError(
            "足端目标超出腿部可达范围: |p|=%.6f m 不在 (%.6f, %.6f) 内" % (radius, reach_min, reach_max)
        )
    psi = math.atan2(-px, -pz)
    cos_beta = (radius * radius + l1 * l1 - l2 * l2) / (2.0 * radius * l1)
    beta = math.acos(max(-1.0, min(1.0, cos_beta)))
    q1 = psi + beta
    knee_x = -l1 * math.sin(q1)
    knee_z = -l1 * math.cos(q1)
    q2 = math.atan2(-(px - knee_x), -(pz - knee_z)) - q1
    return q1, q2


def leg_forward_kinematics(q1, q2, l1, l2):
    """平面正解：供单测做往返校验（IK ∘ FK = 恒等）。"""
    knee_x = -l1 * math.sin(q1)
    knee_z = -l1 * math.cos(q1)
    foot_x = knee_x - l2 * math.sin(q1 + q2)
    foot_z = knee_z - l2 * math.cos(q1 + q2)
    return foot_x, foot_z


def leg_solve(px_m, py_m, pz_m, l1_m, l2_m):
    """三维足端目标 → (hip, thigh, calf) 关节角。

    髋关节绕躯干 +x 转 q_hip 时，矢状面内的点 `(px, pz_s)` 映射为
    `(px, −pz_s·sin q_hip, pz_s·cos q_hip)`；要求它等于目标 `(px, py, pz)` 即得
    `q_hip = atan2(py, −pz)` 与 `pz_s = −hypot(py, pz)`（精确，无小角近似）。
    """
    px = float(px_m)
    py = float(py_m)
    pz = float(pz_m)
    if pz >= 0.0:
        raise CommandRejectedError("足端目标必须在大腿关节锚点下方（pz < 0），实际: %r" % pz)
    q_hip = math.atan2(py, -pz)
    q1, q2 = leg_ik(px, -math.hypot(py, pz), l1_m, l2_m)
    return q_hip, q1, q2


def leg_phase(params, code, elapsed_s):
    """该腿在 `elapsed_s` 时刻的相位（0→1 循环，含声明偏移）。"""
    period = float(params["period_s"])
    offset = float(params["legs"][code]["phase_offset"])
    return ((float(elapsed_s) / period) + offset) % 1.0


def is_stance(params, phase):
    """相位是否处于支撑相（占空比来自声明）。"""
    return (float(phase) % 1.0) < float(params["duty_factor"])


def stabilization_offset(params, body_velocity_mps, body_omega_rad_s, trunk_rel_m):
    """机身阻尼的足端偏移量 `(dx_m, dy_m)`（步态稳定性的核心，见下）。

    为什么需要它：四足只用**对角两条腿**支撑时，关节空间（机身相对）的足端目标等价于
    「足端跟着机身走」——机身一旦有水平速度或角速度，支撑足被一起带走，形成正反馈。
    实测（`build/iraf-24h-2/02/scan2-run1.txt`、`scan4-stepheight.txt`）：
    抬脚高度 ≤0.01 m（足端未离开台面）时机身纹丝不动（漂移 0.006 m、倾角 0.30°），
    抬脚 ≥0.02 m（真正两条腿支撑）后 8 s 内沿 −x 漂移 0.65 m、速度 0.63 m/s 并失稳倒下。

    阻尼形式：让足端目标跟随**该足端处**的机身速度（平动 + 转动）：
    `r_cmd = neutral + k·(v_body + ω_body × r_foot)`。物理含义是把腿视作机身与地面锚点之间的
    弹性杆（力在机身上为 `k_l·(s − f − r_cmd)`），展开后阻尼项符号为负 ⇒ 抑制机身平动与转动；
    `k` 来自声明 `gait.stabilization.linear_damping_s`，偏移按声明上限截断。
    旋转项是必须的：只做平动阻尼时俯仰/滚转无阻尼（实测 8 组增益全部倒下）。
    """
    stabilization = params["stabilization"]
    if not stabilization["enabled"]:
        return 0.0, 0.0
    gain = float(stabilization["linear_damping_s"])
    limit = float(stabilization["max_linear_offset_m"])
    if gain == 0.0:
        return 0.0, 0.0
    omega = np.asarray(body_omega_rad_s, dtype=float)
    radius = np.asarray(trunk_rel_m, dtype=float)
    induced = np.cross(omega, radius)
    dx = gain * (float(body_velocity_mps[0]) + float(induced[0]))
    dy = gain * (float(body_velocity_mps[1]) + float(induced[1]))
    return (
        max(-limit, min(limit, dx)),
        max(-limit, min(limit, dy)),
    )


def gait_joint_targets(params, geometry, home, joint_limits, elapsed_s, amplitude=1.0,
                       body_velocity_mps=(0.0, 0.0, 0.0), body_omega_rad_s=(0.0, 0.0, 0.0)):
    """步态相位 → 12 个关节的目标角；目标越出 Profile 限位即显式失败。

    与 `gait.kind` 无关：相位→足端偏移（`foot_offset`）与支撑/摆动判定（`is_stance`）
    都只消费声明里的 duty 与相位偏移，因此 trot（两组各两条腿）与 wave（四相位各一条腿）
    共用同一条目标角生成路径（差异只在声明，不在代码）。

    `home`：Profile `spec.home`（髋关节由 IK 给出，需与 home 相加的偏移为零）。
    `joint_limits`：Profile 的关节限位；只允许在声明范围内活动（调用参数只能收紧，铁律 1.3）。
    `body_velocity_mps` / `body_omega_rad_s`：机身线速度与角速度（**机身坐标系**，
    由适配器从自由关节速度变换而来），仅用于声明化的阻尼偏移；缺省为零 = 无阻尼（不引入隐式行为）。
    """
    targets = {}
    for code, geom in geometry.items():
        phase = leg_phase(params, code, elapsed_s)
        dx, dz = foot_offset(phase, params, amplitude)
        damping = stabilization_offset(
            params, body_velocity_mps, body_omega_rad_s, geom["trunk_rel_m"]
        )
        q_hip, q1, q2 = leg_solve(
            geom["neutral_x_m"] + dx + damping[0],
            damping[1],
            geom["neutral_z_m"] + dz,
            geom["l1_m"],
            geom["l2_m"],
        )
        joints = geom["joints"]
        targets[joints["hip_joint"]] = float(home[joints["hip_joint"]]) + q_hip
        targets[joints["thigh_joint"]] = q1
        targets[joints["calf_joint"]] = q2
    for joint, value in targets.items():
        if joint not in joint_limits:
            raise DeclarationError("Profile 未声明关节 %s 的限位：不得下发目标" % joint)
        lower, upper = joint_limits[joint]
        if value < float(lower) - 1.0e-9 or value > float(upper) + 1.0e-9:
            raise CommandRejectedError(
                "步态目标角 %.6f rad 越出 Profile 对关节 %s 的限位 [%.6f, %.6f]：显式失败，不做截断"
                % (value, joint, float(lower), float(upper))
            )
    return targets


def trot_joint_targets(*args, **kwargs):
    """旧名薄包装（步骤 02 的 trot 路径仍按旧名调用）：语义与 `gait_joint_targets` 完全相同。"""
    return gait_joint_targets(*args, **kwargs)


def amplitude_at(params, elapsed_s):
    """幅度斜坡：0 → 1，时长来自声明（避免步态起步的阶跃输入）。"""
    ramp = float(params["ramp_s"])
    if ramp <= 0.0:
        return 1.0
    return min(1.0, max(float(elapsed_s), 0.0) / ramp)


# ---- 验收判定（阈值全部来自声明 + 安全策略传入的倾角上限） ----


def _stance_profile(samples, params, leg, bins):
    """稳态窗口内按相位分箱统计该腿的支撑相比例（相位环，值域 [0,1]）。"""
    period = float(params["period_s"])
    ramp = float(params["ramp_s"])
    threshold = float(params["verification"]["contact_force_threshold_n"])
    onset = float(samples[0]["time_s"])
    total = [0] * bins
    stance = [0] * bins
    for sample in samples:
        elapsed = float(sample["time_s"]) - onset
        if elapsed < ramp:
            continue
        index = int(((elapsed / period) % 1.0) * bins) % bins
        total[index] += 1
        if float(sample["contact_n"][leg]) >= threshold:
            stance[index] += 1
    return [
        (float(stance[index]) / total[index]) if total[index] else 0.0 for index in range(bins)
    ]


def _clear_swing_cycles(samples, params, leg):
    """统计「该腿在一个周期内明确离地」的周期数。

    判据（与声明同源）：该周期内支撑相比例 ≤ `1 − min_swing_fraction`，
    即至少有 `min_swing_fraction` 个周期的时间明确离地。
    """
    period = float(params["period_s"])
    ramp = float(params["ramp_s"])
    threshold = float(params["verification"]["contact_force_threshold_n"])
    min_swing = float(params["verification"]["min_swing_fraction"])
    onset = float(samples[0]["time_s"])
    cycles = {}
    for sample in samples:
        elapsed = float(sample["time_s"]) - onset
        if elapsed < ramp:
            continue
        index = int(elapsed / period)
        bucket = cycles.setdefault(index, [0, 0])
        bucket[0] += 1
        if float(sample["contact_n"][leg]) >= threshold:
            bucket[1] += 1
    clear = 0
    for index in sorted(cycles):
        seen, in_contact = cycles[index]
        if not seen:
            continue
        if (float(in_contact) / float(seen)) <= 1.0 - min_swing:
            clear += 1
    return clear, len(cycles)


def assess_gait(samples, params, tilt_limit_deg):
    """按声明判据验收原地踏步（trot 与 wave 共用）；返回 `{checks, failed_checks, metrics}`。

    「接触序列结构」与「支撑腿数」两条判据都按**声明的相位结构**泛化，
    因此对 trot（两组各两条腿、跨组半周期折叠）与 wave（四相位各一条腿、组间等间隔 0.25）
    是同一段代码、同一组阈值：差异只在声明，不在判据实现。
    """
    verification = params["verification"]
    checks = _Checks()
    legs = sorted(params["legs"])
    if not samples:
        raise CommandRejectedError("采样为空：验收无依据，不得判为通过（评测依据不可用）")

    onset = float(samples[0]["time_s"])
    measured_duration = float(samples[-1]["time_s"]) - onset
    ramp = float(params["ramp_s"])
    steady = [sample for sample in samples if float(sample["time_s"]) - onset >= ramp]
    if not steady:
        raise CommandRejectedError("稳态窗口为空（ramp_s=%r 覆盖了全部采样）：验收无依据" % ramp)

    heights = [float(sample["base_height_m"]) for sample in samples]
    steady_heights = [float(sample["base_height_m"]) for sample in steady]
    steady_tilts = [float(sample["tilt_deg"]) for sample in steady]
    height_mean = float(np.mean(steady_heights))
    height_std = float(np.std(steady_heights))
    max_tilt = float(np.max(steady_tilts))
    min_height = float(np.min(heights))
    saturated = int(sum(int(sample["ctrl_saturated"]) for sample in samples))
    tracking = float(np.max([float(sample["tracking_error_rad"]) for sample in samples]))
    origin = [float(item) for item in samples[0]["base_position_xy_m"]]
    displacements = [
        float(
            np.hypot(
                float(sample["base_position_xy_m"][0]) - origin[0],
                float(sample["base_position_xy_m"][1]) - origin[1],
            )
        )
        for sample in samples
    ]
    max_displacement = float(np.max(displacements))
    max_roll = float(np.max([abs(float(sample["roll_deg"])) for sample in steady]))
    max_pitch = float(np.max([abs(float(sample["pitch_deg"])) for sample in steady]))
    # 采样周期（中位数，抗离群）：只用于把「时长判据」的量化误差讲清楚。
    stamps = [float(sample["time_s"]) for sample in samples]
    deltas = sorted(
        [stamps[index + 1] - stamps[index] for index in range(len(stamps) - 1)]
    )
    sample_interval = float(np.median(deltas)) if deltas else 0.0

    checks.add(
        "duration_s",
        measured_duration,
        ">= 声明 %r s − 一个采样周期（量化）" % verification["duration_s"],
        measured_duration >= float(verification["duration_s"]) - sample_interval - 1.0e-6,
        "实测持续时长 %.6f s（声明 %r s；末次采样在时长终点、首次采样在第一个控制周期之后，"
        "差值上限为一个采样周期 %.6f s，不是放宽判据）"
        % (measured_duration, verification["duration_s"], sample_interval),
    )
    checks.add(
        "min_base_height_m",
        min_height,
        ">= gait.verification.fall_base_height_m = %r" % verification["fall_base_height_m"],
        min_height >= float(verification["fall_base_height_m"]),
        "全称量最小机身高度 %.6f m（低于阈值即判为跌倒，PASS 只说明「没有塌到该高度以下」）"
        % min_height,
    )
    checks.add(
        "max_displacement_m",
        max_displacement,
        "<= gait.verification.max_displacement_m = %r" % verification["max_displacement_m"],
        max_displacement <= float(verification["max_displacement_m"]),
        "全程机身水平位移峰值 %.6f m（原地踏步的位移目标恒为 0；本判据证明「没有偷偷走出去」）"
        % max_displacement,
    )
    checks.add(
        "height_mean_m",
        height_mean,
        "|mean - %r| <= %r" % (verification["height_target_m"], verification["height_mean_tolerance_m"]),
        abs(height_mean - float(verification["height_target_m"]))
        <= float(verification["height_mean_tolerance_m"]),
        "稳态机身高度均值 %.6f m（目标 %r m）" % (height_mean, verification["height_target_m"]),
    )
    checks.add(
        "height_std_m",
        height_std,
        "<= gait.verification.height_std_max_m = %r" % verification["height_std_max_m"],
        height_std <= float(verification["height_std_max_m"]),
        "稳态机身高度标准差 %.6f m（把「稳定踏步」与「上下弹跳」分开）" % height_std,
    )
    checks.add(
        "max_tilt_deg",
        max_tilt,
        "<= 安全策略 quadruped_limits.max_tilt_moving_deg = %r" % tilt_limit_deg,
        max_tilt <= float(tilt_limit_deg),
        "稳态最大倾角 %.6f°（相对竖直，取 acos(R[2,2]) 不含偏航；阈值来自安全策略，不写第二份）"
        % max_tilt,
    )
    checks.add(
        "ctrl_saturated_samples",
        saturated,
        "== 0（饱和即判据失败）",
        saturated == 0,
        "控制量被模型 ctrlrange 截断的采样次数（非零说明控制律在被限幅，证据不可用）",
    )
    checks.add(
        "max_tracking_error_rad",
        tracking,
        "<= gait.verification.max_tracking_error_rad = %r" % verification["max_tracking_error_rad"],
        tracking <= float(verification["max_tracking_error_rad"]),
        "全程 max|q − q_des| = %.6f rad（PD 跟踪误差；阈值来自声明）" % tracking,
    )

    threshold = float(verification["contact_force_threshold_n"])
    duty = float(params["duty_factor"])
    bins = int(verification["profile_bins"])
    tolerance = float(verification["diagonal_profile_tolerance"])
    min_clear = int(verification["min_clear_swing_cycles_per_leg"])

    per_leg = {}
    for leg in legs:
        flags = [float(sample["contact_n"][leg]) >= threshold for sample in samples]
        # 稳态窗口（去掉幅度斜坡）内的支撑相比例：斜坡期占空比会偏小，不能与声明 duty 直接比。
        steady_flags = [float(sample["contact_n"][leg]) >= threshold for sample in steady]
        clear, cycles = _clear_swing_cycles(samples, params, leg)
        per_leg[leg] = {
            "stance_fraction": float(np.mean(flags)) if flags else 0.0,
            "steady_stance_fraction": float(np.mean(steady_flags)) if steady_flags else 0.0,
            "clear_swing_cycles": int(clear),
            "cycles_observed": int(cycles),
            "max_contact_n": float(
                np.max([float(sample["contact_n"][leg]) for sample in samples])
            ),
            "stance_profile": _stance_profile(samples, params, leg, bins),
        }
        checks.add(
            "leg_%s_clear_swing_cycles" % leg,
            int(clear),
            ">= %d（该周期内离地比例 ≥ %r）"
            % (min_clear, verification["min_swing_fraction"]),
            clear >= min_clear,
            "腿 %s 在 %d 个稳态周期中有 %d 个周期离地 ≥ %r；全程支撑相比例 %.6f（声明 duty=%r）"
            % (leg, cycles, clear, verification["min_swing_fraction"],
               per_leg[leg]["stance_fraction"], duty),
        )
        duty_tolerance = float(verification["duty_tolerance"])
        steady_fraction = per_leg[leg]["steady_stance_fraction"]
        checks.add(
            "leg_%s_stance_duty" % leg,
            steady_fraction,
            "|支撑相比例 − duty| <= %r（声明 duty=%r）" % (duty_tolerance, duty),
            abs(steady_fraction - duty) <= duty_tolerance,
            "腿 %s 稳态支撑相比例 %.6f 与声明占空比 %r 的偏差必须在容差内："
            "「一直粘在地上」与「一直悬空」都要被拦下" % (leg, steady_fraction, duty),
        )

    # 相位结构判据（按声明泛化，trot 与 wave 共用）：
    # ① **组内一致**：同一相位偏移上的腿（trot 的同组两条腿），支撑相相位环必须逐格一致；
    # ② **组间按声明相位差平移后一致**：把后一组的相位环平移声明相位差对应的格数后，
    #    必须与前一组的相位环一致 ⇒ 观测到的接触序列与声明的相位顺序同序。
    # trot 退化为原有语义（同组 FL/RR 一致；两组按半周期 = bins/2 折叠一致），数值口径不变。
    phase_groups = params["phase_groups"]
    within = []
    for group in phase_groups:
        reference_leg = group["legs"][0]
        for other_leg in group["legs"][1:]:
            within.append(
                max(
                    abs(
                        per_leg[reference_leg]["stance_profile"][index]
                        - per_leg[other_leg]["stance_profile"][index]
                    )
                    for index in range(bins)
                )
            )
    pairs = []
    cross = 0.0
    for index in range(1, len(phase_groups)):
        previous = phase_groups[index - 1]
        current = phase_groups[index]
        delta = (current["offset"] - previous["offset"]) % 1.0
        # 平移方向（**符号很关键，四相位才暴露出来**）：腿的相位是 `(u + offset) % 1`，`u = 时间/周期`。
        # offset 更大 ⇒ 该腿的相位在**同一绝对时刻**更靠前 ⇒ 它的支撑/摆动窗口在绝对相位轴上
        # 出现得更**早**。因此本组（offset 更大）的相位环 = 前一组按 −shift 格平移：
        #   current[k] == reference[(k + shift) % bins]，其中 shift = round(Δ·bins)。
        # 注意 trot 的 Δ = 0.5：+shift 与 −shift 模 bins 相同（半周期在环上自逆），
        # 所以方向写反在 trot 下**完全不可见**，wave（Δ = 0.25）才让它显形
        # （实测三对相邻组 max_abs_diff 全为 1.0，证据 build/iraf-24h-2/02/diagnose-phase-structure.txt）。
        shift = int(round(delta * bins)) % bins
        reference_profile = per_leg[previous["legs"][0]]["stance_profile"]
        moved_profile = per_leg[current["legs"][0]]["stance_profile"]
        difference = max(
            abs(reference_profile[(k + shift) % bins] - moved_profile[k]) for k in range(bins)
        )
        cross = max(cross, difference)
        pairs.append(
            {
                "from": previous["legs"][0],
                "to": current["legs"][0],
                "declared_offset_delta": delta,
                "shift_bins": shift,
                "max_abs_diff": difference,
            }
        )
    checks.add(
        "phase_sequence_structure",
        {"within_group_max_diff": within, "cross_group_max_diff": cross, "pairs": pairs},
        "同组相位环最大差 <= %r，相邻组按声明相位差平移后的最大差 <= %r" % (tolerance, tolerance),
        (not within or max(within) <= tolerance) and cross <= tolerance,
        "接触序列必须与声明的相位顺序一致（把「声明式步态」与「四条腿乱抬」分开）：%s"
        % "；".join(
            "相位偏移 %r → 腿 %s" % (group["offset"], group["legs"]) for group in phase_groups
        ),
    )

    # 支撑腿数判据：把「两条腿同时抬起」（wave 的静态稳定性前提被破坏）拦下。
    # 目标值 = 声明 duty × 腿数（wave: 0.75×4 = 3 条；trot: 0.5×4 = 2 条），
    # 允许量与「单腿支撑相比例」用同一个声明容差（duty_tolerance）折算到腿数上。
    support_profile = [
        float(sum(per_leg[leg]["stance_profile"][index] for leg in legs))
        for index in range(bins)
    ]
    support_target = duty * float(len(legs))
    support_allowance = float(verification["duty_tolerance"]) * float(len(legs))
    checks.add(
        "support_legs_profile",
        {"min": float(min(support_profile)), "max": float(max(support_profile))},
        "任意相位上的平均支撑腿数 >= %.6f（= duty %r × %d 条腿 − duty_tolerance %r × %d）"
        % (support_target - support_allowance, duty, len(legs),
           verification["duty_tolerance"], len(legs)),
        min(support_profile) >= support_target - support_allowance,
        "随相位变化的平均支撑腿数 min=%.6f / max=%.6f（%s）"
        % (
            min(support_profile),
            max(support_profile),
            "wave 的静态稳定性要求任意时刻 ≥ %d 条腿支撑" % int(round(support_target))
            if params["kind"] == "wave"
            else "trot 要求两组对角腿交替支撑（任意时刻 2 条）",
        ),
    )

    failed = [item["name"] for item in checks.items if not item["passed"]]
    metrics = {
        "duration_s": measured_duration,
        "declared_duration_s": float(verification["duration_s"]),
        "height_mean_m": height_mean,
        "height_std_m": height_std,
        "min_base_height_m": min_height,
        "max_displacement_m": max_displacement,
        "max_tilt_deg": max_tilt,
        "max_roll_deg": max_roll,
        "max_pitch_deg": max_pitch,
        "max_tracking_error_rad": tracking,
        "ctrl_saturated_samples": saturated,
        "samples": len(samples),
        "steady_samples": len(steady),
        "contact_force_threshold_n": threshold,
        "per_leg": per_leg,
        "phase_groups": phase_groups,
        "support_legs": {"min": float(min(support_profile)), "max": float(max(support_profile))},
    }
    return {"checks": checks.items, "failed_checks": failed, "metrics": metrics}


def assess_trot(*args, **kwargs):
    """旧名薄包装（既有的 trot 调用点与单测仍按旧名调用）：语义与 `assess_gait` 完全相同。"""
    return assess_gait(*args, **kwargs)
