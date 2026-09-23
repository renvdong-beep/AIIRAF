"""参数化步态生成器（步骤 02）：相位 → 足端轨迹 → 关节角（解析 IK）。

支持 `gait.kind` = `trot`（对角小跑，动态）与 `wave`（准静态四相位，首期步态）；
两者的差异**全部**在声明里（相位偏移、占空比、步高/步频），代码路径共用。

本模块只做三件事，全部**声明驱动**：

1. **解析并校验 `gait` 段**（`load_gait_declaration`）：步态类型、步频、步高、占空比、
   摆动轨迹形状、幅度斜坡、每条腿的关节绑定与接触几何、相位偏移，以及验收判据；
   逐相位重心转移（`gait.sway`）与落足点规划（`gait.foothold`）也在同一处解析校验。
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
    "stance_clearance_m",
    "duty_factor",
    "swing_profile",
    "ramp_s",
    "stabilization",
    "legs",
    "verification",
)

#: `gait.stabilization` 必需键（机身阻尼参数；缺键即失败，不允许实现层默认值）。
#: `gait.stabilization` 必需键。
#:
#: 位置项三键（2026-09-23 新增，契约见 docs/debug/2026-09-23-go2-gait-drift.md §9）：
#: 纯速度反馈在 v→0 时修正量→0，**无法**压住低速残余漂移（实测 19 档声明内调参最好 0.5959 m，
#: 判据 0.05 m）⇒ 增加一个作用在**滑动平均位移**上的低频位置项。三键缺一即显式失败。
REQUIRED_STABILIZATION_KEYS = ("enabled", "linear_damping_s", "max_linear_offset_m",
                               "position_gain_s_per_m", "position_window_s", "position_limit_m")

#: `gait.sway` 必需键（逐相位重心转移；缺键即失败，不允许实现层默认值）。
#:
#: 为什么 wave 必须带它（实测，见 `docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md`）：
#: 对称矩形足迹下两条对角腿的连线**恒过足迹中心**，抬起任意一条腿后三腿支撑三角形与该连线的
#: 交集只有中心一点 ⇒ 支撑余量实测 `-0.000227483 m`（≈0）；且「常量重心平移」的可行域为**空集**
#: （对角腿对所需平移方向夹角 180.00°、两相位余量之和上确界 +0.000000000 m）。
#: 因此 wave 的静态稳定**只能**靠逐相位把重心投影移进当前支撑三角形（决策 A，2026-09-21）。
REQUIRED_SWAY_KEYS = (
    "amplitude_m",
    "axis",
    "phase_map",
    "smooth_s",
    "ramp_s",
    "direction_tolerance_deg",
)

#: `gait.sway.axis` 允许的轴（躯干系水平面；z 不参与重心转移）。
SWAY_AXES = ("x", "y")

#: 重心转移方向的实测校验容差允许区间（度）。上界防止把容差写成 180°
#: 从而让「方向门禁」退化成恒真门禁（门禁绿、计数 0 的同族缺陷）。
SWAY_DIRECTION_TOLERANCE_DEG_RANGE = (0.0, 45.0)

#: `gait.foothold.mode` 的允许取值（落足点规划，专项 C / ADR-0008 决策 1j）：
#:
#: - `static`：落点恒为中立位 = 现行「原地踏步」行为（既有两个入口的行为逐位不变）；
#: - `per_phase`：逐相位把**摆动腿的水平落点**挪到计划位置，让支撑三角形跟着移动，
#:   而不是平移机身（这是「静态支撑余量 ≈ 0」的正面解法：对称矩形足迹下四相位可行域交集
#:   为 0 点、机身常量平移不可行、重心到支撑三角形形心需平移 0.079~0.081 m，
#:   实测见 `docs/progress/2026-09-21-quadruped-walk-special-design.md` §3）。
FOOTHOLD_MODES = ("static", "per_phase")

#: `gait.foothold` 各模式的必需键。模式不同、语义不同；**多余键同样失败** ——
#: 对某模式无定义的键写进声明等于「假声明」，必须显式拒绝而不是静默忽略（同 `sway` 的取向）。
FOOTHOLD_MODE_KEYS = {
    "static": (),
    "per_phase": ("stride_m", "phase_direction_map", "phase_order", "ramp_s", "smooth_s"),
}

#: `gait.foothold` 段的公共必需键（各模式都要有）。
REQUIRED_FOOTHOLD_KEYS = ("mode",)

#: `gait.foothold.verification` 必需键（**落足点规划专项验收**的判据与阈值）。
#:
#: 为什么单独一套而不是复用 `gait.verification`：后者是「**原地踏步**」的口径
#: （`max_displacement_m: 0.05` 量的是全程机身位移峰值，注释写明「位移目标恒为 0，本判据证明
#: 没有偷偷走出去」）。迈步式 crawl **必然**在周期内真实平移（实测重心需移 0.079~0.081 m），
#: 留在原地口径下该路线永远不可能通过 —— 那不是严，是把判据套在错量上。因此 crawl 用**新增**判据集，
#: 既有 20 项判据与阈值一字不动（继续由 `verify_go2_gait_in_place.py` 强制执行）。
#:
#: - `report` / `duration_s`：报告路径与验收时长；
#: - `landing_error_max_m`：落点误差上限（实测足端躯干系位置差 vs 声明 `2·stride_m·方向`）；
#: - `min_stance_contact_rate`：稳态支撑相帧的接触达标率下限（∈ [0, 1]）；
#: - `net_drift_per_cycle_m`：**周期净漂移**上限（"原地"语义：足迹闭环 ⇒ 每周期漂移 → 0）；
#: - `peak_body_excursion_m`：稳态窗内机身水平位移**峰值**上限（周期内真实平移的上界）。
REQUIRED_FOOTHOLD_VERIFICATION_KEYS = (
    "report",
    "duration_s",
    "landing_error_max_m",
    "min_stance_contact_rate",
    "net_drift_per_cycle_m",
    "peak_body_excursion_m",
)

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


def _load_sway(section, legs, kind):
    """解析 `gait.sway`（逐相位重心转移）；返回规范化字典或 `None`。

    - `wave`：**必须**声明本段（静态稳定的前提，决策 A）；
    - 其它步态类型：**不得**声明本段（trot 的两条对角腿在同一窗口摆动，所需重心方向相反，
      逐相位转移无定义 ⇒ 声明即失败，而不是静默忽略）。
    """
    sway = section.get("sway")
    if sway is None:
        if kind == "wave":
            raise DeclarationError(
                "wave 必须声明 gait.sway（逐相位重心转移）：对称足迹下三腿支撑三角形的静态余量"
                "实测 -0.000227483 m（≈0）、常量重心平移可行域为空集 ⇒ 缺重心转移的 wave "
                "不满足静态稳定前提，显式失败而不是照跑"
            )
        return None
    if not isinstance(sway, dict):
        raise DeclarationError("gait.sway 必须是映射（缺段即失败，不允许默认值兜底）")
    if kind != "wave":
        raise DeclarationError(
            "gait.sway 只对 wave 生效：%s 的同一摆动窗口内有两条腿（对角腿对所需重心方向实测夹角 "
            "180.00°），逐相位重心转移无定义" % kind
        )
    missing = [key for key in REQUIRED_SWAY_KEYS if key not in sway]
    if missing:
        raise DeclarationError("gait.sway 缺少必需键: %s" % missing)

    axis = [str(item) for item in (sway["axis"] or [])]
    if not axis or any(item not in SWAY_AXES for item in axis):
        raise DeclarationError(
            "gait.sway.axis 只能取 %s 的非空子集，实际: %r" % (list(SWAY_AXES), sway["axis"])
        )

    amplitude = _positive(sway["amplitude_m"], "gait.sway.amplitude_m", allow_zero=True)
    smooth_s = _positive(sway["smooth_s"], "gait.sway.smooth_s", allow_zero=True)
    ramp_s = _positive(sway["ramp_s"], "gait.sway.ramp_s", allow_zero=True)
    tolerance = _positive(
        sway["direction_tolerance_deg"], "gait.sway.direction_tolerance_deg", allow_zero=True
    )
    low, high = SWAY_DIRECTION_TOLERANCE_DEG_RANGE
    if not (low <= tolerance <= high):
        raise DeclarationError(
            "gait.sway.direction_tolerance_deg 必须落在 [%r, %r]（否则方向门禁会退化成恒真门禁），"
            "实际: %r" % (low, high, sway["direction_tolerance_deg"])
        )

    phase_map = sway["phase_map"]
    if not isinstance(phase_map, dict):
        raise DeclarationError("gait.sway.phase_map 必须是映射（腿 → 重心转移方向，腿名来自声明）")
    if set(str(item) for item in phase_map) != set(legs):
        raise DeclarationError(
            "gait.sway.phase_map 的腿集合必须与 gait.legs 完全一致，实际: %r vs %r"
            % (sorted(str(item) for item in phase_map), sorted(legs))
        )
    directions = {}
    for code, value in phase_map.items():
        code = str(code)
        if isinstance(value, dict):
            keys = ("x", "y")
            if any(key not in value for key in keys):
                raise DeclarationError("gait.sway.phase_map.%s 必须是 {x, y} 数字" % code)
            vector = (float(value["x"]), float(value["y"]))
        elif isinstance(value, (list, tuple)) and len(value) == 2:
            vector = (float(value[0]), float(value[1]))
        else:
            raise DeclarationError(
                "gait.sway.phase_map.%s 必须是 {x, y} 映射或 [x, y] 两元序列" % code
            )
        norm = math.hypot(vector[0], vector[1])
        if norm <= 1.0e-12:
            raise DeclarationError(
                "gait.sway.phase_map.%s 的方向向量是零矢量：重心转移方向无定义" % code
            )
        directions[code] = (vector[0] / norm, vector[1] / norm)

    return {
        "amplitude_m": amplitude,
        "axis": axis,
        "smooth_s": smooth_s,
        "ramp_s": ramp_s,
        "direction_tolerance_deg": tolerance,
        "directions": directions,
    }


def phase_groups_of(legs):
    """按 `phase_offset` 把腿分组（升序）：返回 `(groups, reference)`。

    只消费「腿 → `phase_offset`」，**不依赖 Profile / 模型的关节清单** ——
    因此落足点校验（`load_foothold_declaration`）与完整声明校验（`load_gait_declaration`）
    可以共用同一份相位结构，不必各自实现一遍分组规则。
    """
    groups = {}
    for code, leg in legs.items():
        groups.setdefault(round(float(leg["phase_offset"]), 9), []).append(str(code))
    return groups, sorted(groups)


def _load_foothold(section, legs, groups, reference, kind):
    """解析 `gait.foothold`（落足点规划 / 迈步式 crawl）；返回规范化字典或 `None`。

    - `wave`：**必须**声明本段（理由与 `sway` 同源：对称矩形足迹下「抬起任意一条腿后
      三腿支撑三角形」的静态余量实测 `-0.000227483 m`（≈0），静态可稳只能靠逐相位
      重新构造支撑多边形）；
    - 其它步态类型：`static`（落点不变 = 现行为）可声明；`per_phase` **不得**声明 ——
      同一摆动窗口内有两条对角腿，逐相位落点无定义 ⇒ 声明即失败，而不是静默忽略。
    """
    foothold = section.get("foothold")
    if foothold is None:
        if kind == "wave":
            raise DeclarationError(
                "wave 必须声明 gait.foothold（落足点规划）：对称足迹下三腿支撑三角形的静态余量"
                "实测 -0.000227483 m（≈0）、机身常量平移的可行域为空集 ⇒ 「落点不变」的 wave "
                "没有静态稳定前提，显式失败而不是照跑（见 ADR-0008 决策 1j）"
            )
        return None
    if not isinstance(foothold, dict):
        raise DeclarationError("gait.foothold 必须是映射（缺段即失败，不允许默认值兜底）")

    missing = [key for key in REQUIRED_FOOTHOLD_KEYS if key not in foothold]
    if missing:
        raise DeclarationError("gait.foothold 缺少必需键: %s" % missing)

    mode = str(foothold["mode"])
    if mode not in FOOTHOLD_MODES:
        raise DeclarationError(
            "gait.foothold.mode 只支持 %s，实际: %r" % (list(FOOTHOLD_MODES), foothold["mode"])
        )

    # 必需键**按模式**判定：公共键 + 该模式的专属键。缺任一即显式失败（`DeclarationError`），
    # 不能等到下面按下标取值时抛 `KeyError` —— 那会以「崩溃」形式泄漏，而不是声明层失败。
    # 必需键**按模式**判定：公共键 + 该模式的专属键 + 验收判据段（两种模式都必须声明：
    # 判据描述的是「crawl 要达到什么」，与当前生效的模式无关，生产 static 也要如实登记）。
    allowed = set(REQUIRED_FOOTHOLD_KEYS) | set(FOOTHOLD_MODE_KEYS[mode]) | {"verification"}
    missing = sorted(key for key in allowed if key not in foothold)
    if missing:
        raise DeclarationError(
            "gait.foothold（mode=%s）缺少必需键: %s" % (mode, missing)
        )
    extra = sorted(str(key) for key in foothold if str(key) not in allowed)
    if extra:
        raise DeclarationError(
            "gait.foothold（mode=%s）出现无定义的键: %s（该模式下这些键无定义，"
            "写进声明等于假声明；允许的键: %s）"
            % (mode, extra, sorted(allowed))
        )

    section_verification = foothold["verification"]
    if not isinstance(section_verification, dict):
        raise DeclarationError("gait.foothold.verification 必须是映射（验收判据只能来自声明）")
    v_missing = [key for key in REQUIRED_FOOTHOLD_VERIFICATION_KEYS
                 if key not in section_verification]
    if v_missing:
        raise DeclarationError("gait.foothold.verification 缺少必需键: %s" % v_missing)
    contact_rate = float(section_verification["min_stance_contact_rate"])
    if not (0.0 <= contact_rate <= 1.0):
        raise DeclarationError(
            "gait.foothold.verification.min_stance_contact_rate 必须落在 [0, 1]"
            "（它是接触帧的**比例**下限，不是接触力阈值），实际: %r"
            % (section_verification["min_stance_contact_rate"],)
        )
    verification = {
        "report": str(section_verification["report"]),
        "duration_s": _positive(section_verification["duration_s"],
                                "gait.foothold.verification.duration_s"),
        "landing_error_max_m": _positive(section_verification["landing_error_max_m"],
                                         "gait.foothold.verification.landing_error_max_m"),
        "min_stance_contact_rate": contact_rate,
        "net_drift_per_cycle_m": _positive(section_verification["net_drift_per_cycle_m"],
                                           "gait.foothold.verification.net_drift_per_cycle_m"),
        "peak_body_excursion_m": _positive(section_verification["peak_body_excursion_m"],
                                           "gait.foothold.verification.peak_body_excursion_m"),
    }

    if mode == "static":
        # static = 落点恒为中立位，即现行「原地踏步」行为：没有任何水平落点参数可声明。
        return {"mode": "static", "verification": verification}

    # ---- mode == per_phase：逐相位落点规划 ----
    if kind != "wave":
        raise DeclarationError(
            "gait.foothold.mode=per_phase 只对 wave 生效：%s 的同一摆动窗口内有两条腿，"
            "逐相位落点无定义" % kind
        )

    stride = _positive(foothold["stride_m"], "gait.foothold.stride_m", allow_zero=True)
    ramp_s = _positive(foothold["ramp_s"], "gait.foothold.ramp_s", allow_zero=True)
    smooth_s = _positive(foothold["smooth_s"], "gait.foothold.smooth_s", allow_zero=True)

    # 环形顺序门禁：落点的更新顺序必须与相位偏移升序一致。写错顺序会让落点方向在相邻窗口之间
    # 翻转（对称相位环上 x 分量正负交替），表现为「门禁全绿但机身朝反方向走」——静默失效，
    # 因此在此显式失败（该缺陷族已在 sway 的隔离实验里踩过一次，见调试记录 §9）。
    raw_order = foothold["phase_order"]
    if not isinstance(raw_order, (list, tuple)) or not raw_order:
        raise DeclarationError(
            "gait.foothold.phase_order 必须是腿名序列（落点更新的环形顺序），实际: %r"
            % (foothold["phase_order"],)
        )
    order = [str(item) for item in raw_order]
    unknown = [code for code in order if code not in legs]
    if unknown:
        raise DeclarationError("gait.foothold.phase_order 含未声明的腿: %s" % unknown)
    if len(order) != len(legs) or len(set(order)) != len(order):
        raise DeclarationError(
            "gait.foothold.phase_order 必须恰好包含 %d 条腿各一次（环形顺序），实际: %r"
            % (len(legs), order)
        )
    expected = [sorted(groups[offset])[0] for offset in reference]
    if order != expected:
        raise DeclarationError(
            "gait.foothold.phase_order 必须与相位偏移升序一致，期望 %r，实际: %r"
            "（顺序错会让落点方向逐窗口翻转，属静默失效缺陷）" % (expected, order)
        )

    phase_map = foothold["phase_direction_map"]
    if not isinstance(phase_map, dict):
        raise DeclarationError(
            "gait.foothold.phase_direction_map 必须是映射（腿 → 该腿摆动相落点的行进方向，"
            "腿名来自声明）"
        )
    if set(str(item) for item in phase_map) != set(legs):
        raise DeclarationError(
            "gait.foothold.phase_direction_map 的腿集合必须与 gait.legs 完全一致，实际: %r vs %r"
            % (sorted(str(item) for item in phase_map), sorted(legs))
        )
    directions = {}
    for code, value in phase_map.items():
        code = str(code)
        if isinstance(value, dict):
            if any(key not in value for key in ("x", "y")):
                raise DeclarationError(
                    "gait.foothold.phase_direction_map.%s 必须是 {x, y} 数字" % code
                )
            vector = (float(value["x"]), float(value["y"]))
        elif isinstance(value, (list, tuple)) and len(value) == 2:
            vector = (float(value[0]), float(value[1]))
        else:
            raise DeclarationError(
                "gait.foothold.phase_direction_map.%s 必须是 {x, y} 映射或 [x, y] 两元序列" % code
            )
        norm = math.hypot(vector[0], vector[1])
        if norm <= 1.0e-12:
            raise DeclarationError(
                "gait.foothold.phase_direction_map.%s 的方向向量是零矢量：落点行进方向无定义"
                % code
            )
        directions[code] = (vector[0] / norm, vector[1] / norm)

    # 【语义变更，2026-09-21（专项 C 步骤 10）】此处原有一条**互斥门禁**：
    # `per_phase`（移动落点）与 `sway.amplitude_m > 0`（平移机身）不得同时启用，理由是
    # "两个水平位移源叠加，失稳时无法归因"。该门禁已**解除**，动机是实测：
    # 只改落点、不动机身时，四条腿的水平目标各自独立按落点等级变化，而机身位置由**四条腿的
    # 约束共同决定** ⇒ 过约束、机身被拽着走：0.40 s 时机身水平位移已达 **23.69 mm**
    # （0.63 s 回落 15.65 mm，来回拖），**超过**一个完整周期的落点位移（10 mm/相位、20 mm/周期）。
    # 证据：`build/iraf-24h-3/step09/pivot-ordering.txt`（明细表）、
    #       `docs/debug/2026-09-21-quadruped-crawl-foothold.md` §10.3。
    # ⇒ 结论：机身位移与落点**本来就是同一件事的两半**（必须协同），不是两个互相掩盖的位移源；
    #    把它们判成互斥反而挡住了正确的机制。两键同时声明现在是**允许**的（不设门禁、不设默认值：
    #    是否启用由声明里的 `amplitude_m` 数值决定，语义仍是"逐相位重心转移"）。

    return {
        "mode": "per_phase",
        "stride_m": stride,
        "ramp_s": ramp_s,
        "smooth_s": smooth_s,
        "phase_order": order,
        "directions": directions,
        "verification": verification,
    }


def load_foothold_declaration(declaration):
    """**只**解析校验 `gait.foothold` 段；不依赖 Profile / 模型的关节清单。

    用途：门禁要能独立复核落足点声明（`scripts/profile_check.py` 门禁 9），而完整解析
    `load_gait_declaration` 还需要 Profile 关节清单去校验**关节绑定** —— 那是另一件事的职责；
    让落足点门禁连带依赖关节清单，会在「模型/Profile 与生产不同」的夹具与板卡上误报
    （实测踩到：夹具 Profile 的关节名与真实 Go2 不同 ⇒ 门禁 9 误判整份声明非法）。
    因此本入口只消费 `gait` 段自身的腿标识与相位偏移。
    """
    section = (declaration or {}).get("gait")
    if not isinstance(section, dict):
        raise DeclarationError("声明缺少 gait 段（落足点声明的唯一来源）")
    kind = str(section.get("kind"))
    if kind not in GAIT_KINDS:
        raise DeclarationError(
            "gait.kind 只支持 %s，实际: %r" % (list(GAIT_KINDS), section.get("kind"))
        )
    legs_section = section.get("legs")
    if not isinstance(legs_section, dict) or not legs_section:
        raise DeclarationError("gait.legs 必须是非空映射（相位结构与落点环形顺序只能来自声明）")
    legs = {}
    for code, leg in legs_section.items():
        code = str(code)
        if not isinstance(leg, dict) or "phase_offset" not in leg:
            raise DeclarationError("gait.legs.%s 缺少 phase_offset（落点环形顺序需要它）" % code)
        try:
            offset = float(leg["phase_offset"])
        except (TypeError, ValueError):
            raise DeclarationError(
                "gait.legs.%s.phase_offset 必须是数字，实际: %r" % (code, leg["phase_offset"])
            )
        legs[code] = {"phase_offset": offset}
    groups, reference = phase_groups_of(legs)
    return _load_foothold(section, legs, groups, reference, kind)


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
    # 中立（支撑/摆动）足端目标的离地间隙（m，> 0）：由调用点加到实测中立足端 z 上。
    # 为什么必须 > 0：语义是「目标位形不压进支撑面之下」，取 0 就是「目标恰好压在面上」，
    # 与「声明一个间隙」自相矛盾；缺键/非正则显式失败（不允许实现层默认值）。
    stance_clearance_m = _positive(section["stance_clearance_m"], "gait.stance_clearance_m")
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
    # 位置项取值域（语义见 REQUIRED_STABILIZATION_KEYS 上方与契约 §9）：
    #   · 增益 0 ⇒ 该项关闭（显式声明，不做隐式默认）；负值非法（会变成正反馈）；
    #   · 窗口必须**严格大于一个步态周期**，否则滑动平均会把逐相位 sway 也当成漂移去对抗；
    #   · 上限 > 0：与速度项的 max_linear_offset_m 分开，便于分别取证。
    position_gain = _positive(stabilization["position_gain_s_per_m"],
                              "gait.stabilization.position_gain_s_per_m", allow_zero=True)
    position_window = _positive(stabilization["position_window_s"],
                                "gait.stabilization.position_window_s")
    position_limit = _positive(stabilization["position_limit_m"],
                               "gait.stabilization.position_limit_m")
    period_s = 1.0 / frequency_hz
    if position_gain > 0.0 and position_window <= period_s:
        raise DeclarationError(
            "gait.stabilization.position_window_s=%.6f 必须**严格大于**一个步态周期 %.6f s："
            "否则滑动平均会把逐相位重心转移当成漂移去对抗（契约 §9 的反例）"
            % (position_window, period_s)
        )

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
    groups, reference = phase_groups_of(legs)
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

    sway = _load_sway(section, legs, kind)
    # 落足点规划（wave 必需）：`static` = 落点不变（现行为），`per_phase` = 逐相位落点规划。
    foothold = _load_foothold(section, legs, groups, reference, kind)

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
        "stance_clearance_m": stance_clearance_m,
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
            # 低频位置项三键（2026-09-23；契约 §9）。语义：作用于**滑动平均**后的机身水平位移，
            # 与速度项相加后各自限幅。增益 0 ⇒ 该项关闭（显式声明，不隐式生效）。
            "position_gain_s_per_m": position_gain,
            "position_window_s": position_window,
            "position_limit_m": position_limit,
        },
        "legs": legs,
        # 逐相位重心转移（wave 必需，其它步态类型为 None）：幅度/方向/平滑/斜坡全部来自声明。
        "sway": sway,
        # 落足点规划（wave 必需，其它步态类型可为 None）：`static` = 落点恒定（现行为），
        # `per_phase` = 逐相位摆动落点。Step 02 的落地点在 `gait_joint_targets` 消费本项。
        "foothold": foothold,
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


def sway_windows(params):
    """每条腿的摆动窗口起点（相位环 0→1）与窗口长度。

    腿的相位是 `(u + offset) % 1`，摆动相是 `相位 ≥ duty` ⇒ 摆动窗口为
    `u ∈ [duty − offset, 1 − offset)`。wave（duty = 0.75、四相位等间隔 0.25）下四个窗口
    长度相同且**恰好铺满**相位环（任意时刻恰好一条腿摆动），因此「哪个窗口包含 u」是唯一的。
    """
    duty = float(params["duty_factor"])
    window = 1.0 - duty
    return {code: ((duty - float(leg["phase_offset"])) % 1.0) for code, leg in params["legs"].items()}, window


def sway_leg_at_phase(params, phase):
    """相位 → 当前处于摆动相的那条腿（wave 的窗口铺满相位环，故该映射唯一）。"""
    windows, window = sway_windows(params)
    u = float(phase) % 1.0
    for code, start in sorted(windows.items()):
        end = start + window
        if start <= u < end or (end > 1.0 and u < end - 1.0):
            return code
    raise DeclarationError(
        "相位 %r 不落在任何腿的摆动窗口内：wave 的摆动窗口必须恰好铺满相位环"
        "（检查 duty_factor 与相位偏移间隔）" % u
    )


def _sway_kernel_mass(begin, end, center, span):
    """升余弦平滑核在区间 `[begin, end]` 上的质量占比（解析式，处处连续）。

    核：`K(x) = (1/2)(1 − cos(2πx/span))`，支集 `|x| ≤ span/2`；其 CDF 为
    `(x + span/2)/span − sin(2πx/span)/(2π)`。用解析式而不是离散采样：**离散核在窗口边界上
    会跳变**（核中心那一点正好落在阶梯的台阶上，u 跨过边界时该点整体换边 ⇒ 不连续，
    实测 0.03 的幅度在边界两侧差 0.019）。解析卷积处处连续且不依赖采样密度。
    """
    def cdf(offset):
        x = max(-0.5 * span, min(0.5 * span, offset))
        return (x + 0.5 * span) / span - math.sin(2.0 * math.pi * x / span) / (2.0 * math.pi)

    return max(0.0, cdf(end - center) - cdf(begin - center))


def _sway_direction_blend(params, u, smooth_fraction):
    """相位 → 平滑后的重心转移**单位**方向（升余弦核解析卷积，无阶跃、无跳变）。

    过渡窗口长度 `smooth_fraction`（相位单位）来自声明 `gait.sway.smooth_s`；混合后归一化
    ⇒ 过渡期间幅度不塌陷（只旋转方向，模长恒等于声明幅度）。
    """
    directions = params["sway"]["directions"]
    if smooth_fraction <= 0.0:
        return directions[sway_leg_at_phase(params, u)]
    span = min(float(smooth_fraction), 1.0)
    windows, width = sway_windows(params)
    total_x = 0.0
    total_y = 0.0
    total_mass = 0.0
    # 窗口与核都可能跨过相位环的 0/1 接缝：把每个窗口按 −1/0/+1 三次平移后再求交，
    # 避免在接缝处漏掉质量（漏质量会让方向在接缝附近偏）。
    for code, start in sorted(windows.items()):
        for shift in (-1.0, 0.0, 1.0):
            begin = start + shift
            mass = _sway_kernel_mass(begin, begin + width, u, span)
            if mass <= 0.0:
                continue
            vector = directions[code]
            total_x += mass * vector[0]
            total_y += mass * vector[1]
            total_mass += mass
    norm = math.hypot(total_x, total_y)
    if total_mass <= 1.0e-12 or norm <= 1.0e-9:
        raise DeclarationError(
            "相位 %r 上的重心转移方向平滑结果为零矢量：声明方向互相抵消或窗口未铺满相位环"
            "（检查 phase_map 与 duty_factor）" % u
        )
    return (total_x / norm, total_y / norm)


def sway_offset_m(params, elapsed_s):
    """相位 → **机身（重心）**应移动的水平偏移矢量 `(dx_m, dy_m)`（躯干系）。

    符号约定（调用点必须遵守）：本函数给出机身要移动的方向，因此**足端目标要减去它**
    （足端在机身系里相对机身反向退让，机身才会朝该方向走），见 `gait_joint_targets`。

    证据（为什么需要它）：对称矩形足迹下抬起任意单腿的静态余量实测 `-0.000227483 m`；
    抬脚前把重心推向「剩余三腿支撑三角形」的方向才能拿到正余量（探针实测：目标余量 0.010 m 时
    需平移 FL (−0.005784, −0.007877)、RR (+0.006053, +0.008244) m，方向即支撑三角形最紧边的内法线）。
    """
    sway = params.get("sway")
    if sway is None:
        return 0.0, 0.0
    if float(sway["amplitude_m"]) == 0.0:
        return 0.0, 0.0
    period = float(params["period_s"])
    u = (float(elapsed_s) / period) % 1.0
    direction = _sway_direction_blend(params, u, float(sway["smooth_s"]) / period)
    ramp = float(sway["ramp_s"])
    scale = 1.0 if ramp <= 0.0 else min(1.0, max(float(elapsed_s), 0.0) / ramp)
    amplitude = float(sway["amplitude_m"]) * scale
    axis = sway["axis"]
    return (
        amplitude * direction[0] if "x" in axis else 0.0,
        amplitude * direction[1] if "y" in axis else 0.0,
    )


def foothold_offset_m(params, code, elapsed_s):
    """相位 → 该腿**水平落点**偏移 `(dx_m, dy_m)`（躯干系，相对中立足端）。

    - `static`：恒返回**精确的** `(0.0, 0.0)` —— 既有「原地踏步」路径的数值逐位不变；
    - `per_phase`：摆动相从「上一周期的落点」平滑过渡到「本周期的落点」，支撑相保持本周期落点。
      落点逐周期在 `±stride_m · directions[腿]` 之间交替 ⇒ 足迹闭环、机身净漂移为 0（"原地"语义），
      而支撑三角形随相位改变形状（这是「静态余量 ≈ 0」的正面解法：机身常量平移已证不可行）。

    过渡时长由 `smooth_s` 给出（超过摆动窗口长度时按窗口长度计，**不越窗**）；幅度按 `ramp_s`
    线性建立（`0` = 首次抬腿前就满幅，与 `sway.ramp_s` 同取向）。摆动窗口的定义与 `sway` 共用
    `sway_windows`（窗口只由 `duty_factor` 与相位偏移决定，与重心处理方式无关）。
    """
    foothold = params.get("foothold")
    if not foothold or foothold.get("mode") != "per_phase":
        return (0.0, 0.0)
    period = float(params["period_s"])
    duty = float(params["duty_factor"])
    offset = float(params["legs"][code]["phase_offset"])
    stride = float(foothold["stride_m"])
    direction = foothold["directions"][code]
    smooth = float(foothold["smooth_s"])
    ramp = float(foothold["ramp_s"])

    # 以**该腿自身**的周期为坐标：周期边界取「相位回绕」处，即**摆动结束（落地）时刻**。
    # ⚠ 相位约定必须与 `leg_phase` 一致（`phase = (elapsed/period) + offset`，摆动相 = phase ∈ [duty, 1)）。
    # 首版误写成 `− offset`：等价于把落点交替窗口相对抬腿窗口整体错开 2·offset（FL 0 与 RR 0.5
    # 恰好对齐，FR 0.25 与 RL 0.75 错半个周期）⇒ 腿在支撑相里改落点、在摆动相里保持。
    # 实测签名（`build/iraf-24h-3/step03/reachability.txt`）：只有 FR/RL 的支撑相里出现
    # 未达等级的过渡帧，且落点误差 0.046597/0.045710 m，明显高于 FL/RR 的 0.041839/0.042217 m。
    raw = float(elapsed_s) / period + offset
    cycle_index = int(math.floor(raw))
    u = raw - cycle_index

    # 逐周期交替的落点等级（±1）。支撑相保持的是**上一周期摆动结束时**落下的等级
    # （= −sign），摆动相从该等级过渡到本周期等级（= +sign）—— 这样周期边界处连续。
    sign = 1.0 if (cycle_index % 2 == 0) else -1.0
    previous = -sign
    if u < duty:
        level = previous
    else:
        window_s = (1.0 - duty) * period
        span = window_s if smooth <= 0.0 else min(smooth, window_s)
        progress = 0.0 if span <= 0.0 else (u - duty) * period / span
        progress = min(1.0, max(0.0, progress))
        blend = 0.5 - 0.5 * math.cos(math.pi * progress)
        level = previous + (sign - previous) * blend

    gain = 1.0 if ramp <= 0.0 else min(1.0, max(float(elapsed_s), 0.0) / ramp)
    amount = gain * level * stride
    return (amount * float(direction[0]), amount * float(direction[1]))


def sway_direction_report(params, geometry):
    """按**实测**足迹校验声明的重心转移方向；不规则/方向相反即显式失败。

    判据（可复算）：抬腿 L 后的支撑三角形中，与「足迹中心」（= 整机重心投影的近似）余量最小的
    那条边的**内法线**，就是抬 L 时必须把重心移过去的方向。声明方向与它的夹角必须 ≤ 声明容差。
    这条门禁抓的是「方向写反/写错腿」——它会静默失效（照着错方向跑，判据全绿但机器人翻倒），
    因此必须有：实测方向由几何算出，声明方向只用于比对（不是自己跟自己比）。
    """
    sway = params.get("sway")
    if sway is None:
        return {"declared": False, "reason": "本步态类型不声明 gait.sway"}
    codes = sorted(params["legs"])
    feet = {code: np.asarray(geometry[code]["trunk_rel_m"][:2], dtype=float) for code in codes}
    center = np.mean([feet[code] for code in codes], axis=0)
    tolerance = float(sway["direction_tolerance_deg"])
    items = {}
    for code in codes:
        stance = [feet[other] for other in codes if other != code]
        best = None
        for index in range(len(stance)):
            start = stance[index]
            end = stance[(index + 1) % len(stance)]
            edge = end - start
            normal = np.array([-edge[1], edge[0]], dtype=float)
            length = float(np.linalg.norm(normal))
            if length <= 1.0e-12:
                continue
            normal = normal / length
            if float((np.mean(stance, axis=0) - start).dot(normal)) < 0.0:
                normal = -normal  # 内法线：指向支撑三角形内部
            margin = float((center - start).dot(normal))
            if best is None or margin < best[0]:
                best = (margin, normal)
        if best is None:
            raise DeclarationError("腿 %s 的支撑多边形退化（三点共线）：无法校验重心转移方向" % code)
        declared = np.asarray(sway["directions"][code], dtype=float)
        required = best[1]
        cosine = float(np.clip(declared.dot(required), -1.0, 1.0))
        angle = math.degrees(math.acos(cosine))
        items[code] = {
            "declared": [float(declared[0]), float(declared[1])],
            "required": [float(required[0]), float(required[1])],
            "margin_at_footprint_center_m": float(best[0]),
            "angle_error_deg": float(angle),
            "passed": bool(angle <= tolerance),
        }
    failed = [code for code, item in items.items() if not item["passed"]]
    if failed:
        raise DeclarationError(
            "gait.sway.phase_map 方向与实测支撑三角形内法线不符（容差 %r°）：%s ⇒ 显式失败"
            "（方向写反会让机器人朝支撑三角形外移动，判据仍会「通过」而机身翻倒）"
            % (tolerance, {code: round(items[code]["angle_error_deg"], 6) for code in failed})
        )
    return {
        "declared": True,
        "amplitude_m": float(sway["amplitude_m"]),
        "axis": list(sway["axis"]),
        "smooth_s": float(sway["smooth_s"]),
        "ramp_s": float(sway["ramp_s"]),
        "direction_tolerance_deg": tolerance,
        "footprint_center_xy_m": [float(center[0]), float(center[1])],
        "per_leg": items,
        "max_angle_error_deg": float(max(item["angle_error_deg"] for item in items.values())),
    }


def position_offset(params, mean_drift_xy):
    """机身**低频位移**的足端偏移量 `(dx_m, dy_m)`（2026-09-23 新增；契约见
    `docs/debug/2026-09-23-go2-gait-drift.md` §9）。

    与 `stabilization_offset`（速度项）的分工：
      · 速度项 `k·(v + ω×r)` 在 v→0 时修正量→0 ⇒ **压不住低速残余漂移**（实测声明内 19 档调参
        最好 0.5959 m，判据 0.05 m）；
      · 本函数作用在**滑动平均**后的机身水平位移上（窗口必须 > 1 个步态周期 ⇒ 不含逐相位 sway），
        因此只对"慢慢走掉"这一低频分量起作用，不与 sway 对抗。

    符号（**实测判定，勿凭直觉改动**）：机身漂 `+d` 时修正项取 **`+k·d`**（与速度项同加法位置）。
    两种符号都实测过（基线 step 0.01 + 速度项 0.3/0.06）：
      · `+k·d`：0.5959 m（无位置项）→ 0.5662 m（增益 0.1/上限 0.06，**本组最好**）、
        0.8262 m（0.01）、0.8653 m（0.3/0.20）；
      · `−k·d`（翻转后）：**四档全部更差** —— 7.9783 m（0.1）、7.3935 m（0.3）、10.0651 m（1.0）、
        1.4632 m（0.3/0.20）。
    ⇒ 取 `+k·d`。⚠ 同时如实登记：本项**不足以**把漂移压到判据（最好仍 0.5662 m vs 0.05 m，差 11 倍），
    即"残余漂移 = 低频位置偏移"这一假设**未被证实**，参见本项上方的文档 §9 与 §8。
    实参 `mean_drift_xy` 为**机身系**的平均位移（由适配器的 target_provider 维护）。
    """
    stabilization = params["stabilization"]
    if not stabilization["enabled"]:
        return 0.0, 0.0
    gain = float(stabilization["position_gain_s_per_m"])
    if gain == 0.0:
        return 0.0, 0.0            # 显式声明为 0 ⇒ 该项关闭（不做隐式默许）
    limit = float(stabilization["position_limit_m"])
    drift = np.asarray(mean_drift_xy, dtype=float).reshape(-1)
    if drift.size != 2:
        raise ValueError("mean_drift_xy 必须是 2 维（机身系 x/y），实际 %d" % drift.size)
    if not np.all(np.isfinite(drift)):
        raise ValueError("mean_drift_xy 含非有限值：不得据此生成足端目标")
    return (max(-limit, min(limit, gain * float(drift[0]))),
            max(-limit, min(limit, gain * float(drift[1]))))


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
                       body_velocity_mps=(0.0, 0.0, 0.0), body_omega_rad_s=(0.0, 0.0, 0.0),
                       body_mean_drift_xy=(0.0, 0.0)):
    """步态相位 → 12 个关节的目标角；目标越出 Profile 限位即显式失败。

    与 `gait.kind` 无关：相位→足端偏移（`foot_offset`）与支撑/摆动判定（`is_stance`）
    都只消费声明里的 duty 与相位偏移，因此 trot（两组各两条腿）与 wave（四相位各一条腿）
    共用同一条目标角生成路径（差异只在声明，不在代码）。

    `home`：Profile `spec.home`（髋关节由 IK 给出，需与 home 相加的偏移为零）。
    `joint_limits`：Profile 的关节限位；只允许在声明范围内活动（调用参数只能收紧，铁律 1.3）。
    `body_velocity_mps` / `body_omega_rad_s`：机身线速度与角速度（**机身坐标系**，
    由适配器从自由关节速度变换而来），仅用于声明化的阻尼偏移；缺省为零 = 无阻尼（不引入隐式行为）。
    `body_mean_drift_xy`：机身相对原点的**滑动平均位移**（机身系），仅用于声明化的低频位置项
    （`position_offset`，契约 §9）；缺省为零 = 无位置项（不引入隐式行为）。
    """
    targets = {}
    # 逐相位重心转移（wave 必需）：机身要往 +sway 移动 ⇒ 足端目标在机身系里减 sway。
    # 四条腿**同向**接受该偏移（支撑腿把机身推过去；摆动腿的落点随机身一起走，保持站姿形状），
    # 偏移由声明给出（幅度/方向/平滑/斜坡），实现层不含任何数字默认值。
    sway = sway_offset_m(params, elapsed_s)
    # 落足点规划（步骤 02）：该腿**水平落点**偏移。`static` 时精确为 (0.0, 0.0)，
    # 因此下面的表达式与「无落足点规划」逐位一致（加 0.0 不改变浮点值）。
    # 中立目标的离地间隙（步骤 02 A′ ②）：实测中立足端 z（躯干系，`measure_leg_geometry`）
    # 再抬高本值 ⇒ 目标位形的足端球最低点在支撑面**之上**，而不是压进面下。
    # 只抬高 z，不动 x/y（不与 sway / 落点偏移混叠）。
    clearance = float(params["stance_clearance_m"])
    for code, geom in geometry.items():
        phase = leg_phase(params, code, elapsed_s)
        dx, dz = foot_offset(phase, params, amplitude)
        foothold_x, foothold_y = foothold_offset_m(params, code, elapsed_s)
        damping = stabilization_offset(
            params, body_velocity_mps, body_omega_rad_s, geom["trunk_rel_m"]
        )
        # 低频位置项（与阻尼项同一加法位置、各自限幅）：压住速度项压不住的残余漂移。
        position = position_offset(params, body_mean_drift_xy)
        q_hip, q1, q2 = leg_solve(
            geom["neutral_x_m"] + dx + damping[0] + position[0] - float(sway[0]) + foothold_x,
            damping[1] + position[1] - float(sway[1]) + foothold_y,
            geom["neutral_z_m"] + clearance + dz,
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


def assess_crawl(samples, params, tilt_limit_deg):
    """落足点规划（迈步式 crawl）**专项**验收：声明判据 + 逐腿落点误差与接触达标率。

    与 `assess_gait` 的关系：共用同一套采样字段与**同一套相位约定**，但判据集合与阈值独立声明在
    `gait.foothold.verification`（理由见 `REQUIRED_FOOTHOLD_VERIFICATION_KEYS` 的注释：
    「原地」口径的 `max_displacement_m` 对 crawl 是不适用的量，既有判据与阈值**一字不动**，
    继续由 `verify_go2_gait_in_place.py` 强制执行）。

    缺依据一律**显式失败**，不得判为通过：模式不是 `per_phase`、采样为空、采样缺 `foot_trunk_m`、
    稳态窗为空、某腿在稳态窗内只有一个落点等级（无法比较相邻周期落点）。

    判据口径与一条**已知盲区**（诚实登记，不用叙述掩盖）
    ----------------------------------------------------
    `leg_*_landing_error_m` 测的是「落点**位移**是否兑现」：稳态窗内相邻两个落点等级的足端
    躯干系位置差 vs 声明 `(高等级−低等级)·stride_m·方向`。因为落足点规划**只通过位移**
    改变支撑三角形，这个差分口径才是与机制对应的量。
    盲区：**共模**偏移（所有帧同加一个常量，例如中立足端整体偏 10 mm）在差分里抵消 ⇒ 本判据看不见；
    因此它不校验「中立位是否与几何一致」。后者当前**没有任何判据覆盖**（既有原地验收量的是
    机身高度/位移/倾角，stand 验较量的是位形与高度）—— 登记为缺口，不在本函数里伪造一条弱判据。
    """
    foothold = params.get("foothold")
    if not foothold or foothold.get("mode") != "per_phase":
        raise CommandRejectedError(
            "crawl 验收只对 foothold.mode=per_phase 有效：static 模式下没有可验收的落点内容，"
            "不得判为通过（否则会退化成「零位移 ⇒ 落点误差 0 ⇒ 恒真通过」）"
        )
    verification = foothold["verification"]
    if not samples:
        raise CommandRejectedError("采样为空：验收无依据，不得判为通过")
    if "foot_trunk_m" not in samples[0]:
        raise CommandRejectedError(
            "采样缺少 foot_trunk_m（足端在躯干系的位置）：落点误差无依据，不得判为通过"
        )

    checks = _Checks()
    period = float(params["period_s"])
    duty = float(params["duty_factor"])
    contact_threshold = float(params["verification"]["contact_force_threshold_n"])
    fall_height = float(params["verification"]["fall_base_height_m"])
    onset = float(samples[0]["time_s"])
    # 稳态窗：跳过声明斜坡，且至少 2 个整周期（「相邻周期落点差」要跨周期比较才有定义）。
    settle_s = max(float(params["ramp_s"]), 2.0 * period)
    steady = [sample for sample in samples if float(sample["time_s"]) - onset >= settle_s]
    if not steady:
        raise CommandRejectedError("稳态窗口为空（ramp_s 与 2 个周期覆盖了全部采样）：验收无依据")

    heights = [float(sample["base_height_m"]) for sample in samples]
    min_height = float(min(heights))
    checks.add(
        "min_base_height_m",
        min_height,
        ">= gait.verification.fall_base_height_m = %r" % fall_height,
        min_height >= fall_height,
        "全称量最小机身高度 %.6f m（低于阈值即判为跌倒）" % min_height,
    )
    max_tilt = float(max(float(sample["tilt_deg"]) for sample in steady))
    checks.add(
        "max_tilt_deg",
        max_tilt,
        "<= 安全策略 max_tilt_moving_deg = %r" % tilt_limit_deg,
        max_tilt <= float(tilt_limit_deg),
        "稳态窗最大倾角 %.6f°（阈值来自 profiles/safety，不在此处写第二份数字）" % max_tilt,
    )
    saturated = int(sum(int(sample["ctrl_saturated"]) for sample in samples))
    checks.add(
        "ctrl_saturated_samples",
        saturated,
        "== 0（目标不得被执行器限幅截断）",
        saturated == 0,
        "饱和采样数 %d（截断意味着目标越出模型能力，判据不成立）" % saturated,
    )

    # 机身水平位移：**周期净漂移**（足迹闭环 ⇒ → 0）与**峰值**（周期内真实平移的上界）。
    origin = [float(value) for value in steady[0]["base_position_xy_m"]]
    peak = 0.0
    for sample in steady:
        position = [float(value) for value in sample["base_position_xy_m"]]
        peak = max(peak, math.hypot(position[0] - origin[0], position[1] - origin[1]))
    span_s = float(steady[-1]["time_s"]) - float(steady[0]["time_s"])
    cycles = span_s / period
    tail = [float(value) for value in steady[-1]["base_position_xy_m"]]
    net = math.hypot(tail[0] - origin[0], tail[1] - origin[1])
    per_cycle = (net / cycles) if cycles > 0.0 else float("inf")
    checks.add(
        "net_drift_per_cycle_m",
        per_cycle,
        "<= gait.foothold.verification.net_drift_per_cycle_m = %r"
        % verification["net_drift_per_cycle_m"],
        per_cycle <= float(verification["net_drift_per_cycle_m"]),
        "稳态窗 %.6f s = %.6f 个周期，首末机身位置差 %.6f m ⇒ 每周期净漂移 %.6f m"
        % (span_s, cycles, net, per_cycle),
    )
    checks.add(
        "peak_body_excursion_m",
        peak,
        "<= gait.foothold.verification.peak_body_excursion_m = %r"
        % verification["peak_body_excursion_m"],
        peak <= float(verification["peak_body_excursion_m"]),
        "稳态窗内机身水平位移峰值 %.6f m（相对窗口首帧）" % peak,
    )

    metrics: dict = {"per_leg": {}}
    for code in sorted(params["legs"]):
        offset = float(params["legs"][code]["phase_offset"])
        stride = float(foothold["stride_m"])
        direction = foothold["directions"][code]
        levels = {}
        stance_frames = 0
        contact_ok = 0
        for sample in steady:
            elapsed = float(sample["time_s"]) - onset
            if ((elapsed / period) + offset) % 1.0 >= duty:
                continue
            stance_frames += 1
            if float(sample["contact_n"][code]) >= contact_threshold:
                contact_ok += 1
            plan = foothold_offset_m(params, code, elapsed)
            level = int(round((plan[0] * direction[0] + plan[1] * direction[1]) / stride))
            levels.setdefault(level, []).append([float(value) for value in sample["foot_trunk_m"][code][:2]])

        rate = (contact_ok / stance_frames) if stance_frames else 0.0
        checks.add(
            "leg_%s_stance_contact_rate" % code,
            rate,
            ">= gait.foothold.verification.min_stance_contact_rate = %r"
            % verification["min_stance_contact_rate"],
            rate >= float(verification["min_stance_contact_rate"]),
            "稳态支撑相帧 %d，其中接触 ≥ %.3f N 的 %d 帧 ⇒ 达标率 %.6f"
            % (stance_frames, contact_threshold, contact_ok, rate),
        )

        leg_metrics = {"stance_frames": stance_frames, "contact_ok": contact_ok,
                       "stance_contact_rate": rate}
        if len(levels) >= 2:
            low, high = min(levels), max(levels)
            low_mean = [sum(item[0] for item in levels[low]) / len(levels[low]),
                        sum(item[1] for item in levels[low]) / len(levels[low])]
            high_mean = [sum(item[0] for item in levels[high]) / len(levels[high]),
                         sum(item[1] for item in levels[high]) / len(levels[high])]
            observed = (high_mean[0] - low_mean[0], high_mean[1] - low_mean[1])
            # 等级 L 的落点 = 中立位 + L·stride·方向 ⇒ 两个等级的差 = (high−low)·stride·方向。
            # 用 `high − low` 而不是假定 ±1：等级分组可能包含过渡中出现的 0 级。
            span = float(high - low)
            planned = (stride * direction[0] * span, stride * direction[1] * span)
            error = math.hypot(observed[0] - planned[0], observed[1] - planned[1])
            checks.add(
                "leg_%s_landing_error_m" % code,
                error,
                "<= gait.foothold.verification.landing_error_max_m = %r"
                % verification["landing_error_max_m"],
                error <= float(verification["landing_error_max_m"]),
                "等级 %d→%d（跨度 %g）：实测落点差 (%+.6f, %+.6f) vs 声明 (%+.6f, %+.6f) ⇒ 误差 %.6f m"
                % (low, high, span, observed[0], observed[1], planned[0], planned[1], error),
            )
            leg_metrics.update({"landing_observed_m": list(observed),
                                "landing_planned_m": list(planned),
                                "landing_error_m": error,
                                "levels": sorted(levels)})
        else:
            checks.add(
                "leg_%s_landing_error_m" % code,
                None,
                "稳态窗内需要 ≥2 个落点等级可比",
                False,
                "稳态窗内该腿的落点等级只有 %s ⇒ 无法比较相邻周期落点"
                "（判据不成立即失败，不是跳过）" % sorted(levels),
            )
            leg_metrics["levels"] = sorted(levels)
        metrics["per_leg"][code] = leg_metrics

    metrics.update({"min_base_height_m": min_height, "max_tilt_deg": max_tilt,
                    "ctrl_saturated_samples": saturated,
                    "net_drift_per_cycle_m": per_cycle, "peak_body_excursion_m": peak,
                    "steady_span_s": span_s, "steady_cycles": cycles})
    return {
        "checks": checks.items,
        "failed_checks": [item["name"] for item in checks.items if not item["passed"]],
        "metrics": metrics,
    }


def assess_trot(*args, **kwargs):
    """旧名薄包装（既有的 trot 调用点与单测仍按旧名调用）：语义与 `assess_gait` 完全相同。"""
    return assess_gait(*args, **kwargs)
