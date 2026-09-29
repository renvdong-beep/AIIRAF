"""四足技能 Provider 与声明边界门禁（步骤 17）。

分层
----
- 本模块是**技能层实现**：Provider 只做「把 Skill 输入翻译成能力调用 + 组装证据」，
  不含任何机型数字（关节名、增益、时长、限位全部来自 Profile 与声明）。
- 真正驱动执行器的是适配器（`iraf_adapters/unitree/unitree_go2.py`）：Provider 从不自己
  解析关节名、也不自己限幅，避免同一个语义在两处实现（步骤 16 的纪律）。
- 声明边界门禁（`load_quadruped_limits` / `enforce_velocity_limits` / `check_workspace`）
  消费 `profiles/safety/quadruped_lab.yaml` 的 `spec.quadruped_limits`：
  缺段或缺键即显式失败（退出码 2），**没有**任何数字默认值。

为什么要有声明边界门禁
----------------------
`PolicyGateway` 只懂平台无关的通用维度（RBAC / schema / Profile / 时长 / 能力 / 前置条件），
它不知道"四足的最大速度是多少"。若把速度上限写进脚本或写进 Skill 的 JSON Schema，
就会出现第二份安全边界。因此上限只声明一次（安全策略文件），由本模块的门禁消费：

- `enforce_velocity_limits`：**调度前**执行（属于 pre-dispatch 门禁，不是 Provider 内校验），
  超限即拒绝，并由调用方用 `SkillRuntime.record_pre_dispatch_failure` 落执行记录；
- `check_workspace`：对**实测**状态求位移与倾角，与声明上限比对（"动作有没有把本体带出工作空间"）。

移动语义（步骤 01，战役 `iraf-24h-2`）
------------------------------------
站立语义（"有没有动"）与移动语义（"能不能按这个速度/方向走"）是两套边界，**不共用键**：

- `load_movement_limits`：读 `max_accel_mps2` / `allow_in_place_turn` / `max_tilt_moving_deg` /
  `workspace_m` / `watchdog_timeout_ms`，并按类型分别校验（数值、布尔、矩形、模式表）；
- `enforce_velocity_limits(..., movement=…, current_velocity=…, ramp_s=…)`：在既有线速度/角速度
  上限之外，追加**加速度上限**（由机型声明的 `locomotion.ramp_s` 决定）与**原地转弯开关**；
- `enforce_state_freshness`：看门狗/失联门禁——状态年龄超过声明超时即拒绝新指令；
- `check_workspace_moving`：对**实测**位置比对声明工作空间矩形，并对移动中的倾角用 `max_tilt_moving_deg`；
- `resolve_stop_mode`：按运行状态决定 `stop` 的语义（移动 → `damped_hold`），调用方不得自行选择，
  未知模式与"非急停路径请求失能停机"一律显式失败；
- `load_locomotion_declaration`：读机型声明的 `locomotion` 段（斜坡/心跳/控制频率来源/damped_hold
  参数来源），解析每个"来源"点号路径并与安全策略做跨文件自洽校验（斜坡必须够到达最大速度、
  心跳周期必须小于看门狗超时）。

诚实边界
--------
- 以上门禁是**调度前**边界；其调用方是步态控制器与移动/到点路径（步骤 02~04）。
  本步（步骤 01）只交付"声明 + 门禁 + 正/负路径证据"，**不宣称已接线到运行时**。
- 全部结论属于**仿真**（`simulation: true`）；目标端/真机验收 DEFERRED（板卡不在场）。
- `locomote` 在本后端未实现：Provider 显式拒绝（`UnsupportedCapabilityError`），
  绝不返回"指令已生效"。
"""

import math
from pathlib import Path

import yaml

# 技能层拒绝（与其它 Provider 同一异常类型；`dock_for_handoff` 的"显式结论"要用它）
from iraf_skills.common.motion import SkillRejected  # noqa: E402

#: 声明边界的必需键（点号路径的最后一段）。缺键 = 干净的中文失败，不是运行到一半 KeyError。
LIMIT_KEYS = (
    "max_speed_mps",
    "max_yaw_rate_rad_s",
    "max_base_translation_m",
    "max_tilt_deg",
)

#: 已登记的错误码（`src/iraf_sdk/errors.py`，步骤 16 以 code-only 登记）。
COMMAND_REJECTED_CODE = "IRAF-QUADRUPED-COMMAND-REJECTED"

#: 移动语义的数值上限（步骤 01）：与站立的 4 个键**分开**，因为它们是"能不能按这个速度/方向走"，
#: 而 `LIMIT_KEYS` 是"站着有没有被带走"。两者混用会让"站得稳"与"走得对"共用一条判据。
MOVEMENT_NUMERIC_KEYS = ("max_accel_mps2", "max_tilt_moving_deg", "watchdog_timeout_ms")

#: 工作空间矩形（绝对坐标，单位 m）：四个边界都必须声明，缺一即失败。
WORKSPACE_RECT_KEYS = ("x_min", "x_max", "y_min", "y_max")

#: 已登记的停止语义。缺任一 ⇒ 声明非法（缺 `damped_hold` 就无法表达"移动中受控停止"）。
REGISTERED_STOP_MODES = ("damped_hold", "torque_zero_release")

#: `stop_modes.<模式>` 允许出现的键：`applies_to`（适用路径，必需且非空）+ `detail`（中文说明）。
STOP_MODE_ENTRY_KEYS = ("applies_to", "detail")

#: `locomotion` 段的必需键（缺一即失败；不设默认值）。
LOCOMOTION_REQUIRED_KEYS = (
    "ramp_s",
    "control_frequency_source",
    "heartbeat_period_ms",
    "watchdog_action",
    "damped_hold",
)

#: 看门狗触发时**唯一**允许的停止语义：受控停止。声明成失能停机即失败。
WATCHDOG_STOP_MODE = "damped_hold"



class SkillContractError(ValueError):
    """技能层声明/边界问题：显式失败，不做默认值兜底。"""

    code = COMMAND_REJECTED_CODE

    def __init__(self, message):
        super().__init__(message)
        self.code = type(self).code


def _load_safety_spec(safety_policy_path):
    """读取安全策略并做 kind 校验，返回 `(path, spec)`。

    `load_quadruped_limits`（站立语义）与 `load_movement_limits`（移动语义）共用本函数：
    同一份文件的解析只做一次，避免两套读法对"是不是安全策略"给出不同答案。
    """
    path = Path(safety_policy_path)
    if not path.is_file():
        raise SkillContractError("安全策略文件不存在: %s" % path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise SkillContractError("安全策略不是合法 YAML: %s (%s)" % (path, exc))
    if not isinstance(document, dict) or document.get("kind") != "SafetyPolicy":
        raise SkillContractError("安全策略 kind 必须是 SafetyPolicy: %s" % path)
    return path, (document.get("spec") or {})


def _require_mapping(node, key, scope):
    """取一个必需的子映射；缺失或类型不对都显式失败（不做空字典兜底）。"""
    if key not in node:
        raise SkillContractError("%s 缺少必需段: %s" % (scope, key))
    value = node[key]
    if not isinstance(value, dict):
        raise SkillContractError("%s.%s 必须是映射，实际: %r" % (scope, key, value))
    return value


def _require_positive_number(mapping, key, scope):
    """取一个必需的正有限数值；缺失/非数值/非正数都显式失败。"""
    if key not in mapping:
        raise SkillContractError("%s 缺少必需键: %s" % (scope, key))
    try:
        value = float(mapping[key])
    except (TypeError, ValueError):
        raise SkillContractError("%s.%s 不是数值: %r" % (scope, key, mapping[key]))
    if not math.isfinite(value) or value <= 0.0:
        raise SkillContractError("%s.%s 必须是正的有限数值，实际: %r" % (scope, key, value))
    return value


def _load_quadruped_limits_node(safety_policy_path):
    """返回 `(path, quadruped_limits)`，缺段即显式失败（两个 loader 共用）。"""
    path, spec = _load_safety_spec(safety_policy_path)
    limits = spec.get("quadruped_limits")
    if not isinstance(limits, dict):
        raise SkillContractError(
            "安全策略缺少 spec.quadruped_limits：四足速度/工作空间上限必须显式声明"
            "（禁止在脚本或 Skill schema 里另写一份边界）: %s" % path
        )
    return path, limits


def load_quadruped_limits(safety_policy_path):
    """从安全策略声明读四足边界上限；缺段、缺键、非法取值都显式失败。"""
    path, limits = _load_quadruped_limits_node(safety_policy_path)
    missing = [key for key in LIMIT_KEYS if key not in limits]
    if missing:
        raise SkillContractError("spec.quadruped_limits 缺少必需键: %s" % missing)
    return {key: _require_positive_number(limits, key, "spec.quadruped_limits") for key in LIMIT_KEYS}


def load_stop_modes(safety_policy_path):
    """读 `spec.stop_modes`：登记**两层**停止语义，并写明各自适用路径。

    规则（战役 `iraf-24h-2` 硬规则 3）：`damped_hold`（移动中的受控停止）与
    `torque_zero_release`（失能停机，只用于急停/安全事件）必须**并列登记**；
    缺任一模式、适用路径为空、出现未登记的模式名或未知键，都显式失败——
    停止语义的"混用"必须首先在声明层就不可能表达。
    """
    path, spec = _load_safety_spec(safety_policy_path)
    return _validate_stop_modes(spec, "spec", path)


def _validate_stop_modes(spec, scope, path):
    """校验并归一化停止语义表（loader 与 `resolve_stop_mode` 共用同一份校验）。"""
    stop_modes = spec.get("stop_modes")
    if not isinstance(stop_modes, dict):
        raise SkillContractError(
            "%s 缺少 stop_modes：受控停止与失能停机必须并列登记"
            "（缺声明即失败，不得由实现层猜默认语义）: %s" % (scope, path)
        )
    missing = [mode for mode in REGISTERED_STOP_MODES if mode not in stop_modes]
    if missing:
        raise SkillContractError("%s.stop_modes 缺少已登记语义: %s" % (scope, missing))
    unknown = sorted(set(stop_modes) - set(REGISTERED_STOP_MODES))
    if unknown:
        raise SkillContractError(
            "%s.stop_modes 出现未登记的停止语义: %s（登记集: %s）"
            % (scope, unknown, list(REGISTERED_STOP_MODES))
        )
    parsed = {}
    for mode in REGISTERED_STOP_MODES:
        entry = stop_modes[mode]
        if not isinstance(entry, dict):
            raise SkillContractError("%s.stop_modes.%s 必须是映射，实际: %r" % (scope, mode, entry))
        extra = sorted(set(entry) - set(STOP_MODE_ENTRY_KEYS))
        if extra:
            raise SkillContractError(
                "%s.stop_modes.%s 出现未知键: %s（允许: %s）"
                % (scope, mode, extra, list(STOP_MODE_ENTRY_KEYS))
            )
        applies_to = entry.get("applies_to")
        if not isinstance(applies_to, list) or not applies_to or not all(
            isinstance(item, str) and item for item in applies_to
        ):
            raise SkillContractError(
                "%s.stop_modes.%s 必须用非空的 applies_to 写明适用路径（禁止只写模式名）"
                % (scope, mode)
            )
        parsed[mode] = {"applies_to": list(applies_to), "detail": str(entry.get("detail") or "")}
    return parsed


def load_movement_limits(safety_policy_path):
    """读**移动语义**边界（步骤 01 新增）：加速度上限、原地转弯开关、工作空间矩形、看门狗、停止语义。

    与 `load_quadruped_limits` 分开：返回的是"能不能按这个速度/方向走"的边界，
    调用方是步态控制器与移动/到点路径（步骤 02~04）。
    """
    path, limits = _load_quadruped_limits_node(safety_policy_path)
    scope = "spec.quadruped_limits"
    parsed = {key: _require_positive_number(limits, key, scope) for key in MOVEMENT_NUMERIC_KEYS}

    if "allow_in_place_turn" not in limits:
        raise SkillContractError(
            "%s 缺少 allow_in_place_turn：是否允许原地转弯必须显式声明（不得由实现层假定）" % scope
        )
    allow_in_place_turn = limits["allow_in_place_turn"]
    if not isinstance(allow_in_place_turn, bool):
        raise SkillContractError(
            "%s.allow_in_place_turn 必须是布尔值，实际: %r（字符串 'false' 不是 false）"
            % (scope, allow_in_place_turn)
        )
    parsed["allow_in_place_turn"] = allow_in_place_turn

    rect = _require_mapping(limits, "workspace_m", scope)
    missing_rect = [key for key in WORKSPACE_RECT_KEYS if key not in rect]
    if missing_rect:
        raise SkillContractError("%s.workspace_m 缺少边界: %s" % (scope, missing_rect))
    # 四个边界都是**绝对坐标**，允许为负（工作空间可以整体位于原点左侧/后方）。
    parsed["workspace_m"] = {
        key: _require_number(rect, key, scope + ".workspace_m") for key in WORKSPACE_RECT_KEYS
    }
    for axis in ("x", "y"):
        if parsed["workspace_m"]["%s_min" % axis] >= parsed["workspace_m"]["%s_max" % axis]:
            raise SkillContractError(
                "%s.workspace_m 的 %s 边界自相矛盾：min %r ≥ max %r"
                % (scope, axis, parsed["workspace_m"]["%s_min" % axis],
                   parsed["workspace_m"]["%s_max" % axis])
            )

    parsed["stop_modes"] = load_stop_modes(safety_policy_path)
    return parsed


def _require_number(mapping, key, scope):
    """取一个必需的有限数值（允许负值：工作空间下界本体就是负的）。"""
    if key not in mapping:
        raise SkillContractError("%s 缺少必需键: %s" % (scope, key))
    try:
        value = float(mapping[key])
    except (TypeError, ValueError):
        raise SkillContractError("%s.%s 不是数值: %r" % (scope, key, mapping[key]))
    if not math.isfinite(value):
        raise SkillContractError("%s.%s 必须是有限数值，实际: %r" % (scope, key, value))
    return value


def enforce_velocity_limits(velocity, limits, movement=None, current_velocity=None, ramp_s=None):
    """按声明上限校验速度指令；超限即拒绝（`IRAF-QUADRUPED-COMMAND-REJECTED`）。

    `movement`（可选的移动语义边界）与 `current_velocity` / `ramp_s` **必须同时给出**：
    给了 `movement` 就要求按声明斜坡推算加速度，不允许用隐式默认值（"从零起步"是假定，不是事实）。
    """
    speed = math.hypot(float(velocity["vx_mps"]), float(velocity["vy_mps"]))
    rate = abs(float(velocity["wz_rad_s"]))
    if speed > float(limits["max_speed_mps"]):
        raise SkillContractError(
            "速度指令线速度 %g m/s 超过声明上限 %g m/s（安全策略 quadruped_limits.max_speed_mps）"
            % (speed, float(limits["max_speed_mps"]))
        )
    if rate > float(limits["max_yaw_rate_rad_s"]):
        raise SkillContractError(
            "速度指令偏航角速度 %g rad/s 超过声明上限 %g rad/s"
            "（安全策略 quadruped_limits.max_yaw_rate_rad_s）"
            % (rate, float(limits["max_yaw_rate_rad_s"]))
        )
    measured = {"speed_mps": speed, "yaw_rate_rad_s": rate}
    if movement is None:
        if current_velocity is not None or ramp_s is not None:
            raise SkillContractError(
                "缺少移动语义边界（movement）：给了 current_velocity/ramp_s 却没有声明边界，"
                "无法校验加速度上限与原地转弯开关"
            )
        return measured

    for key in ("max_accel_mps2", "allow_in_place_turn"):
        if key not in movement:
            raise SkillContractError("移动语义边界缺少必需键: %s" % key)
    if current_velocity is None or ramp_s is None:
        raise SkillContractError(
            "移动语义的加速度门禁需要 current_velocity 与 ramp_s（机型声明 locomotion.ramp_s）；"
            "不允许用\"从零起步\"之类的隐式默认值"
        )
    ramp = float(ramp_s)
    if not math.isfinite(ramp) or ramp <= 0.0:
        raise SkillContractError("斜坡时长 ramp_s 必须是正的有限数值，实际: %r" % ramp_s)
    dvx = float(velocity["vx_mps"]) - float(current_velocity["vx_mps"])
    dvy = float(velocity["vy_mps"]) - float(current_velocity["vy_mps"])
    accel = math.hypot(dvx, dvy) / ramp
    max_accel = float(movement["max_accel_mps2"])
    if accel > max_accel:
        raise SkillContractError(
            "速度指令加速度 %g m/s² 超过声明上限 %g m/s²"
            "（斜坡 ramp_s=%g s，安全策略 quadruped_limits.max_accel_mps2）"
            % (accel, max_accel, ramp)
        )
    in_place_turn = bool(speed <= 0.0 and rate > 0.0)
    if in_place_turn and not bool(movement["allow_in_place_turn"]):
        raise SkillContractError(
            "声明不允许原地转弯（quadruped_limits.allow_in_place_turn=false），"
            "但指令为纯偏航速度 %g rad/s 且线速度为零" % rate
        )
    measured.update(
        {"accel_mps2": accel, "in_place_turn": in_place_turn, "ramp_s": ramp}
    )
    return measured


def enforce_state_freshness(state_age_s, movement_limits):
    """看门狗/失联门禁：状态年龄超过声明 `watchdog_timeout_ms` ⇒ 拒绝新指令（ADR-0008 §2）。

    状态过期时的正确动作是拒绝下发（并保持上一次受控停止），**不是**用过期状态继续控制。
    """
    watchdog_s = float(movement_limits["watchdog_timeout_ms"]) / 1000.0
    age = float(state_age_s)
    if not math.isfinite(age) or age < 0.0:
        raise SkillContractError("状态年龄必须是有限非负数值，实际: %r" % state_age_s)
    if age > watchdog_s:
        raise SkillContractError(
            "控制状态已过期：年龄 %g ms > 声明看门狗超时 %g ms"
            "（安全策略 quadruped_limits.watchdog_timeout_ms）⇒ 拒绝下发新指令"
            % (age * 1000.0, watchdog_s * 1000.0)
        )
    return {"state_age_s": age, "watchdog_timeout_s": watchdog_s, "fresh": True}


def check_workspace_moving(before_state, after_state, movement_limits):
    """移动语义工作空间门禁：**绝对位置**落在声明矩形内，且移动中倾角 ≤ `max_tilt_moving_deg`。"""
    rect = movement_limits["workspace_m"]
    before = [float(v) for v in before_state["base_position_m"]]
    after = [float(v) for v in after_state["base_position_m"]]
    tilt = tilt_deg_from_quaternion(after_state["base_quaternion_wxyz"])
    checks = []
    for axis, index in (("x", 0), ("y", 1)):
        low = float(rect["%s_min" % axis])
        high = float(rect["%s_max" % axis])
        value = after[index]
        checks.append(
            {
                "name": "workspace.%s_max" % axis,
                "measured": value,
                "limit": high,
                "passed": bool(value <= high),
                "detail": "移动后躯干绝对坐标 %s 不得越出声明工作空间上界" % axis,
            }
        )
        checks.append(
            {
                "name": "workspace.%s_min" % axis,
                "measured": value,
                "limit": low,
                "passed": bool(value >= low),
                "detail": "移动后躯干绝对坐标 %s 不得越出声明工作空间下界" % axis,
            }
        )
    tilt_limit = float(movement_limits["max_tilt_moving_deg"])
    checks.append(
        {
            "name": "workspace.tilt_moving_deg",
            "measured": tilt,
            "limit": tilt_limit,
            "passed": bool(tilt <= tilt_limit),
            "detail": "移动中躯干相对竖直的倾斜角（不含偏航）",
        }
    )
    return {
        "simulation": True,
        "position_m": after,
        "travel_m": math.hypot(after[0] - before[0], after[1] - before[1]),
        "tilt_deg": tilt,
        "workspace_m": {key: float(rect[key]) for key in WORKSPACE_RECT_KEYS},
        "limits": {
            "max_tilt_moving_deg": tilt_limit,
            "watchdog_timeout_ms": float(movement_limits["watchdog_timeout_ms"]),
        },
        "checks": checks,
        "passed": all(item["passed"] for item in checks),
    }


def resolve_stop_mode(stop_modes, moving, standing_mode, emergency=False, requested=None):
    """按**运行状态**决定本次 `stop` 的语义；调用方不得自行选择，禁止混用。

    规则（ADR-0008 决策 3 / 战役硬规则 3）：
    - 急停或安全事件 → `torque_zero_release`（失能停机，唯一合法入口）；此时若请求受控停止，拒绝（不得降级）。
    - 移动中 → `damped_hold`（减速到零并保持站立）。
    - 未移动 → 机型声明的站立停机语义 `standing_mode`（**既有验收事实**，本步不改变）。
    - 任何非急停路径显式请求 `torque_zero_release` → 拒绝（失能停机不得作为常规停止）。
    - 未知/未登记的模式名 → 拒绝。
    """
    if not isinstance(stop_modes, dict):
        raise SkillContractError("stop_modes 必须是映射（来自安全策略 stop_modes）")
    _validate_stop_modes({"stop_modes": stop_modes}, "spec", "<传入的停止语义表>")
    if standing_mode not in stop_modes:
        raise SkillContractError(
            "站立停机语义 %r 未在 stop_modes 登记（登记集: %s）" % (standing_mode, sorted(stop_modes))
        )
    if requested is not None and requested not in stop_modes:
        raise SkillContractError(
            "未知停止模式: %r（登记集: %s）——未登记的语义不得被请求" % (requested, sorted(stop_modes))
        )
    if emergency:
        if requested is not None and requested != "torque_zero_release":
            raise SkillContractError(
                "急停/安全事件路径不得降级为受控停止（请求 %r）：安全语义只允许收紧" % requested
            )
        return "torque_zero_release"
    if requested == "torque_zero_release":
        raise SkillContractError(
            "常规 stop 不得请求 torque_zero_release（失能停机只用于急停/安全事件）："
            "语义混用被拒绝，请请求 damped_hold 或走急停路径"
        )
    if moving:
        return "damped_hold"
    return str(standing_mode)


def _resolve_declaration_path(declaration, dotted_path, scope):
    """解析机型声明内部的点号路径；任一段缺失即显式失败（禁止静默回退到默认值）。"""
    if not isinstance(dotted_path, str) or not dotted_path:
        raise SkillContractError("%s 必须是声明内的点号路径字符串，实际: %r" % (scope, dotted_path))
    node = declaration
    for part in dotted_path.split("."):
        if not isinstance(node, dict) or part not in node:
            raise SkillContractError(
                "%s 指向的路径 %r 在机型声明中不存在（第 %r 段缺失）——声明来源必须可解析"
                % (scope, dotted_path, part)
            )
        node = node[part]
    return node


def load_locomotion_declaration(declaration, movement_limits, standing_limits):
    """读机型声明的 `locomotion` 段并做跨文件自洽门禁（步骤 01）。

    本段只写**控制实现参数**与**来源路径**，不写任何速度/位移阈值（阈值只在安全策略里，
    避免同一事实两处）。自洽门禁两条（任一不成立即失败）：
    `ramp_s × max_accel_mps2 ≥ max_speed_mps`（斜坡必须够到达声明最高速度）
    与 `heartbeat_period_ms ≤ watchdog_timeout_ms`（心跳周期不得大于失联判定）。
    """
    if not isinstance(declaration, dict):
        raise SkillContractError("机型声明必须是映射")
    locomotion = declaration.get("locomotion")
    if not isinstance(locomotion, dict):
        raise SkillContractError(
            "机型声明缺少 locomotion 段：斜坡时长/控制频率来源/心跳周期/看门狗动作/"
            "damped_hold 参数来源都必须显式声明（缺声明即失败，不得用实现层默认值）"
        )
    missing = [key for key in LOCOMOTION_REQUIRED_KEYS if key not in locomotion]
    if missing:
        raise SkillContractError("locomotion 段缺少必需键: %s" % missing)

    scope = "locomotion"
    ramp_s = _require_positive_number(locomotion, "ramp_s", scope)
    heartbeat_ms = _require_positive_number(locomotion, "heartbeat_period_ms", scope)

    frequency_hz = _resolve_declaration_path(
        declaration, locomotion["control_frequency_source"], scope + ".control_frequency_source"
    )
    try:
        frequency_hz = float(frequency_hz)
    except (TypeError, ValueError):
        raise SkillContractError(
            "locomotion.control_frequency_source 解析结果不是数值: %r" % (frequency_hz,)
        )
    if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
        raise SkillContractError(
            "locomotion.control_frequency_source 解析结果必须为正的有限数值，实际: %r" % (frequency_hz,)
        )

    watchdog_action = locomotion["watchdog_action"]
    stop_modes = movement_limits["stop_modes"]
    if watchdog_action not in stop_modes:
        raise SkillContractError(
            "locomotion.watchdog_action=%r 不在已登记停止语义 %s 内"
            % (watchdog_action, sorted(stop_modes))
        )
    if watchdog_action != WATCHDOG_STOP_MODE:
        raise SkillContractError(
            "locomotion.watchdog_action 必须是 %s（看门狗超时属受控停止；失能停机只用于急停/安全事件），"
            "实际: %r" % (WATCHDOG_STOP_MODE, watchdog_action)
        )

    damped_hold = _require_mapping(locomotion, "damped_hold", scope)
    damped_hold_missing = [
        key for key in ("deceleration_source", "hold_pose_source") if key not in damped_hold
    ]
    if damped_hold_missing:
        raise SkillContractError("locomotion.damped_hold 缺少来源键: %s" % damped_hold_missing)
    resolved_sources = {}
    for key in ("deceleration_source", "hold_pose_source"):
        path_value = damped_hold[key]
        resolved = _resolve_declaration_path(
            declaration, path_value, "%s.damped_hold.%s" % (scope, key)
        )
        resolved_sources[key] = {"path": path_value, "value": resolved}

    required_ramp = float(movement_limits["max_accel_mps2"])
    max_speed = float(standing_limits["max_speed_mps"])
    if ramp_s * required_ramp < max_speed - 1e-12:
        raise SkillContractError(
            "声明自相矛盾：ramp_s=%g s × max_accel_mps2=%g m/s² = %g m/s < max_speed_mps=%g m/s"
            "（斜坡时长必须够到达声明最高速度）" % (ramp_s, required_ramp, ramp_s * required_ramp, max_speed)
        )
    watchdog_ms = float(movement_limits["watchdog_timeout_ms"])
    if heartbeat_ms > watchdog_ms:
        raise SkillContractError(
            "声明自相矛盾：heartbeat_period_ms=%g ms > watchdog_timeout_ms=%g ms"
            "（心跳周期大于失联判定会让每一次心跳都被判超时）" % (heartbeat_ms, watchdog_ms)
        )

    return {
        "ramp_s": ramp_s,
        "control_frequency_hz": frequency_hz,
        "control_frequency_source": locomotion["control_frequency_source"],
        "heartbeat_period_ms": heartbeat_ms,
        "watchdog_action": watchdog_action,
        "damped_hold": resolved_sources,
    }


def tilt_deg_from_quaternion(quaternion_wxyz):
    """相对竖直的倾斜角（**不含偏航**）：`acos(R[2,2])`。

    用 `2·acos(|w|)` 会把原地转向算成"站不直"（步骤 15 的实测教训）。
    """
    w, x, y, z = (float(v) for v in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise SkillContractError("四元数模长为零，无法计算姿态")
    w, x, y, z = (v / norm for v in (w, x, y, z))
    cosine = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def check_workspace(before_state, after_state, limits):
    """工作空间门禁：以**实测**状态比对声明上限，返回可逐项断言的证据。"""
    before = [float(v) for v in before_state["base_position_m"]]
    after = [float(v) for v in after_state["base_position_m"]]
    translation = math.hypot(after[0] - before[0], after[1] - before[1])
    tilt = tilt_deg_from_quaternion(after_state["base_quaternion_wxyz"])
    checks = [
        {
            "name": "workspace.base_translation_m",
            "measured": translation,
            "limit": float(limits["max_base_translation_m"]),
            "passed": bool(translation <= float(limits["max_base_translation_m"])),
            "detail": "站立过程中躯干水平位移（以执行前状态为基准）",
        },
        {
            "name": "workspace.tilt_deg",
            "measured": tilt,
            "limit": float(limits["max_tilt_deg"]),
            "passed": bool(tilt <= float(limits["max_tilt_deg"])),
            "detail": "执行后躯干相对竖直的倾斜角（不含偏航）",
        },
    ]
    return {
        "simulation": True,
        "base_translation_m": translation,
        "tilt_deg": tilt,
        "limits": {key: float(limits[key]) for key in LIMIT_KEYS},
        "checks": checks,
        "passed": all(item["passed"] for item in checks),
    }


def _evidence(report, keys):
    """从适配器报告里抽取输出 schema 需要的键；缺键即显式失败（不是静默丢字段）。"""
    missing = [key for key in keys if key not in report]
    if missing:
        raise SkillContractError("适配器报告缺少输出必需键: %s" % missing)
    return {key: report[key] for key in keys}


class StandProvider:
    """站立：目标位形与时长由适配器按声明解析（Provider 不持有任何机型数字）。"""

    EVIDENCE_KEYS = (
        "simulation",
        "capability",
        "execution_id",
        "fencing_token",
        "duration_ms",
        "control_cycles",
        "substeps_per_control",
        "ctrl_saturated_samples",
        "target_source",
        "torque_limit_source",
        "gravity_feedforward",
        "joint_targets_rad",
        "final_state",
    )

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        report = self.backend.stand(
            lease,
            targets=inputs.get("joint_targets"),
            duration_ms=inputs.get("duration_ms"),
        )
        return {"skill": "stand", "accepted": True, "evidence": _evidence(report, self.EVIDENCE_KEYS)}


class DockForHandoffProvider:
    """停靠（`dock_for_handoff`）：接近参数与**验收判据全部来自声明**，本 Provider 不含任何数字。

    与 `StandProvider` 同构：只做「透传声明 → 调适配器 → 按输出 schema 抽取证据」。
    参数面为零（输入 schema `additionalProperties: false` 且无 properties）：接近速度/增益/容差/
    超时/制动提前量/保持语义都在机型声明里，调用方覆盖它们等于绕过安全与验收边界。
    失败（超时 / 漂出容差 / 目标帧缺失）由适配器在**执行过程中**表达并写进报告 `failure`，
    本 Provider **如实透出**（不吞、不改写、不返回伪造成功）。
    """

    EVIDENCE_KEYS = ("simulation", "capability", "target_frame", "target_frame_world_fixed",
                     "settled_at_s", "final_translation_error_m", "final_yaw_error_deg",
                     "final_speed_mps")

    #: 允许的输入键：**只有站名**。为什么只允许名字（2026-09-29 双臂轮转演示引入第二个站位）：
    #: 停靠目标是场景事实、且站位数量会增长 ⇒ 站名登记在机型声明里（`dock_for_handoff.stations`），
    #: 步骤只按名选择；坐标/容差等数字一律不得由调用方给（与原"无参数"契约同一纪律）。
    ALLOWED_INPUTS = ("station",)

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        unknown = sorted(str(key) for key in (inputs or {}) if str(key) not in self.ALLOWED_INPUTS)
        if unknown:
            raise SkillContractError(
                "dock_for_handoff 只接受站位名 `station`（接近参数与验收判据全部来自机型声明），"
                "不接受其它键：%s" % unknown)
        station = (inputs or {}).get("station")
        acceptance = self.backend.dock_acceptance()      # 来自声明；本层不写数字
        # 适配器签名是 keyword-only（`def dock_for_handoff(self, *, lease, ...)`）
        report = self.backend.dock_for_handoff(lease=lease, station=station, **acceptance)
        # **显式结论**（2026-09-28 §11.23(34)）：停靠未完成时后端只把 `failure` 放进报告、
        # 终态量取不到实数（`final_speed_mps` 会是 None）⇒ 若直接返回，契约会以
        # "Provider output does not match schema: None is not of type 'number'" 报错，
        # **把真实原因（接近失败）盖住**（本轮实测踩到）。⇒ 这里显式拒绝并带上后端给出的原因。
        failure = report.get("failure") or None
        if failure is not None:
            detail = failure.get("reason") if isinstance(failure, dict) else str(failure)
            raise SkillRejected("停靠未完成（%s）：%s"
                                % (failure.get("decision") if isinstance(failure, dict)
                                   else "DOCK_FAILED", detail))
        evidence = _evidence(report, self.EVIDENCE_KEYS)
        evidence["failure"] = None
        return {"skill": "dock_for_handoff", "accepted": True, "evidence": evidence}


class LocomoteProvider:
    """速度指令：指令形状校验 → 适配器 `locomote` → 按输出 schema 抽取证据。

    ⚠ 语义变更（2026-09-23）：本 Provider 曾经只有「能力未实现时必须抛错；**返回即契约破裂**」
    的守卫（首期无步态控制器）。`locomote` 实现并通过四工况验收后，改为**真正消费报告**
    （与 `StandProvider` 同构）—— 否则它会把**成功判成失败**；未声明/未实现的能力仍由
    能力契约层在**执行前**拒绝（实测：`IRAF-SKILL-PROVIDER-UNAVAILABLE`）。
    """

    #: 输出 schema（`skills/locomote/locomote.output.json`）要的 evidence 键。
    #: `velocity` 不在适配器报告里（报告叫 `command`）⇒ 在 `execute` 里显式映射，
    #: 而不是把 `command` 改名（改名会牵动报告契约与其他消费方）。
    EVIDENCE_KEYS = ("simulation", "capability", "duration_ms")

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        velocity = self.backend.resolve_velocity(inputs["velocity"])
        # 时长单位：`inputs["duration_ms"]` 是**毫秒**，原样交给适配器 —— 它内部的
        # `resolve_duration_ms` 是**唯一**的解析点（曾经在这里先转一次，结果把"秒"当"毫秒"
        # 传下去 ⇒ 2000 ms 变 2 ms ⇒「控制周期数不足 1」，实测 2026-09-23）。
        duration_ms = inputs.get("duration_ms")
        report = self.backend.locomote(
            velocity, 1000.0 if duration_ms is None else float(duration_ms), lease
        )
        evidence = _evidence(report, self.EVIDENCE_KEYS)
        evidence["velocity"] = dict(velocity)
        return {"skill": "locomote", "accepted": True, "evidence": evidence}


class AcceptPayloadProvider:
    """载荷确认（`accept_payload`）：四足侧**独立复核**载荷落位与整链静止。

    为什么必须独立（.hermes/plans/2026-09-28-s05-accept-payload.md 风险 R3）：s04 已经给出
    `payload_in_tray`；若本 Provider 只是复述它，等于"自己证明自己"（AGENTS.md 1.5 的精神）。
    后端在**确认时刻重新采样**，并给出 s04 没有的量（`resting_gap_m` / `last_speed_mps`）。

    与 `DockForHandoffProvider` 同构：只做「校验入参 → 调适配器 → 按输出 schema 抽取证据」；
    证据不成立即 `SkillRejected`（不吞、不改写、不返回伪造成功）。
    """

    #: 输出 schema（`skills/accept_payload/accept_payload.output.json`）要的 evidence 键。
    EVIDENCE_KEYS = ("payload_on_target", "contact_margin_m", "payload_low_z_m", "target_top_z_m",
                     "resting_gap_m",
                     "contact_geoms", "payload_center_m", "target_center_m",
                     "offset_from_target_center_m", "last_speed_mps", "runtime_source",
                     "phase_trace")

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        # ⚠ `SkillContractError` 定义在**本模块**（quadruped.py），只有 `SkillRejected` 来自
        # `iraf_skills.common.motion`（照抄别处写法会 ImportError —— 本轮实测踩到）。
        from iraf_skills.common.motion import SkillRejected

        payload_id = str((inputs or {}).get("payload_id") or "")
        place_target_id = str((inputs or {}).get("place_target_id") or "")
        if not payload_id or not place_target_id:
            raise SkillContractError(
                "accept_payload 需要 payload_id 与 place_target_id（在哪确认必须写清）：实际 %s"
                % sorted(inputs or {}))
        if not hasattr(self.backend, "accept_payload"):
            raise SkillRejected("后端未实现 accept_payload（能力未交付，不得伪造确认）")
        report = self.backend.accept_payload(payload_id, place_target_id, lease)
        evidence = _evidence(report, self.EVIDENCE_KEYS)
        if not evidence.get("payload_on_target"):
            raise SkillRejected(
                "载荷未确认落在接收体上：载荷最低点 %s m、承载面 %s m、落位间隙 %s m、接触 geom %s"
                % (evidence.get("payload_low_z_m"), evidence.get("target_top_z_m"),
                   evidence.get("resting_gap_m"), evidence.get("contact_geoms")))
        return {"skill": "accept_payload", "accepted": True,
                "payload_id": payload_id, "place_target_id": place_target_id,
                "confirmation": report.get("confirmation"), "evidence": evidence}
