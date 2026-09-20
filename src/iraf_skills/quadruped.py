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

诚实边界
--------
- 全部结论属于**仿真**（`simulation: true`）；目标端/真机验收 DEFERRED（板卡不在场）。
- `locomote` 在本后端未实现：Provider 显式拒绝（`UnsupportedCapabilityError`），
  绝不返回"指令已生效"。
"""

import math
from pathlib import Path

import yaml

#: 声明边界的必需键（点号路径的最后一段）。缺键 = 干净的中文失败，不是运行到一半 KeyError。
LIMIT_KEYS = (
    "max_speed_mps",
    "max_yaw_rate_rad_s",
    "max_base_translation_m",
    "max_tilt_deg",
)

#: 已登记的错误码（`src/iraf_sdk/errors.py`，步骤 16 以 code-only 登记）。
COMMAND_REJECTED_CODE = "IRAF-QUADRUPED-COMMAND-REJECTED"


class SkillContractError(ValueError):
    """技能层声明/边界问题：显式失败，不做默认值兜底。"""

    code = COMMAND_REJECTED_CODE

    def __init__(self, message):
        super().__init__(message)
        self.code = type(self).code


def load_quadruped_limits(safety_policy_path):
    """从安全策略声明读四足边界上限；缺段、缺键、非法取值都显式失败。"""
    path = Path(safety_policy_path)
    if not path.is_file():
        raise SkillContractError("安全策略文件不存在: %s" % path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise SkillContractError("安全策略不是合法 YAML: %s (%s)" % (path, exc))
    if not isinstance(document, dict) or document.get("kind") != "SafetyPolicy":
        raise SkillContractError("安全策略 kind 必须是 SafetyPolicy: %s" % path)
    spec = document.get("spec") or {}
    limits = spec.get("quadruped_limits")
    if not isinstance(limits, dict):
        raise SkillContractError(
            "安全策略缺少 spec.quadruped_limits：四足速度/工作空间上限必须显式声明"
            "（禁止在脚本或 Skill schema 里另写一份边界）: %s" % path
        )
    missing = [key for key in LIMIT_KEYS if key not in limits]
    if missing:
        raise SkillContractError("spec.quadruped_limits 缺少必需键: %s" % missing)
    parsed = {}
    for key in LIMIT_KEYS:
        try:
            value = float(limits[key])
        except (TypeError, ValueError):
            raise SkillContractError("spec.quadruped_limits.%s 不是数值: %r" % (key, limits[key]))
        if not math.isfinite(value) or value <= 0.0:
            raise SkillContractError(
                "spec.quadruped_limits.%s 必须是正的有限数值，实际: %r" % (key, value)
            )
        parsed[key] = value
    return parsed


def enforce_velocity_limits(velocity, limits):
    """按声明上限校验规范速度字段；超限即拒绝（`IRAF-QUADRUPED-COMMAND-REJECTED`）。"""
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
    return {"speed_mps": speed, "yaw_rate_rad_s": rate}


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


class LocomoteProvider:
    """速度指令：先校验指令形状，再交由能力层显式拒绝（首期无步态控制器）。

    顺序刻意分成两步：`"乱下指令"`（未知字段/非有限数值）与 `"不会做"`（能力未实现）
    在诊断上必须可区分。若后端某天真的返回了运动结果，本 Provider 也会拒绝——
    因为那意味着有人在没有步态证据的情况下声称"走起来了"（铁律 1.5 禁止伪造成功）。
    """

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        velocity = self.backend.resolve_velocity(inputs["velocity"])
        duration_ms = self.backend.resolve_duration_ms(
            inputs.get("duration_ms"), 1.0 if inputs.get("duration_ms") is None else 0.0
        )
        # 适配器在能力未实现时必须抛 UnsupportedCapabilityError；返回即视为契约破裂。
        self.backend.locomote(velocity, duration_ms, lease)
        raise SkillContractError(
            "locomote 返回了结果但未提供步态证据：拒绝伪造成功（首期无步态控制器）"
        )
