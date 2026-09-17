"""工程向导类 Provider：把"新机器人适配流程"变成可复用的会话式向导。

设计约束（关键，直接决定本模块的形态）：

AGENTS.md 铁律 2 规定"所有请求必须经 TaskFlow -> Runtime -> Policy -> Provider，
适配不是任务请求，不得用 skill 去做适配"。因此本 Provider **不是机器人操作
skill**，而是一个**构建期工件生成器**，具体表现为：

- 不声明任何运动能力（capabilities 里没有 move_joint/pick_object/...）；
- **绝不调用 backend 的任何运动方法**，也不申请运动租约；
- 不产生任何物理动作，因此不经过运动安全策略的准入；
- 只输出构建期工件：profile.yaml + adapter 骨架 + 校验报告。

它与 pick_object / stop / move_joint 等运动 skill 的共存方式：
通过独立的 capability 命名空间 ``engineering.*`` 隔离，
注册表里以 ``requires: [engineering.robot_adapter]`` 声明，
不会与运动能力混淆，也不会被意图选择器当成"能动机器人"的能力。

生成物落点保护：既有文件一律拒绝覆写（铁律 5：失败即显式，禁止静默覆盖），
除非调用方显式声明 allow_overwrite。
"""

import json
import re
from pathlib import Path

#: 该向导支持的机器人机型预设。只描述"结构差异"，不含任何发行版私有库。
ROBOT_PRESETS = {
    "piper": {
        "display_name": "AgileX Piper",
        "dof": 6,
        "gripper_type": "parallel",
        "gripper_drive_joints": 2,
        "default_capabilities": ["move_joint", "pick_object", "visual_pick", "stop"],
        "notes": "参考实现；关节 joint1..joint8，joint7/8 为夹爪双驱动",
    },
    "franka": {
        "display_name": "Franka Emika Panda",
        "dof": 7,
        "gripper_type": "parallel",
        "gripper_drive_joints": 2,
        "default_capabilities": ["move_joint", "pick_object", "visual_pick", "stop"],
        "notes": "7 自由度冗余臂；夹爪为独立控制通道，需确认是否映射为两个关节",
    },
    "generic": {
        "display_name": "通用机器人",
        "dof": 6,
        "gripper_type": "parallel",
        "gripper_drive_joints": 2,
        "default_capabilities": ["move_joint", "pick_object", "visual_pick", "stop"],
        "notes": "未预设机型；必须逐项填写关节名与限位",
    },
}

#: 问卷字段定义：用于把客户答案归一化并做完整性校验。
QUESTIONNAIRE = (
    ("robot_id", "机器人唯一标识（用于 profile.metadata.name 与文件命名）", True),
    ("preset", "机型预设（piper / franka / generic）", True),
    ("joints", "全部关节名，按顺序（含夹爪驱动关节）", True),
    ("joint_limits", "每关节限位 [low, high]（弧度 / 米）", True),
    ("arm_joints", "臂关节名清单（参与 IK 求解）", True),
    ("control_frequency_hz", "控制频率（Hz）", True),
    ("wrist_body", "腕部 body 名（IK 位置参考）", True),
    ("finger_geoms", "左右指几何名 [left, right]", True),
    ("gripper_drive_joints", "夹爪驱动关节名（恰好两个）", True),
    ("open_positions", "张开位形 {关节: 值}", True),
    ("closed_positions", "闭合位形 {关节: 值}", True),
    ("simulation", "是否仿真环境（true / false）", False),
    ("pad_offset_m", "指腹相对抓取点的偏移量（米）", False),
    ("camera_pos_m", "相机位置 [x, y, z]（米）", False),
    ("camera_look_at_m", "相机注视点 [x, y, z]（米）", False),
)

_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class WizardValidationError(ValueError):
    """问卷答案不完整或不合规。"""


def _require(answers, key):
    value = answers.get(key)
    if value is None or (isinstance(value, str) and not value.strip()):
        raise WizardValidationError("问卷缺少必填项: " + key)
    return value


def _as_list(value, key, expected=None, minimum=None):
    if isinstance(value, str):
        items = [item.strip() for item in value.replace(",", " ").split() if item.strip()]
    elif isinstance(value, (list, tuple)):
        items = [str(item).strip() for item in value]
    else:
        raise WizardValidationError("字段 %s 必须是列表或空格分隔字符串" % key)
    if expected is not None and len(items) != expected:
        raise WizardValidationError(
            "字段 %s 必须恰好 %d 项，实际 %d 项" % (key, expected, len(items))
        )
    if minimum is not None and len(items) < minimum:
        raise WizardValidationError("字段 %s 至少需要 %d 项" % (key, minimum))
    return items


def _as_float_map(value, key):
    if not isinstance(value, dict) or not value:
        raise WizardValidationError("字段 %s 必须是非空对象" % key)
    parsed = {}
    for name, item in value.items():
        name = str(name)
        if not _IDENTIFIER.match(name):
            raise WizardValidationError("字段 %s 的键不是合法标识符: %s" % (key, name))
        try:
            parsed[name] = float(item)
        except (TypeError, ValueError) as exc:
            raise WizardValidationError(
                "字段 %s 的值必须是数值: %s=%r" % (key, name, item)
            ) from exc
    return parsed


def normalize_answers(raw):
    """把客户问卷答案归一化成结构化配置，并做完整性校验。

    校验原则与 Profile 加载器保持一致：缺失/矛盾一律显式失败，
    不做任何"猜一个默认值"的兜底，否则客户拿到的是一个看似可用
    但语义错误的 profile，问题会推迟到运行期才爆发。
    """
    if not isinstance(raw, dict):
        raise WizardValidationError("问卷答案必须是对象")
    answers = {str(key): value for key, value in raw.items()}
    for key, _question, required in QUESTIONNAIRE:
        if required:
            _require(answers, key)

    robot_id = str(_require(answers, "robot_id")).strip()
    if not _IDENTIFIER.match(robot_id):
        raise WizardValidationError("robot_id 必须是合法标识符: " + robot_id)

    preset = str(_require(answers, "preset")).strip()
    if preset not in ROBOT_PRESETS:
        raise WizardValidationError(
            "未知机型预设 %s，可选: %s" % (preset, ", ".join(sorted(ROBOT_PRESETS)))
        )

    joints = _as_list(_require(answers, "joints"), "joints", minimum=2)
    if len(set(joints)) != len(joints):
        raise WizardValidationError("joints 存在重复项")
    for name in joints:
        if not _IDENTIFIER.match(name):
            raise WizardValidationError("关节名不是合法标识符: " + name)

    raw_limits = _require(answers, "joint_limits")
    if not isinstance(raw_limits, dict):
        raise WizardValidationError("joint_limits 必须是对象")
    limits = {}
    for name in joints:
        if name not in raw_limits:
            raise WizardValidationError("joint_limits 缺少关节: " + name)
        pair = raw_limits[name]
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise WizardValidationError(
                "joint_limits.%s 必须是 [low, high]" % name
            )
        low, high = float(pair[0]), float(pair[1])
        if low >= high:
            raise WizardValidationError("joint_limits.%s 的 low 必须小于 high" % name)
        limits[name] = [low, high]
    for name in raw_limits:
        if str(name) not in joints:
            raise WizardValidationError("joint_limits 含未声明的关节: " + str(name))

    arm_joints = _as_list(_require(answers, "arm_joints"), "arm_joints", minimum=1)
    unknown = [name for name in arm_joints if name not in joints]
    if unknown:
        raise WizardValidationError("arm_joints 引用了未声明的关节: " + ", ".join(unknown))

    drive = _as_list(
        _require(answers, "gripper_drive_joints"),
        "gripper_drive_joints",
        expected=2,
    )
    if drive[0] == drive[1]:
        raise WizardValidationError("夹爪两个驱动关节不能相同")
    unknown = [name for name in drive if name not in joints]
    if unknown:
        raise WizardValidationError(
            "gripper_drive_joints 引用了未声明的关节: " + ", ".join(unknown)
        )
    overlap = [name for name in drive if name in arm_joints]
    if overlap:
        raise WizardValidationError(
            "夹爪驱动关节不得同时作为臂关节: " + ", ".join(overlap)
        )

    open_positions = _as_float_map(_require(answers, "open_positions"), "open_positions")
    closed_positions = _as_float_map(
        _require(answers, "closed_positions"), "closed_positions"
    )
    for key, block in (("open_positions", open_positions), ("closed_positions", closed_positions)):
        if set(block) != set(drive):
            raise WizardValidationError(
                "%s 必须与 gripper_drive_joints 完全对应（%s）"
                % (key, ", ".join(drive))
            )

    frequency = float(_require(answers, "control_frequency_hz"))
    if frequency <= 0:
        raise WizardValidationError("control_frequency_hz 必须为正数")

    wrist_body = str(_require(answers, "wrist_body")).strip()
    if not _IDENTIFIER.match(wrist_body):
        raise WizardValidationError("wrist_body 不是合法标识符: " + wrist_body)
    if wrist_body in joints:
        raise WizardValidationError("wrist_body 不应与关节同名: " + wrist_body)

    finger_geoms = _as_list(
        _require(answers, "finger_geoms"), "finger_geoms", expected=2
    )

    simulation = answers.get("simulation", True)
    if isinstance(simulation, str):
        simulation = simulation.strip().lower() in {"1", "true", "yes", "y"}
    simulation = bool(simulation)

    pad = answers.get("pad_offset_m")
    if pad is not None:
        pad = float(pad)
        if pad < 0:
            raise WizardValidationError("pad_offset_m 必须是非负数")

    def _optional_vector(key):
        value = answers.get(key)
        if value is None:
            return None
        items = _as_list(value, key, expected=3)
        return [float(item) for item in items]

    return {
        "robot_id": robot_id,
        "preset": preset,
        "simulation": simulation,
        "control_frequency_hz": frequency,
        "joints": joints,
        "joint_limits": limits,
        "arm_joints": arm_joints,
        "gripper": {
            "drive_joints": drive,
            "left_index": 0,
            "right_index": 1,
            "open_positions": open_positions,
            "closed_positions": closed_positions,
            "pad_offset_m": pad,
        },
        "bodies": {"wrist": wrist_body},
        "finger_geoms": {"left": finger_geoms[0], "right": finger_geoms[1]},
        "camera": {
            "pos_m": _optional_vector("camera_pos_m"),
            "look_at_m": _optional_vector("camera_look_at_m"),
        },
        "resolved_questions": [
            {"id": key, "question": question, "required": required}
            for key, question, required in QUESTIONNAIRE
        ],
    }


def _yaml_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, int):
        return str(value)
    return str(value)


def _yaml_inline_list(values):
    return "[" + ", ".join(_yaml_scalar(item) for item in values) + "]"


def render_profile_yaml(config):
    """渲染 RobotProfile YAML。

    只生成"结构差异声明"，不写入任何机型专属数值；
    未提供的可选项不写进文件（而不是写 null），
    避免 Profile 加载器把 null 误当成显式声明。
    """
    preset = ROBOT_PRESETS[config["preset"]]
    lines = [
        "apiVersion: iraf.intewell.io/v1",
        "kind: RobotProfile",
        "metadata:",
        "  name: %s" % config["robot_id"],
        "  version: 0.1.0",
        "spec:",
        "  simulation: %s" % _yaml_scalar(config["simulation"]),
        "  verification: development",
        "  control_frequency_hz: %s" % _yaml_scalar(config["control_frequency_hz"]),
        "  joints: %s" % _yaml_inline_list(config["joints"]),
        "  joint_limits:",
    ]
    for name in config["joints"]:
        lines.append("    %s: %s" % (name, _yaml_inline_list(config["joint_limits"][name])))
    lines.append(
        "  capabilities: %s" % _yaml_inline_list(preset["default_capabilities"])
    )
    lines.append("  joint_roles:")
    for name in config["joints"]:
        if name in config["arm_joints"]:
            role = "arm"
        elif name == config["gripper"]["drive_joints"][0]:
            role = "gripper_drive_left"
        elif name == config["gripper"]["drive_joints"][1]:
            role = "gripper_drive_right"
        else:
            role = "arm"
        lines.append("    %s: %s" % (name, role))
    gripper = config["gripper"]
    lines.append("  gripper:")
    lines.append("    drive_joints: %s" % _yaml_inline_list(gripper["drive_joints"]))
    lines.append("    left_index: %d" % gripper["left_index"])
    lines.append("    right_index: %d" % gripper["right_index"])
    lines.append(
        "    open_positions: {%s}"
        % ", ".join(
            "%s: %s" % (name, _yaml_scalar(gripper["open_positions"][name]))
            for name in gripper["drive_joints"]
        )
    )
    lines.append(
        "    closed_positions: {%s}"
        % ", ".join(
            "%s: %s" % (name, _yaml_scalar(gripper["closed_positions"][name]))
            for name in gripper["drive_joints"]
        )
    )
    if gripper["pad_offset_m"] is not None:
        lines.append("    pad_offset_m: %s" % _yaml_scalar(gripper["pad_offset_m"]))
    camera = config["camera"]
    if camera["look_at_m"] is not None:
        lines.append("  camera:")
        lines.append("    lookat_m: %s" % _yaml_inline_list(camera["look_at_m"]))
        lines.append("    distance_m: 1.0")
        lines.append("    azimuth_deg: 180.0")
        lines.append("    elevation_deg: -8.0")
    return "\n".join(lines) + "\n"


def render_adapter_skeleton(config):
    """渲染 Backend 适配骨架。

    骨架只包含契约要求的签名与显式 NotImplementedError，
    不猜测任何机型实现细节——猜测出来的实现会让"未适配"伪装成"已适配"。
    """
    robot_id = config["robot_id"]
    preset = ROBOT_PRESETS[config["preset"]]
    arm = config["arm_joints"]
    drive = config["gripper"]["drive_joints"]
    wrist = config["bodies"]["wrist"]
    geoms = config["finger_geoms"]
    capabilities = preset["default_capabilities"]
    return '''"""%(robot)s 的 IRAF Backend 适配骨架（由 engineering.robot_adapter 向导生成）。

生成时间工件，不是可运行实现：每个方法都显式抛 NotImplementedError，
以避免"未适配"被误当成"已适配"（AGENTS.md 铁律 5）。

适配要点（按契约逐项落地）：
- 臂关节（参与 IK）：%(arm)s
- 腕部 body：%(wrist)s
- 指几何：left=%(left)s, right=%(right)s
- 夹爪驱动关节：%(drive)s
- 需实现的运动能力：%(capabilities)s

注意：本文件不得依赖任何发行版私有库；厂商 SDK 调用应封装在本文件内部，
外部只通过 RobotBackend 契约访问。
"""

from iraf_core.kinematics import solve_position_ik


class %(class_name)sBackend:
    """%(robot)s Backend 骨架。"""

    ARM_JOINTS = %(arm_literal)s
    GRIPPER_DRIVE_JOINTS = %(drive_literal)s
    WRIST_BODY = %(wrist_literal)r
    FINGER_GEOMS = %(geoms_literal)s

    def __init__(self, profile, authority, config=None):
        self.profile = profile
        self.authority = authority
        self.config = config or {}

    @classmethod
    def from_config(cls, config, profile, authority):
        return cls(profile, authority, config)

    # --- 运动能力 ---

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        raise NotImplementedError("请接入 %(robot)s 的关节位置控制接口")

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        self.authority.validate(lease)
        # 抓取闭环建议复用既有阶段机：HOME_HOLD -> APPROACH -> DESCEND -> GRIP -> LIFT。
        # 双指指尖中点目标的 IK 可直接复用机器无关契约：
        #   solve_position_ik(model, data, target, arm_joint_ids, points, **solver)
        raise NotImplementedError("请接入 %(robot)s 的抓取闭环")

    def visual_pick(self, inputs, lease):
        self.authority.validate(lease)
        raise NotImplementedError("请接入 %(robot)s 的视觉位姿估计与 pick_object 串联")

    def stop(self, lease):
        self.authority.validate(lease)
        raise NotImplementedError("请接入 %(robot)s 的急停接口")

    def step(self, count=1):
        raise NotImplementedError("请接入 %(robot)s 的单步推进接口")

    # --- 运行期状态（供 Policy 前置条件求值）---

    def runtime_inventory(self):
        """PolicyGateway 会用它求值 skill.yaml 里的 preconditions。

        缺少对应键会让前置条件直接失败（IRAF-PRECONDITION-FAILED），
        因此必须返回真实状态而不是空字典。
        """
        return {
            "safety": {"estop": False},
            "manipulation": {"target_visible": False},
        }
''' % {
        "robot": preset["display_name"],
        "arm": ", ".join(arm),
        "wrist": wrist,
        "left": geoms["left"],
        "right": geoms["right"],
        "drive": ", ".join(drive),
        "capabilities": ", ".join(capabilities),
        "class_name": "".join(part.capitalize() for part in robot_id.split("_")),
        "arm_literal": repr(list(arm)),
        "drive_literal": repr(list(drive)),
        "wrist_literal": wrist,
        "geoms_literal": repr(dict(geoms)),
    }


def render_config_skeleton(config):
    """渲染与 profile 配套的基线 config 骨架（供参考姿态求解使用）。"""
    arm = config["arm_joints"]
    return '''schema_version: iraf.%(robot)s-baseline/v1
description: %(robot)s 受控仿真抓取基线（向导生成骨架，数值需按实现填写）

model:
  source: <替换为厂商 MJCF/URDF 转换后的模型路径>
  arm_joints: %(arm)s
  bodies:
    wrist: %(wrist)s
  finger_geoms:
    left: %(left)s
    right: %(right)s

grasp:
  # 抓取点（双指指尖中点）在世界系下的标称位置，z 由台面 + 半边长推出
  finger_center_xy_m: [0.0, 0.0]
  pregrasp_offset_m: 0.04
  approach_direction: [0.0, 0.0, 1.0]
  tip_clearance_m: 0.005
  clearance_iterations: 8
  lift_offset_m: 0.08
  randomization:
    # 随机摆放：以标称点为圆心的 xy 半径；z 贴台面；yaw 暂不随机
    xy_radius_m: 0.05
    max_attempts: 32
  solver:
    iterations: 800
    step: 0.5
    tolerance_m: 1.0e-05

gripper:
  joints: %(drive)s
  open: %(open)s
  closed: %(closed)s

acceptance:
  pose_tolerance_m: 0.005
  min_lift_delta_m: 0.02
  min_normal_force_n: 0.2
  max_force_imbalance_ratio: 4.0
''' % {
        "robot": config["robot_id"],
        "arm": _yaml_inline_list(arm),
        "wrist": config["bodies"]["wrist"],
        "left": config["finger_geoms"]["left"],
        "right": config["finger_geoms"]["right"],
        "drive": _yaml_inline_list(config["gripper"]["drive_joints"]),
        "open": "{%s}"
        % ", ".join(
            "%s: %s" % (name, _yaml_scalar(config["gripper"]["open_positions"][name]))
            for name in config["gripper"]["drive_joints"]
        ),
        "closed": "{%s}"
        % ", ".join(
            "%s: %s" % (name, _yaml_scalar(config["gripper"]["closed_positions"][name]))
            for name in config["gripper"]["drive_joints"]
        ),
    }


def render_validation_report(config, artifacts, blocked):
    """渲染校验报告：列出已生成工件、被拒绝的覆写、以及仍需人工补齐的项。"""
    preset = ROBOT_PRESETS[config["preset"]]
    checklist = [
        "按 RobotBackend 契约补齐 move_joint / pick_object / stop / step 的实现",
        "接入 runtime_inventory，返回真实 safety.estop 与 manipulation.target_visible",
        "在 profiles/safety/*.yaml 的 allowed_skills 中放行新 RobotProfile 的能力",
        "用 scripts/verify_ik_equivalence.py 同款思路验证 IK 与既有实现等价",
        "跑抓取验收链：无视觉真值 + 单目标视觉闭环 + 多目标未知姿态",
    ]
    return {
        "schema_version": "iraf.robot-adapter-wizard/v1",
        "robot_id": config["robot_id"],
        "preset": config["preset"],
        "preset_display_name": preset["display_name"],
        "dof": preset["dof"],
        "gripper_type": preset["gripper_type"],
        "declared_capabilities": preset["default_capabilities"],
        "artifacts": artifacts,
        "blocked_overwrites": blocked,
        "remaining_manual_work": checklist,
        "preset_notes": preset["notes"],
    }


def _write_artifact(path, content, allow_overwrite):
    """写工件；既有文件默认拒绝覆写（铁律 5：禁止静默覆盖）。"""
    path = Path(path)
    if path.exists() and not allow_overwrite:
        raise WizardValidationError(
            "目标文件已存在，拒绝静默覆盖: " + str(path) + "（如需覆盖请显式声明）"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return {"path": str(path), "bytes": len(content.encode("utf-8"))}


class RobotAdapterWizardProvider:
    """工程向导 Provider：产出适配工件，不驱动机器人。

    与运动 Provider 的关键差异（务必保持）：
    - 构造时接收 backend 仅为满足 Runtime 统一签名，本类**从不调用它**；
    - lease 参数仅为签名一致，不做任何租约校验或运动请求；
    - 不存在任何物理动作路径，因此不触发运动安全策略。
    """

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        answers = inputs.get("answers")
        if answers is None:
            # 支持分步问答：由调用方逐步投喂，此处只做口径校验。
            raise WizardValidationError(
                "向导需要 answers 字段；请按 questionnaire 逐项回答后一次性提交"
            )
        config = normalize_answers(answers)
        output_dir = Path(inputs.get("output_dir", "build/adapter-scaffold"))
        allow_overwrite = bool(inputs.get("allow_overwrite", False))

        artifacts = []
        blocked = []
        planned = (
            ("profile", output_dir / (config["robot_id"] + "_profile.yaml"),
             render_profile_yaml(config)),
            ("adapter", output_dir / (config["robot_id"] + "_backend.py"),
             render_adapter_skeleton(config)),
            ("baseline", output_dir / (config["robot_id"] + "_baseline.yaml"),
             render_config_skeleton(config)),
        )
        for kind, path, content in planned:
            try:
                record = _write_artifact(path, content, allow_overwrite)
            except WizardValidationError as exc:
                blocked.append({"kind": kind, "path": str(path), "reason": str(exc)})
                continue
            record["kind"] = kind
            artifacts.append(record)

        report = render_validation_report(config, artifacts, blocked)
        report_path = output_dir / (config["robot_id"] + "_validation_report.json")
        try:
            record = _write_artifact(
                report_path,
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                allow_overwrite,
            )
            record["kind"] = "validation_report"
            artifacts.append(record)
        except WizardValidationError as exc:
            blocked.append(
                {"kind": "validation_report", "path": str(report_path), "reason": str(exc)}
            )

        report["artifacts"] = artifacts
        report["blocked_overwrites"] = blocked
        # 有工件被拒写时整体判定为失败：部分成功会让客户以为适配已完成。
        report["accepted"] = not blocked
        return {
            "skill": "engineering.robot_adapter",
            "accepted": report["accepted"],
            "robot_id": config["robot_id"],
            "artifacts": artifacts,
            "blocked_overwrites": blocked,
            "report": report,
        }
