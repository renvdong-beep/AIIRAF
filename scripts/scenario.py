"""S2 脚本化场景执行入口（步骤 18）：`scenario.yaml` 声明的步骤序列 → SkillRuntime 真实执行 → 证据报告。

职责边界
--------
- 本文件是**入口层 + 执行器**：解析参数、把场景包声明与机型声明解析成执行计划、装配运行时、
  逐步执行、把实测值写成报告。
- 判据、路径、耗时、本体绑定一律来自声明（铁律 5.3）：场景包（`scene.yaml` / `scenario.yaml` /
  `baseline.yaml`）给步骤与判据；机型声明（如 `config/go2_loopback.yaml`）给后端入口、Profile、
  安全策略与模型路径。本文件**没有任何**阈值或路径默认值。
- 词表（判据名、动作名、故障类型、终态）从 `config/scene.schema.json` 读取，不在本文件复制一份。

执行链（不绕层）
----------------
`scenario.yaml` 步骤 → `SkillRuntime.execute`（TaskFlow → PolicyGateway → ControlAuthority 租约 →
Provider → 适配器 → MuJoCo）。执行器**不直接驱动后端**：后端只用于读实测状态（`read_state`）
给判据取证，因此"命令已发出"不会被当成"结果达成"。

两类失败注入（本战役交付范围）
------------------------------
1. **传感器不可用**（`faults[].kind: sensor_unavailable`）：在故障点**拒绝下发**该步的运动指令，
   由 `SkillRuntime.record_pre_dispatch_failure` 落一条带错误码（`IRAF-PRECONDITION-FAILED`）的
   执行记录，并进入声明终态；该步绝不出现 SUCCEEDED（禁止伪造成功，铁律 1.5/1.6）。
2. **能力未声明**：步骤用到的能力不在本体 `capabilities` 里 —— 有 `pending_closed_by` 登记时
   显式跳过并记进 `pending_steps`（不是"通过"）；无登记时**显式失败**（退出码 3）。
其余故障类型（`actuator_timeout` / `authority_conflict` / `state_stale` / `estop`）本战役未交付：
声明里出现即退出码 2，绝不静默跳过。

判据口径（可评测的判据只有五个，其余一律显式失败）
--------------------------------------------------
- `min_stable_hold_s` ≥ 阈值：测量值 = 本步执行期间**推进的仿真时间**
  （后端 `read_state()["time_s"]` 前后差）；
- `max_speed_m_s` ≤ 阈值：测量值 = 步骤结束时实测的躯干线速度模长；
- `timeout_s` ≤ 阈值：测量值 = 本步墙钟耗时；
- `translation_error_max_m` ≤ 阈值：测量值 = **技能输出 evidence** 的
  `final_translation_error_m`（停靠结果量；后端末态采样拿不到"相对目标帧的位姿差"）；
- `yaw_error_max_deg` ≤ 阈值：测量值 = `|evidence.final_yaw_error_deg|`（**取绝对值**：
  带符号量直接比阈值会让 −3° 以"−3 ≤ 2"骗过判据）。
后两条依赖技能**如实给出**结果量：缺字段时依据缺失、该条判据显式判失败，不静默通过。
口径是"时间推进量/末态采样"，**不是**稳定性分析：物理稳定性证据见步骤 15 的 loopback 验收
（`build/acceptance/go2-loopback/report.json`），两者不可互换（报告 `not_proved` 里明写）。
未在 `CRITERION_SPEC` 中登记的判据（如 `pose_tolerance_m`、`min_lift_delta_m`）在**预检**阶段
即失败（退出码 2），不拖到运行期才发现"判不了"。

用法与退出码
------------
    PYTHONPATH=src python3 scripts/scenario.py list
    PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario stand_stop

- `0` 通过：所有被执行的步骤满足判据；所有未执行的步骤都有显式登记
- `1` 用法错误（缺参数、场景目录不存在）
- `2` 声明非法或超出本执行器支持范围（契约层失败、未支持的故障类型、判据无评测依据）
- `3` 引用完整性失败（场景包引用失败、本体未声明 `robot.backend` 绑定、技能未注册、模型未生成）
- `4` 后端装配失败（能力契约/模型编译）
- `5` 判据未通过（含 `--require-injected-faults` 但仍有故障未被注入）

诚实边界：全部结论属于**仿真**（报告 `simulation: true`）；目标端/真机验收一律 DEFERRED
（板卡不在场）。故障注入只证明"未继续下发运动指令、未伪造成功"，不构成物理安全停机证据。
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import scene_check  # noqa: E402  （同目录的入口层模块：复用它的三层场景包校验）

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
# S1 交互（interact）用的实时播放：与臂侧 viewer 共用同一份渲染循环实现（机器人无关）
from iraf_adapters.mujoco.viewer_runner import run_request_live  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import RegistryError, SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_BACKEND = 4
EXIT_CRITERIA = 5

SCHEMA_PATH = ROOT / "config" / "scene.schema.json"

#: 本战役交付的故障类型（其余在预检阶段显式失败，不静默跳过）。
SUPPORTED_FAULT_KINDS = ("sensor_unavailable",)
#: 故障注入点拒绝下发时使用的错误码（已登记于 src/iraf_sdk/errors.py；不新造码）。
PRECONDITION_FAILED_CODE = "IRAF-PRECONDITION-FAILED"
#: 执行器记录的场景级终态（词表来自契约的 fault.expect.terminal_state）。
SAFE_HOLD = "SAFE_HOLD"

#: 判据 -> (测量量, 比较方向, 中文口径)。未登记在表中的判据在预检阶段失败（退出码 2）。
CRITERION_SPEC = {
    "min_stable_hold_s": (
        "sim_time_advance_s",
        ">=",
        "本步执行期间推进的仿真时间（后端 read_state 的 time_s 前后差）",
    ),
    "max_speed_m_s": (
        "final_speed_mps",
        "<=",
        "步骤结束时实测的躯干线速度模长（后端 read_state）",
    ),
    "timeout_s": ("wall_seconds", "<=", "本步墙钟耗时"),
    # 停靠（`dock_for_handoff`）的**结果**判据：测量量只能来自**技能输出的 evidence**
    # （适配器报告 `final_translation_error_m` / `final_yaw_error_deg`）—— 后端末态采样
    # 拿不到"相对目标帧的位姿差"，而"推算"它等于把判据建在另一套几何上。
    # evidence 缺失（例如能力未交付/未给出该字段）⇒ 依据缺失 ⇒ 该条判据**判失败**（不静默通过）。
    "translation_error_max_m": (
        "dock_translation_error_m", "<=",
        "停靠结束时实测的平移误差（evidence.final_translation_error_m）",
    ),
    "yaw_error_max_deg": (
        "dock_yaw_error_deg", "<=",
        "停靠结束时实测的偏航误差**绝对值**（|evidence.final_yaw_error_deg|，单位度）",
    ),
    # 抓取（`pick_object`）的三项判据：测量量同样**只能来自技能证据**
    # （抓取点的对齐误差、提起高度、双侧指腹接触），不从末态位姿"推算"。
    # 阈值出处（不是实现层默认值）：`config/machines/piper_joint.yaml: robot.backend_config.
    # target_tolerance_m`（= 0.005，与臂侧基线 `acceptance.pose_tolerance_m` 同值）与
    # 联合报告 `gripper.min_lift_delta_m`（= 0.02）；两侧都是声明，本文件只做评测。
    # `require_bilateral_contact` 是布尔量：拿 1.0/0.0 与 `true` 比（见 evaluate_criteria 的 "=="）。
    "pose_tolerance_m": (
        "grasp_center_distance_m", "<=",
        "抓取时实测的夹爪中心到目标中心的距离（evidence.grasp_alignment.center_distance_m）",
    ),
    "min_lift_delta_m": (
        "grasp_lift_delta_m", ">=",
        "抓取后实测的目标体提起高度（evidence.lift_delta_m）",
    ),
    "require_bilateral_contact": (
        "grasp_bilateral_contact", "==",
        "实测双侧指腹同时接触（evidence.bilateral_contact，布尔量）",
    ),
    # 放置（`place_object`）的三项判据：测量量同样**只来自技能证据**（不推算）。
    # 阈值出处：`max_offset_from_tray_center_m` 由接收体声明尺寸推出并写在 scenario.yaml 的注释里
    # （托盘 0.24×0.16、方块 0.05 ⇒ 完全落入只需 |dx|≤0.095、|dy|≤0.055，取 0.06）；
    # 另两条是布尔事实（夹爪已松开 / 载荷落在承载面内且与接收体接触）。
    "max_offset_from_tray_center_m": (
        "place_offset_from_tray_center_m", "<=",
        "放置后实测载荷中心相对接收体中心的水平偏移（evidence.place_alignment.offset_from_center_m）",
    ),
    "require_release": (
        "place_released", "==",
        "实测夹爪张开后指腹与载荷不再接触（evidence.released，布尔量）",
    ),
    "require_payload_in_tray": (
        "place_payload_in_tray", "==",
        "实测载荷落在接收体承载面内且与接收体接触（evidence.payload_in_tray，布尔量）",
    ),
    # 载荷确认（`accept_payload`）的三项判据：测量量只来自**四足侧**证据（不复用臂侧放置证据，
    # 否则等于"自己证明自己"）。阈值出处：偏移 0.06 与 s04 同口径（托盘 0.24×0.16、方块 0.05）；
    # 末速 0.01 m/s 是"已静止"的工程上界（s02 停靠实测 0.000196922、s03 抓取 0.000208312
    # ⇒ 留约两个数量级余量）。
    "require_payload_confirmation": (
        "accept_payload_on_target", "==",
        "四足侧实测载荷落在接收体承载面上且存在接触（evidence.payload_on_target，布尔量）",
    ),
    "max_accept_offset_m": (
        "accept_offset_from_target_center_m", "<=",
        "载荷中心相对接收体中心的水平偏移（evidence.offset_from_target_center_m）",
    ),
    "max_accept_speed_mps": (
        "accept_last_speed_mps", "<=",
        "确认时刻整链末速 = 所有自由关节线速度上界（evidence.last_speed_mps）",
    ),
}
#: 判据 -> **该判据的测量量从哪来**（2026-09-28 §11.23(46)，甲案）。
#: 取值含义：
#:   · `None`            ⇒ 来自**后端状态/墙钟**（任何步骤都可评测）；
#:   · **候选路径元组**  ⇒ 元素形如 `("grasp_alignment", "center_distance_m")`，表示证据里的字段路径；
#:                          只要该步骤**技能的 output schema** 里存在其中一条，即视为"该技能产得出这条判据"。
#: 为什么需要它：`plan_steps` 原先只检查"判据名在不在 `CRITERION_SPEC`"⇒ 给 `stand` 配 `pose_tolerance_m`
#: （只属于 pick/place）会被当成可评测，直到运行期才以退出码 5 报判据未满足，而不是预检退出码 2。
#: 与 `CRITERION_SPEC` 的完备性由 `_validate_criterion_tables()` 守住（缺登记即显式失败）。
CRITERION_EVIDENCE_PATHS = {
    "min_stable_hold_s": None,                 # 后端状态：本步推进的仿真时间
    "max_speed_m_s": None,                     # 后端状态：末速
    "timeout_s": None,                         # 墙钟
    "translation_error_max_m": (("final_translation_error_m",),),
    "yaw_error_max_deg": (("final_yaw_error_deg",),),
    # `pose_tolerance_m` 有两个合法来源：抓取（pick_object）与放置（place_object）各给一份对齐量
    "pose_tolerance_m": (("grasp_alignment", "center_distance_m"),
                         ("place_alignment", "center_distance_m")),
    "min_lift_delta_m": (("lift_delta_m",),),
    "require_bilateral_contact": (("bilateral_contact",),),
    "max_offset_from_tray_center_m": (("place_alignment", "offset_from_center_m"),),
    "require_release": (("released",),),
    "require_payload_in_tray": (("payload_in_tray",),),
    "require_payload_confirmation": (("payload_on_target",),),
    "max_accept_offset_m": (("offset_from_target_center_m",),),
    "max_accept_speed_mps": (("last_speed_mps",),),
}


def _validate_criterion_tables():
    """两张表必须**同步**（fail-closed）：否则新增判据时会出现"词表里有、证据来源没登记"的静默盲区。"""
    spec, paths = set(CRITERION_SPEC), set(CRITERION_EVIDENCE_PATHS)
    if spec != paths:
        raise ScenarioError(
            "判据表不同步：CRITERION_SPEC 与 CRITERION_EVIDENCE_PATHS 的差集 = %s / %s"
            % (sorted(spec - paths), sorted(paths - spec)), EXIT_DECLARATION)


def _skill_provides_evidence(action, paths, registry):
    """该步骤技能的 output schema 能否产出这些证据路径之一；返回 True/False/None（None = 无法判定）。"""
    if registry is None:
        return None
    skill = registry.resolve(str(action), "")
    if skill is None:
        return None
    schema = (skill.manifest.output_schema or {})
    evidence = (((schema.get("properties") or {}).get("evidence") or {}).get("properties") or {})

    def has(path):
        # ⚠ `evidence` **本身**就是 `properties` 字典（`...evidence.properties`）⇒ 顶层用包装包一层，
        # 否则会多查一层 `properties` 而把所有判据都判成 False（本轮实测踩到：`nominal` 被预检误拒）。
        node = {"properties": evidence}
        for part in path:
            properties = node.get("properties") if isinstance(node, dict) else None
            if not isinstance(properties, dict) or part not in properties:
                return False
            node = properties[part]
        return True

    return any(has(list(path)) for path in paths)


#: 可在报告中出现的测量量键（顺序固定，便于逐项比对）。
MEASUREMENT_KEYS = ("sim_time_advance_s", "final_speed_mps", "wall_seconds", "evidence_duration_s",
                    "dock_translation_error_m", "dock_yaw_error_deg",
                    "grasp_center_distance_m", "grasp_lift_delta_m", "grasp_bilateral_contact",
                    "place_offset_from_tray_center_m", "place_released", "place_payload_in_tray",
                    "accept_payload_on_target", "accept_offset_from_target_center_m",
                    "accept_last_speed_mps", "accept_resting_gap_m",
                    "carry_cadence_steps", "place_correction_lateral_m",
                    "place_correction_vertical_m", "place_correction_applied",
                    "place_nominal_solution_offset_m", "place_nominal_solution_lateral_m",
                    "place_carry_payload_lateral_m",
                    "regrasp_pad_minus_payload_z_m", "regrasp_payload_pad_lateral_m",
                    "regrasp_left_force_n", "regrasp_right_force_n", "regrasp_force_ok",
                    "regrasp_pre_lift_delta_m", "regrasp_pad_minus_payload_after_pre_lift_m",
                    "place_settled_gap_m", "place_settled_speed_mps", "place_settled_on_target")

#: 步骤分类（报告里逐项可见，避免"没跑"和"跑过了"混在一起）。
STEP_EXECUTED = "EXECUTED"
STEP_SKIPPED_PENDING = "SKIPPED_PENDING"
STEP_REFUSED_FAULT = "REFUSED_FAULT"
STEP_SAFETY_ACTION_AFTER_FAULT = "SAFETY_ACTION_AFTER_FAULT"
STEP_NOT_EXECUTED_AFTER_FAULT = "NOT_EXECUTED_AFTER_FAULT"

#: 技能清单里的安全动作标记（`skills/<name>/skill.yaml` 的 `spec.safetyClass`）。
#: 故障触发后只有这类步骤继续执行（它们把系统带到安全状态），其余机动步骤一律不下发。
SAFETY_ACTION_CLASS = "safety_action"
#: "被真正下发过"的步骤分类：只有这些步骤的判据参与通过判定（其余是显式登记，不是通过）。
EXECUTED_KINDS = (STEP_EXECUTED, STEP_REFUSED_FAULT, STEP_SAFETY_ACTION_AFTER_FAULT)

SUBMITTER_ROLES = frozenset({"task.submit", "task.read"})
#: 执行器主体（幂等键/执行记录的作用域）。
SUBJECT = "scenario-runner"


class ScenarioError(RuntimeError):
    """带退出码的显式失败：实现层只抛本类型，入口层按 `exc.code` 映射。"""

    code = EXIT_DECLARATION

    def __init__(self, message, code=None):
        super().__init__(message)
        if code is not None:
            self.code = int(code)


# --------------------------------------------------------------------------
# 词表与基础工具
# --------------------------------------------------------------------------
def scenario_contract():
    """从 `config/scene.schema.json` 取词表（单一事实来源，禁止在本文件复制一份）。"""
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    catalog = (schema.get("definitions") or {}).get("scenario_catalog") or {}
    definitions = catalog.get("definitions") or {}
    step_properties = ((definitions.get("step") or {}).get("properties") or {})
    fault_properties = ((definitions.get("fault") or {}).get("properties") or {})
    expect = ((fault_properties.get("expect") or {}).get("properties") or {})
    return {
        "criteria": list((step_properties.get("criteria") or {}).get("propertyNames", {}).get("enum") or []),
        "actions": list((step_properties.get("action") or {}).get("enum") or []),
        "fault_kinds": list((fault_properties.get("kind") or {}).get("enum") or []),
        "terminal_states": list((expect.get("terminal_state") or {}).get("enum") or []),
    }


def _rel(path, root=None):
    """报告里一律用仓库相对路径；仓库外（临时夹具）返回绝对路径。"""
    try:
        return Path(path).resolve().relative_to(Path(root or ROOT).resolve()).as_posix()
    except ValueError:
        return str(path)


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def _load_yaml(path, label):
    if not Path(path).is_file():
        raise ScenarioError("%s 不存在: %s" % (label, path), EXIT_REFERENCE)
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ScenarioError("%s 不是合法 YAML: %s" % (label, exc), EXIT_DECLARATION)
    if not isinstance(document, dict):
        raise ScenarioError("%s 顶层必须是对象" % label, EXIT_DECLARATION)
    return document


def _dig(document, dotted, label):
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            raise ScenarioError("%s 缺少声明键: %s" % (label, dotted), EXIT_DECLARATION)
        node = node[part]
    return node


def default_report_path(scene_id, scenario_name):
    """默认报告路径：`build/acceptance/<scene>/<scenario>/report.json`（按场景+场景名命名空间化）。"""
    return ROOT / "build" / "acceptance" / str(scene_id) / str(scenario_name) / "report.json"


def check_model_available(declaration, root=None):
    """机型声明里声明的模型文件必须已存在，否则退出码 3（先跑声明里的 `model.builder`）。

    只在声明里出现 `model.file` 时检查：没有该键的机型由适配器在装配期自行显式失败
    （不允许静默回退到厂商原始模型，也不允许用"先跑起来再说"顶替）。
    """
    root = Path(root or ROOT)
    model = declaration.get("model")
    if not isinstance(model, dict) or not model.get("file"):
        return None
    path = _resolve(root, model["file"])
    if not path.is_file():
        raise ScenarioError(
            "机型声明声明的模型不存在: %s（先跑声明里的 model.builder 生成场景；"
            "不静默回退到厂商原始模型）" % _rel(path, root),
            EXIT_REFERENCE,
        )
    return path


def load_package(scene_dir):
    """复用 `scene_check` 的三层校验后再取三份声明（不重复实现契约校验）。"""
    try:
        check_report, exit_code = scene_check.check(scene_dir)
    except scene_check.SceneCheckError as exc:
        raise ScenarioError(str(exc), EXIT_REFERENCE)
    if exit_code == scene_check.EXIT_SCHEMA:
        raise ScenarioError(
            "场景包契约层失败（scene_check 退出码 %d）: %s"
            % (exit_code, "；".join(check_report.get("schema_failures") or [])),
            EXIT_DECLARATION,
        )
    if exit_code not in (scene_check.EXIT_OK, scene_check.EXIT_PENDING):
        raise ScenarioError(
            "场景包引用/模型层失败（scene_check 退出码 %d）: %s"
            % (
                exit_code,
                "；".join(
                    (check_report.get("reference_failures") or [])
                    + (check_report.get("model_failures") or [])
                ),
            ),
            EXIT_REFERENCE,
        )
    scene = _load_yaml(Path(scene_dir) / "scene.yaml", "scene.yaml")
    baseline = _load_yaml(Path(scene_dir) / "baseline.yaml", "baseline.yaml")
    scenario = _load_yaml(Path(scene_dir) / "scenario.yaml", "scenario.yaml")
    return scene, baseline, scenario, check_report


def capability_index(scene):
    index = {}
    for robot in scene.get("robots") or []:
        if not isinstance(robot, dict):
            continue
        index[str(robot.get("id"))] = {
            "kind": robot.get("kind"),
            "role": robot.get("role"),
            "capabilities": {str(item) for item in (robot.get("capabilities") or [])},
            "profile": robot.get("profile"),
        }
    return index


def sensor_index(scene):
    index = {}
    for sensor in scene.get("sensors") or []:
        if not isinstance(sensor, dict):
            continue
        anchor = sensor.get("anchor") or {}
        index[str(sensor.get("id"))] = {
            "kind": sensor.get("kind"),
            "entity": str(anchor.get("entity")) if anchor else "",
        }
    return index


#: 场景包声明的"世界"（同一场景在两套模型上的执行绑定）：
#:   single = 各本体各自的单本体产物（默认，历史行为逐位不变）
#:   joint  = 多本体装进同一份 MJCF 的**联合世界**（共享植物；绑定写在 baseline.robots_joint）
WORLD_BINDING_KEYS = {"single": "robots", "joint": "robots_joint"}


def machine_declaration(robot_id, index, baseline, world="single"):
    """解析某本体的机型声明（后端入口 + Profile 路径），并做两侧一致性核对。

    `world` 选择绑定组：`single`（默认，baseline.robots = 各本体的单本体产物）或
    `joint`（baseline.robots_joint = 多本体同一份 MJCF 的联合世界）。缺声明即显式失败，
    不静默退回另一套（单本体与联合产物是两个不同模型，验收数字不可互换）。
    """
    if world not in WORLD_BINDING_KEYS:
        raise ScenarioError(
            "未知世界 %r（可用：%s）" % (world, sorted(WORLD_BINDING_KEYS)), EXIT_DECLARATION)
    bindings = baseline.get(WORLD_BINDING_KEYS[world])
    if not isinstance(bindings, dict) or not bindings:
        raise ScenarioError(
            "场景包未声明 baseline.%s（世界=%s 的机型绑定）：要跑联合世界必须在 baseline.yaml 补这一组，"
            "键集合与 scene.robots[].id 一致" % (WORLD_BINDING_KEYS[world], world), EXIT_REFERENCE)
    profile_ref = index[robot_id].get("profile")
    if not isinstance(profile_ref, str):
        raise ScenarioError(
            "本体 %s 的 profile 仍是待交付占位（%r）：S2 执行器拒绝在该本体上执行步骤" % (robot_id, profile_ref),
            EXIT_REFERENCE,
        )
    baseline_ref = bindings.get(robot_id)
    if not isinstance(baseline_ref, str):
        raise ScenarioError(
            "本体 %s 的 baseline.robots 引用不是路径（%r）：机型声明缺失" % (robot_id, baseline_ref),
            EXIT_REFERENCE,
        )
    declaration_path = _resolve(ROOT, baseline_ref)
    declaration = _load_yaml(declaration_path, "机型声明 %s" % baseline_ref)
    backend = (declaration.get("robot") or {}).get("backend")
    if not isinstance(backend, str) or not backend:
        raise ScenarioError(
            "本体 %s 的机型声明 %s 未声明 robot.backend（S2 执行器绑定）：本战役未接入该本体，"
            "显式失败而不是静默跳过（禁止猜测后端入口）" % (robot_id, baseline_ref),
            EXIT_REFERENCE,
        )
    if backend not in KNOWN_BACKENDS:
        raise ScenarioError(
            "本体 %s 声明的 robot.backend=%r 不在 Backend 登记表里（可用: %s）"
            % (robot_id, backend, sorted(KNOWN_BACKENDS)),
            EXIT_REFERENCE,
        )
    declared_profile = (declaration.get("robot") or {}).get("profile")
    if str(declared_profile) != str(profile_ref):
        raise ScenarioError(
            "本体 %s 的 Profile 引用不一致：scene.yaml=%s，机型声明=%s（两处声明必须同源）"
            % (robot_id, profile_ref, declared_profile),
            EXIT_DECLARATION,
        )
    return {
        "robot": robot_id,
        "declaration": _rel(declaration_path),
        "declaration_document": declaration,
        "backend": backend,
        "profile": str(profile_ref),
        "safety_policy": (declaration.get("skills") or {}).get("safety_policy"),
    }


# --------------------------------------------------------------------------
# 预检：把声明解析成执行计划（装配后端之前完成，避免"跑了一半才发现声明不可执行"）
# --------------------------------------------------------------------------
#: 步骤声明里允许出现的**参数来源键**（解析成技能输入后从 params 里移除）。
PARAM_SOURCE_KEYS = ("grasp_pose_from",)


def resolve_param_sources(step, binding, root):
    """把步骤声明里的**参数来源键**就地解析成技能输入（失败即显式失败，不静默留空）。

    支持：`grasp_pose_from: report_target` —— `pick_object` 必填的 `grasp_pose` 是**目标体在
    世界系的位置 + 朝向**（适配器会拿 `position` 与 `data.xpos[target_body]` 在
    `pose_tolerance_m` 内核对、拿 `orientation` 推出接近轴，见
    `iraf_adapters/mujoco/mujoco_backend.py: pick_object / _resolve_grasp_axis`）。
    声明侧唯一合法来源是**场景报告的 `targets[]`**（报告与模型同源、由构建器写出）⇒
    在场景声明里手抄一组坐标就是第二份事实（AGENTS.md 5.3），故由本函数做契约适配。

    ⚠ 校正一处我先前的误判：我曾以为来源是臂侧参考姿态求解器
    （`scripts/build_piper_baseline.py: build_reference_poses`）——**不对**：那个求解器给的是
    home/approach/grasp/lift 的**关节空间**参考姿态（TCP 轨迹用），而 `pick_object` 的输入契约要的
    是**目标体位姿**。查证方式是读适配器对该参数的用法，而不是从名字推断。
    """
    params = step.get("params") or {}
    if "grasp_pose_from" not in params:
        return
    source = str(params.pop("grasp_pose_from"))
    if source != "report_target":
        raise ScenarioError(
            "步骤 %s 的 grasp_pose_from=%r 不受支持（可用：%s）"
            % (step["id"], source, list(PARAM_SOURCE_KEYS)), EXIT_DECLARATION)
    if not isinstance(binding, dict):
        raise ScenarioError(
            "步骤 %s 声明了 grasp_pose_from 但本体没有机型声明绑定（无法找到场景报告）"
            % step["id"], EXIT_REFERENCE)
    spec = ((binding.get("declaration_document") or {}).get("robot") or {}).get("backend_config") or {}
    report_ref = spec.get("report")
    if not isinstance(report_ref, str) or not report_ref:
        raise ScenarioError(
            "步骤 %s 的 grasp_pose_from=report_target 需要机型声明里的 "
            "robot.backend_config.report（缺声明即失败，不猜来源）" % step["id"], EXIT_DECLARATION)
    report_path = _resolve(root, report_ref)
    if not report_path.is_file():
        raise ScenarioError("场景报告不存在：%s（先跑构建入口生成）" % report_path, EXIT_REFERENCE)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    target_id = str(params.get("target_id"))
    entry = next((item for item in (report.get("targets") or [])
                  if isinstance(item, dict) and str(item.get("id")) == target_id), None)
    if entry is None:
        raise ScenarioError(
            "场景报告 %s 的 targets[] 里没有目标 %s（无法解析 grasp_pose；不猜坐标）"
            % (report_path, target_id), EXIT_REFERENCE)
    position = entry.get("position_m")
    quat = entry.get("quaternion_wxyz")
    if not (isinstance(position, list) and len(position) == 3
            and isinstance(quat, list) and len(quat) == 4):
        raise ScenarioError(
            "场景报告里目标 %s 缺少 position_m(3)/quaternion_wxyz(4)：实际 %r / %r"
            % (target_id, position, quat), EXIT_DECLARATION)
    params["grasp_pose"] = {
        "frame_id": "world",
        "position": {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])},
        # 报告里的四元数按 wxyz 存（MJCF 口径）；技能输入按 xyzw
        "orientation": {"x": float(quat[1]), "y": float(quat[2]), "z": float(quat[3]),
                        "w": float(quat[0])},
    }
    step["params"] = params
    step["param_sources"] = {
        "grasp_pose": {"from": "report_target", "report": _rel(report_path),
                       "target_id": target_id,
                       "source_pose": {"position_m": [float(v) for v in position],
                                       "quaternion_wxyz": [float(v) for v in quat]}}
    }


def plan_steps(entry, index, contract, registry=None):
    _validate_criterion_tables()
    """分类每一步：可执行 / 待交付跳过 / 引用失败。任何未登记的能力缺口都显式失败。"""
    plan = []
    for step in entry.get("steps") or []:
        if not isinstance(step, dict):
            continue
        record = {
            "id": str(step.get("id")),
            "action": str(step.get("action")),
            "robot": str(step.get("robot")),
            "params": step.get("params") or {},
            "criteria": step.get("criteria") or {},
            "pending_closed_by": step.get("pending_closed_by"),
            "pending_reason": step.get("pending_reason"),
            "kind": STEP_EXECUTED,
        }
        if record["action"] not in contract["actions"]:
            raise ScenarioError(
                "步骤 %s 的动作 %r 不在场景契约的动作词表里（先改契再实现）"
                % (record["id"], record["action"]),
                EXIT_DECLARATION,
            )
        robot = index.get(record["robot"])
        if robot is None:
            raise ScenarioError(
                "步骤 %s 引用的本体 %s 不在 scene.yaml 的 robots 里" % (record["id"], record["robot"]),
                EXIT_REFERENCE,
            )
        unknown = [
            key for key in record["criteria"] if key not in contract["criteria"]
        ]
        if unknown:
            raise ScenarioError(
                "步骤 %s 声明了契约之外的成功判据 %s（判据名必须来自既有词表）"
                % (record["id"], unknown),
                EXIT_DECLARATION,
            )
        # 判据的"不可评测"有两个来源（§11.23(46)）：
        #   ① 判据名不在词表 `CRITERION_SPEC` 里；
        #   ② 判据在词表里，但**该步骤技能产不出**它要的证据（例如给 stand 配 pose_tolerance_m）。
        # 两者都必须进 `unsupported`：待交付步骤登记它，会被下发的步骤则在预检显式失败（退出码 2）。
        unsupported = []
        for key in record["criteria"]:
            if key not in CRITERION_SPEC:
                unsupported.append(key)
                continue
            paths = CRITERION_EVIDENCE_PATHS.get(key, "MISSING")
            if paths == "MISSING":
                raise ScenarioError(
                    "判据 %s 未在 CRITERION_EVIDENCE_PATHS 登记证据来源（两张表必须同步）" % key,
                    EXIT_DECLARATION)
            if paths is None:
                continue                      # 来自后端状态/墙钟 ⇒ 任何步骤都可评测
            if _skill_provides_evidence(record["action"], paths, registry) is False:
                unsupported.append(key)
        if record["pending_closed_by"]:
            # **声明了待交付就是待交付**：该步不执行、不参与通过判定 —— 即使能力已经声明。
            # 为什么需要这条（2026-09-24 实测 s03_pick）：它的能力 `pick_object` 确实已声明，
            # 但它必填的 `grasp_pose`（世界系笛卡尔位姿）还没有声明来源（臂侧参考姿态求解器给的
            # 是关节空间参考姿态）⇒ 若继续按"能力已具备"走参数校验，预检会拿"能力存在"与
            # "输入契约未交付"两件事互相打脸、以退出码 2 卡住**整条场景**（连可执行的 s01/s02
            # 也跑不了）。登记待交付 = 显式承认这一步还交不了，比放宽参数校验诚实。
            record["kind"] = STEP_SKIPPED_PENDING
            record["registration"] = {
                "mechanism": "pending_declaration",
                "closed_by": record["pending_closed_by"],
                "reason": record["pending_reason"],
                # 待交付步骤的判据由交付该能力的步骤实现；这里显式登记"本执行器评测不了"，
                # 而不是把整条场景判为非法（该步不会被下发，也不参与通过判定）。
                "unevaluable_criteria": unsupported,
                "capability_already_declared": record["action"] in robot["capabilities"],
            }
        elif record["action"] not in robot["capabilities"]:
            raise ScenarioError(
                "步骤 %s 使用本体 %s 未声明具备的能力 %s，且未登记待交付（缺 pending_closed_by/reason）："
                "显式失败，不静默跳过" % (record["id"], record["robot"], record["action"]),
                EXIT_REFERENCE,
            )
        elif registry is not None and registry.resolve(record["action"], "") is None:
            raise ScenarioError(
                "步骤 %s 的能力 %s 已声明但没有对应的技能清单（skills/%s/skill.yaml）：引用完整性失败"
                % (record["id"], record["action"], record["action"]),
                EXIT_REFERENCE,
            )
        record["unevaluable_criteria"] = unsupported
        plan.append(record)
    if not plan:
        raise ScenarioError("场景条目没有任何步骤（声明为空即是缺口）", EXIT_DECLARATION)
    return plan


def check_evaluable_criteria(plan):
    """将被真正下发的步骤：判据必须可评测，否则就成了"命令已发出"式的假判据（退出码 2）。

    待交付（不会被下发）的步骤不在此列：它们的判据由交付该能力的步骤实现，已在 registration 里登记。
    """
    for step in plan:
        if step["kind"] != STEP_EXECUTED:
            continue
        if step["unevaluable_criteria"]:
            raise ScenarioError(
                "步骤 %s 声明了本执行器没有评测依据的判据 %s：本战役只交付 %s（不得用\"命令已发出\"顶替）"
                % (step["id"], step["unevaluable_criteria"], list(CRITERION_SPEC)),
                EXIT_DECLARATION,
            )


def plan_faults(entry, plan, sensors, contract):
    """故障预检：类型必须已交付、目标传感器必须存在且归属本步本体。"""
    faults = []
    for fault in entry.get("faults") or []:
        if not isinstance(fault, dict):
            continue
        record = {
            "id": str(fault.get("id")),
            "kind": str(fault.get("kind")),
            "at_step": str(fault.get("at_step")),
            "target": str(fault.get("target")),
            "expect": fault.get("expect") or {},
        }
        if record["kind"] not in contract["fault_kinds"]:
            raise ScenarioError(
                "故障 %s 的类型 %r 不在契约词表里" % (record["id"], record["kind"]), EXIT_DECLARATION
            )
        if record["kind"] not in SUPPORTED_FAULT_KINDS:
            raise ScenarioError(
                "故障 %s 的类型 %s 尚未交付（本战役只交付 %s）：显式失败，不静默跳过"
                % (record["id"], record["kind"], list(SUPPORTED_FAULT_KINDS)),
                EXIT_DECLARATION,
            )
        terminal = str(record["expect"].get("terminal_state"))
        if terminal not in contract["terminal_states"]:
            raise ScenarioError(
                "故障 %s 的期望终态 %r 不在状态词表里" % (record["id"], terminal), EXIT_DECLARATION
            )
        if terminal == "SUCCEEDED":
            raise ScenarioError(
                "故障 %s 的期望终态是 SUCCEEDED 但 fake_success_forbidden 恒为 true：声明自相矛盾"
                "（禁区：把安全事件后的终态回写为成功）" % record["id"],
                EXIT_DECLARATION,
            )
        if record["at_step"] not in [step["id"] for step in plan]:
            raise ScenarioError(
                "故障 %s 的 at_step=%s 不是本场景的步骤" % (record["id"], record["at_step"]),
                EXIT_REFERENCE,
            )
        sensor = sensors.get(record["target"])
        if sensor is None:
            raise ScenarioError(
                "故障 %s 的目标 %s 不是 scene.yaml 声明的传感器" % (record["id"], record["target"]),
                EXIT_REFERENCE,
            )
        step_robot = next(step["robot"] for step in plan if step["id"] == record["at_step"])
        if sensor["entity"] != step_robot:
            raise ScenarioError(
                "故障 %s 的目标传感器 %s 归属本体 %s，与注入点步骤 %s 的本体 %s 不一致："
                "无法判定该故障会阻止本步（拒绝猜测）"
                % (record["id"], record["target"], sensor["entity"], record["at_step"], step_robot),
                EXIT_REFERENCE,
            )
        record["target_entity"] = sensor["entity"]
        record["expected_terminal_state"] = terminal
        record["fake_success_forbidden"] = bool(record["expect"].get("fake_success_forbidden"))
        record["injected"] = False
        record["injection_basis"] = (
            "sensor.anchor.entity == 步骤本体（归属关系判定；能力→传感器的语义依赖声明属后续交付）"
        )
        record["safety_actions"] = []
        record["maneuver_steps_not_dispatched"] = []
        faults.append(record)
    return faults


# --------------------------------------------------------------------------
# 运行时装配与逐步执行
# --------------------------------------------------------------------------
SUPPORTED_BACKEND_CONFIG_MODES = ("scene_report",)

# 共享植物（联合场景：狗 + 臂同一个 MJCF）里的角色声明（`robot.plant.role`）：
#   owner = 唯一时间推进者（四足）；guest = 注入 owner 的植物、不得推进时间（臂）。
# 缺声明 ⇒ 各自自建植物 ⇒ 两个独立世界（实测见 docs/debug/2026-09-24-joint-model-dog-arm.md §5.1）。
PLANT_ROLES = ("owner", "guest")


def build_backend_config(root, declaration, spec):
    """按声明的 `robot.backend_config` 构造后端配置（缺失声明＝用声明文件本身）。

    支持的模式（必须显式声明，缺声明/未知模式即显式失败，禁止猜测）：
      * `scene_report`：后端配置取自场景报告 JSON（机械臂路径）——报告必须存在、可解析，
        且含 `output`（模型）与 `gripper`；抓取目标集合由 `targets[]`/`target_id` 构造，
        容差必须由 `target_tolerance_m` 显式给出（不猜默认值）。
    """
    if not isinstance(spec, dict):
        raise ScenarioError(
            "robot.backend_config 必须是对象（声明 mode 与来源）：实际 %r" % (spec,), EXIT_DECLARATION
        )
    mode = str(spec.get("mode") or "")
    if mode not in SUPPORTED_BACKEND_CONFIG_MODES:
        raise ScenarioError(
            "robot.backend_config.mode=%r 不受支持（可用：%s）；缺声明时应删除整段而不是留空"
            % (mode, list(SUPPORTED_BACKEND_CONFIG_MODES)),
            EXIT_DECLARATION,
        )
    report_ref = spec.get("report")
    if not isinstance(report_ref, str) or not report_ref:
        raise ScenarioError(
            "robot.backend_config(mode=scene_report) 必须声明 report 路径（场景报告 JSON）",
            EXIT_DECLARATION,
        )
    report_path = _resolve(root, report_ref)
    if not report_path.is_file():
        raise ScenarioError(
            "场景报告不存在：%s（先跑构建入口生成场景与报告）" % report_path, EXIT_REFERENCE
        )
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioError("场景报告不可解析（%s）：%s" % (report_path, exc), EXIT_REFERENCE)
    if not isinstance(report, dict):
        raise ScenarioError("场景报告必须是对象：%s" % report_path, EXIT_DECLARATION)

    model_path = report.get("output")
    if not isinstance(model_path, str) or not model_path:
        raise ScenarioError("场景报告缺少 output（模型路径）：%s" % report_path, EXIT_DECLARATION)
    if not Path(model_path).is_file():
        raise ScenarioError("场景报告指向的模型不存在：%s" % model_path, EXIT_REFERENCE)
    gripper = report.get("gripper")
    if not isinstance(gripper, dict) or not gripper:
        raise ScenarioError(
            "场景报告缺少 gripper 段（夹爪几何/位形是装配后端的必需项）：%s" % report_path,
            EXIT_DECLARATION,
        )
    tolerance = spec.get("target_tolerance_m")
    if not isinstance(tolerance, (int, float)) or float(tolerance) <= 0:
        raise ScenarioError(
            "robot.backend_config.target_tolerance_m 必须是正数（抓取判据容差只来自声明）",
            EXIT_DECLARATION,
        )
    entries = report.get("targets") or ([{"id": report.get("target_id")}] if report.get("target_id") else [])
    targets = {
        # `geom` 随目标一起透传：`place_object` 要量载荷的**最低点**（`lowest_mesh_point_z`）
        # 才能判"是否落到承载面"，而 geom 名只有报告里才有（不从 id/body 猜）。
        str(item.get("id")): {"body": str(item.get("id")), "geom": item.get("geom"),
                              "pose_tolerance_m": float(tolerance)}
        for item in entries
        if isinstance(item, dict) and item.get("id")
    }
    if not targets:
        raise ScenarioError("场景报告没有可用目标（targets[]/target_id 均为空）：%s" % report_path, EXIT_DECLARATION)

    realtime = spec.get("realtime")
    if not isinstance(realtime, bool):
        raise ScenarioError("robot.backend_config.realtime 必须是布尔值（是否按实时步进）", EXIT_DECLARATION)
    # 联合模型（`--attach` 产物）× 单本体命名的差异：报告带 `manipulation.name_map` 时**透传**给后端，
    # 使其能把声明名（Profile 口径 `joint1`）解析到联合模型里的实际名字（`piper_joint1`）。
    # fail-closed：报告已是联合报告（声明了 manipulation.attached_robot）却没有 name_map ⇒ 拒绝装配 ——
    # 否则臂会等到第一次运动才报"找不到关节或执行器: joint1"，那时已经跑了一半。
    attached = (report.get("manipulation") or {}).get("attached_robot")
    name_map = (report.get("manipulation") or {}).get("name_map")
    # **联合报告的所有权校验**（2026-09-29 新增，多臂场景的硬门禁）：
    # `manipulation`/`gripper`/`targets` 只描述 `scene.model.joint_manipulator` 指定的**那一台**臂。
    # 若本机型声明的 report 指向一份 `attached_robot ≠ 本本体` 的联合报告，装配出来的后端会去驱动
    # **另一台臂**的关节与指腹 —— 而且**不会报错**：实测 ur5e 会解析到 `piper_left_finger`
    # （那个名字在联合模型里**确实存在**），于是"夹爪张开/闭合"实际作用在 piper 上。
    # 这类跨本体错绑必须在装配前拦下（失败要早于任何运动）。注意本检查**不是**临时措辞：
    # 按本体给出 manipulation 段（计划 I1b）落地后，这里要改成"取本本体的段"而不是删掉。
    robot_identity = str(((declaration.get("robot") or {}).get("id")) or "")
    if attached and robot_identity and str(attached) != robot_identity:
        per_robot = {str(item.get("id")): item for item in
                     ((report.get("manipulation") or {}).get("attached_manipulators") or [])
                     if isinstance(item, dict)}
        state = per_robot.get(robot_identity) or {}
        detail = ("该报告已显式登记本本体 `resolved=%s`（未解它的参考姿态/gripper/name_map）"
                  % state.get("resolved")) if state else "该报告里没有本本体的条目"
        raise ScenarioError(
            "机型 %s 的 backend_config.report 指向的联合报告描述的是**另一台臂**"
            "（manipulation.attached_robot=%s；哪台是主臂由 scene.model.joint_manipulator 声明）：%s。"
            "⇒ 按本体的 manipulation 段落地之前，本本体在联合世界里不得跑技能；"
            "也不要让联合场景的步骤把它作为 robot 下发（见 "
            ".hermes/plans/2026-09-29-dual-arm-shuttle-demo.md 的 I1b）"
            % (robot_identity, attached, detail),
            EXIT_DECLARATION,
        )
    if attached and not name_map:
        raise ScenarioError(
            "场景报告 %s 声明了 manipulation.attached_robot=%s（联合模型）却没有 name_map："
            "联合模型里附加本体的对象一律带前缀，臂无法按声明名解析对象 ⇒ 拒绝装配"
            "（重新运行构建入口生成报告即可带上 name_map）" % (report_path, attached),
            EXIT_DECLARATION,
        )
    if name_map is not None and (not isinstance(name_map, dict) or not name_map):
        raise ScenarioError(
            "场景报告 %s 的 manipulation.name_map 必须是非空对象（声明名 → 模型名）" % report_path,
            EXIT_DECLARATION,
        )
    return {
        "model_path": model_path,
        "manipulation": {"targets": targets, "gripper": gripper},
        # 接收体声明（承载面上的放置目标：托盘随载体运动 ⇒ 报告只声明几何 + 标称位姿，
        # 运行期由后端按实测位姿解析）。缺段 = 场景没有接收体，`place_object` 会显式拒绝。
        "place_targets": report.get("place_targets"),
        "vision": report.get("vision"),
        "realtime": realtime,
        "source_report": str(report_path),
        "name_map": name_map,
    }


def assemble(runtime_inputs):
    """装配某一本体的运行时：Profile + 安全策略 + 技能注册表 + 后端（失败不返回半成品）。"""
    root = runtime_inputs["root"]
    profile_path = _resolve(root, runtime_inputs["profile"])
    try:
        profile = load_robot_profile(profile_path)
    except ProfileError as exc:
        raise ScenarioError("Profile 非法（%s）：%s" % (runtime_inputs["profile"], exc), EXIT_DECLARATION)
    safety_ref = runtime_inputs.get("safety_policy")
    if not isinstance(safety_ref, str):
        raise ScenarioError(
            "机型声明 %s 未声明 skills.safety_policy：场景执行必须走安全策略放行清单（铁律 5.3）"
            % runtime_inputs["declaration"],
            EXIT_DECLARATION,
        )
    safety_path = _resolve(root, safety_ref)
    try:
        safety = load_safety_policy(safety_path)
    except ProfileError as exc:
        raise ScenarioError("安全策略非法（%s）：%s" % (safety_ref, exc), EXIT_DECLARATION)
    registry = SkillRegistry().load_directory(Path(root) / "skills")
    authority = ControlAuthorityManager()
    # 后端配置来源：声明里有 robot.backend_config 就按它构造（机械臂走场景报告）；
    # 没有则该声明文件本身即后端配置（四足路径，行为不变）。
    declaration_spec = (runtime_inputs.get("declaration_document") or {}).get("robot") or {}
    config_spec = declaration_spec.get("backend_config")
    if config_spec is None:
        backend_config = str(_resolve(root, runtime_inputs["declaration"]))
    else:
        backend_config = build_backend_config(root, runtime_inputs.get("declaration_document"), config_spec)
    # ---- 共享植物：owner 先建、guest 注入（顺序是硬约束，违反即显式失败）
    plant_spec = declaration_spec.get("plant")
    plant_kwargs = {}
    if plant_spec is not None:
        if not isinstance(plant_spec, dict):
            raise ScenarioError("robot.plant 必须是对象（role/owner/guest_timeout_factor）", EXIT_DECLARATION)
        role = str(plant_spec.get("role") or "")
        if role not in PLANT_ROLES:
            raise ScenarioError(
                "robot.plant.role 必须是 %s 之一，实际 %r；不需要共享植物时删除整段"
                % (list(PLANT_ROLES), role), EXIT_DECLARATION)
        # ⚠ 变量名必须是 `plant_registry`：本函数上面已经有一个同名局部 `registry`（技能注册表），
        #   我第一版复用它 ⇒ **把技能注册表覆盖成植物登记处**，于是 SkillRuntime 拿到一个 dict，
        #   所有技能解析都失败（表现为 `'dict' object has no attribute 'resolve'`）。
        plant_registry = runtime_inputs.get("plant_registry")
        if not isinstance(plant_registry, dict):
            raise ScenarioError(
                "声明了 robot.plant(role=%s) 就必须由场景层传入 plant_registry（共享植物的登记处）："
                "没有登记处就无从共享，拒绝装配而不是各建一份" % role, EXIT_DECLARATION)
        if role == "guest":
            owner_id = plant_spec.get("owner")
            if not isinstance(owner_id, str) or not owner_id:
                raise ScenarioError("robot.plant.role=guest 必须声明 owner（owner 的 robot.id）", EXIT_DECLARATION)
            plant = plant_registry.get(owner_id)
            if plant is None:
                raise ScenarioError(
                    "guest 声明的 plant.owner=%s 尚未创建：装配顺序必须 owner 先于 guest"
                    "（已登记：%s）" % (owner_id, sorted(plant_registry)), EXIT_REFERENCE)
            factor = plant_spec.get("guest_timeout_factor")
            if not isinstance(factor, (int, float)) or float(factor) <= 0:
                raise ScenarioError(
                    "robot.plant.guest_timeout_factor 必须是正数（guest 等 owner 推进时间的墙钟余量，"
                    "实现层不猜）", EXIT_DECLARATION)
            if isinstance(backend_config, dict):
                backend_config = dict(backend_config)
                backend_config["plant"] = plant
                backend_config["plant_guest_timeout_factor"] = float(factor)
            else:
                plant_kwargs = {"plant": plant, "plant_guest_timeout_factor": float(factor)}
    try:
        backend = load_backend(
            KNOWN_BACKENDS[runtime_inputs["backend"]],
            backend_config,
            profile,
            authority,
            **plant_kwargs,
        )
    except ScenarioError:
        raise
    except (ProfileError, ValueError, OSError) as exc:
        raise ScenarioError(
            "后端装配失败（%s/%s）：%s" % (runtime_inputs["backend"], runtime_inputs["declaration"], exc),
            EXIT_BACKEND,
        )
    except Exception as exc:  # 模型编译等：显式失败，不返回半成品
        raise ScenarioError("后端装配失败（模型编译等）：%s" % exc, EXIT_BACKEND)
    if plant_spec is not None and str(plant_spec.get("role")) == "owner":
        # owner 必须真的成为植物的时间推进者，否则"登记成功"是假的（guest 会等一个没人推的植物）
        owned_plant = getattr(backend, "plant", None)
        if owned_plant is None or not owned_plant.is_owner(runtime_inputs["robot"]):
            raise ScenarioError(
                "robot.plant.role=owner 但后端 %s 没有成为植物 owner（%s）"
                % (runtime_inputs["backend"], runtime_inputs["robot"]), EXIT_DECLARATION)
        plant_registry[runtime_inputs["robot"]] = owned_plant
    runtime = SkillRuntime(
        profile, safety, backend, registry, authority, SqliteExecutionStore(":memory:")
    )
    return {
        "runtime": runtime,
        "backend": backend,
        "profile": profile,
        "safety": safety,
        "registry": registry,
        "profile_path": profile_path,
        "safety_path": safety_path,
    }


def _request(profile, safety, skill, parameters, resource_id, correlation, *, key, deadline_offset_ms=60000):
    return {
        "request_id": correlation,
        "idempotency_key": key,
        "correlation_id": correlation,
        "skill": skill,
        "skill_version_constraint": "1.0.0",
        "parameters": parameters,
        "deadline_unix_ms": int(time.time() * 1000) + int(deadline_offset_ms),
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": resource_id,
        "controller": correlation,
    }


def _context():
    return AuthenticatedContext(SUBJECT, SUBMITTER_ROLES, "local")


def measure_step(before, after, evidence, wall_seconds):
    """从实测状态与技能输出里取可用测量量（缺依据的判据在评测时显式失败）。"""
    measured = {"wall_seconds": float(wall_seconds)}
    if isinstance(before, dict) and isinstance(after, dict):
        try:
            measured["sim_time_advance_s"] = float(after["time_s"]) - float(before["time_s"])
        except (KeyError, TypeError, ValueError):
            pass
        speed = after.get("base_linear_velocity_mps")
        if isinstance(speed, (list, tuple)) and len(speed) == 3:
            measured["final_speed_mps"] = math.sqrt(sum(float(v) ** 2 for v in speed))
        elif speed is not None:
            raise ScenarioError("后端 read_state 的 base_linear_velocity_mps 形状非法: %r" % (speed,))
    if isinstance(evidence, dict) and evidence.get("duration_ms") is not None:
        measured["evidence_duration_s"] = float(evidence["duration_ms"]) / 1000.0
    if isinstance(evidence, dict):
        # 停靠结果量：只有**技能自己**给出的实测值才作为依据（缺字段就不给依据 ⇒ 判失败）。
        if evidence.get("final_translation_error_m") is not None:
            measured["dock_translation_error_m"] = float(evidence["final_translation_error_m"])
        if evidence.get("final_yaw_error_deg") is not None:
            # ⚠ 判据是"误差**大小**"：适配器报告里是**带符号**偏航误差 ⇒ 必须取绝对值，
            # 否则 −3° 会以 −3 ≤ 2 的形式**骗过**判据（实测踩点：符号型量直接比阈值必错）。
            measured["dock_yaw_error_deg"] = abs(float(evidence["final_yaw_error_deg"]))
        # 抓取结果量（同样只认技能自己给出的实测值；缺字段就不给依据 ⇒ 评测时判失败）
        alignment = evidence.get("grasp_alignment")
        if isinstance(alignment, dict) and alignment.get("center_distance_m") is not None:
            measured["grasp_center_distance_m"] = float(alignment["center_distance_m"])
        if evidence.get("lift_delta_m") is not None:
            measured["grasp_lift_delta_m"] = float(evidence["lift_delta_m"])
        if evidence.get("bilateral_contact") is not None:
            # 布尔量统一成 1.0/0.0，使 `require_bilateral_contact: true` 能用 "==" 与 `true` 直接比
            measured["grasp_bilateral_contact"] = 1.0 if evidence["bilateral_contact"] else 0.0
        # 放置结果量（同上：只认技能自己给出的实测值）
        place = evidence.get("place_alignment")
        if isinstance(place, dict) and place.get("offset_from_center_m") is not None:
            measured["place_offset_from_tray_center_m"] = float(place["offset_from_center_m"])
        if evidence.get("released") is not None:
            measured["place_released"] = 1.0 if evidence["released"] else 0.0
        if evidence.get("payload_in_tray") is not None:
            measured["place_payload_in_tray"] = 1.0 if evidence["payload_in_tray"] else 0.0
        # 放置段自检失败的诊断（2026-09-29 稳定性实测：6 次里 2 次报"未确认载荷已放下"，
        # 且失败模式单一 ⇒ 必须能看到失败瞬间的落位间隙/末速/自检值）
        for source_key, target_key in (("place_settled_gap_m", "place_settled_gap_m"),
                                       ("place_settled_speed_mps", "place_settled_speed_mps"),
                                       ("place_settled_on_target", "place_settled_on_target")):
            if evidence.get(source_key) is not None:
                value = evidence[source_key]
                measured[target_key] = (1.0 if value else 0.0) if isinstance(value, bool) else float(value)
        # regrasp 调参用的诊断（§11.23(48)）：力与夹持几何必须可观测，否则只能靠猜
        rg = evidence.get("regrasp")
        if isinstance(rg, dict) and rg.get("applied"):
            if rg.get("pre_lift_delta_m") is not None:
                measured["regrasp_pre_lift_delta_m"] = float(rg["pre_lift_delta_m"])
                measured["regrasp_pad_minus_payload_after_pre_lift_m"] = float(
                    rg["pad_minus_payload_z_after_pre_lift_m"])
            cf = rg.get("contact_forces") or {}
            measured["regrasp_pad_minus_payload_z_m"] = float(rg.get("pad_minus_payload_z_m"))
            measured["regrasp_payload_pad_lateral_m"] = float(rg.get("payload_pad_lateral_m"))
            measured["regrasp_left_force_n"] = float(cf.get("left_normal_force_n"))
            measured["regrasp_right_force_n"] = float(cf.get("right_normal_force_n"))
            measured["regrasp_force_ok"] = 1.0 if rg.get("force_ok_after_regrasp") else 0.0
        # 放置点纠偏的实测（§11.23(48)）：把"接收体**实际停靠位姿**与构建期名义位姿之差"暴露成观测量
        # ⇒ 才答得出"放置偏移里有多少是载体停靠误差、有多少是系统性的"，也才能验"纠偏是否真的启用了"。
        correction = evidence.get("place_pose_correction")
        if isinstance(correction, dict) and correction.get("lateral_m") is not None:
            measured["place_correction_lateral_m"] = float(correction["lateral_m"])
            measured["place_correction_vertical_m"] = float(correction["vertical_m"])
            measured["place_correction_applied"] = 1.0 if correction.get("applied") else 0.0
            if correction.get("carry_payload_lateral_m") is not None:
                measured["place_carry_payload_lateral_m"] = float(correction["carry_payload_lateral_m"])
            if correction.get("nominal_solution_offset_m") is not None:
                measured["place_nominal_solution_offset_m"] = float(
                    correction["nominal_solution_offset_m"])
                measured["place_nominal_solution_lateral_m"] = float(
                    correction["nominal_solution_lateral_m"])
        # 载荷确认结果量（`accept_payload`，由**四足侧**独立复核；同样只认技能自己给出的实测值）
        if evidence.get("payload_on_target") is not None:
            measured["accept_payload_on_target"] = 1.0 if evidence["payload_on_target"] else 0.0
        if evidence.get("offset_from_target_center_m") is not None:
            measured["accept_offset_from_target_center_m"] = float(
                evidence["offset_from_target_center_m"])
        if evidence.get("last_speed_mps") is not None:
            measured["accept_last_speed_mps"] = float(evidence["last_speed_mps"])
        if evidence.get("resting_gap_m") is not None:
            measured["accept_resting_gap_m"] = float(evidence["resting_gap_m"])
        # 搬运节拍实测（诊断量，**不是判据**）：随负载波动（实测 25/40 步），归档进报告便于事后对账
        cadence = evidence.get("carry_cadence")
        if isinstance(cadence, dict) and cadence.get("max_plant_steps_per_iteration") is not None:
            measured["carry_cadence_steps"] = float(cadence["max_plant_steps_per_iteration"])
    return measured


def evaluate_criteria(criteria, measured):
    """逐条评测声明判据；没有评测依据即显式失败（不静默通过）。"""
    checks = []
    for name, limit in criteria.items():
        basis, operator, detail = CRITERION_SPEC[name]
        if basis not in measured:
            checks.append(
                {
                    "name": name,
                    "limit": limit,
                    "basis": basis,
                    "measured": None,
                    "passed": False,
                    "detail": "评测依据不可用：%s（%s）" % (basis, detail),
                }
            )
            continue
        value = float(measured[basis])
        if operator == ">=":
            passed = value >= float(limit)
        elif operator == "==":
            # 布尔判据（如 `require_bilateral_contact: true`）：测量量已归一为 1.0/0.0
            passed = value == (1.0 if limit is True else float(limit))
        else:
            passed = value <= float(limit)
        checks.append(
            {
                "name": name,
                "limit": float(limit),
                "basis": basis,
                "operator": operator,
                "measured": value,
                "passed": bool(passed),
                "detail": detail,
            }
        )
    return checks


def _step_record(step):
    return {
        "id": step["id"],
        "action": step["action"],
        "robot": step["robot"],
        "params": step["params"],
        "criteria": step["criteria"],
        "kind": step["kind"],
    }


def _read_backend_state(backend):
    """读后端状态用于步骤前后测量；**未实现 `read_state` 即返回 None**（不假装有测量）。

    ⚠ 实测（2026-09-24）：`MujocoBackend`（机械臂）没有 `read_state` —— 只有四足后端有
    ⇒ 旧实现直接 `AttributeError: 'MujocoBackend' object has no attribute 'read_state'`，
    机械臂**第一次**被本执行器下发时就崩（此前 s03 待交付、臂从未被下发过，所以没显形）。
    这里按"无测量"处理：依赖 `sim_time_advance_s` / `final_speed_mps` 的判据会因**缺依据**
    而判失败（`measure_step` + `evaluate_criteria` 的既有语义），不会静默通过。
    """
    reader = getattr(backend, "read_state", None)
    return reader() if callable(reader) else None


def _dispatch_step(runtime_state, step, correlation, key):
    """一次真实的技能执行：走 SkillRuntime（TaskFlow → Policy → 租约 → Provider → 适配器）。

    判据取证用**后端实测状态**（执行前后各读一次），因此"命令已发出"不会被当成"结果达成"。
    """
    runtime = runtime_state["runtime"]
    profile = runtime_state["profile"]
    resource_id = profile.name
    before = _read_backend_state(runtime_state["backend"])
    started = time.perf_counter()
    result = runtime.execute(
        _request(
            profile,
            runtime_state["safety"],
            step["action"],
            step["params"],
            resource_id,
            correlation,
            key=key,
        ),
        _context(),
    )
    wall_seconds = time.perf_counter() - started
    after = _read_backend_state(runtime_state["backend"])
    evidence = (result.get("result") or {}).get("evidence") or {}
    measured = measure_step(before, after, evidence, wall_seconds)
    checks = evaluate_criteria(step["criteria"], measured)
    record = _step_record(step)
    record.update(
        {
            "status": str(result.get("status", "")),
            "error_code": str(result.get("error_code", "")),
            "reason": str(result.get("reason", "")),
            "execution_id": str(result.get("execution_id", "")),
            "wall_seconds": wall_seconds,
            "measured": {name: measured[name] for name in MEASUREMENT_KEYS if name in measured},
            "checks": checks,
            "passed": bool(result.get("status") == "SUCCEEDED" and all(c["passed"] for c in checks)),
        }
    )
    return record


# --------------------------------------------------------------------------
# 植物驻留（联合世界）：owner 在 guest 执行期间**持续在线**
# --------------------------------------------------------------------------
def _plant_hold_spec(binding):
    plant = ((binding.get("declaration_document") or {}).get("robot") or {}).get("plant") or {}
    return str(plant.get("role") or ""), plant.get("hold")


# --------------------------------------------------------------------------
# 运行期显示（**只渲染、不推进**）：验收运行边跑边看
# --------------------------------------------------------------------------
def _start_run_display(runtimes, bindings, world, display, render_hz, hold_seconds=0.0):
    """按 `--display` 打开窗口镜像；**不推进**仿真（推进由 owner 驻留线程负责）。

    为什么不让窗口推进（2026-09-28，docs/debug/2026-09-24-joint-model-dog-arm.md §11.9）：
    joint 世界里"谁推进时间"是**声明问题**（`_start_plant_residency`）；窗口若也 step，
    就是第二个时间推进者（会与 owner 争抢同一株植物）。而演示脚本自己造推进者会撞安全策略的
    `max_duration_ms`（实测 stand 60000 ms 被技能层直接拒 ⇒ owner 停步 ⇒ guest 卡在
    「等待 owner 推进」）⇒ 正解是给**验收运行本身**加显示通路，而不是在脚本里造推进者。
    """
    if not display or str(display) == "none":
        return None
    owner_id = None
    for robot_id, binding in (bindings or {}).items():
        role, _hold = _plant_hold_spec(binding)
        if role == "owner":
            owner_id = robot_id
            break
    if owner_id is None:
        owner_id = next(iter(runtimes or {}), None)
    state = (runtimes or {}).get(owner_id)
    backend = (state or {}).get("backend")
    if backend is None or not hasattr(backend, "model"):
        return {"display": str(display), "error": "找不到可渲染的后端（owner=%s）" % owner_id,
                "window_opened": False, "frames": 0}
    stop_event = threading.Event()
    box = {}
    # **渲染声明**（机型声明的 `render` 段）：软件 GL 开关、窗口像素、固定相机。
    # 为什么必须读声明（2026-09-28 §11.23(41)）：本机（4 核、无独显）默认 GL 路径下窗口 3D 视口
    # 几乎不亮，llvmpipe 在大窗口下每帧数秒 ⇒ 使用者反馈"看不到 / 闪一下"。
    # 见 docs/debug/2026-09-23-go2-viewer-3d-black-screen.md。
    owner_binding = (bindings or {}).get(owner_id) or {}
    render_spec = ((owner_binding.get("declaration_document") or {}).get("render") or {})
    software_gl = bool(render_spec.get("software_gl"))
    width_px, height_px = render_spec.get("width_px"), render_spec.get("height_px")
    window_px = ((int(width_px), int(height_px))
                 if isinstance(width_px, int) and isinstance(height_px, int) else None)
    camera = (str(render_spec["camera"]) if render_spec.get("camera") else None)

    def loop():
        try:
            from iraf_adapters.mujoco.viewer_runner import run_live_mirror
            box["report"] = run_live_mirror(backend, render_hz=render_hz, stop_event=stop_event,
                                            display_mode=str(display), camera=camera,
                                            software_gl=software_gl, window_px=window_px,
                                            hold_seconds=hold_seconds)
        except Exception as exc:  # noqa: BLE001 —— 显示失败不得影响验收结论，但必须留痕
            box["error"] = "%s: %s" % (type(exc).__name__, exc)

    thread = threading.Thread(target=loop, name="run-display", daemon=True)
    thread.start()
    time.sleep(0.5)   # 让窗口先起来（否则前几步的观察窗口很短）
    return {"display": str(display), "owner": owner_id, "render_hz": float(render_hz),
            "software_gl": software_gl, "window_px": window_px, "camera": camera,
            "stop_event": stop_event, "thread": thread, "box": box}


def _stop_run_display(summary, seconds):
    """结束显示会话并汇总（`seconds>0` = 运行结束后窗口再留这么多秒，便于看末态）。"""
    if not summary:
        return None
    if float(seconds or 0) > 0:
        time.sleep(float(seconds))
    summary["stop_event"].set()
    summary["thread"].join(timeout=60.0)
    report = dict(summary["box"].get("report") or {})
    if summary["box"].get("error"):
        report["error"] = summary["box"]["error"]
    return {"display": summary["display"], "owner_backend": summary["owner"],
            "render_hz": summary["render_hz"],
            "thread_alive_after_stop": bool(summary["thread"].is_alive()),
            "window_opened": report.get("window_opened"),
            "frames": report.get("frames"), "display_mode": report.get("display_mode"),
            "stopped_by": report.get("stopped_by"), "error": report.get("error"),
            "note": ("只渲染、不推进：时间由 owner 驻留线程推进（见 _start_run_display）")}


def _start_plant_residency(runtimes, bindings, world):
    """按声明让 owner 在整场执行期间持续执行其 `hold` 技能；非联合世界不启动。

    为什么必须（2026-09-24 实测，docs/debug/2026-09-24-joint-model-dog-arm.md §11.1）：
    场景执行器是**逐步串行**的 ⇒ 执行 guest（臂）的步骤时没人推进植物，guest 的等待必然超时
    （实测 `等待 owner(unitree_go2) 推进到第 10751 步超时（0.060 s，当前 10750 步）`）；
    且四足是**力矩型执行器**（PD 在适配器每拍计算）⇒ 没人算控制量它会塌（§5.2 量化 0.288372 →
    0.077172 / 0.1 s）。驻留线程把"owner 一直活着"这件事变成声明驱动的行为，而不是靠步骤顺序碰巧。
    """
    if world != "joint":
        return None
    owner_id = None
    spec = None
    for robot_id, binding in bindings.items():
        role, hold = _plant_hold_spec(binding)
        if role == "owner":
            owner_id, spec = robot_id, hold
            break
    if owner_id is None:
        return None
    if not isinstance(spec, dict):
        raise ScenarioError(
            "联合世界（world=joint）要求 owner 在机型声明里给出 robot.plant.hold"
            "（{skill, duration_ms}）：否则执行 guest 步骤时无人推进植物、等待必然超时"
            "（实测见 docs/debug/2026-09-24-joint-model-dog-arm.md §11.1）", EXIT_DECLARATION)
    skill = spec.get("skill")
    duration_ms = spec.get("duration_ms")
    if not isinstance(skill, str) or not skill:
        raise ScenarioError("robot.plant.hold.skill 必须是非空技能名", EXIT_DECLARATION)
    if not isinstance(duration_ms, int) or isinstance(duration_ms, bool) or duration_ms <= 0:
        raise ScenarioError("robot.plant.hold.duration_ms 必须是正整数（实现层不写默认值）",
                            EXIT_DECLARATION)
    state = runtimes.get(owner_id)
    if state is None:
        raise ScenarioError(
            "声明的植物 owner %s 未在本场景装配（它的 hold 技能无法执行）" % owner_id, EXIT_REFERENCE)
    profile = state["profile"]
    stop_event = threading.Event()
    # 让位闸门：owner **自己**执行场景步骤时必须停驻留（见 _yield_residency_to_step）
    pause_event = threading.Event()
    idle_event = threading.Event()
    idle_event.set()
    records = []

    def loop():
        cycle = 0
        while not stop_event.is_set():
            if pause_event.is_set():
                # 让位：声明"我不在飞"，等 owner 自己的步骤跑完再继续
                idle_event.set()
                time.sleep(0.05)
                continue
            idle_event.clear()
            cycle += 1
            started = time.monotonic()
            status, error = "", ""
            try:
                request = _request(
                    profile, state["safety"], skill, {"duration_ms": int(duration_ms)},
                    profile.name, "plant-residency-%s-%d" % (owner_id, cycle),
                    key="plant-residency-%s-%d" % (owner_id, cycle),
                    deadline_offset_ms=600000)
                result = state["runtime"].execute(request, _context())
                status = str((result or {}).get("status", ""))
                error = "" if status == "SUCCEEDED" else str((result or {}).get("reason", ""))
            except Exception as exc:  # noqa: BLE001 —— 留痕后停止驻留，让主流程看到"驻留已断"
                status, error = "EXCEPTION", "%s: %s" % (type(exc).__name__, exc)
            idle_event.set()
            records.append({"cycle": cycle, "status": status, "error": error,
                            "wall_seconds": time.monotonic() - started})
            if status != "SUCCEEDED":
                break

    thread = threading.Thread(target=loop, name="plant-residency", daemon=True)
    thread.start()
    return {"owner": owner_id, "skill": skill, "duration_ms": int(duration_ms),
            "stop_event": stop_event, "pause_event": pause_event, "idle_event": idle_event,
            "thread": thread, "records": records, "state": state,
            "paused_steps": [], "paused_seconds": 0.0}


def _yield_residency_to_step(residency, robot_id):
    """owner 自己执行场景步骤时让驻留**让位**；返回 {"paused": bool}。

    为什么必须（2026-09-28 实测）：联合世界里 s02_dock 由 owner（四足）自己执行，而驻留线程同时在
    跑 `stand` 循环 ⇒ **两条执行流并发驱动同一株植物、写同一批执行器**，结果取决于线程调度：
    · 无驻留的探针 7 次逐位相同（0.028587600087094413，含 4 个忙循环加压下的 2 次）；
    · 带驻留的 runner 三次给出 0.02812501214102655 / 0.028375962824781182 / 0.03056883116091061
      —— 最后一次**越过声明上限 0.03**（把"机器负载"当成原因是误判，真因是这个竞态）。
    让位后 owner 的步骤独占执行 ⇒ 结果可复现（与探针同口径）。
    """
    if not residency or str(robot_id) != str(residency.get("owner")):
        return {"paused": False}
    started = time.monotonic()
    residency["pause_event"].set()
    if not residency["idle_event"].wait(timeout=60.0):
        raise ScenarioError(
            "植物驻留未在 60 s 内让位（owner 上一轮 hold 未结束）：owner=%s"
            % residency.get("owner"), EXIT_BACKEND)
    residency["paused_seconds"] = float(residency.get("paused_seconds", 0.0)) + (
        time.monotonic() - started)
    return {"paused": True}


def _resume_residency_after_step(residency, yield_record, step_id):
    if residency and yield_record and yield_record.get("paused"):
        residency["paused_steps"].append(str(step_id))
        residency["pause_event"].clear()


def _stop_plant_residency(residency):
    """停止驻留并返回汇总（周期数、失败周期、owner 收尾实测）。"""
    if residency is None:
        return None
    residency["stop_event"].set()
    residency["thread"].join(timeout=120.0)
    records = residency["records"]
    summary = {"owner": residency["owner"], "skill": residency["skill"],
               "duration_ms": residency["duration_ms"], "cycles": len(records),
               "paused_steps": list(residency.get("paused_steps") or []),
               "paused_seconds": round(float(residency.get("paused_seconds", 0.0)), 6),
               "failed_cycles": [item for item in records if item["status"] != "SUCCEEDED"],
               "wall_seconds": round(sum(item["wall_seconds"] for item in records), 6),
               "thread_alive_after_stop": bool(residency["thread"].is_alive())}
    reader = getattr(residency["state"]["backend"], "read_state", None)
    if callable(reader):
        try:
            final = reader() or {}
            summary["owner_final_state_keys"] = sorted(final.keys())
            position = final.get("base_position_m")
            if isinstance(position, (list, tuple)) and len(position) == 3:
                summary["owner_final_base_z_m"] = float(position[2])
        except Exception as exc:  # noqa: BLE001 —— 只记录读取失败，不影响判定
            summary["owner_final_state_error"] = "%s: %s" % (type(exc).__name__, exc)
    return summary


def execute_steps(plan, faults, runtimes, registry, scenario_name, scene_id, residency=None):
    """按声明顺序执行；故障注入点拒绝下发，其后只允许声明为安全动作的步骤继续。"""
    records = []
    faults_by_step = {}
    for fault in faults:
        faults_by_step.setdefault(fault["at_step"], []).append(fault)
    fault_fired = False
    fired_faults = []
    for step in plan:
        record = _step_record(step)
        step_faults = faults_by_step.get(step["id"]) or []
        if step["kind"] == STEP_SKIPPED_PENDING:
            if step_faults:
                # 注入点本身未执行（能力待交付）：故障无法注入，如实登记为未注入。
                for fault in step_faults:
                    fault["not_injected_reason"] = (
                        "注入点步骤 %s 处于待交付（%s），未执行 ⇒ 故障未注入，期望终态 %s 未被验证"
                        % (step["id"], step["pending_closed_by"], fault["expected_terminal_state"])
                    )
            record["reason"] = "能力待交付：%s（%s）" % (step["action"], step["pending_closed_by"])
            record["registration"] = step["registration"]
            records.append(record)
            continue
        if fault_fired:
            skill = registry.resolve(step["action"], "1.0.0")
            safety_class = str(getattr(skill.manifest, "safety_class", ""))
            if safety_class != SAFETY_ACTION_CLASS:
                # 故障后不再下发机动指令：这是"未继续自主机动"的证据，不是"跳过"。
                record["kind"] = STEP_NOT_EXECUTED_AFTER_FAULT
                record["registration"] = {
                    "mechanism": "fault_abort",
                    "reason": "故障注入后不再下发后续机动指令（进入声明终态）",
                }
                for fault in fired_faults:
                    fault["maneuver_steps_not_dispatched"].append(step["id"])
                records.append(record)
                continue
            correlation = "scenario:%s:%s:%s" % (scene_id, scenario_name, step["id"])
            key = "%s:%s" % (scenario_name, step["id"])
            executed = _dispatch_step(runtimes[step["robot"]], step, correlation, key)
            executed["kind"] = STEP_SAFETY_ACTION_AFTER_FAULT
            executed["safety_class"] = safety_class
            executed["detail"] = (
                "故障后执行声明的安全动作（技能清单 safetyClass=%s），把系统带到安全状态"
                % safety_class
            )
            for fault in fired_faults:
                fault["safety_actions"].append(step["id"])
            records.append(executed)
            continue
        runtime_state = runtimes[step["robot"]]
        runtime = runtime_state["runtime"]
        resource_id = runtime_state["profile"].name
        correlation = "scenario:%s:%s:%s" % (scene_id, scenario_name, step["id"])
        key = "%s:%s" % (scenario_name, step["id"])
        if step_faults:
            fault = step_faults[0]
            reason = (
                "传感器 %s 不可用（场景故障注入 %s，归属本体 %s）：拒绝下发 %s 指令，"
                "进入声明终态 %s，禁止伪造成功"
                % (fault["target"], fault["id"], fault["target_entity"], step["action"],
                   fault["expected_terminal_state"])
            )
            result = runtime.record_pre_dispatch_failure(
                _request(
                    runtime_state["profile"],
                    runtime_state["safety"],
                    step["action"],
                    step["params"],
                    resource_id,
                    correlation,
                    key=key,
                ),
                _context(),
                PRECONDITION_FAILED_CODE,
                reason,
                metadata={"resolved_skill": step["action"]},
            )
            record.update(
                {
                    "kind": STEP_REFUSED_FAULT,
                    "status": str(result.get("status", "")),
                    "error_code": str(result.get("error_code", "")),
                    "reason": str(result.get("reason", "")),
                    "execution_id": str(result.get("execution_id", "")),
                    "wall_seconds": 0.0,
                    "fault": fault["id"],
                }
            )
            record["passed"] = bool(
                record["status"] != "SUCCEEDED"
                and record["error_code"] == PRECONDITION_FAILED_CODE
                and reason in record["reason"]
            )
            fault["injected"] = True
            fault["refused_step"] = step["id"]
            fault["refusal_error_code"] = record["error_code"]
            fault_fired = True
            fired_faults = step_faults
            records.append(record)
            continue
        yield_record = _yield_residency_to_step(residency, step["robot"])
        try:
            executed = _dispatch_step(runtime_state, step, correlation, key)
        finally:
            _resume_residency_after_step(residency, yield_record, step["id"])
        executed["plant_residency_yielded"] = bool(yield_record.get("paused"))
        records.append(executed)
    return records


def _fault_records(faults):
    """把故障的执行结果整理成报告条目（未注入的必须给出原因）。"""
    records = []
    for fault in faults:
        record = {
            "id": fault["id"],
            "kind": fault["kind"],
            "at_step": fault["at_step"],
            "target": fault["target"],
            "target_entity": fault["target_entity"],
            "expected_terminal_state": fault["expected_terminal_state"],
            "fake_success_forbidden": fault["fake_success_forbidden"],
            "injected": bool(fault["injected"]),
            "injection_basis": fault["injection_basis"],
            "limitation": (
                "注入的语义是「在故障点拒绝下发机动指令，其后只允许 skills 清单里 "
                "safetyClass=safety_action 的步骤继续」；本条证明「未继续自主机动、未伪造成功」，"
                "不构成真机/物理安全停机证据（真机与目标端验收 DEFERRED）"
            ),
        }
        if fault["injected"]:
            record["refused_step"] = fault["refused_step"]
            record["refusal_error_code"] = fault["refusal_error_code"]
            record["safety_actions_executed"] = list(fault["safety_actions"])
            record["maneuver_steps_not_dispatched"] = list(fault["maneuver_steps_not_dispatched"])
            record["scenario_state_after_fault"] = fault["expected_terminal_state"]
            record["declared_terminal_state_matched"] = (
                fault["expected_terminal_state"] == SAFE_HOLD
            )
            # fake_success / verified 不在这里预置结论：必须在 run_scenario 里按**实测的执行记录**算，
            # 否则就是一个恒真的自证字段（步骤 17 的"假用例"教训）。
        else:
            record["not_injected_reason"] = fault.get(
                "not_injected_reason", "故障未触发（注入点步骤未执行）"
            )
            record["verified"] = False
            record["fake_success"] = None
        records.append(record)
    return records


def run_scenario(scene_dir, scenario_name, *, report_path=None, require_injected_faults=False,
                 world="single", display=None, render_hz=20.0, seconds=0.0):
    """执行一个场景并写报告；返回 (report, exit_code)。"""
    scene_dir = Path(scene_dir)
    if not scene_dir.is_dir():
        raise ScenarioError("场景目录不存在: %s" % scene_dir, EXIT_USAGE)
    scene, baseline, scenario_document, check_report = load_package(scene_dir)
    scenarios = scenario_document.get("scenarios") or {}
    if scenario_name not in scenarios:
        raise ScenarioError(
            "场景 %r 不在 %s 的 scenarios 里（可用: %s）"
            % (scenario_name, _rel(scene_dir / "scenario.yaml"), sorted(scenarios)),
            EXIT_REFERENCE,
        )
    entry = scenarios[scenario_name] or {}
    contract = scenario_contract()
    index = capability_index(scene)
    sensors = sensor_index(scene)

    # ---- 预检（装配后端之前）------------------------------------------------
    skill_root = ROOT / "skills"
    if not skill_root.is_dir():
        raise ScenarioError("技能目录不存在: %s" % skill_root, EXIT_REFERENCE)
    try:
        registry = SkillRegistry().load_directory(skill_root)
    except RegistryError as exc:
        raise ScenarioError("技能清单无法加载: %s" % exc, EXIT_REFERENCE)
    plan = plan_steps(entry, index, contract, registry=registry)
    faults = plan_faults(entry, plan, sensors, contract)

    # 绑定普查（只读声明、不装配）：将被真正下发的本体必须声明 robot.backend，
    # 否则在装配/运动之前就以退出码 3 停下（"未接入"比"跑到一半才发现"便宜得多）。
    bindings = {}
    for step in plan:
        if step["kind"] != STEP_EXECUTED or step["robot"] in bindings:
            continue
        binding = machine_declaration(step["robot"], index, baseline, world=world)
        check_model_available(binding["declaration_document"], ROOT)
        bindings[step["robot"]] = binding

    # ---- 参数来源解析（声明 → 技能输入）：必须在参数契约校验**之前**做，否则
    # `grasp_pose_from` 这种来源键会被技能 schema 当成非法字段拒掉。
    for step in plan:
        if step["kind"] != STEP_EXECUTED:
            continue
        resolve_param_sources(step, bindings.get(step["robot"]), ROOT)

    check_evaluable_criteria(plan)

    # 参数必须符合技能自身的输入契约：声明写完就跑，避免"声明合法但参数非法"的步骤被 Policy 拒绝。
    for step in plan:
        if step["kind"] != STEP_EXECUTED:
            continue
        skill = registry.resolve(step["action"], "1.0.0")
        try:
            skill.validate_inputs(step["params"])
        except RegistryError as exc:
            raise ScenarioError(
                "步骤 %s 的参数不符合技能 %s 的输入契约：%s"
                % (step["id"], step["action"], exc),
                EXIT_DECLARATION,
            )

    runtimes = {}
    robot_records = []
    # 共享植物的登记处（联合世界）：owner 先装配并把植物登记进来，guest 再注入**同一株**；
    # `assemble` 内部强制"guest 早于 owner ⇒ 显式失败"，这里按声明角色排序把顺序变成确定的
    # （不依赖 steps 里谁先出现 —— 那种隐式顺序会在步骤重排后静默失效）。
    plant_registry = {}

    def _plant_role(binding):
        spec = ((binding.get("declaration_document") or {}).get("robot") or {}).get("plant") or {}
        return str(spec.get("role") or "")

    for robot_id, binding in sorted(
            bindings.items(), key=lambda item: 0 if _plant_role(item[1]) == "owner" else 1):
        state = assemble(
            {
                "root": ROOT,
                "robot": robot_id,
                "profile": binding["profile"],
                "declaration": binding["declaration"],
                # ⚠ **必须传 declaration_document**：`scene_report` 模式（机械臂）的后端配置
                # 要从声明里的 `robot.backend_config` 构造；只传路径会让 `assemble` 退化成
                # "声明文件本身即后端配置"，于是 `mujoco_arm` 拿到一个字符串
                # ⇒ `'str' object has no attribute 'get'`（实测：s03_pick 解除待交付、臂第一次
                # 被本执行器装配时才暴露；此前臂从未被装配过所以一直没显形）。
                "declaration_document": binding.get("declaration_document"),
                "backend": binding["backend"],
                "safety_policy": binding["safety_policy"],
                "plant_registry": plant_registry,
            }
        )
        runtimes[robot_id] = state
        robot_records.append(
            {
                "id": robot_id,
                "kind": index[robot_id]["kind"],
                "profile": _rel(state["profile_path"]),
                "profile_name": state["profile"].name,
                "profile_digest": state["profile"].digest,
                "machine_declaration": binding["declaration"],
                "backend_entry": binding["backend"],
                "safety_policy": _rel(state["safety_path"]),
                "capabilities": sorted(index[robot_id]["capabilities"]),
                "backend_contract": getattr(state["backend"], "backend_contract", {}),
            }
        )

    scene_id = str(scene.get("id"))
    # 植物驻留：联合世界下 owner 全程在线（见 _start_plant_residency 的说明）
    residency = _start_plant_residency(runtimes, bindings, world)
    # 显示会话：只渲染、不推进（推进仍是 owner 驻留线程的事）。`--seconds` 作为**末态留观**
    # 传进镜像循环本身（在那里窗口还活着）⇒ 运行结束后窗口仍可见 N 秒，而不是关窗后再空等
    # （2026-09-28 §11.23(41)：旧语义正是"闪一下就不见了"）。
    display_summary = _start_run_display(runtimes, bindings, world, display, render_hz,
                                         hold_seconds=seconds)
    try:
        steps = execute_steps(plan, faults, runtimes, registry, scenario_name, scene_id,
                              residency=residency)
    finally:
        display_summary = _stop_run_display(display_summary, 0.0)
        residency_summary = _stop_plant_residency(residency)
    fault_records = _fault_records(faults)

    # 故障条目按**实测的执行记录**判定，而不是预置结论：被拒绝的步骤不得是 SUCCEEDED
    # （否则就是"安全事件被回写成成功"，铁律 1.6）。
    status_by_step = {item["id"]: item.get("status") for item in steps}
    fake_success_faults = []
    for item in fault_records:
        if not item["injected"]:
            continue
        refused_status = str(status_by_step.get(item.get("refused_step"), ""))
        item["refused_step_status"] = refused_status
        item["fake_success"] = bool(refused_status == "SUCCEEDED")
        item["verified"] = bool(item["declared_terminal_state_matched"] and not item["fake_success"])
        if item["fake_success"]:
            fake_success_faults.append(item["id"])

    failed_checks = [
        "步骤 %s（%s）状态 %s%s：%s"
        % (
            item["id"],
            item["action"],
            item.get("status"),
            "，错误码 %s" % item["error_code"] if item.get("error_code") else "",
            item.get("reason") or "判据未满足",
        )
        for item in steps
        if item["kind"] in EXECUTED_KINDS and not item.get("passed")
    ]
    for item in steps:
        if item["kind"] in EXECUTED_KINDS and not item.get("passed"):
            for check in item.get("checks") or []:
                if not check["passed"]:
                    failed_checks.append(
                        "步骤 %s 判据 %s 未满足：%s %s，测量 %r（%s）"
                        % (
                            item["id"],
                            check["name"],
                            check.get("operator", "") or "<无依据>",
                            check["limit"],
                            check["measured"],
                            check["detail"],
                        )
                    )
    for item in fault_records:
        if item["injected"] and not item.get("declared_terminal_state_matched"):
            failed_checks.append(
                "故障 %s 的期望终态 %s 与执行器实际进入的状态不一致"
                % (item["id"], item["expected_terminal_state"])
            )
    for name in fake_success_faults:
        failed_checks.append(
            "故障 %s 注入后步骤仍报 SUCCEEDED：伪造成功（铁律 1.6 禁止把终态回写为成功）" % name
        )
    unverified_faults = [item["id"] for item in fault_records if not item["verified"]]
    if require_injected_faults and unverified_faults:
        for item in fault_records:
            if not item["verified"]:
                failed_checks.append(
                    "--require-injected-faults：故障 %s 未注入/未验证（%s）"
                    % (item["id"], item.get("not_injected_reason", ""))
                )

    pending_steps = [item["id"] for item in steps if item["kind"] == STEP_SKIPPED_PENDING]
    unregistered = [
        item["id"]
        for item in steps
        if item["kind"] in (STEP_SKIPPED_PENDING, STEP_NOT_EXECUTED_AFTER_FAULT)
        and not item.get("registration")
    ]
    report = {
        "schema_version": "iraf.scenario-run/v1",
        "simulation": True,
        "scene": _rel(scene_dir),
        "scene_id": scene_id,
        "scenario": scenario_name,
        "description": entry.get("description"),
        "runner": {
            "entrypoint": "scripts/scenario.py",
            "supported_fault_kinds": list(SUPPORTED_FAULT_KINDS),
            "evaluable_criteria": list(CRITERION_SPEC),
            "failure_classes": {
                "sensor_unavailable": (
                    "故障点拒绝下发该步指令（落一条带错误码的执行记录），其后只允许技能清单里 "
                    "safetyClass=safety_action 的步骤继续"
                ),
                "capability_not_declared": (
                    "步骤能力未声明：有 pending 登记则显式跳过并记入 pending_steps，无登记则退出码 3"
                ),
            },
            "declaration_discipline": "阈值/路径/耗时/后端绑定全部来自场景包与机型声明；本入口无默认值",
        },
        "package": {
            "scene_check_exit_code": check_report["exit_code"],
            "scene_check_passed": check_report["passed"],
            "pending_refs": len(check_report.get("pending_refs") or []),
            "pending_steps": len(check_report.get("pending_steps") or []),
        },
        "robots": robot_records,
        "steps": steps,
        "faults": fault_records,
        "counts": {
            "steps": len(steps),
            "executed": len([item for item in steps if item["kind"] == STEP_EXECUTED]),
            "skipped_pending": len(pending_steps),
            "refused_fault": len([item for item in steps if item["kind"] == STEP_REFUSED_FAULT]),
            "safety_action_after_fault": len(
                [item for item in steps if item["kind"] == STEP_SAFETY_ACTION_AFTER_FAULT]
            ),
            "not_executed_after_fault": len(
                [item for item in steps if item["kind"] == STEP_NOT_EXECUTED_AFTER_FAULT]
            ),
            "faults": len(fault_records),
            "injected_faults": len([item for item in fault_records if item["injected"]]),
            "unverified_faults": len(unverified_faults),
        },
        # 本次执行用的**世界**（single/joint）：两套绑定的验收数字不可互换，必须随报告留痕。
        "world": str(world),
        # 植物驻留（联合世界）的实测：周期数、失败周期、owner 收尾基座高度 ⇒ 证明"owner 全程在线"
        "plant_residency": residency_summary,
        # 运行期显示（`--display interactive_viewer`）：只渲染、不推进 ⇒ 不影响任何判据；
        # 留痕窗口是否真开、渲染帧数、被谁结束（便于"我看到的和报告里的是一次运行"）
        "display": display_summary,
        "pending_steps": pending_steps,
        "unregistered_steps": unregistered,
        "unverified_faults": unverified_faults,
        "not_proved": [
            "判据口径：min_stable_hold_s = 本步推进的仿真时间，max_speed_m_s = 步骤结束时实测末速；"
            "两者都不是稳定性分析（物理稳定性证据见步骤 15 的 build/acceptance/go2-loopback/report.json）。",
            # 待交付范围**按声明动态列出**，不写死能力名单：原文写"只覆盖 stand/stop"，而 dock/pick
            # 交付后这句仍照旧输出 ⇒ 文案随交付漂移就变成假陈述（2026-09-24 实测踩到）。
            "本执行器只执行声明里**可执行**的步骤；登记为待交付的步骤被显式跳过、不参与通过判定"
            # ⚠ `pending_steps` 是**步骤 id 的字符串列表**（不是字典）；按字典写会在有待交付步骤的
            #   场景上崩 —— 本次就是在 stand_stop（无待交付）上跑绿、差点把这个 bug 留在树里。
            "（本场景待交付：%s）。" % (", ".join(str(item) for item in pending_steps) or "无"),
            "故障注入只交付「传感器不可用」与「能力未声明」两类；注入语义是：在故障点拒绝下发该步指令、"
            "其后只允许技能清单里 safetyClass=safety_action 的步骤继续执行。它证明「未继续自主机动、"
            "未伪造成功」，不构成真机/物理安全停机证据（真机与目标端 DEFERRED）。",
            "目标端/真机验收 DEFERRED（板卡不在场）：本报告全部结论为 simulation=true 的仿真证据。",
        ],
        "failed_checks": failed_checks,
    }
    if not fault_records:
        report["not_proved"].append("本场景未声明故障注入项：故障路径未被覆盖。")
    if unverified_faults:
        report["not_proved"].append(
            "故障 %s 未被注入（注入点未执行/未触发）：其期望终态未被验证。" % unverified_faults
        )
    robot_ids = sorted({item["robot"] for item in steps})
    for robot in robot_ids:
        if robot not in runtimes:
            report["not_proved"].append(
                "本体 %s 在本场景中只有待交付步骤：未装配后端、未执行任何动作。" % robot
            )
    if unregistered:
        report["failed_checks"].append(
            "存在未执行的步骤且没有显式登记：%s（禁止静默通过）" % unregistered
        )
    report["passed"] = not report["failed_checks"]

    target = Path(report_path) if report_path else default_report_path(scene_id, scenario_name)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["report_path"] = _rel(target)
    return report, (EXIT_OK if report["passed"] else EXIT_CRITERIA)


def list_scenes(root=None):
    """列出场景包与声明能力（只读声明，不装配后端、不需要生成模型）。"""
    root = Path(root or ROOT)
    scenes_dir = root / "scenes"
    if not scenes_dir.is_dir():
        raise ScenarioError("scenes/ 目录不存在: %s" % scenes_dir, EXIT_USAGE)
    items = []
    for package in sorted(path for path in scenes_dir.iterdir() if path.is_dir()):
        scene_path = package / "scene.yaml"
        if not scene_path.is_file():
            continue
        scene = yaml.safe_load(scene_path.read_text(encoding="utf-8")) or {}
        baseline_path = package / "baseline.yaml"
        baseline = {}
        if baseline_path.is_file():
            baseline = yaml.safe_load(baseline_path.read_text(encoding="utf-8")) or {}
        scenario_path = package / "scenario.yaml"
        scenario_document = {}
        if scenario_path.is_file():
            scenario_document = yaml.safe_load(scenario_path.read_text(encoding="utf-8")) or {}
        robots = []
        for robot in scene.get("robots") or []:
            if not isinstance(robot, dict):
                continue
            robot_id = str(robot.get("id"))
            backend = None
            reason = None
            try:
                backend = machine_declaration(robot_id, capability_index(scene), baseline)["backend"]
            except ScenarioError as exc:
                reason = str(exc)
            robots.append(
                {
                    "id": robot_id,
                    "kind": robot.get("kind"),
                    "profile": robot.get("profile"),
                    "capabilities": sorted(str(item) for item in (robot.get("capabilities") or [])),
                    "runner_backend": backend,
                    "runner_binding_reason": reason,
                }
            )
        scenarios = []
        for name, entry in sorted((scenario_document.get("scenarios") or {}).items()):
            if not isinstance(entry, dict):
                continue
            steps = [step for step in (entry.get("steps") or []) if isinstance(step, dict)]
            capabilities_by_robot = {item["id"]: set(item["capabilities"]) for item in robots}
            pending = [
                str(step.get("id"))
                for step in steps
                if str(step.get("action")) not in capabilities_by_robot.get(str(step.get("robot")), set())
            ]
            scenarios.append(
                {
                    "name": name,
                    "description": entry.get("description"),
                    "steps": len(steps),
                    "pending_steps": pending,
                    "faults": [str(item.get("id")) for item in (entry.get("faults") or []) if isinstance(item, dict)],
                }
            )
        items.append(
            {
                "scene": _rel(package, root),
                "id": scene.get("id"),
                "simulation": scene.get("simulation"),
                "robots": robots,
                "scenarios": scenarios,
            }
        )
    return {
        "schema_version": "iraf.scenario-list/v1",
        "simulation": True,
        "scenes_root": _rel(scenes_dir, root),
        "supported_fault_kinds": list(SUPPORTED_FAULT_KINDS),
        "evaluable_criteria": list(CRITERION_SPEC),
        "scenes": items,
    }


# --------------------------------------------------------------------------
# S1 命令式交互（interact 子命令）
# --------------------------------------------------------------------------
INTERACT_SCHEMA = "iraf.scenario-interactive/v1"
BUILTIN_READONLY = "state"          # 只读状态（非 Skill）：与场景执行器一致，仅用于取证


def parse_command(line, actions):
    """解析一行交互命令 → {"action","params"}；空行/注释返回 None；格式非法即显式失败。

    格式：`<动作> [键=值 …]`，值按 JSON 解析（`0.3` → 数、`true` → 布尔、`abc` → 字符串）。
    动作词表来自场景契约（config/scene.schema.json），本函数不内置任何动作名。
    """
    text = str(line).strip()
    if not text or text.startswith("#"):
        return None
    tokens = text.split()
    action = tokens[0]
    params = {}
    for token in tokens[1:]:
        if "=" not in token:
            raise ScenarioError(
                "参数必须是 key=value 形式：%r（动作 %s）" % (token, action), EXIT_USAGE
            )
        key, _, raw = token.partition("=")
        if not key:
            raise ScenarioError("参数名不能为空：%r" % (token,), EXIT_USAGE)
        try:
            params[key] = json.loads(raw)
        except json.JSONDecodeError:
            params[key] = raw
    return {"action": action, "params": params}


def _state_snapshot(backend):
    """只读状态快照（不驱动任何控制量）：适配器未实现 read_state 时显式返回 None。"""
    reader = getattr(backend, "read_state", None)
    if not callable(reader):
        return None, "该后端未实现 read_state（只读状态不可用）"
    try:
        return reader(), None
    except Exception as exc:  # noqa: BLE001
        return None, "read_state 失败：" + type(exc).__name__ + ": " + str(exc)


def interact(
    scene_dir,
    robot_id,
    *,
    commands=None,
    report_path=None,
    transcript_path=None,
    display="auto",
    render_hz=60.0,
    seconds=0.0,
    frames_dir=None,
    frame_count=0,
    root=None,
):
    """S1：逐条接受命令 → 只允许**已声明能力** → 走 SkillRuntime → 出 transcript 与报告。

    fail-closed 的三道门（都不靠"命令已发出"冒充成功）：
      1. 动作必须在场景契约词表里；
      2. 动作必须在 RobotProfile 的 `capabilities` 里（未声明即拒绝，给 IRAF-SKILL-PROVIDER-UNAVAILABLE）；
      3. 执行必须经 `SkillRuntime.execute`（PolicyGateway/租约/幂等/deadline 全在内），不直连后端。
    参数与判据一律来自声明：本函数不内置阈值、路径或动作名默认值。
    """
    root = Path(root or ROOT)
    scene_dir = _resolve(root, scene_dir)
    scene, baseline, _scenario_document, check_report = load_package(scene_dir)
    scene_id = str(check_report.get("scene") or scene_dir.name)
    index = capability_index(scene)
    if robot_id not in index:
        raise ScenarioError(
            "场景 %s 未声明本体 %s（已声明：%s）" % (scene_id, robot_id, sorted(index)),
            EXIT_REFERENCE,
        )
    contract = scenario_contract()
    declaration = machine_declaration(robot_id, index, baseline)
    states = assemble({"root": str(root), **declaration})
    profile = states["profile"]
    declared = {str(item) for item in (profile.capabilities or [])}

    transcript = Path(transcript_path) if transcript_path else (
        Path(root) / "build" / "acceptance" / "interactive" / ("transcript-%s.jsonl" % robot_id)
    )
    transcript.parent.mkdir(parents=True, exist_ok=True)
    records = []
    seq = 0

    def emit(record):
        records.append(record)
        with transcript.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(json.dumps({k: record[k] for k in ("seq", "command", "accepted", "status", "reason")
                          if k in record}, ensure_ascii=False), flush=True)

    for raw in commands if commands is not None else sys.stdin:
        seq += 1
        try:
            parsed = parse_command(raw, contract["actions"])
        except ScenarioError as exc:
            emit({"seq": seq, "command": str(raw).strip(), "accepted": False,
                  "status": "REJECTED", "error_code": "IRAF-INPUT-INVALID",
                  "reason": str(exc), "simulation": True})
            continue
        if parsed is None:
            seq -= 1
            continue
        action = parsed["action"]
        params = parsed["params"]
        command = ("%s %s" % (action, " ".join("%s=%s" % (k, v) for k, v in params.items()))).strip()

        if action == BUILTIN_READONLY:
            state, error = _state_snapshot(states["backend"])
            emit({"seq": seq, "command": command, "accepted": True,
                  "status": "READ_ONLY" if state is not None else "REJECTED",
                  "error_code": None if state is not None else "IRAF-INTERNAL",
                  "reason": error or "只读状态（不驱动控制量）", "state": state, "simulation": True})
            continue

        if action not in contract["actions"]:
            emit({"seq": seq, "command": command, "accepted": False, "status": "REJECTED",
                  "error_code": "IRAF-INPUT-INVALID",
                  "reason": "动作 %r 不在场景契约词表里（先改契再实现）" % action, "simulation": True})
            continue
        if action not in declared:
            emit({"seq": seq, "command": command, "accepted": False, "status": "REJECTED",
                  "error_code": "IRAF-SKILL-PROVIDER-UNAVAILABLE",
                  "reason": "本体 %s 的 Profile 未声明能力 %r（已声明：%s）：能力未验收前不得调用"
                            % (robot_id, action, sorted(declared)),
                  "simulation": True})
            continue

        correlation = "interact-%s-%d" % (robot_id, seq)
        request = _request(profile, states["safety"], action, params,
                           "%s-mujoco" % profile.name, correlation, key=correlation)
        started = time.monotonic()
        display_mode = None
        if display != "none":
            display_mode = "auto" if display == "auto" else display
            viewer = run_request_live(
                states["backend"], states["runtime"], request, _context(),
                camera=None, render_hz=render_hz, seconds=seconds,
                pre_roll_frames=0, frames_dir=frames_dir, frame_count=frame_count,
                continue_stepping=True, display_mode=None if display_mode == "auto" else display_mode,
            )
            result = viewer["holder"]["result"]
            # 可选键必须用 get：机械臂后端有 step()，此时不会产生 stepping_note
            # （此前用直接索引取值，导致机械臂窗口路径 KeyError 整条失败 —— 实测于 2026-09-21）
            display_report = {k: viewer.get(k) for k in (
                "display_mode", "window_opened", "frames", "frames_written", "error", "display_env")}
            if viewer.get("stepping_note"):
                display_report["stepping_note"] = viewer["stepping_note"]
            # 执行线程内的异常必须留痕（否则表现为 status=None 而看不出原因）
            if viewer["holder"].get("error"):
                display_report["execution_error"] = viewer["holder"]["error"]
        else:
            result = states["runtime"].execute(request, _context())
            display_report = None
        wall = time.monotonic() - started
        state, state_error = _state_snapshot(states["backend"])
        if result is None or (isinstance(result, dict) and not result.get("status")):
            # 执行返回空/无 status：按 fail-closed 记为 FAILED，并保留原始结果供定位
            emit({"seq": seq, "command": command, "accepted": True,
                  "status": "FAILED",
                  "error_code": (result or {}).get("error_code") or "IRAF-EXECUTION-FAILED",
                  "reason": (result or {}).get("reason") or "执行未返回结构化 status（原始结果见 raw_result）",
                  "raw_result": result, "wall_seconds": wall, "state": state,
                  "state_error": state_error, "display": display_report, "simulation": True})
            continue
        emit({"seq": seq, "command": command, "accepted": True,
              "status": (result or {}).get("status"),
              "error_code": (result or {}).get("error_code"),
              "reason": (result or {}).get("reason"),
              "wall_seconds": wall, "state": state, "state_error": state_error,
              "display": display_report, "simulation": True})

    report = {
        "schema_version": INTERACT_SCHEMA,
        "simulation": True,
        "scene": scene_id,
        "robot": robot_id,
        "declared_capabilities": sorted(declared),
        "contract_actions": sorted(contract["actions"]),
        "display": display,
        "transcript_path": str(transcript),
        "counts": {
            "commands": len(records),
            "accepted": sum(1 for r in records if r.get("accepted")),
            "rejected": sum(1 for r in records if not r.get("accepted")),
            "succeeded": sum(1 for r in records if r.get("status") == "SUCCEEDED"),
        },
        "commands": records,
    }
    target = Path(report_path) if report_path else (
        Path(root) / "build" / "acceptance" / "interactive" / ("report-%s.json" % robot_id)
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["report_path"] = str(target)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser("list", help="列出场景包与声明能力（只读声明）")
    run_parser = subparsers.add_parser("run", help="按声明执行一个场景并写报告")
    run_parser.add_argument("--scene", required=True, type=Path, help="场景包目录")
    run_parser.add_argument("--scenario", required=True, help="场景名（scenario.yaml 的键）")
    run_parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    run_parser.add_argument(
        "--world", choices=tuple(WORLD_BINDING_KEYS), default="single",
        help="single=各本体自己的单本体产物（默认，历史行为不变）；"
             "joint=多本体同一份 MJCF 的联合世界（baseline.robots_joint + 共享植物）")
    run_parser.add_argument(
        "--require-injected-faults",
        action="store_true",
        help="任何故障未被注入即失败（退出码 5）：占位不是通过",
    )
    run_parser.add_argument(
        "--display", choices=("auto", "none", "interactive_viewer", "offscreen_frames"),
        default="none",
        help="运行期显示：none=不开窗（默认，行为与改动前一致）；"
             "interactive_viewer=边跑边看（**只渲染、不推进**，时间由 owner 驻留线程推进）")
    run_parser.add_argument("--render-hz", type=float, default=20.0, help="窗口渲染频率")
    run_parser.add_argument(
        "--seconds", type=float, default=0.0,
        help="运行结束后窗口再保持的秒数（0=立即关闭；调试末态时给几秒）")
    ix_parser = subparsers.add_parser(
        "interact", help="S1 命令式交互：逐条命令 → 只允许已声明能力 → 走 SkillRuntime"
    )
    ix_parser.add_argument("--scene", required=True, type=Path, help="场景包目录")
    ix_parser.add_argument("--robot", required=True, help="场景中声明的本体 id（如 unitree_go2 / piper）")
    ix_parser.add_argument(
        "--commands-from",
        type=Path,
        default=None,
        help="命令清单文件（每行一条 `动作 [键=值 …]`）；缺省从 stdin 读取（支持管道与交互式输入）",
    )
    ix_parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    ix_parser.add_argument("--transcript", type=Path, default=None, help="覆盖 transcript(JSONL) 路径")
    ix_parser.add_argument(
        "--display",
        choices=("auto", "none", "interactive_viewer", "offscreen_frames"),
        default="auto",
        help="auto=探测后自动选择；none=不开窗也不导帧（最快，用于批量/CI）",
    )
    ix_parser.add_argument("--frames-dir", type=Path, default=None, help="离屏帧导出目录")
    ix_parser.add_argument("--frames", type=int, default=0, help="离屏导出帧数（0=不导出）")
    ix_parser.add_argument("--render-hz", type=float, default=60.0, help="窗口渲染频率")
    ix_parser.add_argument("--seconds", type=float, default=0.0, help="每条命令执行后保持窗口的秒数（0=保持到关闭窗口）")
    ix_parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按脚本位置推导）")
    args = parser.parse_args(argv)

    if args.command == "list":
        try:
            report = list_scenes()
        except ScenarioError as exc:
            print(str(exc), file=sys.stderr)
            return exc.code
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return EXIT_OK

    if args.command == "interact":
        commands = None
        if args.commands_from is not None:
            path = args.commands_from
            if not path.is_file():
                print("命令清单不存在：" + str(path), file=sys.stderr)
                return EXIT_USAGE
            commands = path.read_text(encoding="utf-8").splitlines()
        try:
            report = interact(
                args.scene,
                args.robot,
                commands=commands,
                report_path=args.report,
                transcript_path=args.transcript,
                display=args.display,
                render_hz=args.render_hz,
                seconds=args.seconds,
                frames_dir=args.frames_dir,
                frame_count=args.frames,
                root=args.root,
            )
        except ScenarioError as exc:
            print("· " + str(exc), file=sys.stderr)
            print("INTERACT_DECLARATION_ERROR", file=sys.stderr)
            return exc.code
        print(json.dumps({"passed": report["counts"]["rejected"] == 0,
                          "counts": report["counts"],
                          "report_path": report["report_path"],
                          "transcript_path": report["transcript_path"]},
                         ensure_ascii=False, indent=2))
        # 语义：命令全部被接受且无拒绝 → 0；存在被拒绝的命令 → 3（拒绝路径也属"预期行为"，
        # 由调用方按 counts 判断，不把"有拒绝"当作系统故障）
        return EXIT_OK if report["counts"]["rejected"] == 0 else EXIT_BACKEND

    if args.command != "run":
        parser.print_help(file=sys.stderr)
        return EXIT_USAGE

    try:
        report, exit_code = run_scenario(
            args.scene,
            args.scenario,
            report_path=args.report,
            require_injected_faults=args.require_injected_faults,
            world=args.world,
            display=args.display,
            render_hz=args.render_hz,
            seconds=args.seconds,
        )
    except ScenarioError as exc:
        print("· " + str(exc), file=sys.stderr)
        print("SCENARIO_%s_ERROR" % ("DECLARATION" if exc.code == EXIT_DECLARATION else "FAILED"), file=sys.stderr)
        return exc.code

    summary = {
        "passed": report["passed"],
        "scene": report["scene"],
        "scenario": report["scenario"],
        "report_path": report["report_path"],
        "counts": report["counts"],
        "steps": [
            {
                "id": item["id"],
                "action": item["action"],
                "kind": item["kind"],
                "status": item.get("status"),
                "error_code": item.get("error_code"),
                "wall_seconds": item.get("wall_seconds"),
                "measured": item.get("measured"),
                "passed": item.get("passed"),
            }
            for item in report["steps"]
        ],
        "faults": [
            {"id": item["id"], "injected": item["injected"], "verified": item["verified"]}
            for item in report["faults"]
        ],
        "failed_checks": report["failed_checks"],
        "not_proved": report["not_proved"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
