"""场景包校验入口：把 `scenes/<id>/` 从"一堆 YAML"变成可门禁的声明（AGENTS.md 6.4 金路径之一）。

校验三层，缺一层都可能让声明与实际脱节：

1. **契约层**：`scene.yaml` / `baseline.yaml` / `scenario.yaml` 必须符合
   `config/scene.schema.json`（根结构 + `definitions.scene_baseline` +
   `definitions.scenario_catalog`）。缺字段、未知字段、模糊占位一律失败（退出码 2）。
2. **引用层**：声明之间、声明与仓库文件之间必须自洽——本体 profile 文件必须真实存在、
   道具 mesh 必须真实存在、`baseline.robots` 的键集合必须与 `scene.robots[].id` **完全一致**、
   场景步骤引用的本体/传感器/道具必须存在、同一实体 id 不得重复（退出码 1）。
   尚未交付的引用只能写成 `{state: unverified, closed_by, reason}`，会被收进 `pending_refs`；
   **这不是通过，而是显式登记的缺口**（`--require-resolved-refs` 可把它变成硬失败）。
3. **模型层**：给了生成后的模型（`--model`，或 `scene.model.output` 已存在）时，
   相机/雷达/IMU 的 `anchor.name`、道具的 `body` 必须在生成模型里真实存在（退出码 3）。
   厂商 MJCF 不提供相机与雷达，因此 `source: scene` 的传感器只有生成后才会出现——
   模型不存在时登记为 `pending_generation`，不假装校验过。

用法：
  PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab
  PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab \
      --model build/scenes/handoff_lab/handoff_lab.xml --require-model
  PYTHONPATH=src python3 scripts/scene_check.py --scene scenes/handoff_lab --require-resolved-refs

退出码：
  0  通过（摘要里可能仍有显式登记的 pending 引用）
  1  引用完整性失败（引用的文件/本体/道具/传感器不存在，或跨文件声明不一致）
  2  契约层失败（缺字段、字段非法、场景包缺少必需文件）
  3  模型层失败（生成模型里传感器锚点或道具 body 不存在）
  4  `--require-model` 但模型未声明或不存在
  5  `--require-resolved-refs` 但仍有待交付引用/步骤
  6  用法错误（--scene 不是目录、YAML 不可解析等）

stdout 只有一份纯 JSON 摘要（脚本要直接解析它）；中文原因与状态标记走 stderr。
场景是仿真声明：报告中始终带 `simulation: true`，不得据此表述真机能力（AGENTS.md 1.7）。
"""

import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from jsonschema import Draft7Validator

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "config" / "scene.schema.json"

# 场景包必需文件（键 -> 文件名）；README.md 是使用者入口（铁律 6.2：公共能力要有中文说明）。
PACKAGE_FILES = {
    "scene": "scene.yaml",
    "baseline": "baseline.yaml",
    "scenario": "scenario.yaml",
    "readme": "README.md",
}
CONTRACT_KEYS = {"scene": None, "baseline": "scene_baseline", "scenario": "scenario_catalog"}
CONTRACT_IDS = {
    "scene": "iraf.scene/v1",
    "baseline": "iraf.scene-baseline/v1",
    "scenario": "iraf.scenario-catalog/v1",
}
UNVERIFIED = "unverified"
STATIC_MODEL_ONLY = "static_model_only"
#: 只允许静态实体的本体（决策 4.B）：不得出现在任何 Skill 步骤里。
MOTION_ACTIONS = (
    "stand",
    "stop",
    "locomote",
    "navigate",
    "dock_for_handoff",
    "accept_payload",
    "pick_object",
    "place_object",
    "move_joint",
    "visual_pick",
)

EXIT_OK = 0
EXIT_REFERENCE = 1
EXIT_SCHEMA = 2
EXIT_MODEL = 3
EXIT_MODEL_REQUIRED = 4
EXIT_PENDING = 5
EXIT_USAGE = 6


class SceneCheckError(RuntimeError):
    """用法/输入层错误（由 main 转成退出码 6）。"""


def _rel(path):
    """仓库内相对路径用于报告；仓库外（临时夹具）返回绝对路径。"""
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def _load_yaml(path, label):
    if not path.is_file():
        raise SceneCheckError("场景包缺少 %s（%s）" % (label, _rel(path)))
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SceneCheckError("%s 不是合法 YAML: %s" % (label, exc))
    if not isinstance(document, dict):
        raise SceneCheckError("%s 顶层必须是对象" % label)
    return document


def _validator(key):
    """取出并自检子契约。

    三份子契约同文件交付：根结构直接校验 scene.yaml；另两份是**自包含**的（内部只用
    相对 `$ref`，辅助定义嵌套在自己的 definitions 下），因此必须整体提取后单独校验。
    """
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    if key is None:
        sub = copy.deepcopy(schema)
    else:
        definition = (schema.get("definitions") or {}).get(key)
        if not isinstance(definition, dict):
            raise SceneCheckError("契约缺少 definitions.%s（声明无处可依）" % key)
        sub = copy.deepcopy(definition)
    sub["$schema"] = schema["$schema"]
    # 先证明契约本身合法，避免"恒失败/恒通过"的假门禁。
    Draft7Validator.check_schema(sub)
    return Draft7Validator(sub)


def _schema_failures(validator, document, label):
    failures = []
    for error in sorted(validator.iter_errors(document), key=lambda item: [str(p) for p in item.path]):
        location = ".".join(str(part) for part in error.path) or "(根)"
        failures.append("%s 不符合场景契约 %s: %s" % (label, location, error.message))
    return failures


def _is_pending(value):
    return isinstance(value, dict) and value.get("state") == UNVERIFIED


def _pending_entry(label, value):
    return {
        "ref": label,
        "state": UNVERIFIED,
        "closed_by": value.get("closed_by"),
        "reason": value.get("reason"),
    }


def _resolve_inside(root, value, label, failures):
    """把包内相对引用解析到包内；越界（绝对路径或 ..）即失败。"""
    text = str(value)
    candidate = Path(text)
    if candidate.is_absolute() or ".." in candidate.parts:
        failures.append("%s 必须写包内相对路径，不能是绝对路径或含 ..: %s" % (label, text))
        return None
    return root / candidate


def _expand_mjcf(path, seen=None):
    """展开 MJCF 的 `<include file="...">`，返回按文档顺序平铺的元素列表。

    只用标准库：这是"名字是否存在"的检查，不是物理校验（物理证据来自构建/验收步骤）。
    `--model` 指向生成后的场景时，场景会 include 厂商模型，因此必须展开。
    """
    seen = seen or set()
    resolved = path.resolve()
    if resolved in seen:
        raise SceneCheckError("MJCF include 存在环: %s" % _rel(resolved))
    seen = seen | {resolved}
    root = ET.parse(str(resolved)).getroot()
    elements = []
    for element in list(root):
        if element.tag == "include":
            include_file = element.get("file")
            if not include_file:
                raise SceneCheckError("MJCF include 缺少 file 属性: %s" % _rel(resolved))
            elements.extend(_expand_mjcf(resolved.parent / include_file, seen))
        else:
            elements.append(element)
    return elements


def _model_names(model_path):
    """按对象类型收集生成模型里的名字（展开 include）。"""
    names = {"camera": set(), "site": set(), "geom": set(), "body": set()}
    for element in _expand_mjcf(model_path):
        if element.tag in names and element.get("name"):
            names[element.tag].add(element.get("name"))
    return names


def check(scene_dir, model_path=None, require_model=False, require_resolved_refs=False):
    """返回 (report, exit_code)。failures 按层分装，便于定位。"""
    scene_dir = Path(scene_dir)
    if not scene_dir.is_dir():
        raise SceneCheckError("场景目录不存在: " + str(scene_dir))

    package_failures = []
    paths = {}
    for key, name in PACKAGE_FILES.items():
        paths[key] = scene_dir / name
        if not paths[key].is_file():
            package_failures.append("场景包缺少必需文件 %s" % name)

    scene = _load_yaml(paths["scene"], "scene.yaml")
    baseline = _load_yaml(paths["baseline"], "baseline.yaml")
    scenario = _load_yaml(paths["scenario"], "scenario.yaml")

    schema_failures = list(package_failures)
    for key in ("scene", "baseline", "scenario"):
        label = PACKAGE_FILES[key]
        validator = _validator(CONTRACT_KEYS[key])
        declared = scene if key == "scene" else (baseline if key == "baseline" else scenario)
        schema_failures += _schema_failures(validator, declared, label)
    for key, expected in CONTRACT_IDS.items():
        declared = scene if key == "scene" else (baseline if key == "baseline" else scenario)
        if declared.get("schema_version") != expected:
            schema_failures.append(
                "%s 的 schema_version 是 %r，期望 %r（契约与文档必须同版本）"
                % (PACKAGE_FILES[key], declared.get("schema_version"), expected)
            )

    reference_failures = []
    pending_refs = []
    pending_steps = []
    notes = []

    # 场景 id 必须与目录名一致（报告与目录不会各说各话）。
    if str(scene.get("id")) != scene_dir.name:
        reference_failures.append(
            "scene.id %r 与目录名 %r 不一致" % (scene.get("id"), scene_dir.name)
        )

    # baseline / scenario 必须指向**本包自己的** scene.yaml。
    for label, document in (("baseline", baseline), ("scenario", scenario)):
        resolved = _resolve_inside(scene_dir, document.get("scene"), "%s.scene" % label, reference_failures)
        if resolved is not None and resolved.resolve() != paths["scene"].resolve():
            reference_failures.append(
                "%s.scene 解析为 %s，不是本场景包的 scene.yaml（引用完整性）"
                % (label, _rel(resolved))
            )

    robots = [item for item in (scene.get("robots") or []) if isinstance(item, dict)]
    props = [item for item in (scene.get("props") or []) if isinstance(item, dict)]
    sensors = [item for item in (scene.get("sensors") or []) if isinstance(item, dict)]
    robot_ids = [str(item.get("id")) for item in robots]
    prop_ids = [str(item.get("id")) for item in props]
    sensor_ids = [str(item.get("id")) for item in sensors]

    # 实体 id 唯一（不是"看起来像唯一"：重复 id 会让判据命中错对象）。
    for label, ids in (("robots", robot_ids), ("props", prop_ids), ("sensors", sensor_ids)):
        duplicates = sorted({name for name in ids if ids.count(name) > 1})
        if duplicates:
            reference_failures.append("%s 存在重复 id: %s" % (label, duplicates))

    model_only_ids = set()
    for robot in robots:
        label = "robots.%s.profile" % robot.get("id")
        profile = robot.get("profile")
        if _is_pending(profile):
            pending_refs.append(_pending_entry(label, profile))
        elif isinstance(profile, str):
            profile_path = _resolve_inside(ROOT, profile, label, reference_failures)
            if profile_path is not None and not profile_path.is_file():
                reference_failures.append("%s 引用的 profile 不存在: %s" % (label, profile))
            elif profile_path is not None:
                profile_document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
                if profile_document.get("kind") != "RobotProfile":
                    reference_failures.append("%s 不是 RobotProfile（kind=%r）" % (label, profile_document.get("kind")))
                profile_capabilities = set(
                    str(item) for item in ((profile_document.get("spec") or {}).get("capabilities") or [])
                )
                declared_capabilities = set(str(item) for item in (robot.get("capabilities") or []))
                missing = sorted(declared_capabilities - profile_capabilities)
                if missing:
                    reference_failures.append(
                        "%s 声明了 profile 未具备的能力 %s（声明不得超出 profile，铁律 1.3）"
                        % (robot.get("id"), missing)
                    )
        if robot.get("model_only") is True:
            model_only_ids.add(str(robot.get("id")))
            if STATIC_MODEL_ONLY not in (robot.get("capabilities") or []):
                reference_failures.append(
                    "本体 %s 声明 model_only 但 capabilities 未写 %s" % (robot.get("id"), STATIC_MODEL_ONLY)
                )

    entity_ids = set(robot_ids) | set(prop_ids)

    for prop in props:
        label = "props.%s.geometry" % prop.get("id")
        geometry = prop.get("geometry") or {}
        if geometry.get("type") == "mesh":
            mesh_path = _resolve_inside(ROOT, geometry.get("mesh"), label, reference_failures)
            if mesh_path is not None and not mesh_path.is_file():
                reference_failures.append("%s 引用的 mesh 不存在: %s" % (label, geometry.get("mesh")))
        pose = prop.get("pose") or {}
        mount = pose.get("mount") or {}
        if mount and str(mount.get("entity")) not in entity_ids:
            reference_failures.append(
                "props.%s.pose.mount.entity 引用的实体不存在: %s" % (prop.get("id"), mount.get("entity"))
            )

    for sensor in sensors:
        anchor = sensor.get("anchor") or {}
        if anchor and str(anchor.get("entity")) not in entity_ids:
            reference_failures.append(
                "sensors.%s.anchor.entity 引用的实体不存在: %s" % (sensor.get("id"), anchor.get("entity"))
            )

    light_names = [str(item.get("name")) for item in (scene.get("lights") or []) if isinstance(item, dict)]
    duplicates = sorted({name for name in light_names if light_names.count(name) > 1})
    if duplicates:
        reference_failures.append("lights 存在重复 name: %s" % duplicates)

    # 场景构建器引用：占位必须被报出来（否则"声明了还没交付"会变成静默缺口），
    # 已交付的路径必须真实存在。
    builder = (scene.get("model") or {}).get("builder")
    if _is_pending(builder):
        pending_refs.append(_pending_entry("model.builder", builder))
    elif isinstance(builder, str):
        builder_path = _resolve_inside(ROOT, builder, "model.builder", reference_failures)
        if builder_path is not None and not builder_path.is_file():
            reference_failures.append("model.builder 引用的场景构建器不存在: %s" % builder)

    # baseline 与 scene 的键集合必须完全一致：两处声明不一致必然导致"少写一个本体"的静默缺口。
    baseline_robots = baseline.get("robots") or {}
    baseline_state = baseline.get("initial_state") or {}
    for label, mapping in (("baseline.robots", baseline_robots), ("baseline.initial_state", baseline_state)):
        declared_ids = sorted(str(key) for key in mapping)
        if declared_ids != sorted(robot_ids):
            reference_failures.append(
                "%s 的键 %s 与 scene.robots[].id %s 不一致" % (label, declared_ids, sorted(robot_ids))
            )
    for robot in robots:
        robot_id = str(robot.get("id"))
        scene_pending = _is_pending(robot.get("profile"))
        baseline_value = baseline_robots.get(robot_id)
        if _is_pending(baseline_value):
            pending_refs.append(_pending_entry("baseline.robots.%s" % robot_id, baseline_value))
        elif isinstance(baseline_value, str):
            baseline_path = _resolve_inside(ROOT, baseline_value, "baseline.robots.%s" % robot_id, reference_failures)
            if baseline_path is not None and not baseline_path.is_file():
                reference_failures.append(
                    "baseline.robots.%s 引用的机型基线不存在: %s" % (robot_id, baseline_value)
                )
        # 两侧占位状态必须一致：只有一处写 pending，另一处就得是已交付的路径。
        if scene_pending != _is_pending(baseline_value):
            reference_failures.append(
                "本体 %s 的 profile 与 baseline.robots 两处声明不一致（一处待交付、一处已交付）" % robot_id
            )
        state = baseline_state.get(robot_id) or {}
        if state.get("pose_source") == UNVERIFIED:
            pending_refs.append(
                _pending_entry(
                    "baseline.initial_state.%s" % robot_id,
                    {"closed_by": state.get("closed_by"), "reason": state.get("reason")},
                )
            )

    # 场景步骤：引用完整性 + "不得使用未具备的能力而不登记"。
    capabilities_by_robot = {str(robot.get("id")): set(str(c) for c in (robot.get("capabilities") or [])) for robot in robots}
    for name, entry in sorted((scenario.get("scenarios") or {}).items()):
        if not isinstance(entry, dict):
            continue
        steps = [step for step in (entry.get("steps") or []) if isinstance(step, dict)]
        step_ids = [str(step.get("id")) for step in steps]
        duplicates = sorted({item for item in step_ids if step_ids.count(item) > 1})
        if duplicates:
            reference_failures.append("scenarios.%s 存在重复步骤 id: %s" % (name, duplicates))
        for step in steps:
            robot_id = str(step.get("robot"))
            action = str(step.get("action"))
            if robot_id not in capabilities_by_robot:
                reference_failures.append(
                    "scenarios.%s.%s.robot 引用的本体不存在: %s" % (name, step.get("id"), robot_id)
                )
                continue
            if robot_id in model_only_ids and action in MOTION_ACTIONS:
                reference_failures.append(
                    "scenarios.%s.%s 让静态模型本体 %s 执行 %s：决策 4.B 禁止给仅模型实体注册运动/操作能力"
                    % (name, step.get("id"), robot_id, action)
                )
                continue
            if action in capabilities_by_robot[robot_id]:
                continue
            if _is_pending((next(r for r in robots if str(r.get("id")) == robot_id)).get("profile")):
                pending_steps.append(
                    {
                        "scenario": name,
                        "step": step.get("id"),
                        "robot": robot_id,
                        "action": action,
                        "state": UNVERIFIED,
                        "closed_by": "本体 profile 待交付（见 pending_refs）",
                        "reason": "本体 %s 的 profile 未交付，能力未验收" % robot_id,
                    }
                )
            elif step.get("pending_closed_by"):
                pending_steps.append(
                    {
                        "scenario": name,
                        "step": step.get("id"),
                        "robot": robot_id,
                        "action": action,
                        "state": UNVERIFIED,
                        "closed_by": step.get("pending_closed_by"),
                        "reason": step.get("pending_reason"),
                    }
                )
            else:
                reference_failures.append(
                    "scenarios.%s.%s 使用本体 %s 未声明具备的能力 %s，且未登记待交付（缺 pending_closed_by/reason）"
                    % (name, step.get("id"), robot_id, action)
                )
        for fault in entry.get("faults") or []:
            if not isinstance(fault, dict):
                continue
            if str(fault.get("at_step")) not in step_ids:
                reference_failures.append(
                    "scenarios.%s.faults.%s.at_step 不是本场景的步骤: %s"
                    % (name, fault.get("id"), fault.get("at_step"))
                )
            target = str(fault.get("target"))
            if target not in set(entity_ids) | set(sensor_ids):
                reference_failures.append(
                    "scenarios.%s.faults.%s.target 引用的对象不存在（本体/道具/传感器）: %s"
                    % (name, fault.get("id"), target)
                )

    # --- 模型层 ---------------------------------------------------------------
    model_failures = []
    declared_output = (scene.get("model") or {}).get("output")
    resolved_output = ROOT / str(declared_output) if declared_output else None
    candidate = Path(model_path) if model_path else resolved_output
    model_report = {
        "status": "pending_generation",
        "model": str(candidate) if candidate else None,
        "declared_output": declared_output,
        "anchors": [],
        "reason": None,
    }
    if candidate is not None and Path(candidate).is_file():
        names = _model_names(Path(candidate))
        model_report["status"] = "checked"
        model_report["names"] = {key: len(value) for key, value in sorted(names.items())}
        for sensor in sensors:
            anchor = sensor.get("anchor") or {}
            object_kind = str(anchor.get("object_kind"))
            anchor_name = str(anchor.get("name"))
            present = anchor_name in names.get(object_kind, set())
            model_report["anchors"].append(
                {
                    "sensor": sensor.get("id"),
                    "object_kind": object_kind,
                    "name": anchor_name,
                    "present": present,
                }
            )
            if not present:
                model_failures.append(
                    "生成模型里找不到传感器 %s 的锚点 %s(%s)：声明未生效（厂商 MJCF 无相机/雷达，必须由场景构建器注入）"
                    % (sensor.get("id"), object_kind, anchor_name)
                )
        for prop in props:
            body = str(prop.get("body"))
            if body not in names["body"]:
                model_failures.append(
                    "生成模型里找不到道具 %s 的 body %s：抓取判据会命中不到目标" % (prop.get("id"), body)
                )
    else:
        model_report["reason"] = (
            "生成模型不存在，模型层校验登记为 pending_generation（不是通过）："
            "声明 output=%s，由步骤 13 的场景构建器产出" % declared_output
        )
        notes.append(model_report["reason"])

    pending_failures = []
    if require_model and model_report["status"] != "checked":
        pending_failures.append(
            "--require-model：生成模型不存在或未声明，模型层校验无法执行（声明 output=%s）" % declared_output
        )
    if require_resolved_refs and (pending_refs or pending_steps):
        for item in pending_refs:
            pending_failures.append(
                "--require-resolved-refs：引用 %s 仍待交付（closed_by=%s）" % (item["ref"], item["closed_by"])
            )
        for item in pending_steps:
            pending_failures.append(
                "--require-resolved-refs：步骤 %s/%s 的能力仍待交付（closed_by=%s）"
                % (item["scenario"], item["step"], item["closed_by"])
            )

    if schema_failures:
        exit_code = EXIT_SCHEMA
    elif reference_failures:
        exit_code = EXIT_REFERENCE
    elif model_failures:
        exit_code = EXIT_MODEL
    elif require_model and model_report["status"] != "checked":
        exit_code = EXIT_MODEL_REQUIRED
    elif pending_failures:
        exit_code = EXIT_PENDING
    else:
        exit_code = EXIT_OK

    report = {
        "schema_version": "iraf.scene-check/v1",
        "scene": _rel(scene_dir),
        "id": scene.get("id"),
        "contracts": {key: value for key, value in CONTRACT_IDS.items()},
        "simulation": scene.get("simulation"),
        "robots": robot_ids,
        "props": prop_ids,
        "sensors": sensor_ids,
        "lights": light_names,
        "model_check": model_report,
        "pending_refs": pending_refs,
        "pending_steps": pending_steps,
        "schema_failures": schema_failures,
        "reference_failures": reference_failures,
        "model_failures": model_failures,
        "gate_failures": pending_failures,
        "notes": notes,
        "passed": exit_code == 0,
        "exit_code": exit_code,
    }
    return report, exit_code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path, help="场景包目录，例如 scenes/handoff_lab")
    parser.add_argument("--model", type=Path, default=None, help="生成后的 MJCF（缺省用 scene.model.output）")
    parser.add_argument("--require-model", action="store_true", help="模型不存在即失败（退出码 4）")
    parser.add_argument(
        "--require-resolved-refs",
        action="store_true",
        help="任何待交付引用/步骤仍存在即失败（退出码 5）",
    )
    parser.add_argument("--json-out", type=Path, default=None, help="把 JSON 摘要另存到该路径")
    args = parser.parse_args(argv)

    try:
        report, exit_code = check(
            args.scene,
            model_path=args.model,
            require_model=args.require_model,
            require_resolved_refs=args.require_resolved_refs,
        )
    except SceneCheckError as exc:
        print("· " + str(exc), file=sys.stderr)
        print("\nSCENE_CHECK_USAGE_ERROR", file=sys.stderr)
        return EXIT_USAGE

    payload = json.dumps(report, ensure_ascii=False, indent=2)
    print(payload)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload + "\n", encoding="utf-8")

    if exit_code == EXIT_OK:
        print("SCENE_CHECK_PASSED", file=sys.stderr)
        return 0
    for key, label in (
        ("schema_failures", "契约层"),
        ("reference_failures", "引用层"),
        ("model_failures", "模型层"),
        ("gate_failures", "门禁"),
    ):
        for message in report[key]:
            print("· [%s] %s" % (label, message), file=sys.stderr)
    markers = {
        EXIT_SCHEMA: "SCENE_CHECK_FAILED_SCHEMA",
        EXIT_REFERENCE: "SCENE_CHECK_FAILED_REFERENCE",
        EXIT_MODEL: "SCENE_CHECK_FAILED_MODEL",
        EXIT_MODEL_REQUIRED: "SCENE_CHECK_FAILED_MODEL_REQUIRED",
        EXIT_PENDING: "SCENE_CHECK_FAILED_PENDING_REFS",
    }
    print("\n" + markers.get(exit_code, "SCENE_CHECK_FAILED"), file=sys.stderr)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
