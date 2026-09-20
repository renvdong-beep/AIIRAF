"""统一 Profile/基线交叉校验：新机型接入的第一道门（AGENTS.md 6.4 金路径之一）。

校验的是"声明之间、声明与模型之间"的一致性，**不跑动作**，因此可以在没有
真机、没有场景产物的情况下先跑一遍：

1. 基线的臂关节必须都在 profile.joints 中；
2. 基线夹爪执行器名集合必须与 `gripper.open/closed` 的键完全一致
   （config 的 `gripper.joints` 是**执行器名**，profile 的
   `gripper.drive_joints` 是**被驱动关节名**，两者是不同层，不要求相等）；
3. profile 的 capabilities 必须都能在后端类上找到实现（复用 factory 的契约校验）；
4. profile.joint_roles 必须覆盖全部 profile.joints（角色不明会让 IK/夹爪逻辑猜）；
5. 基线声明的夹爪几何（`model.finger_geoms.*`、`pad_boxes`（若声明）与
   `model.bodies.*`）必须在模型里真实存在；
6. 基线声明的模型来源、场景输出与 build 段必须齐备。

任何一项不一致都以非零退出码结束（可进 CI），并打印可定位的中文原因。

用法：
  # 机型基线（既有行为，逐字节不变）
  PYTHONPATH=src python3 scripts/profile_check.py \
      --baseline config/ur5_simulation_baseline.yaml
  # 板卡声明（新增）：未实测字段一律 unverified -> 退出码 2
  PYTHONPATH=src python3 scripts/profile_check.py --board profiles/boards/e300.yaml
  PYTHONPATH=src python3 scripts/profile_check.py \
      --board profiles/boards/e300.yaml --allow-unverified

`--board` 的退出码约定（与 `--baseline` 的 0/1 相容，多出 2 表示\"声明未实测\"）：
  0  校验通过：`verified: true`，或经 `--allow-unverified` 显式放行且摘要写 `verified: false`
  1  契约层失败：schema 不合法、字段缺失、与产物矩阵声明冲突、文件不可读
  2  声明仍含 unverified/pending：预检拒绝；未实测不得当作可用（AGENTS.md 铁律 3）

`--board` 的输出契约：**stdout 只有一份纯 JSON 摘要**（字段路径以 `spec` 为根，见
report 的 `field_scope`），状态标记与逐条中文原因走 stderr，便于
`profile_check.py --board <yaml> --allow-unverified | jq .verified` 这类调用。
"""

import argparse
import copy
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.factory import (  # noqa: E402
    BackendContractError,
    _resolve_class,
    verify_backend_contract,
)
from iraf_core.profile import load_robot_profile  # noqa: E402

BACKEND_ENTRYPOINT = "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"

# --- BoardProfile 校验（--board）引用的契约路径与字面量 -------------------------
# 契约与产物矩阵都只在这里声明一次，禁止在别处复制（铁律 5.3）。
BOARD_SCHEMA_PATH = ROOT / "config" / "sdk" / "package_matrix.schema.json"
MATRIX_PATH = ROOT / "config" / "sdk" / "package_matrix.yaml"
BOARD_SCHEMA_KEY = "board_profile"
UNVERIFIED = "unverified"
PENDING = "pending"
#: --board 专用退出码：声明仍未实测（区别于契约层失败 1）
EXIT_UNVERIFIED = 2


class BoardCheckError(RuntimeError):
    """BoardProfile 校验的契约层错误（由 main 转成退出码 1）。"""


def _import_mujoco():
    """按需导入 mujoco。

    `--board` 是纯声明校验（板卡上多半没装仿真器），因此不再在模块导入期
    硬依赖 mujoco；`--baseline` 走几何校验时才导入，失败即显式报错。
    """
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit("缺少 mujoco 依赖（--baseline 的几何校验需要）: " + str(exc))
    return mujoco



def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def check(baseline, baseline_path, profile_path, backend_entrypoint=BACKEND_ENTRYPOINT):
    """返回 (report, failures)。failures 非空即整体失败。"""
    mujoco = _import_mujoco()
    failures = []
    notes = []

    profile = load_robot_profile(_resolve(ROOT, profile_path))
    model_cfg = baseline.get("model") or {}
    source_path = _resolve(ROOT, model_cfg["source"])
    if not source_path.is_file():
        failures.append("模型来源不存在: " + str(source_path))
        model = None
        model_path = source_path
    else:
        # 几何名是**场景契约**的一部分，由场景生成器产出（Piper 的指腹 geom
        # 在源厂商模型里并不叫 piper_left_finger，是生成器写进场景的）。
        # 因此优先校验生成后的场景；场景尚未生成时退回源模型并注明口径。
        declared_scene = (baseline.get("build") or {}).get("scene")
        scene_path = _resolve(ROOT, declared_scene) if declared_scene else None
        if scene_path is not None and scene_path.is_file():
            model_path = scene_path
            notes.append("几何校验口径：生成后的场景 %s" % scene_path.name)
        else:
            model_path = source_path
            notes.append(
                "几何校验口径：源模型 %s（场景尚未生成；宜先跑 scripts/build_baseline.py）"
                % source_path.name
            )
        model = mujoco.MjModel.from_xml_path(str(model_path))

    # 1) 臂关节 ⊆ profile.joints
    arm_joints = [str(name) for name in model_cfg.get("arm_joints") or []]
    if not arm_joints:
        failures.append("基线缺少 model.arm_joints（IK 需要知道哪些是臂关节）")
    missing = [name for name in arm_joints if name not in profile.joints]
    if missing:
        failures.append("baseline.model.arm_joints 未在 profile.joints 中声明: " + str(missing))

    # 2) 夹爪执行器名与 open/closed 键一致（执行器层，不是关节层）
    gripper_cfg = baseline.get("gripper") or {}
    actuators = [str(name) for name in gripper_cfg.get("joints") or []]
    if not actuators:
        failures.append("基线缺少 gripper.joints（后端按执行器名写 ctrl）")
    for key in ("open", "closed"):
        keys = sorted(str(k) for k in (gripper_cfg.get(key) or {}))
        if keys != sorted(actuators):
            failures.append(
                "gripper.%s 的键 %s 与 gripper.joints %s 不一致" % (key, keys, actuators)
            )

    # 3) 能力声明必须都有实现（复用装配期契约校验）
    contract = None
    try:
        backend_class = _resolve_class(backend_entrypoint)
        contract = verify_backend_contract(backend_class, profile)
    except BackendContractError as exc:
        failures.append("能力契约校验失败: " + str(exc))

    # 4) joint_roles 覆盖全部 joints
    roles = dict(profile.joint_roles or {})
    uncovered = [name for name in profile.joints if name not in roles]
    if uncovered:
        failures.append("profile.joint_roles 未覆盖关节: " + str(uncovered))
    arm_roles = [name for name, role in roles.items() if str(role) == "arm"]
    if arm_joints and sorted(arm_roles) != sorted(arm_joints):
        failures.append(
            "profile 中 arm 角色关节 %s 与 baseline.model.arm_joints %s 不一致"
            % (sorted(arm_roles), sorted(arm_joints))
        )

    # 5) 声明的几何必须在模型里真实存在
    if model is not None:
        for key, value in (model_cfg.get("finger_geoms") or {}).items():
            names = value if isinstance(value, (list, tuple)) else [value]
            for name in names:
                if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(name)) < 0:
                    failures.append("finger_geoms.%s 引用的 geom 不存在: %s" % (key, name))
        for key, name in (model_cfg.get("bodies") or {}).items():
            obj = mujoco.mjtObj.mjOBJ_SITE if key == "flange_site" else mujoco.mjtObj.mjOBJ_BODY
            if mujoco.mj_name2id(model, obj, str(name)) < 0:
                failures.append("bodies.%s 引用的对象不存在: %s" % (key, name))

    # 6) 场景物理参数必须显式声明且真的生效
    # 背景（真实事故）：2026-09-20 改光源时误删了 config 的 scene 段物理键，
    # 场景静默回落到模型默认值 —— 夹爪接触力从 17.45N 掉到 8.84N、抬升位移
    # 从 0.041208 变成 0.050928，而所有门禁仍然"通过"。因此这里把
    # "显式声明" 与 "声明已生效" 都变成硬门禁。
    scene_cfg = baseline.get("scene") or {}
    friction_key = next(
        (k for k in ("pad_friction", "finger_friction") if k in scene_cfg), None
    )
    for key, label in (
        ("timestep_s", "物理步长"),
        ("gravity", "重力"),
        (friction_key, "指腹摩擦"),
    ):
        if key is None or key not in scene_cfg:
            failures.append(
                "基线 scene 段缺少 %s（%s）：场景会静默回落到模型默认值，"
                "实测会让夹持力/抬升位移变化而不报错" % (key, label)
            )

    if model is not None:
        if "timestep_s" in scene_cfg:
            declared_step = float(scene_cfg["timestep_s"])
            if abs(float(model.opt.timestep) - declared_step) > 1e-12:
                failures.append(
                    "生成场景的步长 %.9f 与声明 %.9f 不一致（声明未生效）"
                    % (float(model.opt.timestep), declared_step)
                )
        if "gravity" in scene_cfg:
            declared_gravity = [float(v) for v in str(scene_cfg["gravity"]).split()]
            actual_gravity = [float(v) for v in model.opt.gravity]
            if len(declared_gravity) != 3 or any(
                abs(a - b) > 1e-9 for a, b in zip(declared_gravity, actual_gravity)
            ):
                failures.append(
                    "生成场景的重力 %s 与声明 %s 不一致（声明未生效）"
                    % (actual_gravity, declared_gravity)
                )
        if friction_key and friction_key in scene_cfg:
            declared_friction = [float(v) for v in str(scene_cfg[friction_key]).split()]
            for name in (
                (model_cfg.get("finger_geoms") or {}).get("left"),
                (model_cfg.get("finger_geoms") or {}).get("right"),
            ):
                if not name:
                    continue
                gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, str(name))
                if gid < 0:
                    continue
                actual = [float(v) for v in model.geom_friction[gid]]
                if any(abs(a - b) > 1e-9 for a, b in zip(declared_friction, actual)):
                    failures.append(
                        "生成场景中 %s 的摩擦 %s 与声明 %s 不一致（声明未生效）"
                        % (name, [round(v, 4) for v in actual], declared_friction)
                    )

    # 7) build 段齐备（统一入口依赖它分派）
    build = baseline.get("build") or {}
    for key, label in (
        ("baseline_module", "构建器模块"),
        ("scene", "场景输出路径"),
        ("pose_evidence", "姿态证据路径"),
        ("profile", "RobotProfile 路径"),
    ):
        if not build.get(key):
            failures.append("基线缺少 build.%s（%s），统一入口无法分派" % (key, label))

    report = {
        "schema_version": "iraf.profile-check/v1",
        "baseline": str(baseline_path),
        "profile": {"name": profile.name, "version": profile.version},
        "model": str(model_path),
        "arm_joints": arm_joints,
        "capabilities": sorted(profile.capabilities),
        "contract": contract,
        "failures": failures,
        "notes": notes,
        "passed": not failures,
    }
    return report


def _load_board_schema():
    """取出并自检 BoardProfile 子契约。

    `definitions.board_profile` 是自包含的（内部只用相对 `$ref`），必须整体
    提取后单独校验；再从根结构 `$ref` 指向它会解析到根 definitions。
    """
    if not BOARD_SCHEMA_PATH.is_file():
        raise BoardCheckError("BoardProfile 契约文件不存在: " + str(BOARD_SCHEMA_PATH))
    try:
        schema = json.loads(BOARD_SCHEMA_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise BoardCheckError(
            "BoardProfile 契约不是合法 JSON: %s (%s)" % (BOARD_SCHEMA_PATH, exc)
        )
    try:
        from jsonschema import Draft7Validator
    except ImportError as exc:  # pragma: no cover - 取决于运行环境
        raise BoardCheckError("缺少 jsonschema 依赖，无法校验板卡声明: " + str(exc))
    definition = (schema.get("definitions") or {}).get(BOARD_SCHEMA_KEY)
    if not isinstance(definition, dict):
        raise BoardCheckError("契约缺少 definitions.%s（板卡声明无处可依）" % BOARD_SCHEMA_KEY)
    board_schema = copy.deepcopy(definition)
    board_schema["$schema"] = schema["$schema"]
    # 先证明契约本身是合法 draft-07，避免"恒失败/恒通过"的假门禁。
    Draft7Validator.check_schema(board_schema)
    return Draft7Validator(board_schema)


def _walk_placeholders(node, prefix=""):
    """递归收集声明里的 unverified / pending 字段路径。

    递归而不是写死字段表：schema 新增字段时自动纳入，不会出现"新增的未知
    字段悄悄绕过预检"。返回 (unverified_paths, pending_paths)，保持文件顺序。
    """
    unverified, pending = [], []
    if isinstance(node, dict):
        for key, value in node.items():
            path = "%s.%s" % (prefix, key) if prefix else str(key)
            found_unverified, found_pending = _walk_placeholders(value, path)
            unverified += found_unverified
            pending += found_pending
    elif isinstance(node, list):
        for index, value in enumerate(node):
            found_unverified, found_pending = _walk_placeholders(
                value, "%s[%d]" % (prefix, index)
            )
            unverified += found_unverified
            pending += found_pending
    elif isinstance(node, str):
        if node == UNVERIFIED:
            unverified.append(prefix)
        elif node == PENDING:
            pending.append(prefix)
    return unverified, pending


def _matrix_targets_for(board_name):
    """产物矩阵里声明了本板卡的构建目标（声明之间的一致性交叉校验）。"""
    if not MATRIX_PATH.is_file():
        raise BoardCheckError(
            "产物矩阵不存在，板卡声明的架构/标签无从交叉校验: " + str(MATRIX_PATH)
        )
    matrix = yaml.safe_load(MATRIX_PATH.read_text(encoding="utf-8")) or {}
    return [
        target
        for target in (matrix.get("targets") or [])
        if board_name in ((target or {}).get("boards") or [])
    ]


def check_board(board_path, allow_unverified=False):
    """校验 BoardProfile 声明，返回 report（退出码在 report["exit_code"]）。

    判据：
    1. 声明必须符合 `config/sdk/package_matrix.schema.json` 的
       `definitions.board_profile`（缺字段/未知字段/模糊占位一律拒绝）；
    2. `spec` 内任何 `unverified` / `pending` 都进 `reasons` -> 退出码 2；
       空 `capabilities` 同样视为"未声明能力"，一并拒绝（不得默认放行）；
    3. `--allow-unverified` 只放行第 2 类，摘要写 `verified: false`，
       契约层失败（第 1 类、与矩阵冲突）不因它而通过。
    """
    board_path = Path(board_path)
    if not board_path.is_file():
        raise BoardCheckError("BoardProfile 不存在: " + str(board_path))
    try:
        board = yaml.safe_load(board_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise BoardCheckError("BoardProfile 不是合法 YAML: %s (%s)" % (board_path, exc))
    if not isinstance(board, dict):
        raise BoardCheckError("BoardProfile 顶层必须是对象: " + str(board_path))

    validator = _load_board_schema()
    failures = []
    notes = []
    for error in sorted(validator.iter_errors(board), key=lambda item: [str(p) for p in item.path]):
        location = ".".join(str(part) for part in error.path) or "(根)"
        failures.append("不符合 BoardProfile 契约 %s: %s" % (location, error.message))

    metadata = board.get("metadata") or {}
    spec = board.get("spec") or {}
    unverified_fields, pending_fields = _walk_placeholders(spec)

    capabilities = [str(item) for item in (spec.get("capabilities") or [])]
    if not capabilities:
        # 空数组 = 尚未声明任何能力；预检不得默认放行（schema 的同一句注释）。
        unverified_fields.append("capabilities")

    board_name = str(metadata.get("name") or "")
    matrix_targets = []
    if board_name:
        hits = _matrix_targets_for(board_name)
        if not hits:
            notes.append(
                "产物矩阵未声明板卡 %s；矩阵声明板卡时须同步（本步不据此失败）" % board_name
            )
        if len(hits) > 1:
            failures.append(
                "板卡 %s 被 %d 个产物矩阵目标声明，架构/标签会出现两个来源"
                % (board_name, len(hits))
            )
        declared_arch = (spec.get("target") or {}).get("arch")
        for target in hits:
            matrix_targets.append(
                {
                    "id": target.get("id"),
                    "arch": target.get("arch"),
                    "platform_tag": target.get("platform_tag"),
                    "python_tag": target.get("python_tag"),
                }
            )
            if declared_arch != target.get("arch"):
                failures.append(
                    "target.arch 声明 %s 与产物矩阵目标 %s 的架构 %s 不一致"
                    % (declared_arch, target.get("id"), target.get("arch"))
                )

    reasons = []
    for path in unverified_fields:
        if path == "capabilities":
            reasons.append("capabilities 未声明任何能力（空数组）：预检不得默认放行")
        else:
            reasons.append("%s 未验证（unverified）：请在板卡实测后回填" % path)
    for path in pending_fields:
        reasons.append("%s 未确定（pending）：需在验收签字后回填" % path)

    verified = (
        spec.get("status") == "verified" and not unverified_fields and not pending_fields
    )
    passed = not failures and (verified or allow_unverified)
    if passed:
        exit_code = 0
    elif failures:
        exit_code = 1
    else:
        exit_code = EXIT_UNVERIFIED

    report = {
        "schema_version": "iraf.board-profile-check/v1",
        "board": board_name or None,
        "board_path": str(board_path),
        "profile_version": metadata.get("version"),
        "hyper_models": list(metadata.get("hyper_models") or []),
        "status": spec.get("status"),
        "verified": verified,
        "allowed_unverified": bool(allow_unverified and not verified),
        # 字段路径以 spec 为根（report 自身已限定 scope），与步骤文件示例
        # `target.python_tag 未验证` 一致；避免同一字段出现两种写法。
        "field_scope": "spec",
        "target": dict(spec.get("target") or {}),
        "capabilities": capabilities,
        "matrix_targets": matrix_targets,
        "unverified_fields": unverified_fields,
        "pending_fields": pending_fields,
        "reasons": reasons,
        "failures": failures,
        "notes": notes,
        "passed": passed,
        "exit_code": exit_code,
    }
    return report


def _main_board(args):
    try:
        report = check_board(_resolve(ROOT, args.board), allow_unverified=args.allow_unverified)
    except BoardCheckError as exc:
        print("· " + str(exc), file=sys.stderr)
        print("\nBOARD_PROFILE_CHECK_FAILED", file=sys.stderr)
        return 1

    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["exit_code"] == 0:
        # stdout 保持**纯 JSON 摘要**（打包/预检脚本要直接解析它），
        # 状态标记一律走 stderr，避免 `json.loads(stdout)` 被尾行污染。
        if report["verified"]:
            print("BOARD_PROFILE_CHECK_PASSED", file=sys.stderr)
        else:
            print(
                "BOARD_PROFILE_CHECK_PASSED_ALLOW_UNVERIFIED"
                "（--allow-unverified 显式放行，摘要 verified: false）",
                file=sys.stderr,
            )
        return 0

    for message in report["failures"]:
        print("· " + message, file=sys.stderr)
    if report["exit_code"] == EXIT_UNVERIFIED:
        for message in report["reasons"]:
            print("· " + message, file=sys.stderr)
        print("\nBOARD_PROFILE_CHECK_REJECTED_UNVERIFIED", file=sys.stderr)
    else:
        print("\nBOARD_PROFILE_CHECK_FAILED", file=sys.stderr)
    return report["exit_code"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=None)
    parser.add_argument("--profile", type=Path, default=None)
    parser.add_argument(
        "--board",
        type=Path,
        default=None,
        help="校验板卡声明（BoardProfile）；含 unverified/pending 即退出码 2",
    )
    parser.add_argument(
        "--allow-unverified",
        action="store_true",
        help="显式放行仍含 unverified/pending 的板卡声明（摘要 verified: false）",
    )
    args = parser.parse_args(argv)

    if args.allow_unverified and args.board is None:
        print("--allow-unverified 只对 --board 生效；--baseline 的门禁不可放宽", file=sys.stderr)
        return 1
    if args.board is None and args.baseline is None:
        print("请给出 --baseline（机型基线）或 --board（板卡声明）", file=sys.stderr)
        return 1
    if args.board is not None:
        return _main_board(args)

    baseline_path = _resolve(ROOT, args.baseline)
    if not baseline_path.is_file():
        raise SystemExit("基线配置不存在: " + str(baseline_path))
    baseline = yaml.safe_load(baseline_path.read_text(encoding="utf-8")) or {}
    profile_path = args.profile or (baseline.get("build") or {}).get("profile")
    if not profile_path:
        raise SystemExit("请显式给出 --profile，或让基线声明 build.profile")

    report = check(baseline, baseline_path, profile_path)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["passed"]:
        print("\nPROFILE_CHECK_FAILED", file=sys.stderr)
        return 1
    print("\nPROFILE_CHECK_PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
