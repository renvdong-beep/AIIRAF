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
  PYTHONPATH=src python3 scripts/profile_check.py \
      --baseline config/ur5_simulation_baseline.yaml
"""

import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco  # noqa: E402

from iraf_adapters.factory import (  # noqa: E402
    BackendContractError,
    _resolve_class,
    verify_backend_contract,
)
from iraf_core.profile import load_robot_profile  # noqa: E402

BACKEND_ENTRYPOINT = "iraf_adapters.mujoco.mujoco_backend:MujocoBackend"


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def check(baseline, baseline_path, profile_path, backend_entrypoint=BACKEND_ENTRYPOINT):
    """返回 (report, failures)。failures 非空即整体失败。"""
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=None)
    args = parser.parse_args(argv)

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
