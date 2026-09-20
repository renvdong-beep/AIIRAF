"""统一抓取验收入口：经完整 Skill Runtime 执行 pick_object / visual_pick。

判据与机型无关（AGENTS.md 6.4 金路径），且**判据全部取自 Backend 回传的证据**：
命中目标 + 双侧接触（法向力越阈）+ 抬升位移 + 位置误差。
脚本不自行判定"抓到了"，也不在失败时输出成功报告。

链路：TaskFlow → SkillRuntime → Policy/Authority → Provider → MuJoCo Backend，
不绕过任何一层；profile / safety / 场景 / 时长 / 目标 id 全部来自声明（CLI 可覆盖）。

三种用法：
  # 单目标
  python3 scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml
  # 视觉链路（位姿来自配置声明的视觉证据，判据相同）
  python3 scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml --skill visual_pick
  # 多目标：逐个目标重建场景并各自执行，报告里每个目标一份执行证据
  python3 scripts/verify_pick.py --baseline config/piper_multi_target.yaml --all-targets --skill visual_pick
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from iraf_adapters.factory import load_backend
from iraf_core.authority import ControlAuthorityManager
from iraf_core.policy import AuthenticatedContext
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore

DEFAULT_SAFETY_POLICY = "profiles/safety/simulation_lab.yaml"
DEFAULT_DURATION_MS = 12000


def _declared(config, key, label):
    value = (config.get("build") or {}).get(key)
    if not value:
        raise SystemExit(
            "配置缺少 build.%s（%s），请在基线配置中声明" % (key, label)
        )
    return value


def _resolve(root, value):
    path = Path(str(value))
    return path if path.is_absolute() else Path(root) / path


def _rebuild_scene(root, baseline_arg, config, scene_path, target_id=None):
    """按配置声明的构建器重建场景；多目标时按 target_id 重建。"""
    module_name = _declared(config, "baseline_module", "构建器模块名")
    from build_baseline import load_builder  # 同目录脚本

    builder = load_builder(root, module_name)
    builder.build(
        root,
        baseline_arg,
        scene_path,
        calibration_path=_declared(config, "pose_evidence", "姿态证据路径"),
        target_id=target_id,
    )


def _load_scene(scene_path):
    if not scene_path.is_file():
        raise SystemExit(
            "场景不存在: %s（加 --rebuild 或先跑 scripts/build_baseline.py）" % scene_path
        )
    report_path = scene_path.with_suffix(".json")
    if not report_path.is_file():
        raise SystemExit("场景旁挂报告不存在: " + str(report_path))
    return json.loads(report_path.read_text(encoding="utf-8"))


def _run_target(
    root, skill, scene_path, scene, profile_path, safety_path, duration_ms, target_id
):
    """在给定场景上执行一次技能，返回 (execution_result, 报告片段)。"""
    tolerance = float(scene.get("pose_tolerance_m", 0.005))
    profile = load_robot_profile(profile_path)
    safety = load_safety_policy(safety_path)
    authority = ControlAuthorityManager()

    # 后端经 factory.load_backend 装配：装配期即校验 profile 声明的能力与后端方法，
    # 并消费场景报告声明的 vision 段（未声明则要求请求显式给出证据）。
    backend = load_backend(
        "iraf_adapters.mujoco.mujoco_backend:MujocoBackend",
        {
            "model_path": str(scene_path.resolve()),
            "realtime": False,
            "manipulation": {
                "targets": {
                    target_id: {"body": target_id, "pose_tolerance_m": tolerance}
                },
                "gripper": scene["gripper"],
            },
            "vision": scene.get("vision"),
        },
        profile,
        authority,
    )
    runtime = SkillRuntime(
        profile,
        safety,
        backend,
        SkillRegistry().load_directory(root / "skills"),
        authority,
        SqliteExecutionStore(":memory:"),
    )

    now = int(time.time() * 1000)
    correlation = skill.replace("_", "-") + "-acceptance"
    parameters = {"target_id": target_id, "duration_ms": duration_ms}
    if skill == "pick_object":
        # 无视觉链路：位姿来自场景真值（仅用于仿真验收，不作为感知输入）。
        parameters["grasp_pose"] = {
            "frame_id": "world",
            "position": scene["target_position"],
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
    # visual_pick 不传位姿：位姿由配置声明的视觉证据提供（见 vision 段）。
    request = {
        "request_id": correlation,
        "idempotency_key": correlation + "-" + str(now),
        "correlation_id": correlation,
        "skill": skill,
        "skill_version_constraint": "1.0.0",
        "parameters": parameters,
        "deadline_unix_ms": now + 60000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": profile.name + "-mujoco",
        "controller": correlation,
    }
    result = runtime.execute(
        request,
        AuthenticatedContext(
            correlation, frozenset({"task.submit", "task.read"}), "local"
        ),
    )
    return result


def _vision_accuracy(scene, result, config):
    """把视觉估计与该目标的场景真值比对，返回精度证据（无视觉证据时返回 None）。

    为什么需要：抓取成功只说明"夹爪到了它认为的位置"，不说明"它认为的位置对"。
    旧的多目标验收脚本单独核对这一点，这里把它变成统一入口的一部分。

    姿态比对必须折叠立方体的 **90° 对称等价类**：同一个几何朝向可以写成 4 个
    不同四元数，直接比角度会把 0° 误报成 90°（历史实测）。
    """
    evidence = (result.get("result") or {}).get("evidence") or {}
    vision = evidence.get("vision") or {}
    estimate = vision.get("vision_world_position_m")
    if not estimate:
        return None
    truth = scene.get("target_position") or {}
    truth_position = [float(truth.get(axis, 0.0)) for axis in ("x", "y", "z")]
    position_error = float(np.linalg.norm(np.asarray(estimate) - np.asarray(truth_position)))

    acceptance = config.get("acceptance") or {}
    tolerance_m = float(acceptance.get("vision_position_tolerance_m", 0.005))
    tolerance_deg = float(acceptance.get("vision_orientation_tolerance_deg", 10.0))

    orientation_error_deg = None
    estimate_quat = vision.get("vision_quaternion_wxyz")
    truth_entry = next(
        (
            item
            for item in (scene.get("targets") or [])
            if str(item.get("id")) == str(scene.get("target_id"))
        ),
        None,
    )
    truth_quat = (truth_entry or {}).get("quaternion_wxyz")
    if estimate_quat and truth_quat:
        estimate_rotation = _quat_to_matrix(estimate_quat)
        truth_rotation = _quat_to_matrix(truth_quat)
        best = 180.0
        for quarter in range(4):  # 折叠 90° 对称等价类
            angle = math.radians(90.0 * quarter)
            fold = np.array([
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ])
            relative = estimate_rotation @ (truth_rotation @ fold).T
            cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
            best = min(best, math.degrees(math.acos(cosine)))
        orientation_error_deg = best

    return {
        "vision_world_position_m": [float(v) for v in estimate],
        "truth_position_m": truth_position,
        "position_error_m": position_error,
        "position_tolerance_m": tolerance_m,
        "orientation_error_deg": orientation_error_deg,
        "orientation_tolerance_deg": tolerance_deg,
        "passed": bool(
            position_error <= tolerance_m
            and (
                orientation_error_deg is None
                or orientation_error_deg <= tolerance_deg
            )
        ),
    }


def _quat_to_matrix(quaternion_wxyz):
    w, x, y, z = (float(v) for v in quaternion_wxyz)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _scene_target_ids(config, scene):
    """多目标 id 列表：优先取基线声明的 targets，其次取场景报告。"""
    for source in (config.get("targets"), scene.get("targets")):
        if isinstance(source, list) and source:
            ids = [str(item.get("id")) for item in source if item.get("id")]
            if ids:
                return ids
    return []


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/pick"))
    parser.add_argument("--profile", type=Path, default=None)
    parser.add_argument("--safety", type=Path, default=None)
    parser.add_argument("--duration-ms", type=int, default=None)
    parser.add_argument("--target-id", default=None)
    parser.add_argument(
        "--skill",
        choices=("pick_object", "visual_pick"),
        default="pick_object",
        help="pick_object：用场景真值位姿；visual_pick：用声明的视觉证据（同一套判据）",
    )
    parser.add_argument(
        "--all-targets",
        action="store_true",
        help="多目标基线：逐个目标重建场景并执行（报告里每个目标一份执行证据）",
    )
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="先按配置声明的构建器重建参考姿态与场景（离线验收的常规做法）",
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)

    root = args.root.resolve()
    baseline_path = _resolve(root, args.baseline)
    if not baseline_path.is_file():
        raise SystemExit("基线配置不存在: " + str(baseline_path))
    config = yaml.safe_load(baseline_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise SystemExit("基线配置必须是对象: " + str(baseline_path))

    scene_path = _resolve(
        root, args.scene or _declared(config, "scene", "场景 MJCF 输出路径")
    )
    profile_path = _resolve(
        root, args.profile or _declared(config, "profile", "RobotProfile 路径")
    )
    safety_path = _resolve(
        root,
        args.safety
        or (config.get("build") or {}).get("safety_policy")
        or DEFAULT_SAFETY_POLICY,
    )
    duration_ms = int(
        args.duration_ms
        if args.duration_ms is not None
        else (config.get("acceptance") or {}).get("duration_ms", DEFAULT_DURATION_MS)
    )
    if duration_ms < 1 or duration_ms > 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

    if args.all_targets:
        # 多目标：先按任一目标取一次场景报告以获得目标列表（列表来自配置或场景）
        if args.rebuild:
            _rebuild_scene(root, args.baseline, config, scene_path)
        scene = _load_scene(scene_path)
        target_ids = _scene_target_ids(config, scene)
        if not target_ids:
            raise SystemExit("--all-targets 需要基线或场景声明 targets 列表")
        entries = []
        for target_id in target_ids:
            # 每个目标单独重建场景：搬运约束/参考姿态是按目标生成的
            _rebuild_scene(root, args.baseline, config, scene_path, target_id=target_id)
            target_scene = _load_scene(scene_path)
            result = _run_target(
                root,
                args.skill,
                scene_path,
                target_scene,
                profile_path,
                safety_path,
                duration_ms,
                target_id,
            )
            entry = {
                "target_id": target_id,
                "scene": target_scene,
                "execution": result,
            }
            accuracy = _vision_accuracy(target_scene, result, config)
            if accuracy is not None:
                entry["vision_accuracy"] = accuracy
            entries.append(entry)
        report = {
            "schema_version": "iraf.pick-acceptance/v1",
            "simulation_only": True,
            "baseline": str(args.baseline),
            "skill": args.skill,
            "all_targets": True,
            "targets": entries,
            # 通过判据 = 每个目标都抓取成功，且（若有视觉证据）视觉精度达标：
            # 抓取成功只说明"夹爪到了它认为的位置"，视觉精度才说明"它认为的对"。
            "passed": all(
                entry["execution"].get("status") == "SUCCEEDED"
                and (entry.get("vision_accuracy") or {}).get("passed", True)
                for entry in entries
            ),
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 1

    if args.rebuild:
        _rebuild_scene(root, args.baseline, config, scene_path, target_id=args.target_id)
    scene = _load_scene(scene_path)
    target_id = args.target_id or scene["target_id"]
    result = _run_target(
        root, args.skill, scene_path, scene, profile_path, safety_path, duration_ms, target_id
    )
    report = {
        "schema_version": "iraf.pick-acceptance/v1",
        "simulation_only": True,
        "baseline": str(args.baseline),
        "skill": args.skill,
        "scene": scene,
        "execution": result,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if result.get("status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
