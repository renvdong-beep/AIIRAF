"""统一抓取验收入口：经完整 Skill Runtime 执行 pick_object。

判据与机型无关（AGENTS.md 6.4 金路径），且**判据全部取自 Backend 回传的证据**：
命中目标 + 双侧接触（法向力越阈）+ 抬升位移 + 位置误差。
脚本不自行判定"抓到了"，也不在失败时输出成功报告。

链路：TaskFlow → SkillRuntime → Policy/Authority → Provider → MuJoCo Backend，
不绕过任何一层；profile / safety / 场景 / 时长 / 目标 id 全部来自声明（CLI 可覆盖）。

用法：
  PYTHONPATH=src python3 scripts/verify_pick.py \
      --baseline config/ur5_simulation_baseline.yaml \
      --rebuild                      # 可选：先按配置声明的构建器重建场景
"""

import argparse
import json
import time
from pathlib import Path

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
        "--rebuild", action="store_true",
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

    if args.rebuild:
        module_name = _declared(config, "baseline_module", "构建器模块名")
        from build_baseline import load_builder  # 同目录脚本

        builder = load_builder(root, module_name)
        builder.build(
            root,
            args.baseline,
            scene_path,
            calibration_path=_declared(config, "pose_evidence", "姿态证据路径"),
        )

    if not scene_path.is_file():
        raise SystemExit(
            "场景不存在: %s（加 --rebuild 或先跑 scripts/build_baseline.py）" % scene_path
        )
    report_path = scene_path.with_suffix(".json")
    if not report_path.is_file():
        raise SystemExit("场景旁挂报告不存在: " + str(report_path))
    scene = json.loads(report_path.read_text(encoding="utf-8"))

    target_id = args.target_id or scene["target_id"]
    tolerance = float(scene.get("pose_tolerance_m", 0.005))
    profile = load_robot_profile(profile_path)
    safety = load_safety_policy(safety_path)
    authority = ControlAuthorityManager()

    # 后端经 factory.load_backend 装配：装配期即校验 profile 声明的能力与后端方法，
    # 并消费场景报告声明的 vision 段（未声明则不刷新视觉）。
    backend = load_backend(
        "iraf_adapters.mujoco.mujoco_backend:MujocoBackend",
        {
            "model_path": str(scene_path.resolve()),
            "realtime": False,
            "manipulation": {
                "targets": {
                    target_id: {
                        "body": target_id,
                        "pose_tolerance_m": tolerance,
                    }
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
    correlation = "pick-acceptance"
    request = {
        "request_id": correlation,
        "idempotency_key": correlation + "-" + str(now),
        "correlation_id": correlation,
        "skill": "pick_object",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": target_id,
            "grasp_pose": {
                "frame_id": "world",
                "position": scene["target_position"],
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "duration_ms": duration_ms,
        },
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

    report = {
        "schema_version": "iraf.pick-acceptance/v1",
        "simulation_only": True,
        "baseline": str(args.baseline),
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
