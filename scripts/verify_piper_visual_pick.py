"""生成 Piper 场景并通过视觉抓取 Skill Runtime 验证完整闭环。

与 verify_piper_pick.py 的区别：抓取点不再来自场景报告，而是由
scripts/detect_piper_target.py 从相机渲染中检测、写入视觉证据文件，
再由 MuJoCo Backend.visual_pick 读取并驱动 pick_object。
"""

import argparse
import json
import time
from pathlib import Path

from build_piper_pick_scene import build_scene
from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_core.authority import ControlAuthorityManager
from iraf_core.policy import AuthenticatedContext
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/piper-pick-scene.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/piper-visual-pick")
    )
    parser.add_argument("--duration-ms", type=int, default=10000)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="受控仿真基线配置；提供时按基线求解参考姿态并生成场景",
    )
    parser.add_argument(
        "--vision-file",
        type=Path,
        default=Path("build/calibration/piper-vision-target.json"),
        help="视觉目标证据文件路径",
    )
    args = parser.parse_args(argv)
    if args.duration_ms < 1 or args.duration_ms > 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

    root = Path(__file__).resolve().parents[1]
    if args.baseline is not None:
        from build_piper_baseline import build_baseline_scene

        scene = build_baseline_scene(root, args.baseline, args.source, args.scene)
    else:
        scene = build_scene(args.source, args.scene)
    tolerance = float(scene.get("pose_tolerance_m", 0.005))
    profile = load_robot_profile(root / "profiles/piper_mujoco.yaml")
    safety = load_safety_policy(root / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()
    backend = MujocoBackend.from_config(
        {
            "model_path": str(args.scene.resolve()),
            "manipulation": {
                "targets": {
                    scene["target_id"]: {
                        "body": scene["target_id"],
                        "pose_tolerance_m": tolerance,
                    }
                },
                "gripper": scene["gripper"],
            },
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
    request = {
        "request_id": "piper-visual-pick-acceptance",
        "idempotency_key": "piper-visual-pick-acceptance-" + str(now),
        "correlation_id": "piper-visual-pick-acceptance",
        "skill": "visual_pick",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": scene["target_id"],
            "duration_ms": args.duration_ms,
            "vision_file": str(args.vision_file),
        },
        "deadline_unix_ms": now + 60000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "piper-mujoco",
        "controller": "piper-visual-pick-acceptance",
    }
    result = runtime.execute(
        request,
        AuthenticatedContext(
            "piper-visual-pick-acceptance",
            frozenset({"task.submit", "task.read"}),
            "local",
        ),
    )
    report = {
        "schema_version": "iraf.piper-visual-pick-acceptance/v1",
        "simulation_only": True,
        "scene": scene,
        "vision_file": str(args.vision_file),
        "execution": result,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0 if result.get("status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
