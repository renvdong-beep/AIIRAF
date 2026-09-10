"""生成 Piper 场景并通过完整 Skill Runtime 验证双指接触抓取。"""

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
        "--output", type=Path, default=Path("build/acceptance/piper-pick")
    )
    parser.add_argument("--duration-ms", type=int, default=2500)
    args = parser.parse_args(argv)
    if args.duration_ms < 1 or args.duration_ms > 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

    scene = build_scene(args.source, args.scene)
    root = Path(__file__).resolve().parents[1]
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
                        "pose_tolerance_m": 0.005,
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
        "request_id": "piper-pick-acceptance",
        "idempotency_key": "piper-pick-acceptance-" + str(now),
        "correlation_id": "piper-pick-acceptance",
        "skill": "pick_object",
        "skill_version_constraint": "1.0.0",
        "parameters": {
            "target_id": scene["target_id"],
            "grasp_pose": {
                "frame_id": "world",
                "position": scene["target_position"],
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "duration_ms": args.duration_ms,
        },
        "deadline_unix_ms": now + 60000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "piper-mujoco",
        "controller": "piper-pick-acceptance",
    }
    result = runtime.execute(
        request,
        AuthenticatedContext(
            "piper-pick-acceptance", frozenset({"task.submit", "task.read"}), "local"
        ),
    )
    report = {
        "schema_version": "iraf.piper-pick-acceptance/v1",
        "simulation_only": True,
        "scene": scene,
        "execution": result,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0 if result.get("status") == "SUCCEEDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
