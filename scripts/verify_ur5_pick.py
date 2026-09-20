"""UR5e + 2F-85 无视觉真值抓取验收：经完整 Skill Runtime 执行 pick_object。

与 Piper 的 `verify_piper_pick.py` **对等**（口径一致、判据相同）：
- 同样的四项判据：命中目标 + 双侧接触（法向力越阈）+ 抬升位移 + 位置误差；
- 同样经 TaskFlow → Runtime → Policy → Provider → Backend，不绕过任何一层；
- 同样不读仿真真值做感知（本脚本只做"无视觉"版本，感知误差由随机抓取脚本产出）。

差异只在**适配层**，全部由 profile 与 config 表达，脚本本身不含机型名称：
- 夹爪是 tendon 驱动（单 ctrl 通道 0..255）而非双位置关节；
- 双指对等判定用 `finger_geoms.left/right`（2F-85 是 pad box，
  Piper 是指腹 mesh），口径都是"双侧法向力"。
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path, default=Path("config/ur5_simulation_baseline.yaml")
    )
    parser.add_argument(
        "--scene", type=Path, default=Path("build/models/ur5-pick-scene.xml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/ur5-pick")
    )
    parser.add_argument("--profile", type=Path, default=Path("profiles/ur5_mujoco.yaml"))
    parser.add_argument(
        "--safety", type=Path, default=Path("profiles/safety/simulation_lab.yaml")
    )
    parser.add_argument("--duration-ms", type=int, default=12000)
    args = parser.parse_args(argv)

    if args.duration_ms < 1 or args.duration_ms > 30000:
        parser.error("duration-ms 必须在 1..30000 之间")

    root = Path(__file__).resolve().parents[1]
    baseline = yaml.safe_load(
        (root / args.baseline).read_text(encoding="utf-8")
    )
    scene_path = (root / args.scene).resolve()
    if not scene_path.is_file():
        raise FileNotFoundError(
            "场景不存在（请先跑 scripts/build_ur5_baseline.py）: " + str(scene_path)
        )

    # 场景 report（含后端需要的 gripper 配置与目标位置）。
    report_path = scene_path.with_suffix(".json")
    if not report_path.is_file():
        raise FileNotFoundError("场景 report 不存在: " + str(report_path))
    scene = json.loads(report_path.read_text(encoding="utf-8"))

    acceptance = baseline.get("acceptance") or {}
    tolerance = float(acceptance.get("pose_tolerance_m", 0.005))

    profile_path = args.profile
    safety_path = args.safety
    profile = load_robot_profile(root / profile_path)
    safety = load_safety_policy(root / safety_path)
    authority = ControlAuthorityManager()

    # 后端经 factory.load_backend 装配：装配期会交叉校验
    # profile 声明的能力与后端实际方法，缺方法在装配期即失败。
    backend = load_backend(
        "iraf_adapters.mujoco.mujoco_backend:MujocoBackend",
        {
            "model_path": str(scene_path),
            "realtime": False,
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
        "request_id": "ur5-pick-acceptance",
        "idempotency_key": "ur5-pick-acceptance-" + str(now),
        "correlation_id": "ur5-pick-acceptance",
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
        "resource_id": profile.name + "-mujoco",
        "controller": "ur5-pick-acceptance",
    }
    result = runtime.execute(
        request,
        AuthenticatedContext(
            "ur5-pick-acceptance", frozenset({"task.submit", "task.read"}), "local"
        ),
    )
    report = {
        "schema_version": "iraf.ur5-pick-acceptance/v1",
        "simulation_only": True,
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
