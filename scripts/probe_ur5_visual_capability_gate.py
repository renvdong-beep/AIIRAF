"""验证"未声明能力"会在策略层被拒，而不是运行到一半才失败。

背景（AGENTS.md 铁律 5/6）：UR5e 没有视觉 Provider，profile 因此不声明
`visual_pick`。本脚本用真实 Runtime 提交一个 visual_pick 任务，确认它被
**策略层**拒绝，且错误信息可定位（而不是落到后端才报"未提供视觉证据路径"）。
"""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.factory import load_backend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=Path("build/models/ur5-pick-scene.xml"))
    parser.add_argument("--profile", type=Path, default=Path("profiles/ur5_mujoco.yaml"))
    args = parser.parse_args(argv)

    scene_path = (ROOT / args.scene).resolve()
    scene = json.loads(scene_path.with_suffix(".json").read_text(encoding="utf-8"))
    profile = load_robot_profile(ROOT / args.profile)
    safety = load_safety_policy(ROOT / "profiles/safety/simulation_lab.yaml")
    authority = ControlAuthorityManager()
    backend = load_backend(
        "iraf_adapters.mujoco.mujoco_backend:MujocoBackend",
        {
            "model_path": str(scene_path),
            "manipulation": {
                "targets": {scene["target_id"]: {
                    "body": scene["target_id"], "pose_tolerance_m": 0.005}},
                "gripper": scene["gripper"],
            },
            "vision": scene.get("vision"),
        },
        profile,
        authority,
    )
    print("装配期契约报告:", json.dumps(backend.backend_contract, ensure_ascii=False))
    runtime = SkillRuntime(
        profile, safety, backend, SkillRegistry().load_directory(ROOT / "skills"),
        authority, SqliteExecutionStore(":memory:"),
    )
    now = int(time.time() * 1000)
    result = runtime.execute(
        {
            "request_id": "ur5-visual-declared-check",
            "idempotency_key": "ur5-visual-declared-check-" + str(now),
            "correlation_id": "ur5-visual-declared-check",
            "skill": "visual_pick",
            "skill_version_constraint": "1.0.0",
            "parameters": {"target_id": scene["target_id"], "duration_ms": 1000},
            "deadline_unix_ms": now + 10000,
            "profile_name": profile.name,
            "profile_version": profile.version,
            "profile_digest": profile.digest,
            "safety_policy_name": safety.name,
            "safety_policy_version": safety.version,
            "safety_policy_digest": safety.digest,
            "resource_id": profile.name + "-mujoco",
            "controller": "ur5-visual-declared-check",
        },
        AuthenticatedContext("probe", frozenset({"task.submit", "task.read"}), "local"),
    )
    print("visual_pick 任务结果: status=%s error_code=%s" % (result.get("status"), result.get("error_code")))
    print("reason=%s" % str(result.get("reason"))[:300])
    rejected_before_backend = result.get("status") != "SUCCEEDED" and "未提供视觉证据路径" not in str(
        result.get("reason", "")
    )
    print("结论: 在策略/校验层被拒（未落到后端）:", rejected_before_backend)
    return 0 if rejected_before_backend else 1


if __name__ == "__main__":
    raise SystemExit(main())
