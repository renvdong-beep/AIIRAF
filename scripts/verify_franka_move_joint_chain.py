import json, sys, time, uuid
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from iraf_core.authority import ControlAuthorityManager
from iraf_core.policy import AuthenticatedContext
from iraf_core.profile import load_robot_profile, load_safety_policy
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore
from iraf_adapters.franka.backend import FrankaPandaBackend

def main():
    profile = load_robot_profile(ROOT / "profiles" / "franka_panda_mujoco.yaml")
    safety = load_safety_policy(ROOT / "profiles" / "safety" / "simulation_lab.yaml")
    registry = SkillRegistry(); registry.load_directory(ROOT / "skills")
    authority = ControlAuthorityManager(); store = SqliteExecutionStore(":memory:")
    backend = FrankaPandaBackend.from_config(
        {"root": str(ROOT), "model_file": "vendor/franka_emika_panda/scene.xml", "control_frequency_hz": 100.0},
        profile, authority)
    runtime = SkillRuntime(profile, safety, backend, registry, authority, store)
    context = AuthenticatedContext("verify", frozenset({"task.submit"}), "local")
    request = {
        "request_id": str(uuid.uuid4()), "idempotency_key": str(uuid.uuid4()),
        "correlation_id": str(uuid.uuid4()), "deadline_unix_ms": int(time.time()*1000)+60000,
        "parameters": {"positions": {"joint1": 0.5, "joint2": -0.3}, "duration_ms": 1000},
        "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest,
        "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest,
        "skill": "move_joint", "skill_version_constraint": "1.0.0",
        "resource_id": "franka_panda", "controller": "verify_franka_move_joint_chain",
    }
    result = runtime.execute(request, context)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get("status") == "SUCCEEDED":
        print("\nFRANKA_MOVE_JOINT_CHAIN_SUCCEEDED", file=sys.stderr); return 0
    print("\nFRANKA_MOVE_JOINT_CHAIN_FAILED: %s" % result.get("reason", ""), file=sys.stderr); return 1

if __name__ == "__main__":
    raise SystemExit(main())
