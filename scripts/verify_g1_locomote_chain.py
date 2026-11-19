"""G1 行走整链验收（进程内）：TaskFlow→Runtime→Policy→Skill→Adapter→Backend。"""
import json, sys, time, uuid
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from iraf_adapters.unitree_g1.backend import UnitreeG1Backend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402

def main():
    profile = load_robot_profile(ROOT / "profiles" / "unitree_g1_mujoco.yaml")
    safety = load_safety_policy(ROOT / "profiles" / "safety" / "humanoid_lab.yaml")
    registry = SkillRegistry(); registry.load_directory(ROOT / "skills")
    authority = ControlAuthorityManager(); store = SqliteExecutionStore(":memory:")
    backend = UnitreeG1Backend.from_config(ROOT / "config" / "unitree_g1_loopback.yaml", profile, authority)
    runtime = SkillRuntime(profile, safety, backend, registry, authority, store)
    context = AuthenticatedContext("verify", frozenset({"task.submit"}), "local")
    request = {
        "request_id": str(uuid.uuid4()), "idempotency_key": str(uuid.uuid4()),
        "correlation_id": str(uuid.uuid4()),
        "deadline_unix_ms": int(time.time() * 1000) + 120000,
        "parameters": {"velocity": {"vx_mps": 0.5, "vy_mps": 0.0, "wz_rad_s": 0.0}},
        "profile_name": profile.name, "profile_version": profile.version, "profile_digest": profile.digest,
        "safety_policy_name": safety.name, "safety_policy_version": safety.version, "safety_policy_digest": safety.digest,
        "skill": "locomote", "skill_version_constraint": "1.0.0",
        "resource_id": "unitree_g1", "controller": "verify_g1_locomote_chain",
    }
    result = runtime.execute(request, context)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get("status") == "SUCCEEDED":
        print("\nG1_LOCOMOTE_CHAIN_SUCCEEDED", file=sys.stderr); return 0
    print("\nG1_LOCOMOTE_CHAIN_FAILED: %s" % result.get("reason", ""), file=sys.stderr); return 1

if __name__ == "__main__":
    raise SystemExit(main())
