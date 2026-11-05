"""G1 行走验收：装配适配器 → walk → 打印报告。simulation=true。"""
import json, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402
from iraf_adapters.unitree_g1.backend import UnitreeG1Backend  # noqa: E402

def main():
    profile = load_robot_profile(ROOT / "profiles" / "unitree_g1_mujoco.yaml")
    authority = ControlAuthorityManager()
    adapter = UnitreeG1Backend.from_config(
        ROOT / "config" / "unitree_g1_loopback.yaml", profile, authority)
    lease = authority.acquire("unitree_g1", "verify_g1_locomote", ttl_seconds=60.0)
    report = adapter.locomote({"vx_mps": 0.5, "vy_mps": 0.0, "wz_rad_s": 0.0}, 5000, lease)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["fell"] and report["distance_m"] > 0.5:
        print("\nG1_LOCOMOTE_PASSED", file=sys.stderr)
        return 0
    print("\nG1_LOCOMOTE_FAILED", file=sys.stderr)
    return 1

if __name__ == "__main__":
    raise SystemExit(main())
