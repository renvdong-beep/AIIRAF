"""G1 站立验收（流程验证）：装配适配器 → stand → 打印报告。

用法：PYTHONPATH=src python3 scripts/verify_g1_stand.py
退出码 0 = passed；1 = 失败。全程 simulation=true。
"""
import json
import sys
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
        ROOT / "config" / "unitree_g1_loopback.yaml", profile, authority
    )
    lease = authority.acquire("unitree_g1", "verify_g1_stand", ttl_seconds=30.0)
    report = adapter.stand(lease)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["passed"]:
        print("\nG1_STAND_PASSED", file=sys.stderr)
        return 0
    print("\nG1_STAND_FAILED", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
