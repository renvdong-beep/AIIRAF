"""兼容入口（已由 `scripts/profile_check.py` 取代）。

保留原因：既有文档与适配记录里出现过本路径。行为一致（通用交叉校验），
只是默认带上 UR5 的基线配置。
新代码请用：`scripts/profile_check.py --baseline config/<robot>_simulation_baseline.yaml`
"""

import sys
from pathlib import Path

import profile_check


def main(argv=None):
    print(
        "[deprecated] scripts/check_ur5_profile.py 已由 scripts/profile_check.py 取代，"
        "本脚本仅转发参数。",
        file=sys.stderr,
    )
    args = list(argv if argv is not None else sys.argv[1:])
    if "--baseline" not in args:
        args = ["--baseline", str(Path("config/ur5_simulation_baseline.yaml"))] + args
    return profile_check.main(args)


if __name__ == "__main__":
    raise SystemExit(main())
