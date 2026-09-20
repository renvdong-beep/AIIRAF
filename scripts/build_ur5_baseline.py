"""兼容入口（已由 `scripts/build_robot_baseline.py` 取代）。

保留原因：既有文档、配置与命令里出现过本路径（含 UR5e 的适配记录）。
行为完全一致：本模块就是通用构建器的薄包装，只是默认带上 UR5 的基线配置。
新代码请直接用：
    PYTHONPATH=src python3 scripts/build_baseline.py --baseline config/<robot>_simulation_baseline.yaml
"""

import sys
from pathlib import Path

import build_robot_baseline


def main(argv=None):
    print(
        "[deprecated] scripts/build_ur5_baseline.py 已由 scripts/build_baseline.py / "
        "build_robot_baseline.py 取代，本脚本仅转发参数。",
        file=sys.stderr,
    )
    args = list(argv if argv is not None else sys.argv[1:])
    if "--baseline" not in args:
        args = ["--baseline", str(Path("config/ur5_simulation_baseline.yaml"))] + args
    if "--scene" not in args:
        args += ["--scene", str(Path("build/models/ur5-pick-scene.xml"))]
    if "--pose-evidence" not in args:
        args += ["--pose-evidence", str(Path("build/calibration/ur5-baseline-pose.json"))]
    return build_robot_baseline.main(args)


if __name__ == "__main__":
    raise SystemExit(main())
