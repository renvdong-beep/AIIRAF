"""兼容入口（已由 `scripts/view_mujoco.py` 取代）。

保留原因：既有文档与操作手册里出现过本路径（含 Viewer 的 Mesa/C++ ABI 启动前置）。
行为完全一致，只是默认带上传入 Piper 的基线配置。
新代码请直接用：`scripts/view_mujoco.py --baseline config/<robot>_simulation_baseline.yaml`
"""

import argparse
import sys
from pathlib import Path

import view_mujoco


def main(argv=None):
    print(
        "[deprecated] scripts/view_piper_mujoco.py 已由 scripts/view_mujoco.py 取代，"
        "本脚本仅转发参数；请更新调用方。",
        file=sys.stderr,
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--pick-duration-ms", type=int, default=12000)
    parser.add_argument("--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml"))
    parser.add_argument("--target-id", default=None)
    parser.add_argument("--correlation-id", default="viewer-pick")
    args = parser.parse_args(argv)

    forwarded = [
        "--baseline", str(args.baseline),
        "--seconds", str(args.seconds),
        "--pick-duration-ms", str(args.pick_duration_ms),
        "--correlation-id", args.correlation_id,
    ]
    if args.model is not None:
        forwarded += ["--model", str(args.model)]
    if args.target_id is not None:
        forwarded += ["--target-id", str(args.target_id)]
    return view_mujoco.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
