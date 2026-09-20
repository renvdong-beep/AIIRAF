"""兼容入口（已由 `scripts/detect_targets.py` 取代）。

保留原因：既有文档、脚本与部署配置里出现过本路径。参数解析后转发给通用入口，
检测算法与证据格式完全一致（`iraf_adapters.mujoco.target_detection`）。
新代码请直接用 `scripts/detect_targets.py`。
"""

import argparse
import sys
from pathlib import Path

import detect_targets


def main(argv=None):
    print(
        "[deprecated] scripts/detect_piper_targets.py 已由 scripts/detect_targets.py 取代，"
        "本脚本仅转发参数；请更新调用方。",
        file=sys.stderr,
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--config",
        "--baseline",
        dest="config",
        default="config/piper_multi_target.yaml",
    )
    parser.add_argument("--output", default="build/calibration/piper-vision-targets.json")
    parser.add_argument("--extrinsics", default="build/calibration/camera_to_base.json")
    parser.add_argument("--camera", default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    args = parser.parse_args(argv)

    forwarded = [
        "--model", str(args.model),
        "--config", str(args.config),
        "--output", str(args.output),
        "--extrinsics", str(args.extrinsics),
    ]
    if args.camera:
        forwarded += ["--camera", args.camera]
    if args.width:
        forwarded += ["--width", str(args.width)]
    if args.height:
        forwarded += ["--height", str(args.height)]
    return detect_targets.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
