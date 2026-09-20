"""兼容入口（已由 `scripts/verify_pick.py --all-targets` 取代）。

保留原因：既有文档与操作手册里出现过本路径。行为一致：参数解析后转发给统一入口，
逐目标重建场景、按 ID 抓取、四判据 + 视觉精度对真值的核对全部保留
（统一入口的 report schema 为 `iraf.pick-acceptance/v1`，每个目标一条证据 + vision_accuracy）。
新代码请用：
    PYTHONPATH=src python3 scripts/verify_pick.py \
        --baseline config/piper_multi_target.yaml --all-targets --skill visual_pick
"""

import argparse
import sys
from pathlib import Path

import verify_pick


def main(argv=None):
    print(
        "[deprecated] scripts/verify_piper_multi_target_pick.py 已由 "
        "scripts/verify_pick.py --all-targets 取代，本脚本仅转发参数。",
        file=sys.stderr,
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=Path("config/piper_multi_target.yaml"))
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/piper-multi-target-pick"))
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--vision-file", type=Path, default=None,
                        help="（已废弃）证据路径由配置的 vision 段声明；仅为兼容旧调用而接受")
    parser.add_argument("--source", type=Path, default=None,
                        help="（已废弃）模型来源由配置的 model.source 决定")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)

    forwarded = [
        "--baseline", str(args.baseline),
        "--output", str(args.output),
        "--duration-ms", str(args.duration_ms),
        "--skill", "visual_pick",
        "--all-targets",
        # 统一入口按目标逐个重建场景，因此无需 --rebuild
    ]
    if args.scene is not None:
        forwarded += ["--scene", str(args.scene)]
    return verify_pick.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
