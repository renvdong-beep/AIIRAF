"""兼容入口（已由 `scripts/verify_pick.py --skill visual_pick` 取代）。

保留原因：既有文档与操作手册里出现过本路径。行为一致：参数解析后转发给统一入口，
四判据、链路与报告 schema 完全相同（统一入口的 report schema 为
`iraf.pick-acceptance/v1`，旧的 `iraf.piper-visual-pick-acceptance/v1` 已弃用）。
新代码请用：
    PYTHONPATH=src python3 scripts/verify_pick.py \
        --baseline config/piper_simulation_baseline.yaml --skill visual_pick
"""

import argparse
import sys
from pathlib import Path

import verify_pick


def main(argv=None):
    print(
        "[deprecated] scripts/verify_piper_visual_pick.py 已由 scripts/verify_pick.py "
        "--skill visual_pick 取代，本脚本仅转发参数。",
        file=sys.stderr,
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=None,
                        help="（已废弃）模型来源由配置的 model.source 决定；仅为兼容旧调用而接受")
    parser.add_argument("--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml"))
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/piper-visual-pick"))
    parser.add_argument("--duration-ms", type=int, default=12000)
    parser.add_argument("--target-id", default=None)
    args = parser.parse_args(argv)

    forwarded = [
        "--baseline", str(args.baseline),
        "--output", str(args.output),
        "--duration-ms", str(args.duration_ms),
        "--skill", "visual_pick",
        # 旧行为：给了 --baseline 就重建场景；转发时保持该语义。
        "--rebuild",
    ]
    if args.scene is not None:
        forwarded += ["--scene", str(args.scene)]
    if args.target_id is not None:
        forwarded += ["--target-id", str(args.target_id)]
    return verify_pick.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
