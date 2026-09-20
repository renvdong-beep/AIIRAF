"""兼容入口（已由 `scripts/verify_pick.py` 取代）。

本脚本保留是为了不让既有文档/脚本立刻失效：解析原参数后**转发**给统一入口，
验收判据、链路与报告内容完全一致。新代码请直接用：

    PYTHONPATH=src python3 scripts/verify_pick.py \
        --baseline config/piper_simulation_baseline.yaml [--rebuild]
"""

import argparse
import sys
from pathlib import Path

import verify_pick


def main(argv=None):
    print(
        "[deprecated] %s 已由 scripts/verify_pick.py 取代，本脚本仅转发参数；"
        "请更新调用方。" % Path(__file__).name,
        file=sys.stderr,
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=None)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/piper-pick"))
    parser.add_argument("--profile", type=Path, default=None)
    parser.add_argument("--safety", type=Path, default=None)
    parser.add_argument(
        "--source", type=Path, default=None,
        help="（已废弃）模型来源由配置的 model.source 决定；仅为了兼容旧调用而接受",
    )
    parser.add_argument("--duration-ms", type=int, default=2500)
    parser.add_argument("--target-id", default=None)
    # 旧行为：给了 --baseline 就重建场景；转发时保持该语义。
    parser.add_argument("--rebuild", action="store_true", default=False)
    args, unknown = parser.parse_known_args(argv)
    if unknown:
        raise SystemExit("未知参数: " + str(unknown))

    forwarded = [
        "--baseline", str(args.baseline or "config/piper_simulation_baseline.yaml"),
        "--duration-ms", str(args.duration_ms),
    ]
    for flag, value in (
        ("--scene", args.scene),
        ("--output", args.output),
        ("--profile", args.profile),
        ("--safety", args.safety),
        ("--target-id", args.target_id),
    ):
        if value is not None:
            forwarded += [flag, str(value)]
    if args.rebuild or args.baseline is not None:
        forwarded.append("--rebuild")
    return verify_pick.main(forwarded)


if __name__ == "__main__":
    raise SystemExit(main())
