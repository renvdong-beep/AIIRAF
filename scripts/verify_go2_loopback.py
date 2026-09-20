"""Go2 MuJoCo loopback 验收入口（统一入口之一）：`--config <声明>` → 报告 + 退出码。

本文件只是入口层：解析参数、调用 `iraf_adapters.unitree.loopback`、映射退出码。
全部数字（PD 增益、目标高度、姿态/速度容差、站立时长、采样频率、报告路径）都来自声明
`config/go2_loopback.yaml` 与 `profiles/unitree_go2_mujoco.yaml`（铁律 5.3）。

用法::

    PYTHONPATH=src python3 scripts/verify_go2_loopback.py --config config/go2_loopback.yaml

退出码：

- `0` 全部判据通过（报告 `passed: true`）
- `1` 用法错误（缺参数、配置文件不存在）
- `2` 声明非法（缺必需键、取值非法、声明自相矛盾、控制频率与模型 timestep 不整除）
- `3` 引用完整性失败（Profile 缺失 / 被测模型未生成 / 关键帧或传感器不存在 / 关节无执行器）
- `4` 模型编译失败
- `5` 判据未通过（报告 `failed_checks` 逐条列出）

诚实边界：全部结论属于**仿真**（报告 `simulation: true`）；目标端/真机验收不在本入口职责内
（板卡不在场 → DEFERRED），本入口不得被表述为真机能力验收。
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.unitree import loopback, scene_builder  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Go2 MuJoCo loopback 验收：站立保持 / 松力停机 / 状态读取（仿真）"
    )
    parser.add_argument("--config", type=Path, default=None, help="loopback 声明文件（config/go2_loopback.yaml）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="先用 scripts/build_scene.py 的逻辑重建场景模型（默认只校验已生成的产物）",
    )
    parser.add_argument("--scene", type=Path, default=Path("scenes/handoff_lab"), help="--rebuild 时的场景包")
    parser.add_argument("--robot", default=None, help="--rebuild 时的本体 id（默认取声明里的 robot.id）")
    args = parser.parse_args(argv)

    if args.config is None:
        print("用法错误：必须给出 --config <声明文件>", file=sys.stderr)
        return loopback.EXIT_USAGE
    if not args.config.is_file():
        print("用法错误：配置文件不存在: %s" % args.config, file=sys.stderr)
        return loopback.EXIT_USAGE

    root = args.root or scene_builder.repo_root()

    if args.rebuild:
        robot = args.robot or "unitree_go2"
        try:
            scene_builder.build_scene_model(args.scene, robot, root=root)
        except scene_builder.SceneBuildError as exc:
            print("重建失败（退出码 %d）：%s" % (exc.code, exc), file=sys.stderr)
            return int(exc.code)

    try:
        report, exit_code = loopback.run_loopback(args.config, root=root, report_path=args.report)
    except loopback.LoopbackError as exc:
        print("loopback 验收失败（退出码 %d）：%s" % (exc.code, exc), file=sys.stderr)
        return int(exc.code)

    summary = {
        "passed": report["passed"],
        "exit_code": exit_code,
        "report_path": report["report_path"],
        "simulation": report["simulation"],
        "model_sha256": report["model"]["sha256"],
        "stand": {
            "height_mean_m": report["stand"]["height_mean_m"],
            "height_std_m": report["stand"]["height_std_m"],
            "hold_seconds": report["stand"]["hold_seconds"],
            "max_attitude_error_deg": report["stand"]["max_attitude_error_deg"],
            "max_tracking_error_rad": report["stand"]["max_tracking_error_rad"],
        },
        "stop": {
            "static_entered": report["stop"]["static_entered"],
            "final_speed_mps": report["stop"]["final_speed_mps"],
            "seconds_to_static": report["stop"]["seconds_to_static"],
            "collapsed": report["stop"]["collapsed"],
        },
        "checks": len(report["checks"]),
        "failed_checks": report["failed_checks"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
