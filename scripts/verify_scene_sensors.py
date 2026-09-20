"""场景传感器验收入口（统一入口之一）：`--scene <场景包>` → 传感器证据报告 + 退出码。

本文件只是入口层：解析参数、调用 `iraf_adapters.unitree.scene_sensor_evidence`、映射退出码。
阈值与被测传感器全部来自声明（`scenes/<id>/baseline.yaml` 的 `sensor_acceptance`、
`scenes/<id>/scene.yaml` 的 `sensors`），脚本内不含任何机型专有名称与数字默认值
（铁律 5.3 / 6.3）。

用法::

    PYTHONPATH=src python3 scripts/verify_scene_sensors.py --scene scenes/handoff_lab

退出码：

- `0` 全部判据通过（报告 `passed: true`）
- `1` 用法错误（缺参数、路径不存在）
- `2` 声明非法（缺 `sensor_acceptance` 段/键、传感器声明不唯一、扫描契约与 num_rays 不一致）
- `3` 引用完整性失败（生成模型/相机/挂载 site/传感器绑定/标定文件引用不到）
- `4` 相机离屏渲染不可用（EGL/GL 环境原因；相机部分标 BLOCKED，雷达与 IMU 已完成）
- `5` 判据未通过（报告 `failed_checks` 逐条列出）

前置条件：被测 MJCF 必须已由 `scripts/build_scene.py` 生成（本入口不重建场景；
需要重建时先跑 `build_scene.py`，或加 `--rebuild` 让本入口显式重建一次）。

诚实边界：本入口的全部结论都属于**仿真**（报告带 `simulation: true`）；
目标端/真机验收不在本入口职责内（板卡不在场 → DEFERRED），
本入口不含任何真机证据，也不得被表述为真机能力。
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.unitree import scene_builder, scene_sensor_evidence  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="场景传感器验收：相机内参/离屏渲染、雷达射线统计、IMU 与力矩传感器读数"
    )
    parser.add_argument("--scene", type=Path, required=True, help="场景包目录（含 scene.yaml / baseline.yaml）")
    parser.add_argument("--robot", default=None, help="被测本体 id（默认取雷达所在实体）")
    parser.add_argument("--model", type=Path, default=None, help="覆盖 scene.yaml 的 model.output")
    parser.add_argument("--report", type=Path, default=None, help="覆盖报告输出路径")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="先用 scripts/build_scene.py 的逻辑重建场景模型（默认只校验已生成的产物）",
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="跳过离屏渲染（相机渲染部分标 blocked；用于无 GL 环境的显式降级，不得当作通过）",
    )
    args = parser.parse_args(argv)

    if not args.scene.exists():
        print("用法错误：场景目录不存在: %s" % args.scene, file=sys.stderr)
        return scene_sensor_evidence.EXIT_USAGE

    root = args.root or scene_builder.repo_root()
    if args.rebuild:
        try:
            scene_builder.build_scene_model(args.scene, args.robot or "unitree_go2", root=root)
        except scene_builder.SceneBuildError as exc:
            print("重建失败（退出码 %d）：%s" % (exc.code, exc), file=sys.stderr)
            return int(exc.code)

    try:
        report, exit_code = scene_sensor_evidence.verify_scene_sensors(
            args.scene,
            root=root,
            robot=args.robot,
            model_path=args.model,
            report_path=args.report,
            render=not args.no_render,
        )
    except scene_sensor_evidence.SensorEvidenceError as exc:
        print("传感器验收失败（退出码 %d）：%s" % (exc.code, exc), file=sys.stderr)
        return int(exc.code)

    summary = {
        "passed": report["passed"],
        "exit_code": exit_code,
        "report_path": report["report_path"],
        "scene": report["scene"]["id"],
        "model_sha256": report["scene"]["model_sha256"],
        "camera": {
            "fovy_rel_error": report["camera"].get("fovy_rel_error"),
            "focal_rel_error": report["camera"].get("focal_rel_error"),
            "render_status": report["camera"].get("render", {}).get("status"),
        },
        "lidar": {
            "points": report["lidar"].get("points"),
            "rays": report["lidar"].get("rays"),
            "miss_fraction": report["lidar"].get("miss_fraction"),
            "ground_fraction": report["lidar"].get("ground_fraction"),
        },
        "imu": {
            "gyro_error_rad_s": report["imu"]["gyro"]["max_abs_error_rad_s"],
            "acc_rel_error": report["imu"]["acc"]["rel_error"],
            "torque_max_abs_error_nm": report["imu"]["torque"]["max_abs_error_nm"],
        },
        "checks": len(report["checks"]),
        "failed_checks": report["failed_checks"],
        "not_proved": report["not_proved"],
    }
    import json

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
