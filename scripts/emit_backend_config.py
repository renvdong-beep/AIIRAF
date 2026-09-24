"""从场景旁挂报告生成 IRAF_BACKEND_CONFIG（部署用 JSON）。

为什么需要：运行期后端配置（model_path / manipulation / vision）与场景一一对应，
在部署脚本里手写这份 JSON 会随契约演进静默过期 —— 实测发生过两次：
1. 夹爪几何字段改为必填后，旧 JSON 缺少 wrist_body / *_finger_geom，服务装配即失败；
2. 视觉入口改为声明式后，旧 JSON 没有 vision 段，visual_pick 会报"未提供视觉证据路径"。

因此单源化：这份配置一律由场景报告生成，不手写。

用法：
  PYTHONPATH=src python3 scripts/emit_backend_config.py \
      --scene build/models/ur5-pick-scene.xml [--realtime] [--compact]
"""

import argparse
import json
from pathlib import Path


def build_config(scene, scene_path, realtime=False):
    """把场景报告翻译成后端配置（只搬场景已有声明，不添加默认值）。"""
    target_id = scene["target_id"]
    return {
        "model_path": str(Path(scene_path).resolve()),
        "realtime": bool(realtime),
        "manipulation": {
            "targets": {
                target_id: {
                    "body": target_id,
                    "pose_tolerance_m": float(scene.get("pose_tolerance_m", 0.005)),
                }
            },
            "gripper": scene["gripper"],
        },
        # 视觉 Provider 声明（未声明时为 None，后端据此要求请求显式给 evidence）
        "vision": scene.get("vision"),
        # 联合模型（`--attach` 产物）专有：声明名 → 模型名。**必须一起搬**，否则臂在联合模型上
        # 会以"找不到关节或执行器: joint1"在第一次运动时失败（单本体报告没有这一段 ⇒ None）。
        "name_map": (scene.get("manipulation") or {}).get("name_map"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument(
        "--compact", action="store_true",
        help="输出单行紧凑 JSON（便于直接写入 IRAF_BACKEND_CONFIG=... 的环境文件）",
    )
    args = parser.parse_args(argv)

    scene_path = args.scene if args.scene.is_absolute() else args.root / args.scene
    report_path = scene_path.with_suffix(".json")
    if not report_path.is_file():
        raise SystemExit(
            "场景旁挂报告不存在: %s（先跑 scripts/build_baseline.py）" % report_path
        )
    scene = json.loads(report_path.read_text(encoding="utf-8"))
    config = build_config(scene, scene_path, realtime=args.realtime)
    if args.compact:
        print(json.dumps(config, ensure_ascii=True, separators=(",", ":")))
    else:
        print(json.dumps(config, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
