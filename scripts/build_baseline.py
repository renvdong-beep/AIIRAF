"""统一基线构建入口：按配置声明的构建器生成参考姿态与受控场景。

为什么需要（AGENTS.md 6.3 / 6.4）：接入新机型时不应让使用者"挑机型脚本"。
本入口只做三件事：

1. 读基线配置（含 `build.baseline_module` 与 `build.pose_evidence` 声明）；
2. 按声明分派到对应构建器模块（模块必须提供
   `build(root, baseline_path, scene_path, calibration_path=None)`）；
3. 打印可审计的产物摘要（产物路径、目标 id、模型来源与哈希、姿态证据路径）。

构建器**不在这里实现**：机型差异只应落在 profiles/ 与 config/，
算法在 src/iraf_core/。

用法：
  PYTHONPATH=src python3 scripts/build_baseline.py \
      --baseline config/ur5_simulation_baseline.yaml \
      --scene build/models/ur5-pick-scene.xml
"""

import argparse
import importlib
import json
import sys
from pathlib import Path

import yaml


def load_config(path):
    if not Path(path).is_file():
        raise SystemExit("基线配置不存在: " + str(path))
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise SystemExit("基线配置必须是对象: " + str(path))
    return config


def resolve_build_declaration(config, baseline_path):
    """取出 `build` 段声明；缺失即显式失败（不猜机型）。"""
    declaration = config.get("build")
    if not isinstance(declaration, dict):
        raise SystemExit(
            "基线配置缺少 build 段：请在 %s 声明 build.baseline_module"
            "（提供 build() 的构建器模块名）与 build.pose_evidence（姿态证据落盘路径）"
            % baseline_path
        )
    module_name = declaration.get("baseline_module")
    if not module_name or not isinstance(module_name, str):
        raise SystemExit("build.baseline_module 必须是非空字符串: " + str(baseline_path))
    pose_evidence = declaration.get("pose_evidence")
    if not pose_evidence or not isinstance(pose_evidence, str):
        raise SystemExit("build.pose_evidence 必须是非空字符串: " + str(baseline_path))
    return module_name, pose_evidence


def load_builder(root, module_name):
    """导入构建器模块并校验接口（缺 build() 即显式失败）。"""
    scripts_dir = str(Path(root) / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise SystemExit("构建器模块导入失败: %s (%s)" % (module_name, exc)) from exc
    builder = getattr(module, "build", None)
    if not callable(builder):
        raise SystemExit(
            "构建器模块 %s 未提供 build(root, baseline_path, scene_path, calibration_path)"
            % module_name
        )
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument(
        "--scene", type=Path, default=None,
        help="场景 MJCF 输出路径；缺省时读配置 build.scene（两者都缺即失败）",
    )
    parser.add_argument(
        "--pose-evidence", type=Path, default=None,
        help="参考姿态证据落盘路径；缺省时读配置 build.pose_evidence",
    )
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)

    root = args.root.resolve()
    config = load_config(args.baseline)
    module_name, declared_evidence = resolve_build_declaration(config, args.baseline)

    scene = args.scene
    if scene is None:
        declared_scene = (config.get("build") or {}).get("scene")
        if not declared_scene:
            raise SystemExit(
                "未指定 --scene，且配置未声明 build.scene"
            )
        scene = Path(declared_scene)
    pose_evidence = args.pose_evidence or Path(declared_evidence)

    builder = load_builder(root, module_name)
    result = builder.build(
        root, args.baseline, scene, calibration_path=pose_evidence
    )

    summary = {
        "schema_version": "iraf.baseline-build/v1",
        "status": "BUILT",
        "baseline": str(args.baseline),
        "builder_module": module_name,
        "scene": str(scene),
        "pose_evidence": str(pose_evidence),
        "target_id": result.get("target_id"),
        # 模型来源：UR5e 侧写 model_source，Piper 侧写 model_source_lock（同义留证）。
        "model_source": (result.get("model_source") or {}).get("source"),
        "model_sha256": (result.get("model_source") or {}).get("sha256"),
        "model_source_lock": result.get("model_source_lock"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
