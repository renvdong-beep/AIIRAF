"""场景构建入口（统一入口之一）：`--scene <场景包> --robot <本体>` → 生成 MJCF + 报告。

本文件只是入口层：解析参数、调用 `iraf_adapters.unitree.scene_builder`、映射退出码。
所有机型/场景知识都在声明里（`profiles/<robot>_mujoco.yaml`、`scenes/<id>/scene.yaml`），
脚本内不含任何机型专有名称（铁律 5.3 / 6.3）。

用法::

    PYTHONPATH=src python3 scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2

退出码：

- `0` 成功（产物写到 scene.yaml 的 `model.output`，报告写成同名 `.json`）
- `1` 用法错误（缺参数、目录不存在）
- `2` 声明非法（schema 不通过、缺必需字段、Profile 未通过核心校验）
- `3` 引用完整性失败（本体/Profile/挂载参考系/传感器锚点/道具 mesh 引用不到）
- `4` 厂商资产锁校验失败（未登记或 SHA-256 不一致 —— 厂商文件只读）
- `5` 生成模型校验失败（MuJoCo 编译不通过，或注入对象在模型里找不到）

目标端（板卡）验收与真实安装不在本入口职责内；本入口的产物全部为仿真产物
（报告带 `simulation: true`），不得表述为真机/实时能力。
"""

import argparse
import yaml
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.unitree import scene_builder  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="按场景声明生成可加载的 MJCF（厂商文件只读，传感器/道具/光照由构建器注入）"
    )
    parser.add_argument("--scene", type=Path, required=True, help="场景包目录（含 scene.yaml）")
    parser.add_argument("--robot", required=True, help="场景内本体的 id（也是 Profile 的 metadata.name）")
    parser.add_argument("--output", type=Path, default=None, help="覆盖 scene.yaml 的 model.output")
    parser.add_argument("--attach", action="append", default=[], metavar="ROBOT",
                        help="附加本体的 id（可重复）：把声明了 placement 的本体合成进同一模型，"
                             "产物写 scene.yaml 的 model.joint_output（单本体产物不受影响）")
    parser.add_argument("--root", type=Path, default=None, help="仓库根（默认按本文件位置推断）")
    parser.add_argument("--quiet", action="store_true", help="只打印退出码对应摘要，不打印完整报告")
    args = parser.parse_args(argv)

    if not args.scene.exists():
        print("用法错误：场景目录不存在: %s" % args.scene, file=sys.stderr)
        return scene_builder.EXIT_REFERENCE
    try:
        # `--attach` 时写联合产物路径（声明里的 model.joint_output），否则写 model.output
        output = args.output
        if args.attach and output is None:
            scene_doc = yaml.safe_load((args.scene / "scene.yaml").read_text(encoding="utf-8"))
            joint_output = (scene_doc.get("model") or {}).get("joint_output")
            if not joint_output:
                print("用法错误：使用 --attach 时 scene.yaml 必须声明 model.joint_output", file=sys.stderr)
                return scene_builder.EXIT_DECLARATION
            output = Path(joint_output)
        report = scene_builder.build_scene_model(
            args.scene, args.robot, root=args.root, output=output, attach=tuple(args.attach)
        )
    except scene_builder.SceneBuildError as exc:
        print("场景生成失败（退出码 %d）：%s" % (exc.code, exc), file=sys.stderr)
        return int(exc.code)
    if args.quiet:
        print(
            json.dumps(
                {
                    "output": report["output"],
                    "robot": report["robot"]["id"],
                    "sensors_injected": [item["name"] for item in report["sensors"]["injected"]],
                    "sensors_vendor_referenced": [
                        item["name"] for item in report["sensors"]["vendor_referenced"]
                    ],
                    "model_facts": {
                        key: report["model_facts"][key]
                        for key in ("nq", "nv", "nu", "ncam", "nsite", "nsensor")
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
