"""通用视觉目标检测入口：从渲染的 RGB-D 估计场景中目标的 6DoF 位姿。

与机型无关：目标 id 与颜色来自**场景旁挂报告**生成的检测配置，相机名、标定文件、
图像尺寸由参数声明，证据格式为 `iraf.vision-targets/v1`。
算法编排在 `iraf_adapters.mujoco.target_detection`，本脚本只是 CLI。

用法：
  MUJOCO_GL=egl PYTHONPATH=src python3 scripts/detect_targets.py \
      --model build/models/ur5-pick-scene.xml \
      --config build/calibration/detection-config.json \
      --output build/calibration/vision-targets.json
"""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import sys  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_adapters.mujoco.depth_channel import (  # noqa: E402
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
)
from iraf_adapters.mujoco.target_detection import (  # noqa: E402
    DEFAULT_CAMERA_NAME,
    detect,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="场景 MJCF（相机必须是 fixed 模式）")
    parser.add_argument(
        "--config",
        dest="config",
        required=True,
        help="检测参数与目标列表来源；由后端按场景旁挂报告生成",
    )
    parser.add_argument("--output", default="build/calibration/vision-targets.json")
    parser.add_argument(
        "--extrinsics",
        default="build/calibration/camera_to_base.json",
        help="相机标定文件；不存在时回退到模型自带内参（不静默猜测外参）",
    )
    parser.add_argument("--camera", default=DEFAULT_CAMERA_NAME)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    args = parser.parse_args(argv)

    calibration = None
    candidate = Path(args.extrinsics)
    if candidate.is_file():
        calibration = json.loads(candidate.read_text(encoding="utf-8"))
    detect(
        args.model,
        args.output,
        args.config,
        calibration,
        args.width,
        args.height,
        args.camera,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
