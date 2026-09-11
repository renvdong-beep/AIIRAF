"""在 Ubuntu X11 图形会话中打开 Piper MuJoCo Viewer。"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("build/models/piper-pick-scene.xml"),
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=0,
        help="自动关闭时间；0 表示保持窗口直到手动关闭",
    )
    args = parser.parse_args(argv)
    if not args.model.is_file():
        parser.error("模型文件不存在: " + str(args.model))
    if args.seconds < 0:
        parser.error("seconds 不能为负数")

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        started = time.monotonic()
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            time.sleep(model.opt.timestep)
    print("VIEWER_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
