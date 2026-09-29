#!/usr/bin/env python3
"""从装配模型自己的 keyframe 导出"关节初值"声明块（UR5e 进联合世界用）。

为什么：联合模型的关键帧**不猜值** —— 附加本体的每个关节都必须在 Profile 里声明初值，
否则构建器以退出码 2 拦下（实测报错：`关节 rq2f85_right_coupler_joint 未在 Profile spec.model.home
声明初值`）。这些值**不手写、不猜测**，直接取自 `scripts/assemble_ur5e_2f85.py` 产出的模型里
它自己的 `<keyframe>`。

用法：python3 scripts/gen_ur5e_model_home.py <装配模型.xml>   （输出可直接粘进 Profile）
"""

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "build/models/ur5e_2f85/ur5e_2f85.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    if int(model.nkey) < 1:
        raise SystemExit("模型没有 keyframe，无法导出初值（先跑 assemble_ur5e_2f85.py）")
    print("# nkey=%d nq=%d njnt=%d" % (model.nkey, model.nq, model.njnt))
    print("  model_home:")
    for index in range(int(model.njnt)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        address = int(model.jnt_qposadr[index])
        value = float(model.key_qpos[0][address])
        print("      %s: %.6f" % (name, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
