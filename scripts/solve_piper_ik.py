"""使用 MuJoCo 位置雅可比求解 Piper 预抓取位置 IK。

IK 算法本体已提取到机器人无关的 iraf_core.kinematics 契约层；
本脚本只保留 Piper 侧的模型参数解析、目标文件解析与结果落盘，
以便 solve_piper_ik 与仿真验收链共用同一套求解实现。

输出 schema 与字段保持 iraf.piper-ik/v1 不变。
"""
import argparse
import json
from pathlib import Path
import sys

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from iraf_core.kinematics import (  # noqa: E402
    KinematicsError,
    joint_ids,
    solve_position_ik,
)


def solve(
    model_path,
    target_path,
    output,
    iterations=200,
    step=0.45,
    body_name="link6",
    geom_name=None,
    joint_names=None,
    width=3,
):
    """求解并写出 iraf.piper-ik/v1 结果。

    width 为求解输出中回写的关节个数（Piper 臂为 6），
    关节名缺省沿用既有 joint1..joint6 约定，但不再在契约层硬编码。
    """
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    target_data = json.loads(Path(target_path).read_text(encoding="utf-8"))
    target_value = next(
        (
            target_data[key]
            for key in (
                "ik_target_position_m",
                "ik_link6_pregrasp_position_m",
                "target_position_m",
                "grasp_position_m",
                "pregrasp_position_m",
            )
            if key in target_data
        ),
        None,
    )
    if target_value is None:
        raise ValueError("IK 输入缺少目标位置")
    target = target_value

    if joint_names is None:
        joint_names = ["joint%d" % index for index in range(1, int(width) + 1)]
    arm_joints = joint_ids(model, joint_names)

    if geom_name:
        points = [
            {
                "kind": "geom",
                "id": int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)),
            }
        ]
        if points[0]["id"] < 0:
            raise KinematicsError("MJCF 缺少 geom: " + str(geom_name))
    else:
        body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
        if body < 0:
            raise ValueError("MJCF 缺少 " + str(body_name))
        points = [{"kind": "body", "id": body}]

    result = solve_position_ik(
        model,
        data,
        target,
        arm_joints,
        points,
        iterations=int(iterations),
        step=float(step),
        tolerance_m=1e-4,
    )

    payload = {
        "schema_version": "iraf.piper-ik/v1",
        "body": body_name,
        "geom": geom_name,
        "target_position_m": [float(value) for value in target],
        "solved_position_m": [float(value) for value in result.solved_position_m],
        "position_error_m": float(result.position_error_m),
        "iterations": int(result.iterations),
        "joint_positions": {
            name: float(result.joint_positions[name]) for name in joint_names
        },
    }
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(
        json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=True, indent=2))
    return payload


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--output", default="build/calibration/piper-ik.json")
    parser.add_argument("--body", default="link6")
    parser.add_argument("--geom")
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--step", type=float, default=0.45)
    parser.add_argument(
        "--joints",
        default=None,
        help="逗号分隔的臂关节名；缺省为 joint1..joint6",
    )
    args = parser.parse_args()
    names = (
        [item.strip() for item in args.joints.split(",") if item.strip()]
        if args.joints
        else None
    )
    solve(
        args.model,
        args.target,
        args.output,
        iterations=args.iterations,
        step=args.step,
        body_name=args.body,
        geom_name=args.geom,
        joint_names=names,
        width=len(names) if names else 6,
    )
