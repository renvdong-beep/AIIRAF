"""在官方 2f85.xml 上验证"动态"夹爪开合：tendon 能不能真正驱动 driver。

前面的敏感性测试是**纯运动学**的（直接写 qpos 后只做 mj_forward），
它证明driver/coupler 的 qpos 不直接影响 pad —— 这在官方模型上同样成立，
因为 pad 只挂在 follower 上，driver 是通过 **equality 约束**去驱动
follower 的，而 equality 只在 **mj_step**（动力学）里起作用。

因此本脚本做**动态**测试：
1. 在官方模型（无 UR5e）上跑 tendon 执行器，扫 ctrl，看 pad 间隙是否随
   ctrl 单调变化、行程多少；
2. 用同样的方式跑组装模型，两者对比，判断"夹爪不动"到底来自
   官方模型本身、还是来自我们的组装改动。

这一步是区分"官方模型参数问题"与"我的组装引入的问题"的关键证据。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ACTUATOR_NAME = "fingers_actuator"
PAD_ASSEMBLY = ("rq2f85_left_pad1", "rq2f85_right_pad1")
PAD_OFFICIAL = ("left_pad1", "right_pad1")


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def pad_gap(model, data, names):
    left = _geom(model, names[0])
    right = _geom(model, names[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    return float(np.linalg.norm(centres[1] - centres[0])) - 2.0 * half_y


def resolve(model, assembly):
    return {
        "pad": PAD_ASSEMBLY if assembly else PAD_OFFICIAL,
        "actuator": (("rq2f85_" + ACTUATOR_NAME) if assembly else ACTUATOR_NAME),
        "driver": (
            ["rq2f85_right_driver_joint", "rq2f85_left_driver_joint"]
            if assembly
            else ["right_driver_joint", "left_driver_joint"]
        ),
        "follower": (
            ["rq2f85_right_follower_joint", "rq2f85_left_follower_joint"]
            if assembly
            else ["right_follower_joint", "left_follower_joint"]
        ),
        "spring": (
            ["rq2f85_right_spring_link_joint", "rq2f85_left_spring_link_joint"]
            if assembly
            else ["right_spring_link_joint", "left_spring_link_joint"]
        ),
    }


def dynamic_sweep(model, data, names, ctrl_values, steps, arm_ctrl=None):
    rows = []
    actuator_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, names["actuator"]
    )
    if actuator_id < 0:
        raise ValueError("缺少执行器: " + names["actuator"])
    actuator_id = int(actuator_id)
    qpos_of = lambda joint: int(
        model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        ]
    )
    for value in ctrl_values:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        if arm_ctrl:
            for actuator_name, target in arm_ctrl.items():
                aid = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
                )
                if aid >= 0:
                    data.ctrl[int(aid)] = float(target)
        mujoco.mj_forward(model, data)
        before = pad_gap(model, data, names["pad"])
        data.ctrl[actuator_id] = float(value)
        unstable = False
        for _ in range(int(steps)):
            if arm_ctrl:
                for actuator_name, target in arm_ctrl.items():
                    aid = mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
                    )
                    if aid >= 0:
                        data.ctrl[int(aid)] = float(target)
            data.ctrl[actuator_id] = float(value)
            try:
                mujoco.mj_step(model, data)
            except Exception:
                unstable = True
                break
            if not np.all(np.isfinite(data.qacc)):
                unstable = True
                break
        rows.append(
            {
                "ctrl": float(value),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(pad_gap(model, data, names["pad"]), 6),
                "gap_delta_m": round(pad_gap(model, data, names["pad"]) - before, 6),
                "unstable": bool(unstable),
                "driver_rad": [round(float(data.qpos[qpos_of(j)]), 6) for j in names["driver"]],
                "spring_rad": [round(float(data.qpos[qpos_of(j)]), 6) for j in names["spring"]],
                "follower_rad": [round(float(data.qpos[qpos_of(j)]), 6) for j in names["follower"]],
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--official",
        default="/home/coretek/MuJoCoBin/mujoco_menagerie/robotiq_2f85/2f85.xml",
    )
    parser.add_argument("--assembly", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument(
        "--output", type=Path, default=Path("build/calibration/ur5-gripper-dynamic.json")
    )
    args = parser.parse_args()

    values = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 255.0]
    report = {"schema_version": "iraf.ur5-gripper-dynamic/v1"}

    # --- 官方模型（无臂）---
    official = mujoco.MjModel.from_xml_path(str(Path(args.official).resolve()))
    official_data = mujoco.MjData(official)
    names_official = resolve(official, assembly=False)
    report["official"] = {
        "forcerange": [
            float(v)
            for v in official.actuator_forcerange[
                mujoco.mj_name2id(official, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
            ]
        ],
        "sweep": dynamic_sweep(
            official, official_data, names_official, values, args.steps
        ),
    }
    print("=== 官方 2f85.xml（仅夹爪，动态 mj_step）===")
    print("forcerange:", report["official"]["forcerange"])
    for row in report["official"]["sweep"]:
        print("  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f) unstable=%s driver=%s follower=%s"
              % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"], row["driver_rad"],
                 row["follower_rad"]))
    gaps = [row["gap_after_m"] for row in report["official"]["sweep"]]
    print("  行程 %.6f（%.6f .. %.6f）" % (max(gaps) - min(gaps), min(gaps), max(gaps)))

    # --- 组装模型（带 UR5e，臂用 ctrl 保持）---
    assembly = mujoco.MjModel.from_xml_path(str(Path(args.assembly).resolve()))
    assembly_data = mujoco.MjData(assembly)
    names_assembly = resolve(assembly, assembly=True)
    arm_ctrl = {
        "shoulder_pan": 0.0,
        "shoulder_lift": -np.pi / 2,
        "elbow": np.pi / 2,
        "wrist_1": -np.pi / 2,
        "wrist_2": -np.pi / 2,
        "wrist_3": 0.0,
    }
    report["assembly"] = {
        "forcerange": [
            float(v)
            for v in assembly.actuator_forcerange[
                mujoco.mj_name2id(
                    assembly, mujoco.mjtObj.mjOBJ_ACTUATOR, names_assembly["actuator"]
                )
            ]
        ],
        "sweep": dynamic_sweep(
            assembly, assembly_data, names_assembly, values, args.steps,
            arm_ctrl=arm_ctrl,
        ),
    }
    print("\n=== 组装模型（UR5e + 2F-85，动态 mj_step，臂 ctrl 保持）===")
    print("forcerange:", report["assembly"]["forcerange"])
    for row in report["assembly"]["sweep"]:
        print("  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f) unstable=%s driver=%s follower=%s"
              % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"], row["driver_rad"],
                 row["follower_rad"]))
    gaps_a = [row["gap_after_m"] for row in report["assembly"]["sweep"]]
    print("  行程 %.6f（%.6f .. %.6f）" % (max(gaps_a) - min(gaps_a), min(gaps_a), max(gaps_a)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
