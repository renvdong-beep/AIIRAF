"""诊断 2F-85 夹爪的运动学：哪些关节能承受执行器力、哪些被等式约束锁住。

前两轮实测暴露的问题（必须先用数据回答，不能猜）：

1. `ctrl` 从 0 到 255 时 driver 关节角在 -1.2 rad 附近剧烈抖动、pad 间隙
   不单调、反复出现 "Nan, Inf or huge value in QACC"。
   说明 tendon 执行器与 `equality`（joint1/joint2 的 polycoef 约束 +
   connect 约束）在互相打架。

2. driver 关节 range 是 [0, 0.8]，但实测收敛到 -1.3 rad —— 这是
   `solimplimit/solreflimit` 软限位下被等式约束硬拖出去的结果。

本脚本的目的是把"是执行器太弱"与"是约束结构不允许"分开：

- A 组：保留全部 equality，扫 ctrl，看 driver 角与 pad 间隙；
- B 组：把 4 个 driver/coupler 关节临时改成直接位置执行器
  （不依赖 tendon），看纯位置驱动下 pad 间隙是否单调、行程多少；
- C 组：全部 equality 关闭，只留 tendon，看 tendon 单独能否驱动。

三组结果直接决定"是否必须替换 tendon 执行器为位置执行器"。
"""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ACTUATOR_NAME = "rq2f85_fingers_actuator"
DRIVER_JOINTS = ("rq2f85_right_driver_joint", "rq2f85_left_driver_joint")
SPRING_JOINTS = ("rq2f85_right_spring_link_joint", "rq2f85_left_spring_link_joint")
COUPLER_JOINTS = ("rq2f85_right_coupler_joint", "rq2f85_left_coupler_joint")
FOLLOWER_JOINTS = ("rq2f85_right_follower_joint", "rq2f85_left_follower_joint")
PAD_GEOMS = ("rq2f85_left_pad1", "rq2f85_right_pad1")


def direct_qpos_sweep(model, data, joint_name, values, steps):
    """直接把 driver 关节的 qpos 设为给定值并保持，测纯运动学行程。

    为什么必须先做这一步：所有"通过执行器"的试验（tendon、位置执行器）
    都得到 driver ≈ -1.2 rad，而该关节 range 是 [0, 0.8]。这说明
    qpos 被**软限位之外的力**拖到了区间外，driver 角不能作为行程的依据。
    直接枚举 qpos 才能回答"这个 4 杆机构本身有多少行程"。
    """
    rows = []
    adr = _qpos_adr(model, joint_name)
    for value in values:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[adr] = float(value)
        mujoco.mj_forward(model, data)
        rows.append(
            {
                "qpos_rad": float(value),
                "gap_m": round(pad_gap(model, data), 6),
                "pad_centre": [
                    round(float(v), 6) for v in pad_centre(model, data)
                ],
            }
        )
    return rows


def _geom(model, name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + name)
    return int(geom_id)


def _qpos_adr(model, name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError("缺少 joint: " + name)
    return int(model.jnt_qposadr[joint_id])


def pad_centre(model, data):
    left = _geom(model, PAD_GEOMS[0])
    right = _geom(model, PAD_GEOMS[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    return (centres[0] + centres[1]) / 2.0


def pad_gap(model, data):
    left = _geom(model, PAD_GEOMS[0])
    right = _geom(model, PAD_GEOMS[1])
    centres = np.asarray(data.geom_xpos[[left, right]], dtype=float)
    half_y = float(model.geom_size[left][1])
    return float(np.linalg.norm(centres[1] - centres[0])) - 2.0 * half_y


def joint_state(model, data):
    return {
        name: round(float(data.qpos[_qpos_adr(model, name)]), 6)
        for name in DRIVER_JOINTS + SPRING_JOINTS + COUPLER_JOINTS + FOLLOWER_JOINTS
    }


def reset(model, data):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0


def settle(model, data, steps, ctrl_targets=None):
    unstable = False
    for _ in range(int(steps)):
        if ctrl_targets:
            for actuator_id, value in ctrl_targets.items():
                data.ctrl[int(actuator_id)] = float(value)
        try:
            mujoco.mj_step(model, data)
        except Exception:
            unstable = True
            break
        if not np.all(np.isfinite(data.qacc)):
            unstable = True
            break
    return unstable


def sweep_tendon(model, data, actuator_id, values, steps):
    rows = []
    for value in values:
        reset(model, data)
        mujoco.mj_forward(model, data)
        before = pad_gap(model, data)
        unstable = settle(model, data, steps, {actuator_id: value})
        rows.append(
            {
                "ctrl": float(value),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(pad_gap(model, data), 6),
                "gap_delta_m": round(pad_gap(model, data) - before, 6),
                "unstable": bool(unstable),
                **joint_state(model, data),
            }
        )
    return rows


def sweep_driver_targets(model, data, driver_actuator_ids, values, steps):
    """直接给左右 driver 关节下位置指令（绕过 tendon），看行程上限。"""
    rows = []
    for value in values:
        reset(model, data)
        mujoco.mj_forward(model, data)
        before = pad_gap(model, data)
        targets = {actuator_id: float(value) for actuator_id in driver_actuator_ids}
        unstable = settle(model, data, steps, targets)
        rows.append(
            {
                "driver_target_rad": float(value),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(pad_gap(model, data), 6),
                "gap_delta_m": round(pad_gap(model, data) - before, 6),
                "unstable": bool(unstable),
                **joint_state(model, data),
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/calibration/ur5-gripper-kinematics.json"),
    )
    args = parser.parse_args()

    source = Path(args.model).resolve()
    report = {"schema_version": "iraf.ur5-gripper-kinematics/v1", "model": str(source)}

    # ---- A 组：原样（tendon + 全部 equality）----
    model_a = mujoco.MjModel.from_xml_path(str(source))
    data_a = mujoco.MjData(model_a)
    actuator_a = int(
        mujoco.mj_name2id(model_a, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    )
    values = [0.0, 50.0, 100.0, 150.0, 200.0, 255.0]
    report["A_tendon_default"] = sweep_tendon(model_a, data_a, actuator_a, values, args.steps)
    print("=== A: tendon + 全部 equality ===")
    for row in report["A_tendon_default"]:
        print("  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f) unstable=%s driver=%s"
              % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"],
                 [row[name] for name in DRIVER_JOINTS]))

    # ---- B 组：关掉全部 equality，只留 tendon ----
    model_b = mujoco.MjModel.from_xml_path(str(source))
    data_b = mujoco.MjData(model_b)
    actuator_b = int(
        mujoco.mj_name2id(model_b, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    )
    for index in range(model_b.neq):
        model_b.eq_active0[index] = 0
    report["B_tendon_no_equality"] = sweep_tendon(
        model_b, data_b, actuator_b, values, args.steps
    )
    print("\n=== B: tendon，equality 全关 ===")
    for row in report["B_tendon_no_equality"]:
        print("  ctrl=%6.1f gap %+.6f -> %+.6f (delta %+.6f) unstable=%s driver=%s"
              % (row["ctrl"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"],
                 [row[name] for name in DRIVER_JOINTS]))

    # ---- C 组：把 driver 关节换成直接位置执行器（不依赖 tendon）----
    # 做法：在内存里给 driver 关节追加 position 执行器，tendon 执行器保留
    # 但 ctrl 置 0（不参与），只看位置驱动能拉出多少行程。
    # 追加执行器会让 nu 从 7 变 9，而 UR5e 自带的 home 关键帧只覆盖 7 个
    # ctrl，直接编译会报 "keyframe 0: invalid ctrl size, expected length 9"。
    # MjSpec 的 Python 绑定没有 delete()，因此改走"改写 XML 后编译"：
    # 摘掉 keyframe 再 MjModel.from_xml_string。
    import xml.etree.ElementTree as ET

    tree = ET.parse(str(source))
    root = tree.getroot()
    keyframe_element = root.find("keyframe")
    if keyframe_element is not None:
        root.remove(keyframe_element)
    # meshdir 原本是相对路径 "assets"；from_string 之后没有文件系统上下文，
    # 必须改写为绝对路径，否则报 "Error opening file 'assets/base_0.obj'"。
    compiler_element = root.find("compiler")
    if compiler_element is not None:
        compiler_element.set("meshdir", str((source.parent / "assets").resolve()))
    stripped = ET.tostring(root, encoding="unicode")
    spec = mujoco.MjSpec.from_string(stripped)
    for name in DRIVER_JOINTS:
        actuator = spec.add_actuator()
        actuator.name = "probe_" + name
        actuator.target = name
        actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
        actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        actuator.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        actuator.gainprm[0] = 200.0
        actuator.biasprm[1] = -200.0
        actuator.biasprm[2] = -10.0
    model_c = spec.compile()
    data_c = mujoco.MjData(model_c)
    probe_ids = {}
    for name in DRIVER_JOINTS:
        probe_ids[name] = int(
            mujoco.mj_name2id(model_c, mujoco.mjtObj.mjOBJ_ACTUATOR, "probe_" + name)
        )
    tendon_id = int(
        mujoco.mj_name2id(model_c, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    )

    rows_c = []
    for target in [0.0, 0.1, 0.2, 0.4, 0.6, 0.8]:
        data_c.qpos[:] = 0.0
        data_c.qvel[:] = 0.0
        data_c.ctrl[:] = 0.0
        mujoco.mj_forward(model_c, data_c)
        before = pad_gap(model_c, data_c)
        targets = {probe_ids[name]: target for name in DRIVER_JOINTS}
        targets[tendon_id] = 0.0
        unstable = settle(model_c, data_c, args.steps, targets)
        rows_c.append(
            {
                "driver_target_rad": float(target),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(pad_gap(model_c, data_c), 6),
                "gap_delta_m": round(pad_gap(model_c, data_c) - before, 6),
                "unstable": bool(unstable),
                **joint_state(model_c, data_c),
            }
        )
    report["C_position_driven"] = rows_c
    print("\n=== C: driver 直接位置驱动（equality 保留）===")
    for row in rows_c:
        print("  target=%+.2f rad gap %+.6f -> %+.6f (delta %+.6f) unstable=%s driver=%s"
              % (row["driver_target_rad"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"],
                 [row[name] for name in DRIVER_JOINTS]))

    # ---- D 组：driver 位置驱动 + equality 全关 ----
    model_d = model_c  # MjSpec 只能 compile 一次，D 组复用同一模型
    for index in range(model_d.neq):
        model_d.eq_active0[index] = 0
    data_d = mujoco.MjData(model_d)
    probe_ids_d = {
        name: int(mujoco.mj_name2id(model_d, mujoco.mjtObj.mjOBJ_ACTUATOR, "probe_" + name))
        for name in DRIVER_JOINTS
    }
    tendon_d = int(
        mujoco.mj_name2id(model_d, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    )
    rows_d = []
    for target in [0.0, 0.1, 0.2, 0.4, 0.6, 0.8]:
        data_d.qpos[:] = 0.0
        data_d.qvel[:] = 0.0
        data_d.ctrl[:] = 0.0
        mujoco.mj_forward(model_d, data_d)
        before = pad_gap(model_d, data_d)
        targets = {probe_ids_d[name]: target for name in DRIVER_JOINTS}
        targets[tendon_d] = 0.0
        unstable = settle(model_d, data_d, args.steps, targets)
        rows_d.append(
            {
                "driver_target_rad": float(target),
                "gap_before_m": round(before, 6),
                "gap_after_m": round(pad_gap(model_d, data_d), 6),
                "gap_delta_m": round(pad_gap(model_d, data_d) - before, 6),
                "unstable": bool(unstable),
                **joint_state(model_d, data_d),
            }
        )
    report["D_position_driven_no_equality"] = rows_d
    print("\n=== D: driver 位置驱动 + equality 全关 ===")
    for row in rows_d:
        print("  target=%+.2f rad gap %+.6f -> %+.6f (delta %+.6f) unstable=%s"
              % (row["driver_target_rad"], row["gap_before_m"], row["gap_after_m"],
                 row["gap_delta_m"], row["unstable"]))

    # ---- E 组：直接枚举 driver qpos，测纯运动学行程 ----
    # 这一组最关键：它把"执行器问题"与"机构本身行程"彻底分开。
    model_e = mujoco.MjModel.from_xml_path(str(source))
    data_e = mujoco.MjData(model_e)
    rows_e = {
        joint: direct_qpos_sweep(
            model_e, data_e, joint,
            [
                -0.90, -0.45, -0.20, -0.05, 0.0, 0.05, 0.20, 0.45, 0.80, 1.20,
            ],
            args.steps,
        )
        for joint in DRIVER_JOINTS
    }
    report["E_direct_joint_qpos"] = rows_e
    print("\n=== E: 直接枚举 driver qpos（纯运动学）===")
    for joint, rows in rows_e.items():
        print(" " + joint)
        for row in rows:
            print("   qpos=%+.2f -> gap %+.6f  pad_centre=%s"
                  % (row["qpos_rad"], row["gap_m"], row["pad_centre"]))

    # 弹簧回位刚度带来的阻力矩：tau = k * (qpos - springref)
    spring = {}
    for joint in SPRING_JOINTS:
        joint_id = mujoco.mj_name2id(model_e, mujoco.mjtObj.mjOBJ_JOINT, joint)
        dof = int(model_e.jnt_dofadr[joint_id])
        spring[joint] = {
            "stiffness": round(float(model_e.jnt_stiffness[joint_id]), 6),
            "springref": round(float(model_e.qpos_spring[
                _qpos_adr(model_e, joint)
            ]), 6),
            "damping": round(float(model_e.dof_damping[dof]), 6),
        }
    report["spring_link"] = spring
    print("\n弹簧回位参数（k * (qpos - springref) 即回位力矩）:")
    for joint, info in spring.items():
        print("  %s %s" % (joint, info))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("\nWROTE", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
