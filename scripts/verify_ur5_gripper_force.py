"""实测 Robotiq 2F-85 夹持 50mm 方块的能力：力上限提高前 vs 后。

为什么要这个脚本（决策 q1 的复核）：
官方 2F-85 的 tendon 执行器 forcerange 只有 ±5N，加上 spring_link 回位刚度
0.05，实测 pad 间距几乎不变、夹不住 40g 方块。用户决定先提高力上限，
用数据决定是否需要改成位置执行器。因此本脚本必须给出**对比数据**而不是
"能夹住/夹不住"的定性结论：

1. pad 间距 vs ctrl 曲线（0..255 分段），看是否单调、行程是否够 50mm；
2. 与方块接触后的双侧法向力（用 mj_contactForce 分解到接触法向）；
3. 抬升位移：闭合后沿竖直方向抬 80mm，看方块是否跟随（纯摩擦，无焊接）。

场景是程序化生成的最小场景（工作台 + 方块 + 直接落到台面上的夹爪），
不做整臂 IK —— 本脚本只回答"夹爪能不能夹住"，臂的问题由另一个脚本回答。
"""

import argparse
import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np

ACTUATOR_NAME = "rq2f85_fingers_actuator"
OFFICIAL_FORCERANGE = (-5.0, 5.0)


def _name(model, obj, index):
    return mujoco.mj_id2name(model, obj, index)


def _geom(model, geom_name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        raise ValueError("缺少 geom: " + geom_name)
    return int(geom_id)


def _qpos_adr(model, joint_name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError("缺少 joint: " + joint_name)
    return int(model.jnt_qposadr[joint_id])


def _dof_adr(model, joint_name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    return int(model.jnt_dofadr[joint_id])


def build_scene(base_model, output, half_size, mass_kg, top_z, box_xy):
    """在组装模型后面追加工作台与方块，输出可加载的完整场景。

    注意必须**摘掉 keyframe**：UR5e 自带的 home 关键帧只有 6 个臂关节的
    qpos，追加 freejoint 后模型 nq 变成 21，直接加载会报
    "keyframe 0: invalid qpos size, expected length 21"。
    本脚本不使用 keyframe（臂位姿由脚本显式给定），因此整体移除最干净。
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(str(base_model))
    root = tree.getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("组装模型缺少 worldbody")
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)

    ET.SubElement(
        world,
        "geom",
        name="workbench",
        type="box",
        pos="0 0 %.9f" % (top_z - 0.025),
        size="0.8 0.8 0.025",
        friction="1.0 0.02 0.001",
        rgba="0.35 0.35 0.38 1",
    )
    box = ET.SubElement(
        world,
        "body",
        name="box_01",
        pos="%.9f %.9f %.9f" % (box_xy[0], box_xy[1], top_z + half_size),
    )
    ET.SubElement(box, "freejoint", name="box_01_free")
    ET.SubElement(
        box,
        "geom",
        name="box_01_geom",
        type="box",
        size="%.9f %.9f %.9f" % (half_size, half_size, half_size),
        mass=str(mass_kg),
        friction="2.0 0.05 0.001",
        rgba="0.82 0.22 0.12 1",
    )

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="    ")
    tree.write(str(output), encoding="unicode", xml_declaration=True)
    return output


def pad_gap(model, data, left_pad_geom, right_pad_geom):
    """双侧 pad 内表面的间隙估计：两 geom 中心距减去两者在 y 方向的半厚。"""
    centres = data.geom_xpos[[left_pad_geom, right_pad_geom]]
    thickness = float(model.geom_size[left_pad_geom][1]) * 2.0
    return float(np.linalg.norm(centres[0] - centres[1])) - thickness


def contact_force_on_box(model, data, box_geom, pad_geoms):
    """返回方块受到的双侧法向力（分解到接触法向），单位 N。"""
    forces = {"left": 0.0, "right": 0.0}
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        pair = {int(contact.geom1), int(contact.geom2)}
        if box_geom not in pair:
            continue
        other = (pair - {box_geom}).pop()
        side = None
        for pad_geom, label in ((pad_geoms[0], "left"), (pad_geoms[1], "right")):
            if other == pad_geom:
                side = label
        if side is None:
            continue
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, index, force)
        forces[side] += float(force[0])
    return forces


def _write_arm_ctrl(model, data, arm_positions):
    """把臂关节的 ctrl 设为目标角，由官方执行器维持（不是直接写 qpos）。

    为什么不能直接写 qpos：直接写 qpos 等于把臂当刚体硬拖，
    接触与惯性会把方块加速到 1e4 m（实测方块 z=12220m），
    数据完全不可用。UR5e 的 <general> 执行器本身就是位置伺服
    （gainprm=2000、biasprm=[0,-2000,-400]），把 ctrl 设为目标角即可
    得到稳定的位置保持——这也是后端应有的用法。
    """
    for joint_name, value in arm_positions.items():
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR,
            joint_name.replace("_joint", ""),
        )
        if actuator_id < 0:
            raise ValueError("缺少关节对应的执行器: " + joint_name)
        data.ctrl[int(actuator_id)] = float(value)


def _arm_positions(args):
    return {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": float(args.shoulder_lift_deg) * np.pi / 180.0,
        "elbow_joint": float(args.elbow_deg) * np.pi / 180.0,
        "wrist_1_joint": float(args.wrist1_deg) * np.pi / 180.0,
        "wrist_2_joint": float(args.wrist2_deg) * np.pi / 180.0,
        "wrist_3_joint": float(args.wrist3_deg) * np.pi / 180.0,
    }


def sweep_ctrl(model, data, actuator_id, ctrl_values, settle_steps,
               arm_positions, left_pad, right_pad):
    """扫 ctrl 取 pad 间距曲线（方块不存在时按运动学考察）。"""
    rows = []
    for value in ctrl_values:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        _write_arm_ctrl(model, data, arm_positions)
        data.ctrl[actuator_id] = float(value)
        mujoco.mj_forward(model, data)
        for _ in range(int(settle_steps)):
            _write_arm_ctrl(model, data, arm_positions)
            mujoco.mj_step(model, data)
        rows.append(
            {
                "ctrl": float(value),
                "pad_gap_m": round(pad_gap(model, data, left_pad, right_pad), 6),
                "driver_left_rad": round(
                    float(data.qpos[_qpos_adr(model, "rq2f85_left_driver_joint")]), 6
                ),
                "driver_right_rad": round(
                    float(data.qpos[_qpos_adr(model, "rq2f85_right_driver_joint")]), 6
                ),
            }
        )
    return rows


def grip_and_lift(model, data, actuator_id, ctrl_value, settle_steps,
                  lift_steps, arm_positions, box_body, left_pad, right_pad,
                  lift_delta_rad):
    """闭合夹爪 -> 稳定 -> 竖直抬升，返回全过程证据。"""
    box_geom = _geom(model, "box_01_geom")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box_body)
    if box_id < 0:
        raise ValueError("缺少 body: " + box_body)
    box_id = int(box_id)
    start_z = float(data.xpos[box_id][2])

    data.ctrl[:] = 0.0
    data.ctrl[actuator_id] = float(ctrl_value)
    _write_arm_ctrl(model, data, arm_positions)
    mujoco.mj_forward(model, data)

    peak = {"left": 0.0, "right": 0.0}
    for _ in range(int(settle_steps)):
        data.ctrl[actuator_id] = float(ctrl_value)
        _write_arm_ctrl(model, data, arm_positions)
        mujoco.mj_step(model, data)
        forces = contact_force_on_box(model, data, box_geom, (left_pad, right_pad))
        peak["left"] = max(peak["left"], abs(forces["left"]))
        peak["right"] = max(peak["right"], abs(forces["right"]))

    grip_gap = pad_gap(model, data, left_pad, right_pad)
    grip_forces = contact_force_on_box(model, data, box_geom, (left_pad, right_pad))
    held_box = np.asarray(data.xpos[box_id], dtype=float).copy()
    held_pad_mid = (
        np.asarray(data.geom_xpos[left_pad], dtype=float)
        + np.asarray(data.geom_xpos[right_pad], dtype=float)
    ) / 2.0

    # 竖直抬升：叠加关节位移把末端抬起。
    # shoulder_lift 负向 + elbow 正向是"抬腕"的常见组合，位移量由参数给定；
    # 这里用一个明确的关节增量而不是 IK —— 目的是抬升，不是精确轨迹。
    lift_plan = dict(arm_positions)
    lift_plan["shoulder_lift_joint"] = (
        arm_positions["shoulder_lift_joint"] - float(lift_delta_rad)
    )
    lift_plan["elbow_joint"] = arm_positions["elbow_joint"] + float(lift_delta_rad)
    for _ in range(int(lift_steps)):
        data.ctrl[actuator_id] = float(ctrl_value)
        _write_arm_ctrl(model, data, lift_plan)
        mujoco.mj_step(model, data)

    lifted_box = np.asarray(data.xpos[box_id], dtype=float).copy()
    lifted_pad_mid = (
        np.asarray(data.geom_xpos[left_pad], dtype=float)
        + np.asarray(data.geom_xpos[right_pad], dtype=float)
    ) / 2.0
    # 方块相对 pad 中点的位移才是"是否被夹住"的硬证据：
    # 只看方块 z 增加会被"机械臂把方块顶起来"污染。
    relative_shift = float(np.linalg.norm(lifted_box - lifted_pad_mid)
                           - np.linalg.norm(held_box - held_pad_mid))
    return {
        "ctrl": float(ctrl_value),
        "pad_gap_closed_m": round(grip_gap, 6),
        "start_box_z_m": round(start_z, 6),
        "held_box_z_m": round(float(held_box[2]), 6),
        "lifted_box_z_m": round(float(lifted_box[2]), 6),
        "lift_delta_m": round(float(lifted_box[2] - start_z), 6),
        "pad_mid_rise_m": round(float(lifted_pad_mid[2] - held_pad_mid[2]), 6),
        "box_relative_to_pad_shift_m": round(relative_shift, 6),
        "peak_normal_force_n": {
            "left": round(peak["left"], 6),
            "right": round(peak["right"], 6),
        },
        "normal_force_n": {
            "left": round(abs(grip_forces["left"]), 6),
            "right": round(abs(grip_forces["right"]), 6),
        },
        "held": bool(relative_shift < 0.010),
    }


def _write_with_relocated_assets(model, template_scene, output):
    """把内存模型序列化到模板场景同目录，并修正 meshdir / 网格引用。

    mj_saveLastXML 写出的是"编译后模型的反向工程"，它会把每个网格写成
    自己的资产文件（base_0.obj 等），且 meshdir 指向序列化时的相对位置。
    因此必须：
    1) 输出到与模板场景相同的目录（assets/ 就在旁边）；
    2) 显式写入 meshdir="assets"，使相对路径可解析。
    """
    import xml.etree.ElementTree as ET

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(output), model)
    tree = ET.parse(str(output))
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", "assets")
    compiler.set("angle", "radian")
    ET.indent(tree, space="    ")
    tree.write(str(output), encoding="unicode", xml_declaration=True)
    return output


def measure(model_path, args, forcerange_label):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
    if actuator_id < 0:
        raise ValueError("缺少夹爪执行器 " + ACTUATOR_NAME)
    actuator_id = int(actuator_id)
    left_pad = _geom(model, "rq2f85_left_pad1")
    right_pad = _geom(model, "rq2f85_right_pad1")
    arm_positions = _arm_positions(args)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    _write_arm_ctrl(model, data, arm_positions)
    mujoco.mj_forward(model, data)
    # 用 pinch site 与双侧 pad 中点的相对位置说明"抓取点在夹爪的哪里"。
    pinch_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pinch"))
    pinch_pos = np.asarray(data.site_xpos[pinch_id], dtype=float).copy()
    pad_mid = (
        np.asarray(data.geom_xpos[left_pad], dtype=float)
        + np.asarray(data.geom_xpos[right_pad], dtype=float)
    ) / 2.0
    box_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01"))
    box_pos = np.asarray(data.xpos[box_id], dtype=float).copy()

    sweep = sweep_ctrl(
        model, data, actuator_id,
        ctrl_values=list(range(0, 256, 15)) + [255],
        settle_steps=int(args.sweep_steps),
        arm_positions=arm_positions,
        left_pad=left_pad,
        right_pad=right_pad,
    )
    grip = grip_and_lift(
        model, data, actuator_id,
        ctrl_value=float(args.ctrl),
        settle_steps=int(args.settle_steps),
        lift_steps=int(args.lift_steps),
        arm_positions=arm_positions,
        box_body="box_01",
        left_pad=left_pad,
        right_pad=right_pad,
        lift_delta_rad=float(args.lift_delta_deg) * np.pi / 180.0,
    )
    gaps = [row["pad_gap_m"] for row in sweep]
    monotonic = all(
        gaps[index] >= gaps[index + 1] - 1e-4 for index in range(len(gaps) - 1)
    )
    return {
        "forcerange_label": forcerange_label,
        "forcerange": [float(v) for v in model.actuator_forcerange[actuator_id]],
        "pad_mid_world_m": [round(float(v), 6) for v in pad_mid],
        "pinch_site_world_m": [round(float(v), 6) for v in pinch_pos],
        "pinch_minus_pad_mid_m": [round(float(v), 6) for v in (pinch_pos - pad_mid)],
        "box_center_world_m": [round(float(v), 6) for v in box_pos],
        "pad_gap_open_m": round(gaps[0], 6),
        "pad_gap_closed_m": round(gaps[-1], 6),
        "pad_stroke_m": round(gaps[0] - gaps[-1], 6),
        "pad_gap_monotonic": bool(monotonic),
        "ctrl_sweep": sweep,
        "grip": grip,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-model", default="build/models/ur5e_2f85/ur5e_2f85.xml")
    parser.add_argument(
        "--scene", type=Path,
        default=Path("build/models/ur5e_2f85/gripper-force-scene.xml"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/ur5-gripper-force")
    )
    parser.add_argument("--half-size", type=float, default=0.025)
    parser.add_argument("--mass-kg", type=float, default=0.04)
    parser.add_argument("--ctrl", type=float, default=255.0)
    parser.add_argument("--sweep-steps", type=int, default=1200)
    parser.add_argument("--settle-steps", type=int, default=2500)
    parser.add_argument("--lift-steps", type=int, default=2500)
    parser.add_argument("--lift-delta-deg", type=float, default=12.0)
    parser.add_argument("--shoulder-lift-deg", type=float, default=-90.0)
    parser.add_argument("--elbow-deg", type=float, default=90.0)
    parser.add_argument("--wrist1-deg", type=float, default=-90.0)
    parser.add_argument("--wrist2-deg", type=float, default=-90.0)
    parser.add_argument("--wrist3-deg", type=float, default=0.0)
    args = parser.parse_args(argv)

    results = []
    scene = build_scene(
        Path(args.base_model).resolve(),
        args.scene,
        float(args.half_size),
        float(args.mass_kg),
        top_z=0.0,
        box_xy=(0.0, 0.0),
    )
    results.append(measure(scene, args, "raised_50N"))

    # 对比组：把 forcerange 改回官方 ±5N（只改内存模型，不改源文件）。
    # 注意 mj_saveLastXML 会把 meshdir 写成绝对路径，而 mesh 引用是相对路径；
    # 因此对比场景必须写在**与主场景同一目录**下（assets/ 才对得上），
    # 不能放到 /tmp（实测报 "Error opening file '/tmp/.../assets/base_0.obj'"）。
    official_path = scene.with_name("gripper-force-scene-official5n.xml")
    model_official = mujoco.MjModel.from_xml_path(str(scene.resolve()))
    for index in range(model_official.nu):
        if _name(model_official, mujoco.mjtObj.mjOBJ_ACTUATOR, index) == ACTUATOR_NAME:
            model_official.actuator_forcerange[index] = OFFICIAL_FORCERANGE
    _write_with_relocated_assets(model_official, scene, official_path)
    results.append(measure(official_path, args, "official_5N"))

    report = {
        "schema_version": "iraf.ur5-gripper-force/v1",
        "half_size_m": float(args.half_size),
        "mass_kg": float(args.mass_kg),
        "ctrl": float(args.ctrl),
        "measurements": results,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    for item in results:
        print("=== %s forcerange=%s ===" % (item["forcerange_label"], item["forcerange"]))
        print("  pad_gap open=%.6f closed=%.6f stroke=%.6f monotonic=%s"
              % (item["pad_gap_open_m"], item["pad_gap_closed_m"],
                 item["pad_stroke_m"], item["pad_gap_monotonic"]))
        print("  grip:", json.dumps(item["grip"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
