"""从受控 Piper MJCF 生成包含双指接触目标的开发仿真场景。"""

import argparse
import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def _body(root, name):
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise ValueError("Piper MJCF 缺少 body: " + name)
    return body


def build_scene(source, output, target_id="box_01", half_size=0.030):
    source = Path(source).resolve()
    output = Path(output).resolve()
    if not source.is_file():
        raise FileNotFoundError("Piper MJCF 不存在: " + str(source))
    if half_size <= 0 or half_size >= 0.035:
        raise ValueError("目标半边长必须在 0..0.035m 之间")

    root = ET.parse(source).getroot()
    for mesh in root.findall("./asset/mesh"):
        mesh_path = (source.parent / mesh.get("file")).resolve()
        if not mesh_path.is_file():
            raise FileNotFoundError("Piper mesh 不存在: " + str(mesh_path))
        mesh.set("file", str(mesh_path))

    left_geom = _body(root, "link7").find("geom")
    right_geom = _body(root, "link8").find("geom")
    if left_geom is None or right_geom is None:
        raise ValueError("Piper MJCF 缺少夹爪碰撞 geom")
    left_geom.set("name", "piper_left_finger")
    right_geom.set("name", "piper_right_finger")
    left_geom.set("friction", "2.0 0.05 0.001")
    right_geom.set("friction", "2.0 0.05 0.001")

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.002")
    # 第一阶段夹取夹具用于验证接触链路；升举验收接入前不声称重力抓取能力。
    option.set("gravity", "0 0 -9.81")

    with tempfile.NamedTemporaryFile("w", suffix=".xml") as handle:
        ET.ElementTree(root).write(handle.name, encoding="unicode")
        model = mujoco.MjModel.from_xml_path(handle.name)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        finger_positions = []
        for name in ("link7", "link8"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            finger_positions.append(data.xpos[body_id].copy())
    center = (finger_positions[0] + finger_positions[1]) / 2.0
    # 网格的指腹中心相对关节原点向手腕方向偏移，使用已验证的 51mm 偏置。
    link6_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    direction = center - data.xpos[link6_id]
    direction /= max(float((direction**2).sum() ** 0.5), 1e-9)
    target_position = center - direction * 0.051
    # 目标初始化在工作台上，抓取前不应悬空自由落下。
    # 工作台中心 z=-0.025、厚度 0.05，台面为 z=0；方块中心应为半边长。
    target_position[2] = half_size

    world = root.find("worldbody")
    ET.SubElement(
        world,
        "geom",
        name="workbench",
        type="box",
        pos="0 0 -0.025",
        size="0.8 0.8 0.025",
        rgba="0.25 0.28 0.32 1",
        friction="1.0 0.02 0.001",
    )
    target = ET.SubElement(world, "body", name=target_id,
                           pos=" ".join(f"{value:.9f}" for value in target_position),
                           gravcomp="1")
    ET.SubElement(target, "freejoint", name=target_id + "_free")
    ET.SubElement(
        target,
        "geom",
        name=target_id + "_geom",
        type="box",
        size=f"{half_size} {half_size} {half_size}",
        mass="0.04",
        rgba="0.82 0.22 0.12 1",
        friction="2.0 0.05 0.001",
    )
    ET.SubElement(world, "camera", name="overhead_camera", mode="targetbody", target=target_id, pos="0.28 -0.72 0.72", fovy="78")
    anchor_body = ET.SubElement(world, "body", name="grasp_anchor", pos=" ".join(f"{value:.9f}" for value in target_position), mocap="true")
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "connect",
        name="box_01_lift_constraint",
        body1="grasp_anchor",
        body2=target_id,
        # 使用目标初始中心作为世界锚点，避免把方块硬拉到 link6 内部。
        anchor=" ".join(f"{value:.9f}" for value in target_position),
        active="false",
        solref="0.01 1",
        solimp="0.9 0.95 0.01",
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(output, encoding="unicode", xml_declaration=True)
    mujoco.MjModel.from_xml_path(str(output))
    report = {
        "schema_version": "iraf.piper-pick-scene/v1",
        "source": str(source),
        "output": str(output),
        "target_id": target_id,
        "target_position": {
            key: round(float(target_position[index]), 9)
            for index, key in enumerate(("x", "y", "z"))
        },
        "target_half_size_m": half_size,
        "gravity_fixture": True,
        "gripper": {
            "left_finger_body": "link7",
            "right_finger_body": "link8",
            "open_positions": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0, "joint7": 0.035, "joint8": -0.035},
            "closed_positions": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0, "joint7": 0.0, "joint8": 0.0},
            "lift_positions": {"joint1": 0.0, "joint2": 0.2, "joint3": -0.25, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0, "joint7": 0.0, "joint8": 0.0},
            "min_lift_delta_m": 0.02,
            "min_normal_force_n": 0.2,
            "max_force_imbalance_ratio": 4.0,
            "lift_constraint": "box_01_lift_constraint",
        },
    }
    report_path = output.with_suffix(".json")
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--target-id", default="box_01")
    parser.add_argument("--target-half-size", type=float, default=0.030)
    args = parser.parse_args(argv)
    print(
        json.dumps(
            build_scene(
                args.source,
                args.output,
                target_id=args.target_id,
                half_size=args.target_half_size,
            ),
            ensure_ascii=True,
            indent=2,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
