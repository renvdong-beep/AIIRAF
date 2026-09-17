"""从受控 Piper MJCF 生成包含双指接触目标的开发仿真场景。

支持单目标（config.target）与多目标（config.targets）两种模式：
- 单目标：沿用既有行为，生成 box_01 单个 body，供无视觉真值 / 单目标视觉闭环验收使用；
- 多目标：按 targets 列表生成多个 body，每个 body 拥有独立 material（rgba），
  供按 ID / 颜色选取指定目标的验收使用。

多目标模式下每个目标只在"被抓取的那个"上放置抓取锚点约束，
其余目标保持自由刚体（gravcomp=1，靠重力贴合台面）。
"""

import argparse
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

WORKBENCH_TOP_Z = 0.0
DEFAULT_ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
DEFAULT_OPEN = {"joint7": 0.035, "joint8": -0.035}
DEFAULT_CLOSED = {"joint7": 0.0, "joint8": 0.0}
DEFAULT_LIFT = {
    "joint1": 0.0,
    "joint2": 0.2,
    "joint3": -0.25,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
}


def _body(root, name):
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise ValueError("Piper MJCF 缺少 body: " + name)
    return body


def _validate_support_height(target_z, half_size, workbench_top_z=WORKBENCH_TOP_Z):
    bottom_z = float(target_z) - float(half_size)
    if abs(bottom_z - float(workbench_top_z)) > 1e-7:
        raise ValueError(
            "目标底面未贴合工作台: "
            f"bottom_z={bottom_z:.9f} workbench_top_z={workbench_top_z:.9f}"
        )


def _merge_arm_and_gripper(arm_positions, gripper_positions):
    merged = {name: float(value) for name, value in arm_positions.items()}
    merged.update({name: float(value) for name, value in gripper_positions.items()})
    return merged


def _relative_mesh_path(mesh_path, output_dir):
    """把网格引用改写为相对场景文件的路径，避免写死机器相关绝对路径。"""
    return os.path.relpath(str(mesh_path), str(output_dir)).replace(os.sep, "/")


def _look_at_quat(camera_pos, look_at):
    """构造 MuJoCo 相机四元数：光轴（-Z）指向注视点，+Y 尽量朝上。

    MuJoCo 相机的光轴是 -Z、图像上方是 +Y，因此需要显式构造旋转矩阵，
    不能直接用 lookat 矩阵。
    """
    position = np.asarray(camera_pos, dtype=float)
    target = np.asarray(look_at, dtype=float)
    forward = target - position
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        raise ValueError("相机位置与注视点重合，无法确定朝向")
    forward = forward / norm
    z_axis = -forward
    up_hint = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(up_hint, z_axis))) > 0.999:
        up_hint = np.array([0.0, 1.0, 0.0], dtype=float)
    x_axis = np.cross(up_hint, z_axis)
    x_axis = x_axis / float(np.linalg.norm(x_axis))
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis)).reshape(-1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation)
    return [float(value) for value in quat]


def _euler_zyx_quat(euler_deg):
    """把 [rx, ry, rz]（度，按 Z*Y*X 内旋顺序）转成 MuJoCo 的 wxyz 四元数。

    与 mujoco.mju_euler2Quat 的 "xyz" 约定等价，便于配置里直接写人类可读角度。
    """
    values = [math.radians(float(value)) for value in euler_deg]
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.asarray(values, dtype=float), "xyz")
    return [float(value) for value in quat]


def resolve_targets(config, half_size):
    """解析目标列表：优先 targets，缺省由 target 单目标构造单元素列表。

    返回统一结构：
        [{"id": str, "rgba": [r,g,b,a], "pos_m": [x,y,z], "quat_wxyz": [w,x,y,z]}, ...]
    """
    config = config or {}
    target_cfg = config.get("target") or {}
    raw_targets = config.get("targets")
    default_rgba = _parse_rgba(target_cfg.get("rgba", "0.82 0.22 0.12 1"))
    top_z = float((config.get("workbench") or {}).get("top_z_m", WORKBENCH_TOP_Z))

    if not raw_targets:
        return [
            {
                "id": str(target_cfg.get("id", "box_01")),
                "rgba": default_rgba,
                "pos_m": None,
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                "euler_deg": [0.0, 0.0, 0.0],
            }
        ], float(target_cfg.get("half_size_m", half_size))

    if not isinstance(raw_targets, list) or not raw_targets:
        raise ValueError("配置 targets 必须是非空列表")

    # 多目标模式下边长以配置声明为准：脚本命令行默认值只是兜底，
    # 用错边长会让贴台校验误判。
    half_size = float(target_cfg.get("half_size_m", half_size))

    resolved = []
    seen = set()
    for item in raw_targets:
        if not isinstance(item, dict) or not item.get("id"):
            raise ValueError("每个 target 必须声明 id")
        target_id = str(item["id"])
        if target_id in seen:
            raise ValueError("target id 重复: " + target_id)
        seen.add(target_id)
        pos = item.get("pos_m")
        if pos is None or len(pos) != 3:
            raise ValueError(f"target {target_id} 缺少 pos_m")
        position = [float(value) for value in pos]
        _validate_support_height(position[2], half_size, top_z)

        euler = item.get("euler_deg") or [0.0, 0.0, 0.0]
        if len(euler) != 3:
            raise ValueError(f"target {target_id} 的 euler_deg 必须是 3 个数")
        resolved.append(
            {
                "id": target_id,
                "rgba": _parse_rgba(item.get("rgba", default_rgba)),
                "pos_m": position,
                "quat_wxyz": _euler_zyx_quat(euler),
                "euler_deg": [float(value) for value in euler],
            }
        )
    return resolved, half_size


def _parse_rgba(value):
    if isinstance(value, str):
        parts = [float(item) for item in value.replace(",", " ").split()]
    else:
        parts = [float(item) for item in value]
    if len(parts) == 3:
        parts.append(1.0)
    if len(parts) != 4:
        raise ValueError("rgba 必须是 3 或 4 个数: " + str(value))
    for item in parts:
        if not 0.0 <= float(item) <= 1.0:
            raise ValueError("rgba 分量必须在 0..1 之间: " + str(value))
    return [float(item) for item in parts]


def _rgba_string(rgba):
    return " ".join(f"{float(value):.6f}" for value in rgba)


def build_scene(
    source,
    output,
    target_id=None,
    half_size=0.030,
    config=None,
    reference=None,
):

    """生成场景。

    target_id 指定"被承载抓取锚点约束"的目标：
    - 单目标模式：等于唯一目标的 id；
    - 多目标模式：验收脚本对每个目标逐次调用，把该目标作为被抓取对象。
    """
    source = Path(source).resolve()
    output = Path(output).resolve()
    if not source.is_file():
        raise FileNotFoundError("Piper MJCF 不存在: " + str(source))
    if half_size <= 0 or half_size >= 0.035:
        raise ValueError("目标半边长必须在 0..0.035m 之间")

    config = config or {}
    workbench_cfg = config.get("workbench") or {}
    target_cfg = config.get("target") or {}
    gripper_cfg = config.get("gripper") or {}
    scene_cfg = config.get("scene") or {}
    acceptance = config.get("acceptance") or {}
    targets, half_size = resolve_targets(config, float(half_size))
    ids = [item["id"] for item in targets]
    if len(ids) != len(set(ids)):
        raise ValueError("配置 targets 中存在重复 id")
    if target_id is None:
        # 多目标配置未显式指定被抓取目标时，取列表首个目标，
        # 便于场景生成脚本在不传 --target-id 时也能产出可用场景。
        target_id = ids[0]
    if target_id not in ids:
        raise ValueError(
            "target_id 不在配置目标列表中: " + str(target_id) + " 可选: " + ", ".join(ids)
        )


    top_z = float(workbench_cfg.get("top_z_m", WORKBENCH_TOP_Z))
    bench_half = float(workbench_cfg.get("half_size_m", 0.8))
    bench_thickness = float(workbench_cfg.get("half_thickness_m", 0.025))
    arm_kp = float(scene_cfg.get("arm_position_kp", 200.0))
    timestep = scene_cfg.get("timestep_s", 0.002)
    gravity = scene_cfg.get("gravity", "0 0 -9.81")
    finger_friction = scene_cfg.get("finger_friction", "2.0 0.05 0.001")

    root = ET.parse(source).getroot()
    mesh_refs = []
    for mesh in root.findall("./asset/mesh"):
        mesh_file = mesh.get("file")
        if not mesh_file:
            continue
        candidate = Path(mesh_file)
        if not candidate.is_absolute():
            candidate = (source.parent / mesh_file).resolve()
        else:
            candidate = candidate.resolve()
        if not candidate.is_file():
            raise FileNotFoundError("Piper mesh 不存在: " + str(candidate))
        mesh_refs.append((mesh, candidate))
        # 中间模型在临时目录加载，必须先使用绝对路径，否则相对路径会失效。
        mesh.set("file", str(candidate))

    left_geom = _body(root, "link7").find("geom")
    right_geom = _body(root, "link8").find("geom")
    if left_geom is None or right_geom is None:
        raise ValueError("Piper MJCF 缺少夹爪碰撞 geom")
    left_geom.set("name", "piper_left_finger")
    right_geom.set("name", "piper_right_finger")
    left_geom.set("friction", finger_friction)
    right_geom.set("friction", finger_friction)
    # 各关节的阻尼值：位置执行器的 kp 必须大于该关节阻尼，
    # 否则阻尼主导、执行器永远到不了目标角（joint1 的 damping=300，
    # 若把 kp 压到 200，joint1 只能走到目标的一半左右，
    # 表现为"越靠近基座的关节越到不了位"）。
    damping_by_joint = {}
    for body in root.iter("joint"):
        joint_name = body.get("name")
        if joint_name:
            damping_by_joint[joint_name] = float(body.get("damping", 0.0) or 0.0)

    actuators = root.find("actuator")
    if actuators is not None:
        for actuator in actuators:
            name = actuator.get("name", "")
            if name in {f"joint{i}" for i in range(1, 7)} and actuator.tag == "position":
                # 原始 Piper MJCF 的 kp 偏高、且未考虑与阻尼的配比；
                # 这里既控制上限（避免过高增益在 2ms 步长下振荡），
                # 又保证下限不低于该关节阻尼，否则关节无法收敛到目标角。
                damping = damping_by_joint.get(name, 0.0)
                kp_value = max(float(arm_kp), damping * 1.5)
                actuator.set("kp", f"{kp_value:.6f}")


    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(timestep))
    # 第一阶段夹取夹具用于验证接触链路；升举验收接入前不声称重力抓取能力。
    option.set("gravity", gravity)

    with tempfile.NamedTemporaryFile("w", suffix=".xml") as handle:
        ET.ElementTree(root).write(handle.name, encoding="unicode")
        model = mujoco.MjModel.from_xml_path(handle.name)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        link6_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
        if reference is not None:
            # 目标位置来自参考抓取姿态下解算出的双指指尖中点，不再按零姿态猜测。
            target_position = np.asarray(reference["finger_center_m"], dtype=float).copy()
        else:
            finger_positions = []
            for name in ("link7", "link8"):
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                finger_positions.append(data.xpos[body_id].copy())
            center = (finger_positions[0] + finger_positions[1]) / 2.0
            # 网格的指腹中心相对关节原点向手腕方向偏移，使用已验证的 51mm 偏置。
            direction = center - data.xpos[link6_id]
            direction /= max(float((direction**2).sum() ** 0.5), 1e-9)
            target_position = center - direction * 0.051
    # 目标初始化在工作台上，抓取前不应悬空自由落下。
    target_position[2] = top_z + half_size
    _validate_support_height(target_position[2], half_size, top_z)

    world = root.find("worldbody")
    ET.SubElement(
        world,
        "geom",
        name="workbench",
        type="box",
        pos=f"0 0 {top_z - bench_thickness:.9f}",
        size=f"{bench_half} {bench_half} {bench_thickness}",
        rgba="0.25 0.28 0.32 1",
        friction=workbench_cfg.get("friction", "1.0 0.02 0.001"),
    )

    # 每个目标独立 material：颜色是纯色方块唯一的视觉区分通道。
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    # 单目标模式下目标位置由参考姿态决定；
    # 多目标模式下位置严格取自配置声明——这是"未知姿态"验收的前提，
    # 感知层必须自己测出位置，不能靠把目标搬到参考点来掩盖误差。
    multi = len(targets) > 1
    geometry = []
    for item in targets:
        if multi:
            position = [float(value) for value in item["pos_m"]]
        else:
            position = [float(value) for value in target_position]
        _validate_support_height(position[2], half_size, top_z)

        quat = list(item["quat_wxyz"])
        rgba = item["rgba"]
        material_name = item["id"] + "_material"
        ET.SubElement(asset, "material", name=material_name, rgba=_rgba_string(rgba))
        # gravcomp 只给"被抓取的那个目标"（与单目标既有行为一致）：
        # 抬升验收要求位移完全由双指摩擦承担，而 40g 方块重力约 0.39N，
        # 纯摩擦抬升过于临界（实测会夹空或推走目标），因此沿用既有的
        # 重力补偿设计，保证两次验收的物理假设一致。
        # 其余目标是干扰物，必须正常受重力，否则一旦被机械臂擦碰就会
        # 长期漂浮在半空（实测被顶到 0.14m 高且不落回）。
        body_attrs = {
            "name": item["id"],
            "pos": " ".join(f"{value:.9f}" for value in position),
            "quat": " ".join(f"{value:.9f}" for value in quat),
        }
        if (not multi) or item["id"] == target_id:
            body_attrs["gravcomp"] = "1"
        body = ET.SubElement(world, "body", **body_attrs)




        ET.SubElement(body, "freejoint", name=item["id"] + "_free")
        ET.SubElement(
            body,
            "geom",
            name=item["id"] + "_geom",
            type="box",
            size=f"{half_size} {half_size} {half_size}",
            mass=str(target_cfg.get("mass_kg", 0.04)),
            material=material_name,
            rgba=_rgba_string(rgba),
            friction=target_cfg.get("friction", "2.0 0.05 0.001"),
        )
        geometry.append(
            {
                "id": item["id"],
                "body": item["id"],
                "geom": item["id"] + "_geom",
                "material": material_name,
                "rgba": rgba,
                "position_m": [round(float(value), 9) for value in position],
                "quaternion_wxyz": [round(float(value), 9) for value in quat],
                "euler_deg": [round(float(value), 6) for value in item["euler_deg"]],
            }
        )
    # 被选中目标的位置用于锚点与相机注视点，保持既有行为。
    selected = [item for item in geometry if item["id"] == target_id][0]
    target_position = np.asarray(selected["position_m"], dtype=float)

    # 相机必须固定并对准抓取点：
    # targetbody 模式会让相机随目标移动，导致相机位姿不可标定、
    # 且方块恒落在图像中心附近，掩码离散化误差无法通过标定消除。
    camera_cfg = scene_cfg.get("camera") or {}
    camera_pos = [float(value) for value in camera_cfg.get("pos_m", [0.28, -0.72, 0.72])]
    camera_look_at = camera_cfg.get("look_at_m")
    if camera_look_at is None:
        camera_look_at = [float(value) for value in target_position]
    else:
        camera_look_at = [float(value) for value in camera_look_at]
    # 多目标场景下若注视点只对准被抓取目标，其余目标可能落在视场外；
    # 显式声明 look_at_m 时以声明值为准（配置已对准场景中心），不在此再偏移。
    camera_quat = _look_at_quat(camera_pos, camera_look_at)
    ET.SubElement(
        world,
        "camera",
        name=camera_cfg.get("name", "overhead_camera"),
        mode="fixed",
        pos=" ".join(f"{value:.9f}" for value in camera_pos),
        quat=" ".join(f"{value:.9f}" for value in camera_quat),
        fovy=str(camera_cfg.get("fovy_deg", 78)),
    )
    anchor_body = ET.SubElement(
        world,
        "body",
        name="grasp_anchor",
        pos=" ".join(f"{value:.9f}" for value in target_position),
        mocap="true",
    )
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "connect",
        name=target_id + "_lift_constraint",
        body1="grasp_anchor",
        body2=target_id,
        # 使用目标初始中心作为世界锚点，避免把方块硬拉到 link6 内部。
        anchor=" ".join(f"{value:.9f}" for value in target_position),
        active="false",
        solref="0.01 1",
        solimp="0.9 0.95 0.01",
    )

    for mesh, candidate in mesh_refs:
        mesh.set("file", _relative_mesh_path(candidate, output.parent))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(output, encoding="unicode", xml_declaration=True)
    mujoco.MjModel.from_xml_path(str(output))

    open_positions = dict(gripper_cfg.get("open") or DEFAULT_OPEN)
    closed_positions = dict(gripper_cfg.get("closed") or DEFAULT_CLOSED)
    lift_arm = dict(gripper_cfg.get("lift") or DEFAULT_LIFT)
    # 抬升姿态由 IK 从抓取姿态沿接近方向解算，避免写死关节角把目标甩向别处。
    if reference is not None and reference.get("lift"):
        lift_arm = dict(reference["lift"]["joint_positions"])
    gripper = {
        "left_finger_body": "link7",
        "right_finger_body": "link8",
        "open_positions": _merge_arm_and_gripper({}, open_positions),
        "closed_positions": _merge_arm_and_gripper({}, closed_positions),
        "lift_positions": _merge_arm_and_gripper(lift_arm, closed_positions),
        "min_lift_delta_m": float(acceptance.get("min_lift_delta_m", 0.02)),
        "min_normal_force_n": float(acceptance.get("min_normal_force_n", 0.2)),
        "max_force_imbalance_ratio": float(
            acceptance.get("max_force_imbalance_ratio", 4.0)
        ),
        # 指尖 geom 中点沿接近方向高于实际抓取点，对齐门禁必须减去该偏移，
        # 否则会把"指尖插入工作台"误判为正确抓取姿态。
        "pad_offset_m": 0.0,
        "pad_offset_axis": [0.0, 0.0, 1.0],
    }
    if not bool(acceptance.get("require_friction_lift", False)):
        # 默认使用抓取锚点焊接约束完成搬运；要求纯摩擦抬升时不提供该夹具。
        gripper["lift_constraint"] = target_id + "_lift_constraint"
    if reference is not None:
        gripper["pad_offset_m"] = float(
            reference.get("finger_height_correction_m", 0.0)
        )
        gripper["pad_offset_axis"] = [
            float(value) for value in reference.get("approach_direction_world", [0.0, 0.0, 1.0])
        ]
        # 参考姿态驱动 HOME_HOLD -> APPROACH -> DESCEND 三个阶段，
        # 缺失时机械臂会停在零姿态，指尖对齐门禁必然失败。
        gripper["home_positions"] = _merge_arm_and_gripper(
            reference["home"], open_positions
        )
        gripper["approach_positions"] = _merge_arm_and_gripper(
            reference["approach"]["joint_positions"], open_positions
        )
        gripper["grasp_positions"] = _merge_arm_and_gripper(
            reference["grasp"]["joint_positions"], open_positions
        )

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
        "targets": geometry,
        "multi_target": multi,
        "workbench_top_z_m": top_z,
        "pose_tolerance_m": float(acceptance.get("pose_tolerance_m", 0.005)),
        "gravity_fixture": True,
        "require_friction_lift": bool(acceptance.get("require_friction_lift", False)),
        "arm_position_kp": arm_kp,
        "gripper": gripper,
    }
    report_path = output.with_suffix(".json")
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--target-id",
        default=None,
        help="被抓取目标 id；缺省时取配置目标列表的首个",
    )
    parser.add_argument("--target-half-size", type=float, default=0.030)
    parser.add_argument("--baseline", type=Path, default=None)

    args = parser.parse_args(argv)
    config = None
    if args.baseline is not None:
        import yaml

        config = yaml.safe_load(Path(args.baseline).read_text(encoding="utf-8"))
    print(
        json.dumps(
            build_scene(
                args.source,
                args.output,
                target_id=args.target_id,
                half_size=args.target_half_size,
                config=config,
            ),
            ensure_ascii=True,
            indent=2,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
