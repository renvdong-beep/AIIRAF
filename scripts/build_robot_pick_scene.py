"""机器人无关的抓取场景生成器：在工作台上放置目标方块与固定相机。

为什么新增这个脚本而不是改 `build_piper_pick_scene.py`：

`build_piper_pick_scene.py` 里写死了 Piper 的 body 名（`link6/link7/link8`）
与关节名（`joint1..joint8`），并且它的行为已经被 4 条验收链锁住。
按 IRAF"控制爆炸半径"的约束，**不动它**，把"从基线配置读机器人身份"
这件事做成新入口；Piper 侧继续用它原来的脚本，两条路径互不影响。

本脚本的机器人差异**全部来自基线配置**，脚本内不含任何机型专有名称：

- `model.arm_joints` — 参与 IK 的臂关节
- `model.finger_geoms.left/right` — 双指接触判别用的 geom（Piper 是指腹
  网格，2F-85 是 pad box）
- `model.bodies.wrist` — 计算夹爪指向用的腕部 body
- `scene.inject_arm_position_gains` — 是否注入 `arm_position_kp`。
  Piper 需要（官方模型缺少阻尼），UR5e **不需要**（官方 `<general>`
  执行器自带 gainprm/biasprm 阻尼位置伺服，覆盖它反而破坏官方标定）。

其余（工作台、目标、锚点夹具、相机）与 Piper 侧保持同一套语义，
保证"对等验收"的口径一致。
"""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

# 抓取段纠偏的**模式枚举**与运行期同一处（`payload_facts`）：构建期与运行期必须同口径，
# 否则会出现"构建期接受、运行期拒绝"（本会话在放置侧踩过同类坑，见 §11.25(f-4) 的枚举链）。
from iraf_adapters.mujoco.payload_facts import (
    GRASP_POSE_CORRECTION_MODES,
    PLACE_POSE_CORRECTION_MODES,
)
# 抬升段锚点驱动口径的**唯一事实来源**（后端常量）：生成器只做声明校验与透传，不复制取值集合。
from iraf_adapters.mujoco.mujoco_backend import LIFT_ANCHOR_MODES
import yaml

from iraf_adapters.mujoco.scene_lighting import inject_lights

WORKBENCH_TOP_Z = 0.0

LIFT_CONSTRAINT_TEMPLATE = "%s_lift_constraint"


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
    return " ".join("%.6f" % float(value) for value in rgba)


def _relative_path(target, start):
    """计算 target 相对 start 的路径（跨盘符时退回绝对路径）。

    MJCF 的 meshdir 允许相对与绝对两种写法；同盘符时用相对路径
    才能让场景目录整体可搬迁，跨盘符时只能用绝对路径。
    """
    target = Path(target)
    start = Path(start)
    try:
        import os

        return os.path.relpath(str(target), str(start)).replace(os.sep, "/")
    except ValueError:
        return str(target).replace(os.sep, "/")


def _euler_zyx_quat(euler_deg):
    """[rx, ry, rz]（度）转 wxyz 四元数，与 mujoco.mju_euler2Quat 的 xyz 约定一致。"""
    values = [np.radians(float(value)) for value in euler_deg]
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.asarray(values, dtype=float), "xyz")
    return [float(value) for value in quat]


def _look_at_quat(camera_pos, look_at):
    """MuJoCo 相机四元数：光轴（-Z）指向注视点，+Y 尽量朝上。"""
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


def resolve_targets(config, half_size):
    """解析目标列表：优先 targets，缺省由 target 单目标构造单元素列表。"""
    config = config or {}
    target_cfg = config.get("target") or {}
    raw_targets = config.get("targets")
    default_rgba = _parse_rgba(target_cfg.get("rgba", "0.82 0.22 0.12 1"))
    top_z = float((config.get("workbench") or {}).get("top_z_m", WORKBENCH_TOP_Z))
    declared_half = float(target_cfg.get("half_size_m", half_size))

    if not raw_targets:
        # 单目标：位置由 grasp.finger_center_xy_m 推出，z 由"贴台面"约束推出。
        # 这样单一真值来源（配置）同时决定场景与 IK 目标，避免两处不一致。
        grasp_cfg = config.get("grasp") or {}
        xy = grasp_cfg.get("finger_center_xy_m")
        if not xy or len(xy) != 2:
            raise ValueError(
                "单目标模式必须声明 grasp.finger_center_xy_m（用于推导目标位置）"
            )
        position = [float(xy[0]), float(xy[1]), top_z + declared_half]
        return [
            {
                "id": str(target_cfg.get("id", "box_01")),
                "rgba": default_rgba,
                "pos_m": position,
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                "euler_deg": [0.0, 0.0, 0.0],
            }
        ], declared_half

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
            raise ValueError("target %s 缺少 pos_m" % target_id)
        position = [float(value) for value in pos]
        bottom_z = position[2] - declared_half
        if abs(bottom_z - top_z) > 1e-7:
            raise ValueError(
                "目标 %s 底面未贴合工作台: bottom_z=%.9f workbench_top_z=%.9f"
                % (target_id, bottom_z, top_z)
            )
        euler = item.get("euler_deg") or [0.0, 0.0, 0.0]
        if len(euler) != 3:
            raise ValueError("target %s 的 euler_deg 必须是 3 个数" % target_id)
        resolved.append(
            {
                "id": target_id,
                "rgba": _parse_rgba(item.get("rgba", default_rgba)),
                "pos_m": position,
                "quat_wxyz": _euler_zyx_quat(euler),
                "euler_deg": [float(value) for value in euler],
            }
        )
    return resolved, declared_half


def build_scene(source, output, target_id=None, half_size=0.030, config=None,
                reference=None):
    """生成场景 MJCF，并返回与 Piper 侧同构的 report 字典。

    `reference` 为已求解的参考姿态（含 home/approach/grasp），提供时会把
    它写进 gripper 段供后端六阶段状态机使用。
    """
    source = Path(source).resolve()
    output = Path(output).resolve()
    if not source.is_file():
        raise FileNotFoundError("机器人 MJCF 不存在: " + str(source))

    config = config or {}
    model_cfg = config.get("model") or {}
    arm_joints = list(model_cfg.get("arm_joints") or [])
    if not arm_joints:
        raise ValueError("基线缺少 model.arm_joints")
    finger_geoms = model_cfg.get("finger_geoms") or {}
    if not finger_geoms.get("left") or not finger_geoms.get("right"):
        raise ValueError("基线缺少 model.finger_geoms.left/right")

    tree = ET.parse(str(source))
    root = tree.getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("机器人 MJCF 缺少 worldbody")

    # 输出场景可能与源模型不同目录，而 meshdir 是相对路径。
    # 必须把 meshdir 改写成"从输出目录指向源 assets"的相对路径，
    # 否则会报 "Error opening file '<output_dir>/assets/base_0.obj'"。
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    declared_meshdir = compiler.get("meshdir")
    if declared_meshdir:
        declared = Path(declared_meshdir)
        if not declared.is_absolute():
            declared = (source.parent / declared).resolve()
        compiler.set("meshdir", _relative_path(declared, output.parent))

    targets, half_size = resolve_targets(config, half_size)
    target_cfg = config.get("target") or {}
    workbench_cfg = config.get("workbench") or {}
    scene_cfg = config.get("scene") or {}
    acceptance = config.get("acceptance") or {}
    grasp_cfg = config.get("grasp") or {}
    top_z = float(workbench_cfg.get("top_z_m", WORKBENCH_TOP_Z))

    selected_id = target_id or targets[0]["id"]
    if selected_id not in {item["id"] for item in targets}:
        raise ValueError("未找到目标 id: " + str(selected_id))

    # --- 0) 光源（纯视觉；缺省不注入，保持既有行为）---
    # 未声明光源时 MuJoCo 只有默认头灯，离屏相机图接近全黑（实测均值 16~24/255）；
    # 声明后由配置决定亮度，跨机型可移植（方向光与工作空间位置无关）。
    _injected_lights = inject_lights(world, scene_cfg.get("lights"))

    # --- 1) 工作台 ---
    ET.SubElement(
        world,
        "geom",
        name="workbench",
        type="box",
        pos="0 0 %.9f" % (top_z - float(workbench_cfg.get("half_thickness_m", 0.025))),
        size="%.6f %.6f %.6f" % (
            float(workbench_cfg.get("half_size_m", 0.8)),
            float(workbench_cfg.get("half_size_m", 0.8)),
            float(workbench_cfg.get("half_thickness_m", 0.025)),
        ),
        friction=workbench_cfg.get("friction", "1.0 0.02 0.001"),
        rgba="0.35 0.35 0.38 1",
        contype="1",
        conaffinity="1",
    )

    # --- 2) 目标方块（含独立材质，供按颜色区分）---
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    multi = len(targets) > 1
    geometry = []
    for item in targets:
        rgba = item["rgba"]
        material_name = item["id"] + "_material"
        ET.SubElement(asset, "material", name=material_name, rgba=_rgba_string(rgba))
        position = item["pos_m"]
        if position is None:
            raise ValueError(
                "目标 %s 缺少 pos_m；机器人无关路径要求显式声明位姿" % item["id"]
            )
        quat = item["quat_wxyz"]
        attrs = {
            "name": item["id"],
            "pos": " ".join("%.9f" % value for value in position),
            "quat": " ".join("%.9f" % value for value in quat),
        }
        # 只有"被抓取的那个"给重力补偿：抬升验收要求位移完全由双指摩擦
        # 承担，40g 方块的重力（0.39N）会让纯摩擦抬升过于临界。
        # 干扰目标必须正常受重力，否则被擦碰后会长期浮空。
        if (not multi) or item["id"] == selected_id:
            attrs["gravcomp"] = "1"
        body = ET.SubElement(world, "body", **attrs)
        ET.SubElement(body, "freejoint", name=item["id"] + "_free")
        ET.SubElement(
            body,
            "geom",
            name=item["id"] + "_geom",
            type="box",
            size="%.9f %.9f %.9f" % (half_size, half_size, half_size),
            mass=str(target_cfg.get("mass_kg", 0.04)),
            material=material_name,
            friction=target_cfg.get("friction", "2.0 0.05 0.001"),
        )
        geometry.append(
            {
                "id": item["id"],
                "body": item["id"],
                "geom": item["id"] + "_geom",
                "material": material_name,
                "rgba": rgba,
                "position_m": [round(float(v), 9) for v in position],
                "quaternion_wxyz": [round(float(v), 9) for v in quat],
                "euler_deg": [round(float(v), 6) for v in item["euler_deg"]],
            }
        )
    target_position = np.asarray(
        [item for item in geometry if item["id"] == selected_id][0]["position_m"],
        dtype=float,
    )

    # --- 3) 抓取锚点夹具（require_friction_lift=false 时使用）---
    if not bool(acceptance.get("require_friction_lift", False)):
        # 锚点体名可由 `grasp.lift_anchor_body` 声明（2026-09-30 §11.30）：联合世界里
        # **多台臂共用同一件场景级夹具**（mocap body）时，硬编码 `<目标>_lift_anchor` 会与
        # 先注入的那台冲突（同名不同 body1）。缺省仍是旧名 ⇒ 单臂场景行为不变。
        anchor_name = str((config.get("grasp") or {}).get("lift_anchor_body")
                          or (selected_id + "_lift_anchor"))
        anchor_pos = target_position.copy()
        ET.SubElement(
            world,
            "body",
            name=anchor_name,
            pos=" ".join("%.9f" % value for value in anchor_pos),
            mocap="true",
        )
        ET.SubElement(
            root.find("equality") if root.find("equality") is not None
            else ET.SubElement(root, "equality"),
            "weld",
            name=LIFT_CONSTRAINT_TEMPLATE % selected_id,
            body1=anchor_name,
            body2=selected_id,
            active="false",
            solref="0.02 1",
            solimp="0.9 0.95 0.001",
        )

    # --- 4) 固定相机 ---
    camera_cfg = scene_cfg.get("camera") or {}
    camera_pos = [float(v) for v in camera_cfg.get("pos_m", [-0.75, -0.55, 0.65])]
    camera_look_at = camera_cfg.get("look_at_m")
    if camera_look_at is None:
        camera_look_at = [float(v) for v in target_position]
    else:
        camera_look_at = [float(v) for v in camera_look_at]
    camera_quat = _look_at_quat(camera_pos, camera_look_at)
    ET.SubElement(
        world,
        "camera",
        name=camera_cfg.get("name", "overhead_camera"),
        pos=" ".join("%.9f" % v for v in camera_pos),
        quat=" ".join("%.9f" % v for v in camera_quat),
        fovy="%.6f" % float(camera_cfg.get("fovy_deg", 60.0)),
        mode="fixed",
    )

    # --- 5) 臂关节增益（可选注入）---
    # Piper 需要注入（官方模型缺阻尼）；UR5e 不需要（官方 <general> 自带
    # gainprm/biasprm 阻尼位置伺服）。默认不注入，避免覆盖官方标定。
    arm_kp = float(scene_cfg.get("arm_position_kp", 0.0))
    inject_gains = bool(scene_cfg.get("inject_arm_position_gains", False))
    if inject_gains:
        if arm_kp <= 0:
            raise ValueError("inject_arm_position_gains=true 时必须给正 arm_position_kp")
        actuator = root.find("actuator")
        if actuator is None:
            raise ValueError("机器人 MJCF 缺少 actuator 段")
        touched = 0
        for element in actuator:
            joint = element.get("joint")
            if joint in arm_joints:
                element.set("gainprm", "%.6f 0 0" % arm_kp)
                element.set("biasprm", "0 %.6f -400" % (-arm_kp))
                touched += 1
        if touched != len(arm_joints):
            raise ValueError(
                "臂关节增益注入不完整: %d/%d（配置的 arm_joints 与执行器不匹配）"
                % (touched, len(arm_joints))
            )

    # --- 6) 夹爪/接触配置 ---
    gripper_cfg = config.get("gripper") or {}
    open_positions = dict(gripper_cfg.get("open") or {})
    closed_positions = dict(gripper_cfg.get("closed") or {})
    if not open_positions or not closed_positions:
        raise ValueError("基线缺少 gripper.open / gripper.closed")
    if set(open_positions) != set(closed_positions):
        raise ValueError("gripper.open 与 gripper.closed 的键必须一致")
    # lift 段的臂位形优先用**求解出的** lift 参考姿态；只有参考姿态缺失时
    # 才回退到基线手写的 `gripper.lift`（Piper 的既有行为）。
    # 理由见 config/ur5_simulation_baseline.yaml 里对 `lift` 的说明：
    # 手写关节角可能与求解出的 grasp 落在不同的腕部解支上。
    if reference is not None and reference.get("lift"):
        lift_arm = {
            str(k): float(v)
            for k, v in reference["lift"]["joint_positions"].items()
        }
    else:
        lift_arm = {
            str(k): float(v) for k, v in (gripper_cfg.get("lift") or {}).items()
        }

    def merged(arm_part, gripper_part):
        result = {str(k): float(v) for k, v in arm_part.items()}
        result.update({str(k): float(v) for k, v in gripper_part.items()})
        return result

    # --- 6b) 把"关节名"翻译成"执行器名" ---
    # 后端 `_set_controls` 是按 **actuator 名** 索引 ctrl 的，而基线里的
    # `lift` / 参考姿态里的 `joint_positions` 用的是**关节名**。
    # Piper 恰好 actuator 名与关节名相同（joint1..joint8），因此既有的
    # Piper 基线直接写关节名就能工作；UR5e 的 actuator 名是
    # shoulder_pan / shoulder_lift / elbow / wrist_1..3（**不带 _joint 后缀**），
    # 直接沿用关节名会报 "actuator not found: shoulder_pan_joint"（实测）。
    #
    # 因此这里做一次显式翻译：对每个臂关节，找出它在 actuator 段里
    # 通过 `joint=` 指向它的执行器名；找不到就显式失败（铁律 5），
    # 不做命名推断。
    actuator = root.find("actuator")
    if actuator is None:
        raise ValueError("机器人 MJCF 缺少 actuator 段")
    joint_to_actuator = {}
    for element in actuator:
        target_joint = element.get("joint")
        if target_joint:
            joint_to_actuator[target_joint] = element.get("name") or target_joint

    def to_actuator_names(positions):
        """把关节名键翻译成执行器名键；已是执行器名的保持不变。"""
        translated = {}
        for key, value in positions.items():
            key = str(key)
            if key in joint_to_actuator:
                translated[joint_to_actuator[key]] = float(value)
            else:
                translated[key] = float(value)
        return translated

    bodies_cfg = model_cfg.get("bodies") or {}
    gripper = {
        "left_finger_body": str(bodies_cfg.get("left_finger", "")),
        "right_finger_body": str(bodies_cfg.get("right_finger", "")),
        "wrist_body": str(bodies_cfg.get("wrist", "")),
        "open_positions": to_actuator_names(merged({}, open_positions)),
        "closed_positions": to_actuator_names(merged({}, closed_positions)),
        "lift_positions": to_actuator_names(merged(lift_arm, closed_positions)),
        "min_lift_delta_m": float(acceptance.get("min_lift_delta_m", 0.02)),
        "min_normal_force_n": float(acceptance.get("min_normal_force_n", 0.2)),
        "max_force_imbalance_ratio": float(
            acceptance.get("max_force_imbalance_ratio", 4.0)
        ),
        "pad_offset_m": 0.0,
        "pad_offset_axis": [0.0, 0.0, 1.0],
        # 双指接触面 geom 名：后端算"抓取点"时必须用**接触面**而不是
        # 父 body 原点。Piper 的 link7/link8 body 原点与指腹网格接近，
        # 差异不明显；2F-85 的 pad body 原点在铰链附近，
        # 与 pad box 中心相差约 2cm，用 body 会让对齐门禁误报 94mm 偏差。
        "left_finger_geom": str(finger_geoms["left"]),
        "right_finger_geom": str(finger_geoms["right"]),
        # 夹持区定义（可选）：声明后**运行时对齐门禁**与 IK、参考姿态证据
        # 使用同一个"抓取点"（这些 geom 中心的算术平均）。不声明则门禁只用
        # 左右代表接触面 —— 两者相差一个固定几何量，必须三方同口径。
        **( {"pad_boxes": [str(name) for name in finger_geoms["pad_boxes"]]}
            if finger_geoms.get("pad_boxes") else {} ),
    }
    if not bool(acceptance.get("require_friction_lift", False)):
        gripper["lift_constraint"] = LIFT_CONSTRAINT_TEMPLATE % selected_id
        # 后端在抬升段要驱动该锚点跟随指腹中点（`_advance_with_grasp_anchor`）⇒ 名字必须进报告；
        # 与场景里注入的 body 名**同源**（声明 `grasp.lift_anchor_body`，缺省旧名）。
        gripper["lift_anchor_body"] = str((config.get("grasp") or {}).get("lift_anchor_body")
                                          or (selected_id + "_lift_anchor"))
        # 抬升段锚点的**驱动口径**与"结束时必须仍在夹口里"（I4-a 2026-09-30 §11.54，第 9 处缺口）：
        # 后端抬升段读的同样是**报告**里的这两个键；不转发 ⇒ 运行期退回旧口径（把载荷硬拽出夹口）。
        # 声明即消费：给出即校验取值（乱填必须显式失败），不给则**不写键**（旧口径逐位不变）。
        _anchor_mode = (config.get("grasp") or {}).get("lift_anchor_mode")
        if _anchor_mode is not None:
            if _anchor_mode not in LIFT_ANCHOR_MODES:
                raise ValueError("grasp.lift_anchor_mode 只允许 %s（实际 %r）"
                                 % (list(LIFT_ANCHOR_MODES), _anchor_mode))
            gripper["lift_anchor_mode"] = str(_anchor_mode)
        _require_contact_at_end = (config.get("grasp") or {}).get("require_contact_at_lift_end")
        if _require_contact_at_end is not None:
            if not isinstance(_require_contact_at_end, bool):
                raise ValueError("grasp.require_contact_at_lift_end 必须是布尔（实际 %r）"
                                 % (_require_contact_at_end,))
            gripper["require_contact_at_lift_end"] = bool(_require_contact_at_end)
    # 放置段声明**转发进报告**（I4-a 2026-09-30 §11.52，第 8 处同族缺口）：运行期 `place_object`
    # 读的是**报告**里的 `place_settle_ms` / `lift_gripper` / `place_pose_correction`，
    # 生成器不转发 ⇒ 运行期报「缺少 place_pose_correction 声明（grasp.place_pose_correction）：
    # 实现层不给默认值」。与 Piper 侧同结构：声明透传，**缺声明即失败**。
    _place_settle_ms = (config.get("grasp") or {}).get("place_settle_ms")
    if (not isinstance(_place_settle_ms, int) or isinstance(_place_settle_ms, bool)
            or _place_settle_ms <= 0):
        raise ValueError("grasp.place_settle_ms 必须是正整数（缺声明即失败，不给默认值；实际 %r）"
                         % (_place_settle_ms,))
    gripper["place_settle_ms"] = int(_place_settle_ms)
    _lift_gripper = (config.get("grasp") or {}).get("lift_gripper")
    if not isinstance(_lift_gripper, str) or not _lift_gripper:
        raise ValueError("grasp.lift_gripper 必须是非空字符串（缺声明即失败；实际 %r）"
                         % (_lift_gripper,))
    gripper["lift_gripper"] = str(_lift_gripper)
    _place_correction = (config.get("grasp") or {}).get("place_pose_correction")
    if not isinstance(_place_correction, dict) or not _place_correction:
        raise ValueError("grasp.place_pose_correction 必须声明为对象（含 mode；实现层不给默认值）")
    if str(_place_correction.get("mode") or "") not in PLACE_POSE_CORRECTION_MODES:
        raise ValueError("grasp.place_pose_correction.mode 必须是 %s，实际: %r"
                         % ("/".join(PLACE_POSE_CORRECTION_MODES), _place_correction.get("mode")))
    gripper["place_pose_correction"] = dict(_place_correction)
    # 抓取段的**运行期闭环纠偏**声明（2026-09-30 §11.29）：构建期的关节解按**标称目标**求，
    # 而联合世界里目标会被搬动（狗背托盘里的载荷：实测偏差 15.812 mm > 判据 5 mm）⇒ 运行期按
    # **实测**目标重解下压/抬升两段。声明只来自基线（`grasp.grasp_pose_correction`），
    # 实现层不给默认值；**缺声明 = 不纠偏**（行为与改动前一致）。
    _pick_correction = (config.get("grasp") or {}).get("grasp_pose_correction")
    if _pick_correction is not None:
        if not isinstance(_pick_correction, dict) or not _pick_correction:
            raise ValueError("grasp.grasp_pose_correction 必须声明为对象（含 mode；实现层不给默认值）")
        _mode = str(_pick_correction.get("mode") or "")
        if _mode not in GRASP_POSE_CORRECTION_MODES:
            raise ValueError("grasp.grasp_pose_correction.mode 必须是 %s，实际: %r"
                             % ("/".join(GRASP_POSE_CORRECTION_MODES), _pick_correction.get("mode")))
        # 这两个键**任何模式都要有**（`measure_only` 也要判"是否超容差/超上限"）
        for _key in ("residual_tolerance_m", "max_correction_m"):
            _value = _pick_correction.get(_key)
            if not isinstance(_value, (int, float)) or isinstance(_value, bool) or not float(_value) > 0:
                raise ValueError("grasp_pose_correction 必须声明正的 %s（实现层不写默认值），实际: %r"
                                 % (_key, _value))
        if _mode == "resolved":
            for _key in ("ik_iterations", "ik_step", "ik_tolerance_m", "max_axis_deg",
                         "align_max_attempts"):
                _value = _pick_correction.get(_key)
                if (not isinstance(_value, (int, float)) or isinstance(_value, bool)
                        or not float(_value) > 0):
                    raise ValueError(
                        "grasp_pose_correction.mode=resolved 时必须声明正的 %s（实现层不写默认值），"
                        "实际: %r" % (_key, _value))
            # 下压段子段数（2026-09-30 §11.56）：可选；给出即必须是 ≥1 的整数（缺省 1 = 单段）。
            _splits = _pick_correction.get("descend_splits")
            if _splits is not None:
                if (not isinstance(_splits, int) or isinstance(_splits, bool) or _splits < 1):
                    raise ValueError(
                        "grasp_pose_correction.descend_splits 必须是 ≥1 的整数（实际 %r）："
                        "子段数决定下压段重解次数" % (_splits,))
        gripper["grasp_pose_correction"] = dict(_pick_correction)
    # 搬运段抓取约束（2026-09-30 §11.55）：**镜像 Piper 侧**的声明块形状（同一 equality/anchor），
    # 差异只在数值与 `release_gripper`。缺失 ⇒ **不写键**（放置段退回"只看双侧指腹接触"，
    # 即本臂此前的行为）；给出 ⇒ 逐项校验（乱填必须显式失败）。
    _carry = (config.get("grasp") or {}).get("carry_constraint") or {}
    if _carry:
        if not isinstance(_carry, dict):
            raise ValueError("grasp.carry_constraint 必须是对象")
        _carry_type = str(_carry.get("type") or "")
        if _carry_type not in ("connect", "weld"):
            raise ValueError("grasp.carry_constraint.type 必须是 connect 或 weld（实际 %r）："
                             "搬运要不要约束旋转必须由声明决定" % _carry_type)
        _carry_equality = str(_carry.get("equality_name") or "")
        _carry_anchor = str(_carry.get("anchor_body") or "")
        if bool(_carry.get("enabled", False)) and not (_carry_equality and _carry_anchor):
            raise ValueError("grasp.carry_constraint.enabled=true 时必须声明 equality_name 与 anchor_body")
        _carry_solref = _carry.get("solref")
        _carry_solimp = _carry.get("solimp")
        if (not isinstance(_carry_solref, (list, tuple)) or len(_carry_solref) != 2
                or not isinstance(_carry_solimp, (list, tuple)) or len(_carry_solimp) != 3):
            raise ValueError("grasp.carry_constraint 必须声明 solref（2 个数）与 solimp（3 个数）："
                             "实测 solref 0.01 太软会让载荷在 xy 漂移约 3 cm")
        _carry_steps = _carry.get("max_plant_steps_per_iteration")
        if (not isinstance(_carry_steps, int) or isinstance(_carry_steps, bool)
                or _carry_steps <= 0):
            raise ValueError("grasp.carry_constraint 必须声明正的 max_plant_steps_per_iteration")
        _carry_release = bool(_carry.get("release_gripper", False))
        _carry_slip = _carry.get("max_slip_m")
        # `max_slip_m` 只在**张爪搬运**（release_gripper=true）时才被运行期消费 ⇒ 只在那种情形要求它，
        # 否则就是"声明了却不被消费"（AGENTS.md 6.2）。
        if _carry_release and not (isinstance(_carry_slip, (int, float))
                                   and not isinstance(_carry_slip, bool) and float(_carry_slip) > 0):
            raise ValueError("grasp.carry_constraint.release_gripper=true 时必须声明正的 max_slip_m"
                             "（张爪后不能看指腹接触，只能用焊缝滑移阈值）")
        if not _carry_release and _carry_slip is not None:
            raise ValueError("grasp.carry_constraint.release_gripper=false 时不应声明 max_slip_m"
                             "（运行期不消费它；判据是双侧指腹接触）—— 要么删掉，要么改 release_gripper")
        gripper["carry_constraint"] = {
            "enabled": bool(_carry.get("enabled", False)),
            "type": _carry_type,
            "equality_name": _carry_equality,
            "anchor_body": _carry_anchor,
            "solref": [float(v) for v in _carry_solref],
            "solimp": [float(v) for v in _carry_solimp],
            "release_gripper": _carry_release,
            "max_slip_m": (float(_carry_slip) if isinstance(_carry_slip, (int, float))
                           and not isinstance(_carry_slip, bool) else 0.0),
            "max_plant_steps_per_iteration": int(_carry_steps),
            "source": "ur5_simulation_baseline.yaml:grasp.carry_constraint",
        }
    if reference is not None:
        gripper["pad_offset_m"] = float(reference.get("finger_height_correction_m", 0.0))
        gripper["pad_offset_axis"] = [
            float(v) for v in reference.get("approach_direction_world", [0.0, 0.0, 1.0])
        ]
        gripper["home_positions"] = to_actuator_names(
            merged(reference["home"], open_positions)
        )
        gripper["approach_positions"] = to_actuator_names(
            merged(reference["approach"]["joint_positions"], open_positions)
        )
        gripper["grasp_positions"] = to_actuator_names(
            merged(reference["grasp"]["joint_positions"], open_positions)
        )
        # --- 重力前馈（逐段 ctrl 增量，ctrl 通道名键）---
        # 纯 PD 执行器（UR5e 官方模型没有真机控制器自带的重力补偿）在重力矩
        # 不为零的位形下存在稳态位置误差 Δq = τ_g / gain，实测 shoulder_lift
        # 约 0.015 rad ≈ 末端 15mm，超过 5mm 抓取容差。
        # 参考姿态求解器按 model 的 qfrc_bias 与 gainprm[0] 算出逐姿态的
        # ctrl 增量（键为**关节名**），这里翻译成 ctrl 通道名交给后端。
        # 键必须与该段位置指令一一对应：错位会让前馈静默失效
        # （表现为"精度莫名不达标"），因此显式校验、不做兜底。
        feedforward = {}
        for phase, positions_key in (
            ("home", "home_positions"),
            ("approach", "approach_positions"),
            ("grasp", "grasp_positions"),
            ("lift", "lift_positions"),
        ):
            declared = (reference.get("gravity_feedforward") or {}).get(phase)
            if not declared:
                continue
            offsets = to_actuator_names(declared)
            unknown = sorted(set(offsets) - set(gripper[positions_key]))
            if unknown:
                raise ValueError(
                    "重力前馈 %s 含该段位置指令里不存在的 ctrl 通道: %s"
                    % (phase, unknown)
                )
            feedforward[phase] = offsets
        gripper["gravity_feedforward"] = feedforward

    # --- 7) 指腹摩擦：接触摩擦决定纯摩擦抬升能否成立 ---
    pad_friction = scene_cfg.get("pad_friction")
    finger_geoms_set = {finger_geoms["left"], finger_geoms["right"]}
    if pad_friction:
        for geom in root.iter("geom"):
            if geom.get("name") in finger_geoms_set:
                geom.set("friction", pad_friction)

    # --- 8) 关键帧维度补齐 ---
    # 注入自由关节（目标方块）后 nq 变大，模型自带的关键帧（如 UR5e 的
    # `home` 只覆盖 6 个臂关节）会因维度不符而报
    # "keyframe 0: invalid qpos size, expected length 21"。
    # 做法：摘掉 keyframe 先编译一次拿到真实 nq/nu，再按维度补 0 放回；
    # 补 0 的语义是"新增自由度处于零位"，对自由关节即"位于原点"，
    # 与 keyframe 只描述机械臂位形的原意一致。
    keyframe_element = root.find("keyframe")
    saved_keys = []
    if keyframe_element is not None:
        import copy as _copy

        saved_keys = [_copy.deepcopy(key) for key in list(keyframe_element)]
        root.remove(keyframe_element)
        ET.indent(tree, space="    ")
        tree.write(str(output), encoding="unicode", xml_declaration=True)
        probe = mujoco.MjModel.from_xml_path(str(output))
        nq, nu = int(probe.nq), int(probe.nu)
        rebuilt = ET.SubElement(root, "keyframe")
        for key in saved_keys:
            qpos = (key.get("qpos") or "").split()
            ctrl = (key.get("ctrl") or "").split()
            if qpos:
                if len(qpos) > nq:
                    raise ValueError(
                        "关键帧 qpos 超出模型维度: %d > %d" % (len(qpos), nq)
                    )
                key.set("qpos", " ".join(qpos + ["0"] * (nq - len(qpos))))
            if ctrl:
                if len(ctrl) > nu:
                    raise ValueError(
                        "关键帧 ctrl 超出模型维度: %d > %d" % (len(ctrl), nu)
                    )
                key.set("ctrl", " ".join(ctrl + ["0"] * (nu - len(ctrl))))
            rebuilt.append(key)

    # --- 9) 覆盖 keyframe[0] 为抓取起始位形（reference["home"]）---
    # **为什么必须这样做（实测结论）**：
    # 后端 pick_object 的 HOME_HOLD 段从**模型当前位形**出发做插值。
    # 若模型停在 UR5e 的全零位（手臂竖直向上、夹爪水平朝 -y），
    # 插值到 HOME 的路径会让 pad 一度降到 z=-0.005（穿过工作台），
    # 沿途把方块顶到 z=5.3m（probe_home_transit.py 实测 t=0.33 处 pad_z=-0.0047）。
    # 把 keyframe[0] 设成 HOME 位形后，验收脚本用 key_qpos[0] 初始化，
    # 机械臂**一开始就停在 HOME**，HOME_HOLD 段退化为原地保持，
    # 不存在跨台面扫掠。
    if reference is not None and reference.get("home"):
        key_elements = list(root.findall("keyframe/key"))
        if not key_elements:
            raise ValueError("模型缺少 keyframe，无法写入起始位形")
        first = key_elements[0]
        # 按关节名逐个写入；未列出的自由度保持 0（自由关节即在原点）。
        qpos_parts = ["0"] * nq
        for name, value in reference["home"].items():
            joint_id = mujoco.mj_name2id(
                probe, mujoco.mjtObj.mjOBJ_JOINT, str(name)
            )
            if joint_id < 0:
                continue
            qpos_parts[int(probe.jnt_qposadr[joint_id])] = "%.9f" % float(value)
        # **目标的自由关节也必须写进关键帧**：后端用
        # `mj_resetDataKeyframe` 初始化，它会把**所有**自由度按关键帧赋值；
        # 关键帧里目标段若为 0，方块会被搬回世界原点，
        # 后端随即报 "抓取位姿与目标位置不一致: distance=0.567m"（实测）。
        for item in geometry:
            joint_id = mujoco.mj_name2id(
                probe, mujoco.mjtObj.mjOBJ_JOINT, item["id"] + "_free"
            )
            if joint_id < 0:
                continue
            adr = int(probe.jnt_qposadr[joint_id])
            position = item["position_m"]
            quaternion = item["quaternion_wxyz"]
            for offset, value in enumerate(list(position) + list(quaternion)):
                qpos_parts[adr + offset] = "%.9f" % float(value)
        first.set("qpos", " ".join(qpos_parts))
        # ctrl 也要对齐：否则首步 ctrl=0 会把臂往零位拉。
        # 注意 ctrl 通道按**执行器名**索引，而 reference["home"] 的键是
        # 关节名；两者在 Piper 上同名、在 UR5 上不同名，
        # 因此这里用与 gripper 段同一套 joint→actuator 翻译。
        home_ctrl = to_actuator_names(
            {str(k): float(v) for k, v in reference["home"].items()}
        )
        ctrl_parts = ["0"] * nu
        for index in range(nu):
            actuator_name = mujoco.mj_id2name(
                probe, mujoco.mjtObj.mjOBJ_ACTUATOR, index
            )
            if actuator_name in home_ctrl:
                ctrl_parts[index] = "%.9f" % float(home_ctrl[actuator_name])
        first.set("ctrl", " ".join(ctrl_parts))

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="    ")
    tree.write(str(output), encoding="unicode", xml_declaration=True)

    report = {
        "schema_version": "iraf.robot-pick-scene/v1",
        "source": str(source),
        "output": str(output),
        "target_id": selected_id,
        "target_position": {
            key: round(float(target_position[index]), 9)
            for index, key in enumerate(("x", "y", "z"))
        },
        "target_half_size_m": float(half_size),
        "targets": geometry,
        "multi_target": multi,
        "workbench_top_z_m": top_z,
        "pose_tolerance_m": float(acceptance.get("pose_tolerance_m", 0.005)),
        "gravity_fixture": True,
        "require_friction_lift": bool(acceptance.get("require_friction_lift", False)),
        "arm_position_kp": arm_kp,
        "arm_gains_injected": inject_gains,
        "finger_geoms": finger_geoms,
        "arm_joints": arm_joints,
        "gripper": gripper,
        # 视觉 Provider 声明（可选，来自基线配置的 vision 段）。
        # 未声明＝本机型未接入视觉；visual_pick 会要求请求显式给出 vision_file。
        "vision": config.get("vision"),
    }
    output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--target-id", default=None)
    parser.add_argument("--target-half-size", type=float, default=0.025)
    args = parser.parse_args(argv)

    config = yaml.safe_load(args.baseline.read_text(encoding="utf-8"))
    scene = build_scene(
        Path((config.get("model") or {}).get("source", "")),
        args.output,
        target_id=args.target_id,
        half_size=args.target_half_size,
        config=config,
    )
    print(json.dumps(scene, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
