"""按场景声明生成可加载的 MJCF：厂商文件只读，相机/雷达/托盘/地形/光照由本模块注入。

为什么单独成模块（而不是写进 `scripts/build_scene.py`）：
场景生成要在验收里被逐项断言（"注入后模型里能按名字找到每个声明对象"），
因此实现必须是可导入的库；`scripts/build_scene.py` 只做参数解析与退出码映射
（与 08/09/10 的"shell/CLI 入口 + 实现层"同一分层惯例）。

职责边界：

- 本模块只做"声明 → 模型"的机械变换，**不含任何机型专有名称**：
  机型差异来自 `profiles/<robot>_mujoco.yaml` 的 `spec.model.*`（躯干 body、挂载参考系、
  厂商模型路径），场景差异来自 `scenes/<id>/scene.yaml`（地形/道具/传感器/光照）。
- 厂商 MJCF **只读**：先用 SHA-256 与 `vendor/<vendor>/source-lock.json` 对账，
  再在内存里改写并写到 `build/` 下的产物（AGENTS.md 5.5）；任何情况下都不写回 `vendor/`。
- 未知即失败：引用不到的 body / site / frame / 传感器锚点、未锁定的厂商资产、
  声明缺字段，一律显式报错并给出中文原因；不静默跳过、不给默认值（铁律 1.5/2.2）。

报告契约：沿用既有场景报告 `iraf.robot-pick-scene/v1` 的键集合（`target_id` / `gripper` /
`vision` / `workbench_top_z_m` 等），保证 `scripts/emit_backend_config.py` 仍能消费同一份报告
（不新增第二套 schema）；本模块**新增** `report_kind` / `sensors` / `injections` / `model_facts`
四段，并新增 `scene` / `robot` / `vendor_source` 三段的来源信息。
注意：本机型无夹爪与抓取技能，`target_id` 与 `gripper` 显式为 `null`，
运行期装配仍以 Piper 抓取场景报告为准 —— 不得据此把四足场景当作操作场景。
"""

import copy
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import jsonschema
import mujoco
import numpy as np
import yaml

from iraf_core.profile import ProfileError, load_robot_profile
from iraf_adapters.mujoco.scene_lighting import inject_lights

# 退出码（由 scripts/build_scene.py 映射；库内错误自带 code，便于验收逐条核对）。
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_VENDOR_LOCK = 4
EXIT_MODEL = 5

REPORT_SCHEMA_VERSION = "iraf.robot-pick-scene/v1"
REPORT_KIND = "scene_model"

#: `spec.model.initial_alignment.geometry_scope` 支持的取值：只支持可碰撞几何
#: （"最低点"的物理含义是"能触地的点"；把视觉网格算进来会得到没有物理意义的抬升量）。
GEOM_SCOPES = ("collision",)


class SceneBuildError(ValueError):
    """场景生成失败（显式失败，不降级、不伪造成功）。"""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


def _fail(code, message):
    raise SceneBuildError(code, message)


def repo_root():
    # src/iraf_adapters/unitree/scene_builder.py -> 仓库根
    return Path(__file__).resolve().parents[3]


def _read_yaml(path, label):
    path = Path(path)
    if not path.is_file():
        _fail(EXIT_REFERENCE, "%s 不存在: %s" % (label, path))
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        _fail(EXIT_DECLARATION, "%s 解析失败: %s（%s）" % (label, path, exc))


def load_scene(scene_dir, root=None):
    """读取并校验 `scenes/<id>/scene.yaml`（复用 config/scene.schema.json，不新增契约）。"""
    root = Path(root or repo_root())
    scene_dir = Path(scene_dir)
    if not scene_dir.is_absolute():
        scene_dir = root / scene_dir
    if not scene_dir.is_dir():
        _fail(EXIT_REFERENCE, "场景目录不存在: " + str(scene_dir))
    scene_path = scene_dir / "scene.yaml"
    scene = _read_yaml(scene_path, "场景声明")
    schema_path = root / "config" / "scene.schema.json"
    if not schema_path.is_file():
        _fail(EXIT_REFERENCE, "场景契约 schema 不存在: " + str(schema_path))
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    errors = sorted(
        jsonschema.Draft7Validator(schema).iter_errors(scene),
        key=lambda error: list(error.absolute_path),
    )
    if errors:
        reasons = []
        for error in errors[:10]:
            location = "/".join(str(item) for item in error.absolute_path) or "<root>"
            reasons.append("%s: %s" % (location, error.message))
        _fail(
            EXIT_DECLARATION,
            "场景声明不符合 %s：%s" % (schema_path.name, "；".join(reasons)),
        )
    if str(scene.get("id")) != scene_dir.name:
        _fail(
            EXIT_DECLARATION,
            "scene.id=%r 与目录名 %r 不一致（场景包自校验）" % (scene.get("id"), scene_dir.name),
        )
    return scene_path, scene


def load_scene_baseline(scene_dir):
    """读取同包的 baseline.yaml（可选）；用于把场景级验收阈值带进报告。"""
    path = Path(scene_dir) / "baseline.yaml"
    if not path.is_file():
        return None, None
    return path, _read_yaml(path, "场景基线")


def resolve_robot(scene, robot):
    """按 id 解析场景本体；找不到即显式失败并列出候选。"""
    robots = [item for item in (scene.get("robots") or []) if isinstance(item, dict)]
    ids = [str(item.get("id")) for item in robots]
    if str(robot) not in ids:
        _fail(
            EXIT_REFERENCE,
            "场景 %s 里没有本体 %r；已声明的本体: %s"
            % (scene.get("id"), robot, sorted(ids)),
        )
    return next(item for item in robots if str(item.get("id")) == str(robot))


def resolve_profile(root, entity, robot):
    """解析本体 Profile 路径。

    两条**显式**来源，都不做"文件存在就用"的隐式回退：

    1. `scene.robots[].profile` 是路径 ⇒ 用它（并要求 profile 名称与本体 id 一致）；
    2. 是结构化占位（尚未闭合的能力声明）⇒ 按**声明身份**在 `profiles/*.mujoco.yaml` 中
       查找 `metadata.name == <robot>` 的 Profile，0 个或多个命中都显式失败。

    第 2 条存在的原因：`scene.robots[].capabilities` 必须为空（能力未验收），
    而 scene.schema.json 的门禁规定"已交付 profile ⇒ 至少一项能力"，
    因此能力未验收前 robots[].profile 必须保持占位；模型声明不因此缺失，
    只是来源不同 —— 报告里的 `profile_source` 会写明到底走了哪条，绝不静默。
    """
    root = Path(root)
    declared = entity.get("profile")
    if isinstance(declared, str):
        path = root / declared
        if not path.is_file():
            _fail(EXIT_REFERENCE, "robots.%s.profile 引用的文件不存在: %s" % (robot, declared))
        document = _read_yaml(path, "本体 Profile")
        name = str((document.get("metadata") or {}).get("name") or "")
        # 场景 id 与 Profile 身份可以不同名，但**必须在声明里显式写明映射**（`profile_name`）：
        # 例：场景 id `piper` ↔ Profile `piper_mujoco`（历史命名）。仍然是硬校验（值必须等于
        # Profile 的真实 metadata.name），只是把"允许不同名"这件事摆到声明里，不做隐式放行。
        declared_name = entity.get("profile_name")
        if declared_name is not None:
            if str(declared_name) != name:
                _fail(EXIT_DECLARATION,
                      "robots.%s.profile_name=%s 与 Profile 的 metadata.name=%s 不一致"
                      % (robot, declared_name, name))
            return path, "scene.robots[].profile"
        if name != str(robot):
            _fail(
                EXIT_DECLARATION,
                "robots.%s.profile 指向的 Profile 名称是 %r（必须与本体 id 一致）" % (robot, name),
            )
        return path, "scene.robots[].profile"

    profiles_dir = root / "profiles"
    if not profiles_dir.is_dir():
        _fail(EXIT_REFERENCE, "profiles/ 目录不存在: " + str(profiles_dir))
    matches = []
    for candidate in sorted(profiles_dir.glob("*_mujoco.yaml")):
        document = _read_yaml(candidate, "本体 Profile")
        if str((document.get("metadata") or {}).get("name") or "") == str(robot):
            matches.append(candidate)
    if not matches:
        _fail(
            EXIT_REFERENCE,
            "本体 %s 的 Profile 未交付：profiles/ 下没有 metadata.name=%s 的 *_mujoco.yaml"
            % (robot, robot),
        )
    if len(matches) > 1:
        _fail(
            EXIT_DECLARATION,
            "本体 %s 匹配到多个 Profile（声明身份不唯一）: %s"
            % (robot, [str(item) for item in matches]),
        )
    return matches[0], "declared_identity_lookup"


def load_build_declarations(profile_path, robot):
    """从 Profile 读构建期声明 `spec.model.*`（缺失即失败，不给默认值）。"""
    document = _read_yaml(profile_path, "本体 Profile")
    if document.get("kind") != "RobotProfile":
        _fail(EXIT_DECLARATION, "%s 不是 RobotProfile（kind=%r）" % (profile_path, document.get("kind")))
    # 用核心校验器确认它真的是一份合法 Profile（关节/限位/频率），
    # 避免"看起来像 Profile 的文件"绕过契约（能力可以为空，那是事实不是漏写）。
    try:
        load_robot_profile(profile_path)
    except ProfileError as exc:
        _fail(EXIT_DECLARATION, "Profile 未通过核心校验: %s（%s）" % (exc, profile_path))
    spec = document.get("spec") or {}
    model = spec.get("model")
    if not isinstance(model, dict):
        _fail(
            EXIT_DECLARATION,
            "本体 %s 的 Profile 缺少 spec.model（厂商模型路径/躯干 body/挂载参考系必须声明）" % robot,
        )
    for key in ("vendor", "file", "trunk_body"):
        if not model.get(key):
            _fail(EXIT_DECLARATION, "Profile 的 spec.model 缺少必需字段: " + key)
    frames = model.get("mount_frames")
    if frames is not None and not isinstance(frames, dict):
        _fail(EXIT_DECLARATION, "spec.model.mount_frames 必须是对象")
    return model, spec


def verify_vendor_source(root, model, label):
    """厂商资产锁定校验：文件必须在锁内登记，且 SHA-256 与锁一致（字节级 provenance）。"""
    root = Path(root)
    vendor = str(model["vendor"])
    relative = str(model["file"])
    prefix = "vendor/%s/" % vendor
    if not relative.startswith(prefix):
        _fail(
            EXIT_REFERENCE,
            "%s 的厂商模型路径 %r 不在 vendor/%s/ 下（跨目录引用会绕过锁定校验）" % (label, relative, vendor),
        )
    source = root / relative
    if not source.is_file():
        _fail(EXIT_REFERENCE, "%s 的厂商模型不存在: %s" % (label, relative))
    lock_path = root / "vendor" / vendor / "source-lock.json"
    if not lock_path.is_file():
        _fail(EXIT_REFERENCE, "厂商锁文件不存在: " + str(lock_path))
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    schema = str(lock.get("schema_version"))
    inner = relative[len(prefix):]
    # 厂商锁的**两套既有约定**（显式支持，不做"字段像就收"的隐式兼容；锁里必须写清 provenance）：
    #   · `iraf.vendor-source-lock/v1`（厂商 U1）：`files[].path` 相对 `vendor/<vendor>/`
    #   · `iraf.piper-model-source-lock/v1`（机械臂侧）：`files[].relative_path` 相对**模型目录名**，
    #     且 `files[].path` 是**本机绝对路径**（⇒ 该锁是 host-specific，报告里如实标注）
    if schema == "iraf.vendor-source-lock/v1":
        key, entries = "path", [item for item in (lock.get("files") or [])
                                if str(item.get("path")) == inner]
        lock_meta = {"lock_schema": schema, "path_key": "path", "host_specific": False}
    elif schema == "iraf.piper-model-source-lock/v1":
        model_dir = inner.split("/", 1)[0]
        key = "relative_path"
        entries = [item for item in (lock.get("files") or [])
                   if "%s/%s" % (model_dir, str(item.get("relative_path"))) == inner]
        lock_meta = {
            "lock_schema": schema, "path_key": "relative_path", "host_specific": True,
            "note": "该锁的 files[].path 为本机绝对路径（资产目录是符号链接，未真正入库）"
                    "⇒ 只在**本机**可复现；迁移到 U1 约定需要把资产实体入库并重写锁",
        }
    else:
        _fail(EXIT_DECLARATION,
              "厂商锁 schema_version 非法（只接受 iraf.vendor-source-lock/v1 或 "
              "iraf.piper-model-source-lock/v1）: " + schema)
    if not entries:
        _fail(
            EXIT_VENDOR_LOCK,
            "厂商资产未在锁内登记: %s（先更新 source-lock.json 与许可证 BOM）" % relative,
        )
    import hashlib

    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    locked = str(entries[0].get("sha256"))
    if digest != locked:
        _fail(
            EXIT_VENDOR_LOCK,
            "厂商文件哈希与锁不一致: %s（当前 %s，锁内 %s）" % (relative, digest, locked),
        )
    return {
        "path": relative,
        "sha256": digest,
        "locked_sha256": locked,
        "blob_sha1": entries[0].get("blob_sha1"),
        **lock_meta,
        "lock": str(lock_path.relative_to(root)),
        "match": True,
    }


def _vec3(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        _fail(EXIT_DECLARATION, "%s 必须是 3 个数值" % label)
    values = [float(item) for item in value]
    if not all(np.isfinite(item) for item in values):
        _fail(EXIT_DECLARATION, "%s 必须是有限数" % label)
    return values


def _quat_wxyz(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        _fail(EXIT_DECLARATION, "%s 必须是 4 个数值（wxyz）" % label)
    return [float(item) for item in value]


def _numbers(values, fmt):
    return " ".join(fmt % float(item) for item in values)


def _look_at_quat(camera_pos, look_at):
    """MuJoCo 相机四元数：光轴（-Z）指向注视点，+Y 尽量朝上。"""
    position = np.asarray(camera_pos, dtype=float)
    target = np.asarray(look_at, dtype=float)
    forward = target - position
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        _fail(EXIT_DECLARATION, "相机位置与注视点重合，无法确定朝向")
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
    return [float(item) for item in quat]


def _relative_path(target, start):
    target = Path(target).resolve()
    start = Path(start).resolve()
    try:
        return os.path.relpath(str(target), str(start)).replace(os.sep, "/")
    except ValueError:
        return str(target).replace(os.sep, "/")


def _find_body(world, name):
    for body in world.iter("body"):
        if body.get("name") == name:
            return body
    return None


def _inject_workbench(world, terrain):
    workbench = terrain.get("workbench") or {}
    top_z = float(workbench["top_z_m"])
    half_thickness = float(workbench["half_thickness_m"])
    half_size = float(workbench["half_size_m"])
    ET.SubElement(
        world,
        "geom",
        name="workbench",
        type="box",
        pos="0 0 %.9f" % (top_z - half_thickness),
        size="%.6f %.6f %.6f" % (half_size, half_size, half_thickness),
        friction=str(workbench["friction"]),
        rgba="0.35 0.35 0.38 1",
        contype="1",
        conaffinity="1",
    )
    return {
        "workbench": {
            "top_z_m": top_z,
            "half_size_m": half_size,
            "half_thickness_m": half_thickness,
            "friction": str(workbench["friction"]),
        }
    }


def _prop_geometry_attributes(geometry, label):
    kind = str(geometry.get("type"))
    if kind not in ("box", "sphere", "cylinder", "mesh"):
        _fail(EXIT_DECLARATION, "%s 的几何类型未支持: %s" % (label, kind))
    attributes = {"type": kind}
    if kind == "mesh":
        attributes["mesh"] = "prop_mesh_%s" % label.split(".")[1]
    else:
        size = geometry.get("size_m")
        if not isinstance(size, (list, tuple)) or not size:
            _fail(EXIT_DECLARATION, "%s.size_m 必须是非空数组" % label)
        attributes["size"] = _numbers(size, "%.9f")
    return attributes


def _inject_props(
    root, world, scene, robot, model, trunk_body_name, trunk_body_element, root_dir, output_dir
):
    """注入道具：绝对位姿的落 worldbody（带自由关节），挂载位姿的挂到声明参考系上。"""
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    injected = []
    frames = (model.get("mount_frames") or {})
    for prop in scene.get("props") or []:
        prop_id = str(prop["id"])
        body_name = str(prop["body"])
        label = "props.%s" % prop_id
        material_name = prop_id + "_material"
        rgba = [float(item) for item in prop["rgba"]]
        ET.SubElement(
            asset,
            "material",
            name=material_name,
            rgba=_numbers(rgba, "%.6f"),
        )
        geometry = prop["geometry"]
        if geometry.get("type") == "mesh":
            # 路径按**生效的**输出目录计算（`--output` 覆盖声明时也要正确）。
            mesh_path = _relative_path(root_dir / str(geometry["mesh"]), output_dir)
            ET.SubElement(asset, "mesh", name="prop_mesh_%s" % prop_id, file=mesh_path)
        attributes = _prop_geometry_attributes(geometry, label)
        pose = prop["pose"]
        record = {
            "id": prop_id,
            "body": body_name,
            "kind": str(prop["kind"]),
            "material": material_name,
            "rgba": rgba,
            "mass_kg": float(prop["mass_kg"]),
            "friction": str(prop["friction"]),
            "geometry": {"type": str(geometry.get("type"))},
        }
        if "mount" in pose:
            mount = pose["mount"]
            frame_name = str(mount["frame"])
            entity = str(mount["entity"])
            if entity != str(robot):
                _fail(
                    EXIT_REFERENCE,
                    "%s.pose.mount.entity=%s 不是本次构建的本体 %s（跨本体挂载需要单独构建）"
                    % (label, entity, robot),
                )
            if frame_name not in frames:
                _fail(
                    EXIT_REFERENCE,
                    "%s.pose.mount.frame=%s 在 Profile spec.model.mount_frames 里未声明" % (label, frame_name),
                )
            declaration = frames[frame_name]
            frame_body = str(declaration.get("body") or "")
            if frame_body != trunk_body_name:
                _fail(
                    EXIT_REFERENCE,
                    "参考系 %s 声明挂在 body %s 上，与 Profile 的 trunk_body=%s 不一致"
                    % (frame_name, frame_body, trunk_body_name),
                )
            pos = _vec3(declaration.get("pos_m"), "mount_frames.%s.pos_m" % frame_name)
            quat = _quat_wxyz(declaration.get("quat_wxyz"), "mount_frames.%s.quat_wxyz" % frame_name)
            body = ET.SubElement(
                trunk_body_element,
                "body",
                name=body_name,
                pos=_numbers(pos, "%.9f"),
                quat=_numbers(quat, "%.9f"),
            )
            ET.SubElement(
                body,
                "geom",
                name=body_name + "_geom",
                mass="%.9f" % float(prop["mass_kg"]),
                material=material_name,
                friction=str(prop["friction"]),
                **attributes,
            )
            record["mount"] = {"frame": frame_name, "entity": entity, "body": body_name}
            record["pose_source"] = "mount_frame"
            record["position_m"] = pos
        else:
            position = _vec3(pose["pos_m"], "%s.pose.pos_m" % label)
            quaternion = _quat_wxyz(pose["quat_wxyz"], "%s.pose.quat_wxyz" % label)
            body = ET.SubElement(
                world,
                "body",
                name=body_name,
                pos=_numbers(position, "%.9f"),
                quat=_numbers(quaternion, "%.9f"),
            )
            ET.SubElement(body, "freejoint", name=body_name + "_free")
            ET.SubElement(
                body,
                "geom",
                name=body_name + "_geom",
                mass="%.9f" % float(prop["mass_kg"]),
                material=material_name,
                friction=str(prop["friction"]),
                **attributes,
            )
            record["pose_source"] = "world"
            record["position_m"] = position
            record["quaternion_wxyz"] = quaternion
        injected.append(record)
    return injected, frames


def _attach_robots(staging, scene, root, attached_ids):
    """把**附加本体**合成进主模型（`MjSpec.attach(child, prefix, frame)`，库原生合成）。

    为什么用 MjSpec 而不是拼 XML：`attach` 由 MuJoCo 自己完成 asset/actuator/joint 引用的改写，
    手写名字前缀重写极易漏引用（mesh/material/actuator 的 joint 名）。
    关键帧：合成后 qpos 维度变长 ⇒ 用**附加本体 Profile 声明的 home** 按关节名补齐（**不猜值**：
    缺声明即显式失败），并把补齐细节记进报告。
    """
    import mujoco

    staging_parsed = ET.parse(str(staging)).getroot()
    key_qpos_by_index = {}
    for parsed_key in staging_parsed.find("keyframe") or []:
        if parsed_key.tag == "key" and parsed_key.get("qpos"):
            key_qpos_by_index[len(key_qpos_by_index)] = [
                float(v) for v in parsed_key.get("qpos").split()]
    spec = mujoco.MjSpec.from_file(str(staging))
    records = []
    attached_robot_values = []
    for robot_id in attached_ids:
        entity = resolve_robot(scene, robot_id)
        profile_path, _source = resolve_profile(root, entity, robot_id)
        model, profile_spec = load_build_declarations(profile_path, robot_id)
        verify_vendor_source(root, model, "spec.model")
        placement = entity.get("placement")
        if not isinstance(placement, dict):
            _fail(EXIT_REFERENCE,
                  "robots.%s 缺少 placement（附加本体的相对位姿必须来自声明，不得由构建器假定）"
                  % robot_id)
        pos = _vec3(placement.get("pos_m"), "robots.%s.placement.pos_m" % robot_id)
        quat = _quat_wxyz(placement.get("quat_wxyz"), "robots.%s.placement.quat_wxyz" % robot_id)
        child_path = root / str(model["file"])
        child = mujoco.MjSpec.from_file(str(child_path))
        child_dir = child_path.resolve().parent
        # 关节初值来自**声明的**两个来源（缺键即显式失败，不猜）：
        #   · `spec.home`：本体标称位形（臂的 joint1..6）
        #   · `spec.gripper.open_positions`：抓手的"张开"位形（臂的 joint7/8 在这段里声明）
        home = dict(profile_spec.get("home") or {})
        gripper = profile_spec.get("gripper")
        if isinstance(gripper, dict):
            home.update({str(k): float(v) for k, v in (gripper.get("open_positions") or {}).items()})
        joint_values = []
        for joint in child.joints:
            name = str(joint.name)
            kind = str(joint.type).upper()
            if "FREE" in kind:
                joint_values.extend([0.0] * 7)
                continue
            if "BALL" in kind:
                joint_values.extend([0.0] * 4)
                continue
            if name not in home:
                _fail(EXIT_DECLARATION,
                      "附加本体 %s 的关节 %s 未在 Profile spec.model.home 声明初值："
                      "联合模型的关键帧不猜值" % (robot_id, name))
            joint_values.append(float(home[name]))
        prefix = "%s_" % robot_id
        # MjSpec 需要**数值列表**（`_numbers()` 是给 XML 属性用的字符串，别混用）
        frame = spec.worldbody.add_frame(pos=[float(v) for v in pos],
                                        quat=[float(v) for v in quat])
        spec.attach(child, prefix=prefix, frame=frame)

        attached_robot_values.append(list(joint_values))
        records.append({"id": robot_id, "prefix": prefix, "source": str(model["file"]),
                        "placement": {"pos_m": [float(v) for v in pos],
                                      "quat_wxyz": [float(v) for v in quat]},
                        "child_dir": str(child_dir),
                        "home_joints": len(joint_values),
                        "keyframes_extended": len(list(getattr(spec, "keys", []) or []))})
    # ---- 关键帧：**覆写**为「主模型关键帧 + 各附加本体的声明初值」（所有本体附加完成后统一做）
    # ⚠ 两处实测教训：① `attach()` 可能自行扩展主模型的关键帧 ⇒ 必须覆写而不是追加；
    # ② 收集附加初值的顺序必须在覆写**之前**（我第一版把覆写放在循环内、用还没收集完的列表
    #    ⇒ 实际写回 26 而非 34，报 `keyframe 0: invalid qpos size, expected length 34`）。
    if attached_robot_values:
        flat = [value for values in attached_robot_values for value in values]
        base_qpos = [float(v) for v in key_qpos_by_index.get(0, [])]
        for index, key in enumerate(list(getattr(spec, "keys", []) or [])):
            base = key_qpos_by_index.get(index, base_qpos)
            key.qpos = list(base) + flat

    # ---- 资产自包含（A1）：两个本体的 meshdir 不同 ⇒ 合成后相对路径互指（实测
    # `.../piper_description/mujoco_model/../../../vendor/unitree_go2/.../base_link.STL` 不存在）。
    # 做法：把每个附加本体引用的 mesh/texture 拷进 `<产物目录>/assets/<本体 id>/`，
    # 并把合成 spec 里的资产路径改写为**相对产物目录**的路径 ⇒ 产物自包含、不写死本机绝对路径。
    output_dir = Path(staging).parent
    # ⚠ MuJoCo 把资产路径解析为 **`compiler.meshdir` + file**（实测：改写后的相对路径会被拼到
    # 子模型的 meshdir 后面 ⇒ `.../go2/assets/assets/piper/base_link.STL`）⇒ 必须把合成 spec 的
    # meshdir 指向**产物目录**，file 才按"相对产物目录"解析。
    # ⚠ MjSpec 的 compiler 没有 `meshdir` 可写属性（实测 AttributeError）⇒ 保持主模型声明的 meshdir，
    # 把搬过来的资产按**相对 meshdir 的路径**写（与主模型自己的资产同一套约定 ⇒ 产物仍可移植）。
    staging_root = ET.parse(str(staging)).getroot()
    compiler = staging_root.find("compiler")
    meshdir = compiler.get("meshdir") if compiler is not None else None
    meshdir_abs = (output_dir / str(meshdir)).resolve() if meshdir else output_dir.resolve()
    copied = 0
    for item in spec.meshes:
        path = str(getattr(item, "file", "") or "")
        if not path or path.startswith("assets/"):
            continue
        for record in records:
            candidate = Path(record["child_dir"]) / path
            if candidate.is_file():
                target_rel = Path("assets") / str(record["id"]) / Path(path).name
                target = output_dir / target_rel
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(candidate, target)
                # ⚠ `to_xml()` 仍按**附加子模型的 meshdir** 解析这些路径（实测：写成相对路径会被
                # 拼到 `.../piper_description/mujoco_model/` 后面）⇒ 先落**绝对路径**并登记为
                # host-specific 债务（自包含的相对表达受阻于 MjSpec 的 per-attach meshdir 行为）。
                item.file = str(target.resolve())
                copied += 1
                break
    for item in getattr(spec, "textures", []):
        path = str(getattr(item, "file", "") or "")
        if not path or path.startswith("assets/"):
            continue
        for record in records:
            candidate = Path(record["child_dir"]) / path
            if candidate.is_file():
                target_rel = Path("assets") / str(record["id"]) / Path(path).name
                target = output_dir / target_rel
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(candidate, target)
                item.file = str(target.resolve())
                copied += 1
                break
    for record in records:
        record.pop("child_dir", None)
        record["assets_copied"] = copied
        record["assets_path_style"] = "absolute_host_path"
        record["assets_note"] = ("合成产物的资产用绝对路径（MjSpec 的 per-attach meshdir 使两个本体的"
                                 "资产无法用单一相对 meshdir 表达）⇒ 该产物是 **host-specific**；"
                                 "迁到相对表达需进一步研究 MjSpec 的资产搬迁语义")
    Path(staging).write_text(spec.to_xml(), encoding="utf-8")
    return records


def _inject_world_frames(world, scene):
    """按声明注入**世界固定**参考系（site 直接挂 `worldbody`）—— 导航/停靠/交接的目标帧。

    为什么需要这一类（2026-09-24 实测，docs/debug/2026-09-24-dock-target-self-frame.md）：
    停靠的目标帧必须世界固定且**不与机器人刚性相连**，否则量到的是自指量：
      · `mount_frames` 注入在**躯干**上 ⇒ 与机身同一刚体：偏航误差恒 `0.0`（同一个 `xmat`）、
        平移误差恒为「帧本地偏置在机身姿态下的 xy 投影」，接近过程**从不执行**；
      · prop `body` 是**自由体**（每个 prop 都带自由关节）⇒ 会被机器人推走：实测站距 0.19 m 时
        目标被推 ~35 mm（占末态误差 45%）。
    世界固定 site 无质量、无自由关节、不参与碰撞 ⇒ 既不会被推，也不占台面。
    """
    frames = scene.get("frames") or []
    existing = {element.get("name") for element in world.iter("*") if element.get("name")}
    injected = []
    for index, frame in enumerate(frames):
        label = "frames[%d]" % index
        kind = frame.get("kind")
        if kind != "site":
            _fail(EXIT_DECLARATION, "%s.kind 只支持 site（世界固定帧的唯一形态），实际 %r" % (label, kind))
        name = str(frame.get("id"))
        if name in existing:
            _fail(EXIT_DECLARATION, "%s.id=%s 与场景里既有对象（body/site/camera）重名" % (label, name))
        pose = frame.get("pose")
        if not isinstance(pose, dict):
            _fail(EXIT_DECLARATION, "%s.pose 必须是对象（世界系 pos_m + quat_wxyz）" % label)
        pos = _vec3(pose.get("pos_m"), "%s.pose.pos_m" % label)
        quat = _quat_wxyz(pose.get("quat_wxyz"), "%s.pose.quat_wxyz" % label)
        ET.SubElement(world, "site", name=name, pos=_numbers(pos, "%.9f"),
                      quat=_numbers(quat, "%.9f"))
        existing.add(name)
        injected.append({"id": name, "kind": "site", "injected_as": "world_site",
                         "pose_source": "declaration", "pos_m": list(pos), "quat_wxyz": list(quat)})
    return injected


def _inject_mount_frames(trunk_body, model, used_frames):
    """把所有被引用的挂载参考系作为 site 注入躯干（供位姿查询与验收判据使用）。"""
    frames = model.get("mount_frames") or {}
    injected = []
    for name in sorted(used_frames):
        declaration = frames[name]
        ET.SubElement(
            trunk_body,
            "site",
            name=name,
            pos=_numbers(_vec3(declaration.get("pos_m"), "mount_frames.%s.pos_m" % name), "%.9f"),
            quat=_numbers(
                _quat_wxyz(declaration.get("quat_wxyz"), "mount_frames.%s.quat_wxyz" % name), "%.9f"
            ),
        )
        injected.append(name)
    return injected


def _inject_sensors(scene, robot, trunk_body, trunk_body_name, world, vendor_site_names):
    """按声明注入相机/雷达；`source=vendor` 的只做存在性校验（厂商文件只读）。"""
    injected = []
    vendor_referenced = []
    deferred = []
    for sensor in scene.get("sensors") or []:
        sensor_id = str(sensor["id"])
        kind = str(sensor["kind"])
        source = str(sensor["source"])
        anchor = sensor["anchor"]
        object_kind = str(anchor["object_kind"])
        anchor_name = str(anchor["name"])
        entity = str(anchor["entity"])
        label = "sensors.%s" % sensor_id
        if source == "vendor":
            if object_kind != "site" or anchor_name not in vendor_site_names:
                _fail(
                    EXIT_REFERENCE,
                    "%s 声明 source=vendor 但厂商模型里没有 %s(%s)（厂商资产被改动？）"
                    % (label, object_kind, anchor_name),
                )
            vendor_referenced.append(
                {
                    "id": sensor_id,
                    "kind": kind,
                    "object_kind": object_kind,
                    "name": anchor_name,
                    "entity": entity,
                    "rate_hz": float(sensor["rate_hz"]),
                    "injected": False,
                    "note": "厂商 MJCF 自带，场景构建器只读引用，不改写厂商文件。",
                }
            )
            continue
        if source != "scene":
            _fail(EXIT_DECLARATION, "%s.source 非法: %s" % (label, source))
        if entity != str(robot):
            if object_kind == "camera":
                # 相机按绝对位姿注入 world（本仓库既有语义：相机是固定世界相机，
                # anchor.entity 只登记归属，不代表挂在本体的 body 上）。
                pass
            else:
                _fail(
                    EXIT_REFERENCE,
                    "%s.anchor.entity=%s 不是本次构建的本体 %s，无法把 %s 挂到它的几何上"
                    % (label, entity, robot, object_kind),
                )
        if kind == "camera":
            if object_kind != "camera":
                _fail(EXIT_DECLARATION, "%s(kind=camera) 的 anchor.object_kind 必须是 camera" % label)
            pos = _vec3(sensor["pos_m"], "%s.pos_m" % label)
            look_at = _vec3(sensor["look_at_m"], "%s.look_at_m" % label)
            quat = _look_at_quat(pos, look_at)
            ET.SubElement(
                world,
                "camera",
                name=anchor_name,
                pos=_numbers(pos, "%.9f"),
                quat=_numbers(quat, "%.9f"),
                fovy="%.6f" % float(sensor["fovy_deg"]),
                mode="fixed",
            )
            injected.append(
                {
                    "id": sensor_id,
                    "kind": kind,
                    "object_kind": "camera",
                    "name": anchor_name,
                    "entity": entity,
                    "injected_as": "world_camera",
                    "pos_m": pos,
                    "look_at_m": look_at,
                    "fovy_deg": float(sensor["fovy_deg"]),
                    "note": "世界固定相机：位姿是场景绝对坐标；anchor.entity 登记归属，不表示挂在 body 上。",
                }
            )
        elif kind == "lidar":
            if object_kind != "site":
                _fail(EXIT_DECLARATION, "%s(kind=lidar) 的 anchor.object_kind 必须是 site" % label)
            height = float(sensor["mount_height_m"])
            ET.SubElement(
                trunk_body,
                "site",
                name=anchor_name,
                pos="0 0 %.9f" % height,
                quat="1 0 0 0",
            )
            injected.append(
                {
                    "id": sensor_id,
                    "kind": kind,
                    "object_kind": "site",
                    "name": anchor_name,
                    "entity": entity,
                    "injected_as": "trunk_site",
                    "parent_body": trunk_body_name,
                    "mount_height_m": height,
                    "num_rays": int(sensor["num_rays"]),
                    "range_m": float(sensor["range_m"]),
                    "mujoco_sensor": None,
                    "note": "MuJoCo 无雷达传感器类型：本步骤只注入可挂载的 site 与扫描契约"
                    "（射线数/量程由运行期按声明做光线投射，点云统计属步骤 14）。",
                }
            )
        else:
            _fail(
                EXIT_DECLARATION,
                "%s 的 kind=%s 不能由场景构建器注入（scene 只支持 camera/lidar）" % (label, kind),
            )
    return injected, vendor_referenced, deferred


def _fix_keyframe_dimensions(xml_root, tree, path, free_props):
    """把厂商关键帧补齐到注入自由关节后的维度，并写回**声明位姿**。

    实测症状（本步骤首跑）：注入 `box_01` 的自由关节后 nq 19 → 26，厂商
    `<keyframe><key name="home" .../></keyframe>` 只有 19 个 qpos，编译即报
    `keyframe 0: invalid qpos size, expected length 26`。

    做法与既有抓取场景生成器同源：先摘掉 keyframe 编译一次拿到真实 nq/nu，
    再按维度补 0 放回。**新注入的自由道具必须写回声明位姿** ——
    运行期用 `mj_resetDataKeyframe` 初始化时会给所有自由度赋值，
    补 0 会把道具搬到世界原点（既有生成器的实测结论）。
    厂商模型若本来没有 keyframe，本函数**不新增**（不替厂商决定初始位形）。
    """
    keyframe_element = xml_root.find("keyframe")
    saved = []
    if keyframe_element is not None:
        saved = [copy.deepcopy(key) for key in list(keyframe_element)]
        xml_root.remove(keyframe_element)
    ET.indent(tree, space="    ")
    tree.write(str(path), encoding="unicode", xml_declaration=True)
    probe = mujoco.MjModel.from_xml_path(str(path))
    nq, nu = int(probe.nq), int(probe.nu)
    if not saved:
        return
    rebuilt = ET.SubElement(xml_root, "keyframe")
    for key in saved:
        qpos = (key.get("qpos") or "").split()
        ctrl = (key.get("ctrl") or "").split()
        if qpos:
            if len(qpos) > nq:
                _fail(EXIT_MODEL, "关键帧 qpos 超出模型维度: %d > %d" % (len(qpos), nq))
            qpos = qpos + ["0"] * (nq - len(qpos))
            for prop in free_props:
                joint_name = prop["body"] + "_free"
                joint_id = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id < 0:
                    _fail(EXIT_MODEL, "注入的自由关节在模型里找不到: " + joint_name)
                address = int(probe.jnt_qposadr[joint_id])
                values = list(prop["position_m"]) + list(prop["quaternion_wxyz"])
                for offset, value in enumerate(values):
                    qpos[address + offset] = "%.9f" % float(value)
            key.set("qpos", " ".join(qpos))
        if ctrl:
            if len(ctrl) > nu:
                _fail(EXIT_MODEL, "关键帧 ctrl 超出模型维度: %d > %d" % (len(ctrl), nu))
            key.set("ctrl", " ".join(ctrl + ["0"] * (nu - len(ctrl))))
        rebuilt.append(key)
    ET.indent(tree, space="    ")
    tree.write(str(path), encoding="unicode", xml_declaration=True)


def _dig_scene_path(document, dotted, label):
    """按点号路径取场景声明里的量；解析不到即显式失败（不允许默认值兜底）。"""
    node = document
    for part in str(dotted).split("."):
        if not isinstance(node, dict) or part not in node:
            _fail(
                EXIT_DECLARATION,
                "%s 的路径 %r 在场景声明里解析不到（在 %r 处断开）" % (label, dotted, part),
            )
        node = node[part]
    return node


def _subtree_body_ids(model, mujoco, root_body_id):
    """root body 及其全部子孙 body 的 id 集合（按 body_parentid 走，不假设命名）。"""
    ids = {int(root_body_id)}
    changed = True
    while changed:
        changed = False
        for index in range(int(model.nbody)):
            if index in ids:
                continue
            if int(model.body_parentid[index]) in ids:
                ids.add(index)
                changed = True
    return ids


def _lowest_geom_z(model, data, mujoco, body_ids, scope):
    """给定 body 集合内所有 geom 的**最低点**世界 z（按几何类型精确算，不用包围球近似）。

    返回 `(最低点 z, 取到最低点的 geom 名)`。类型不在支持表内即显式失败 —— 宁可报错，也不要拿
    包围球半径充当"最低点"（那会把抬升量算错，而且错得看不出来，属"静默错误结果"）。

    `scope`：量哪些 geom。只支持 `collision`（`contype/conaffinity` 非零的可碰撞几何）——
    "最低点"的物理含义是"能触地的点"，视觉网格不参与接触，把它们算进来会得到没有物理意义的抬升量
    （本步实测：把视觉网格用包围球下界算进最低点，抬升量从 18.372 mm 变成 96.406 mm）。

    mesh 口径（**实测**，见 build/iraf-24h-2/02/diagnose_mesh_scale.py）：MuJoCo 把 mesh geom 的
    `geom_size` 存成网格本地的半边长（实测 `FL_calf/calf_0`：size.z=0.157580154 == |本地极值 z|），
    顶点数据不做缩放 ⇒ 世界坐标 = `geom_xpos + geom_xmat @ v`。用 `pos − rbound` 当最低点会低估
    （`calf_0`：−0.096406 m vs 真实最低点 +0.000502 m），是本次首跑抬升 96 mm 的成因。
    """
    kind = mujoco.mjtGeom
    sphere = int(kind.mjGEOM_SPHERE)
    box = int(kind.mjGEOM_BOX)
    capsule = int(kind.mjGEOM_CAPSULE)
    cylinder = int(kind.mjGEOM_CYLINDER)
    mesh = int(kind.mjGEOM_MESH)
    ellipsoid = int(getattr(kind, "mjGEOM_ELLIPSOID", -1))
    lowest = None
    lowest_name = ""
    lowest_type = ""
    if scope != "collision":
        _fail(
            EXIT_DECLARATION,
            "initial_alignment.geometry_scope 只支持 collision（可碰撞几何），实际: %r" % (scope,),
        )
    for gid in range(int(model.ngeom)):
        if int(model.geom_bodyid[gid]) not in body_ids:
            continue
        if scope == "collision" and not (
            int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid])
        ):
            continue
        position = np.asarray(data.geom_xpos[gid], dtype=float)
        size = np.asarray(model.geom_size[gid], dtype=float)
        rotation = np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3)
        gtype = int(model.geom_type[gid])
        if gtype == sphere:
            z = float(position[2]) - float(size[0])
        elif gtype == box:
            z = float(position[2]) - float(
                abs(rotation[2, 0]) * size[0]
                + abs(rotation[2, 1]) * size[1]
                + abs(rotation[2, 2]) * size[2]
            )
        elif gtype in (capsule, cylinder):
            # 胶囊/圆柱的轴是 geom 局部 z：最低点 = 中心 − (|R[2,2]|·半长 + 半径)。
            z = float(position[2]) - float(abs(rotation[2, 2]) * size[1] + size[0])
        elif gtype == mesh:
            mesh_id = int(model.geom_dataid[gid])
            if mesh_id < 0:
                _fail(EXIT_MODEL, "geom %d 是 mesh 但没有网格数据" % gid)
            start = int(model.mesh_vertadr[mesh_id])
            count = int(model.mesh_vertnum[mesh_id])
            if count <= 0:
                _fail(EXIT_MODEL, "geom %d 的网格没有顶点" % gid)
            verts = np.asarray(model.mesh_vert[start : start + count], dtype=float)
            # 每个顶点在世界系下的 z = geom 原点 z + R 的第三行 · 顶点（顶点不做缩放）。
            z = float(np.min(float(position[2]) + verts.dot(rotation[2, :])))
        elif gtype == ellipsoid:
            # 椭球最低点没有解析式（半轴长度 = size、局部轴为 geom 的坐标轴）：
            # 用 |R[2,·]|·size 的保守下界（只会多抬，不会少抬），并在报告里写明用的是下界。
            z = float(position[2]) - float(np.linalg.norm(size[:3]))
        else:
            _fail(
                EXIT_MODEL,
                "无法计算 geom %d（type=%d）的最低点：初始位姿对齐不支持该几何类型，不做近似"
                % (gid, gtype),
            )
        if lowest is None or z < float(lowest):
            lowest = float(z)
            lowest_name = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ("geom#%d" % gid)
            )
            lowest_type = str(mujoco.mjtGeom(gtype)).split(".")[-1]
    if lowest is None:
        _fail(
            EXIT_MODEL,
            "本体子树内没有任何 %s geom：无法做初始位姿对齐" % scope,
        )
    return float(lowest), lowest_name, lowest_type


def _align_initial_pose(xml_root, tree, path, model_decl, scene, trunk_body_name):
    """A′ ①（步骤 02）：按**实测**把初始位姿抬到「本体最低几何点刚好落在支撑面」。

    为什么必须显式声明（`spec.model.initial_alignment`）：这是**改变初始条件**的构建期行为，
    隐式施加会让任何"看起来像 Profile"的文件都能改物理初始态；缺本段即不做（其它本体不受影响）。

    实测背景：关键帧 `home` 下四足足端球最低点在台面下 18.372 mm
    （build/iraf-24h-2/02/probe-neutral-clearance.txt）。本函数只做**一次平移**（基座自由关节的 z），
    不动关节角、不动姿态；抬升量由编译后的模型实测得出，不是写死的常数。
    **首期只处理厂商自带关键帧**：厂商没有关键帧时不新增（不替厂商决定初始位形）。
    """
    section = model_decl.get("initial_alignment")
    if section is None:
        return None
    if not isinstance(section, dict):
        _fail(EXIT_DECLARATION, "spec.model.initial_alignment 必须是映射")
    enabled = section.get("enabled")
    if not isinstance(enabled, bool):
        _fail(EXIT_DECLARATION, "spec.model.initial_alignment.enabled 必须显式给 true/false")
    if not enabled:
        return {"enabled": False, "note": "声明为 false：不做初始位姿对齐"}
    mode = str(section.get("mode", ""))
    if mode != "lift_lowest_geom_to_support_plane":
        _fail(
            EXIT_DECLARATION,
            "spec.model.initial_alignment.mode 只支持 lift_lowest_geom_to_support_plane，"
            "实际: %r（其余语义未实现，不做近似）" % (section.get("mode"),),
        )
    source = section.get("support_plane_source")
    if not source:
        _fail(
            EXIT_DECLARATION,
            "spec.model.initial_alignment.support_plane_source 缺失（支撑面 z 的来源路径必须声明）",
        )
    tolerance = section.get("tolerance_m")
    if (
        not isinstance(tolerance, (int, float))
        or isinstance(tolerance, bool)
        or not np.isfinite(float(tolerance))
        or float(tolerance) <= 0.0
    ):
        _fail(EXIT_DECLARATION, "spec.model.initial_alignment.tolerance_m 必须是正数")
    tolerance = float(tolerance)
    scope = str(section.get("geometry_scope", ""))
    if scope not in GEOM_SCOPES:
        _fail(
            EXIT_DECLARATION,
            "spec.model.initial_alignment.geometry_scope 只支持 %s，实际: %r"
            % (list(GEOM_SCOPES), section.get("geometry_scope")),
        )
    support_z = float(
        _dig_scene_path(scene, source, "spec.model.initial_alignment.support_plane_source")
    )

    keyframe_element = xml_root.find("keyframe")
    if keyframe_element is None:
        return {
            "enabled": True,
            "mode": mode,
            "applied": False,
            "reason": "厂商模型没有 keyframe：不新增（不替厂商决定初始位形）",
            "support_plane_z_m": support_z,
            "geometry_scope": scope,
        }

    probe = mujoco.MjModel.from_xml_path(str(path))
    trunk_id = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, trunk_body_name)
    if trunk_id < 0:
        _fail(EXIT_REFERENCE, "初始位姿对齐：模型里没有躯干 body " + str(trunk_body_name))
    body_ids = _subtree_body_ids(probe, mujoco, trunk_id)
    base_joint = -1
    for index in range(int(probe.njnt)):
        if (
            int(probe.jnt_type[index]) == int(mujoco.mjtJoint.mjJNT_FREE)
            and int(probe.jnt_bodyid[index]) == int(trunk_id)
        ):
            base_joint = index
            break
    if base_joint < 0:
        _fail(
            EXIT_REFERENCE,
            "初始位姿对齐：躯干 body %s 没有自由关节，无法抬升初始位姿" % trunk_body_name,
        )
    address = int(probe.jnt_qposadr[base_joint])
    data = mujoco.MjData(probe)

    entries = []
    for key in list(keyframe_element):
        name = str(key.get("name"))
        qpos = (key.get("qpos") or "").split()
        if len(qpos) < address + 3:
            _fail(
                EXIT_MODEL,
                "关键帧 %r 的 qpos 长度 %d 不足以容纳基座自由关节（需要 ≥ %d）"
                % (name, len(qpos), address + 3),
            )
        key_id = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_KEY, name)
        if key_id < 0:
            _fail(EXIT_MODEL, "关键帧 %r 在编译后的模型里找不到" % name)
        mujoco.mj_resetDataKeyframe(probe, data, key_id)
        mujoco.mj_forward(probe, data)
        lowest, geom_name, geom_type = _lowest_geom_z(probe, data, mujoco, body_ids, scope)
        base_before = float(data.qpos[address + 2])
        lift = support_z - lowest
        entries.append(
            {
                "name": name,
                "lowest_z_before_m": lowest,
                "lowest_geom": geom_name,
                "lowest_geom_type": geom_type,
                "base_z_before_m": base_before,
                "lift_m": lift,
                "base_z_after_m": base_before + lift,
            }
        )
        qpos[address + 2] = "%.9f" % (base_before + lift)
        key.set("qpos", " ".join(qpos))

    ET.indent(tree, space="    ")
    tree.write(str(path), encoding="unicode", xml_declaration=True)

    # 后置校验：重新编译，逐关键帧复核最低点与支撑面的残差（抬升量必须真的生效）。
    verify_model = mujoco.MjModel.from_xml_path(str(path))
    verify_data = mujoco.MjData(verify_model)
    verify_trunk = mujoco.mj_name2id(verify_model, mujoco.mjtObj.mjOBJ_BODY, trunk_body_name)
    verify_ids = _subtree_body_ids(verify_model, mujoco, verify_trunk)
    residuals = []
    for entry in entries:
        key_id = mujoco.mj_name2id(verify_model, mujoco.mjtObj.mjOBJ_KEY, entry["name"])
        mujoco.mj_resetDataKeyframe(verify_model, verify_data, key_id)
        mujoco.mj_forward(verify_model, verify_data)
        lowest, geom_name, geom_type = _lowest_geom_z(
            verify_model, verify_data, mujoco, verify_ids, scope
        )
        residual = lowest - support_z
        entry["residual_after_m"] = residual
        entry["lowest_geom_after"] = geom_name
        entry["lowest_geom_type_after"] = geom_type
        residuals.append(abs(residual))
        if abs(residual) > tolerance:
            _fail(
                EXIT_MODEL,
                "初始位姿对齐未收敛：关键帧 %r 的最低点残差 %.9f m 超出容差 %.9f m"
                % (entry["name"], residual, tolerance),
            )
    return {
        "enabled": True,
        "mode": mode,
        "applied": True,
        "support_plane_z_m": support_z,
        "support_plane_source": str(source),
        "geometry_scope": scope,
        "tolerance_m": tolerance,
        "trunk_body": trunk_body_name,
        "base_joint_qpos_address": address,
        "keyframes": entries,
        "max_abs_residual_m": max(residuals),
        "method": "编译后模型 + 关键帧 + mj_forward 实测最低点，单次平移基座 z",
    }


def _model_facts(path):
    """编译生成模型并回报名字表（注入是否真的生效以编译结果为准）。"""
    model = mujoco.MjModel.from_xml_path(str(path))

    def names(objtype, count):
        return [
            mujoco.mj_id2name(model, objtype, index) or ""
            for index in range(count)
            if mujoco.mj_id2name(model, objtype, index)
        ]

    return model, {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nbody": int(model.nbody),
        "ncam": int(model.ncam),
        "nsite": int(model.nsite),
        "nsensor": int(model.nsensor),
        "njnt": int(model.njnt),
        "timestep_s": float(model.opt.timestep),
        "gravity": [float(item) for item in model.opt.gravity],
        "bodies": names(mujoco.mjtObj.mjOBJ_BODY, model.nbody),
        "cameras": names(mujoco.mjtObj.mjOBJ_CAMERA, model.ncam),
        "sites": names(mujoco.mjtObj.mjOBJ_SITE, model.nsite),
        "joints": names(mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
        "actuators": names(mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
    }


def build_scene_model(scene_dir, robot, root=None, output=None, attach=()):
    """按声明生成模型与报告；返回报告字典（不打印、不退出，便于单测直接断言）。"""
    root = Path(root or repo_root())
    scene_path, scene = load_scene(scene_dir, root)
    scene_dir = scene_path.parent
    baseline_path, baseline = load_scene_baseline(scene_dir)
    entity = resolve_robot(scene, robot)
    profile_path, profile_source = resolve_profile(root, entity, robot)
    model, profile_spec = load_build_declarations(profile_path, robot)

    vendor = verify_vendor_source(root, model, "spec.model")

    source = root / str(model["file"])
    declared_output = output or scene["model"]["output"]
    output_path = Path(declared_output)
    if not output_path.is_absolute():
        output_path = root / output_path
    trunk_body_name = str(model["trunk_body"])

    tree = ET.parse(str(source))
    xml_root = tree.getroot()
    world = xml_root.find("worldbody")
    if world is None:
        _fail(EXIT_REFERENCE, "厂商 MJCF 缺少 worldbody: " + str(source))
    # meshdir 是相对路径：输出目录与源模型不同目录，必须改写成"从输出目录指向源 assets"，
    # 否则会报 "Error opening file '<output_dir>/assets/...'"（既有生成器的实测结论）。
    compiler = xml_root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(xml_root, "compiler")
    declared_meshdir = compiler.get("meshdir")
    if declared_meshdir:
        declared = Path(declared_meshdir)
        if not declared.is_absolute():
            declared = (source.parent / declared).resolve()
        compiler.set("meshdir", _relative_path(declared, output_path.parent))

    trunk_body = _find_body(world, trunk_body_name)
    if trunk_body is None:
        _fail(
            EXIT_REFERENCE,
            "厂商模型里没有躯干 body %s（spec.model.trunk_body 与模型不一致）" % trunk_body_name,
        )
    vendor_site_names = {site.get("name") for site in xml_root.iter("site")}
    if str(model.get("imu_site")) not in vendor_site_names:
        _fail(
            EXIT_REFERENCE,
            "厂商模型里没有 IMU site %s（spec.model.imu_site 与模型不一致）" % model.get("imu_site"),
        )

    lights = inject_lights(world, scene.get("lights"))
    injections = {"lights": list(lights)}
    injections.update(_inject_workbench(world, scene["terrain"]))
    props, frames = _inject_props(
        xml_root,
        world,
        scene,
        robot,
        model,
        trunk_body_name,
        trunk_body,
        root,
        output_path.parent,
    )
    injections["props"] = props
    injections["mount_frames"] = _inject_mount_frames(trunk_body, model, {item["mount"]["frame"] for item in props if "mount" in item})
    # 世界固定帧：必须在 mount_frames 之后注入（重名检查要能看见已注入的挂载参考系）
    injections["world_frames"] = _inject_world_frames(world, scene)
    injected_sensors, vendor_sensors, deferred_sensors = _inject_sensors(
        scene, robot, trunk_body, trunk_body_name, world, vendor_site_names
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    staging = output_path.with_name(output_path.stem + ".staging.xml")
    _fix_keyframe_dimensions(
        xml_root, tree, staging, [item for item in props if "quaternion_wxyz" in item]
    )
    # 初始位姿对齐（A′ ①）：按实测把本体最低几何点抬到支撑面。缺声明即不做（返回 None）。
    initial_alignment = _align_initial_pose(
        xml_root, tree, staging, model, scene, trunk_body_name
    )
    # 联合模型：把声明的附加本体合成进主模型（只在 `--attach` 时发生；单本体产物逐位不变）
    attached_robots = []
    if attach:
        attached_robots = _attach_robots(staging, scene, root, list(attach))
    injections["attached_robots"] = attached_robots
    try:
        compiled, facts = _model_facts(staging)
    except Exception as exc:  # MuJoCo 编译失败即显式失败，不落半成品
        _fail(EXIT_MODEL, "生成模型无法编译: %s（%s）" % (exc, staging))
    # 单测口径：注入后模型里必须能按名字找到每个声明对象（以编译结果为准，不看 XML 文本）。
    failures = []
    for item in injected_sensors:
        if item["name"] not in facts["sites"] and item["name"] not in facts["cameras"]:
            failures.append("传感器 %s 的注入对象 %s 不在生成模型里" % (item["id"], item["name"]))
    for item in props:
        if item["body"] not in facts["bodies"]:
            failures.append("道具 %s 的 body %s 不在生成模型里" % (item["id"], item["body"]))
    for name in injections["mount_frames"]:
        if name not in facts["sites"]:
            failures.append("挂载参考系 %s 不在生成模型里" % name)
    for item in injections["world_frames"]:
        if item["id"] not in facts["sites"]:
            failures.append("世界固定帧 %s 不在生成模型里" % item["id"])
    for item in attached_robots:
        # 附加本体的**根 body** 必须按前缀出现在合成模型里（前缀由 attach 保证唯一）
        if item["prefix"] not in " ".join(facts["bodies"]) and not any(
                name.startswith(item["prefix"]) for name in facts["bodies"]):
            failures.append("附加本体 %s 的 body 未按前缀 %s 出现在合成模型里"
                            % (item["id"], item["prefix"]))
    if trunk_body_name not in facts["bodies"]:
        failures.append("躯干 body %s 不在生成模型里" % trunk_body_name)
    if failures:
        _fail(EXIT_MODEL, "注入未生效：" + "；".join(failures))
    os.replace(str(staging), str(output_path))

    report = {
        # 沿用既有场景报告契约（键集合不变，新增字段见下），emit_backend_config.py 可直接消费。
        "schema_version": REPORT_SCHEMA_VERSION,
        "report_kind": REPORT_KIND,
        "source": str(source),
        "output": str(output_path),
        "scene": {
            "id": str(scene["id"]),
            "path": str(scene_path.relative_to(root)),
            "baseline": str(baseline_path.relative_to(root)) if baseline_path else None,
            "robot": str(robot),
            "simulation": True,
        },
        "robot": {
            "id": str(robot),
            "kind": str(entity["kind"]),
            "role": str(entity["role"]),
            "capabilities": [str(item) for item in (entity.get("capabilities") or [])],
            "profile": str(profile_path.relative_to(root)),
            "profile_source": profile_source,
            "profile_reference_in_scene": entity.get("profile"),
            "trunk_body": trunk_body_name,
        },
        # 本机型无夹爪/抓取技能：以下四个键显式为 null/空，保持键集合兼容，
        # 但**不得**据此装配操作后端（运行期以 Piper 抓取场景报告为准）。
        "target_id": None,
        "targets": [],
        "gripper": None,
        "vision": None,
        "manipulation_absent_reason": (
            "本机机型为四足（kind=%s），无夹爪与抓取技能：target_id/gripper/vision 为 null，"
            "manipulation 段不参与运行期装配。" % entity["kind"]
        ),
        "gravity_fixture": False,
        "require_friction_lift": False,
        "arm_joints": [],
        "finger_geoms": None,
        "workbench_top_z_m": injections["workbench"]["top_z_m"],
        "acceptance": (baseline or {}).get("acceptance"),
        "simulation": True,
        "vendor_source": vendor,
        # 初始位姿对齐的实测证据（抬升量/最低点 geom/后置残差）；未声明该段的本体为 null。
        "initial_alignment": initial_alignment,
        "injections": injections,
        "sensors": {
            "injected": injected_sensors,
            "vendor_referenced": vendor_sensors,
            "deferred": deferred_sensors,
        },
        "model_facts": facts,
        # Profile 的身份凭据随报告留痕（名称/关节数/能力/校验状态）：报告与 Profile 一起可复算。
        "profile_identity": {
            "name": str(robot),
            "joints": [str(item) for item in (profile_spec.get("joints") or [])],
            "capabilities": [str(item) for item in (profile_spec.get("capabilities") or [])],
            "verification": str(profile_spec.get("verification")),
            "simulation": bool(profile_spec.get("simulation")),
        },
    }
    output_path.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return report
