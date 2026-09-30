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
import importlib.util
import json
import os
import shutil
import sys
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
    """厂商资产锁定校验：文件必须在锁内登记，且 SHA-256 与锁一致（字节级 provenance）。

    **派生模型**（2026-09-29 新增，`spec.model.derived_from_report`）：附加本体的模型可能不是**单个**
    厂商文件，而是**多个厂商件组装**出来的产物（UR5e+2F-85 由 `scripts/assemble_ur5e_2f85.py` 组装，
    落在 `build/models/` 下 ⇒ 天然不在 `vendor/` 里，撞上 vendor 前缀门禁）。
    此时**不绕过** provenance，而是把锁换成**该臂侧报告里的 `model_source.sha256`**：
    报告由组装流程本身产出、记录 sha256 ⇒ 文件必须与该 sha256 逐位一致，否则 fail-closed。
    为什么用"声明一个报告路径"而不是"把 sha256 写进 Profile"：哈希写死会随模型更新静默过期。
    """
    root = Path(root)
    vendor = str(model["vendor"])
    relative = str(model["file"])
    derived_report = model.get("derived_from_report")
    if derived_report:
        source = root / str(relative)
        if not source.is_file():
            _fail(EXIT_REFERENCE, "%s 的派生模型不存在: %s" % (label, relative))
        report_path = root / str(derived_report)
        if not report_path.is_file():
            _fail(EXIT_REFERENCE, "%s.derived_from_report 不存在: %s" % (label, derived_report))
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            _fail(EXIT_REFERENCE, "%s.derived_from_report 不可解析: %s" % (label, error))
        locked = str(((report.get("model_source") or {}).get("sha256")) or "")
        if not locked:
            _fail(EXIT_REFERENCE,
                  "%s.derived_from_report 指向的报告没有 model_source.sha256（无从校验派生模型）" % label)
        import hashlib
        digest = hashlib.sha256()
        with open(str(source), "rb") as handle:
            for chunk in iter(lambda: handle.read(1 << 20), b""):
                digest.update(chunk)
        actual = digest.hexdigest()
        if actual != locked:
            _fail(EXIT_REFERENCE,
                  "%s 的派生模型 sha256 与报告不一致（模型被改过或报告过期）: 实际 %s ≠ 锁 %s"
                  % (label, actual, locked))
        return {"source": str(source), "vendor": vendor, "derived_from_report": str(derived_report),
                "locked_sha256": locked, "actual_sha256": actual, "lock_schema": "derived-report"}

    prefix = "vendor/%s/" % vendor
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
                # **道具的接触 margin 必须显式写 0**（2026-09-29 §11.23(48)）：挂到本体上的道具位于
                # 厂商 `<default class=...>` 作用域内 ⇒ 会继承厂商的 `margin="0.001"`，而挂在 worldbody
                # 上的道具是引擎默认 0 ⇒ 同一对"载荷-托盘"geom 的接触判定踩在 ~0.65 mm 刀口上
                # （实测 gap 0.000627866 有接触通过、0.000675377 无接触失败）。显式 0 = 引擎默认值，
                # 不引入实现层新数字，只是**不再继承**厂商宽容度，判据完全由物理接触决定。
                margin="0",
                **attributes,
            )
            record["mount"] = {"frame": frame_name, "entity": entity, "body": body_name}
            record["pose_source"] = "mount_frame"
            record["has_free_joint"] = False
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
            # **静态支撑体**（声明 `static: true`，2026-09-28 §11.23(41)）：注入到 worldbody 但
            # **不带 freejoint** ⇒ 刚接到世界。为什么需要：把交接面抬到台面之上必须有支撑结构，
            # 而带 freejoint 的台架会直接自由落体；静态体的位姿完全确定 ⇒ 构建期按名义位姿解出的
            # 放置四段在运行期逐位有效（自由体托盘会有落体/被推动的位姿漂移）。
            static = bool(prop.get("static", False))
            if not static:
                ET.SubElement(body, "freejoint", name=body_name + "_free")
            ET.SubElement(
                body,
                "geom",
                name=body_name + "_geom",
                mass="%.9f" % float(prop["mass_kg"]),
                material=material_name,
                friction=str(prop["friction"]),
                # **道具的接触 margin 必须显式写 0**（2026-09-29 §11.23(48)）：挂到本体上的道具位于
                # 厂商 `<default class=...>` 作用域内 ⇒ 会继承厂商的 `margin="0.001"`，而挂在 worldbody
                # 上的道具是引擎默认 0 ⇒ 同一对"载荷-托盘"geom 的接触判定踩在 ~0.65 mm 刀口上
                # （实测 gap 0.000627866 有接触通过、0.000675377 无接触失败）。显式 0 = 引擎默认值，
                # 不引入实现层新数字，只是**不再继承**厂商宽容度，判据完全由物理接触决定。
                margin="0",
                **attributes,
            )
            record["pose_source"] = "world_static" if static else "world"
            record["has_free_joint"] = (not static)
            record["position_m"] = position
            record["quaternion_wxyz"] = quaternion
        injected.append(record)
    return injected, frames


#: 附加本体位置执行器刚度口径的必需声明键（Profile `spec.model.position_gain.*`）。
POSITION_GAIN_KEYS = ("baseline", "section", "joints_section")

#: 指腹摩擦口径的必需声明键（Profile `spec.model.finger_friction.*`）。
FINGER_FRICTION_KEYS = ("baseline", "section", "key")


def _declared_finger_friction(root, profile_spec, robot_id):
    """按声明取**指腹摩擦**（3 个数），未声明返回 None（不注入、不猜）。

    为什么必须（2026-09-28 实测）：厂商 Piper MJCF 的指腹摩擦是 [1.0, 0.005, 0.0001]，
    臂自己场景用基线 `scene.finger_friction`（[2.0, 0.05, 0.001]）⇒ 联合世界里摩擦只有一半，
    抓取后的**搬运**（0.6 m 横移）会滑掉：实测 approach 段结束时方块已落回台面
    （载荷最低点 −0.000216 m、双侧指腹接触 false），下行 IK 因此不可达。
    """
    declaration = (profile_spec.get("model") or {}).get("finger_friction")
    if not declaration:
        return None
    for key in FINGER_FRICTION_KEYS:
        if not declaration.get(key):
            _fail(EXIT_DECLARATION, "robots.%s.spec.model.finger_friction 缺少 %s" % (robot_id, key))
    baseline_path = Path(str(declaration["baseline"]))
    if not baseline_path.is_absolute():
        baseline_path = root / baseline_path
    baseline_path = baseline_path.resolve()
    if not baseline_path.is_file():
        _fail(EXIT_REFERENCE, "robots.%s.spec.model.finger_friction.baseline 不存在: %s"
              % (robot_id, baseline_path))
    document = _read_yaml(baseline_path, "robots.%s.spec.model.finger_friction.baseline" % robot_id)
    section = document.get(str(declaration["section"]))
    if not isinstance(section, dict):
        _fail(EXIT_REFERENCE, "robots.%s.spec.model.finger_friction.section=%s 不是字典"
              % (robot_id, declaration["section"]))
    raw = section.get(str(declaration["key"]))
    if not isinstance(raw, str) or len(raw.split()) != 3:
        _fail(EXIT_DECLARATION, "robots.%s 的 %s.%s 必须是 3 个数值的字符串（MuJoCo friction 口径）"
              % (robot_id, declaration["section"], declaration["key"]))
    try:
        values = [float(item) for item in raw.split()]
    except ValueError:
        _fail(EXIT_DECLARATION, "robots.%s 的 %s.%s 解析失败: %r"
              % (robot_id, declaration["section"], declaration["key"], raw))
    if any(value < 0 for value in values) or values[0] <= 0:
        _fail(EXIT_DECLARATION, "robots.%s 的指腹摩擦必须非负且滑动摩擦为正: %r" % (robot_id, raw))
    return values


def _place_targets(scene, model, injections, trunk_body):
    """承载面上的**接收体**（`props` 里带 `mount` 的道具）+ 其**标称停靠位姿**（2026-09-28，§11.12）。

    为什么需要：`place_object` 的输入契约只给接收体的**名字**（托盘随载体运动 ⇒ 任何预写的世界位姿都会过期）。
    但"放置点"仍然要有一个基准，报告因此声明两件事：
      ① **几何**（半尺寸、质量、挂载帧）——用于运行期判"载荷是否落在承载面内"；
      ② **标称停靠位姿**（载体停在声明的交接站位时的 FK）——构建期解放置姿态的唯一可能基准。
    运行期**必须**以实测位姿为准（证据里给 `place_alignment.center_distance_m` 与载荷偏移）；
    本段的 `nominal_*` 只作构建基准与留痕，不当作运行期真值。
    """
    declaration = {str(item.get("id")): item for item in (scene.get("props") or [])
                   if isinstance(item, dict)}
    targets = []
    station_record = None
    for item in (injections.get("world_frames") or []):
        if str(item.get("id")) == "handoff_station_frame":
            station_record = item
            break
    station_pose = None
    if station_record is not None:
        station_pose = (list(_vec3(station_record.get("pos_m"), "frames.handoff_station_frame.pos_m")),
                        list(_quat_wxyz(station_record.get("quat_wxyz"),
                                        "frames.handoff_station_frame.quat_wxyz")))
    data = mujoco.MjData(model)
    dof_address = None
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(trunk_body))
    if body_id >= 0 and int(model.body_jntnum[body_id]) > 0:
        candidate = int(model.body_jntadr[body_id])
        if int(model.jnt_type[candidate]) == int(mujoco.mjtJoint.mjJNT_FREE):
            dof_address = int(model.jnt_qposadr[candidate])
    if station_pose is not None and dof_address is not None:
        if int(getattr(model, "nkey", 0) or 0) > 0:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        # ⚠ 站位的 z 是**地面**高度（提交声明 (0.45, 0, 0)），而载体停在那里时机身并不在地面：
        # 名义基座高度取**关键帧实测值**（本模型 0.288372 m，来自模型 FK，不手写数字）。
        base_height = float(data.qpos[dof_address + 2]) if int(getattr(model, "nkey", 0) or 0) else 0.0
        data.qpos[dof_address:dof_address + 3] = [station_pose[0][0], station_pose[0][1], base_height]
        data.qpos[dof_address + 3:dof_address + 7] = station_pose[1]
    mujoco.mj_forward(model, data)
    for record in (injections.get("props") or []):
        mount = record.get("mount")
        prop_id = str(record.get("id"))
        declared = declaration.get(prop_id) or {}
        geometry = declared.get("geometry") or {}
        # 接收体有两种：
        #   ① **随载体运动**（如狗背托盘）：声明 `pose.mount` ⇒ 标称位姿 = 「载体停在站位」时的 FK；
        #   ② **世界固定**（如交接台上的托盘，2026-09-28 §11.23(36)）：没有 `mount`、但有 `pose.pos_m`
        #      ⇒ 标称位姿仍取 FK（静态体不随任何载体动，FK 就是它自己的位姿）。
        # 两者都缺 ⇒ 不是接收体（跳过，保持原行为）。
        has_mount = isinstance(mount, dict)
        # 无 `mount` 的道具**必须显式声明 `receiving: true`** 才算接收体：
        # 否则载荷自身（同样有 pose.pos_m）会被误当接收体（本轮实测：targets[0] 变成方块自己）。
        if not has_mount and not bool(declared.get("receiving", False)):
            continue
        geom_name = "%s_geom" % str(record.get("body"))
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        target = {
            "id": prop_id,
            "body": str(record.get("body")),
            "geom": geom_name if geom_id >= 0 else None,
            "kind": str(record.get("kind")),
            "mass_kg": record.get("mass_kg"),
            "mount": dict(mount) if has_mount else None,
            "mount_offset_m": [round(float(v), 9) for v in (record.get("position_m") or [])],
            "size_m": ([round(float(v), 9) for v in geometry.get("size_m")]
                       if isinstance(geometry.get("size_m"), (list, tuple)) else None),
            "pose_source": ("nominal_docked_station" if has_mount else "scene_declared_pose"),
            "runtime_pose_source": "live_fk",
        }
        prop_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(record.get("body")))
        if prop_body >= 0:
            target["nominal_pose_m"] = [round(float(v), 9) for v in data.xpos[prop_body]]
            target["nominal_quaternion_wxyz"] = [round(float(v), 9) for v in data.xquat[prop_body]]
        targets.append(target)
    return {"targets": targets,
            "station_frame": (str(station_record.get("id")) if station_record else None),
            "station_pose_m": (station_pose[0] if station_pose else None),
            "nominal_base_height_m": (round(float(data.qpos[dof_address + 2]), 9)
                                      if (dof_address is not None
                                          and int(getattr(model, "nkey", 0) or 0)) else None),
            "note": ("接收体按名字引用（托盘随载体运动）；`nominal_*` 是「载体停在交接站位」时的 FK，"
                     "只作构建基准；运行期以实测位姿为准（证据给实测偏移）。")}


def _apply_declared_position_gains(root, child, profile_spec, robot_id):
    """按声明把**位置执行器刚度**注入附加本体的子模型（数字与规则都来自声明，构建器不写数字）。

    为什么必须（2026-09-24 实测，`docs/debug/2026-09-24-joint-model-dog-arm.md` §11.6）：
    `--attach` 附加的是 Profile 声明的**厂商 MJCF**（kp = 10000/2000/500/50/20/5），而臂自己场景
    注入的是**声明增益**（450/200/200/200/200/200）⇒ 两个模型里的臂**不是同一个动力学系统**：
    厂商口径下 approach/grasp/lift 的静态保持残余 0.029975223 / 0.036213257 / 0.206096435 rad
    （声明容差 0.001），臂场景同样四项 ≤3.8e-06 rad。臂自己场景的验收数字（抓取残差 9.39e-06 m）
    只在**声明口径**下成立 ⇒ 联合模型必须同口径，否则"联合世界"验的不是同一台臂。

    契约（Profile `spec.model.position_gain = {baseline, section, joints_section}`）：
      · `baseline[section].inject_arm_position_gains`（bool，false ⇒ 不注入，沿用厂商口径）；
      · `baseline[section].arm_position_kp` / `arm_position_kp_damping_ratio`；
      · `baseline[joints_section].arm_joints`（参与注入的关节名）；
      · 规则（与 `scripts/build_piper_pick_scene.py` 同一条，两处都读同一份声明）：
        `kp := max(arm_position_kp, 关节阻尼 × ratio)`。
    未声明该段 ⇒ 返回 `{"declared": False, ...}`（留痕为"沿用厂商口径"，不猜、不静默注入）。
    """
    declaration = (profile_spec.get("model") or {}).get("position_gain")
    if not declaration:
        return {"declared": False,
                "note": "未声明 spec.model.position_gain ⇒ 沿用附加本体自带（厂商）执行器口径"}
    for key in POSITION_GAIN_KEYS:
        if not declaration.get(key):
            _fail(EXIT_DECLARATION, "robots.%s.spec.model.position_gain 缺少 %s" % (robot_id, key))
    baseline_path = Path(str(declaration["baseline"]))
    if not baseline_path.is_absolute():
        baseline_path = root / baseline_path
    baseline_path = baseline_path.resolve()
    if not baseline_path.is_file():
        _fail(EXIT_REFERENCE, "robots.%s.spec.model.position_gain.baseline 不存在: %s"
              % (robot_id, baseline_path))
    document = _read_yaml(baseline_path, "robots.%s.spec.model.position_gain.baseline" % robot_id)
    section = document.get(str(declaration["section"]))
    if not isinstance(section, dict):
        _fail(EXIT_REFERENCE, "robots.%s.spec.model.position_gain.section=%s 在 %s 里不是字典"
              % (robot_id, declaration["section"], baseline_path))
    if not section.get("inject_arm_position_gains"):
        return {"declared": True, "baseline": str(baseline_path),
                "inject_arm_position_gains": False,
                "note": "声明里 inject_arm_position_gains=false ⇒ 沿用厂商执行器口径（不注入）"}
    arm_kp = section.get("arm_position_kp")
    ratio = section.get("arm_position_kp_damping_ratio")
    joints_section = document.get(str(declaration["joints_section"])) or {}
    joints = [str(item) for item in (joints_section.get("arm_joints") or [])]
    for label, value in (("arm_position_kp", arm_kp), ("arm_position_kp_damping_ratio", ratio)):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) <= 0:
            _fail(EXIT_DECLARATION, "robots.%s 的执行器刚度声明 %s 必须是正数（在 %s 的 %s 段）"
                  % (robot_id, label, baseline_path, declaration["section"]))
    if not joints:
        _fail(EXIT_DECLARATION, "robots.%s 的执行器刚度声明缺少关节清单（%s 的 %s.arm_joints）"
              % (robot_id, baseline_path, declaration["joints_section"]))
    damping = {str(joint.name): float(joint.damping)
               for joint in (getattr(child, "joints", []) or [])}
    applied = []
    for actuator in (getattr(child, "actuators", []) or []):
        name = str(getattr(actuator, "name", "") or "")
        if name not in joints:
            continue
        # 位置执行器的判别：仿射偏置 biasprm[1] = −kp（厂商 MJCF 的 `<position>` 即此形状）。
        # 不是位置执行器 ⇒ 显式失败，而不是"改了增益但没生效"。
        # ⚠ 坑位：MjSpec 的 gainprm/biasprm 是 ndarray ⇒ 既不能用 `x or []`（真值歧义），
        # 也不能逐元素赋值（不可变视图）⇒ 整段读成 list、改完再整体写回。
        bias = [float(v) for v in actuator.biasprm]
        if len(bias) < 2 or abs(bias[1]) < 1e-12:
            _fail(EXIT_MODEL, "附加本体 %s 的关节 %s 的执行器 %s 不是位置执行器"
                  "（biasprm[1]=%s），无法按声明注入刚度" % (robot_id, name, name, bias[:2]))
        gain = [float(v) for v in actuator.gainprm]
        previous = float(gain[0])
        kp = max(float(arm_kp), float(damping.get(name, 0.0)) * float(ratio))
        gain[0] = float(kp)
        bias[1] = -float(kp)
        actuator.gainprm = gain
        actuator.biasprm = bias
        applied.append({"joint": name, "damping": damping.get(name),
                        "kp_before": previous, "kp_after": round(float(kp), 6)})
    if not applied:
        _fail(EXIT_MODEL, "附加本体 %s 声明了执行器刚度口径，但子模型里没有任何声明的关节执行器"
              "（声明关节 %s）" % (robot_id, joints))
    return {"declared": True, "baseline": str(baseline_path),
            "inject_arm_position_gains": True,
            "arm_position_kp": float(arm_kp), "arm_position_kp_damping_ratio": float(ratio),
            "rule": "kp := max(arm_position_kp, 关节阻尼 × ratio)（与臂自己场景同一条规则）",
            "applied": applied}


#: 场景级**物理口径**声明的允许键（`scene.yaml: world_physics`）。
WORLD_PHYSICS_KEYS = ("cone", "impratio", "timestep", "gravity")


def _apply_world_physics(staging, scene, required):
    """把声明的物理口径写进合成产物的 `<option>`，并从**编译后的模型**读回实测值。

    为什么必须显式声明（2026-09-28，docs/debug/2026-09-24-joint-model-dog-arm.md §11.23）：
    合成产物继承的是**主模型（四足厂商模型）**的 `<option>`
    （`vendor/unitree_go2/unitree_robots/go2/go2.xml:4` → `cone="elliptic" impratio="100"`），
    而同一台臂在**自己的世界**里用的是 MuJoCo 默认（`cone=pyramidal impratio=1`）
    ⇒ 夹持/接触类结论**跨世界不可移植**（本轮 s04 丢件就是这么来的：能搬 0.20 m 是在臂世界量的）。
    数值全部来自声明，构建器里不写任何默认值：缺声明即拒绝构建（fail-closed）。
    """
    declaration = dict(((scene or {}).get("world_physics") or {}))
    if not declaration:
        if required:
            _fail(EXIT_DECLARATION,
                  "联合世界（存在附加本体）必须在 scene.yaml 声明 `world_physics`"
                  "（至少 cone/impratio）：不得继承厂商模型的隐式 `<option>` —— 否则同一份"
                  "夹持/接触结论在单本体世界与联合世界里不可比（见 §11.23）")
        return None
    unknown = sorted(key for key in declaration if key not in WORLD_PHYSICS_KEYS)
    if unknown:
        _fail(EXIT_DECLARATION, "world_physics 含未知键 %s（允许：%s）"
              % (unknown, list(WORLD_PHYSICS_KEYS)))
    tree = ET.parse(str(staging))
    root = tree.getroot()
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        compiler = root.find("compiler")
        root.insert((list(root).index(compiler) + 1) if compiler is not None else 0, option)
    for key, value in declaration.items():
        if isinstance(value, (list, tuple)):
            option.set(str(key), " ".join("%.9g" % float(item) for item in value))
        elif isinstance(value, str):
            # 枚举型（cone=pyramidal/elliptic）按声明原文写，不做数值转换
            option.set(str(key), value)
        else:
            option.set(str(key), "%.9g" % float(value))
    tree.write(str(staging), encoding="utf-8", xml_declaration=False)
    return {"declared": {str(key): (list(value) if isinstance(value, (list, tuple)) else value)
                         for key, value in declaration.items()},
            "source": "scene.yaml:world_physics",
            "note": ("物理口径必须声明而不是继承：合成产物的 <option> 来自主模型（厂商四足模型），"
                     "与附加本体自己世界的默认值不同 ⇒ 结论不可跨世界引用（§11.23）")}


def _inject_carry_constraint(staging, scene, root):
    """把**搬运段抓取约束**（mocap anchor + `<connect>` 焊接）注入合成产物。

    为什么（2026-09-28 §11.23(29)，使用者裁定走"声明化搬运约束"）：本几何下唯一撑得住抬升的抓取是
    "斜夹过盈"，而它在搬运中沿夹口轴向把载荷楔出去；纯摩擦路线已被 30+ 条量化否证穷尽。
    与臂侧世界**同款机制**（`build/models/piper-pick-scene.xml` 里同名 `<connect ... active="false">`）：
    约束默认**惰性**（`active="false"`），只有后端在报告声明 `gripper.carry_constraint.enabled=true`
    且抓取力达标时才置 `eq_active=1`；证据里已留 `constraint_activated`。
    名字与启用与否**全部来自声明**（臂侧报告的 `gripper.carry_constraint`），构建器不写死。
    """
    declaration = None
    for robot in (scene.get("robots") or []):
        report_ref = robot.get("manipulation_report")
        if not report_ref:
            continue
        report_path = Path(str(report_ref))
        if not report_path.is_absolute():
            report_path = root / report_path
        if not report_path.is_file():
            continue
        try:
            document = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        candidate = ((document.get("gripper") or {}).get("carry_constraint") or {})
        if candidate.get("enabled"):
            declaration = candidate
            break
    if not declaration:
        return None
    equality_name = str(declaration.get("equality_name") or "")
    anchor_name = str(declaration.get("anchor_body") or "")
    if not equality_name or not anchor_name:
        _fail(EXIT_DECLARATION, "carry_constraint 声明缺少 equality_name / anchor_body")
    # **约束类型**必须由声明给出（fail-closed，不给实现层默认值；2026-09-28 §11.23(41)）：
    # `connect` = 球铰，只约束平移 ⇒ 实测载荷会在夹口里翻滚/楔出（丢手前接触对只有两个指腹、
    # 力 18→42 N 抬升 5 帧后 0.1 s 下坠 11 cm）；`weld` = 6 自由度刚性焊 ⇒ 位置与姿态同时跟随。
    constraint_type = str(declaration.get("type") or "")
    if constraint_type not in ("connect", "weld"):
        _fail(EXIT_DECLARATION,
              "carry_constraint.type 必须是 connect 或 weld（实际 %r）：搬运要不要约束**旋转**"
              "必须由声明决定，不得由构建器猜（connect 是球铰 ⇒ 载荷会翻滚）" % constraint_type)
    # 约束刚度同样必须由声明给出（2026-09-28 §11.23(41)：原先硬编码在本函数里，违反"值进声明"）。
    solref = declaration.get("solref")
    solimp = declaration.get("solimp")
    if (not isinstance(solref, (list, tuple)) or len(solref) != 2
            or not isinstance(solimp, (list, tuple)) or len(solimp) != 3):
        _fail(EXIT_DECLARATION,
              "carry_constraint 必须声明 solref（2 个数）与 solimp（3 个数）："
              "实测 solref 0.01 太软，s04 摆基座时载荷相对指腹在 xy 漂移 ~3 cm")
    solref_text = "%.9f %.9f" % (float(solref[0]), float(solref[1]))
    solimp_text = "%.9f %.9f %.9f" % (float(solimp[0]), float(solimp[1]), float(solimp[2]))
    # anchor 位置 = 该目标在**本场景**里的位姿（不能用臂侧报告里的坐标：那是臂自己世界的）
    target_id = None
    for robot in (scene.get("robots") or []):
        report_ref = robot.get("manipulation_report")
        if not report_ref:
            continue
        report_path = Path(str(report_ref))
        if not report_path.is_absolute():
            report_path = root / report_path
        if report_path.is_file():
            target_id = json.loads(report_path.read_text(encoding="utf-8")).get("target_id")
            if target_id:
                break
    prop = next((item for item in (scene.get("props") or [])
                 if item.get("id") == target_id or item.get("body") == target_id), None)
    if prop is None or not (prop.get("pose") or {}).get("pos_m"):
        _fail(EXIT_REFERENCE, "carry_constraint 需要目标 %r 在场景 props 里声明 pose.pos_m（anchor 位置）"
              % (target_id,))
    anchor_pos = [float(v) for v in prop["pose"]["pos_m"]]
    tree = ET.parse(str(staging))
    xml_root = tree.getroot()
    world = xml_root.find("worldbody")
    if world is None:
        _fail(EXIT_MODEL, "合成产物没有 worldbody，无法注入 carry_constraint 的 anchor")
    if xml_root.find(".//body[@name='%s']" % anchor_name) is None:
        ET.SubElement(world, "body", {"name": anchor_name,
                                      "pos": _numbers(anchor_pos, "%.9f"), "mocap": "true"})
    equality = xml_root.find("equality")
    if equality is None:
        equality = ET.SubElement(xml_root, "equality")
    if equality.find("%s[@name='%s']" % (constraint_type, equality_name)) is None:
        attributes = {
            "name": equality_name,
            "body1": anchor_name,
            "body2": str(target_id),
            "active": "false",
            "solref": solref_text,      # ← 声明值（不再硬编码）
            "solimp": solimp_text,
        }
        if constraint_type == "connect":
            attributes["anchor"] = _numbers(anchor_pos, "%.9f")
        else:
            # `weld` 的 `anchor` 是 **body2（载荷）局部系**里的焊点 ⇒ 取载荷原点；
            # 不给 `relpose` ⇒ body1（mocap anchor）的位姿须等于载荷位姿：后端在激活时把 anchor
            # 摆到载荷当前位姿，之后按「腕部姿态 ⊗ 激活瞬间相对姿态」整体刚性跟随（见 §11.23(41)）。
            attributes["anchor"] = "0 0 0"
        ET.SubElement(equality, constraint_type, attributes)
    tree.write(str(staging), encoding="utf-8", xml_declaration=False)
    return {"equality_name": equality_name, "anchor_body": anchor_name,
            "type": constraint_type,
            "solref": [float(solref[0]), float(solref[1])],
            "solimp": [float(solimp[0]), float(solimp[1]), float(solimp[2])],
            "anchor_pos_m": anchor_pos, "active": False,
            "source": "gripper.carry_constraint（臂侧报告声明）"}


def _read_world_physics(staging):
    """从**编译后的模型**读回生效口径（证据用：声明 ≠ 生效时要能立刻看见）。"""
    model = mujoco.MjModel.from_xml_path(str(staging))
    # `opt.cone` 读回是整数枚举 ⇒ 归一到与声明同名的字符串（否则 "pyramidal" vs 0 会误报不一致）
    try:
        cone_name = str(mujoco.mjtCone(model.opt.cone).name).split("_")[-1].lower()
    except Exception:  # pragma: no cover - 绑定差异兜底
        cone_name = str(model.opt.cone)
    return {"cone": cone_name, "cone_name": cone_name,
            "impratio": float(model.opt.impratio), "timestep": float(model.opt.timestep),
            "gravity": [float(value) for value in model.opt.gravity]}


def _child_compiler_dirs(child_path):
    """读附加本体 XML 的 `<compiler meshdir/assetsdir>`（相对子模型目录的字符串）。

    为什么必须读（2026-09-29 实测）：资产拷贝原先只按 `子模型目录 / file` 找源文件，这对 Piper
    成立（它的 `meshdir` 是默认值、STL 就放在模型目录旁），但 UR5e 装配模型写的是
    `meshdir="assets/"` + `file="shoulder_0.obj"` ⇒ `child_dir/shoulder_0.obj` 不存在，
    真身是 `child_dir/assets/shoulder_0.obj`。结果是这些 mesh 保持**相对路径**留在联合产物里，
    合成后按主模型的 meshdir（Go2 的 assets 目录）解析 ⇒ 编译期 ENOENT：
    `vendor/unitree_go2/unitree_robots/go2/assets/shoulder_0.obj`（实测）。
    """
    node = ET.parse(str(child_path)).getroot()
    compiler = node.find("compiler")
    meshdir = str(compiler.get("meshdir")) if compiler is not None and compiler.get("meshdir") else "."
    assetsdir = (str(compiler.get("assetsdir"))
                 if compiler is not None and compiler.get("assetsdir") else ".")
    return meshdir, assetsdir


def _declared_actuator_ctrl(child_path, home, robot_id):
    """按子模型自己的 `<actuator>` 声明，给出**每个执行器的关键帧 ctrl**（声明驱动，不猜）。

    为什么必须（2026-09-29 实测，**真缺陷**）：`MjSpec.attach` 会把关键帧的 ctrl 用**零**扩展，
    而 ctrl=0 对**位置执行器**意味着"目标关节角 = 0" ⇒ 附加本体一上电就**摆到零位**。
    实测（联合产物关键帧）：`ur5e_shoulder_pan` ctrl=0 而 qpos=−1.570800（差 −1.570800）、
    `ur5e_elbow` ctrl=0 而 qpos=+1.570800（差 +1.570800）。
    Piper 的零位恰好"在臂上方"（且它自己的场景构建器写过 ctrl），所以这个缺陷一直藏着；
    UR5e 的摆动会扫过四足走的走廊 ⇒ 表现成整株植物失稳
    （`Nan, Inf or huge value in QACC at DOF 0 ... Time = 276.802`，s01 末速 131.3 m/s）。

    口径：ctrl = `home[joint] / gear`（位置执行器 gear 通常 1；用 gear 换算而不是直接抄角度，
    因为 ctrl 是**执行器空间**的量）；执行器的传输**不是关节**（如 2F-85 的指爪走 tendon）
    时取 0.0（与厂商 keyframe 的 0 一致）并登记来源；传输是关节但**未在 Profile 声明初值**
    ⇒ 显式失败（与 qpos 那条规则同源，禁止用零顶替）。
    """
    root = ET.parse(str(child_path)).getroot()
    section = root.find("actuator")
    rows = []
    if section is None:
        return rows
    for node in section:
        joint = node.get("joint")
        gear = float(node.get("gear") or 1.0)
        name = str(node.get("name") or "<unnamed>")
        if joint and str(joint) in home:
            rows.append({"actuator": name, "joint": str(joint),
                         "ctrl": float(home[str(joint)]) / (gear if gear else 1.0),
                         "gear": gear, "source": "declared_home_div_gear"})
        elif joint:
            _fail(EXIT_DECLARATION,
                  "附加本体 %s 的执行器 %s 的传输关节 %s 未在 Profile 声明关键帧初值"
                  "（spec.home / spec.model.home / spec.gripper.open_positions 之一）"
                  "：缺声明即失败，不用 0 顶替" % (robot_id, name, joint))
        else:
            rows.append({"actuator": name, "joint": None, "ctrl": 0.0, "gear": gear,
                         "source": "no_joint_transmission_zero"})
    return rows


def _child_asset_source(record, path):
    """按**子模型自己的编译器规则**列出源文件候选（meshdir → assetsdir → 子模型目录），取首个存在的文件。

    顺序即声明优先级：先信子模型声明的 meshdir，再退到 assetsdir，最后才是目录本身。
    """
    base = Path(str(record.get("child_dir")))
    roots = []
    for key in ("child_meshdir", "child_assetsdir"):
        value = str(record.get(key) or "")
        if value and value not in (".", "./"):
            roots.append(base / value)
    roots.append(base)
    for root in roots:
        candidate = root / path
        if candidate.is_file():
            return candidate
    return None


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
    key_ctrl_by_index = {}
    for parsed_key in staging_parsed.find("keyframe") or []:
        if parsed_key.tag == "key" and parsed_key.get("qpos"):
            key_qpos_by_index[len(key_qpos_by_index)] = [
                float(v) for v in parsed_key.get("qpos").split()]
        if parsed_key.tag == "key" and parsed_key.get("ctrl") is not None:
            key_ctrl_by_index[len(key_ctrl_by_index)] = [
                float(v) for v in (parsed_key.get("ctrl") or "").split()]
    spec = mujoco.MjSpec.from_file(str(staging))
    records = []
    attached_robot_values = []
    # 关键帧 **ctrl** 的附加部分（每个附加本体一组，顺序与 attached_ids / 子模型 actuator 声明一致）：
    # 与 qpos 分开收集，因为它们长度不同（qpos 含自由关节/全部关节，ctrl 只有执行器）。
    attached_ctrl_values = []
    # 指爪 geom 的**声明名**占用表（跨附加本体）：重名即显式失败，见下方注释（MuJoCo 允许 geom 重名，
    # 但按名字解析会静默指到任意一个 ⇒ 不能让"附加顺序"变成隐式语义）。
    claimed_finger_geoms = set()
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
        # 子模型的资产解析目录（资产拷贝要按**它自己的**规则找源文件，见 _child_compiler_dirs）
        child_meshdir, child_assetsdir = _child_compiler_dirs(child_path)
        # 执行器刚度口径（Profile 声明 → 按声明注入；未声明即留痕"沿用厂商口径"，不猜）：
        # 联合模型里的臂必须与臂自己场景同一动力学口径，否则臂侧验收数字不可移植（§11.6）。
        gain_record = _apply_declared_position_gains(root, child, profile_spec, robot_id)
        # 关节初值来自**声明的**两个来源（缺键即显式失败，不猜）：
        #   · `spec.home`：本体标称位形（臂的 joint1..6）
        #   · `spec.gripper.open_positions`：抓手的"张开"位形（臂的 joint7/8 在这段里声明）
        home = dict(profile_spec.get("home") or {})
        # **`spec.model.home` 也必须读**（2026-09-29 修）：错误信息一直写着"未在 Profile spec.model.home
        # 声明初值"，而代码只读了 `spec.home` ⇒ 报错指向的键和行为不一致。这个键是**联合构建专用**的
        # 初值来源：被动关节（如 2F-85 的 coupler/spring_link/follower）不能进 `spec.joints`（会让
        # move_joint 直接控制它们、破坏 4 杆机构），因此也不能进 `spec.home`（Profile 校验会拒），
        # 但它们**必须**有联合关键帧初值 ⇒ 落在这里。
        model_home = (profile_spec.get("model") or {}).get("home")
        if isinstance(model_home, dict):
            home.update({str(k): float(v) for k, v in model_home.items()})
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
        # 声明名清单（联合报告 `manipulation.name_map` 的**输入**）：附加本体在**自己的模型**里的
        # 对象名。采集必须在 `attach` **之前** —— attach 之后这些名字一律带上前缀，问不回原名。
        # 只收集后端能按名字解析的种类（body/geom/site/joint）：执行器与关节在 Piper 上同名，
        # 由关节解析覆盖；mesh/material 由 MjSpec 自己改写引用，不进这张表。
        declared_sets = {"bodies": set(), "geoms": set(), "sites": set(), "joints": set(),
                     "actuators": set()}

        def _collect_names(kind, items):
            """收集某一类的对象名；同时把 body 的**子对象**按自己的类别归档。

            （MjSpec 顶层 `.geoms` / `.sites` 在本版本里为空而 body 内有对象 ⇒ 必须下钻；
            归档到各自类别，否则报告里的 kind 标签会把关节写成 body。）
            """
            for item in items:
                name = str(getattr(item, "name", "") or "")
                if name:
                    declared_sets[kind].add(name)
                for attribute in ("geoms", "sites", "joints"):
                    for nested in (getattr(item, attribute, []) or []):
                        nested_name = str(getattr(nested, "name", "") or "")
                        if nested_name:
                            declared_sets[attribute].add(nested_name)

        # `actuators` 也要收集（2026-09-29）：臂侧 gripper 引用执行器名（如 rq2f85_fingers_actuator），
        # 它必须进 name_map 才能被后端按声明名解析到联合模型里的 ur5e_ 前缀名。
        for bucket in ("bodies", "geoms", "sites", "joints", "actuators"):
            _collect_names(bucket, getattr(child, bucket, []) or [])
        declared_names = {key: sorted(value) for key, value in declared_sets.items()}
        # 关键帧 ctrl：附加本体的执行器必须**按声明保持初值**（否则位置执行器把它们驱到零位，
        # 详见 `_declared_actuator_ctrl` 的说明与实际故障记录）。顺序 = 子模型 <actuator> 的声明顺序。
        ctrl_rows = _declared_actuator_ctrl(child_path, home, robot_id)
        attached_ctrl_values.append([row["ctrl"] for row in ctrl_rows])
        prefix = "%s_" % robot_id
        # MjSpec 需要**数值列表**（`_numbers()` 是给 XML 属性用的字符串，别混用）
        frame = spec.worldbody.add_frame(pos=[float(v) for v in pos],
                                        quat=[float(v) for v in quat])
        spec.attach(child, prefix=prefix, frame=frame)
        # 指爪 geom 命名（A 方案）：厂商 Piper MJCF 里指爪 geom **无名**（实测 `piper_link7 -> [None]`），
        # 而臂侧自己的场景构建器按 `gripper.left_finger_geom/right_finger_geom` 的约定命名
        # ⇒ 联合模型必须沿用**同一个声明名**，否则 grasp 判据（按 geom 命中双边接触）指不到对象。
        arm_report_path = entity.get("manipulation_report")
        if arm_report_path and (root / str(arm_report_path)).is_file():
            arm_doc = json.loads((root / str(arm_report_path)).read_text(encoding="utf-8"))
            arm_gripper = arm_doc.get("gripper") or {}
            for side in ("left", "right"):
                body_name = arm_gripper.get("%s_finger_body" % side)
                geom_name = arm_gripper.get("%s_finger_geom" % side)
                if not body_name or not geom_name:
                    continue
                body = spec.body("%s%s" % (prefix, body_name))
                if body is None:
                    _fail(EXIT_REFERENCE, "附加本体 %s 的指爪 body %s%s 不在合成模型里"
                          % (robot_id, prefix, body_name))
                geoms = list(getattr(body, "geoms", []) or [])
                # **按声明的 geom 名取**（2026-09-29 修）：原先要求"指爪 body 只能有 1 个 geom"，那是
                # Piper 侧"注入单个 pad"的约定；UR5e+2F-85 每侧有 **2 个 pad** + 1 个 visual geom
                # （基线以 `pad_boxes` 声明 `rq2f85_*_pad1/pad2`）⇒ 该假设不成立
                # （实测退出码 3：`指爪 body 有 3 个 geom`）。
                # ⚠ 第二个坑（同一轮实测）：`spec.attach(prefix=)` 会给**被附加子树里的所有名字**加前缀
                #   ⇒ 厂商名 `rq2f85_left_pad1` 在联合模型里是 `ur5e_rq2f85_left_pad1`，只按裸名查必然落空
                #   （Piper 之所以没暴露：它每侧只有 1 个 geom，走了下面的"单 geom 回退"重命名）。
                #   因此**裸名与加前缀名都接受**，命中后统一改回**声明名**，与臂侧报告/后端/判据
                #   使用的名字保持同一口径（同一条"声明名 → 联合名"的规则，见 name_map）。
                candidates = {str(geom_name), "%s%s" % (prefix, geom_name)}
                named = [item for item in geoms if str(getattr(item, "name", "")) in candidates]
                if not named:
                    if len(geoms) == 1:
                        named = [geoms[0]]
                    else:
                        _fail(EXIT_REFERENCE,
                              "附加本体 %s 的指爪 body %s 下没有声明名字的 geom %s（该 body 有 %d 个 geom）"
                              % (robot_id, body_name, geom_name, len(geoms)))
                # 声明名必须可唯一解析：两台同型臂（或厂商模型自带重名）都改回同一个裸名时，
                # MuJoCo 不会报错、但**后续按名字解析会静默指到任意一个** ⇒ 显式失败，让人去改声明。
                declared_key = str(geom_name)
                if declared_key in claimed_finger_geoms:
                    _fail(EXIT_REFERENCE,
                          "指爪 geom 声明名 %s 在联合模型里不唯一（已有另一台附加本体占用）："
                          "请在本体 Profile 的 model.finger_geoms 里给出本体唯一的指爪 geom 名，"
                          "不要靠附加顺序区分" % declared_key)
                claimed_finger_geoms.add(declared_key)
                for item in named:
                    item.name = declared_key
                # 指腹摩擦按声明注入（见 profiles/piper_mujoco.yaml 的 finger_friction 说明）：
                # 与 position_gain 同一个理由 —— 联合模型里的臂必须与已验收的臂场景同一物理口径，
                # 否则"抓起来"能过、"搬过去"会滑掉（实测）。
                # 口径与臂侧构建器一致：只注入**报告声明的 `*_finger_geom`**（每侧 1 个 pad），
                # 不扩散到同类 pad_boxes 的其余 box（臂侧 build_piper_pick_scene 也只注入这两个）。
                friction_value = _declared_finger_friction(root, profile_spec, robot_id)
                if friction_value is not None:
                    for item in named:
                        item.friction = list(friction_value)

        attached_robot_values.append(list(joint_values))
        records.append({"id": robot_id, "prefix": prefix, "source": str(model["file"]),
                        "placement": {"pos_m": [float(v) for v in pos],
                                      "quat_wxyz": [float(v) for v in quat]},
                        "child_dir": str(child_dir),
                        # 子模型自己的资产解析目录（meshdir/assetsdir）：资产拷贝按它找源文件，
                        # 也必须留痕 —— 否则"某台臂的 mesh 用了别台的目录"这种事故无从复盘。
                        "child_meshdir": str(child_meshdir),
                        "child_assetsdir": str(child_assetsdir),
                        # 附加本体在**自己模型**里的名字清单（联合报告 name_map 的输入与留痕）
                        "declared_names": declared_names,
                        # 执行器刚度口径的留痕（声明出处 / 逐关节 kp_before→kp_after / 规则）
                        "position_gain": gain_record,
                        "home_joints": len(joint_values),
                        # 关键帧 **ctrl** 的逐执行器来源（声明驱动；0 只出现在"传输非关节"的情形）
                        "keyframe_ctrl": ctrl_rows,
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
    if attached_ctrl_values:
        ctrl_flat = [value for values in attached_ctrl_values for value in values]
        base_ctrl = [float(v) for v in key_ctrl_by_index.get(0, [])]
        for index, key in enumerate(list(getattr(spec, "keys", []) or [])):
            base = key_ctrl_by_index.get(index, base_ctrl)
            # ⚠ 长度必须等于 nu（主模型执行器数 + 附加本体执行器数）；不等即显式失败 ——
            # 静默写错长度会得到"某个执行器拿了别人的 ctrl"这种最难查的错。
            expected = len(base) + len(ctrl_flat)
            if expected != len(list(getattr(spec, "actuators", []) or [])):
                _fail(EXIT_MODEL,
                      "关键帧 ctrl 长度自检失败：%d（主模型 %d + 附加 %d）≠ 模型执行器数 %d"
                      % (expected, len(base), len(ctrl_flat),
                         len(list(getattr(spec, "actuators", []) or []))))
            key.ctrl = list(base) + ctrl_flat

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
            # **只处理属于该附加本体的 mesh**（`attach` 会给 mesh 名加前缀 ⇒ 名字前缀即所有权）。
            # 为什么必须有这条：资产查找改为按子模型 meshdir 解析后，"同名不同机"的 mesh 会互撞
            # （实测 Go2 与 UR5e 都有 base_0.obj/baser_*.obj 这类名字）—— 没有这条过滤，
            # 主模型自己的 base_0 mesh 会被改写成 UR5e 的资产绝对路径（静默串味，比编译失败更坏）。
            mesh_name = str(getattr(item, "name", "") or "")
            if not mesh_name.startswith("%s_" % record["id"]):
                continue
            candidate = _child_asset_source(record, path)
            if candidate is not None:
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
            texture_name = str(getattr(item, "name", "") or "")
            if texture_name and not texture_name.startswith("%s_" % record["id"]):
                continue
            candidate = _child_asset_source(record, path)
            if candidate is not None:
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


#: 附加本体**自碰撞排除**的声明键（Profile `spec.model.self_collision_excludes`）。
#: 为什么必须有（2026-09-28 §11.23(41)，实测）：厂商 Piper MJCF 里**安装基座**是挂在 `world`
#: 下的网格，与首节连杆 `link1` 的网格**设计上必然重叠**（本臂实测 6 mm）。该自重叠在本场景
#: 冻结的 `impratio=100 / cone=elliptic` 口径下产生 **9.21×10⁸ N** 的接触力，把整条臂连同
#: joint1 的铅垂轴旋转一起钉死：joint1 指令 0.749801 rad 只走到 0.017582（2.3%），而同臂其余
#: 关节全部到位。**只排除这一对**后同一指令 Δq 从 0.000424 → 0.058217 rad（137 倍），与
#: "接触全关"完全同值 ⇒ 根因即此。声明形状：`[[body1, body2], ...]`（名字用**声明名**，
#: 注入时按"前缀存在才加前缀"的同一规则解析成联合模型名）。
def _declared_self_collision_excludes(profile_spec, robot_id):
    section = (profile_spec.get("model") or {}).get("self_collision_excludes")
    if section is None:
        return []
    if not isinstance(section, list) or not section:
        _fail(EXIT_DECLARATION,
              "robots.%s.spec.model.self_collision_excludes 必须是非空数组（缺省即不排除）" % robot_id)
    pairs = []
    for item in section:
        if (not isinstance(item, (list, tuple)) or len(item) != 2
                or not all(isinstance(v, str) and v for v in item)):
            _fail(EXIT_DECLARATION,
                  "robots.%s 的 self_collision_excludes 每项必须是 [body1, body2] 两个非空字符串"
                  % robot_id)
        pairs.append((str(item[0]), str(item[1])))
    return pairs


def _inject_self_collision_excludes(staging, scene, root, attached_records):
    """按声明把指定 body 对排除自碰撞（`<contact><exclude/>`）；未声明即**不注入**（返回 None）。"""
    pairs = []
    for record in attached_records or []:
        robot_id = str(record["id"])
        entity = resolve_robot(scene, robot_id)
        profile_path, _source = resolve_profile(root, entity, robot_id)
        _model_decl, profile_spec = load_build_declarations(profile_path, robot_id)
        prefix = str(record.get("prefix") or "")
        for body1, body2 in _declared_self_collision_excludes(profile_spec, robot_id):
            pairs.append((body1, body2, prefix))
    if not pairs:
        return None
    model = mujoco.MjModel.from_xml_path(str(staging))
    known = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
             for i in range(model.nbody)}

    def resolve(name, prefix):
        """名字解析：`<prefix>name` 存在就用它，否则用原名（与 name_map 的同一规则）。

        厂商模型里挂基座的 `world` 体在合成时并入主世界（名字仍是 `world`）⇒ 加前缀会不存在。
        """
        candidate = "%s%s" % (prefix, name)
        if prefix and candidate in known:
            return candidate
        if name in known:
            return name
        _fail(EXIT_REFERENCE,
              "self_collision_excludes 引用的 body %r（前缀候选 %r）不在合成模型里" % (name, candidate))

    resolved = []
    for body1, body2, prefix in pairs:
        resolved.append((resolve(body1, prefix), resolve(body2, prefix)))
    tree = ET.parse(str(staging))
    xml_root = tree.getroot()
    contact = xml_root.find("contact")
    if contact is None:
        contact = ET.SubElement(xml_root, "contact")
    for body1, body2 in resolved:
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})
    tree.write(str(staging), encoding="utf-8", xml_declaration=False)
    return {"pairs": [list(pair) for pair in resolved],
            "source": "Profile spec.model.self_collision_excludes"}


#: `robots[].reference_solver` 的必需键（缺键或缺文件即显式失败，不给默认值）：
#: `module`（仓内脚本路径）、`entry`（求解入口函数名）、`baseline`（该求解器的声明配置）。
#: gripper 段里的**语义开关**（非对象名，不参与前缀改写）：见 _joint_manipulation
SEMANTIC_GRIPPER_KEYS = ("close_hold", "approach_hold", "carry_gripper", "lift_path",
                         "regrasp", "carry_constraint", "place_settle_ms", "place_pose_correction",
                         "lift_gripper", "lift_constraint", "require_friction_lift",
                         "lift_anchor_body")

#: `robots[].reference_solver` 的必需键（缺键或缺文件即显式失败，不给默认值）：
REFERENCE_SOLVER_KEYS = ("module", "entry", "baseline")

#: 参考姿态的四个相位 → 联合报告里的**位置指令键**（求解器输出键与报告键的对应关系）。
REFERENCE_POSE_PHASES = (
    ("home", "home_positions"),
    ("approach", "approach_positions"),
    ("grasp", "grasp_positions"),
    ("lift", "lift_positions"),
)


def _quat_to_matrix(quat_wxyz):
    """wxyz 四元数 → 3×3 旋转矩阵（纯代数换算，不引入任何机型语义）。

    只用于把世界目标换算到**臂基座系**：`local = Rᵀ·(world − t)`。
    """
    quat = np.asarray([float(item) for item in quat_wxyz], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        _fail(EXIT_DECLARATION, "placement.quat_wxyz 是零四元数，无法求旋转矩阵")
    w, x, y, z = quat / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _load_declared_callable(root, declaration, robot_id, purpose):
    """按**声明**把仓内脚本文件当模块加载，并取回声明的入口函数（构建器不 import 任何机型专有模块）。

    为什么要动态加载而不是直接 import（AGENTS.md 6.3 "机型差异只进 profiles/config"）：
    本模块是平台无关的场景构建器，机型差异只能来自声明；把 `module` 声明成仓内路径、
    运行时加载，既保证"按声明调用"，又让本模块的名字表里不出现任何机型/脚本名。

    实测坑位：仓内脚本之间用 `from build_robot_baseline import …` 这种**平级导入**
    （脚本以 `python3 scripts/xxx.py` 运行时 `sys.path[0]` 就是脚本目录），
    因此加载前必须把**声明模块所在目录**放进 `sys.path`，否则被加载模块自身导入失败。
    """
    module_value = str(declaration.get("module") or "")
    entry_value = str(declaration.get("entry") or "")
    if not module_value or not entry_value:
        _fail(EXIT_DECLARATION, "robots.%s.%s 缺少 module/entry 声明" % (robot_id, purpose))
    module_path = Path(module_value)
    if not module_path.is_absolute():
        module_path = root / module_path
    module_path = module_path.resolve()
    if not module_path.is_file():
        _fail(EXIT_REFERENCE, "robots.%s.%s 声明的模块不存在: %s"
              % (robot_id, purpose, module_path))
    module_dir = str(module_path.parent)
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)
    module_name = "iraf_declared_%s_%s" % (str(purpose).replace(".", "_"), robot_id)
    spec = importlib.util.spec_from_file_location(module_name, str(module_path))
    if spec is None or spec.loader is None:
        _fail(EXIT_REFERENCE, "robots.%s.%s 声明的模块无法作为 Python 模块加载: %s"
              % (robot_id, purpose, module_path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as error:  # noqa: BLE001 —— 统一转成构建失败，带中文原因与出处
        _fail(EXIT_MODEL, "robots.%s.%s 声明的模块加载失败（%s）: %s"
              % (robot_id, purpose, module_path, error))
    entry = getattr(module, entry_value, None)
    if not callable(entry):
        _fail(EXIT_DECLARATION, "robots.%s.%s 声明的入口 %s 在模块 %s 里不存在或不可调用"
              % (robot_id, purpose, entry_value, module_path))
    return entry, module_path


def _measure_grip_height_m(model, gripper, targets):
    """在**已验证的 pick 抓取位形**上 FK，实测「指腹中点 − 载荷中心」= 夹口高度（m）。

    为什么必须实测（2026-09-29 §11.25(f-2)）：place 段原先复用 pick 侧的 `pad_offset_m`，而那个名字
    在 pick 侧指"指尖离台配平的修正量"（本场景 joint 解析 0.032016224），不是本段需要的
    `pad_mid − 载荷中心`（同场景实测 0.0437 − 0.025 = 0.0187）⇒ 相差 13.3 mm ⇒ 方块在承载面上方
    13.3 mm 被松手（再叠加托盘比名义低 7.6 mm = 实测 20.9 mm 落差）⇒ 落点双峰（托盘内/台面）。

    口径与判据/后端一致：指腹中点 = **双侧 pad geom 的中点**（声明名，非 body 中心）；
    载荷中心 = 目标 body 在联合模型里的 FK 位置（报告 `targets[].position_m`，来源 joint_model_fk）。
    返回 (grip_height_m, 证据 dict)。缺输入即显式失败（不猜、不用 0 兜底）。
    """
    positions = {str(k): float(v) for k, v in (gripper.get("grasp_positions") or {}).items()}
    pad_names = [str(gripper.get(key) or "") for key in ("left_finger_geom", "right_finger_geom")]
    if not positions:
        _fail(EXIT_REFERENCE, "无法实测夹口高度：报告 gripper 缺少 grasp_positions")
    if not all(pad_names):
        _fail(EXIT_REFERENCE, "无法实测夹口高度：报告 gripper 未声明 left/right_finger_geom")
    target = (targets or [{}])[0]
    if target.get("position_m") is None:
        _fail(EXIT_REFERENCE, "无法实测夹口高度：目标缺少 position_m（联合模型 FK 结果）")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for name, value in positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            _fail(EXIT_REFERENCE, "夹口高度实测：抓取位形里的关节 %r 不在联合模型里" % name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
    mujoco.mj_forward(model, data)
    points = []
    for name in pad_names:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            _fail(EXIT_REFERENCE, "夹口高度实测：指腹 geom %r 不在联合模型里" % name)
        points.append(np.asarray(data.geom_xpos[geom_id], dtype=float))
    pad_mid_z = float((points[0] + points[1])[2] / 2.0)
    payload_center_z = float(target["position_m"][2])
    height = pad_mid_z - payload_center_z
    # 合理性闸：夹口高度是"指腹在载荷中心上方多少"，本场景量级 1~5 cm；越界说明位形/位姿不匹配
    if not (0.0 < height < 0.2):
        _fail(EXIT_MODEL,
              "夹口高度实测不合理（%.9f m；pad_mid_z=%.9f − 载荷中心 z=%.9f）："
              "检查报告里的 grasp_positions 与 targets[].position_m 是否来自同一模型"
              % (height, pad_mid_z, payload_center_z))
    return height, {"grip_height_m": height, "pad_mid_z_m": pad_mid_z,
                    "payload_center_z_m": payload_center_z, "pad_geoms": pad_names,
                    "source": "FK@报告 gripper.grasp_positions（联合模型）",
                    "replaces": "pick 侧 pad_offset_m（含义不同，差 13.3 mm）"}


def _joint_reference_resolution(root, solver, placement, robot_id, targets, model, prefix):
    """在**臂基座系**重解参考姿态：目标换算 → **按声明**调用求解器 → 校验输出。

    背景（`docs/debug/2026-09-24-joint-model-dog-arm.md` §11.3–§11.5）：
    臂侧报告里的 `home/approach/grasp/lift_positions` 是在**臂自己基座系**里解出的关节解，
    而 MJCF 的 qpos 是**局部量** ⇒ 附加本体一旦有 `placement`（本场景 (0.45, −0.45, 0) 绕 z 90°），
    直接照搬会让指腹落到别处（实测残差 0.367696068 m，运行期表现为"末端未到达目标抓取位姿"）。
    把目标换算到基座系后重解 ⇒ 0.018389447 m；再把 `pad_offset_m` 一起重解 ⇒ 5.681e-06 m
    （pad_offset 不是通用常数：它由该场景的方块尺寸与指尖配平决定）。

    求解器契约（声明方 = 场景的 `robots[].reference_solver`）：
      · `entry(root, baseline_doc, target_xy_override_m=[x, y], target_z_override_m=z)`
        → 形如 `{home: {...}, approach: {joint_positions: {...}}, grasp:…, lift:…}`
        且含 `finger_height_correction_m`（= 该模型/该场景下的 `pad_offset_m`）；
      · 可选 `feedforward_entry(model, reference, baseline_doc, prefix)`
        → `({相位: {模型内关节名: ctrl 增量}}, 证据)`，用于在**运行期同款模型**上重算重力前馈
        （两个模型的执行器增益不同 ⇒ 继承前馈会按增益比例失真）。
      · `baseline` 是求解器自己的声明配置路径（本场景 `config/piper_simulation_baseline.yaml`）。

    **摆放松弛度**：目标按"基座系 xy + 世界 z"表达 ⇒ 只对**绕 z 的偏航摆放**自洽
    （有倾斜分量时"世界 z"不再是基座系的 z）。本函数显式检查这一点并失败，
    而不是把一个近似当成正确（要支持倾斜摆放需扩展契约）。
    """
    if not isinstance(solver, dict):
        _fail(EXIT_DECLARATION, "robots.%s.reference_solver 必须是对象" % robot_id)
    missing = [key for key in REFERENCE_SOLVER_KEYS if not solver.get(key)]
    if missing:
        _fail(EXIT_DECLARATION, "robots.%s.reference_solver 缺少必需键 %s"
              % (robot_id, missing))
    if not isinstance(placement, dict):
        _fail(EXIT_REFERENCE, "robots.%s 缺少 placement，无法把目标换算到臂基座系" % robot_id)
    pos = _vec3(placement.get("pos_m"), "robots.%s.placement.pos_m" % robot_id)
    quat = _quat_wxyz(placement.get("quat_wxyz"), "robots.%s.placement.quat_wxyz" % robot_id)
    rotation = _quat_to_matrix(quat)
    if (abs(float(rotation[2, 0])) > 1e-9 or abs(float(rotation[2, 1])) > 1e-9
            or abs(float(rotation[2, 2]) - 1.0) > 1e-9):
        _fail(EXIT_DECLARATION,
              "robots.%s.placement 含倾斜分量（R[2,:] = %s）：参考姿态求解器契约把目标表达为"
              "「基座系 xy + 世界 z」，只对绕 z 的偏航摆放自洽 ⇒ 需先扩展契约再构建"
              % (robot_id, [round(float(v), 9) for v in rotation[2]]))
    if not targets:
        _fail(EXIT_REFERENCE, "报告没有 targets[]，无法确定要重解哪个目标点的参考姿态")
    target = targets[0]
    body = str(target.get("body") or "")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) if body else -1
    if body_id < 0:
        _fail(EXIT_REFERENCE, "联合模型里找不到目标 body %r，无法重解参考姿态" % body)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    target_world = np.asarray(data.xpos[body_id], dtype=float)
    local = rotation.T @ (target_world - np.asarray(pos, dtype=float))
    # 按臂的**世界系目标覆盖**（I1b/I3，2026-09-29）：某些臂在本联合场景里的抓取目标**不是**场景
    # 道具的默认位置（实测：UR5e 基座 (0.45,0.90) 朝 −y，而默认目标台面方块 (0.28,−0.28) 距基座
    # 1.18 m > 可达 1.05 m ⇒ 姿态翻反向分支：夹爪指向偏差 171.715 deg / 开合轴 32.670 deg，limit 3 deg）。
    # 目标由 `robots[].reference_solver.target_override_world_m` 声明（世界系 xyz），缺省不覆盖
    # ⇒ 行为与改动前逐位一致。下面 entry 调用传的 xy/z override 由此**自动**变成覆盖后的值。
    _target_override = (solver or {}).get("target_override_world_m")
    if isinstance(_target_override, (list, tuple)) and len(_target_override) == 3:
        target_world = np.asarray([float(v) for v in _target_override], dtype=float)
        local = rotation.T @ (target_world - np.asarray(pos, dtype=float))

    entry, module_path = _load_declared_callable(root, solver, robot_id, "reference_solver")
    baseline_path = Path(str(solver["baseline"]))
    if not baseline_path.is_absolute():
        baseline_path = root / baseline_path
    baseline_path = baseline_path.resolve()
    if not baseline_path.is_file():
        _fail(EXIT_REFERENCE, "robots.%s.reference_solver.baseline 不存在: %s"
              % (robot_id, baseline_path))
    baseline_doc = _read_yaml(baseline_path, "robots.%s.reference_solver.baseline" % robot_id)
    # ⚠ **三个分量必须同口径**：全部是**臂基座系**的局部坐标（`local`，见上）。
    # 实测踩点（2026-09-28 §11.23(45)）：原先 z 传的是 `target_world[2]` ⇒ 基座在 z=0 时两种写法
    # 等价、这个错处一直休眠；把基座抬到 0.15 后，s03 立刻报"末端未到达目标抓取位姿
    # distance=0.150208 m"，z 偏差恰好等于基座高度（FK 实测指腹在 0.20392、方块 0.025、
    # 设计差应为 pad_offset 0.0289，实际 0.17892 ⇒ 多出 0.150）。
    try:
        reference = entry(root, baseline_doc,
                          target_xy_override_m=[float(local[0]), float(local[1])],
                          target_z_override_m=float(local[2]))
    except Exception as error:  # noqa: BLE001 —— 统一转成构建失败，带中文原因
        _fail(EXIT_MODEL, "robots.%s.reference_solver.entry(%s) 求解失败: %s"
              % (robot_id, str(solver["entry"]), error))
    if not isinstance(reference, dict):
        _fail(EXIT_MODEL, "robots.%s.reference_solver.entry 必须返回字典，实际 %r"
              % (robot_id, type(reference).__name__))

    resolved = {}
    for phase, positions_key in REFERENCE_POSE_PHASES:
        pose = reference.get(phase)
        if pose is None:
            _fail(EXIT_MODEL, "参考姿态求解结果缺少相位 %s（契约要求 home/approach/grasp/lift）"
                  % phase)
        raw = pose if phase == "home" else (pose.get("joint_positions") if isinstance(pose, dict)
                                           else None)
        if not isinstance(raw, dict) or not raw:
            _fail(EXIT_MODEL, "参考姿态相位 %s 的位置形状非法（home 需扁平字典，其余需 "
                  "joint_positions 字典）：%r" % (phase, pose))
        values = {}
        for name, value in raw.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                _fail(EXIT_MODEL, "参考姿态相位 %s 的关节 %s 不是数值：%r" % (phase, name, value))
            values[str(name)] = float(value)
        resolved[phase] = {"positions": values, "requested_key": positions_key}
    pad_offset = reference.get("finger_height_correction_m")
    if not isinstance(pad_offset, (int, float)) or isinstance(pad_offset, bool):
        _fail(EXIT_MODEL, "参考姿态求解结果缺少 finger_height_correction_m（= 该场景下的 "
              "pad_offset_m）：联合场景的 pad_offset 不是通用常数，必须随姿态一起重解")
    return {
        "robot_id": str(robot_id),
        "solver": {"module": str(module_path), "entry": str(solver["entry"]),
                   "baseline": str(baseline_path),
                   "feedforward_entry": (str(solver["feedforward_entry"])
                                         if solver.get("feedforward_entry") else None)},
        "placement": {"pos_m": [float(v) for v in pos],
                      "quat_wxyz": [float(v) for v in quat]},
        "target_world_m": [round(float(v), 9) for v in target_world],
        "local_target_m": [round(float(v), 9) for v in local],
        "local_xy_radius_m": round(float(np.linalg.norm(local[:2])), 9),
        "reference": reference,
        "baseline_doc": baseline_doc,
        "resolved": resolved,
        "pad_offset_m": round(float(pad_offset), 9),
    }


def _joint_reference_feedforward(root, solver, model, resolution, prefix, gripper_positions):
    """按声明在**联合模型**上重算重力前馈；`gripper_positions` 必须给该相位的完整位置指令。

    为什么与"参考姿态重解"分开：前馈要的输入是**四段位置指令的最终值**（臂关节来自重解、
    夹爪关节来自该相位声明的开合），而"最终值"要在四段位置合并之后才确定。
    为什么必须带夹爪（2026-09-24 实测踩坑）：只设臂关节时 joint7/8 停在默认/关键帧状态，
    闭合的指腹卡进 50 mm 方块（张开向量模长 0.020378284 m vs 0.090362481 m）⇒ 接触力把臂顶离姿态，
    量到的是"手指卡住的动力学"。判别法：量指腹张开向量模长。
    """
    if not solver.get("feedforward_entry"):
        return None, None
    ff_entry, _ff_path = _load_declared_callable(
        root, {"module": solver["module"], "entry": solver["feedforward_entry"]},
        robot_id=resolution["robot_id"], purpose="reference_solver.feedforward_entry")
    try:
        feedforward, evidence = ff_entry(model, resolution["reference"],
                                        resolution["baseline_doc"], prefix, gripper_positions)
    except Exception as error:  # noqa: BLE001 —— 含 core 的静态保持判据失败（残差即证据）
        _fail(EXIT_MODEL, "重力前馈在联合模型上重算失败（%s）: %s"
              % (str(solver["feedforward_entry"]), error))
    if not isinstance(feedforward, dict) or not feedforward:
        _fail(EXIT_MODEL, "重力前馈重算必须返回非空字典（相位 → {模型内关节名: 增量}）")
    return feedforward, evidence


def _joint_place_resolution(root, solver, resolution, place_targets, arm_report, out_gripper,
                            rename, model, prefix="", carrier_trunk="", pick_targets=None):
    """按声明解出**放置四段**（关节空间）并写进报告的 gripper 段。

    为什么构建期（§11.16）：已验证的 pick 能抬 88 mm 靠的是**构建期求解器**给的关节空间位形，
    后端只回放；运行时现解 IK 会落错分支、轨迹中途把载荷打掉。放置照同一条路做。

    局部目标 = 接收体**承载面中心**在**臂基座系**里的坐标（用 placement 的旋转把世界坐标换过去），
    参数（载荷半高 / pad_offset / 净间隙）全部来自声明或已解出的量，函数内不写死数字。
    """
    targets = (place_targets or {}).get("targets") or []
    if not targets:
        _fail(EXIT_REFERENCE, "场景报告没有 place_targets：无法解放置段（接收体必须声明 body/size_m）")
    entry, module_path = _load_declared_callable(
        root, {"module": solver["module"], "entry": solver["place_entry"]},
        resolution["robot_id"], "reference_solver.place_entry")
    baseline_path = Path(str(solver["baseline"]))
    if not baseline_path.is_absolute():
        baseline_path = root / baseline_path
    baseline_doc = _read_yaml(baseline_path.resolve(), "reference_solver.baseline")
    # 搬运段的**夹爪语义**：必须由声明给出（缺声明即失败）。它决定后端是否会在搬运中合拢夹口
    # 从而把载荷挤出夹口（§11.23(12)：hold = 目标取当前实测 qpos；trajectory = 按位置插值，
    # 实测会把方块沿夹口轴向挤出去：力 12 N → 2.6 N → 脱离）。**不给实现层默认值。**
    carry_mode = (baseline_doc.get("grasp") or {}).get("carry_gripper")
    if carry_mode not in ("hold", "trajectory"):
        _fail(EXIT_REFERENCE,
              "基线缺少 grasp.carry_gripper（只允许 hold / trajectory）：搬运段的夹爪语义必须由声明给出，"
              "不得在实现层写默认值（见 docs/debug/2026-09-24-joint-model-dog-arm.md §11.23(12)）")
    out_gripper["carry_gripper"] = {"mode": str(carry_mode),
                                    "source": "%s:grasp.carry_gripper" % baseline_path.name}
    placement = resolution["placement"]
    rotation = _quat_to_matrix(placement["quat_wxyz"])
    base_pos = np.asarray(placement["pos_m"], dtype=float)
    # 臂基座的高度取**模型实测**（放置目标在基座系里，z 必须相对同一个原点）
    record = None
    for item in targets:
        if item.get("nominal_pose_m") is None or item.get("size_m") is None:
            continue
        top_world = np.asarray(item["nominal_pose_m"], dtype=float) + np.asarray(
            [0.0, 0.0, float(item["size_m"][2])], dtype=float)
        local = rotation.T @ (top_world - base_pos)
        pad_offset = float(resolution["pad_offset_m"])
        # ---- 夹口高度：**构建期实测**（替代含义不同的 pick 侧 pad_offset，见 §11.25(f-2)）
        # ⚠ 必须用**联合模型 FK 出来的** pick 目标（`pick_targets[].position_m`，
        # 由 `_joint_manipulation` 在本模型上算得），不能用臂侧报告里的坐标
        # （那是臂自己场景的位置，联合场景可能不同）。
        grip_height, grip_evidence = _measure_grip_height_m(
            model, out_gripper, pick_targets or (arm_report or {}).get("targets") or [])
        # 触地间隙：必须由声明给出（缺声明即失败，不猜）
        touch_clearance = (baseline_doc.get("grasp") or {}).get("place_touch_clearance_m")
        if (not isinstance(touch_clearance, (int, float)) or isinstance(touch_clearance, bool)
                or float(touch_clearance) < 0.0):
            _fail(EXIT_REFERENCE,
                  "基线 %s 缺少 grasp.place_touch_clearance_m（触地间隙=下降终点让载荷最低点停在"
                  "承载面上方多少；缺声明即失败，不用 0 兜底）" % baseline_path.name)
        # 放置段的净间隙：优先用基线声明的 `grasp.place_clearance_m`（与 pick 的 pregrasp_offset 分开），
        # 缺声明则回退到 pregrasp_offset_m（行为与改动前一致，不新造数字）。
        place_clearance = ((baseline_doc.get("grasp") or {}).get("place_clearance_m"))
        clearance = float(place_clearance if isinstance(place_clearance, (int, float))
                          else (out_gripper.get("pregrasp_offset_m") or 0.0))
        payload_half = (arm_report.get("target_half_size_m")
                        if isinstance(arm_report.get("target_half_size_m"), (int, float)) else None)
        if payload_half is None:
            _fail(EXIT_REFERENCE, "臂侧报告缺少 target_half_size_m：放置段需要载荷半高，不给默认值")
        # 绕行航点的局部目标 = **当前抬升段的指腹中点 xy** + 承载面高度（+ 高度 + 间隙）。
        # 指腹中点用**联合模型 FK** 从报告的 lift_positions 实测得到（不抄数字）。
        lift_positions = {rename(name): float(value)
                          for name, value in (out_gripper.get("lift_positions") or {}).items()}
        pad_geoms = [str(out_gripper.get(k) or "")
                     for k in ("left_finger_geom", "right_finger_geom")]
        transit_world = None
        lift_axis_world = None
        if lift_positions and all(pad_geoms):
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            for name, value in lift_positions.items():
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id >= 0:
                    data.qpos[int(model.jnt_qposadr[joint_id])] = value
            mujoco.mj_forward(model, data)
            points = []
            for name in pad_geoms:
                geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                if geom_id < 0:
                    points = []
                    break
                points.append(np.asarray(data.geom_xpos[geom_id], dtype=float))
            if points:
                transit_world = (points[0] + points[1]) / 2.0
                wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                             str(out_gripper.get("wrist_body") or ""))
                if wrist_id >= 0:
                    axis = transit_world - np.asarray(data.xpos[wrist_id], dtype=float)
                    norm = float(np.linalg.norm(axis))
                    if norm > 1e-9:
                        lift_axis_world = [float(v) for v in (axis / norm)]
        transit_local = None
        if transit_world is not None:
            lift_local = rotation.T @ (transit_world - base_pos)
            transit_local = [float(lift_local[0]), float(lift_local[1]),
                             float(local[2] + float(payload_half) + grip_height + clearance)]
        # IK 种子 = 已验证的 pick **抬升位形**（把模型名还原成声明名：报告里是 piper_jointN）
        seed_positions = {}
        for name, value in (out_gripper.get("lift_positions") or {}).items():
            declared = str(name)[len(str(prefix)):] if prefix and str(name).startswith(str(prefix)) else str(name)
            seed_positions[declared] = float(value)
        try:
            poses = entry(root, baseline_doc, [float(v) for v in local], float(payload_half),
                          grip_height, clearance, float(touch_clearance),
                          **({"transit_local_m": transit_local} if transit_local else {}),
                          seed_positions=seed_positions)
        except Exception as error:  # noqa: BLE001
            _fail(EXIT_MODEL, "放置段求解失败（%s）: %s" % (solver["place_entry"], error))
        written = {}
        for phase, pose in (poses.get("poses") or {}).items():
            positions = {rename(name): float(value)
                         for name, value in (pose.get("joint_positions") or {}).items()}
            key = "place_%s_positions" % str(phase)
            _gkeys = _declared_gripper_keys(out_gripper)
            merged = {str(k): v for k, v in (out_gripper.get("lift_positions") or {}).items()
                      if str(k) in _gkeys}
            merged.update(positions)
            _assert_positions_are_joints(model, merged, key)
            out_gripper[key] = merged
            written[key] = {"replaced_joints": sorted(positions),
                            "position_error_m": pose.get("position_error_m")}
        # ---- 构建期自证（§11.25(f-2) ③）：下降终点必须把载荷**放到**承载面上，而不是悬空松手。
        # 为什么必须（实测教训）：上一版把方块放在承载面上方 20.9 mm 松手（自由落体 ⇒ 落点双峰、
        # 失败率约 1/4），而构建期一路"成功" ⇒ 只能靠 6 轮连跑才发现。这条闸把该量变成**构建期**判据。
        # 口径：FK 下降位形 → 载荷最低点 = pad_mid_z(下降) − grip_height − payload_half，
        #       要求 ≤ 承载面世界 z + 触地间隙 + 声明容差。
        descend_positions = out_gripper.get("place_descend_positions") or {}
        tolerance_m = (baseline_doc.get("acceptance") or {}).get("pose_tolerance_m")
        if not isinstance(tolerance_m, (int, float)):
            _fail(EXIT_REFERENCE, "基线 %s 缺少 acceptance.pose_tolerance_m（构建期自证需要容差）"
                  % baseline_path.name)
        if descend_positions:
            check_data = mujoco.MjData(model)
            mujoco.mj_forward(model, check_data)
            for name, value in descend_positions.items():
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name))
                if joint_id >= 0:
                    check_data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
            mujoco.mj_forward(model, check_data)
            check_points = []
            for pad_name in (str(out_gripper.get("left_finger_geom") or ""),
                             str(out_gripper.get("right_finger_geom") or "")):
                geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
                if geom_id >= 0:
                    check_points.append(np.asarray(check_data.geom_xpos[geom_id], dtype=float))
            if len(check_points) != 2:
                _fail(EXIT_MODEL, "构建期自证：下降位形下找不到声明的指腹 geom，无法核算落点高度")
            descend_pad_mid_z = float((check_points[0] + check_points[1])[2] / 2.0)
            predicted_payload_low = descend_pad_mid_z - grip_height - float(payload_half)
            allowed_low = float(top_world[2]) + float(touch_clearance) + float(tolerance_m)
            place_selfcheck = {
                "descend_pad_mid_z_m": round(descend_pad_mid_z, 9),
                "grip_height_m": round(float(grip_height), 9),
                "predicted_payload_low_m": round(predicted_payload_low, 9),
                "bearing_surface_z_m": round(float(top_world[2]), 9),
                "touch_clearance_m": float(touch_clearance),
                "allowed_low_m": round(allowed_low, 9),
                "gap_above_bearing_m": round(predicted_payload_low - float(top_world[2]), 9),
                "tolerance_m": float(tolerance_m),
                "grip_height_evidence": grip_evidence,
            }
            if predicted_payload_low > allowed_low:
                _fail(EXIT_MODEL,
                      "放置段构建期自证失败：下降终点把载荷停在承载面上方 %.6f m"
                      "（载荷最低点 %.9f > 允许 %.9f = 承载面 %.9f + 触地间隙 %.6f + 容差 %.6f）"
                      "⇒ 会变成自由落体（实测该情形下失败率约 1/4）。检查 grasp 侧"
                      "place_touch_clearance_m 与夹口高度实测值。"
                      % (predicted_payload_low - float(top_world[2]), predicted_payload_low,
                         allowed_low, float(top_world[2]), float(touch_clearance), float(tolerance_m)))
            out_gripper["place_selfcheck"] = place_selfcheck
        else:
            out_gripper["place_selfcheck"] = {"status": "skipped",
                                              "reason": "没有 place_descend_positions（未解放置段）"}

        # 航点间隙自检：把每段关节解设进**联合模型**做 FK，检查**臂的 geom**是否与**载体的 geom**
        # 接触（载体 = 名字不带该附加本体前缀的 body；台面/道具不算"载体"）。违规即显式失败 ——
        # 航点是关节解 ⇒ 构建期能 FK，这正是"构建期求解"相对"运行时现解"的优势（§11.17）。
        prefix = str(prefix or "")
        carrier_root = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                            str(carrier_trunk))) if carrier_trunk else -1

        def _in_carrier_subtree(body_id):
            """该 body 是否属于**载体**（主本体的躯干子树）。

            判定必须按**运动学子树**走，不能按"名字里没前缀"：后者会把 `world`（台面）也算成载体 ——
            实测第一版就把"臂底座 ↔ world"这条**正常固定接触**报成了撞载体（fail-closed 生效但过宽）。
            道具（方块/托盘）挂在躯干下 ⇒ 也算载体子树；而"载荷压在托盘上"是**预期**接触（载荷不是臂的 geom）。
            """
            if carrier_root < 0:
                return False
            current = int(body_id)
            while current > 0:
                if current == carrier_root:
                    return True
                current = int(model.body_parentid[current])
            return False

        waypoint_contacts = []
        data = mujoco.MjData(model)
        for phase_name, pose in (poses.get("poses") or {}).items():
            data.qpos[:] = 0.0
            mujoco.mj_forward(model, data)
            for joint_name, joint_value in (pose.get("joint_positions") or {}).items():
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                             rename(str(joint_name)))
                if joint_id >= 0:
                    data.qpos[int(model.jnt_qposadr[joint_id])] = float(joint_value)
            mujoco.mj_forward(model, data)
            for index in range(int(data.ncon)):
                contact = data.contact[index]
                bodies = [int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])]
                names = [str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, item) or "")
                         for item in bodies]
                arm_hit = [name for name in names if name.startswith(prefix)]
                carrier_hit = [name for name in names
                               if _in_carrier_subtree(bodies[names.index(name)])]
                if arm_hit and carrier_hit:
                    waypoint_contacts.append({"phase": str(phase_name), "arm_body": arm_hit[0],
                                              "other_body": carrier_hit[0],
                                              "dist_m": round(float(contact.dist), 6)})
        if waypoint_contacts:
            _fail(EXIT_MODEL, "放置航点与载体/场景发生接触（构建期 FK 自检）：%s ⇒ 会撞上并推走载体，"
                  "需调整航点或摆放（见 §11.17）" % waypoint_contacts[:3])
        # 夹爪轴半球**诊断**：每段的"腕→指腹"轴与**pick 抬升段**的轴比对（对照必须是被验证过的那一段，
        # 不能拿放置段自己当基准 —— 第一版就是拿 transit 自比 ⇒ 全部"通过"、什么也没发现）。
        # 轴翻到相反半球说明位置型 IK 落到了翻转分支；**根因可能是"该放置姿态在几何上不可达"**
        # （实测本场景：托盘顶面 0.449 m 处要求腕部再高 0.05 m ⇒ 0.67 m > 可达 0.594 m ⇒ IK 只能翻夹爪）。
        # 这里只**记录**不失败：place_object 能力尚未声明 ⇒ 这些解 inert；真正的门禁是运行期判据。
        axes = {str(name): pose.get("gripper_axis_world")
                for name, pose in (poses.get("poses") or {}).items()}
        axes_flipped = []
        if lift_axis_world is not None:
            for name, axis in axes.items():
                if not axis:
                    continue
                dot = sum(float(a) * float(b) for a, b in zip(axis, lift_axis_world))
                if dot < 0.0:
                    axes_flipped.append({"phase": name, "dot_with_pick_lift": round(dot, 6),
                                         "axis": axis})
        if axes_flipped:
            note = ("放置航点的夹爪轴相对 **pick 抬升段**翻到了相反半球（dot<0）：%s ⇒ 位置型 IK 落到了"
                    "翻转分支；本场景的根因是**该姿态几何不可达**（托盘顶面 0.449 m 处要求腕部再高约 0.05 m"
                    "⇒ 0.67 m > 可达 0.594 m，见 §11.18）⇒ 需要改场景/托盘高度或加朝向约束；"
                    "本段解在他处仍 inert（place_object 未声明）。" % axes_flipped[:3])
            print("[scene_builder] 警告：" + note, flush=True)
        record = {"source": "resolved_for_joint_model",
                  "solver": {"module": str(module_path), "entry": str(solver["place_entry"]),
                             "baseline": str(baseline_path.resolve())},
                  "place_target_id": item.get("id"),
                  "seed_positions": {str(k): round(float(v), 9) for k, v in seed_positions.items()},
                  "gripper_axes": axes,
                  "transit_local_m": ([round(float(v), 9) for v in transit_local]
                                      if transit_local else None),
                  "waypoint_contacts": waypoint_contacts,
                  "lift_axis_world": ([round(float(v), 9) for v in lift_axis_world]
                                      if lift_axis_world is not None else None),
                  "axes_flipped_vs_pick_lift": axes_flipped,
                  "target_local_m": [round(float(v), 9) for v in local],
                  "payload_half_m": float(payload_half), "pad_offset_m": pad_offset,
                  "clearance_m": clearance, "written": written,
                  "note": ("放置四段的关节解由**声明的求解器**按上述局部目标解出（构建期），"
                           "后端只回放；运行期以实测接收体位姿判据（见 place_object 证据）。")}
        break
    if record is None:
        _fail(EXIT_REFERENCE, "place_targets 里没有任何带 nominal_pose_m 与 size_m 的接收体")
    return record



def _declared_gripper_keys(gripper):
    """声明的**夹爪关节键**（`open_positions` / `closed_positions` 的键）。

    为什么需要（2026-09-29 实测）：重解只产出**臂关节**的关节解，夹爪键必须原样保留；
    而"先拷贝旧字典再 update"的写法会把**臂关节的旧键**一并留下 —— 对 UR5e 就是
    `ur5e_shoulder_pan`（执行器名，模型里确实存在）与 `ur5e_shoulder_pan_joint`（关节名）**两套共存**，
    轨迹回放按名解析时会拿到 -1（要么静默跳过、要么运行期报错）。Piper 未暴露：
    它的旧键与重解键同名、被 update 覆盖 ⇒ 只剩一套。
    """
    keys = set()
    for name in ("open_positions", "closed_positions"):
        section = gripper.get(name)
        if isinstance(section, dict):
            keys |= {str(key) for key in section}
    return keys


def _assert_positions_are_joints(model, positions, label):
    """构建期自证：位置字典的**每个键**都必须是联合模型里真实存在的**关节**（缺一即失败）。

    口径与后端一致（后端按 `name_map` → 模型关节名解析）；这里用编译后的模型直接校验，
    把"键名错一层"（执行器名/旧场景名）挡在构建期，而不是留给运行期的轨迹回放。
    """
    bad = [str(name) for name in positions
           if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, str(name)) < 0]
    if bad:
        _fail(EXIT_MODEL,
              "%s 的位置字典里这些键不是联合模型里的关节: %s"
              "（重解结果必须覆盖臂关节；夹爪键只能来自声明）" % (label, sorted(bad)))


def _joint_manipulation(root, arm_report_path, prefix, joint_facts, model, declared_names=None,
                        reference_solver=None, placement=None, robot_id=None, place_targets=None,
                        carrier_trunk=None):
    """联合报告的 manipulation 事实：继承**臂自己报告**的声明事实，并按**可判定规则**改写名字。

    改名规则（不猜）：某个名字 `n` 改写成 `<prefix>n` **当且仅当** `<prefix>n` 出现在联合模型的
    事实里、且 `n` 本身不在。这样臂自有对象（body/geom/joint）被正确指向联合模型里的实际名字，
    而场景道具（如 `box_01`）与场景几何保持原名 —— 避免照抄臂自己场景的命名（联合模型里不存在）。

    `name_map`（A 方案，2026-09-24）：把上述规则的结果**机械地**落成一张
    `声明名 → 联合模型名` 的表，使臂后端能按 Profile 名字（`joint1..8`）解析到联合模型里的对象
    （`piper_joint1..8`），而不需要把前缀泄漏进 Profile/技能参数。四个判定桶都会被记录：

      * `mapped`：只有 `prefix+n` 存在 ⇒ 进表（这是唯一会改名的情形）；
      * `identical`：只有 `n` 存在 ⇒ 不进表（声明名 == 模型名，无需映射）；
      * `conflict`：**两者都存在** ⇒ 不进表（映射有歧义：`n` 指的是主本体的对象，还是附加本体的？
        留痕到 `name_map_facts.conflicts`，供人工裁定；被 manipulation 直接引用的名字仍按原规则取 `n`）；
      * `missing`：两者都不存在 ⇒ 该对象在联合模型里不存在；**被 manipulation 引用的名字**按原规则
        显式失败（fail-closed），未被引用的（如被 attach 丢弃的对象）只留痕。
    """
    if not arm_report_path or not Path(arm_report_path).is_file():
        _fail(EXIT_REFERENCE,
              "联合构建需要臂侧场景报告以继承 manipulation 事实（不存在: %s）" % arm_report_path)
    report = json.loads(Path(arm_report_path).read_text(encoding="utf-8"))
    mj = mujoco
    name_map = {}

    def exists(name):
        """名字是否存在于**联合模型**里（任意 kind）——用编译后的模型校验，不看文本。"""
        text = str(name)
        # ⚠ ACTUATOR 也要在内（2026-09-29 实测）：`attach` 同样给**执行器**加前缀
        # （联合模型里是 `ur5e_rq2f85_fingers_actuator`），而臂侧报告的 gripper 会引用执行器名
        # ⇒ 原来只查 body/geom/site/joint/camera/material/texture 会让第二台臂的段解不出来
        #（实测：`…引用的名字 rq2f85_fingers_actuator 在联合模型里既不存在原名也不存在 ur5e_ 前缀名`）。
        for kind in (mj.mjtObj.mjOBJ_BODY, mj.mjtObj.mjOBJ_GEOM, mj.mjtObj.mjOBJ_SITE,
                     mj.mjtObj.mjOBJ_JOINT, mj.mjtObj.mjOBJ_ACTUATOR, mj.mjtObj.mjOBJ_CAMERA,
                     mj.mjtObj.mjOBJ_MATERIAL, mj.mjtObj.mjOBJ_TEXTURE):
            if mj.mj_name2id(model, kind, text) >= 0:
                return True
        return False

    known = set()
    for bucket in ("bodies", "geoms", "sites", "joints", "actuators", "cameras"):
        for name in ((joint_facts or {}).get(bucket) or []):
            known.add(str(name))

    def rename(name):
        text = str(name)
        prefixed = "%s%s" % (prefix, text)
        if exists(prefixed) and not exists(text):
            name_map[text] = prefixed
            return prefixed
        if exists(text):
            return text
        # fail-closed：既不原名存在、也不带前缀存在 ⇒ 名字无法指向联合模型里的对象
        _fail(EXIT_REFERENCE,
              "联合报告的 manipulation 引用的名字 %s 在联合模型里既不存在原名也不存在 %s 前缀名"
              "（臂侧报告与联合模型的命名不一致 ⇒ 装配会指错对象）" % (text, prefix))

    gripper = report.get("gripper")
    if not isinstance(gripper, dict) or not gripper:
        _fail(EXIT_REFERENCE, "臂侧场景报告缺少 gripper 段，无法继承: %s" % arm_report_path)
    out_gripper = {}
    for key, value in gripper.items():
        if key in SEMANTIC_GRIPPER_KEYS:
            # **语义开关**（不是对象名）：原样继承，不参与前缀改写 ——
            # 否则会被当成名字去联合模型里查，必然 fail-closed（实测 close_hold: pin_payload）。
            out_gripper[key] = value
            continue
        if isinstance(value, str):
            out_gripper[key] = rename(value)
        elif isinstance(value, dict) and value and all(isinstance(item, dict) for item in value.values()):
            # **嵌套字典**（如 `gravity_feedforward: {相位: {关节: 增量}}`）：外层键是相位名
            # （不是模型对象，**不改名**），内层键才是关节名（要改名）。
            # 实测踩点（2026-09-24）：只处理"字符串/数值字典"时，这里会把相位名 `home` 当成对象名
            # 去 rename ⇒ 报"名字 home 在联合模型里既不存在原名也不存在 piper_ 前缀名"、退出码 3，
            # 把整个联合构建挡死（症状看着像命名问题，其实是"没预料到的字典形状"）。
            out_gripper[key] = {
                str(phase): {rename(name): float(item) for name, item in inner.items()}
                for phase, inner in value.items()
            }
        elif isinstance(value, dict):
            out_gripper[key] = {rename(k): float(v) for k, v in value.items()}
        else:
            out_gripper[key] = value
    targets = []
    # ⚠ 目标的**世界位姿必须来自联合模型的 FK**，不能照抄臂侧报告的数值：方块在联合场景里的位置由
    # `scene.yaml` 的 props 声明决定，与臂自己场景可能不同（本场景已按使用者要求把方块移到臂旁边：
    # 0.19 → (0.28, −0.28)）。运行期 `grasp_pose_from: report_target` 会按本报告组装世界抓取位姿 ⇒
    # 继承旧坐标会指到另一个地方（这正是"继承场景专属数据"这一类缺陷）。
    _target_data = mujoco.MjData(model)
    mujoco.mj_forward(model, _target_data)
    for item in (report.get("targets") or []):
        entry = dict(item)
        for key in ("body", "geom", "material"):
            if isinstance(entry.get(key), str):
                entry[key] = rename(entry[key])
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(entry.get("body") or ""))
        if body_id >= 0:
            entry["position_m"] = [round(float(v), 9) for v in _target_data.xpos[body_id]]
            entry["quaternion_wxyz"] = [round(float(v), 9) for v in _target_data.xquat[body_id]]
            entry["pose_source"] = "joint_model_fk"
        else:
            entry["pose_source"] = "inherited_from_arm_report"
        targets.append(entry)
    # ---- 全量判定桶（A 方案 name_map 的输入）：对附加本体**自己模型里的每个对象名**套同一条规则。
    # 只对被 manipulation 引用的名字 fail-closed（上方 rename），其余只留痕 —— attach 可能丢弃对象，
    # 把它当构建失败会误伤；但"声明名既不在原名也不在前缀名里"必须能被人看见。
    buckets = {"mapped": [], "identical": [], "conflicts": [], "missing": []}
    declared = {"bodies": [], "geoms": [], "sites": [], "joints": [], "actuators": []}
    for bucket in declared:
        for value in ((declared_names or {}).get(bucket) or []):
            text = str(value)
            declared[bucket].append(text)
            prefixed = "%s%s" % (prefix, text)
            has_prefixed = exists(prefixed)
            has_plain = exists(text)
            if has_prefixed and has_plain:
                buckets["conflicts"].append({"kind": bucket, "declared": text, "model": prefixed})
            elif has_prefixed:
                buckets["mapped"].append({"kind": bucket, "declared": text, "model": prefixed})
                name_map.setdefault(text, prefixed)
            elif has_plain:
                buckets["identical"].append({"kind": bucket, "declared": text, "model": text})
            else:
                buckets["missing"].append({"kind": bucket, "declared": text})
    facts = {
        "rule": "声明名→联合模型名：只对 <prefix>name 存在且 name 不存在的情形映射",
        "counts": {key: len(value) for key, value in buckets.items()},
        "mapped": buckets["mapped"], "identical": buckets["identical"],
        "conflicts": buckets["conflicts"], "missing": buckets["missing"],
        "declared_names_source": "附加本体自身模型（attach 之前采集）",
        "declared_object_counts": {key: len(value) for key, value in declared.items()},
        "note": ("conflict（同名对象在主本体与附加本体里都存在）**不进 name_map**：映射有歧义，"
                 "需人工裁定；missing 只留痕，被 manipulation 直接引用的名字已在 rename() 里显式失败。"),
    }
    # ---- 按声明重解参考姿态（联合模式）----
    # 为什么必须（§11.3/§11.4 实测 0.367696068 m）：继承来的 `*_positions` 是在**臂自己基座系**
    # 里解出的关节解（MJCF 的 qpos 是局部量）⇒ 换基座位姿后照搬，指腹会落到别处，
    # 运行期表现为"末端未到达目标抓取位姿"。求解器**不在本模块内**：由场景声明给出
    # （`robots[].reference_solver`），本模块只做"目标换算到臂基座系 + 按声明调用 + 覆写报告"，
    # 名字表里不出现任何机型/脚本名（AGENTS.md 6.3）。
    # 缺声明 ⇒ 显式失败，**不**回退到静默继承（静默继承正是这次 0.368903942 m 的来源）。
    if not reference_solver:
        _fail(EXIT_DECLARATION,
              "附加本体 %s 声明了 manipulation_report（继承臂侧 gripper/targets）⇒ 必须同时声明 "
              "`reference_solver`（联合模式参考姿态求解器）：继承来的参考姿态是**场景专属**的"
              "（取决于附加本体的 placement 与本场景的目标几何），联合模型必须重解；"
              "静默继承曾造成 0.367696068 m 的抓取残差" % robot_id)
    resolution = _joint_reference_resolution(
        root, reference_solver, placement, robot_id, targets, model, prefix)
    replaced = {}
    for phase, positions_key in REFERENCE_POSE_PHASES:
        positions = dict(resolution["resolved"][phase]["positions"])
        _gkeys = _declared_gripper_keys(out_gripper)
        merged = {str(k): v for k, v in (out_gripper.get(positions_key) or {}).items()
                  if str(k) in _gkeys}
        if not merged and not _gkeys:
            _fail(EXIT_REFERENCE, "臂侧报告的 gripper 缺少 %s 且未声明夹爪键，无法在它之上覆写重解结果"
                  % positions_key)
        diff = []
        for name, value in positions.items():
            model_name = rename(name)  # 复用同一条改名规则（不存在即显式失败）
            previous = merged.get(model_name)
            if previous is None or abs(float(previous) - float(value)) > 0.0:
                diff.append({"joint": model_name,
                             "inherited": (None if previous is None else float(previous)),
                             "resolved": float(value)})
            merged[model_name] = float(value)
        out_gripper[positions_key] = merged
        replaced[phase] = {"positions_key": positions_key,
                           "replaced_joints": [item["joint"] for item in diff],
                           "diff_vs_inherited": diff}
    # **regrasp 两段必须同样按联合模型重解**（2026-09-29 §11.23(48)）：它们此前**不在覆写清单里**
    # ⇒ 原样继承臂侧世界的位形（臂基座在原点、方块在腕高的那个世界）⇒ 在"臂被抬高 0.12 m"的联合
    # 世界里同一组关节角指向完全不同的位置：实测 pre_lift_delta_m 0.174019（声明 0.04）、
    # 工具多走 175 mm、载荷被顶到夹口上方 ⇒ regrasp 随机失败。
    # 判据（构建期自证）：当 placement 把臂**抬高**（z ≠ 0）时，重解结果**必须**与继承值不同
    # ⇒ 相同即 fail-closed（这类"静默继承"不可能再溜过去）。
    regrasp_reference = (resolution["reference"].get("regrasp") or {})
    if regrasp_reference:
        lift_z = float(placement.get("pos_m")[2]) if isinstance(placement.get("pos_m"), (list, tuple)) \
            else 0.0
        for label, positions_key, pose_key in (("pre_lift", "pre_lift_positions", "pre_lift"),
                                               ("regrasp", "regrasp_positions", "regrasp")):
            pose = regrasp_reference.get(pose_key)
            merged = dict(out_gripper.get(positions_key) or {})
            if not pose or not merged:
                continue
            diff = []
            for name, value in (pose.get("joint_positions") or {}).items():
                model_name = rename(name)
                previous = merged.get(model_name)
                if previous is None or abs(float(previous) - float(value)) > 0.0:
                    diff.append({"joint": model_name,
                                 "inherited": (None if previous is None else float(previous)),
                                 "resolved": float(value)})
                merged[model_name] = float(value)
            out_gripper[positions_key] = merged
            replaced[label] = {"positions_key": positions_key,
                               "replaced_joints": [item["joint"] for item in diff],
                               "diff_vs_inherited": diff}
            if lift_z != 0.0 and not diff:
                _fail(EXIT_REFERENCE,
                      "regrasp.%s 的联合重解与臂侧继承值**逐位相同**，而 placement 抬高了 %.6f m "
                      "⇒ 说明重解没有真正发生（场景专属位形被静默继承，§11.23(48) 的同类缺陷）"
                      % (label, lift_z))
    inherited_pad_offset = out_gripper.get("pad_offset_m")
    out_gripper["pad_offset_m"] = float(resolution["pad_offset_m"])
    # 放置段（`place_object`）的接近/抬离间隙：取臂侧**已声明**的 `pregrasp_offset_m`
    # （臂报告 reference_poses 段），不新造数字；缺声明即显式失败（放置需要它）。
    pregrasp_offset = (report.get("reference_poses") or {}).get("pregrasp_offset_m")
    if not isinstance(pregrasp_offset, (int, float)) or float(pregrasp_offset) <= 0:
        _fail(EXIT_REFERENCE,
              "臂侧报告缺少 reference_poses.pregrasp_offset_m（正数）：放置段的接近间隙只能来自声明，"
              "不在实现层写默认值（%s）" % arm_report_path)
    out_gripper["pregrasp_offset_m"] = float(pregrasp_offset)
    # 出处标注：读到 `resolved_for_joint_model` 即表示这些数字来自**本联合模型 + 本 placement**，
    # 不是从臂自己场景继承来的（`inherited_from` 仍留痕，便于追溯）。
    out_gripper["reference_pose_source"] = "resolved_for_joint_model"
    # ---- 放置四段（**构建期**解出关节空间解，后端只回放；理由见 §11.16）----
    place_record = None
    if reference_solver and reference_solver.get("place_entry") and place_targets:
        place_record = _joint_place_resolution(
            root, reference_solver, resolution, place_targets, report, out_gripper, rename,
            model, prefix=prefix, carrier_trunk=carrier_trunk, pick_targets=targets)
    feedforward_source = "inherited_from_arm_report"
    feedforward_evidence = None
    # 前馈在**四段位置合并之后**算：`gripper_positions` 必须是该相位的完整位置指令
    # （臂关节 = 重解值，夹爪 joint7/8 = 该相位声明的开合），否则量的是"手指卡在方块里"的动力学。
    if reference_solver and reference_solver.get("feedforward_entry"):
        gripper_positions = {phase: dict(out_gripper.get(positions_key) or {})
                             for phase, positions_key in REFERENCE_POSE_PHASES}
        feedforward, feedforward_evidence = _joint_reference_feedforward(
            root, reference_solver, model, resolution, prefix, gripper_positions)
        gravity_feedforward = {}
        for phase, positions_key in REFERENCE_POSE_PHASES:
            offsets = (feedforward or {}).get(phase)
            if not offsets:
                continue
            declared_keys = set(out_gripper.get(positions_key) or {})
            unknown = sorted(set(offsets) - declared_keys)
            if unknown:
                _fail(EXIT_MODEL,
                      "重算的重力前馈 %s 含该段位置指令里不存在的通道 %s（前馈键必须与位置指令"
                      "一一对应：前缀漏掉会让前馈静默失效，表现为「精度莫名不达标」）"
                      % (phase, unknown))
            gravity_feedforward[phase] = {str(name): float(value)
                                          for name, value in offsets.items()}
        out_gripper["gravity_feedforward"] = gravity_feedforward
        feedforward_source = "resolved_for_joint_model"
    # FK 自检（构建期，防"静默继承场景专属参考姿态"）----
    # 为什么必须（2026-09-24 实测，docs/debug/2026-09-24-joint-model-dog-arm.md §11.3/§11.4）：
    # 上一轮继承来的 `*_positions` 是**在臂自己基座系里求解的关节解**（MJCF 的 qpos 是局部量）；
    # 换到联合模型的基座位姿后直接照搬，指腹会落到别处（实测残差 0.368903942 m），而构建期
    # 毫无提示 ⇒ 一直藏到运行期、以"末端未到达目标抓取位姿"的形式出现。
    # 这里用**联合模型 + 报告里的命令位姿**做一次 FK，把残差与判读写进报告（阈值取目标自己声明的
    # `pose_tolerance_m`，不新造数字）；重解生效后该残差应回到 µm 量级。
    # 声明容差的来源：臂侧报告的**顶层** `pose_tolerance_m`（该报告里 target 条目本身不带它；
    # 运行期门禁用的是机型声明的 `target_tolerance_m`，两者同值 0.005 ⇒ 取臂报告这一处即可）。
    reference_pose_check = _reference_pose_check(
        model, out_gripper, targets,
        declared_pose_tolerance_m=report.get("pose_tolerance_m"))
    # 侵入自检（同一次 FK）：指腹中点对了不等于姿态可用 —— 臂的其它 geom 可能已经插进方块。
    reference_pose_clearance = _reference_pose_clearance_check(model, out_gripper, targets, prefix)
    return {"gripper": out_gripper, "targets": targets,
            "target_id": (targets[0]["id"] if targets else report.get("target_id")),
            "vision": report.get("vision"),
            "name_map": dict(sorted(name_map.items())),
            "name_map_facts": facts,
            "reference_pose_check": reference_pose_check,
            "reference_pose_clearance_check": reference_pose_clearance,
            "reference_pose_resolution": {
                "source": "resolved_for_joint_model",
                "solver": resolution["solver"],
                "placement": resolution["placement"],
                "target_world_m": resolution["target_world_m"],
                "local_target_m": resolution["local_target_m"],
                "local_xy_radius_m": resolution["local_xy_radius_m"],
                "resolved_pad_offset_m": resolution["pad_offset_m"],
                "inherited_pad_offset_m": inherited_pad_offset,
                "place_reference_source": (place_record or {}).get("source"),
                "place_reference": place_record,
                "feedforward_source": feedforward_source,
                "feedforward_evidence": feedforward_evidence,
                "replaced_phases": replaced,
                "note": ("参考姿态与 pad_offset 均由**联合模型 + 本 placement** 重解"
                         "（求解器按 `robots[].reference_solver` 声明调用）；"
                         "`resolved` 的关节解是**臂基座系**下的关节角，MJCF qpos 也是局部量 ⇒ 可直接命令。"),
            },
            "inherited_from": str(Path(arm_report_path)), "rename_rule":
                "改写成 <prefix>name 当且仅当 <prefix>name 在联合模型事实里、且 name 不在"}


def _reference_pose_clearance_check(model, gripper, targets, prefix):
    """重解姿态下"臂与目标几何是否侵入"的构建期判据（**无阈值**：接触即侵入）。

    为什么必须（2026-09-24 实测）：`_reference_pose_check` 只核对**指腹中点**的位置（本场景 5.663e-06
    通过），看不见"臂的其它 geom 已经插进方块"：
      · `box_01_geom ↔ 未命名臂 geom` 重叠 **−0.01626 m**（0.52 m 摆放、指腹张开时）；
      · 关键帧手指状态下 `box_01_geom ↔ piper_left/right_finger` 重叠 **−0.021008 / −0.02345 m**。
    这种侵入会让接触力把臂顶离姿态（静态保持残余 0.0205~0.0522 rad，限 0.001），
    而运行期只表现为"末端未到达目标抓取位姿 / 未确认已抓取"。

    判据口径**与臂自己场景一致**（`scripts/build_piper_baseline.validate_grasp_pose`：
    "张开时手指不与任何物体接触"）——因此不新造容差：**存在接触对即违反**，并把重叠量写成数字。
    返回 `{"checked": True, "overlapping": bool, "violations": [...]}`；缺输入时 `{"skipped": 原因}`。
    """
    if not targets:
        return {"skipped": "报告没有 targets[]，无法确定目标体"}
    target = targets[0]
    box_body = str(target.get("body") or "")
    box_geom = str(target.get("geom") or "")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box_body) if box_body else -1
    box_geom_id = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, box_geom)
                   if box_geom else -1)
    if box_id < 0 or box_geom_id < 0:
        return {"skipped": "联合模型里找不到目标 body/geom: %r / %r" % (box_body, box_geom)}
    positions = {str(joint): float(value)
                 for joint, value in (gripper.get("grasp_positions") or {}).items()}
    if not positions:
        return {"skipped": "gripper 段没有 grasp_positions，无法做侵入检查"}
    pads = {str(gripper.get(key) or "") for key in ("left_finger_geom", "right_finger_geom")}

    def describe(geom_id):
        """geom 的标签与归属：**必须**走 `geom_bodyid`，不能靠名字前缀。

        实测坑位（2026-09-24）：联合模型里 `piper_link1..link6` 的 geom **全部没有名字**
        （id 60–66；厂商 MJCF 只给 finger geom 命名，本构建器也只命名 finger geom），
        于是"名字带 piper_ 前缀"的过滤把真正侵入方块的 `<未命名#66>(body=piper_link6)`
        （与 `box_01_geom` 重叠 **−0.014516 m**）**静默跳过** —— 检查报"无接触"，实际有。
        """
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        body_id = int(model.geom_bodyid[geom_id])
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        label = str(name) if name else "<未命名#%d>" % geom_id
        return label, (str(body) if body else "<未命名 body#%d>" % body_id)

    data = mujoco.MjData(model)
    for joint, value in positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if joint_id < 0:
            return {"skipped": "联合模型里找不到关节 %r" % joint}
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
    mujoco.mj_forward(model, data)
    violations = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        ids = (int(contact.geom1), int(contact.geom2))
        if box_geom_id not in ids:
            continue
        other_id = ids[1] if ids[0] == box_geom_id else ids[0]
        label, body = describe(other_id)
        # 只关心**附加本体**的 geom（按宿主 body 的前缀界定）；台面/道具/主本体的接触不属本判据。
        if not body.startswith(str(prefix)):
            continue
        violations.append({"geom": label, "body": body, "is_pad": label in pads,
                           "dist_m": round(float(contact.dist), 6)})
    violations.sort(key=lambda item: item["dist_m"])
    return {
        "checked": True,
        "target_body": box_body, "target_geom": box_geom,
        "commanded_grasp": {key: round(value, 9) for key, value in positions.items()},
        "arm_geoms_touching_target": violations,
        "pad_touching_target": [item for item in violations if item["is_pad"]],
        "non_pad_touching_target": [item for item in violations if not item["is_pad"]],
        "overlapping": bool(violations),
        "note": ("归属按 `geom_bodyid` → 宿主 body 的前缀判定（**不看 geom 名**：本场景臂的 link geom 无名，"
                 "按名字过滤会漏掉真正侵入方块的那一个 —— 实测 `<未命名#66>(piper_link6)` 与 "
                 "`box_01_geom` 重叠 −0.014516 m）。判据与臂自己场景一致"
                 "（`validate_grasp_pose`：张开时手指不与任何物体接触）："
                 "**存在接触对即几何侵入**（无阈值）。侵入会把臂顶离姿态 ⇒ 静态保持不达标，"
                 "而运行期只表现为「未到达抓取位姿 / 未确认已抓取」；修法在求解器层"
                 "（朝向约束 + 碰撞校验），见 docs/debug/2026-09-24-joint-model-dog-arm.md §11.6。"),
    }


def _reference_pose_check(model, gripper, targets, declared_pose_tolerance_m=None):
    """用**联合模型** + 继承来的命令位姿做 FK，量指腹中点与目标抓取点的残差。

    判据口径与运行期门禁**完全一致**（`MujocoBackend._grasp_alignment_evidence`）：
        center = pad中点 − 轴·pad_offset_m ；要求 |center − 目标中心| ≤ pose_tolerance_m
    （阈值取目标自己声明的 `pose_tolerance_m`，本函数不新造数字。）
    残差超阈值不是"构建失败"，而是**构建期可见的诊断**：它说明继承的参考姿态与这份模型的
    基座位姿不匹配（本场景实测 0.368903942 m ⇒ 需要在联合模型上重算，见 §11.4）。

    缺任一必需输入（位姿/指腹 geom/目标位置）时返回 `{"skipped": 原因}`，不猜、也不静默当成通过。
    """
    import math as _math

    import numpy as np

    def _pad_midpoint(data):
        points = []
        for key in ("left_finger_geom", "right_finger_geom"):
            name = str(gripper.get(key) or "")
            ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) if name else -1
            if ident < 0:
                return None, "联合模型里找不到声明的指腹 geom: %r" % name
            points.append(np.asarray(data.geom_xpos[ident], dtype=float))
        return (points[0] + points[1]) / 2.0, None

    positions = {str(joint): float(value)
                 for joint, value in (gripper.get("grasp_positions") or {}).items()}
    if not positions:
        return {"skipped": "继承的 gripper 段没有 grasp_positions，无法做 FK 自检"}
    if not targets:
        return {"skipped": "报告没有 targets[]，无法确定目标中心"}
    target = targets[0]
    body = str(target.get("body") or "")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) if body else -1
    if body_id < 0:
        return {"skipped": "联合模型里找不到目标 body: %r" % body}

    data = mujoco.MjData(model)
    for joint, value in positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if joint_id < 0:
            return {"skipped": "联合模型里找不到关节 %r（位姿键与模型不一致）" % joint}
        data.qpos[int(model.jnt_qposadr[joint_id])] = value
    mujoco.mj_forward(model, data)
    midpoint, error = _pad_midpoint(data)
    if midpoint is None:
        return {"skipped": error}
    center = np.asarray(data.xpos[body_id], dtype=float)
    axis = np.asarray(gripper.get("pad_offset_axis") or [0.0, 0.0, 1.0], dtype=float)
    norm = float(np.linalg.norm(axis))
    axis = axis / norm if norm > 1e-9 else np.asarray([0.0, 0.0, 1.0], dtype=float)
    offset = float(gripper.get("pad_offset_m") or 0.0)
    residual = midpoint - axis * offset - center
    tolerance = (target.get("pose_tolerance_m")
                 if isinstance(target.get("pose_tolerance_m"), (int, float))
                 else declared_pose_tolerance_m)
    distance = float(np.sqrt(float(residual @ residual)))
    return {
        "target_id": target.get("id"),
        "target_body": body,
        "target_center_m": [round(float(v), 9) for v in center],
        "pad_midpoint_m": [round(float(v), 9) for v in midpoint],
        "pad_offset_m": offset,
        "pad_offset_axis": [round(float(v), 9) for v in axis],
        "residual_m": [round(float(v), 9) for v in residual],
        "distance_m": round(distance, 9),
        "declared_pose_tolerance_m": (None if not isinstance(tolerance, (int, float))
                                      else float(tolerance)),
        "within_declared_tolerance": (None if not isinstance(tolerance, (int, float))
                                      else bool(distance <= float(tolerance))),
        "note": ("继承的 `*_positions` 若来自别的基座位姿，本残差会显著大于声明容差 ⇒ "
                 "必须在**联合模型**上重算参考姿态（见 docs/debug/2026-09-24-joint-model-dog-arm.md §11.4）"),
    }


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
        xml_root, tree, staging, [item for item in props if item.get("has_free_joint")]
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
    # 附加本体的**自碰撞排除**（按声明注入 `<contact><exclude>`；缺声明即不注入）
    self_collision_excludes = _inject_self_collision_excludes(staging, scene, root, attached_robots)
    injections["self_collision_excludes"] = self_collision_excludes
    # 物理口径：联合世界必须由**声明**给出，不得继承厂商模型 <option>（§11.23）
    world_physics = _apply_world_physics(staging, scene, required=bool(attach))
    # 搬运段抓取约束（声明驱动，惰性注入；见 _inject_carry_constraint）
    carry_constraint_injection = _inject_carry_constraint(staging, scene, root) if attach else None
    if world_physics is not None:
        world_physics["effective"] = _read_world_physics(staging)
        # 读回比对：数值按 1e-9 容差比（声明 1 与读回 1.0 是同一个值，不能当成不一致），
        # 枚举/数组分别比；不一致即显式失败（声明 ≠ 生效是最危险的静默失效）。
        mismatch = {}
        for key, declared_value in world_physics["declared"].items():
            effective_value = world_physics["effective"].get(key)
            if isinstance(declared_value, (list, tuple)):
                ok = (isinstance(effective_value, (list, tuple))
                      and len(effective_value) == len(declared_value)
                      and all(abs(float(a) - float(b)) <= 1e-9
                              for a, b in zip(declared_value, effective_value)))
            elif isinstance(declared_value, (int, float)) and not isinstance(declared_value, bool):
                ok = (isinstance(effective_value, (int, float))
                      and abs(float(declared_value) - float(effective_value)) <= 1e-9)
            else:
                ok = str(declared_value) == str(effective_value)
            if not ok:
                mismatch[str(key)] = [declared_value, effective_value]
        world_physics["mismatch"] = mismatch
        if mismatch:
            _fail(EXIT_MODEL, "声明的物理口径未生效：%s" % mismatch)
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

    # 接收体声明（承载面：托盘随载体运动 ⇒ 只给名字 + 几何 + 标称停靠位姿；见 _place_targets）
    place_targets = _place_targets(scene, compiled, injections, trunk_body_name)
    report = {
        # 物理口径（声明 + 实测 + 不一致检查）：见 _apply_world_physics / §11.23
        "world_physics": world_physics,
        # 搬运段抓取约束的注入事实（惰性；由后端按报告声明激活）
        "carry_constraint": carry_constraint_injection,
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
        "place_targets": place_targets,
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
    if attached_robots:
        # 联合报告：把臂侧的 manipulation 事实（gripper/targets）按**可判定规则**改名后并入，
        # 使 `robot.backend_config.mode: scene_report` 能直接指向本报告装配臂后端
        # （`output` 已是联合产物路径）。
        # 联合报告顶层 `manipulation`/`gripper`/`targets` 只描述**一台**臂（臂后端按它装配）。
        # 多台臂各自带 `manipulation_report` 时**必须由声明指明是哪一台**（scene.model.joint_manipulator）：
        # 原先按 `--attach` 顺序取第一台 ⇒ 第二台的 gripper/targets/name_map 被**静默丢弃**，
        # 而"哪台是主臂"这种语义绝不能由命令行顺序隐式决定（AGENTS.md 2.3 / 5.3）。
        arm_with_report = []
        for attached in attach:
            entity = resolve_robot(scene, attached)
            if entity.get("manipulation_report"):
                arm_with_report.append(str(attached))
        model_section = (scene or {}).get("model") or {}
        declared_primary = (model_section.get("joint_manipulator")
                            if isinstance(model_section, dict) else None)
        declared_primary = str(declared_primary) if declared_primary else None
        if declared_primary is not None and declared_primary not in arm_with_report:
            _fail(EXIT_DECLARATION,
                  "scene.model.joint_manipulator 声明为 %s，但它不是本次 --attach 里带 "
                  "`manipulation_report` 的本体（%s）" % (declared_primary, arm_with_report or "无"))
        if len(arm_with_report) > 1 and declared_primary is None:
            _fail(EXIT_DECLARATION,
                  "本次 --attach 有 %d 台本体带 `manipulation_report`（%s），但场景未声明 "
                  "`model.joint_manipulator` —— 联合报告的顶层 manipulation/gripper/targets 只能描述一台，"
                  "缺声明会导致其余本体的操纵事实被静默丢弃（不接受按 --attach 顺序隐式决定）"
                  % (len(arm_with_report), arm_with_report))
        if not arm_with_report:
            _fail(EXIT_DECLARATION,
                  "使用 --attach 时必须在本体声明 `manipulation_report`（继承 gripper/targets 的"
                  "来源；缺声明即失败，不猜）")
        robot_id = declared_primary or arm_with_report[0]
        prefix = None
        arm_report = None
        arm_declared_names = None
        arm_reference_solver = None
        arm_placement = None
        entity = resolve_robot(scene, robot_id)
        arm_report = root / str(entity.get("manipulation_report"))
        prefix = "%s_" % robot_id
        # 联合模式参考姿态求解器与绑定位姿**都来自声明**（构建器只按声明调用）。
        arm_reference_solver = entity.get("reference_solver")
        arm_placement = entity.get("placement")
        for record in attached_robots:
            if str(record.get("id")) == str(robot_id):
                arm_declared_names = record.get("declared_names")
        manipulation = _joint_manipulation(root, arm_report, prefix, facts, compiled,
                                           arm_declared_names,
                                           reference_solver=arm_reference_solver,
                                           placement=arm_placement, robot_id=robot_id,
                                           place_targets=place_targets,
                                           carrier_trunk=trunk_body_name)
        # ---- I1b：**按本体**给出 manipulation 段（2026-09-29）
        # 为什么必须（多臂）：顶层 manipulation/gripper/targets 只能描述一台臂。第二台臂若要跑技能，
        # 必须能从**它自己**的段里解析 name_map/参考姿态/夹持事实 —— 否则要么被 `scenario.py` 的
        # 所有权门禁拦下，要么被错绑到主臂的对象上（实测：ur5e 会解析到 `piper_left_finger`，
        # 于是"夹爪张开"实际作用在 piper 上，且**不会报错**）。
        # 这里对**每台**带 `manipulation_report` 的附加臂各算一份；主臂复用已算好的（不重复求解），
        # 顶层仍保留主臂的段（向后兼容，piper 侧逐位不变）。
        # ⚠ 非主臂解不出时**不做**静默降级：整条失败原因写进 `per_robot[<id>].error`，且
        # `resolved=False` —— 调用方（scenario.py）据此拒绝装配，失败要早于任何运动。
        per_robot = {}
        for attached in attach:
            attached_entity = resolve_robot(scene, attached)
            attached_report = attached_entity.get("manipulation_report")
            if not attached_report:
                continue
            attached_id = str(attached)
            if attached_id == str(robot_id):
                record = manipulation
            else:
                declared_names = None
                for item in attached_robots:
                    if str(item.get("id")) == attached_id:
                        declared_names = item.get("declared_names")
                try:
                    record = _joint_manipulation(
                        root, root / str(attached_report), "%s_" % attached_id, facts, compiled,
                        declared_names,
                        reference_solver=attached_entity.get("reference_solver"),
                        placement=attached_entity.get("placement"), robot_id=attached_id,
                        place_targets=place_targets, carrier_trunk=trunk_body_name)
                except (Exception, SystemExit) as error:  # noqa: BLE001
                    per_robot[attached_id] = {
                        "resolved": False, "prefix": "%s_" % attached_id,
                        "manipulation_report": str(attached_report),
                        "error": "%s: %s" % (type(error).__name__, str(error)[:400])}
                    continue
            _override = ((attached_entity.get("reference_solver") or {})
                         .get("target_override_world_m"))
            _first_target = ((record.get("targets") or [{}]) or [{}])[0]
            per_robot[attached_id] = {
                "resolved": True, "prefix": "%s_" % attached_id,
                "manipulation_report": str(attached_report),
                # **本场景的抓取目标**（I3，2026-09-29）：位置取声明覆盖（缺则退回目标体的 FK 位姿），
                # 朝向取目标体在联合模型里的姿态。用途：运行期 `grasp_pose_from: joint_scene_target`
                # 据此构造 `pick_object` 的判据位姿 —— 对"托盘里的活体载荷"而言，臂侧报告里的
                # 初始位姿是错的（那是台面上的位置），必须用本场景的目标。
                "joint_scene_target_m": (
                    [float(v) for v in _override] if isinstance(_override, (list, tuple)) and len(_override) == 3
                    else _first_target.get("position_m")),
                "joint_scene_target_quat_wxyz": _first_target.get("quaternion_wxyz"),
                "gripper": record.get("gripper"), "targets": record.get("targets"),
                "target_id": record.get("target_id"), "vision": record.get("vision"),
                "name_map": record.get("name_map"), "name_map_facts": record.get("name_map_facts"),
                "rename_rule": record.get("rename_rule"),
                "inherited_from": record.get("inherited_from"),
                "reference_pose_check": record.get("reference_pose_check"),
                "reference_pose_clearance_check": record.get("reference_pose_clearance_check"),
                "reference_pose_resolution": record.get("reference_pose_resolution"),
            }
        reference_pose_check = manipulation.get("reference_pose_check")
        report["gripper"] = manipulation["gripper"]
        report["targets"] = manipulation["targets"]
        report["target_id"] = manipulation["target_id"]
        report["vision"] = manipulation["vision"]
        # `name_map`：声明名 → 联合模型名的**机械**产物（臂后端据此按 Profile 名解析对象）；
        # `name_map_facts`：四个判定桶的留痕（mapped/identical/conflicts/missing）。
        # `reference_pose_resolution`：参考姿态/pad_offset/重力前馈的**重解留痕**（求解器出处、
        # 绑定位姿、基座系目标、与继承值的差）；缺它说明这些数字是继承来的 —— 本构建已不允许。
        # 每台带 `manipulation_report` 的附加本体都留一条记录：主臂 `resolved: true`，
        # 其余 `resolved: false` + 原因 —— "还没为它做参考姿态重解"必须是**显式状态**，
        # 不能靠"顶层字段里没有它"来表达（那正是被静默丢弃的形态）。
        attached_manipulators = []
        for attached in attach:
            attached_entity = resolve_robot(scene, attached)
            if not attached_entity.get("manipulation_report"):
                continue
            is_primary = str(attached) == str(robot_id)
            attached_manipulators.append({
                "id": str(attached),
                "prefix": "%s_" % attached,
                "manipulation_report": str(attached_entity.get("manipulation_report")),
                "reference_solver": attached_entity.get("reference_solver"),
                "has_placement": bool(attached_entity.get("placement")),
                "resolved": is_primary,
                # `name_map`（声明名 → 联合模型名）当前只按主臂的声明名生成 ⇒ 非主臂必须**显式**
                # 说明"没包含"，不能让调用方把"空 name_map"误读成"这台臂没有可解析的名字"。
                "name_map_included": is_primary,
                "note": ("本报告的顶层 manipulation/gripper/targets/name_map 取自本本体"
                         "（scene.model.joint_manipulator）"
                         if is_primary else
                         "已在同一物理世界里装配（几何/执行器/资产都在），但本报告**没有**为它解"
                         "参考姿态/gripper/name_map 事实（scene.model.joint_manipulator 未指向它）"
                         "⇒ 在联合世界里跑它的技能前必须先补按本体的 manipulation 段；"
                         "它的技能验收属于后续增量"),
            })
        report["manipulation"] = {"attached_robot": str(robot_id),
                                  "declared_primary": declared_primary,
                                  # I1b：按本体的 manipulation 段（每台带 manipulation_report 的附加臂一份）
                                  "per_robot": per_robot,
                                  "attached_manipulators": attached_manipulators,
                                  "inherited_from": manipulation["inherited_from"],
                                  "rename_rule": manipulation["rename_rule"],
                                  "name_map": manipulation["name_map"],
                                  "name_map_facts": manipulation["name_map_facts"],
                                  "reference_pose_check": reference_pose_check,
                                  "reference_pose_clearance_check":
                                      manipulation["reference_pose_clearance_check"],
                                  "reference_pose_resolution":
                                      manipulation["reference_pose_resolution"]}
        report["manipulation_absent_reason"] = None
    output_path.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    return report
