"""场景传感器验收实现层（步骤 14）：相机内参/离屏渲染、雷达射线统计、IMU 与力矩传感器读数。

为什么单独成模块（而不是写进 `scripts/verify_scene_sensors.py`）：
验收要能被逐项断言（"每条判据的实测值/判据/通过与否都在报告里"），
因此实现必须是可导入的库；`scripts/verify_scene_sensors.py` 只做参数解析与退出码映射，
与步骤 08/09/10 的"CLI 入口 + 实现层"分层惯例一致，也与步骤 13 的
`scripts/build_scene.py` + `iraf_adapters.unitree.scene_builder` 同构。

职责边界：

- 只做"读数 + 与声明比对 + 落证据"，**不含任何机型专有名称**：
  本体、传感器、厂商模型路径、躯干 body、IMU site 全部来自
  `scenes/<id>/scene.yaml`、`scenes/<id>/baseline.yaml` 与 `profiles/<robot>_mujoco.yaml`；
  阈值全部来自 `baseline.yaml` 的 `sensor_acceptance` 段（缺段/缺键即显式失败，不给默认值）。
- 被测产物是步骤 13 生成的 MJCF（`scene.yaml` 的 `model.output`）；本模块**不**重新构建场景，
  只校验"生成物里的传感器与声明一致"，并把产物 SHA-256 写进报告（证据与产物绑定）。
- 诚实边界：本模块的所有结论都属于仿真（报告 `simulation: true`）。
  目标端/真机验收不在本模块职责内（板卡不在场 → DEFERRED）。
  本场景**无平衡控制器**：`settle.steps` 只用于证明"仿真推进链路可用且采样状态准静态"，
  不代表四足具备站立/步态能力（能力验收属步骤 16/17）。
- 相机离屏渲染不可用时（EGL/GL 环境问题）：相机部分标 `blocked` 并写明环境原因，
  雷达与 IMU 部分继续完成（按步骤 14「失败/阻塞处理」的要求）。

报告契约：`iraf.scene-sensor-evidence/v1`，默认写到
`build/acceptance/<场景 id 连字符化>-scene-sensors/report.json`。
"""

import hashlib
import json
import os
import platform
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from iraf_adapters.unitree import scene_builder as sb  # noqa: E402

REPORT_SCHEMA_VERSION = "iraf.scene-sensor-evidence/v1"
REPORT_KIND = "scene_sensor_evidence"

# 退出码（由 scripts/verify_scene_sensors.py 映射；库内错误自带 code，便于验收逐条核对）。
EXIT_USAGE = 1
EXIT_DECLARATION = 2
EXIT_REFERENCE = 3
EXIT_RENDER = 4
EXIT_FAILED = 5

REQUIRED_SECTIONS = ("settle", "camera", "lidar", "imu")

#: 实现层的必需键（与 config/scene.schema.json 的 `required` 必须同文；两份门禁缺一不可：
#: schema 拦声明，这里拦"schema 被绕过/旧声明"的情况）。缺键即显式失败，不读默认值。
REQUIRED_KEYS = {
    "settle": ("steps", "max_base_drift_m"),
    "camera": (
        "model_vs_declaration_rel_error_max",
        "calibration_fovy_rel_error_max",
        "calibration_focal_rel_error_max",
        "resolution_tolerance_px",
        "min_nonblack_fraction",
    ),
    "lidar": (
        "elevation_rings_deg",
        "range_histogram_edges_m",
        "ground_geoms",
        "min_points",
        "max_miss_fraction",
        "min_ground_fraction",
        "workbench_geometry_tolerance_m",
        "ray_box_tolerance_m",
    ),
    "imu": (
        "quat_norm_tolerance",
        "gyro_max_abs_error_rad_s",
        "acc_rel_error_max",
        "torque_max_abs_error_nm",
    ),
}

#: 传感器类型 → 声明 kind 的映射（用 MuJoCo 的 sensor_type 而不是名字来识别传感器，
#: 名字是厂商的自由命名，类型才是模型语义）。
SENSOR_TYPES = {
    "quat": mujoco.mjtSensor.mjSENS_FRAMEQUAT,
    "gyro": mujoco.mjtSensor.mjSENS_GYRO,
    "accelerometer": mujoco.mjtSensor.mjSENS_ACCELEROMETER,
    "joint_torque": mujoco.mjtSensor.mjSENS_JOINTACTFRC,
}


class SensorEvidenceError(ValueError):
    """传感器验收失败（显式失败，不降级、不伪造成功）。"""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


def _fail(code, message):
    raise SensorEvidenceError(code, message)


def _load_yaml(path, label):
    path = Path(path)
    if not path.is_file():
        _fail(EXIT_REFERENCE, "%s 不存在: %s" % (label, path))
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        _fail(EXIT_DECLARATION, "%s 解析失败: %s（%s）" % (label, path, exc))


def _rel_error(measured, reference):
    if reference == 0:
        _fail(EXIT_DECLARATION, "参考值为 0，无法计算相对误差（声明非法）")
    return abs(float(measured) - float(reference)) / abs(float(reference))


def _name(model, objtype, index):
    return mujoco.mj_id2name(model, objtype, index)


def _sensor_values(model, data, sensor_id):
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return np.asarray(data.sensordata[address:address + dimension], dtype=float).copy()


def _sensors_of(model, sensor_type, objtype, objid):
    """按**类型 + 绑定对象**找传感器（不按名字），返回 sensor id 列表。"""
    found = []
    for index in range(model.nsensor):
        if int(model.sensor_type[index]) != int(sensor_type):
            continue
        if int(model.sensor_objtype[index]) != int(objtype):
            continue
        if int(model.sensor_objid[index]) != int(objid):
            continue
        found.append(index)
    return found


def _one_sensor(model, sensor_type, objtype, objid, label, kind_name):
    found = _sensors_of(model, sensor_type, objtype, objid)
    names = [_name(model, mujoco.mjtObj.mjOBJ_SENSOR, item) for item in found]
    if not found:
        _fail(EXIT_REFERENCE, "生成模型里没有绑定到 %s 的 %s 传感器（模型缺传感器声明）" % (label, kind_name))
    if len(found) > 1:
        _fail(EXIT_DECLARATION, "%s 上声明了多个 %s 传感器，无法唯一确定: %s" % (label, kind_name, names))
    return found[0], names[0]


def _named_object(model, objtype, name, label):
    index = mujoco.mj_name2id(model, objtype, str(name))
    if index < 0:
        _fail(EXIT_REFERENCE, "生成模型里找不到 %s: %r（声明与模型不一致）" % (label, name))
    return index


def _body_tree(model, body_id):
    """返回 body 及其全部后代的 id 集合（用于把"打到本体自己"与"打到道具"区分开）。"""
    members = {int(body_id)}
    for index in range(model.nbody):
        ancestor = int(model.body_parentid[index])
        while ancestor > 0:
            if ancestor in members:
                members.add(int(index))
                break
            ancestor = int(model.body_parentid[ancestor])
    return members


def _ray_box_distance(origin, direction, center, half):
    """射线与轴对齐盒体的最近正向交点（slab 法）；无交返回 None。

    只用于与**声明**（scene.yaml 的 workbench 段）比对，不作为几何事实来源：
    几何事实来自 `mujoco.mj_ray`，两者之差就是本步骤的一条判据。
    """
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    center = np.asarray(center, dtype=float)
    half = np.asarray(half, dtype=float)
    t_min, t_max = -np.inf, np.inf
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if abs(origin[axis] - center[axis]) > half[axis]:
                return None
            continue
        first = (center[axis] - half[axis] - origin[axis]) / direction[axis]
        second = (center[axis] + half[axis] - origin[axis]) / direction[axis]
        t_min = max(t_min, min(first, second))
        t_max = min(t_max, max(first, second))
    if t_max < max(t_min, 0.0):
        return None
    return float(t_min if t_min > 0.0 else t_max)


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _default_renderer(model, height, width):
    """真实离屏渲染器（默认实现）。单测注入替身时，报告的 render.implementation 字段会留痕。"""
    return mujoco.Renderer(model, height=height, width=width)


def _thresholds(baseline, baseline_path):
    section = baseline.get("sensor_acceptance")
    if not isinstance(section, dict):
        _fail(
            EXIT_DECLARATION,
            "%s 缺少 sensor_acceptance 段：阈值必须来自声明，脚本内不得写死默认值" % baseline_path,
        )
    for key in REQUIRED_SECTIONS:
        if not isinstance(section.get(key), dict):
            _fail(EXIT_DECLARATION, "sensor_acceptance.%s 必须是对象（缺段即失败）" % key)
    for key in REQUIRED_SECTIONS:
        missing = [name for name in REQUIRED_KEYS[key] if name not in section[key]]
        if missing:
            _fail(
                EXIT_DECLARATION,
                "sensor_acceptance.%s 缺少必需阈值 %s：阈值必须来自声明，不得用默认值"
                % (key, missing),
            )
    return section


def _pick_sensor(scene, kind, robot=None):
    """按 kind 取唯一声明的传感器；0 个或多个都显式失败（不猜）。"""
    items = []
    for sensor in scene.get("sensors") or []:
        if str(sensor.get("kind")) != str(kind):
            continue
        if robot is not None and str((sensor.get("anchor") or {}).get("entity")) != str(robot):
            continue
        items.append(sensor)
    if not items:
        _fail(
            EXIT_DECLARATION,
            "场景 %s 没有声明 kind=%s 的传感器（本验收要求相机/雷达/IMU 三类均有声明）"
            % (scene.get("id"), kind),
        )
    if len(items) > 1:
        _fail(
            EXIT_DECLARATION,
            "场景声明了多个 kind=%s 传感器，无法唯一确定: %s"
            % (kind, sorted(str(item.get("id")) for item in items)),
        )
    return items[0]


class _Checks:
    """判据台账：每条判据都留 measured/criterion/passed，报告里可逐条核对。"""

    def __init__(self):
        self.items = []

    def add(self, ident, measured, criterion, passed, detail=""):
        self.items.append(
            {
                "id": ident,
                "measured": measured,
                "criterion": criterion,
                "passed": bool(passed),
                "detail": detail,
            }
        )
        return bool(passed)

    @property
    def failed(self):
        return [item["id"] for item in self.items if not item["passed"]]


def _camera_section(model, data, scene, baseline, root, thresholds, checks, render, renderer=None):
    """相机：声明 → 模型 回环 + 标定内参一致性 + 离屏渲染一次。"""
    sensor = _pick_sensor(scene, "camera")
    sensor_id = str(sensor["id"])
    name = str(sensor["anchor"]["name"])
    camera = _named_object(model, mujoco.mjtObj.mjOBJ_CAMERA, name, "相机")
    entity = str(sensor["anchor"]["entity"])
    if entity not in [str(item.get("id")) for item in scene.get("robots") or []]:
        _fail(EXIT_REFERENCE, "相机 %s 的 anchor.entity=%s 不是场景内本体" % (sensor_id, entity))

    # MuJoCo 的 cam_fovy 单位是度（compiler angle="radian" 不影响它）：这里显式写清，
    # 避免"按弧度解读"这类静默错误（既有 target_detection 用 deg2rad 解读，口径一致）。
    fovy_model_deg = float(model.cam_fovy[camera])
    fovy_declared_deg = float(sensor["fovy_deg"])
    declared_error = _rel_error(fovy_model_deg, fovy_declared_deg)
    checks.add(
        "camera.fovy_model_matches_declaration",
        declared_error,
        "<= %g" % float(thresholds["camera"]["model_vs_declaration_rel_error_max"]),
        declared_error <= float(thresholds["camera"]["model_vs_declaration_rel_error_max"]),
        "scene.yaml 的 fovy_deg 必须逐位体现在生成的 camera 上",
    )

    # 标定文件链路：scene.yaml 的 anchor.entity → baseline.robots.<entity> → 机型基线的
    # vision.detector.calibration_file → focal_px / image_size_px。全部来自声明，无硬编码路径。
    machine_ref = (baseline.get("robots") or {}).get(entity)
    if not isinstance(machine_ref, str):
        _fail(
            EXIT_REFERENCE,
            "baseline.robots.%s 不是机型基线路径（当前 %r）：无法解析相机标定文件链路"
            % (entity, machine_ref),
        )
    machine = _load_yaml(root / machine_ref, "机型基线")
    calibration_rel = (((machine.get("vision") or {}).get("detector") or {}).get("calibration_file"))
    if not calibration_rel:
        _fail(
            EXIT_DECLARATION,
            "%s 的 vision.detector.calibration_file 未声明：相机内参验收必须声明标定文件" % machine_ref,
        )
    calibration_file = root / str(calibration_rel)
    if not calibration_file.is_file():
        _fail(
            EXIT_REFERENCE,
            "标定文件不存在: %s（机型基线 %s 声明）—— 相机内参验收需要本机型标定证据"
            % (calibration_rel, machine_ref),
        )
    calibration = json.loads(calibration_file.read_text(encoding="utf-8"))
    for key in ("focal_px", "image_size_px", "principal_point_px"):
        if key not in calibration:
            _fail(EXIT_DECLARATION, "标定文件 %s 缺少 %s" % (calibration_rel, key))

    image_size = [int(value) for value in calibration["image_size_px"]]
    width, height = image_size
    focal_calibration_px = float(calibration["focal_px"])
    focal_model_px = 0.5 * float(height) / float(np.tan(np.radians(fovy_model_deg) * 0.5))
    fovy_implied_deg = 2.0 * float(np.degrees(np.arctan(0.5 * float(height) / focal_calibration_px)))
    fovy_rel_error = _rel_error(fovy_implied_deg, fovy_model_deg)
    focal_rel_error = _rel_error(focal_calibration_px, focal_model_px)
    checks.add(
        "camera.calibration_fovy_consistency",
        fovy_rel_error,
        "<= %g" % float(thresholds["camera"]["calibration_fovy_rel_error_max"]),
        fovy_rel_error <= float(thresholds["camera"]["calibration_fovy_rel_error_max"]),
        "标定 focal_px 反推的视场角与模型 fovy 的相对误差（跨机型复用标定会在此暴露）",
    )
    checks.add(
        "camera.calibration_focal_consistency",
        focal_rel_error,
        "<= %g" % float(thresholds["camera"]["calibration_focal_rel_error_max"]),
        focal_rel_error <= float(thresholds["camera"]["calibration_focal_rel_error_max"]),
        "标定 focal_px 与理想针孔 focal=(H/2)/tan(fovy/2) 的相对误差",
    )

    section = {
        "id": sensor_id,
        "name": name,
        "entity": entity,
        "status": "checked",
        "fovy_model_deg": fovy_model_deg,
        "fovy_declared_deg": fovy_declared_deg,
        "model_vs_declaration_rel_error": declared_error,
        "calibration_file": str(calibration_rel),
        "calibration_image_size_px": image_size,
        "focal_calibration_px": focal_calibration_px,
        "focal_model_px": focal_model_px,
        "focal_rel_error": focal_rel_error,
        "fovy_implied_by_calibration_deg": fovy_implied_deg,
        "fovy_rel_error": fovy_rel_error,
        "principal_point_px": [float(value) for value in calibration["principal_point_px"]],
        "render": {"status": "not_attempted"},
    }
    renderer_factory = renderer or _default_renderer
    renderer_source = getattr(renderer_factory, "__name__", type(renderer_factory).__name__)
    if not render:
        section["render"] = {
            "status": "disabled_by_usage",
            "reason": "--no-render 由调用方显式指定",
            "implementation": renderer_source,
        }
        section["status"] = "blocked"
        return section

    import time

    layout = _named_object(model, mujoco.mjtObj.mjOBJ_CAMERA, name, "相机")
    started = time.time()
    try:
        renderer = renderer_factory(model, height, width)
        try:
            renderer.update_scene(data, camera=name)
            frame = renderer.render().copy()
        finally:
            renderer.close()
    except Exception as exc:  # noqa: BLE001 - 环境原因要原样上报，不静默跳过
        elapsed = time.time() - started
        section["status"] = "blocked"
        section["render"] = {
            "status": "blocked",
            "gl": os.environ.get("MUJOCO_GL"),
            "implementation": renderer_source,
            "error": "%s: %s" % (type(exc).__name__, exc),
            "elapsed_s": elapsed,
            "camera_index": int(layout),
        }
        checks.add(
            "camera.offscreen_render",
            "blocked",
            "必须产出 1 帧离屏图像",
            False,
            "离屏渲染不可用（EGL/GL 环境原因原样上报）；雷达与 IMU 部分继续执行",
        )
        return section

    elapsed = time.time() - started
    frame_array = np.asarray(frame, dtype=float)
    nonblack = float(np.mean(frame_array.sum(axis=2) > 0))
    resolution_diff = abs(int(frame.shape[0]) - height) + abs(int(frame.shape[1]) - width)
    checks.add(
        "camera.render_resolution_matches_declaration",
        resolution_diff,
        "<= %d 像素" % int(thresholds["camera"]["resolution_tolerance_px"]),
        resolution_diff <= int(thresholds["camera"]["resolution_tolerance_px"]),
        "渲染帧尺寸必须等于标定声明的 image_size_px",
    )
    checks.add(
        "camera.render_nonblack_fraction",
        nonblack,
        ">= %g" % float(thresholds["camera"]["min_nonblack_fraction"]),
        nonblack >= float(thresholds["camera"]["min_nonblack_fraction"]),
        "非全黑像素占比：证明确实产出了图像而不是黑帧",
    )
    section["render"] = {
        "status": "ok",
        "gl": os.environ.get("MUJOCO_GL"),
        "implementation": renderer_source,
        "shape_hw": [int(frame.shape[0]), int(frame.shape[1])],
        "resolution_diff_px": resolution_diff,
        "nonblack_fraction": nonblack,
        "mean_pixel_value": float(frame_array.mean()),
        "frame_sha256": hashlib.sha256(np.ascontiguousarray(frame).tobytes()).hexdigest(),
        "elapsed_s": elapsed,
    }
    return section


def _lidar_section(model, data, scene, robot, trunk_body, thresholds, checks):
    """雷达：按声明的仰角圈做光线投射，统计有效点数/量程分布/台面占比，并与声明解析求交比对。"""
    sensor = _pick_sensor(scene, "lidar", robot=robot)
    sensor_id = str(sensor["id"])
    site_name = str(sensor["anchor"]["name"])
    site = _named_object(model, mujoco.mjtObj.mjOBJ_SITE, site_name, "雷达挂载 site")
    num_rays = int(sensor["num_rays"])
    range_m = float(sensor["range_m"])
    rings = [float(value) for value in thresholds["lidar"]["elevation_rings_deg"]]
    if not rings:
        _fail(EXIT_DECLARATION, "sensor_acceptance.lidar.elevation_rings_deg 不能为空")
    if num_rays % len(rings) != 0:
        _fail(
            EXIT_DECLARATION,
            "num_rays=%d 不能被仰角圈数 %d 整除：每圈方位数无法确定（scene.yaml 与 baseline.yaml 不一致）"
            % (num_rays, len(rings)),
        )
    per_ring = num_rays // len(rings)

    prop_bodies = {str(item["body"]) for item in scene.get("props") or []}
    ground_geoms = [str(value) for value in thresholds["lidar"]["ground_geoms"]]
    ground_geom_ids = {}
    for name in ground_geoms:
        ground_geom_ids[name] = _named_object(model, mujoco.mjtObj.mjOBJ_GEOM, name, "地平面 geom")
    robot_bodies = _body_tree(model, trunk_body)

    # 声明 → 模型 回环：注入的工作台 geom 必须与 scene.yaml 的 workbench 段一致。
    workbench = (scene.get("terrain") or {}).get("workbench") or {}
    if "workbench" in ground_geom_ids:
        geom_id = ground_geom_ids["workbench"]
        declared_center = np.array(
            [0.0, 0.0, float(workbench["top_z_m"]) - float(workbench["half_thickness_m"])]
        )
        declared_half = np.array(
            [
                float(workbench["half_size_m"]),
                float(workbench["half_size_m"]),
                float(workbench["half_thickness_m"]),
            ]
        )
        position_error = float(np.abs(np.asarray(model.geom_pos[geom_id]) - declared_center).max())
        size_error = float(np.abs(np.asarray(model.geom_size[geom_id]) - declared_half).max())
        tolerance = float(thresholds["lidar"]["workbench_geometry_tolerance_m"])
        checks.add(
            "lidar.workbench_geometry_matches_declaration",
            max(position_error, size_error),
            "<= %g m" % tolerance,
            max(position_error, size_error) <= tolerance,
            "生成模型里的 workbench geom 位姿/半边长 vs scene.yaml 的 terrain.workbench 声明"
            "（位置误差 %.3e、尺寸误差 %.3e）" % (position_error, size_error),
        )
        if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            _fail(
                EXIT_REFERENCE,
                "声明的 ground_geom=%s 不是 box（无法按声明做解析求交对照）" % name,
            )
    else:
        _fail(
            EXIT_DECLARATION,
            "scene_acceptance.lidar.ground_geoms 未包含 workbench：无法把台面声明与注入结果对照",
        )

    origin = np.asarray(data.site_xpos[site], dtype=float).copy()
    rotation = np.asarray(data.site_xmat[site], dtype=float).reshape(3, 3).copy()
    geom_pointer = np.zeros(1, dtype=np.int32)
    ground_hits = []
    valid_distances = []
    classification = {"ground": 0, "self": 0, "prop": 0, "unclassified": 0}
    per_ring_counts = []
    misses = 0
    out_of_range = 0
    ray_box_compared = 0
    ray_box_worst = 0.0
    for elevation in rings:
        ring_valid = 0
        for step in range(per_ring):
            azimuth = 2.0 * np.pi * step / per_ring
            elevation_rad = np.radians(elevation)
            direction = rotation @ np.array(
                [
                    np.cos(elevation_rad) * np.cos(azimuth),
                    np.cos(elevation_rad) * np.sin(azimuth),
                    np.sin(elevation_rad),
                ],
                dtype=float,
            )
            distance = float(mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geom_pointer))
            if distance < 0.0:
                misses += 1
                continue
            if distance > range_m:
                out_of_range += 1
                continue
            ring_valid += 1
            valid_distances.append(distance)
            geom_id = int(geom_pointer[0])
            geom_name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if geom_name in ground_geoms:
                classification["ground"] += 1
                point = origin + distance * direction
                ground_hits.append(float(point[2]))
                if geom_name == "workbench":
                    analytic = _ray_box_distance(origin, direction, declared_center, declared_half)
                    if analytic is not None:
                        ray_box_compared += 1
                        ray_box_worst = max(ray_box_worst, abs(analytic - distance))
            else:
                body_id = int(model.geom_bodyid[geom_id])
                body_name = _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                if body_name in prop_bodies:
                    classification["prop"] += 1
                elif body_id in robot_bodies:
                    classification["self"] += 1
                else:
                    classification["unclassified"] += 1
        per_ring_counts.append(
            {"elevation_deg": elevation, "rays": per_ring, "points": ring_valid}
        )

    points = len(valid_distances)
    distances = np.asarray(valid_distances, dtype=float) if valid_distances else np.zeros(0)
    edges = [float(value) for value in thresholds["lidar"]["range_histogram_edges_m"]]
    if any(edges[index] >= edges[index + 1] for index in range(len(edges) - 1)):
        _fail(
            EXIT_DECLARATION,
            "sensor_acceptance.lidar.range_histogram_edges_m 必须严格递增: %s" % edges,
        )
    histogram = []
    for index in range(len(edges) - 1):
        low, high = edges[index], edges[index + 1]
        count = int(((distances >= low) & (distances < high)).sum()) if points else 0
        histogram.append({"low_m": low, "high_m": high, "count": count})
    ground_fraction = float(classification["ground"]) / float(num_rays)
    miss_fraction = float(misses + out_of_range) / float(num_rays)

    checks.add(
        "lidar.points_min",
        points,
        ">= %d" % int(thresholds["lidar"]["min_points"]),
        points >= int(thresholds["lidar"]["min_points"]),
        "有效点数（点数 = 0 必然失败）",
    )
    checks.add(
        "lidar.miss_fraction_max",
        miss_fraction,
        "<= %g" % float(thresholds["lidar"]["max_miss_fraction"]),
        miss_fraction <= float(thresholds["lidar"]["max_miss_fraction"]),
        "落空（未命中或超量程）占比",
    )
    checks.add(
        "lidar.ground_fraction_min",
        ground_fraction,
        ">= %g" % float(thresholds["lidar"]["min_ground_fraction"]),
        ground_fraction >= float(thresholds["lidar"]["min_ground_fraction"]),
        "地平面（%s）点占比" % ",".join(ground_geoms),
    )
    checks.add(
        "lidar.ray_vs_declared_box",
        ray_box_worst,
        "<= %g m" % float(thresholds["lidar"]["ray_box_tolerance_m"]),
        ray_box_worst <= float(thresholds["lidar"]["ray_box_tolerance_m"]),
        "mj_ray 命中台面的距离 vs 按声明解析求交（对照 %d 条）" % ray_box_compared,
    )

    return {
        "id": sensor_id,
        "name": site_name,
        "status": "checked",
        "site_world_m": [float(value) for value in origin],
        "rays": num_rays,
        "rings_deg": rings,
        "points": points,
        "misses": misses,
        "out_of_range": out_of_range,
        "miss_fraction": miss_fraction,
        "ground_points": classification["ground"],
        "ground_fraction": ground_fraction,
        "ground_hit_z_m": {
            "min": float(np.min(ground_hits)) if ground_hits else None,
            "max": float(np.max(ground_hits)) if ground_hits else None,
        },
        "range_m": float(range_m),
        "range_stats_m": {
            "min": float(distances.min()) if points else None,
            "mean": float(distances.mean()) if points else None,
            "max": float(distances.max()) if points else None,
        },
        "range_histogram": histogram,
        "classification": classification,
        "per_ring": per_ring_counts,
        "ray_box_comparison": {
            "declared_box_center_m": [float(value) for value in declared_center],
            "declared_box_half_m": [float(value) for value in declared_half],
            "compared": ray_box_compared,
            "max_deviation_m": ray_box_worst,
        },
    }


def _imu_section(model, data, profile_spec, thresholds, checks):
    """IMU 与力矩传感器：读数与**独立计算**的模型量比对（不是与硬编码常数比对）。"""
    model_spec = (profile_spec.get("model") or {})
    site_name = model_spec.get("imu_site")
    if not site_name:
        _fail(EXIT_DECLARATION, "Profile 的 spec.model.imu_site 未声明：无法定位 IMU site")
    site = _named_object(model, mujoco.mjtObj.mjOBJ_SITE, site_name, "IMU site")
    rotation = np.asarray(data.site_xmat[site], dtype=float).reshape(3, 3).copy()

    quat_sensor, quat_name = _one_sensor(
        model, SENSOR_TYPES["quat"], mujoco.mjtObj.mjOBJ_SITE, site, "site %s" % site_name, "framequat"
    )
    gyro_sensor, gyro_name = _one_sensor(
        model, SENSOR_TYPES["gyro"], mujoco.mjtObj.mjOBJ_SITE, site, "site %s" % site_name, "gyro"
    )
    acc_sensor, acc_name = _one_sensor(
        model,
        SENSOR_TYPES["accelerometer"],
        mujoco.mjtObj.mjOBJ_SITE,
        site,
        "site %s" % site_name,
        "accelerometer",
    )

    quaternion = _sensor_values(model, data, quat_sensor)
    gyro = _sensor_values(model, data, gyro_sensor)
    acceleration = _sensor_values(model, data, acc_sensor)

    quat_norm_error = abs(float(np.linalg.norm(quaternion)) - 1.0)
    checks.add(
        "imu.quat_norm",
        quat_norm_error,
        "<= %g" % float(thresholds["imu"]["quat_norm_tolerance"]),
        quat_norm_error <= float(thresholds["imu"]["quat_norm_tolerance"]),
        "framequat 读数必须是单位四元数",
    )

    # mj_objectVelocity / mj_objectAcceleration 的 6D 约定是 (rot, lin)，flg_local=0 取世界系。
    velocity = np.zeros(6, dtype=float)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, site, velocity, 0)
    acceleration_model = np.zeros(6, dtype=float)
    mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_SITE, site, acceleration_model, 0)
    gyro_model = rotation.T @ velocity[0:3]
    gyro_error = float(np.abs(gyro - gyro_model).max())
    checks.add(
        "imu.gyro_matches_model_angular_velocity",
        gyro_error,
        "<= %g rad/s" % float(thresholds["imu"]["gyro_max_abs_error_rad_s"]),
        gyro_error <= float(thresholds["imu"]["gyro_max_abs_error_rad_s"]),
        "陀螺仪读数 vs 独立计算的传感器系角速度（抓「挂错 site」这类绑定错误）",
    )

    acceleration_model_site = rotation.T @ acceleration_model[3:6]
    denominator = float(np.linalg.norm(acceleration_model_site))
    if denominator <= 1e-12:
        _fail(EXIT_REFERENCE, "模型线加速度为 0，无法做相对误差判据（请在非静止状态采样）")
    acc_error = float(np.linalg.norm(acceleration - acceleration_model_site) / denominator)
    checks.add(
        "imu.acc_matches_model_linear_acceleration",
        acc_error,
        "<= %g（相对误差）" % float(thresholds["imu"]["acc_rel_error_max"]),
        acc_error <= float(thresholds["imu"]["acc_rel_error_max"]),
        "加速度计读数 vs 独立计算的传感器系线加速度",
    )

    # 力矩传感器：profile.joints 的每个关节都必须有 jointactuatorfrc，且读数 = ctrl × gear。
    joints = [str(value) for value in (profile_spec.get("joints") or [])]
    if not joints:
        _fail(EXIT_DECLARATION, "Profile 的 spec.joints 为空：无法核对力矩传感器覆盖")
    worst_error = 0.0
    torque_rows = []
    for joint_name in joints:
        joint = _named_object(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name, "关节")
        sensor_id, sensor_name = _one_sensor(
            model,
            SENSOR_TYPES["joint_torque"],
            mujoco.mjtObj.mjOBJ_JOINT,
            joint,
            "关节 %s" % joint_name,
            "jointactuatorfrc",
        )
        actuator = -1
        for index in range(model.nu):
            if int(model.actuator_trnid[index][0]) == int(joint):
                actuator = index
                break
        if actuator < 0:
            measured = float(_sensor_values(model, data, sensor_id)[0])
            torque_rows.append(
                {"joint": joint_name, "sensor": sensor_name, "actuator": None,
                 "measured_nm": measured, "expected_nm": None, "abs_error_nm": None}
            )
            continue
        gear = float(model.actuator_gear[actuator][0])
        expected = float(data.ctrl[actuator]) * gear
        measured = float(_sensor_values(model, data, sensor_id)[0])
        error = abs(measured - expected)
        worst_error = max(worst_error, error)
        torque_rows.append(
            {"joint": joint_name, "sensor": sensor_name,
             "actuator": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator),
             "gear": gear, "ctrl": float(data.ctrl[actuator]),
             "measured_nm": measured, "expected_nm": expected, "abs_error_nm": error}
        )
    checks.add(
        "imu.torque_sensors_match_control",
        worst_error,
        "<= %g N·m" % float(thresholds["imu"]["torque_max_abs_error_nm"]),
        worst_error <= float(thresholds["imu"]["torque_max_abs_error_nm"]),
        "%d 个关节的 jointactuatorfrc 读数 vs ctrl×gear 的最大偏差" % len(joints),
    )

    return {
        "id": "base_imu",
        "name": site_name,
        "status": "checked",
        "site_world_m": [float(value) for value in np.asarray(data.site_xpos[site])],
        "quat": {"sensor": quat_name, "value": [float(value) for value in quaternion],
                 "norm": float(np.linalg.norm(quaternion)), "norm_error": quat_norm_error},
        "gyro": {"sensor": gyro_name, "value_rad_s": [float(value) for value in gyro],
                 "model_rad_s": [float(value) for value in gyro_model], "max_abs_error_rad_s": gyro_error},
        "acc": {"sensor": acc_name, "value_m_s2": [float(value) for value in acceleration],
                "model_m_s2": [float(value) for value in acceleration_model_site],
                "rel_error": acc_error,
                "norm_m_s2": float(np.linalg.norm(acceleration)),
                "note": "加速度计量纲为传感器系线加速度（MuJoCo 语义）；本场景无平衡控制器，"
                        "落台后的读数不是稳定的 9.81（见 not_proved）"},
        "torque": {"joints": len(joints), "max_abs_error_nm": worst_error, "rows": torque_rows},
    }


def verify_scene_sensors(scene_dir, root=None, robot=None, model_path=None, report_path=None,
                         render=True, write_report=True, renderer=None):
    """执行传感器验收，返回 (report, exit_code)。

    只做入口层的错误口径统一（把 `scene_builder` 的场景/Profile 解析错误
    归一为 `SensorEvidenceError`，退出码沿用其 code），实现都在 `_verify_scene_sensors`。
    """
    try:
        return _verify_scene_sensors(
            scene_dir,
            root=root,
            robot=robot,
            model_path=model_path,
            report_path=report_path,
            render=render,
            write_report=write_report,
            renderer=renderer,
        )
    except sb.SceneBuildError as exc:
        raise SensorEvidenceError(int(exc.code), str(exc)) from exc


def _verify_scene_sensors(scene_dir, root=None, robot=None, model_path=None, report_path=None,
                          render=True, write_report=True, renderer=None):
    root = Path(root or sb.repo_root())
    scene_path, scene = sb.load_scene(scene_dir, root)
    scene_dir = scene_path.parent
    baseline_path, baseline = sb.load_scene_baseline(scene_dir)
    if baseline is None:
        _fail(EXIT_DECLARATION, "场景包缺少 baseline.yaml：验收阈值必须来自声明")
    thresholds = _thresholds(baseline, baseline_path)
    checks = _Checks()

    # 被测本体：默认取雷达所在的实体（kinematics 与 IMU 都来自该本体的 Profile）。
    lidar_sensor = _pick_sensor(scene, "lidar")
    owner = str(lidar_sensor["anchor"]["entity"]) if robot is None else str(robot)
    entity = sb.resolve_robot(scene, owner)
    profile_path, profile_source = sb.resolve_profile(root, entity, owner)
    declared_model, profile_spec = sb.load_build_declarations(profile_path, owner)
    trunk_body_name = str((profile_spec.get("model") or {})["trunk_body"])

    # 被测产物：步骤 13 生成的 MJCF（scene.yaml 声明；本模块不重建场景）。
    declared_output = model_path or scene["model"]["output"]
    model_file = Path(declared_output)
    if not model_file.is_absolute():
        model_file = root / model_file
    if not model_file.is_file():
        _fail(
            EXIT_REFERENCE,
            "被测模型不存在: %s（先运行 scripts/build_scene.py --scene %s --robot %s）"
            % (model_file, scene_path.relative_to(root) if str(scene_path).startswith(str(root)) else scene_path, owner),
        )

    model = mujoco.MjModel.from_xml_path(str(model_file))
    data = mujoco.MjData(model)
    if int(model.nkey) < 1:
        _fail(EXIT_REFERENCE, "生成模型没有 keyframe：采样状态不可复现（厂商模型应自带 home 关键帧）")
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    trunk = _named_object(model, mujoco.mjtObj.mjOBJ_BODY, trunk_body_name, "躯干 body")
    reference_position = np.asarray(data.xpos[trunk], dtype=float).copy()
    settle_steps = int(thresholds["settle"]["steps"])
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    drift = float(np.linalg.norm(np.asarray(data.xpos[trunk], dtype=float) - reference_position))
    checks.add(
        "state.base_drift_after_settle",
        drift,
        "<= %g m（%d 帧）" % (float(thresholds["settle"]["max_base_drift_m"]), settle_steps),
        drift <= float(thresholds["settle"]["max_base_drift_m"]),
        "采样窗口内躯干位移：证明统计是在准静态状态下做的，不代表平衡能力",
    )

    camera = _camera_section(
        model, data, scene, baseline, root, thresholds, checks, render, renderer=renderer
    )
    lidar = _lidar_section(model, data, scene, owner, trunk, thresholds, checks)
    imu = _imu_section(model, data, profile_spec, thresholds, checks)

    failed = checks.failed
    # 相机渲染只有 "ok" 才算验收过：blocked / disabled_by_usage 都是"未验证"，
    # 退出码统一为 EXIT_RENDER（不得把未渲染的报告当成通过）。
    render_blocked = camera.get("render", {}).get("status") != "ok"
    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "report_kind": REPORT_KIND,
        "simulation": True,
        "scene": {
            "id": str(scene["id"]),
            "path": str(scene_path.relative_to(root)),
            "baseline": str(baseline_path.relative_to(root)),
            "robot": owner,
            "profile": str(profile_path.relative_to(root)),
            "profile_source": profile_source,
            "model": str(model_file.relative_to(root)),
            "model_sha256": _sha256(model_file),
        },
        "environment": {
            "python": sys.version.split()[0],
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
            "platform": platform.platform(),
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "interpreter": sys.executable,
        },
        "declarations": {
            "source": "%s#sensor_acceptance" % baseline_path.relative_to(root),
            "sensor_acceptance": thresholds,
            "sensors": {
                "camera": _camera_declaration(scene),
                "lidar": {"id": str(lidar_sensor["id"]), "num_rays": int(lidar_sensor["num_rays"]),
                          "range_m": float(lidar_sensor["range_m"])},
            },
        },
        "state": {
            "keyframe": _name(model, mujoco.mjtObj.mjOBJ_KEY, 0),
            "settle_steps": settle_steps,
            "base_drift_m": drift,
            "trunk_body": trunk_body_name,
            "trunk_position_m": [float(value) for value in np.asarray(data.xpos[trunk])],
        },
        "camera": camera,
        "lidar": lidar,
        "imu": imu,
        "checks": checks.items,
        "failed_checks": failed,
        "passed": not failed and not render_blocked,
        "not_proved": [
            "本场景无平衡控制器：settle.steps 只证明仿真推进与采样可复现，不构成站立/步态/导航能力",
            "雷达只做光线投射统计：未交付点云消息、扫描时序、噪声模型（属后续步骤）",
            "目标端/真机验收一律 DEFERRED（板卡不在场）：本报告不含任何真机证据",
        ],
    }
    output = report_path or (root / "build" / "acceptance" /
                             ("%s-scene-sensors" % str(scene["id"]).replace("_", "-")) / "report.json")
    output = Path(output)
    if not output.is_absolute():
        output = root / output
    report["report_path"] = str(output.relative_to(root)) if str(output).startswith(str(root)) else str(output)
    if write_report:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    if render_blocked:
        return report, EXIT_RENDER
    if failed:
        return report, EXIT_FAILED
    return report, 0


def _camera_declaration(scene):
    for sensor in scene.get("sensors") or []:
        if str(sensor.get("kind")) == "camera":
            return {"id": str(sensor["id"]), "fovy_deg": float(sensor["fovy_deg"]),
                    "entity": str(sensor["anchor"]["entity"])}
    return None
