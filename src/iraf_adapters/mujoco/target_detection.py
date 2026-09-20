"""多目标 / 未知姿态视觉检测（与机型无关的编排层）。

本模块由 `scripts/detect_piper_targets.py`（历史入口，现为薄包装）
与 `scripts/detect_targets.py`（通用入口）共同使用；除此之外不含任何
机型专有内容：目标 id 与颜色来自场景旁挂报告，相机名与 schema 由调用方声明。


流程：
1. 固定相机渲染 RGB + 线性深度（depth_channel）；
2. 按配置的 id → rgba 映射做颜色分割，得到每个目标的像素掩码；
3. 掩码 ∧ 有效深度 → 内参反投影 → 外参变换 → 世界系点云；
4. 调 pose_estimation 估计 6DoF（顶面平面拟合 + 最小外接矩形 + 对称消歧）。

严格约束：
- 相机必须是 fixed 模式，否则坐标不可审计；
- 本模块**禁止**读取 data.xpos / data.xmat 等仿真真值，
  真值只在验收脚本里用于比对。
"""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from iraf_adapters.mujoco.depth_channel import (  # noqa: E402
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    camera_pose,
    depth_range,
    render_rgbd,
    unproject,
)
from iraf_adapters.mujoco.pose_estimation import estimate_cube_pose  # noqa: E402

DEFAULT_CAMERA_NAME = "overhead_camera"

#: 证据文件 schema。**不得带机型名**：换机器人时证据格式应当不变，
#: 否则每个消费方都要按机型分支解析。
TARGETS_SCHEMA_VERSION = "iraf.vision-targets/v1"


def _resolve_intrinsics(model, cam, calibration, width, height):
    """解析内参：优先标定文件，其次模型 fovy 的理想针孔模型。"""
    if calibration:
        focal = calibration.get("focal_px")
        principal = calibration.get("principal_point_px")
        if focal is not None and principal is not None:
            image_size = calibration.get("image_size_px") or [width, height]
            if int(image_size[0]) != int(width) or int(image_size[1]) != int(height):
                scale_x = float(width) / float(image_size[0])
                scale_y = float(height) / float(image_size[1])
                focal = float(focal) * scale_y
                principal = [float(principal[0]) * scale_x, float(principal[1]) * scale_y]
            return {
                "focal_px": float(focal),
                "principal_point_px": [float(principal[0]), float(principal[1])],
                "source": "calibration_file",
            }
    return {
        "focal_px": 0.5 * float(height) / np.tan(np.deg2rad(float(model.cam_fovy[cam])) * 0.5),
        "principal_point_px": [width / 2.0, height / 2.0],
        "source": "model_fovy",
    }


def _rgba_to_color(rgba):
    """把 MJCF 线性 rgba 转到用于比对的近似 sRGB 观测量。

    渲染结果经过光照与 Gamma 处理，与材质声明值不会是同一数值，
    因此只用于"最近颜色匹配"，不要求数值相等。
    """
    values = [float(value) for value in rgba[:3]]
    return np.asarray(values, dtype=np.float64)


def _observed_color(rgb, mask):
    """取掩码内像素的中位颜色，抑制高光与阴影的极端值。"""
    pixels = rgb[mask]
    return np.median(pixels, axis=0).astype(np.float64) / 255.0


def _segment_by_color(rgb, target_color, tolerance):
    """按颜色距离做分割，返回布尔掩码。

    纯色方块在俯视图里颜色稳定，用归一化颜色距离比固定阈值更鲁棒，
    不需要 HSV 转换即可区分本场景的红/绿/蓝/橙。
    """
    observed = np.asarray(rgb, dtype=np.float64) / 255.0
    # 归一化亮度，降低明暗差异带来的颜色漂移。
    observed_norm = observed / np.maximum(observed.sum(axis=2, keepdims=True), 1e-6)
    target = np.asarray(target_color, dtype=np.float64)
    target_norm = target / max(float(target.sum()), 1e-6)
    distance = np.linalg.norm(observed_norm - target_norm, axis=2)
    return distance <= tolerance


def _largest_component(mask):
    """保留最大连通域，去掉同色杂散像素（受抗锯齿/反射影响）。

    使用 numpy 实现的标签传播，避免引入 scipy / OpenCV。
    """
    height, width = mask.shape
    labels = np.zeros((height, width), dtype=np.int32)
    current = 0
    best_label = 0
    best_size = 0
    # 迭代式洪水填充，使用显式栈避免递归深度问题。
    ys, xs = np.nonzero(mask)
    for start_y, start_x in zip(ys, xs):
        if labels[start_y, start_x] != 0:
            continue
        current += 1
        stack = [(int(start_y), int(start_x))]
        labels[start_y, start_x] = current
        size = 0
        while stack:
            y, x = stack.pop()
            size += 1
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < height and 0 <= nx < width:
                    if mask[ny, nx] and labels[ny, nx] == 0:
                        labels[ny, nx] = current
                        stack.append((ny, nx))
        if size > best_size:
            best_size = size
            best_label = current
    if best_label == 0:
        return np.zeros_like(mask, dtype=bool)
    return labels == best_label


def detect(
    model_path,
    output,
    baseline,
    calibration=None,
    width=DEFAULT_WIDTH,
    height=DEFAULT_HEIGHT,
    camera_name=DEFAULT_CAMERA_NAME,
    gripper_yaw_hint=None,
    schema_version=TARGETS_SCHEMA_VERSION,
):
    model_path = Path(model_path).resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam, cam_pos, cam_rot = camera_pose(model, data, camera_name)
    # cam_mode: 0=fixed, 1=track, 2=targetbody
    if int(model.cam_mode[cam]) != 0:
        raise RuntimeError(
            "视觉检测要求相机为 fixed 模式（cam_mode=0）；"
            "targetbody 模式下相机随目标移动，坐标不可审计"
        )
    depth_range(model)  # 提前校验深度范围合法，失败信息更直观

    config = yaml.safe_load(Path(baseline).read_text(encoding="utf-8"))
    target_cfg = config.get("target") or {}
    depth_cfg = config.get("depth") or {}
    raw_targets = config.get("targets")
    if raw_targets:
        targets_cfg = [
            {
                "id": str(item["id"]),
                "rgba": item.get("rgba", target_cfg.get("rgba", [0.82, 0.22, 0.12, 1.0])),
            }
            for item in raw_targets
        ]
    else:
        targets_cfg = [
            {
                "id": str(target_cfg.get("id", "box_01")),
                "rgba": target_cfg.get("rgba", "0.82 0.22 0.12 1"),
            }
        ]
    half_size = float(target_cfg.get("half_size_m", 0.025))

    intrinsics = _resolve_intrinsics(model, cam, calibration, width, height)
    rgb, depth, valid = render_rgbd(model, data, camera_name, width, height)
    points_world = unproject(depth, valid, intrinsics, cam_pos, cam_rot)

    # 把世界系点云按像素索引保留，便于按掩码取子集。
    height_px, width_px = depth.shape
    us, vs = np.meshgrid(np.arange(width_px), np.arange(height_px))
    pixel_index = np.stack([us[valid], vs[valid]], axis=1)

    color_tolerance = float(depth_cfg.get("color_tolerance", 0.18))
    min_points = int(depth_cfg.get("min_points_per_target", 100))
    max_points = int(depth_cfg.get("max_points_per_target", 5000))
    size_tolerance = float(depth_cfg.get("size_tolerance_m", 0.006))

    results = []
    for item in targets_cfg:
        target_id = item["id"]
        color = _rgba_to_color(_parse_rgba(item["rgba"]))
        mask = _segment_by_color(rgb, color, color_tolerance)
        # 混入的极小同色区域会让点云出现双峰，取最大连通域后再拟合。
        mask = _largest_component(mask)
        if not mask.any():
            raise RuntimeError(
                f"目标 {target_id} 颜色分割为空（tolerance={color_tolerance}）；"
                "请检查渲染是否正常或颜色容差是否过小"
            )
        ys, xs = np.where(mask)
        bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        # 几何中心取包围盒中心，而非亮度质心：斜视时可见面亮度不均，
        # 质心会被暗侧面拉偏约 1 像素，放大到世界系就是数毫米误差。
        pixel_center = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)

        selected = mask[valid]
        if int(selected.sum()) < min_points:
            raise RuntimeError(
                f"目标 {target_id} 掩码内有效深度点不足: {int(selected.sum())} < {min_points}"
            )
        target_points = points_world[selected]
        estimate = estimate_cube_pose(
            target_points,
            half_size_m=half_size,
            size_tolerance_m=size_tolerance,
            target_id=target_id,
            gripper_yaw_hint=gripper_yaw_hint,
            ransac_iterations=int(depth_cfg.get("ransac_iterations", 200)),
            ransac_inlier_m=float(depth_cfg.get("ransac_inlier_m", 0.002)),
            outlier_mad_scale=float(depth_cfg.get("outlier_mad_scale", 3.0)),
            min_points=min_points,
            max_points=max_points,
        )
        record = estimate.as_dict()
        record["pixel_center"] = [round(float(value), 3) for value in pixel_center]
        record["pixel_bbox"] = bbox
        record["observed_color_rgb"] = [
            round(float(value), 6) for value in _observed_color(rgb, mask)
        ]
        record["mask_pixels"] = int(mask.sum())
        results.append(record)

    report = {
        "schema_version": str(schema_version),
        "simulation_only": True,
        "scene": str(model_path),
        "camera": camera_name,
        "camera_mode": "fixed",
        "image_size_px": [width, height],
        "intrinsics": intrinsics,
        "target_half_size_m": half_size,
        "targets": results,
    }
    result_path = Path(output)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return report


def _parse_rgba(value):
    if isinstance(value, str):
        parts = [float(item) for item in value.replace(",", " ").split()]
    else:
        parts = [float(item) for item in value]
    if len(parts) == 3:
        parts.append(1.0)
    if len(parts) != 4:
        raise ValueError("rgba 必须是 3 或 4 个数: " + str(value))
    return parts
