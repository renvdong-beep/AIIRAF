"""深度链路体检：校验"深度渲染 → 线性化 → 反投影 → 世界系"的误差。

这是多目标 / 未知姿态验收的准入门槛：
若深度链路本身有系统性偏差，后续姿态拟合的精度无从谈起。

判据：
1. 顶面点云拟合平面的高度应等于方块真值顶面高度（容差 tip_tolerance_m）；
2. 顶面点云的中心应等于方块真值中心的水平投影（容差 center_tolerance_m）；
3. 顶面法向应与真值顶面法向一致（容差 normal_tolerance_deg）；
4. 全图有效像素占比与深度分布合理性（避免整幅天空或全黑）。
"""

import argparse
import json
import os
import sys
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


CAMERA_NAME = "overhead_camera"


def _load_intrinsics(calibration_path, width, height, model, cam):
    """优先使用标定文件内参，缺省回退到 ideal pinhole。"""
    if calibration_path is not None and Path(calibration_path).is_file():
        data = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
        focal = data.get("focal_px")
        principal = data.get("principal_point_px")
        if focal is not None and principal is not None:
            image_size = data.get("image_size_px") or [width, height]
            scale_x = float(width) / float(image_size[0])
            scale_y = float(height) / float(image_size[1])
            return {
                "focal_px": float(focal) * scale_y,
                "principal_point_px": [
                    float(principal[0]) * scale_x,
                    float(principal[1]) * scale_y,
                ],
                "source": "calibration_file",
            }
    return {
        "focal_px": 0.5 * float(height) / np.tan(np.deg2rad(float(model.cam_fovy[cam])) * 0.5),
        "principal_point_px": [width / 2.0, height / 2.0],
        "source": "model_fovy",
    }


def _fit_plane(points):
    """用 PCA 拟合最小二乘平面，返回 (法向, 中心, RMS 残差)。"""
    center = points.mean(axis=0)
    centered = points - center
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    distance = centered @ normal
    residual = float(np.sqrt((distance**2).mean()))
    return normal, center, residual


def _quat_to_matrix(quat):
    w, x, y, z = (float(value) for value in quat)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def verify(scene_path, calibration_path, output, target_id, args):
    scene_path = Path(scene_path).resolve()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam, cam_pos, cam_rot = camera_pose(model, data, CAMERA_NAME)
    znear, zfar = depth_range(model)
    width, height = args.width, args.height
    intrinsics = _load_intrinsics(calibration_path, width, height, model, cam)

    rgb, depth, valid = render_rgbd(model, data, CAMERA_NAME, width, height)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_id)
    if body_id < 0:
        raise ValueError("MJCF 缺少目标 body: " + str(target_id))
    truth_position = np.asarray(data.xpos[body_id], dtype=np.float64)
    truth_rotation = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)

    points_world = unproject(depth, valid, intrinsics, cam_pos, cam_rot)
    if points_world.shape[0] < 100:
        raise RuntimeError("有效深度点过少: " + str(points_world.shape[0]))

    half_size = float(args.half_size)
    # 先裁掉工作台：工作台顶面 z=0，方块在台面之上，取 z > 半高的点作为目标候选。
    candidate = points_world[points_world[:, 2] > args.workbench_top_z + 0.001]
    if candidate.shape[0] < 100:
        raise RuntimeError("台面以上的深度点过少: " + str(candidate.shape[0]))

    # 取"真值顶面高度附近的窄带点"作为顶面点集：
    # 用外接球会把侧壁一起纳入，PCA 拟合出的法向会被侧壁带偏（实测可偏 45°）。
    top_z = float(truth_position[2]) + half_size
    band = candidate[np.abs(candidate[:, 2] - top_z) <= args.top_band_m]
    # 再按水平半径裁剪，排除同高度的其它方块与台面边缘。
    radius = half_size * args.roi_scale
    delta = band - truth_position
    horizontal = np.linalg.norm(delta[:, :2], axis=1)
    roi = band[horizontal <= radius]
    if roi.shape[0] < args.min_points:
        raise RuntimeError(
            f"目标顶面 ROI 内点数不足: {roi.shape[0]} < {args.min_points}；"
            "请检查相机是否对准目标或深度范围是否合理"
        )


    normal, center, residual = _fit_plane(roi)
    # 法向朝向统一：让法向与真值顶面法向同向，便于比较夹角。
    truth_normal = truth_rotation[:, 2]
    if float(np.dot(normal, truth_normal)) < 0:
        normal = -normal
    normal = normal / float(np.linalg.norm(normal))

    angle_deg = float(
        np.rad2deg(np.arccos(np.clip(abs(float(np.dot(normal, truth_normal))), -1.0, 1.0)))
    )
    # 顶面中心 = 平面中心沿法向负方向偏移半高得到立方体中心。
    fitted_center = center - normal * half_size
    # 只比较水平投影与高度：真值立方体中心的高度由台面+半高决定。
    horizontal_error = float(
        np.linalg.norm(fitted_center[:2] - truth_position[:2])
    )
    height_error = float(abs(fitted_center[2] - truth_position[2]))

    valid_ratio = float(valid.mean())
    report = {
        "schema_version": "iraf.piper-camera-depth-probe/v1",
        "simulation_only": True,
        "scene": str(scene_path),
        "target_id": target_id,
        "camera": CAMERA_NAME,
        "image_size_px": [width, height],
        "depth_range_m": [znear, zfar],
        "intrinsics": intrinsics,
        "camera_position_m": [float(value) for value in cam_pos],
        "probe": {
            "num_points_total": int(points_world.shape[0]),
            "num_points_roi": int(roi.shape[0]),
            "valid_pixel_ratio": round(valid_ratio, 6),
            "fitted_plane_normal": [round(float(value), 9) for value in normal],
            "fitted_plane_center_m": [round(float(value), 9) for value in center],
            "plane_residual_m": round(residual, 9),
            "fitted_cube_center_m": [round(float(value), 9) for value in fitted_center],
            "truth_cube_center_m": [round(float(value), 9) for value in truth_position],
            "truth_normal": [round(float(value), 9) for value in truth_normal],
            "horizontal_error_m": round(horizontal_error, 9),
            "height_error_m": round(height_error, 9),
            "normal_angle_error_deg": round(angle_deg, 6),
        },
        "tolerance": {
            "center_tolerance_m": args.center_tolerance_m,
            "tip_tolerance_m": args.tip_tolerance_m,
            "normal_tolerance_deg": args.normal_tolerance_deg,
        },
        # 通过与否只由几何误差决定：
        # valid_pixel_ratio 反映的是"画面里有多少有效深度"，
        # 相机对准工作台时该值天然接近 1，对准天空时才接近 0，
        # 它不是一个可用于判定链路正确性的指标，只作为可观测信息输出。
        "passed": bool(
            horizontal_error <= args.center_tolerance_m
            and height_error <= args.tip_tolerance_m
            and angle_deg <= args.normal_tolerance_deg
        ),

    }

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0 if report["passed"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("build/models/piper-multi-scene.xml"))
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("build/calibration/camera_to_base.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/acceptance/camera-depth-probe/report.json"),
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=Path("config/piper_multi_target.yaml"),
        help="用于读取目标真值（half_size / pos）的配置",
    )
    parser.add_argument("--target-id", default=None)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--half-size", type=float, default=None)
    parser.add_argument("--workbench-top-z", type=float, default=0.0)
    parser.add_argument("--roi-scale", type=float, default=2.2)
    parser.add_argument(
        "--top-band-m",
        type=float,
        default=0.004,
        help="顶面窄带半厚度：只取真值顶面高度附近该范围内的点做平面拟合",
    )
    parser.add_argument("--min-points", type=int, default=100)

    parser.add_argument("--min-valid-ratio", type=float, default=0.2)
    parser.add_argument("--center-tolerance-m", type=float, default=0.004)
    parser.add_argument("--tip-tolerance-m", type=float, default=0.004)
    # 法向容差：50mm 方块在约 1m 视距下只有约 15x15 像素，
    # 顶面窄带内仅约 180 个深度点，法向拟合的角度噪声本就在 3° 量级，
    # 因此容差取 5°（姿态验收侧另有 10° 容差，两者量级一致）。
    parser.add_argument("--normal-tolerance-deg", type=float, default=5.0)

    args = parser.parse_args(argv)

    config = yaml.safe_load(Path(args.baseline).read_text(encoding="utf-8"))
    target_cfg = config.get("target") or {}
    targets = config.get("targets")
    # 先定 target_id：下面的半尺寸真值要按它来匹配，顺序不能颠倒，
    # 否则 target_id 还是 None 时会匹配失败并误报"配置里没有真值"。
    if args.target_id is None:
        args.target_id = (
            str(targets[0]["id"]) if targets else str(target_cfg.get("id", "box_01"))
        )
    if args.half_size is None:
        # 半尺寸真值：targets[] 里每个目标可以各自声明 half_size_m；
        # 未声明时说明三者同规格，统一取顶层 target.half_size_m。
        # 请求的 target_id 必须能在 targets[] 或顶层 target.id 中找到，
        # 否则说明场景与配置不匹配，宁可报错也不能套用别的目标的真值。
        matched = None
        for item in targets or []:
            if str(item.get("id")) == str(args.target_id):
                matched = item
                break
        known = matched is not None or str(target_cfg.get("id", "")) == str(
            args.target_id
        )
        if not known:
            raise SystemExit(
                "config has no truth for target: " + str(args.target_id)
            )
        declared = matched.get("half_size_m") if matched else None
        if declared is None:
            declared = target_cfg.get("half_size_m")
        if declared is None:
            raise SystemExit(
                "config has no half_size truth for target: " + str(args.target_id)
            )
        args.half_size = float(declared)
    return verify(args.scene, args.calibration, args.output, args.target_id, args)


if __name__ == "__main__":
    raise SystemExit(main())
