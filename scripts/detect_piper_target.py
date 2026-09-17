"""使用 MuJoCo Renderer 分割相机检测抓取目标。

相机固定并对准抓取点，因此可以用标定文件里的内参
（principal_point_px / focal_px）做精确反投影；
不再使用写死的主点 (320,240) 或手工偏移。
"""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480


def _load_calibration(path):
    """读取相机标定文件；缺省返回 None，此时使用模型自带内参。"""
    if path is None:
        return None
    candidate = Path(path)
    if not candidate.is_file():
        return None
    return json.loads(candidate.read_text(encoding="utf-8"))


def _resolve_intrinsics(model, cam, calibration, width, height):
    """解析内参：优先使用标定值，否则回退到 ideal pinhole。"""
    if calibration:
        focal = calibration.get("focal_px")
        principal = calibration.get("principal_point_px")
        if focal is not None and principal is not None:
            image_size = calibration.get("image_size_px") or [width, height]
            if int(image_size[0]) != int(width) or int(image_size[1]) != int(height):
                # 分辨率不一致时按比例换算，避免标定值被误用。
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


def detect(model_path, output, extrinsics=None):
    model_path = Path(model_path).resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_01_geom")
    if geom < 0:
        raise RuntimeError("MJCF 缺少目标 geom: box_01_geom")
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01")
    if body < 0:
        raise RuntimeError("MJCF 缺少目标 body: box_01")
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overhead_camera")
    if cam < 0:
        raise RuntimeError("MJCF 缺少相机: overhead_camera")
    # cam_mode: 0=fixed, 1=track, 2=targetbody
    if int(model.cam_mode[cam]) != 0:
        raise RuntimeError(
            "视觉检测要求相机为 fixed 模式（cam_mode=0）；"
            "targetbody 模式下相机随目标移动，坐标不可审计"
        )

    width, height = DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera="overhead_camera")
        seg = renderer.render()
        mask = seg[:, :, 1] == geom
        source = "renderer_segmentation"
        if not mask.any():
            renderer.disable_segmentation_rendering()
            renderer.update_scene(data, camera="overhead_camera")
            rgb = renderer.render()
            mask = (
                (rgb[:, :, 0] > rgb[:, :, 1] * 1.6)
                & (rgb[:, :, 0] > rgb[:, :, 2] * 1.4)
                & (rgb[:, :, 0] > 60)
            )
            source = "rgb_color_threshold"
        if not mask.any():
            raise RuntimeError("视觉相机未检测到 box_01")
        ys, xs = np.where(mask)
        bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        # 目标几何中心对应掩码包围盒中心，而非亮度加权质心：
        # 斜视时方块可见面亮度不均，质心会被暗侧面拉偏约 1 像素，
        # 经 1m 视距放大后就是数毫米的世界坐标误差。
        px = (bbox[0] + bbox[2]) / 2.0
        py = (bbox[1] + bbox[3]) / 2.0
    finally:
        renderer.close()

    calibration = _load_calibration(extrinsics)
    intrinsics = _resolve_intrinsics(model, cam, calibration, width, height)
    focal = intrinsics["focal_px"]
    center_u, center_v = intrinsics["principal_point_px"]

    cam_pos = np.asarray(data.cam_xpos[cam], dtype=float)
    cam_rot = np.asarray(data.cam_xmat[cam], dtype=float).reshape(3, 3)
    # 射线方向：像素偏移除以焦距，光轴为相机坐标系 -Z。
    ray_world = cam_rot @ np.array(
        [(px - center_u) / focal, -(py - center_v) / focal, -1.0]
    )
    if abs(float(ray_world[2])) < 1e-9:
        raise RuntimeError("视觉射线与目标平面平行，无法求交")
    # 目标放在工作台上，用目标中心高度求射线与支撑平面的交点，
    # 避免把方块顶面深度误当成目标中心深度。
    plane_z = float(data.xpos[body][2])
    ray_scale = (plane_z - float(cam_pos[2])) / float(ray_world[2])
    vision_pos = cam_pos + ray_scale * ray_world

    world_pos = np.asarray(data.xpos[body], dtype=float)
    result = {
        "schema_version": "iraf.piper-vision-target/v1",
        "camera": "overhead_camera",
        "camera_mode": "fixed",
        "target_id": "box_01",
        "pixel_bbox": bbox,
        "pixel_center": [px, py],
        "depth_m": float(np.linalg.norm(world_pos - cam_pos)),
        "intrinsics": intrinsics,
        "vision_world_position_m": [float(value) for value in vision_pos],
        "world_position_m": [float(value) for value in world_pos],
        "source": source,
    }
    if extrinsics is not None:
        result["extrinsics_file"] = str(extrinsics)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(
        json.dumps(result, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", default="build/calibration/piper-vision-target.json")
    parser.add_argument(
        "--extrinsics",
        default="build/calibration/camera_to_base.json",
        help="相机标定文件；不存在时回退到模型自带内参",
    )
    args = parser.parse_args()
    detect(args.model, args.output, args.extrinsics)
