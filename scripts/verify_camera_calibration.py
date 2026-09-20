"""相机标定验收：外参（相机→基座刚体变换）+ 内参（主点与焦距）。

流程：
1. 外参：用 MJCF 真实相机位姿构造 camera_m / base_m 对应点对，做刚体拟合。
2. 内参：把目标方块移动到多个已知位置，用渲染掩码中心 + 真实方块中心
   拟合 principal_point_px 与 focal_px。
3. 写出标定文件（rotation_matrix / translation_m / principal_point_px / focal_px）。

标定文件供 scripts/detect_piper_target.py --extrinsics 使用，
替代原先写死的主点 (320,240) 与手工偏移。
"""

import argparse
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

WIDTH, HEIGHT = 640, 480
# 采样点覆盖相机视野内不同像素位置，才能约束主点与焦距。
# 相机固定后，目标可以在工作台范围内自由移动而不改变相机位姿。
SAMPLE_OFFSETS = [
    (-0.12, -0.12),
    (-0.12, 0.12),
    (0.0, 0.0),
    (0.12, -0.12),
    (0.12, 0.12),
    (-0.18, 0.0),
    (0.18, 0.0),
    (0.0, -0.18),
    (0.0, 0.18),
]


def scene_with_target(scene_path, target_xy, half_size):
    """在场景同目录生成临时场景，把目标方块移到指定 XY。

    临时文件必须与原场景同目录，否则相对网格路径会失效。
    """
    handle = tempfile.NamedTemporaryFile(
        "w", suffix=".xml", dir=str(scene_path.parent), delete=False
    )
    handle.close()
    output = Path(handle.name)
    tree = ET.parse(scene_path)
    root = tree.getroot()
    for body in root.iter("body"):
        if body.get("name") == "box_01":
            body.set("pos", "%.9f %.9f %.9f" % (target_xy[0], target_xy[1], half_size))
            break
    tree.write(output, encoding="unicode", xml_declaration=True)
    return output


def mask_centroid(model, data, geom_id, renderer):
    """返回目标在图像中的掩码中心与来源。"""
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera="overhead_camera")
    seg = renderer.render()
    mask = seg[:, :, 1] == geom_id
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
    ys, xs = np.where(mask)
    if not len(xs):
        raise RuntimeError("视觉相机未检测到目标")
    # 与 detect_piper_target.py 保持一致：使用包围盒中心表示目标几何中心。
    return (
        (float(xs.min()) + float(xs.max())) / 2.0,
        (float(ys.min()) + float(ys.max())) / 2.0,
        source,
    )


def sample_intrinsics(scene_path, target_position, half_size):
    """移动目标方块，采集 pixel / base_m 对应的内参样本。"""
    samples = []
    for offset_x, offset_y in SAMPLE_OFFSETS:
        xy = (target_position[0] + offset_x, target_position[1] + offset_y)
        temp_scene = scene_with_target(scene_path, xy, half_size)
        try:
            model = mujoco.MjModel.from_xml_path(str(temp_scene))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_01_geom")
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01")
            renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
            try:
                u, v, source = mask_centroid(model, data, geom_id, renderer)
            finally:
                renderer.close()
            samples.append(
                {
                    "pixel": [u, v],
                    "base_m": [float(value) for value in data.xpos[body_id]],
                    "source": source,
                }
            )
        finally:
            temp_scene.unlink(missing_ok=True)
    return samples


def sample_extrinsics(model, data, camera, samples=8):
    """用真实相机位姿构造 camera_m / base_m 对应点对。"""
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    if cam_id < 0:
        raise ValueError("MJCF 缺少相机: " + camera)
    mujoco.mj_forward(model, data)
    camera_position = np.asarray(data.cam_xpos[cam_id], dtype=float)
    camera_rotation = np.asarray(data.cam_xmat[cam_id], dtype=float).reshape(3, 3)
    pairs = []
    for index in range(samples):
        angle = 2.0 * np.pi * index / float(samples)
        world_point = np.array(
            [
                0.20 + 0.04 * np.cos(angle),
                0.04 * np.sin(angle),
                0.02 + 0.005 * index,
            ]
        )
        # camera_m 为相机坐标系下的点；base_m 用世界坐标，
        # 因为当前 AIIRAF 把机械臂基座与仿真世界视为同一坐标系。
        pairs.append(
            {
                "camera_m": [
                    float(value) for value in camera_rotation.T @ (world_point - camera_position)
                ],
                "base_m": [float(value) for value in world_point],
            }
        )
    return pairs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--camera", default="overhead_camera")
    parser.add_argument("--target-id", default="box_01")
    parser.add_argument(
        "--profile",
        type=Path,
        default=None,
        help="RobotProfile；缺省时读基线配置的 build.profile（不得隐式假设机型）",
    )
    parser.add_argument("--baseline", type=Path, default=None,
                        help="基线配置；用于解析 build.profile")
    parser.add_argument("--target-offset", type=float, default=0.05)
    parser.add_argument(
        "--extrinsics-output",
        type=Path,
        default=Path("build/calibration/camera_to_base.json"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("build/acceptance/camera-calibration")
    )
    parser.add_argument("--tolerance-mm", type=float, default=1.0)
    parser.add_argument(
        "--focal-tolerance",
        type=float,
        default=0.15,
        help="拟合焦距与模型理论焦距(fovy 推出)的最大相对偏差",
    )
    parser.add_argument(
        "--residual-px",
        type=float,
        default=2.0,
        help="内参拟合的像素残差 RMS 上限",
    )
    args = parser.parse_args(argv)

    root = Path(__file__).resolve().parents[1]
    scene_path = args.scene.resolve()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, args.target_id)
    if body_id < 0:
        raise SystemExit("场景里没有目标 body: " + args.target_id)
    mujoco.mj_forward(model, data)
    target_position = [float(value) for value in data.xpos[body_id]]
    geom_name = args.target_id + "_geom"
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        raise SystemExit("场景里没有目标 geom: " + geom_name)
    half_size = float(model.geom_size[geom_id][2])

    pairs = sample_extrinsics(model, data, args.camera)
    intrinsics_samples = sample_intrinsics(scene_path, target_position, half_size)

    authority = ControlAuthorityManager()
    profile_path = args.profile
    if profile_path is None:
        declared = None
        if args.baseline is not None:
            import yaml
            declared = (yaml.safe_load((root / args.baseline).read_text(encoding="utf-8")).get("build") or {}).get("profile")
        if not declared:
            raise SystemExit("请显式给出 --profile，或提供带 build.profile 的 --baseline")
        profile_path = Path(declared)
    profile = load_robot_profile((root / profile_path) if not profile_path.is_absolute() else profile_path)
    backend = MujocoBackend.from_config({"model_path": str(scene_path)}, profile, authority)
    lease = authority.acquire(profile.name + "-mujoco", "camera-calibration")
    evidence = backend.calibrate_camera_to_base(
        {
            "pairs": pairs,
            "intrinsics_samples": intrinsics_samples,
            "image_size_px": [WIDTH, HEIGHT],
            "tolerance_m": args.tolerance_mm / 1000.0,
        },
        lease,
    )

    extrinsics = {
        "schema_version": "iraf.camera-extrinsics/v1",
        "camera": args.camera,
        "rotation_matrix": evidence["rotation_matrix"],
        "translation_m": evidence["translation_m"],
        "mean_error_m": evidence["mean_error_m"],
        "max_error_m": evidence["max_error_m"],
        "point_count": evidence["point_count"],
    }
    intrinsics = evidence.get("intrinsics")
    if intrinsics:
        extrinsics["focal_px"] = intrinsics["focal_px"]
        extrinsics["principal_point_px"] = intrinsics["principal_point_px"]
        extrinsics["image_size_px"] = intrinsics["image_size_px"]

    args.extrinsics_output.parent.mkdir(parents=True, exist_ok=True)
    args.extrinsics_output.write_text(
        json.dumps(extrinsics, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )

    report = {
        "schema_version": "iraf.camera-calibration-acceptance/v1",
        "simulation_only": True,
        "scene": str(scene_path),
        "camera": args.camera,
        "pairs": len(pairs),
        "intrinsics_samples": len(intrinsics_samples),
        "evidence": evidence,
        "extrinsics_file": str(args.extrinsics_output),
        "tolerance_mm": args.tolerance_mm,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=True, indent=2))

    # 内参门禁：以**模型自带 fovy 推出的理论焦距**为基准，判定相对偏差与像素残差。
    # 原实现写死 200 < focal < 400 px —— 那是 Piper 相机（292px）留下的机型假设，
    # 换相机就会误判（实测 UR5e 的 404.8px 被判 CALIBRATION_FAILED，而几何上完全正确：
    # 由 fovy=60° 推出的理论焦距 415.7px，相对偏差仅 2.6%）。
    intrinsic_ok = False
    if intrinsics:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
        if cam_id < 0:
            raise SystemExit("场景里没有相机: " + args.camera)
        half_height = float(HEIGHT) / 2.0
        focal_expected = half_height / math.tan(
            math.radians(float(model.cam_fovy[cam_id]) / 2.0)
        )
        relative_error = abs(float(intrinsics["focal_px"]) - focal_expected) / focal_expected
        residual_ok = float(intrinsics.get("residual_px_rms", 0.0)) <= float(args.residual_px)
        intrinsics["focal_expected_px"] = round(float(focal_expected), 4)
        intrinsics["focal_relative_error"] = round(float(relative_error), 6)
        intrinsic_ok = bool(relative_error <= args.focal_tolerance and residual_ok)
    passed = (
        bool(evidence["passed"])
        and evidence["max_error_m"] <= args.tolerance_mm / 1000.0
        and intrinsic_ok
    )
    print("CALIBRATION_PASSED" if passed else "CALIBRATION_FAILED")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
