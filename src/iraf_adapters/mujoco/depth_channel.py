"""MuJoCo 离屏深度渲染与反投影工具。

本模块只负责"渲染 → 线性深度 → 世界系点云"这一段，
不承担任何目标识别或姿态拟合职责，便于独立单测。

关键事实（MuJoCo / OpenGL）：
- 深度缓冲是非线性的 OpenGL 深度值 d ∈ [0, 1]（NDC 的 [-1,1] 线性映射），
  必须用 znear/zfar 反算真实相机系距离，否则近处点会被系统性压缩；
- 深度值为 1.0 的像素是"天空/无物体"，必须整体掩膜剔除；
- MuJoCo 相机光轴是 -Z，图像上方是 +Y，因此世界系坐标必须用
  data.cam_xmat 的列向量作为相机基，且相机系 Z 为负值。
"""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480

# MuJoCo 深度渲染对"无几何/天空"像素写入的固定值。
SKY_DEPTH_VALUE = 1.0



def linearize_depth(depth_buffer, znear, zfar):
    """归一化 MuJoCo 深度渲染结果，返回相机系沿光轴的直线距离（米）。

    实测结论：MuJoCo 的 ``enable_depth_rendering()`` 输出**已经是米制线性距离**
    （最大值为 zfar，本场景 113.16m 与 zfar 完全一致），
    并非 [0,1] 的 OpenGL 非线性深度缓冲。
    因此此处不能再做一次 z_ndc 反算，否则数值会被彻底破坏。

    本函数只做：
    1. 非法值（NaN / 负数）置为 inf，便于统一剔除；
    2. 超过 zfar 的值截断为 inf（无几何/天空）。
    """
    distance = np.asarray(depth_buffer, dtype=np.float64).copy()
    if not (zfar > znear > 0):
        raise ValueError(f"深度范围非法: znear={znear} zfar={zfar}")
    invalid = ~np.isfinite(distance) | (distance <= 0.0) | (distance >= zfar)
    distance[invalid] = np.inf
    return distance



def depth_range(model):
    """解析渲染所用的 znear/zfar（沿用 MuJoCo 自身的可见范围设置）。"""
    extent = float(model.stat.extent)
    znear = float(model.vis.map.znear) * extent
    zfar = float(model.vis.map.zfar) * extent
    if not (zfar > znear > 0):
        raise ValueError(f"深度范围非法: znear={znear} zfar={zfar}")
    return znear, zfar


def render_rgbd(model, data, camera, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
    """渲染一帧 RGB + 线性深度。

    返回 (rgb, depth_m, depth_valid)：depth_m 为相机系沿光轴距离，
    depth_valid 标记有效几何像素（非天空、非无穷远）。
    """
    znear, zfar = depth_range(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=camera)
        raw = renderer.render().copy()
        renderer.disable_depth_rendering()
    finally:
        renderer.close()
    distance = linearize_depth(raw, znear, zfar)
    valid = np.isfinite(distance)
    # MuJoCo 深度渲染把"无几何/天空"像素写成精确的 1.0，
    # 这个值远小于 zfar（本场景 zfar≈113m），只按 zfar 截断无法剔除，
    # 必须显式排除，否则背景会被当成 1m 处的平面参与拟合。
    valid &= np.asarray(raw, dtype=np.float64) != SKY_DEPTH_VALUE
    return rgb, distance, valid




def unproject(depth_m, valid, intrinsic, camera_pos, camera_rot):
    """把深度图反投影到世界系点云。

    与 mujoco_backend 中的投影约定严格互逆：
        u = f * X / depth + cu
        v = -f * Y / depth + cv
    其中 depth = -Z_cam（相机看向 -Z），因此：
        X = (u - cu) * d / f
        Y = -(v - cv) * d / f
        Z = -d
    """
    height, width = depth_m.shape
    focal = float(intrinsic["focal_px"])
    cu, cv = (float(value) for value in intrinsic["principal_point_px"])
    if focal <= 0:
        raise ValueError("焦距必须为正数")

    us, vs = np.meshgrid(np.arange(width), np.arange(height))
    d = np.asarray(depth_m, dtype=np.float64)
    mask = np.asarray(valid, dtype=bool)

    x_cam = (us - cu) * d / focal
    y_cam = -(vs - cv) * d / focal
    z_cam = -d
    points_cam = np.stack((x_cam, y_cam, z_cam), axis=-1)

    rotation = np.asarray(camera_rot, dtype=np.float64).reshape(3, 3)
    position = np.asarray(camera_pos, dtype=np.float64).reshape(3)
    points_world = points_cam.reshape(-1, 3) @ rotation.T + position
    return points_world[mask.reshape(-1)]


def camera_pose(model, data, camera_name):
    """返回相机世界位置与旋转矩阵（列为相机基向量）。"""
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam < 0:
        raise ValueError("MJCF 缺少相机: " + str(camera_name))
    position = np.asarray(data.cam_xpos[cam], dtype=np.float64).copy()
    rotation = np.asarray(data.cam_xmat[cam], dtype=np.float64).reshape(3, 3).copy()
    return cam, position, rotation
