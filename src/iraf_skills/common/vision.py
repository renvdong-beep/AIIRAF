"""可替换的视觉抓取数据处理接口。

实现只依赖 NumPy 风格数组；GraspNet、Open3D 等重依赖由上层适配器注入，
避免把特定模型和 UR5 环境耦合进 AIIRAF Runtime。
"""

import math


def depth_to_point_cloud(depth, intrinsics, depth_scale=1.0, max_depth_m=2.0):
    """将深度图转换为有效点云，返回 ``[(x, y, z), ...]``。"""
    if depth_scale <= 0 or not math.isfinite(float(depth_scale)):
        raise ValueError("depth_scale 必须为正有限数")
    fx, fy, cx, cy = (float(intrinsics[key]) for key in ("fx", "fy", "cx", "cy"))
    if min(fx, fy) <= 0 or not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        raise ValueError("相机内参无效")
    limit = float(max_depth_m)
    if limit <= 0 or not math.isfinite(limit):
        raise ValueError("max_depth_m 必须为正有限数")
    points = []
    for row, values in enumerate(depth):
        for col, raw in enumerate(values):
            z = float(raw) / depth_scale
            if not math.isfinite(z) or z <= 0 or z > limit:
                continue
            points.append(((col - cx) * z / fx, (row - cy) * z / fy, z))
    return points


def sample_points(points, count, seed=0):
    """确定性采样点云，数量不足时允许重复采样。"""
    import random

    count = int(count)
    if count < 1 or not points:
        raise ValueError("点云和采样数量必须非空")
    rng = random.Random(seed)
    if len(points) >= count:
        return [points[index] for index in rng.sample(range(len(points)), count)]
    return list(points) + [points[rng.randrange(len(points))] for _ in range(count - len(points))]


def filter_grasp_candidates(candidates, occupied_points, collision_radius_m=0.01):
    """按点云近邻碰撞过滤抓取候选，候选需提供 ``position``。"""
    radius = float(collision_radius_m)
    if radius <= 0 or not math.isfinite(radius):
        raise ValueError("collision_radius_m 必须为正有限数")
    radius_sq = radius * radius
    accepted = []
    for candidate in candidates:
        position = candidate.get("position") if isinstance(candidate, dict) else None
        if not position or len(position) != 3:
            continue
        if any(sum((float(position[i]) - float(point[i])) ** 2 for i in range(3)) < radius_sq for point in occupied_points):
            continue
        accepted.append(candidate)
    return accepted
