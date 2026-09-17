"""纯 numpy 的立方体 6DoF 姿态估计。

设计依据（俯视单目 + 深度通道的几何限制）：
相机只能看到顶面与少量侧壁，因此直接对"全部可见表面点"做 PCA
会被可见面偏置带偏。稳健流程是：

1. MAD 统计去噪，剔除台面与背景边缘的离群点；
2. RANSAC 三点法拟合**最大支撑平面**（即方块顶面）→ 法向 n；
3. 顶面点投影到平面内，用**旋转卡壳**求最小面积外接矩形
   → 中心 c、边长 (w, h)、面内主轴 a；
4. 校验 (w, h) ≈ 2*half_size（超出容差即抛错，不做静默兜底）；
5. 顶面中心沿 -n 偏移 half_size 得立方体几何中心；
6. 构造旋转矩阵 z=n、x=a（正交化）、y=z×x，并做**立方体 90° 对称消歧**；
7. 输出四元数（wxyz）与残差指标。

不引入 OpenCV / Open3D / scipy，全部用 numpy 向量化实现。
"""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class PoseEstimate:
    """单个目标的 6DoF 位姿估计结果（世界系）。"""

    target_id: str
    position: np.ndarray
    quaternion: np.ndarray
    normal: np.ndarray
    size_m: tuple
    residual_m: float
    num_points: int
    pixel_center: tuple = field(default=None)
    diagnostics: dict = field(default_factory=dict)

    def as_dict(self):
        return {
            "id": self.target_id,
            "position_m": [round(float(value), 9) for value in self.position],
            "quaternion_wxyz": [round(float(value), 9) for value in self.quaternion],
            "normal": [round(float(value), 9) for value in self.normal],
            "size_m": [round(float(value), 6) for value in self.size_m],
            "residual_m": round(float(self.residual_m), 9),
            "num_points": int(self.num_points),
            "pixel_center": (
                [round(float(value), 3) for value in self.pixel_center]
                if self.pixel_center
                else None
            ),
            "diagnostics": self.diagnostics,
        }


def mad_outlier_mask(points, scale=3.0):
    """用 MAD 剔除离群点。

    判据基于**到点云中心的距离**而非逐轴独立门限：
    逐轴 MAD 在多面体点云上会失效——当某个轴上有约一半的点恰好聚在
    极窄的带上（例如正对相机的立方体表面），该轴的 MAD 接近 0，
    门限随之收缩到亚毫米，把同属目标的另一半点全部误剔除。

    距离是旋转不变的，对"目标 + 台面/背景"这类外点结构更稳健。
    1.4826 使距离 MAD 与正态标准差可比。
    """
    if scale <= 0:
        raise ValueError("outlier_mad_scale 必须为正数")
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points 必须是 (N,3) 数组")
    center = np.median(values, axis=0)
    distance = np.linalg.norm(values - center, axis=1)
    median = float(np.median(distance))
    mad = float(np.median(np.abs(distance - median)))
    if mad <= 1e-12:
        # 距离高度集中时不做剔除，避免把有效点全部滤掉。
        return np.ones(values.shape[0], dtype=bool)
    limit = median + scale * 1.4826 * mad
    return distance <= limit



def _refine_plane(values, mask, inlier_m):
    """用内点做最小二乘精修，返回 (法向, 偏移, 精修后内点掩码)。"""
    # PCA 最小奇异向量即法向。
    inliers = values[mask]
    center = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / float(np.linalg.norm(normal))
    offset = -float(normal @ center)
    # 用精修后的平面重新统计内点，避免三点初值偏差带来的内点集偏斜。
    distance = np.abs(values @ normal + offset)
    refined = distance <= inlier_m
    if int(refined.sum()) < 3:
        refined = mask
        inliers = values[mask]
        center = inliers.mean(axis=0)
        _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
        normal = vh[-1]
        normal = normal / float(np.linalg.norm(normal))
        offset = -float(normal @ center)
    return normal, offset, refined


def fit_plane_ransac(points, iterations=200, inlier_m=0.002, rng=None):
    """RANSAC 三点法拟合平面，返回 (法向, 平面偏移, 内点掩码)。

    平面方程 n·x + d = 0，法向单位化。

    评分准则不是单纯的内点数最大，而是"内点最多，且平面在候选中的高度最高"：
    俯视视角下倾斜方块的可见面同时包含**顶面与侧壁**，侧壁因投影面积大
    往往给出更多内点，只按内点数选会拟合到竖直侧壁（实测法向偏差可达 80°）。
    顶面必然是最靠近相机（世界系 z 最大）的那个面，因此以高度为主判据。
    """
    values = np.asarray(points, dtype=np.float64)
    count = values.shape[0]
    if count < 3:
        raise ValueError("平面拟合至少需要 3 个点")
    if rng is None:
        rng = np.random.default_rng(0)

    candidates = []
    for _ in range(int(iterations)):
        idx = rng.choice(count, size=3, replace=False)
        p0, p1, p2 = values[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal = normal / norm
        # 法向统一朝上，保证"高度"这一判据在不同采样下含义一致。
        if normal[2] < 0:
            normal = -normal
        offset = -float(normal @ p0)
        distance = np.abs(values @ normal + offset)
        mask = distance <= inlier_m
        total = int(mask.sum())
        if total < 3:
            continue
        inlier_points = values[mask]
        center = inlier_points.mean(axis=0)
        # 平面高度：内点重心沿世界系 z 的高度。
        height = float(center[2])
        # 竖直侧壁的判据：法向接近水平（z 分量很小）时不可作为顶面。
        if abs(float(normal[2])) < 0.5:
            continue
        candidates.append((total, height, mask, normal, offset))

    if not candidates:
        raise ValueError("RANSAC 未能拟合出有效平面（候选均非水平顶面）")

    # 先按高度取最高的若干候选，再在其中选内点最多的：
    # 顶面一定是可见面里 z 最高的平面，单纯比内点数会被大面积侧壁带偏。
    candidates.sort(key=lambda item: item[1], reverse=True)
    top = candidates[: max(1, len(candidates) // 5)]
    best = max(top, key=lambda item: item[0])
    normal, offset, mask = _refine_plane(values, best[2], inlier_m)

    # 精修可能把法向翻转到下方，统一朝上。
    if normal[2] < 0:
        normal = -normal
        offset = -offset
    if abs(float(normal[2])) < 0.5:
        # 精修后仍接近竖直说明点云不含可用顶面，显式失败而非给出错误姿态。
        raise ValueError(
            f"拟合平面法向过于水平（z={float(normal[2]):.4f}），点云可能只有侧壁"
        )
    return normal, offset, mask



def min_area_rect_2d(points_2d):
    """旋转卡壳求最小面积外接矩形。

    输入 (N,2) 平面内点，返回 (center(2,), axis(2,), size(2,))：
    axis 为矩形长边方向单位向量，size 为 (长, 宽)。
    """
    values = np.asarray(points_2d, dtype=np.float64)
    if values.shape[0] < 3:
        raise ValueError("最小外接矩形至少需要 3 个点")
    center_all = values.mean(axis=0)
    centered = values - center_all

    # 候选角度取所有点对方向的法向，凸包边缘方向必然来自点构成的方向。
    angles = np.arctan2(centered[:, 1], centered[:, 0])
    # 只用 [0, pi/2) 内的角度即可覆盖矩形的全部等价朝向。
    candidates = np.unique(np.mod(angles, math.pi / 2.0))
    if candidates.size > 512:
        # 候选过多时均匀抽样，避免 O(N^2) 退化为性能瓶颈。
        candidates = np.linspace(0.0, math.pi / 2.0, 512, endpoint=False)

    best = None
    for theta in candidates:
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        rotation = np.array([[cos_t, sin_t], [-sin_t, cos_t]])
        rotated = centered @ rotation.T
        low = rotated.min(axis=0)
        high = rotated.max(axis=0)
        size = high - low
        area = float(size[0] * size[1])
        if best is None or area < best[0]:
            local_center = (low + high) / 2.0
            world_center = rotation.T @ local_center + center_all
            axis = rotation.T @ np.array([1.0, 0.0])
            best = (area, world_center, axis, size)
    _, center, axis, size = best
    axis = axis / float(np.linalg.norm(axis))
    # 保证长边在前，便于对照 half_size 校验。
    if size[0] < size[1]:
        size = np.array([size[1], size[0]])
        axis = np.array([-axis[1], axis[0]])
    return center, axis, size


def _orthonormal_basis(normal, axis_hint):
    """构造以 normal 为 z、axis_hint 为 x 的正交基。"""
    z_axis = np.asarray(normal, dtype=np.float64)
    z_axis = z_axis / float(np.linalg.norm(z_axis))
    hint = np.asarray(axis_hint, dtype=np.float64)
    hint = hint - z_axis * float(hint @ z_axis)
    norm = float(np.linalg.norm(hint))
    if norm < 1e-9:
        # 主轴与法向平行（退化）时选任意正交向量，保证旋转矩阵仍然合法。
        fallback = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        hint = fallback - z_axis * float(fallback @ z_axis)
        norm = float(np.linalg.norm(hint))
    x_axis = hint / norm
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def matrix_to_quaternion(matrix):
    """旋转矩阵转 wxyz 四元数（与 mujoco.mju_mat2Quat 同约定）。"""
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=np.float64)
    return quat / float(np.linalg.norm(quat))


def _quat_multiply(a, b):
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def disambiguate_symmetry(quaternion, gripper_yaw_hint=None):
    """立方体 yaw 有 90° 等价类，取与夹爪朝向夹角最小的解。

    旋转矩阵的 z 轴（顶面法向）在等价类内不变，
    只有面内 x/y 轴按 90° 旋转，因此只需在 4 个候选里挑最优。
    """
    quat = np.asarray(quaternion, dtype=np.float64)
    quat = quat / float(np.linalg.norm(quat))
    if gripper_yaw_hint is None:
        return quat, 0
    hint = np.asarray(gripper_yaw_hint, dtype=np.float64)
    hint = hint[:2]
    norm = float(np.linalg.norm(hint))
    if norm < 1e-9:
        return quat, 0
    hint = hint / norm
    # 绕 z 轴旋转 90° 的四元数
    half = math.pi / 4.0
    step = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)
    best = None
    current = quat.copy()
    for index in range(4):
        matrix = _quat_to_matrix(current)
        # 等价类内 x 轴在世界系 xy 平面的投影
        axis = matrix[:, 0][:2]
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm > 1e-9:
            axis = axis / axis_norm
            score = abs(float(axis @ hint))
        else:
            score = 0.0
        if best is None or score > best[0]:
            best = (score, current.copy(), index)
        current = _quat_multiply(step, current)
    return best[1], best[2]


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


def estimate_cube_pose(
    points_world,
    half_size_m,
    size_tolerance_m,
    target_id="target",
    gripper_yaw_hint=None,
    ransac_iterations=200,
    ransac_inlier_m=0.002,
    outlier_mad_scale=3.0,
    min_points=100,
    max_points=5000,
    rng=None,
):
    """从世界系点云估计立方体 6DoF 位姿。

    任一步失败都抛带数值的异常，不做静默兜底——静默兜底会让验收失去意义。
    """
    values = np.asarray(points_world, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points_world 必须是 (N,3) 数组")
    if values.shape[0] < min_points:
        raise ValueError(
            f"目标 {target_id} 点数不足: {values.shape[0]} < {min_points}"
        )
    half_size = float(half_size_m)
    if half_size <= 0:
        raise ValueError("half_size_m 必须为正数")

    # 下采样控制耗时：点数超过阈值时随机抽样，对拟合精度影响可忽略。
    if values.shape[0] > max_points:
        if rng is None:
            rng = np.random.default_rng(0)
        index = rng.choice(values.shape[0], size=int(max_points), replace=False)
        values = values[np.sort(index)]

    keep = mad_outlier_mask(values, outlier_mad_scale)
    filtered = values[keep]
    if filtered.shape[0] < min_points:
        raise ValueError(
            f"目标 {target_id} 去噪后点数不足: {filtered.shape[0]} < {min_points}"
        )

    normal, offset, inlier_mask = fit_plane_ransac(
        filtered,
        iterations=ransac_iterations,
        inlier_m=ransac_inlier_m,
        rng=rng,
    )
    plane_points = filtered[inlier_mask]
    if plane_points.shape[0] < 3:
        raise ValueError(f"目标 {target_id} 顶面内点不足: {plane_points.shape[0]}")

    # 顶面法向统一朝上（工作台上的物体顶面法向与重力反向）。
    if float(normal[2]) < 0:
        normal = -normal
        offset = -offset

    # 把顶面点投影到平面内的二维坐标系，求最小面积外接矩形。
    basis = _orthonormal_basis(normal, np.array([1.0, 0.0, 0.0]))
    plane_center = plane_points.mean(axis=0)
    local = (plane_points - plane_center) @ basis
    rect_center_2d, rect_axis_2d, size = min_area_rect_2d(local[:, :2])

    rect_center_3d = basis @ np.array(
        [rect_center_2d[0], rect_center_2d[1], 0.0]
    ) + plane_center
    axis_3d = basis[:, :2] @ rect_axis_2d
    axis_3d = axis_3d - normal * float(axis_3d @ normal)
    axis_norm = float(np.linalg.norm(axis_3d))
    if axis_norm < 1e-9:
        raise ValueError(f"目标 {target_id} 面内主轴退化")
    axis_3d = axis_3d / axis_norm

    expected = 2.0 * half_size
    errors = [abs(float(value) - expected) for value in size]
    if max(errors) > float(size_tolerance_m):
        raise ValueError(
            f"目标 {target_id} 尺寸校验失败: fitted={[round(float(v), 6) for v in size]} "
            f"expected={expected:.6f} max_error={max(errors):.6f} "
            f"tolerance={float(size_tolerance_m):.6f}"
        )

    # 顶面中心沿法向反向偏移半高，得到立方体几何中心。
    position = rect_center_3d - normal * half_size
    rotation = _orthonormal_basis(normal, axis_3d)
    quaternion = matrix_to_quaternion(rotation)
    quaternion, symmetry_index = disambiguate_symmetry(quaternion, gripper_yaw_hint)

    residual = float(
        np.sqrt(((plane_points @ normal + offset) ** 2).mean())
    )
    return PoseEstimate(
        target_id=str(target_id),
        position=position,
        quaternion=quaternion,
        normal=normal,
        size_m=(float(size[0]), float(size[1])),
        residual_m=residual,
        num_points=int(plane_points.shape[0]),
        diagnostics={
            "input_points": int(values.shape[0]),
            "filtered_points": int(filtered.shape[0]),
            "plane_offset": round(float(offset), 9),
            "size_errors_m": [round(float(value), 6) for value in errors],
            "symmetry_index": int(symmetry_index),
            "ransac_iterations": int(ransac_iterations),
            "ransac_inlier_m": float(ransac_inlier_m),
        },
    )
