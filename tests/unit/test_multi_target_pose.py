"""多目标 / 未知姿态链路的单元测试。

覆盖纯算法部分（不依赖 MuJoCo 渲染）：
- MAD 离群剔除
- RANSAC 平面拟合（含倾斜平面）
- 最小面积外接矩形（含 45° 旋转算例）
- 立方体 6DoF 姿态估计（含倾斜、倾斜 + yaw）
- 尺寸校验失败即抛错
- 90° 对称消歧
- 旋转矩阵/四元数互转
- 深度反投影与投影互逆
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.mujoco.depth_channel import linearize_depth  # noqa: E402
from iraf_adapters.mujoco.pose_estimation import (  # noqa: E402
    _orthonormal_basis,
    _quat_to_matrix,
    disambiguate_symmetry,
    estimate_cube_pose,
    fit_plane_ransac,
    mad_outlier_mask,
    matrix_to_quaternion,
    min_area_rect_2d,
)


def cube_top_points(half_size, euler_deg=(0.0, 0.0, 0.0), samples=40, noise=0.0, seed=0):
    """生成立方体顶面的采样点（世界系），模拟深度通道得到的顶面点云。"""
    rng = np.random.default_rng(seed)
    us = np.linspace(-half_size * 0.95, half_size * 0.95, samples)
    grid_u, grid_v = np.meshgrid(us, us)
    local = np.stack(
        [grid_u.ravel(), grid_v.ravel(), np.full(grid_u.size, half_size)], axis=1
    )
    rx, ry, rz = (math.radians(value) for value in euler_deg)
    rotation_x = np.array(
        [[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]]
    )
    rotation_y = np.array(
        [[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]]
    )
    rotation_z = np.array(
        [[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]]
    )
    rotation = rotation_z @ rotation_y @ rotation_x
    points = local @ rotation.T
    if noise > 0:
        points = points + rng.normal(scale=noise, size=points.shape)
    return points, rotation


class MadOutlierTest(unittest.TestCase):
    def test_removes_far_outliers(self):
        rng = np.random.default_rng(1)
        core = rng.normal(scale=0.001, size=(200, 3))
        outliers = np.array([[1.0, 1.0, 1.0], [-2.0, 0.5, 3.0]])
        points = np.vstack([core, outliers])
        keep = mad_outlier_mask(points, 3.0)
        # 两个远处离群点必须被剔除；正态本体的少量尾部点被剔除属统计去噪的正常行为。
        self.assertFalse(bool(keep[-1]))
        self.assertFalse(bool(keep[-2]))
        self.assertGreaterEqual(int(keep[:200].sum()), 195)


    def test_degenerate_mad_keeps_all(self):
        # MAD 为 0 时不应把全部点误剔除。
        points = np.zeros((10, 3))
        keep = mad_outlier_mask(points, 3.0)
        self.assertEqual(int(keep.sum()), 10)

    def test_rejects_non_positive_scale(self):
        with self.assertRaises(ValueError):
            mad_outlier_mask(np.zeros((5, 3)), 0.0)

    def test_flat_axis_does_not_drop_valid_points(self):
        """某个轴上约一半的点聚成极窄带时，不应把另一半误剔除。

        这是俯视立方体的真实情况：正对相机的表面在某一轴上 MAD≈0，
        逐轴独立门限会收缩到亚毫米并把同属目标的点全部滤掉。
        """
        rng = np.random.default_rng(31)
        # y 轴：一半点精确落在 0，另一半点散布在 ±0.025
        y = np.concatenate([np.zeros(150), rng.uniform(-0.025, 0.025, 150)])
        x = rng.uniform(-0.025, 0.025, 300)
        z = rng.uniform(0.0, 0.05, 300)
        points = np.stack([x, y, z], axis=1)
        keep = mad_outlier_mask(points, 3.0)
        # 目标自身的点应当基本都被保留，而不是只剩一半。
        self.assertGreater(int(keep.sum()), 250)



class PlaneFitTest(unittest.TestCase):
    def test_horizontal_plane(self):
        rng = np.random.default_rng(2)
        x = rng.uniform(-0.025, 0.025, 500)
        y = rng.uniform(-0.025, 0.025, 500)
        z = np.full(500, 0.05) + rng.normal(scale=0.0002, size=500)
        points = np.stack([x, y, z], axis=1)
        normal, offset, mask = fit_plane_ransac(points, 200, 0.001, rng)
        self.assertGreater(abs(float(normal[2])), 0.999)
        self.assertGreater(int(mask.sum()), 450)
        self.assertAlmostEqual(float(-offset), 0.05, delta=0.002)

    def test_tilted_plane(self):
        # 绕 y 轴倾斜 20° 的平面法向应为 (sin20, 0, cos20)。
        angle = math.radians(20.0)
        expected = np.array([math.sin(angle), 0.0, math.cos(angle)])
        rng = np.random.default_rng(3)
        u = rng.uniform(-0.02, 0.02, 500)
        v = rng.uniform(-0.02, 0.02, 500)
        # 平面上两点：沿 x 轴方向含倾斜分量。
        x = u * math.cos(angle)
        z = 0.05 - u * math.sin(angle)
        points = np.stack([x, v, z], axis=1)
        normal, _, mask = fit_plane_ransac(points, 300, 0.001, rng)
        if float(np.dot(normal, expected)) < 0:
            normal = -normal
        angle_error = math.degrees(
            math.acos(max(-1.0, min(1.0, abs(float(np.dot(normal, expected))))))
        )
        self.assertLess(angle_error, 1.0)

    def test_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            fit_plane_ransac(np.zeros((2, 3)), 10, 0.001)

    def test_prefers_top_face_over_larger_side_wall(self):
        """顶面与侧壁同时可见时必须选顶面，而不是面积更大的侧壁。

        俯视倾斜方块时侧壁的投影点更多，只按内点数评分会拟合到侧壁
        （实测法向偏差可达 80°）。顶面是可见面中 z 最高的平面。
        """
        rng = np.random.default_rng(11)
        # 顶面：z=0.05，40x40 网格（169 点）
        us = np.linspace(-0.02, 0.02, 13)
        gu, gv = np.meshgrid(us, us)
        top = np.stack([gu.ravel(), gv.ravel(), np.full(gu.size, 0.05)], axis=1)
        # 侧壁：y=0.02 的竖直面，点数刻意多一倍（338 点）
        side_u = np.linspace(-0.02, 0.02, 26)
        side_w = np.linspace(0.0, 0.05, 26)
        su, sw = np.meshgrid(side_u, side_w)
        side = np.stack([su.ravel(), np.full(su.size, 0.02), sw.ravel()], axis=1)
        points = np.vstack([top, side]) + rng.normal(scale=0.0002, size=(top.shape[0] + side.shape[0], 3))
        normal, offset, mask = fit_plane_ransac(points, 400, 0.0015, rng)
        # 必须恢复到顶面法向 [0,0,1]，而不是侧壁法向 [0,1,0]。
        self.assertGreater(float(normal[2]), 0.99)
        self.assertAlmostEqual(float(-offset), 0.05, delta=0.003)

    def test_rejects_side_wall_only_point_cloud(self):
        """点云只有竖直侧壁时应显式失败，而不是给出水平法向的错误姿态。"""
        rng = np.random.default_rng(12)
        u = rng.uniform(-0.02, 0.02, 400)
        w = rng.uniform(0.0, 0.05, 400)
        points = np.stack([u, np.full(400, 0.02), w], axis=1)
        with self.assertRaises(ValueError) as context:
            fit_plane_ransac(points, 200, 0.0015, rng)
        # 候选筛除阶段就会因"法向过于水平"拒绝，错误信息需指明原因。
        message = str(context.exception)
        self.assertTrue(
            "侧壁" in message or "水平" in message,
            "错误信息应说明拒绝原因，实际为: " + message,
        )




class MinAreaRectTest(unittest.TestCase):
    def test_axis_aligned_rectangle(self):
        rng = np.random.default_rng(4)
        x = rng.uniform(-0.03, 0.03, 400)
        y = rng.uniform(-0.01, 0.01, 400)
        center, axis, size = min_area_rect_2d(np.stack([x, y], axis=1))
        self.assertAlmostEqual(float(size[0]), 0.06, delta=0.002)
        self.assertAlmostEqual(float(size[1]), 0.02, delta=0.002)
        self.assertAlmostEqual(float(abs(axis[0])), 1.0, delta=0.05)

    def test_rotated_rectangle(self):
        # 45° 旋转的 60x20 矩形：最小外接矩形仍应恢复出 0.06 x 0.02。
        angle = math.radians(45.0)
        rng = np.random.default_rng(5)
        u = rng.uniform(-0.03, 0.03, 500)
        v = rng.uniform(-0.01, 0.01, 500)
        rotation = np.array(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )
        points = np.stack([u, v], axis=1) @ rotation.T
        center, axis, size = min_area_rect_2d(points)
        self.assertAlmostEqual(float(size[0]), 0.06, delta=0.003)
        self.assertAlmostEqual(float(size[1]), 0.02, delta=0.003)
        self.assertAlmostEqual(float(np.linalg.norm(center)), 0.0, delta=0.002)

    def test_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            min_area_rect_2d(np.zeros((2, 2)))


class QuaternionTest(unittest.TestCase):
    def test_identity_round_trip(self):
        quat = matrix_to_quaternion(np.eye(3))
        self.assertAlmostEqual(float(quat[0]), 1.0, delta=1e-9)
        self.assertAlmostEqual(float(np.linalg.norm(quat)), 1.0, delta=1e-9)

    def test_yaw_round_trip(self):
        angle = math.radians(35.0)
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        quat = matrix_to_quaternion(rotation)
        restored = _quat_to_matrix(quat)
        self.assertLess(float(np.abs(restored - rotation).max()), 1e-9)
        # yaw=35° 对应 w=cos(17.5°), z=sin(17.5°)
        self.assertAlmostEqual(float(quat[0]), math.cos(angle / 2.0), delta=1e-9)
        self.assertAlmostEqual(float(quat[3]), math.sin(angle / 2.0), delta=1e-9)

    def test_symmetry_disambiguation_prefers_hint(self):
        # 无提示时保持原解；给出夹爪朝向后应在 4 个等价类里挑最接近的。
        base = matrix_to_quaternion(np.eye(3))
        hint = np.array([0.0, 1.0])
        quat, index = disambiguate_symmetry(base, hint)
        matrix = _quat_to_matrix(quat)
        axis = matrix[:, 0][:2]
        axis = axis / float(np.linalg.norm(axis))
        self.assertGreater(abs(float(axis @ (hint / np.linalg.norm(hint)))), 0.999)
        self.assertIn(index, (0, 1, 2, 3))

    def test_symmetry_without_hint_is_identity(self):
        base = matrix_to_quaternion(np.eye(3))
        quat, index = disambiguate_symmetry(base, None)
        self.assertLess(float(np.abs(quat - base).max()), 1e-12)
        self.assertEqual(index, 0)


class EstimateCubePoseTest(unittest.TestCase):
    def setUp(self):
        self.half = 0.025
        self.tol = 0.006

    def test_axis_aligned_cube(self):
        points, _ = cube_top_points(self.half)
        estimate = estimate_cube_pose(
            points, self.half, self.tol, "box_a", rng=np.random.default_rng(0)
        )
        # 顶面中心在 z=+half 处生成，立方体中心应在原点。
        self.assertLess(float(np.linalg.norm(estimate.position)), 0.002)
        self.assertGreater(float(abs(estimate.normal[2])), 0.999)
        self.assertAlmostEqual(float(estimate.size_m[0]), 0.05, delta=0.003)
        self.assertAlmostEqual(float(estimate.size_m[1]), 0.05, delta=0.003)

    def test_yaw_cube(self):
        # yaw=30° 时位置仍需恢复，法向保持竖直。
        points, _ = cube_top_points(self.half, euler_deg=(0.0, 0.0, 30.0))
        estimate = estimate_cube_pose(
            points, self.half, self.tol, "box_b", rng=np.random.default_rng(0)
        )
        self.assertLess(float(np.linalg.norm(estimate.position)), 0.003)
        self.assertGreater(float(abs(estimate.normal[2])), 0.999)

    def test_tilted_cube_height_compensation(self):
        # 绕 x 轴倾斜 15°：中心高度应比顶面中心低 half*cos(15°)，
        # 若沿 Z 而非沿法向补偿会引入明显偏差。
        angle = math.radians(15.0)
        points, rotation = cube_top_points(self.half, euler_deg=(15.0, 0.0, 0.0))
        estimate = estimate_cube_pose(
            points, self.half, self.tol, "box_c", rng=np.random.default_rng(0)
        )
        expected_normal = rotation[:, 2]
        dot = abs(float(np.dot(estimate.normal, expected_normal)))
        self.assertGreater(dot, 0.999)
        # 位置在原点附近（点云是绕原点旋转生成的）。
        self.assertLess(float(np.linalg.norm(estimate.position)), 0.004)

    def test_size_mismatch_raises(self):
        # 用 20mm 半边长生成点云，却按 25mm 校验，必须显式失败。
        points, _ = cube_top_points(0.020)
        with self.assertRaises(ValueError) as context:
            estimate_cube_pose(points, self.half, 0.002, "box_d", rng=np.random.default_rng(0))
        self.assertIn("尺寸校验失败", str(context.exception))

    def test_too_few_points_raises(self):
        points = np.zeros((10, 3))
        with self.assertRaises(ValueError) as context:
            estimate_cube_pose(points, self.half, self.tol, "box_e")
        self.assertIn("点数不足", str(context.exception))

    def test_outliers_do_not_break_estimate(self):
        points, _ = cube_top_points(self.half)
        outliers = np.array([[0.5, 0.5, 0.9]] * 30)
        merged = np.vstack([points, outliers])
        estimate = estimate_cube_pose(
            merged, self.half, self.tol, "box_f", rng=np.random.default_rng(0)
        )
        self.assertLess(float(np.linalg.norm(estimate.position)), 0.003)

    def test_visible_side_wall_does_not_break_tilted_estimate(self):
        """倾斜方块同时可见顶面与侧壁时，姿态仍应来自顶面。

        这是真实俯视场景的必备能力：倾斜会露出侧壁，
        若拟合到侧壁，法向与实际姿态完全不符。
        """
        points, rotation = cube_top_points(self.half, euler_deg=(0.0, 18.0, 0.0))
        expected_normal = rotation[:, 2]
        # 沿 -y 方向补一个竖直侧壁点集（模拟俯视露出的侧面）。
        rng = np.random.default_rng(21)
        u = rng.uniform(-self.half, self.half, 400)
        w = rng.uniform(-self.half, self.half, 400)
        side = np.stack([u, np.full(400, -self.half), w], axis=1) @ rotation.T + expected_normal * self.half
        merged = np.vstack([points, side])
        estimate = estimate_cube_pose(
            merged, self.half, self.tol, "box_g", rng=np.random.default_rng(0)
        )
        dot = abs(float(np.dot(estimate.normal, expected_normal)))
        self.assertGreater(dot, 0.99)



class OrthonormalBasisTest(unittest.TestCase):
    def test_axes_are_orthonormal(self):
        matrix = _orthonormal_basis(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
        self.assertLess(float(np.abs(matrix.T @ matrix - np.eye(3)).max()), 1e-12)
        self.assertAlmostEqual(float(np.linalg.det(matrix)), 1.0, delta=1e-12)

    def test_degenerate_hint_falls_back(self):
        # 主轴与法向平行时不应产生 NaN，仍需给出合法旋转。
        matrix = _orthonormal_basis(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))
        self.assertTrue(np.all(np.isfinite(matrix)))
        self.assertLess(float(np.abs(matrix.T @ matrix - np.eye(3)).max()), 1e-12)


class DepthLinearizeTest(unittest.TestCase):
    def test_invalid_pixels_become_infinite(self):
        raw = np.array([[0.5, 1.0], [0.0, -1.0]], dtype=np.float64)
        result = linearize_depth(raw, 0.02, 100.0)
        # 0.5 与 1.0 都是合法距离；1.0 是 MuJoCo 写下的天空值，
        # 由 render_rgbd 依 SKY_DEPTH_VALUE 显式剔除，此处不应改写。
        self.assertAlmostEqual(float(result[0, 0]), 0.5, delta=1e-12)
        self.assertAlmostEqual(float(result[0, 1]), 1.0, delta=1e-12)
        # 0 与负数非法。
        self.assertTrue(np.isinf(result[1, 0]))
        self.assertTrue(np.isinf(result[1, 1]))

    def test_beyond_far_plane_becomes_infinite(self):
        raw = np.array([[150.0, 100.0]], dtype=np.float64)
        result = linearize_depth(raw, 0.02, 100.0)
        self.assertTrue(np.isinf(result[0, 0]))
        self.assertTrue(np.isinf(result[0, 1]))


    def test_rejects_invalid_range(self):
        with self.assertRaises(ValueError):
            linearize_depth(np.zeros((2, 2)), 1.0, 0.5)


if __name__ == "__main__":
    unittest.main()
