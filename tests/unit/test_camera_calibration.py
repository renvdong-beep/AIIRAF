"""相机标定单元测试：外参刚体拟合与内参 LM 求解。

不依赖 MuJoCo 渲染，只用合成数据验证数学正确性，
可在 CI 中无显示环境运行。
"""

import math
import unittest

import numpy as np

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend


class _StubAuthority:
    def validate(self, lease):
        return None


class _StubBackend(MujocoBackend):
    """绕过 __init__，只测试标定数学。"""

    def __init__(self):
        self.authority = _StubAuthority()


def _rotation_z(angle):
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


class CameraExtrinsicsTests(unittest.TestCase):
    def setUp(self):
        self.backend = _StubBackend()

    def test_recovers_known_rigid_transform(self):
        rotation = _rotation_z(0.4)
        translation = np.array([0.3, -0.2, 0.9])
        camera_points = np.array(
            [
                [0.10, 0.05, -0.80],
                [-0.12, 0.03, -0.75],
                [0.08, -0.09, -0.90],
                [-0.05, -0.07, -1.05],
                [0.15, 0.11, -0.95],
            ]
        )
        base_points = camera_points @ rotation.T + translation
        pairs = [
            {
                "camera_m": [float(v) for v in camera_points[i]],
                "base_m": [float(v) for v in base_points[i]],
            }
            for i in range(len(camera_points))
        ]
        evidence = self.backend.calibrate_camera_to_base(
            {"pairs": pairs, "tolerance_m": 1e-6}, None
        )
        self.assertTrue(evidence["passed"])
        self.assertLess(evidence["max_error_m"], 1e-9)
        recovered = np.asarray(evidence["rotation_matrix"], dtype=float)
        np.testing.assert_allclose(recovered, rotation, atol=1e-9)
        np.testing.assert_allclose(
            np.asarray(evidence["translation_m"], dtype=float), translation, atol=1e-9
        )

    def test_rejects_too_few_pairs(self):
        with self.assertRaisesRegex(ValueError, "至少需要 4 组"):
            self.backend.calibrate_camera_to_base(
                {"pairs": [{"camera_m": [0, 0, -1], "base_m": [0, 0, 1]}]}, None
            )

    def test_rejects_non_finite_points(self):
        pairs = [
            {"camera_m": [0.1, 0.0, -1.0], "base_m": [0.1, 0.0, 1.0]},
            {"camera_m": [0.2, 0.0, -1.0], "base_m": [0.2, 0.0, 1.0]},
            {"camera_m": [0.3, 0.0, -1.0], "base_m": [0.3, 0.0, 1.0]},
            {"camera_m": [math.inf, 0.0, -1.0], "base_m": [0.4, 0.0, 1.0]},
        ]
        with self.assertRaisesRegex(ValueError, "有限数值"):
            self.backend.calibrate_camera_to_base({"pairs": pairs}, None)

    def test_rejects_malformed_pair(self):
        pairs = [
            {"camera_m": [0.1, 0.0, -1.0], "base_m": [0.1, 0.0, 1.0]},
            {"camera_m": [0.2, 0.0], "base_m": [0.2, 0.0, 1.0]},
            {"camera_m": [0.3, 0.0, -1.0], "base_m": [0.3, 0.0, 1.0]},
            {"camera_m": [0.4, 0.0, -1.0], "base_m": [0.4, 0.0, 1.0]},
        ]
        with self.assertRaisesRegex(ValueError, "3 个数值"):
            self.backend.calibrate_camera_to_base({"pairs": pairs}, None)


class CameraIntrinsicsTests(unittest.TestCase):
    """内参应能从合成像素数据中恢复出真值。"""

    def setUp(self):
        self.backend = _StubBackend()
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self.focal_true = 296.4
        self.center_true = (320.0, 240.0)

    def _pairs(self):
        rotation = _rotation_z(0.3)
        translation = np.array([0.0, 0.2, 0.5])
        camera_points = np.array(
            [
                [-0.15, -0.10, -1.10],
                [0.18, -0.12, -1.05],
                [0.02, 0.20, -1.20],
                [-0.20, 0.14, -1.15],
                [0.12, 0.05, -0.95],
                [-0.08, -0.18, -1.25],
                [0.22, 0.17, -1.30],
                [0.00, 0.00, -1.00],
            ]
        )
        base_points = camera_points @ rotation.T + translation
        return rotation, translation, camera_points, base_points

    def test_recovers_synthetic_intrinsics(self):
        rotation, translation, camera_points, base_points = self._pairs()
        samples = []
        for index in range(len(camera_points)):
            x, y, z = camera_points[index]
            depth = -float(z)
            u = self.focal_true * float(x) / depth + self.center_true[0]
            v = -self.focal_true * float(y) / depth + self.center_true[1]
            samples.append(
                {"pixel": [u, v], "base_m": [float(value) for value in base_points[index]]}
            )
        evidence = self.backend.calibrate_camera_to_base(
            {
                "pairs": [
                    {
                        "camera_m": [float(v) for v in camera_points[i]],
                        "base_m": [float(v) for v in base_points[i]],
                    }
                    for i in range(len(camera_points))
                ],
                "intrinsics_samples": samples,
                "image_size_px": [640, 480],
            },
            None,
        )
        intrinsics = evidence["intrinsics"]
        self.assertAlmostEqual(self.focal_true, intrinsics["focal_px"], delta=0.5)
        self.assertAlmostEqual(
            self.center_true[0], intrinsics["principal_point_px"][0], delta=0.5
        )
        self.assertAlmostEqual(
            self.center_true[1], intrinsics["principal_point_px"][1], delta=0.5
        )
        self.assertLess(intrinsics["residual_px_rms"], 0.01)

    def test_rejects_sample_behind_camera(self):
        _, _, _, base_points = self._pairs()
        samples = [
            {"pixel": [320.0, 240.0], "base_m": [float(v) for v in base_points[0]]},
            {"pixel": [321.0, 241.0], "base_m": [0.0, 0.0, 5.0]},
            {"pixel": [322.0, 242.0], "base_m": [float(v) for v in base_points[2]]},
            {"pixel": [323.0, 243.0], "base_m": [float(v) for v in base_points[3]]},
        ]
        with self.assertRaisesRegex(ValueError, "相机前方"):
            self.backend._calibrate_intrinsics(
                samples, [640, 480], np.eye(3), np.zeros(3)
            )

    def test_rejects_low_lateral_spread(self):
        samples = [
            {"pixel": [320.0, 240.0], "base_m": [0.0, 0.0, -1.0]},
            {"pixel": [321.0, 241.0], "base_m": [0.0, 0.0, -1.1]},
            {"pixel": [322.0, 242.0], "base_m": [0.0, 0.0, -1.2]},
            {"pixel": [323.0, 243.0], "base_m": [0.0, 0.0, -1.3]},
        ]
        with self.assertRaisesRegex(ValueError, "横向散布过小"):
            self.backend._calibrate_intrinsics(
                samples, [640, 480], np.eye(3), np.zeros(3)
            )

    def test_skips_intrinsics_when_absent(self):
        rotation, translation, camera_points, base_points = self._pairs()
        evidence = self.backend.calibrate_camera_to_base(
            {
                "pairs": [
                    {
                        "camera_m": [float(v) for v in camera_points[i]],
                        "base_m": [float(v) for v in base_points[i]],
                    }
                    for i in range(len(camera_points))
                ]
            },
            None,
        )
        self.assertNotIn("intrinsics", evidence)


if __name__ == "__main__":
    unittest.main()
