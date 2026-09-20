"""`iraf_core.kinematics` 新能力的契约测试：位姿型 IK、离台配平、重力前馈。

这些能力从 `scripts/build_ur5_baseline.py` 提到 core（见
`.hermes/plans/2026-09-20_120000-generality-step2-core-extraction.md`），
用最小 MJCF 自建模型测试，不依赖任何机型资产。
"""

import unittest

import mujoco
import numpy as np

from iraf_core.kinematics import (
    KinematicsError,
    balance_tip_clearance,
    gravity_hold_ctrl,
    solve_pose_ik,
    tool_pose_from_axes,
)

#: 单自由度**滑轨**模型：平移关节不改变姿态，因此位姿型 IK 的旋转目标
#: 恰好可达，可以把"位置收敛"与"姿态保持"分开验证。
SLIDE_MJCF = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="slider" pos="0 0 0">
      <joint name="slide" type="slide" axis="0 0 1" range="0 0.6" damping="0.5"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="tip" type="box" size="0.02 0.02 0.02" pos="0 0 0.1"/>
      <site name="flange" pos="0 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="slide_act" joint="slide" gaintype="fixed" biastype="affine"
             gainprm="1000" biasprm="0 -1000 -100" ctrlrange="0 0.6" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""


def _model(xml=SLIDE_MJCF):
    return mujoco.MjModel.from_xml_string(xml)


class ToolPoseFromAxesTests(unittest.TestCase):
    def test_returns_orthonormal_basis_with_axes_in_place(self):
        pose = tool_pose_from_axes([0, 0, -1], [1, 0, 0])
        # 列向量：x = 开合轴、z = 工具指向，构成右手系
        np.testing.assert_allclose(pose[:, 2], [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(pose[:, 0], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(pose @ pose.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(1.0, float(np.linalg.det(pose)), places=12)

    def test_accepts_non_unit_input(self):
        pose = tool_pose_from_axes([0, 0, -2.5], [3, 0, 0])
        np.testing.assert_allclose(pose[:, 2], [0, 0, -1], atol=1e-12)

    def test_rejects_non_orthogonal_axes(self):
        """姿态期望自相矛盾时必须显式失败，不能静默正交化。"""
        with self.assertRaisesRegex(KinematicsError, "正交"):
            tool_pose_from_axes([0, 0, -1], [1, 0, 1])

    def test_rejects_zero_vector_and_wrong_shape(self):
        with self.assertRaisesRegex(KinematicsError, "零向量"):
            tool_pose_from_axes([0, 0, 0], [1, 0, 0])
        with self.assertRaisesRegex(KinematicsError, "3 个数值"):
            tool_pose_from_axes([0, 0], [1, 0, 0])


class SolvePoseIkTests(unittest.TestCase):
    def setUp(self):
        self.model = _model()
        self.data = mujoco.MjData(self.model)
        self.joint = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "slide")
        )
        self.geom = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "tip")
        )
        self.site = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "flange")
        )
        self.points = [{"kind": "geom", "id": self.geom}]

    def _tip_position(self, value):
        self.data.qpos[int(self.model.jnt_qposadr[self.joint])] = value
        mujoco.mj_forward(self.model, self.data)
        return np.asarray(self.data.geom_xpos[self.geom], dtype=float)

    def test_converges_to_position_and_keeps_orientation(self):
        target = self._tip_position(0.45)
        result = solve_pose_ik(
            self.model,
            self.data,
            target,
            np.eye(3),
            [self.joint],
            self.points,
            self.site,
            iterations=200,
            tolerance_m=1e-6,
        )
        self.assertLess(result.position_error_m, 1e-6)
        self.assertLess(result.extra["rotation_error_rad"], 1e-6)
        self.assertAlmostEqual(
            0.45, result.joint_positions["slide"], places=6
        )
        self.assertEqual(0, result.extra["best_iteration"] - result.iterations)

    def test_reports_residual_when_iterations_exhausted(self):
        """迭代耗尽时返回实际残差，不伪造收敛。"""
        target = self._tip_position(0.6) + np.array([0.0, 0.5, 0.0])  # 不可达方向
        result = solve_pose_ik(
            self.model,
            self.data,
            target,
            np.eye(3),
            [self.joint],
            self.points,
            self.site,
            iterations=5,
        )
        self.assertGreater(result.position_error_m, 0.0)
        self.assertTrue(np.isfinite(result.extra["best_position_error_m"]))

    def test_rejects_bad_targets(self):
        with self.assertRaisesRegex(KinematicsError, "3 个数值"):
            solve_pose_ik(
                self.model, self.data, [0, 0], np.eye(3), [self.joint],
                self.points, self.site,
            )
        with self.assertRaisesRegex(KinematicsError, "3x3"):
            solve_pose_ik(
                self.model, self.data, [0, 0, 0], np.eye(2), [self.joint],
                self.points, self.site,
            )
        with self.assertRaisesRegex(KinematicsError, "至少需要一个臂关节"):
            solve_pose_ik(
                self.model, self.data, [0, 0, 0], np.eye(3), [],
                self.points, self.site,
            )
        with self.assertRaisesRegex(KinematicsError, "位置目标点"):
            solve_pose_ik(
                self.model, self.data, [0, 0, 0], np.eye(3), [self.joint],
                [], self.site,
            )


class BalanceTipClearanceTests(unittest.TestCase):
    """配平逻辑用替身验证：真实几何由调用方的 `solve` 与模型提供。"""

    class _FakeData:
        def __init__(self, tip_z):
            self.geom_xmat = np.tile(np.eye(3).reshape(-1), (2, 1))
            self.geom_xpos = np.array([[0.0, 0.0, tip_z], [0.0, 0.0, tip_z]])

    class _FakeModel:
        geom_type = np.array([mujoco.mjtGeom.mjGEOM_BOX] * 2)
        # 半尺寸取 0：本用例只验证"迭代抬高"的循环逻辑，
        # box 的旋转投影边界由 UR5 验收覆盖（那里是真实 pad box）。
        geom_size = np.array([[0.0, 0.0, 0.0]] * 2)
        geom_dataid = np.array([-1, -1])

    def _run(self, tip_series, clearance=0.003):
        data = self._FakeData(tip_series[0])
        calls = []

        def solve(target):
            calls.append(np.asarray(target, dtype=float).copy())
            data.geom_xpos[:, 2] = tip_series[min(len(calls) - 1, len(tip_series) - 1)]

            class _Result:
                position_error_m = 1e-7
                extra = {"rotation_error_rad": 0.0}

            return _Result()

        result, correction, trace, cleared = balance_tip_clearance(
            self._FakeModel(),
            data,
            solve,
            [0.0, 0.0, 0.025],
            [0.0, 0.0, 1.0],
            [0, 1],
            0.0,
            clearance,
        )
        return result, correction, trace, cleared, calls

    def test_raises_target_until_clearance_satisfied(self):
        # 第一轮指尖低于台面 5mm → 抬高 8mm 后达标
        result, correction, trace, cleared, calls = self._run([-0.005, 0.004])
        self.assertTrue(cleared)
        self.assertEqual(2, len(trace))
        self.assertAlmostEqual(0.008, correction, places=9)
        # 第二轮目标确实沿方向抬高了
        np.testing.assert_allclose(calls[1], [0.0, 0.0, 0.033], atol=1e-12)
        self.assertEqual(0, trace[0]["iteration"])

    def test_reports_not_cleared_without_raising(self):
        result, correction, trace, cleared, calls = self._run([-0.005])
        self.assertFalse(cleared)
        self.assertGreater(correction, 0.0)
        self.assertEqual(8, len(trace))  # 默认迭代上限

    def test_rejects_zero_direction(self):
        with self.assertRaisesRegex(KinematicsError, "零向量"):
            balance_tip_clearance(
                self._FakeModel(), self._FakeData(0.0), lambda target: None,
                [0, 0, 0.02], [0, 0, 0], [0], 0.0, 0.003,
            )


class GravityHoldCtrlTests(unittest.TestCase):
    def setUp(self):
        self.model = _model()
        self.joint_name = "slide"
        # 质量 2kg，重力 9.81 → 需要 19.62N；执行器增益 1000 → 前馈 0.0196
        self.data = mujoco.MjData(self.model)
        self.data.qpos[int(self.model.jnt_qposadr[0])] = 0.3
        mujoco.mj_forward(self.model, self.data)
        self.expected_torque = float(self.data.qfrc_bias[int(self.model.jnt_dofadr[0])])

    def test_feedforward_equals_gravity_torque_over_gain(self):
        offsets, evidence = gravity_hold_ctrl(
            self.model, [self.joint_name], {self.joint_name: 0.3}, hold_ms=2000
        )
        expected = self.expected_torque / 1000.0
        self.assertAlmostEqual(expected, offsets[self.joint_name], places=9)
        # 自证：静态保持后关节误差在门限内
        self.assertLessEqual(evidence["worst_residual_rad"], evidence["tolerance_rad"])
        self.assertEqual("qfrc_bias_plus_static_hold", evidence["method"])

    def test_zero_feedforward_when_gravity_cancelled(self):
        """无重力时前馈必须为零：结构性判据，不依赖具体数值。"""
        model = _model(SLIDE_MJCF.replace('gravity="0 0 -9.81"', 'gravity="0 0 0"'))
        offsets, _ = gravity_hold_ctrl(model, ["slide"], {"slide": 0.2}, hold_ms=200)
        self.assertAlmostEqual(0.0, offsets["slide"], places=12)

    def test_does_not_mutate_caller_state(self):
        data = mujoco.MjData(self.model)
        before = np.array(data.qpos, dtype=float)
        gravity_hold_ctrl(self.model, ["slide"], {"slide": 0.25}, hold_ms=200)
        np.testing.assert_allclose(before, np.array(data.qpos, dtype=float))

    def test_rejects_unknown_joint_and_missing_target(self):
        with self.assertRaisesRegex(KinematicsError, "缺少臂关节"):
            gravity_hold_ctrl(self.model, ["nope"], {"nope": 0.1})
        with self.assertRaisesRegex(KinematicsError, "缺少臂关节目标位形"):
            gravity_hold_ctrl(self.model, ["slide"], {})


if __name__ == "__main__":
    unittest.main()
