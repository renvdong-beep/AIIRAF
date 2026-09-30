"""抓取段运行期纠偏的回归测试（2026-09-30 §11.29）。

被测的是**声明达标与否的判定**（纯函数）与**声明解析**（后端解析层）：两者共同保证
"声明了就必须被消费、不合规的声明必须显式失败"，而 IK 与三条门禁（残差/姿态/侵入）在
`MujocoBackend._correct_grasp_column`，由端到端验收覆盖（`nominal --world joint` 的 s06）。

实测背景：构建期关节解按**标称目标**求，而联合世界里目标被搬动（卸载步实测偏 15.812 mm）
⇒ 运行期按实测目标重解下压/抬升两段，**不放宽判据**。
"""

import unittest

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_adapters.mujoco.payload_facts import (
    GRASP_POSE_CORRECTION_MODES,
    resolve_grasp_pose_correction,
)

DECLARATION = {"mode": "resolved", "residual_tolerance_m": 0.002, "max_correction_m": 0.05,
               "ik_iterations": 400, "ik_step": 0.5, "ik_tolerance_m": 0.0005, "max_axis_deg": 3.0,
               "align_max_attempts": 3}


class ResolveGraspPoseCorrectionTests(unittest.TestCase):
    def test_measured_unload_offset_requires_correction(self):
        """卸载步实测偏差 [0.011634776…, −0.002302729…, 0.01045626…] ⇒ 需要纠偏且在限内。

        注意口径差：这里的 `norm_m` 是「实测目标 − **名义解抓取点**」的模（0.015811514 m），
        与运行期到位门禁报的 `distance=0.015812 m`（实测目标 − DESCEND 后实测夹持区中点）同量级
        但**不是同一个量**（后者还含臂的静态保持残差）。
        """
        report = resolve_grasp_pose_correction(
            DECLARATION, [0.011634776373839473, -0.0023027294396569253, 0.010456260989722743])
        self.assertAlmostEqual(report["norm_m"], 0.015811514, places=8)
        self.assertTrue(report["required"])
        self.assertFalse(report["refused"])
        self.assertFalse(report["within_tolerance"])
        self.assertTrue(report["within_limit"])

    def test_small_offset_is_not_corrected_but_is_reported(self):
        report = resolve_grasp_pose_correction(DECLARATION, [0.0005, 0.0, 0.0])
        self.assertFalse(report["required"])
        self.assertFalse(report["applied"] if "applied" in report else False)
        self.assertIn("无需修正", report["reason"])

    def test_over_limit_is_refused(self):
        """超出声明上限 ⇒ refused（调用方必须显式失败：这不是"小偏差"）。"""
        report = resolve_grasp_pose_correction(DECLARATION, [0.2, 0.0, 0.0])
        self.assertTrue(report["refused"])
        self.assertFalse(report["within_limit"])
        self.assertIn("上限", report["reason"])

    def test_measure_only_never_corrects(self):
        declaration = dict(DECLARATION, mode="measure_only")
        report = resolve_grasp_pose_correction(declaration, [0.02, 0.0, 0.0])
        self.assertFalse(report["required"])
        self.assertIn("measure_only", report["reason"])

    def test_rejects_unknown_mode_and_bad_numbers(self):
        for bad in (dict(DECLARATION, mode="auto"), dict(DECLARATION, residual_tolerance_m=0.0),
                    dict(DECLARATION, max_correction_m=-1.0),
                    dict(DECLARATION, residual_tolerance_m=0.06)):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    resolve_grasp_pose_correction(bad, [0.01, 0.0, 0.0])

    def test_modes_tuple_matches_declaration_contract(self):
        self.assertEqual(("measure_only", "resolved"), GRASP_POSE_CORRECTION_MODES)


def _gripper(**overrides):
    """最小可解析的夹爪配置（Piper 口径；与 `test_mujoco_pick_config._gripper` 同形）。"""
    config = {
        "wrist_body": "link6",
        "left_finger_body": "link7",
        "right_finger_body": "link8",
        "left_finger_geom": "piper_left_finger",
        "right_finger_geom": "piper_right_finger",
        "open_positions": {"joint7": 0.035, "joint8": -0.035},
        "closed_positions": {"joint7": 0.0, "joint8": 0.0},
        "home_positions": {"joint7": 0.035, "joint8": -0.035},
        "approach_positions": {"joint7": 0.035, "joint8": -0.035},
        "grasp_positions": {"joint7": 0.035, "joint8": -0.035},
        "lift_positions": {"joint7": 0.0, "joint8": 0.0},
    }
    config.update(overrides)
    return config


class GraspPoseCorrectionParseTests(unittest.TestCase):
    """解析层：声明必须被消费，非法声明显式失败（缺声明 = 不纠偏，行为不变）。"""

    def test_absent_declaration_is_none(self):
        parsed = MujocoBackend._parse_manipulation_config({"gripper": _gripper()})
        self.assertIsNone(parsed["gripper"]["grasp_pose_correction"])

    def test_parses_full_declaration(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {"gripper": _gripper(grasp_pose_correction=dict(DECLARATION))})
        correction = parsed["gripper"]["grasp_pose_correction"]
        self.assertEqual("resolved", correction["mode"])
        self.assertEqual(400, correction["ik_iterations"])
        self.assertAlmostEqual(0.002, correction["residual_tolerance_m"])

    def test_rejects_unknown_field(self):
        with self.assertRaisesRegex(ValueError, "未知字段"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": _gripper(grasp_pose_correction=dict(DECLARATION, magic=1))})

    def test_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "mode"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": _gripper(grasp_pose_correction=dict(DECLARATION, mode="auto"))})

    def test_resolved_mode_requires_ik_keys(self):
        partial = {k: v for k, v in DECLARATION.items()
                   if k not in ("ik_iterations", "ik_step", "ik_tolerance_m", "max_axis_deg",
                                "align_max_attempts")}
        with self.assertRaisesRegex(ValueError, "ik_iterations"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": _gripper(grasp_pose_correction=partial)})

    def test_measure_only_needs_only_tolerances(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {"gripper": _gripper(grasp_pose_correction={
                "mode": "measure_only", "residual_tolerance_m": 0.002, "max_correction_m": 0.05})})
        self.assertEqual("measure_only",
                         parsed["gripper"]["grasp_pose_correction"]["mode"])


if __name__ == "__main__":
    unittest.main()
