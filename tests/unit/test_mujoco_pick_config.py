import unittest

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend


def _gripper(**overrides):
    """最小可解析的夹爪配置（Piper 口径），其余字段用 overrides 覆盖。"""
    config = {
        "left_finger_body": "link7",
        "right_finger_body": "link8",
        "open_positions": {"joint7": 0.035, "joint8": -0.035},
        "closed_positions": {"joint7": 0.0, "joint8": 0.0},
        "home_positions": {"joint7": 0.035, "joint8": -0.035},
        "approach_positions": {"joint7": 0.035, "joint8": -0.035},
        "grasp_positions": {"joint7": 0.035, "joint8": -0.035},
        "lift_positions": {"joint7": 0.0, "joint8": 0.0},
    }
    config.update(overrides)
    return config


class MujocoPickConfigTests(unittest.TestCase):
    def test_parses_bounded_target_and_gripper(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {
                "targets": {
                    "box_01": {"body": "box_01", "pose_tolerance_m": 0.02}
                },
                "gripper": {
                    "left_finger_body": "link7",
                    "right_finger_body": "link8",
                    "open_positions": {"joint7": 0.035, "joint8": -0.035},
                    "closed_positions": {"joint7": 0.0, "joint8": 0.0},
                },
            }
        )

        self.assertEqual("box_01", parsed["targets"]["box_01"]["body"])
        self.assertEqual(-0.035, parsed["gripper"]["open_positions"]["joint8"])

    def test_rejects_incomplete_gripper(self):
        with self.assertRaisesRegex(ValueError, "缺少字段"):
            MujocoBackend._parse_manipulation_config(
                {
                    "gripper": {
                        "left_finger_body": "link7",
                        "right_finger_body": "link8",
                    }
                }
            )

    def test_rejects_non_positive_pose_tolerance(self):
        with self.assertRaisesRegex(ValueError, "正有限数"):
            MujocoBackend._parse_manipulation_config(
                {"targets": {"box_01": {"body": "box_01", "pose_tolerance_m": 0}}}
            )

    # --- 夹持区（grip region）：让运行时门禁与 IK 用同一个"抓取点"定义 ---

    def test_defaults_grip_region_to_none(self):
        """未声明 pad_boxes 时保持既有行为（门禁回退左右代表接触面）。"""
        parsed = MujocoBackend._parse_manipulation_config(
            {"gripper": _gripper()}
        )
        self.assertNotIn("pad_boxes", parsed["gripper"])

    def test_parses_grip_region_list(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {
                "gripper": _gripper(
                    pad_boxes=["left_pad1", "left_pad2", "right_pad1", "right_pad2"]
                )
            }
        )
        self.assertEqual(
            ["left_pad1", "left_pad2", "right_pad1", "right_pad2"],
            parsed["gripper"]["pad_boxes"],
        )

    def test_rejects_empty_or_malformed_grip_region(self):
        for bad in ([], "left_pad1", [""], ["a", "a"], [1, 2]):
            with self.subTest(value=bad):
                with self.assertRaisesRegex(ValueError, "pad_boxes"):
                    MujocoBackend._parse_manipulation_config(
                        {"gripper": _gripper(pad_boxes=bad)}
                    )

    # --- 伺服前馈（gravity_feedforward）：逐段 ctrl 增量 ---

    def test_defaults_gravity_feedforward_to_empty(self):
        """未声明前馈 = 零前馈：真机或已做重力补偿的模型行为不变。"""
        parsed = MujocoBackend._parse_manipulation_config(
            {"gripper": _gripper()}
        )
        self.assertEqual({}, parsed["gripper"]["gravity_feedforward"])

    def test_parses_per_phase_feedforward(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {
                "gripper": _gripper(
                    gravity_feedforward={
                        "home": {"joint7": 0.0, "joint8": -0.014},
                        "grasp": {"joint8": -0.017},
                    }
                )
            }
        )
        self.assertEqual(
            {"joint7": 0.0, "joint8": -0.014},
            parsed["gripper"]["gravity_feedforward"]["home"],
        )
        self.assertNotIn("lift", parsed["gripper"]["gravity_feedforward"])

    def test_rejects_unknown_phase(self):
        with self.assertRaisesRegex(ValueError, "未知段名"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": _gripper(gravity_feedforward={"home_pose": {"joint7": 0.0}})}
            )

    def test_rejects_feedforward_channel_not_in_positions(self):
        """前馈通道必须来自该段的位置指令：错位会让前馈静默失效。"""
        with self.assertRaisesRegex(ValueError, "未在该段位置指令中声明"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": _gripper(gravity_feedforward={"home": {"joint9": 0.01}})}
            )

    def test_rejects_feedforward_without_positions(self):
        gripper = _gripper()
        gripper.pop("home_positions")
        gripper["gravity_feedforward"] = {"home": {"joint7": 0.01}}
        with self.assertRaisesRegex(ValueError, "缺少对应的"):
            MujocoBackend._parse_manipulation_config({"gripper": gripper})

    def test_rejects_feedforward_out_of_range_and_non_finite(self):
        for bad in (0.5, float("nan"), float("inf")):
            with self.subTest(value=bad):
                with self.assertRaisesRegex(ValueError, "gravity_feedforward"):
                    MujocoBackend._parse_manipulation_config(
                        {"gripper": _gripper(gravity_feedforward={"home": {"joint7": bad}})}
                    )


if __name__ == "__main__":
    unittest.main()
