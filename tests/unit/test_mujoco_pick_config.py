import unittest

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend


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


if __name__ == "__main__":
    unittest.main()
