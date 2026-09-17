"""工程向导 skill 的契约测试。

重点锁定的不变量：
- 向导属于构建期工具：**绝不调用 backend 运动方法**（铁律 2 的核心约束）；
- 问卷校验失败即显式抛错，不做默认值兜底（铁律 5）；
- 既有文件拒绝覆写，且整体判定为失败（禁止静默覆盖）；
- 产出的 profile 能被既有 load_robot_profile 加载，且与问卷语义一致。
"""

import os
import json
import sys
import tempfile
import unittest
from pathlib import Path

# 从测试文件位置反推仓库根（tests/unit/x.py -> 仓库根），
# 避免硬编码开发者本机路径；CI 可用 IRAF_ROOT 覆盖。
ROOT = Path(os.environ.get("IRAF_ROOT") or Path(__file__).resolve().parents[2])
sys.path.insert(0, str(ROOT / "src"))

from iraf_core.profile import load_robot_profile  # noqa: E402
from iraf_skills.engineering.robot_adapter import (  # noqa: E402
    RobotAdapterWizardProvider,
    WizardValidationError,
    normalize_answers,
    render_profile_yaml,
)

BASE_ANSWERS = {
    "robot_id": "demo_arm",
    "preset": "generic",
    "joints": ["j1", "j2", "j3", "j4", "j5", "j6", "grip_l", "grip_r"],
    "joint_limits": {
        "j1": [-3.0, 3.0], "j2": [-3.0, 3.0], "j3": [-3.0, 3.0],
        "j4": [-3.0, 3.0], "j5": [-3.0, 3.0], "j6": [-3.0, 3.0],
        "grip_l": [0.0, 0.04], "grip_r": [-0.04, 0.0],
    },
    "arm_joints": ["j1", "j2", "j3", "j4", "j5", "j6"],
    "control_frequency_hz": 100,
    "wrist_body": "wrist_link",
    "finger_geoms": ["finger_l", "finger_r"],
    "gripper_drive_joints": ["grip_l", "grip_r"],
    "open_positions": {"grip_l": 0.04, "grip_r": -0.04},
    "closed_positions": {"grip_l": 0.005, "grip_r": -0.005},
}


class _MotionRecorderBackend:
    """记录任何运动调用；向导调用它即视为违规。"""

    def __init__(self):
        self.calls = []

    def move_joint(self, *args, **kwargs):
        self.calls.append("move_joint")
        raise AssertionError("向导不得调用运动方法")

    def pick_object(self, *args, **kwargs):
        self.calls.append("pick_object")
        raise AssertionError("向导不得调用运动方法")

    def visual_pick(self, *args, **kwargs):
        self.calls.append("visual_pick")
        raise AssertionError("向导不得调用运动方法")

    def stop(self, *args, **kwargs):
        self.calls.append("stop")
        raise AssertionError("向导不得调用运动方法")

    def step(self, *args, **kwargs):
        self.calls.append("step")
        raise AssertionError("向导不得调用运动方法")


class NormalizeTests(unittest.TestCase):
    def test_valid_answers_normalized(self):
        config = normalize_answers(BASE_ANSWERS)
        self.assertEqual("demo_arm", config["robot_id"])
        self.assertEqual(["j1", "j2", "j3", "j4", "j5", "j6"], config["arm_joints"])
        self.assertEqual(["grip_l", "grip_r"], config["gripper"]["drive_joints"])

    def test_missing_required_field_rejected(self):
        for key in ("robot_id", "preset", "joints", "joint_limits", "arm_joints"):
            answers = dict(BASE_ANSWERS)
            answers.pop(key)
            with self.assertRaises(WizardValidationError) as context:
                normalize_answers(answers)
            self.assertIn(key, str(context.exception))

    def test_unknown_preset_rejected(self):
        answers = dict(BASE_ANSWERS)
        answers["preset"] = "does_not_exist"
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_invalid_robot_id_rejected(self):
        answers = dict(BASE_ANSWERS)
        answers["robot_id"] = "9bad-name"
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_joint_limits_must_be_ordered(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["joint_limits"]["j1"] = [3.0, -3.0]
        with self.assertRaises(WizardValidationError) as context:
            normalize_answers(answers)
        self.assertIn("low", str(context.exception))

    def test_joint_limits_must_cover_all_joints(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["joint_limits"].pop("j6")
        with self.assertRaises(WizardValidationError) as context:
            normalize_answers(answers)
        self.assertIn("j6", str(context.exception))

    def test_joint_limits_must_not_have_extra_joints(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["joint_limits"]["ghost"] = [0.0, 1.0]
        with self.assertRaises(WizardValidationError) as context:
            normalize_answers(answers)
        self.assertIn("ghost", str(context.exception))

    def test_arm_joints_must_be_declared(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["arm_joints"] = ["j1", "nope"]
        with self.assertRaises(WizardValidationError) as context:
            normalize_answers(answers)
        self.assertIn("nope", str(context.exception))

    def test_gripper_must_have_exactly_two_drives(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["gripper_drive_joints"] = ["grip_l"]
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_gripper_drives_must_not_overlap_arm(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["gripper_drive_joints"] = ["j1", "j2"]
        with self.assertRaises(WizardValidationError) as context:
            normalize_answers(answers)
        self.assertIn("不得同时作为臂关节", str(context.exception))

    def test_open_positions_must_match_drives(self):
        answers = json.loads(json.dumps(BASE_ANSWERS))
        answers["open_positions"] = {"grip_l": 0.04}
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_non_positive_frequency_rejected(self):
        answers = dict(BASE_ANSWERS)
        answers["control_frequency_hz"] = 0
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_negative_pad_offset_rejected(self):
        answers = dict(BASE_ANSWERS)
        answers["pad_offset_m"] = -0.01
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_wrist_body_must_not_collide_with_joint_names(self):
        answers = dict(BASE_ANSWERS)
        answers["wrist_body"] = "j1"
        with self.assertRaises(WizardValidationError):
            normalize_answers(answers)

    def test_non_object_answers_rejected(self):
        with self.assertRaises(WizardValidationError):
            normalize_answers(["not", "a", "dict"])


class RenderTests(unittest.TestCase):
    def test_profile_yaml_loads_with_existing_loader(self):
        """生成的 profile 必须能通过既有加载器——这是"工件可用"的硬证据。"""
        config = normalize_answers(BASE_ANSWERS)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "demo_arm_profile.yaml"
            path.write_text(render_profile_yaml(config), encoding="utf-8")
            profile = load_robot_profile(path)
        self.assertEqual("demo_arm", profile.name)
        self.assertEqual(8, len(profile.joints))
        self.assertEqual(["grip_l", "grip_r"], profile.gripper["drive_joints"])
        self.assertEqual("arm", profile.joint_roles["j1"])
        self.assertEqual("gripper_drive_left", profile.joint_roles["grip_l"])
        self.assertEqual("gripper_drive_right", profile.joint_roles["grip_r"])

    def test_profile_omits_camera_when_not_answered(self):
        """未提供的可选项不写进文件，避免 null 被当成显式声明。"""
        config = normalize_answers(BASE_ANSWERS)
        text = render_profile_yaml(config)
        self.assertNotIn("camera:", text)
        self.assertNotIn("null", text)


class ProviderTests(unittest.TestCase):
    def test_wizard_never_calls_motion_methods(self):
        """铁律 2 的核心约束：工程向导不得驱动 backend。"""
        backend = _MotionRecorderBackend()
        provider = RobotAdapterWizardProvider(profile=None, backend=backend)
        with tempfile.TemporaryDirectory() as tmp:
            result = provider.execute(
                {"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None
            )
        self.assertEqual([], backend.calls)
        self.assertTrue(result["accepted"])

    def test_wizard_works_without_lease(self):
        """不申请运动租约：lease 为 None 也必须正常工作。"""
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with tempfile.TemporaryDirectory() as tmp:
            result = provider.execute(
                {"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None
            )
        self.assertTrue(result["accepted"])
        kinds = sorted(item["kind"] for item in result["artifacts"])
        self.assertEqual(
            ["adapter", "baseline", "profile", "validation_report"], kinds
        )

    def test_existing_files_are_not_overwritten(self):
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with tempfile.TemporaryDirectory() as tmp:
            provider.execute({"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None)
            second = provider.execute(
                {"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None
            )
        self.assertFalse(second["accepted"])
        self.assertEqual(4, len(second["blocked_overwrites"]))
        for item in second["blocked_overwrites"]:
            self.assertIn("拒绝静默覆盖", item["reason"])

    def test_allow_overwrite_is_explicit(self):
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with tempfile.TemporaryDirectory() as tmp:
            provider.execute({"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None)
            second = provider.execute(
                {"answers": BASE_ANSWERS, "output_dir": tmp, "allow_overwrite": True},
                lease=None,
            )
        self.assertTrue(second["accepted"])
        self.assertEqual([], second["blocked_overwrites"])

    def test_missing_answers_is_rejected(self):
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with self.assertRaises(WizardValidationError):
            provider.execute({}, lease=None)

    def test_report_lists_remaining_manual_work(self):
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with tempfile.TemporaryDirectory() as tmp:
            result = provider.execute(
                {"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None
            )
        work = result["report"]["remaining_manual_work"]
        self.assertGreaterEqual(len(work), 3)
        self.assertTrue(any("RobotBackend" in item for item in work))

    def test_adapter_skeleton_is_valid_python(self):
        provider = RobotAdapterWizardProvider(profile=None, backend=None)
        with tempfile.TemporaryDirectory() as tmp:
            provider.execute({"answers": BASE_ANSWERS, "output_dir": tmp}, lease=None)
            skeleton = Path(tmp) / "demo_arm_backend.py"
            compile(skeleton.read_text(encoding="utf-8"), str(skeleton), "exec")


if __name__ == "__main__":
    unittest.main()
