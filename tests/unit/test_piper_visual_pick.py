"""为 visual_pick 技能与 pad_offset 配置解析补充单元测试。

保护本轮改动：
1. _parse_manipulation_config 正确解析 pad_offset_m / pad_offset_axis，默认 0 且向后兼容。
2. VisualPickProvider 只接受 Backend 返回的真实抓取结果，拒绝伪造成功。
"""

import math
import time
import unittest
from pathlib import Path

from iraf_adapters.mujoco.mujoco_backend import MujocoBackend
from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.store import SqliteExecutionStore


class PadOffsetConfigTests(unittest.TestCase):
    """pad_offset_m / pad_offset_axis 的解析与默认值。"""

    def _gripper(self, **extra):
        config = {
            "wrist_body": "link6",
            "left_finger_body": "link7",
            "right_finger_body": "link8",
            "left_finger_geom": "piper_left_finger",
            "right_finger_geom": "piper_right_finger",
            "open_positions": {"joint7": 0.035, "joint8": -0.035},
            "closed_positions": {"joint7": 0.023, "joint8": -0.023},
        }
        config.update(extra)
        return config

    def test_defaults_to_zero_and_up_axis(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {"gripper": self._gripper()}
        )
        gripper = parsed["gripper"]
        self.assertEqual(0.0, gripper["pad_offset_m"])
        self.assertEqual([0.0, 0.0, 1.0], gripper["pad_offset_axis"])

    def test_parses_explicit_offset_and_axis(self):
        parsed = MujocoBackend._parse_manipulation_config(
            {
                "gripper": self._gripper(
                    pad_offset_m=0.0298,
                    pad_offset_axis=[0.0, 0.0, 1.0],
                )
            }
        )
        gripper = parsed["gripper"]
        self.assertAlmostEqual(0.0298, gripper["pad_offset_m"], places=6)
        self.assertEqual([0.0, 0.0, 1.0], gripper["pad_offset_axis"])

    def test_rejects_negative_offset(self):
        with self.assertRaisesRegex(ValueError, "非负"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": self._gripper(pad_offset_m=-0.01)}
            )

    def test_rejects_zero_axis(self):
        with self.assertRaisesRegex(ValueError, "零向量"):
            MujocoBackend._parse_manipulation_config(
                {"gripper": self._gripper(pad_offset_axis=[0.0, 0.0, 0.0])}
            )


class VisualPickProviderTests(unittest.TestCase):
    """VisualPickProvider 必须把 Backend 的真实抓取结果转成可审计证据。"""

    class FakeBackend:
        def __init__(self, authority, *, result=None):
            self.authority = authority
            self.result = result
            self.calls = []

        def runtime_inventory(self):
            return {
                "safety": {"estop": False},
                "manipulation": {"target_visible": True},
                "mode": "simulation",
            }

        def visual_pick(self, inputs, lease):
            self.authority.validate(lease)
            self.calls.append(inputs)
            return self.result

    def setUp(self):
        self.authority = ControlAuthorityManager()
        self.profile = RobotProfile(
            "piper",
            "1.0.0",
            True,
            100,
            ("joint1",),
            {"joint1": (-1.0, 1.0)},
            frozenset({"visual_pick", "stop"}),
            "development",
            "profile-digest",
        )
        self.policy = SafetyPolicy(
            "lab",
            "1.0.0",
            "development",
            True,
            frozenset({"visual_pick", "stop"}),
            30000,
            "policy-digest",
        )
        self.context = AuthenticatedContext(
            "tester", frozenset({"task.submit"}), "local"
        )
        self.registry = SkillRegistry().load_directory(
            Path(__file__).resolve().parents[2] / "skills"
        )

    def _request(self):
        return {
            "request_id": "vpick-request",
            "correlation_id": "vpick-correlation",
            "idempotency_key": "vpick-key",
            "skill": "visual_pick",
            "skill_version_constraint": "1.0.0",
            "parameters": {
                "target_id": "box_01",
                "duration_ms": 1000,
                "vision_file": "build/calibration/piper-vision-target.json",
            },
            "deadline_unix_ms": int(time.time() * 1000) + 5000,
            "profile_name": "piper",
            "profile_version": "1.0.0",
            "profile_digest": "profile-digest",
            "safety_policy_name": "lab",
            "safety_policy_version": "1.0.0",
            "safety_policy_digest": "policy-digest",
        }

    def _runtime(self, backend):
        return SkillRuntime(
            self.profile,
            self.policy,
            backend,
            self.registry,
            self.authority,
            SqliteExecutionStore(":memory:"),
        )

    def test_verified_backend_result_succeeds(self):
        backend = self.FakeBackend(
            self.authority,
            result={
                "target_id": "box_01",
                "grasped": True,
                "confirmation": "contact",
                "evidence": {
                    "bilateral_contact": True,
                    "vision": {
                        "source": "renderer_segmentation",
                        "frame_id": "world",
                        "vision_world_position_m": [0.19, 0.0, 0.025],
                        "evidence_file": "build/calibration/piper-vision-target.json",
                    },
                },
            },
        )
        result = self._runtime(backend).execute(self._request(), self.context)
        self.assertEqual("SUCCEEDED", result["status"])
        self.assertEqual("common_visual_pick", result["provider"]["name"])
        self.assertTrue(result["result"]["evidence"]["bilateral_contact"])
        self.assertEqual(
            "renderer_segmentation",
            result["result"]["evidence"]["vision"]["source"],
        )
        self.assertEqual(1, len(backend.calls))

    def test_backend_without_grasp_fails_closed(self):
        backend = self.FakeBackend(
            self.authority,
            result={
                "target_id": "box_01",
                "grasped": False,
                "confirmation": "contact",
            },
        )
        result = self._runtime(backend).execute(self._request(), self.context)
        self.assertEqual("FAILED", result["status"])
        self.assertIn("未通过", result["reason"])

    def test_missing_profile_capability_is_rejected(self):
        backend = self.FakeBackend(self.authority, result={"grasped": True})
        profile = RobotProfile(
            "piper",
            "1.0.0",
            True,
            100,
            ("joint1",),
            {"joint1": (-1.0, 1.0)},
            frozenset({"stop"}),
            "development",
            "profile-digest",
        )
        runtime = SkillRuntime(
            profile,
            self.policy,
            backend,
            self.registry,
            self.authority,
            SqliteExecutionStore(":memory:"),
        )
        result = runtime.execute(self._request(), self.context)
        self.assertEqual("IRAF-SKILL-PROVIDER-UNAVAILABLE", result["error_code"])
        self.assertEqual([], backend.calls)


if __name__ == "__main__":
    unittest.main()
