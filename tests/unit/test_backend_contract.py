"""装配期契约校验的测试。

覆盖的契约（AGENTS.md 铁律 1 契约先于实现、铁律 5 失败即显式）：
- Profile 声明的 capability 在 Backend 上缺失时，装配期立即失败；
- 声明了未登记的 capability 时不得静默放过；
- Backend 实现了运动能力但 Profile 未声明时，必须显式暴露配置漂移；
- 合法组合必须通过，且报告内容可审计。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from iraf_adapters.factory import (  # noqa: E402
    BackendContractError,
    verify_backend_contract,
)


class _Profile:
    """最小 Profile 替身：只暴露契约校验关心的字段。"""

    def __init__(self, capabilities, name="test_profile"):
        self.capabilities = frozenset(capabilities)
        self.name = name


class _CompleteBackend:
    @classmethod
    def from_config(cls, config, profile, authority):
        return cls()

    def move_joint(self, positions, duration_ms, lease):
        return None

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        return {}

    def stop(self, lease):
        return None

    def step(self, count=1):
        return {}

    def home_pose(self):
        return {}


class _MissingPickBackend:
    """缺少 pick_object：必须在装配期暴露，而不是运行到一半才 SkillRejected。"""

    @classmethod
    def from_config(cls, config, profile, authority):
        return cls()

    def move_joint(self, positions, duration_ms, lease):
        return None

    def stop(self, lease):
        return None


class _NoFromConfigBackend:
    """只有 from_config 缺失这一处不合规，其余能力齐备以免干扰断言目标。"""

    capabilities = ()

    def move_joint(self, positions, duration_ms, lease):
        return None

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        return {}

    def stop(self, lease):
        return None


class _MoveAndStopBackend:
    """只实现 move_joint 与 stop，用于验证"部分运动能力"声明可正常通过。"""

    @classmethod
    def from_config(cls, config, profile, authority):
        return cls()

    def move_joint(self, positions, duration_ms, lease):
        return None

    def stop(self, lease):
        return None


class BackendContractTests(unittest.TestCase):
    def test_complete_backend_passes(self):
        profile = _Profile(["move_joint", "pick_object", "stop", "step"])
        report = verify_backend_contract(_CompleteBackend, profile)
        self.assertTrue(report["passed"])
        self.assertEqual(
            ["move_joint", "pick_object", "step", "stop"],
            report["declared_capabilities"],
        )
        self.assertEqual(
            {
                "move_joint": "move_joint",
                "pick_object": "pick_object",
                "step": "step",
                "stop": "stop",
            },
            report["verified_methods"],
        )

    def test_missing_capability_fails_at_assembly(self):
        profile = _Profile(["move_joint", "pick_object", "stop"])
        with self.assertRaises(BackendContractError) as context:
            verify_backend_contract(_MissingPickBackend, profile)
        self.assertIn("pick_object", str(context.exception))

    def test_unknown_capability_is_not_silently_accepted(self):
        """声明了未登记的 capability，会导致"策略通过但无人实现"的悬空能力。"""
        profile = _Profile(["move_joint", "teleport"])
        with self.assertRaises(BackendContractError) as context:
            verify_backend_contract(_CompleteBackend, profile)
        self.assertIn("teleport", str(context.exception))
        self.assertIn("未登记", str(context.exception))

    def test_undeclared_motion_capability_is_recorded_not_fatal(self):
        """实现了但未声明的运动能力只记录、不拦装配。

        真实场景（本项目实测）：同一个 MuJoCo 后端类同时服务 Piper 与 UR5e，
        而 UR5e 尚未接入视觉 Provider，profile **诚实地**不声明 visual_pick。
        若这里强制"实现即必须声明"，就等于逼 profile 声明它做不到的能力 ——
        反而制造虚假声明。要防的是反向的悬空能力（声明了却没有实现）。
        """
        profile = _Profile([])
        report = verify_backend_contract(_CompleteBackend, profile)
        self.assertTrue(report["passed"])
        self.assertEqual(
            ["move_joint", "pick_object", "stop"],
            report["undeclared_motion_capabilities"],
        )
        # 非运动能力（step/home_pose）不进该字段：它们不驱动机械臂。
        self.assertNotIn("step", report["undeclared_motion_capabilities"])

    def test_non_motion_extra_methods_are_allowed(self):
        """home_pose 等非运动能力方法不需要在 capabilities 中声明。"""
        profile = _Profile(["move_joint", "stop"])
        report = verify_backend_contract(_MoveAndStopBackend, profile)
        self.assertTrue(report["passed"])
        self.assertEqual([], report["undeclared_motion_capabilities"])

    def test_missing_from_config_is_rejected(self):
        """from_config 缺失沿用既有的显式失败行为。"""
        profile = _Profile([])
        with self.assertRaises(ValueError) as context:
            verify_backend_contract(_NoFromConfigBackend, profile)
        self.assertIn("from_config", str(context.exception))

    def test_declared_capability_without_method_still_fails(self):
        """单向化的只是"反向"，"声明必须实现"这一方向不能放松。"""
        profile = _Profile(["move_joint", "visual_pick", "stop"])
        with self.assertRaises(BackendContractError) as context:
            verify_backend_contract(_MoveAndStopBackend, profile)
        self.assertIn("visual_pick", str(context.exception))

    def test_report_is_json_serialisable(self):
        import json

        profile = _Profile(["move_joint", "stop"])
        report = verify_backend_contract(_MoveAndStopBackend, profile)
        json.dumps(report)

    def test_declaring_subset_records_remaining_implemented_capabilities(self):
        """只声明部分已实现能力：装配通过，未声明的部分进入审计报告。"""
        profile = _Profile(["move_joint", "stop"])
        report = verify_backend_contract(_CompleteBackend, profile)
        self.assertTrue(report["passed"])
        self.assertEqual(["pick_object"], report["undeclared_motion_capabilities"])
        self.assertEqual(["move_joint", "stop"], report["declared_capabilities"])


if __name__ == "__main__":
    unittest.main()
