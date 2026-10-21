# -*- coding: utf-8 -*-
"""机器人开发模式状态机（纯逻辑，不依赖 mcp/grpc）。

口径（2026-10-21 使用者）：不改别人的容器/框架，只在 IRAF 自己的北向加一个显式模式——
用户说"开始机器人开发模式" → 平台调 iraf_robot_mode(enter) → 后续任务落在机器人任务域。
"""
import unittest

from iraf_adapters.mcp.mode import MODE_IDLE, MODE_ROBOT, RobotModeState


class FakeClock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class RobotModeTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.state = RobotModeState(clock=self.clock)

    def test_initial_state_is_idle(self):
        self.assertEqual(self.state.name(), MODE_IDLE)
        self.assertFalse(self.state.active())
        self.assertEqual(self.state.remaining_s(), 0)
        self.assertIn("iraf_robot_mode", self.state.snapshot()["hint"])

    def test_enter_then_active(self):
        self.assertEqual(self.state.enter(600), 600)
        self.assertTrue(self.state.active())
        self.assertEqual(self.state.name(), MODE_ROBOT)
        self.assertEqual(self.state.remaining_s(), 600)
        self.assertIn("机器人开发模式已开启", self.state.snapshot()["hint"])

    def test_expires_after_ttl(self):
        self.state.enter(60)
        self.clock.advance(59)
        self.assertTrue(self.state.active())
        self.clock.advance(2)
        self.assertFalse(self.state.active())
        self.assertEqual(self.state.name(), MODE_IDLE)

    def test_exit_clears(self):
        self.state.enter(600)
        self.assertTrue(self.state.exit())
        self.assertFalse(self.state.active())
        # 再次退出：was_active=False，不抛错
        self.assertFalse(self.state.exit())

    def test_rejects_non_positive_ttl(self):
        with self.assertRaises(ValueError):
            self.state.enter(0)
        with self.assertRaises(ValueError):
            self.state.enter(-5)

    def test_reenter_refreshes_ttl(self):
        self.state.enter(60)
        self.clock.advance(50)
        self.state.enter(120)
        self.assertEqual(self.state.remaining_s(), 120)

    def test_routing_policy_is_robot_first_not_blind_forward(self):
        """分派规则必须表达"优先交给 IRAF + 其余由平台自己处理"，不是无脑转发。"""
        policy = self.state.snapshot()["routing_policy"]
        self.assertIn("优先调用 iraf_submit_task", policy)
        self.assertIn("IRAF-NOT-ROBOT-TASK", policy)
        self.assertIn("平台自身", policy)


if __name__ == "__main__":
    unittest.main()
