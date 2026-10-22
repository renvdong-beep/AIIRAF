# -*- coding: utf-8 -*-
"""给平台看的"下一步"文本（纯逻辑，不依赖 mcp）。

背景（2026-10-21/22 实机）：AgentOS 在抓取失败后读了我们十几个源文件、还改了它自己的 skill，
却始终没调 iraf_submit_task —— 它不知道"失败该怎么做"。这些文本随工具返回一起给出，
把"怎么核验 / 什么情况是环境问题 / 什么时候该停手"讲清楚。
"""
import unittest

from iraf_adapters.mcp.hints import (FILTERED_EXAMPLE, WORKED_EXAMPLE, enter_next_steps,
                                     next_steps)


class NextStepsTest(unittest.TestCase):
    def test_success_points_to_event_store(self):
        steps = next_steps({"status": "SUCCEEDED", "execution_id": "abc123"})
        self.assertTrue(any("iraf_get_task" in s and "abc123" in s for s in steps))
        self.assertTrue(any("场景" in s for s in steps))

    def test_distance_failure_is_environment_not_implementation(self):
        steps = next_steps({"status": "FAILED",
                            "error_code": "IRAF-EXECUTION-FAILED",
                            "reason": "末端未到达目标抓取位姿: distance=0.374435m tolerance=0.005000m"})
        joined = " ".join(steps)
        self.assertIn("场景状态", joined)
        self.assertIn("不要修改源码", joined)

    def test_not_robot_task_says_do_not_retry(self):
        steps = next_steps({"status": "FAILED", "error_code": "IRAF-NOT-ROBOT-TASK"})
        self.assertTrue(any("不要重试" in s for s in steps))

    def test_policy_denied_says_no_relax(self):
        steps = next_steps({"status": "FAILED", "error_code": "IRAF-POLICY-DENIED"})
        self.assertTrue(any("策略" in s for s in steps))

    def test_examples_are_self_consistent(self):
        self.assertEqual(WORKED_EXAMPLE["expected"]["status"], "SUCCEEDED")
        self.assertEqual(FILTERED_EXAMPLE["expected"]["error_code"], "IRAF-NOT-ROBOT-TASK")
        self.assertTrue(any("唯一受支持入口" in n for n in WORKED_EXAMPLE["notes"]))

    def test_worked_example_is_not_a_smoke_test(self):
        """示例是语义映射说明，不是验收脚本（2026-10-22 冒烟提交事故）。"""
        self.assertIn("不要求执行", WORKED_EXAMPLE["purpose"])
        self.assertTrue(any("不要为了" in n and "执行" in n for n in WORKED_EXAMPLE["notes"]))


class EnterNextStepsTest(unittest.TestCase):
    """进入模式后的固定顺序：先盘点，禁止拿真任务探路。"""

    def setUp(self):
        self.steps = enter_next_steps()

    def test_first_step_is_report_catalog_only(self):
        self.assertIn("skills", self.steps[0])
        self.assertIn("盘点", self.steps[0])

    def test_forbids_smoke_submit(self):
        joined = " ".join(self.steps)
        self.assertIn("不要为了验证链路而提交任务", joined)

    def test_requires_verify_with_event_store(self):
        self.assertTrue(any("iraf_get_task" in s for s in self.steps))

    def test_warns_scene_is_stateful(self):
        joined = " ".join(self.steps)
        self.assertIn("0.374435", joined)
        self.assertIn("复位", joined)

    def test_returns_copy(self):
        self.steps.append("污染")
        self.assertNotIn("污染", enter_next_steps())


if __name__ == "__main__":
    unittest.main()
