# -*- coding: utf-8 -*-
"""给平台看的"下一步"文本（纯逻辑，不依赖 mcp）。

背景（2026-10-21/22 实机）：AgentOS 在抓取失败后读了我们十几个源文件、还改了它自己的 skill，
却始终没调 iraf_submit_task —— 它不知道"失败该怎么做"。这些文本随工具返回一起给出，
把"怎么核验 / 什么情况是环境问题 / 什么时候该停手"讲清楚。
"""
import unittest

from iraf_adapters.mcp.hints import FILTERED_EXAMPLE, WORKED_EXAMPLE, next_steps


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


if __name__ == "__main__":
    unittest.main()
