"""搬运节拍的**声明化检查**单测（§11.23(43)）：成功路径 + 超限路径 + 缺声明路径。

为什么需要它：搬运段把载荷焊在 mocap anchor 上，而 anchor 每个**控制迭代**只跟随一次，植物 owner
一次可能推进很多步（实测均值 18.01 步/迭代）⇒ "载荷随指腹刚性搬运"这一前提此前是**隐式**的、
只靠软约束兜着。现在由 `carry_constraint.max_plant_steps_per_iteration` 声明上限，超限即响亮失败。
"""

import unittest

from iraf_adapters.mujoco.payload_facts import (accumulate_carry_cadence, check_carry_cadence)


class CarryCadenceTests(unittest.TestCase):
    def test_within_limit_accumulates_and_passes(self):
        sink = {"limit": 24}
        previous = 1000
        for delta in (18, 21, 3, 24, 1):
            previous += delta
            accumulate_carry_cadence(sink, previous - delta, previous)
        self.assertEqual(sink["max_plant_steps_per_iteration"], 24)
        self.assertEqual(sink["iterations"], 5)
        self.assertIsNone(check_carry_cadence(sink), "上限内不得失败")

    def test_over_limit_is_reported_with_numbers(self):
        sink = {"limit": 24}
        accumulate_carry_cadence(sink, 0, 40)          # 单次迭代前进 40 步 > 24
        failure = check_carry_cadence(sink)
        self.assertIsNotNone(failure)
        self.assertIn("40", failure)
        self.assertIn("24", failure)

    def test_missing_declared_limit_fails_explicitly(self):
        """缺声明不得静默通过（AGENTS.md：不给实现层默认值）。"""
        with self.assertRaises(ValueError):
            check_carry_cadence({"max_plant_steps_per_iteration": 3})

    def test_limit_zero_is_also_rejected(self):
        with self.assertRaises(ValueError):
            check_carry_cadence({"limit": 0, "max_plant_steps_per_iteration": 0})


if __name__ == "__main__":
    unittest.main()
