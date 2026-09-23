"""整机质量的**子树口径**：挂载物计入、台面游离道具不计入。

要证明的三件事（对应实测踩点：`sum(model.body_mass)` 把 `box_01` 算进机器人质量，
⇒ mg 偏大 0.392400 N / 0.2571%，而这条质量进 balance 路径的重力前馈）：
1. 子树内 body 全部计入 —— 包括**焊接在躯干上的挂载物**（本场景托盘 `tray_01` 0.35 kg）；
2. 子树外的 body **一律不计** —— 台面上带自由关节的道具（`box_01` 0.04 kg）不属于机器人；
3. 与旧口径的差 = 子树外有质量的 body 之和（可逐项对账，不是"看起来差不多"）。
"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.unitree.mpc import state_bridge  # noqa: E402


class _FakeModel:
    """只带 `trunk_subtree_bodies` / `robot_subtree_mass_kg` 需要的字段（纯单测，不建模型）。

    结构（仿场景模型：Go2 子树 + 挂载托盘 + 台面自由道具）：
        id 0 world      parent -1   mass 0
        id 1 base_link  parent  0   mass 10.0     ← 机器人根（躯干）
        id 2 tray_01    parent  1   mass  0.35    ← 焊在躯干上：**应计入**
        id 3 FL_hip     parent  1   mass  1.0
        id 4 FL_calf    parent  3   mass  0.5
        id 5 box_01     parent  0   mass  0.04    ← 台面自由道具：**不应计入**
    """

    def __init__(self):
        self.nbody = 6
        self.body_parentid = [-1, 0, 1, 1, 3, 0]
        self.body_mass = [0.0, 10.0, 0.35, 1.0, 0.5, 0.04]


class RobotSubtreeMassTest(unittest.TestCase):
    def setUp(self):
        self.model = _FakeModel()

    def test_subtree_includes_root_mounted_and_descendants(self):
        # 10.0（躯干）+ 0.35（挂载托盘）+ 1.0（hip）+ 0.5（calf）= 11.85
        self.assertAlmostEqual(state_bridge.robot_subtree_mass_kg(self.model, 1), 11.85, places=12)

    def test_free_prop_is_excluded(self):
        total = sum(self.model.body_mass)
        subtree = state_bridge.robot_subtree_mass_kg(self.model, 1)
        self.assertAlmostEqual(total - subtree, 0.04, places=12)      # 只差 box_01
        self.assertAlmostEqual(total, 11.89, places=12)

    def test_mounted_payload_counts(self):
        # 托盘（id 2）在子树内 ⇒ 计入；把它挪到 world 下（自由关节道具）⇒ 立刻不计
        with_tray = state_bridge.robot_subtree_mass_kg(self.model, 1)
        self.model.body_parentid[2] = 0
        without_tray = state_bridge.robot_subtree_mass_kg(self.model, 1)
        self.assertAlmostEqual(with_tray - without_tray, 0.35, places=12)

    def test_root_itself_counts(self):
        self.assertAlmostEqual(state_bridge.robot_subtree_mass_kg(self.model, 1), 11.85, places=12)
        # 以 hip 为根：只剩 hip + calf
        self.assertAlmostEqual(state_bridge.robot_subtree_mass_kg(self.model, 3), 1.5, places=12)

    def test_subtree_ids_are_int_coerced(self):
        # 模型给的是 int；若上游传字符串 id，子树遍历必须仍成立（实测踩点：类型不一致会漏判）
        self.assertAlmostEqual(state_bridge.robot_subtree_mass_kg(self.model, "1"), 11.85, places=12)


if __name__ == "__main__":
    unittest.main()
