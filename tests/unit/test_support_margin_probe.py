"""`scripts/probe_quadruped_support_margin.py` 第 ⑤ 项的契约测试（战役 iraf-24h-2 步骤 02）。

为什么要有这个文件
------------------
探针新增的判定「常量重心平移不可能让四个单支撑相位同时余量为正」是一条**几何定理**，
不是实测巧合。定理类门禁有一个天然的自伤风险：**恒为真的门禁等于没有门禁**。
所以本文件必须同时钉住两件事：

1. **正例对照**：把「某一条腿所需的平移」施加回去，该腿的余量必须**真的**达到目标值
   ⇒ 证明这套余量计算既不是恒负、也不是恒败，门禁才有意义。
2. **定理本身**：合成矩形与**随机凸四边形**站姿上，常量平移下四相位的最优余量 ≤ 0，
   对角腿对的余量之和上确界 ≤ 0，且穷举判定报不可行。

另覆盖一条失败路径：退化站姿（两足重合）必须在计算前显式失败，而不是给出 NaN/0 的假结论。
本文件只依赖 numpy（纯几何），不加载任何 MJCF。
"""

import importlib.util
import random
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "probe_quadruped_support_margin.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("probe_quadruped_support_margin", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PROBE = _load_module()

LEGS = ["FL", "FR", "RL", "RR"]
TARGET_MARGIN_M = 0.01


def rectangle_feet(length=0.4, width=0.28):
    """合成矩形站姿：与实测 Go2 站姿同形（长 > 宽），不依赖任何模型文件。"""
    half_length, half_width = length / 2.0, width / 2.0
    return {
        "FL": np.array([+half_length, +half_width]),
        "FR": np.array([+half_length, -half_width]),
        "RL": np.array([-half_length, +half_width]),
        "RR": np.array([-half_length, -half_width]),
    }


class SupportMarginProbeTests(unittest.TestCase):
    def test_required_shift_reaches_target_for_every_leg(self):
        """正例对照：施加「所需平移」后该腿余量达到目标值 ⇒ 余量计算不是恒负/恒败。"""
        feet = rectangle_feet()
        com = np.array([0.0, 0.0])
        margins, normals = PROBE.margins_and_normals(feet, com, LEGS)
        shifts = PROBE.required_shifts(margins, normals, TARGET_MARGIN_M)
        for leg in LEGS:
            with self.subTest(leg=leg):
                self.assertAlmostEqual(
                    margins[leg], 0.0, delta=1e-9,
                    msg="矩形站姿、重心在中心时该腿余量应为 0（重心落在对角线上）")
                achieved = PROBE.margin_for_shift(feet, com, LEGS, leg, shifts[leg])
                self.assertAlmostEqual(
                    achieved, TARGET_MARGIN_M, delta=1e-9,
                    msg="施加所需平移后该腿余量必须达到目标值")

    def test_offset_com_is_feasible_but_centred_com_is_not(self):
        """定理核对：把重心移进某一支撑三角形内部可行，但**四相位同时**不可行。"""
        feet = rectangle_feet()
        centered = np.array([0.0, 0.0])
        shifted = np.array([-0.04, -0.04])  # 朝对角腿 RR 一侧，即「抬 FL」支撑三角形的内部
        self.assertGreater(PROBE.margin_for_shift(feet, centered, LEGS, "FL", (0.0, 0.0)),
                           -1e-9)
        self.assertGreater(PROBE.margin_for_shift(feet, shifted, LEGS, "FL", (0.0, 0.0)),
                           TARGET_MARGIN_M)
        opposite = PROBE.margin_for_shift(feet, shifted, LEGS, "RR", (0.0, 0.0))
        self.assertLess(opposite, 0.0, "同一平移必然让对角相位（抬 RR）余量变负")

    def test_constant_shift_infeasible_on_rectangle(self):
        """穷举常量平移：矩形站姿（重心在中心）下四相位最优余量 ≤ 0 且 feasible=False。"""
        feet = rectangle_feet()
        com = np.array([0.0, 0.0])
        result = PROBE.constant_shift_feasibility(feet, com, LEGS, 0.05, 0.005)
        self.assertEqual(result["grid_points"], 21 * 21)
        self.assertLessEqual(result["best_min_margin_m"], 1e-12,
                             "矩形站姿下不存在让四相位余量同时为正的常量平移")
        self.assertFalse(result["best_min_margin_m"] > 0.0)

    def test_pair_sum_bound_is_non_positive_for_random_convex_stances(self):
        """定理核对：随机凸四边形站姿上，对角腿对的余量之和上确界恒 ≤ 0。"""
        rng = random.Random(20260921)
        base_angles = [0.0, 0.5 * np.pi, np.pi, 1.5 * np.pi]
        for trial in range(12):
            with self.subTest(trial=trial):
                points = []
                for base in base_angles:
                    angle = base + rng.uniform(-0.3, 0.3)
                    radius = rng.uniform(0.10, 0.30)
                    points.append(np.array([radius * np.cos(angle), radius * np.sin(angle)]))
                feet = {code: points[index] for index, code in enumerate(LEGS)}
                centroid = np.mean(points, axis=0)
                com = centroid + np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02)])
                for left_index, right_index in ((0, 2), (1, 3)):
                    pair = (LEGS[left_index], LEGS[right_index])
                    bound = PROBE.pair_sum_bound(feet, com, LEGS, pair, 0.05, 0.01)
                    self.assertLessEqual(
                        bound, 1e-12,
                        "凸站姿的对角腿对余量之和必须 ≤ 0（解析界），实测 %.12f" % bound)

    def test_degenerate_stance_is_rejected(self):
        """失败路径：两足重合构成退化边 ⇒ 必须显式失败，而不是给出假余量。"""
        feet = rectangle_feet()
        feet["RR"] = feet["FL"].copy()
        com = np.array([0.0, 0.0])
        with self.assertRaises(SystemExit):
            PROBE.margin_for_shift(feet, com, LEGS, "FR", (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
