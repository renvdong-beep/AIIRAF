"""`dock_for_handoff` 的纯函数层：位姿误差与接近指令（不碰模型/锁/时钟）。

要证明的三件事：
1. **误差口径正确**：平移取世界系差、偏航解卷绕到 (-π, π]（跨 ±π 不得给出 2π 级假误差）；
2. **到位即零**：同时满足平移与偏航容差时返回**精确** `(0.0, 0.0, 0.0)`（"停下"是可判定的）；
3. **不含隐含常数**：世界→机身按机身偏航旋转、偏航未到位按 `max(0, cos(yaw_error))` 抑制平移、
   双向限幅只允许收紧；缺参/非有限/非正上限一律显式失败（本层**没有**任何默认值）。
"""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.unitree.dock import (  # noqa: E402
    DockDeclarationError,
    approach_command,
    pose_error,
)


def _cmd(dx, dy, yaw_error, body_yaw=0.0, **overrides):
    params = dict(gain_s_inv=1.0, max_speed_mps=0.2, max_yaw_rate_rad_s=0.5,
                  position_tolerance_m=0.03, yaw_tolerance_rad=math.radians(2.0),
                  body_yaw_rad=body_yaw)
    params.update(overrides)
    return approach_command(dx, dy, yaw_error, **params)


class PoseErrorTest(unittest.TestCase):
    def test_translation_is_world_frame_difference(self):
        dx, dy, yaw = pose_error((1.0, -2.0), 0.0, (0.4, 0.5), 0.0)
        self.assertAlmostEqual(dx, 0.6, places=15)
        self.assertAlmostEqual(dy, -2.5, places=15)
        self.assertAlmostEqual(yaw, 0.0, places=15)

    def test_yaw_error_is_unwrapped(self):
        """跨 ±π 必须解卷绕：目标 −179°、机身 +179° ⇒ 误差 +2°（不是 −358°）。"""
        target = math.radians(-179.0)
        body = math.radians(179.0)
        _dx, _dy, yaw = pose_error((0.0, 0.0), target, (0.0, 0.0), body)
        self.assertAlmostEqual(yaw, math.radians(2.0), places=12)
        # 反向同理
        _dx, _dy, yaw = pose_error((0.0, 0.0), math.radians(179.0), (0.0, 0.0),
                                   math.radians(-179.0))
        self.assertAlmostEqual(yaw, math.radians(-2.0), places=12)

    def test_non_finite_fails(self):
        for args in (((float("nan"), 0.0), 0.0, (0.0, 0.0), 0.0),
                     ((0.0, 0.0), float("inf"), (0.0, 0.0), 0.0),
                     ((0.0, 0.0), 0.0, (0.0, 0.0), float("nan"))):
            with self.assertRaises(DockDeclarationError):
                pose_error(*args)


class ApproachCommandTest(unittest.TestCase):
    def test_at_target_returns_exact_zero(self):
        cmd = _cmd(0.01, -0.02, math.radians(1.0))
        self.assertEqual(cmd, (0.0, 0.0, 0.0))

    def test_position_within_tolerance_but_yaw_out_is_not_zero(self):
        """平移已进容差但偏航没进 ⇒ **不得**返回零（"到位"必须两个条件同时满足）。"""
        cmd = _cmd(0.01, 0.0, math.radians(30.0))
        self.assertNotEqual(cmd, (0.0, 0.0, 0.0))
        self.assertGreater(abs(cmd[2]), 0.0)
        # 且平移分量按几何投影被抑制为 gain·dx·cos(yaw_error)（不是 0，也不是全量 0.01）
        self.assertAlmostEqual(cmd[0], 0.01 * math.cos(math.radians(30.0)), places=12)

    def test_world_error_is_rotated_into_body_frame(self):
        """机身偏航 90°、目标在世界 +x ⇒ 目标在机身**右侧** ⇒ vy 为负、vx≈0。"""
        cmd = _cmd(1.0, 0.0, 0.0, body_yaw=math.pi / 2.0)
        self.assertAlmostEqual(cmd[0], 0.0, places=12)
        self.assertLess(cmd[1], 0.0)

    def test_yaw_error_suppresses_translation_geometrically(self):
        """偏航差 90° ⇒ cos=0 ⇒ 平移分量为零（先转再走，不用经验常数）。"""
        cmd = _cmd(1.0, 0.0, math.radians(90.0))
        self.assertAlmostEqual(cmd[0], 0.0, places=12)
        self.assertAlmostEqual(cmd[1], 0.0, places=12)
        self.assertGreater(abs(cmd[2]), 0.0)

    def test_clamping_only_tightens(self):
        cmd = _cmd(100.0, 0.0, 0.0)
        self.assertAlmostEqual(math.hypot(cmd[0], cmd[1]), 0.2, places=12)
        cmd = _cmd(0.0, 0.0, math.radians(170.0))
        self.assertAlmostEqual(abs(cmd[2]), 0.5, places=12)

    def test_missing_or_bad_parameters_fail_explicitly(self):
        with self.assertRaises(TypeError):
            approach_command(0.1, 0.0, 0.0)          # 缺声明值 ⇒ Python 层直接拒绝（本层没有默认值）
        for bad in (0.0, -1.0, float("nan")):
            with self.assertRaises(DockDeclarationError):
                _cmd(0.1, 0.0, 0.0, gain_s_inv=bad)
            with self.assertRaises(DockDeclarationError):
                _cmd(0.1, 0.0, 0.0, max_speed_mps=bad)


if __name__ == "__main__":
    unittest.main()
