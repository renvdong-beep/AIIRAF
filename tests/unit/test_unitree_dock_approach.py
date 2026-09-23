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

from iraf_adapters.unitree import dock  # noqa: E402
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


class TestTargetIsWorldFixed(unittest.TestCase):
    """停靠目标帧必须世界固定（自指帧会让停靠**看起来完美**却什么都没做）。

    实测背景（build/iraf-a6a14/dock_frame_probe.py）：本场景 `tray_frame` 挂在四足躯干 body 上，
    ⇒ 偏航误差恒为 0.0（同一 xmat）、平移误差恒为帧本地偏置的 xy 投影（3.632386e-05 m）、
    第 0 拍即"在位"（`settled_at_s = 0.0`），接近过程从未执行。
    """

    def _call(self, body_id, robot_bodies):
        return dock.assert_target_is_world_fixed(
            frame_label="tray_frame", frame_kind="site", frame_body_id=body_id,
            frame_body_label="base_link", robot_body_ids=robot_bodies,
            robot_root_label="base_link",
        )

    def test_frame_on_robot_trunk_fails(self):
        # 实测的正是这一情形：tray_frame 的所属 body = 躯干（id 1）⇒ 必须显式失败
        with self.assertRaises(DockDeclarationError) as ctx:
            self._call(1, [1, 2, 3, 4])
        message = str(ctx.exception)
        self.assertIn("tray_frame", message)
        self.assertIn("刚性挂在机器人 base_link 上", message)
        # 报错必须给出可操作方向（换世界固定的交接站位），不能只说"非法"
        self.assertIn("世界固定", message)

    def test_frame_on_robot_leg_fails(self):
        # 子树成员（腿/足端）同样不行：任何与本体刚性相连的帧都是自指量
        with self.assertRaises(DockDeclarationError):
            self._call(7, [1, 2, 3, 7])

    def test_world_fixed_prop_passes(self):
        self.assertIsNone(self._call(20, [1, 2, 3, 4]))

    def test_free_prop_passes(self):
        # 有自由关节的场景物（box_01 之类）不是"世界固定"但也不是本体的自指量 ⇒ 由调用方
        # 另行决定语义；本门禁只负责挡住"挂在机器人自己身上"这一类。
        self.assertIsNone(self._call(21, [1, 2, 3, 4]))

    def test_empty_robot_subtree_still_accepts_world_body(self):
        self.assertIsNone(self._call(5, []))

    def test_body_ids_are_coerced_not_compared_as_strings(self):
        # 实测踩点：模型给的是 int、声明给的是字符串时若直接比对会漏判 ⇒ 统一强制成 int
        self.assertIsNone(self._call(1, ["11", "12"]))
        with self.assertRaises(DockDeclarationError):
            self._call("11", ["11", "12"])


if __name__ == "__main__":
    unittest.main()
