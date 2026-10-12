# -*- coding: utf-8 -*-
"""抓取目标**从载体站位帧派生**的回归测试（P5 / A2，2026-10-12）。

锁住三件事（不必跑整链即可单独抓到破坏）：
1. 挂载接收体的 `nominal_pose_m`（= 站位 FK）+ 声明偏移 = 派生目标（同一事实单一来源）；
2. 派生是**确定性纯函数**（相同输入 ⇒ 相同输出，逐位可复现）；
3. 无法派生时**返回原因**（fail-closed 诊断），不静默给错值：
   - 无挂载接收体；多个挂载接收体（有歧义）；偏移向量形状非法。
"""
import unittest

import mujoco

from iraf_adapters.unitree.scene_builder import _derive_grasp_target_from_station

MODEL = """
<mujoco>
  <worldbody>
    <body name="base_link" pos="0 0 0.2">
      <freejoint name="base_free"/>
      <body name="tray_01" pos="0.1 0 0.1">
        <geom name="tray_01_geom" size="0.05"/>
      </body>
    </body>
    <body name="box_01" pos="0.5 0.5 0.025">
      <geom name="box_01_geom" size="0.025"/>
    </body>
  </worldbody>
</mujoco>
"""

# 站位帧放在 (0.3, 0.4, 0)；载体摆到该站位后，tray_01 的世界位姿 = base(0.3,0.4,0.2)+local(0.1,0,0.1)
INJECTIONS = {"world_frames": [{"id": "handoff_station_frame",
                               "pos_m": [0.3, 0.4, 0.0],
                               "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}]}

PLACE_TARGETS = {"targets": [
    {"id": "tray_01", "body": "tray_01", "mount": {"entity": "carrier", "frame": "f", "body": "tray_01"},
     "nominal_pose_m": [0.1, 0.1, 0.1]},
    {"id": "box_01", "body": "box_01", "mount": None, "nominal_pose_m": [0.5, 0.5, 0.025]},
]}


class GraspTargetDerivationTests(unittest.TestCase):
    def _model(self):
        return mujoco.MjModel.from_xml_string(MODEL)

    def test_derives_from_station_and_offset(self):
        """派生 = 挂载接收体在**站位**下的 FK + 声明偏移（确定性）。"""
        model = self._model()
        solver = {"target_derivation": "from_carrier_station",
                  "target_derivation_offset_m": [0.0, 0.0, 0.05]}
        target, reason = _derive_grasp_target_from_station(PLACE_TARGETS, model, INJECTIONS, solver)
        self.assertIsNone(reason)
        # 无 keyframe ⇒ base_height=0 ⇒ 载体摆到 (0.3, 0.4, 0)；
        # tray 世界位姿 = (0.3,0.4,0) + (0.1,0,0.1) = (0.4,0.4,0.1)；+0.05 z
        self.assertAlmostEqual(target[0], 0.4, places=9)
        self.assertAlmostEqual(target[1], 0.4, places=9)
        self.assertAlmostEqual(target[2], 0.15, places=9)

    def test_deterministic(self):
        """相同输入 ⇒ 相同输出（逐位可复现）。"""
        model = self._model()
        solver = {"target_derivation": "from_carrier_station"}
        a, _ = _derive_grasp_target_from_station(PLACE_TARGETS, model, INJECTIONS, solver)
        b, _ = _derive_grasp_target_from_station(PLACE_TARGETS, model, INJECTIONS, solver)
        self.assertEqual(a, b)

    def test_station_move_tracks(self):
        """站点帧移动 ⇒ 派生目标随之移动（同一事实，不需另改目标键）。"""
        model = self._model()
        solver = {"target_derivation": "from_carrier_station"}
        t0, _ = _derive_grasp_target_from_station(PLACE_TARGETS, model, INJECTIONS, solver)
        moved = {"world_frames": [{"id": "handoff_station_frame",
                                   "pos_m": [0.3, 0.25, 0.0],
                                   "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}]}
        t1, _ = _derive_grasp_target_from_station(PLACE_TARGETS, model, moved, solver)
        self.assertAlmostEqual(t1[1] - t0[1], -0.15, places=9)

    def test_no_mounted_target_reports_reason(self):
        """没有挂载接收体 ⇒ 返回原因（不静默给值）。"""
        model = self._model()
        only_box = {"targets": [PLACE_TARGETS["targets"][1]]}
        target, reason = _derive_grasp_target_from_station(
            only_box, model, INJECTIONS, {"target_derivation": "from_carrier_station"})
        self.assertIsNone(target)
        self.assertIn("挂载接收体", reason)

    def test_ambiguous_multiple_mounted_reports_reason(self):
        """多个挂载接收体 ⇒ 有歧义，返回原因（不猜第一个）。"""
        model = self._model()
        two = {"targets": PLACE_TARGETS["targets"] + [
            {"id": "tray_02", "body": "box_01", "mount": {"entity": "carrier", "frame": "f", "body": "box_01"},
             "nominal_pose_m": [0.5, 0.5, 0.025]}]}
        target, reason = _derive_grasp_target_from_station(
            two, model, INJECTIONS, {"target_derivation": "from_carrier_station"})
        self.assertIsNone(target)
        self.assertIn("歧义", reason)

    def test_bad_offset_shape_reports_reason(self):
        """偏移向量形状非法 ⇒ 返回原因。"""
        model = self._model()
        target, reason = _derive_grasp_target_from_station(
            PLACE_TARGETS, model, INJECTIONS,
            {"target_derivation": "from_carrier_station", "target_derivation_offset_m": [0.0, 0.0]})
        self.assertIsNone(target)
        self.assertIn("长度 3", reason)

    def test_missing_station_reports_reason(self):
        """找不到站位帧 ⇒ 返回原因。"""
        model = self._model()
        target, reason = _derive_grasp_target_from_station(
            PLACE_TARGETS, model, {"world_frames": []},
            {"target_derivation": "from_carrier_station"})
        self.assertIsNone(target)
        self.assertIn("站位帧", reason)


if __name__ == "__main__":
    unittest.main()
