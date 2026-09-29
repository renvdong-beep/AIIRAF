"""停靠**站位选择**：声明驱动的站名解析 + 技能层只接受站名（2026-09-29 双臂轮转演示）。

为什么要有这组用例：
  · 第二个站位（B 站，第二台臂 UR5e 侧）不能靠给技能塞坐标实现 —— 站名必须来自声明、
    调用方只能给名字（`skills/dock_for_handoff` 的原契约是"无参数、不得自带数字"）；
  · "同一事实不出现两处"：`target_frame` 与 `stations[default_station]` 必须一致，
    写歪了要在执行**前**失败（否则会走到一个没登记的帧上）；
  · 缺声明必须显式失败（不猜默认值）。
"""

import unittest

from iraf_adapters.unitree import dock
from iraf_skills.quadruped import DockForHandoffProvider, SkillContractError


class ResolveDockStationTests(unittest.TestCase):
    """纯函数层：站名 → 帧名。"""

    def test_without_registry_legacy_behaviour_is_unchanged(self):
        frame, name = dock.resolve_dock_station({"target_frame": "handoff_station_frame"})
        self.assertEqual(frame, "handoff_station_frame")
        self.assertIsNone(name)

    def test_without_registry_rejects_station_name(self):
        with self.assertRaises(dock.DockDeclarationError) as ctx:
            dock.resolve_dock_station({"target_frame": "handoff_station_frame"},
                                      station="handoff_b")
        self.assertIn("没有 stations 登记", str(ctx.exception))

    def test_default_station_is_selected_when_none_given(self):
        section = {
            "target_frame": "handoff_station_frame",
            "stations": {"handoff_a": "handoff_station_frame",
                         "handoff_b": "handoff_station_frame_b"},
            "default_station": "handoff_a",
        }
        frame, name = dock.resolve_dock_station(section)
        self.assertEqual((frame, name), ("handoff_station_frame", "handoff_a"))

    def test_named_station_resolves_to_its_frame(self):
        section = {
            "target_frame": "handoff_station_frame",
            "stations": {"handoff_a": "handoff_station_frame",
                         "handoff_b": "handoff_station_frame_b"},
            "default_station": "handoff_a",
        }
        frame, name = dock.resolve_dock_station(section, station="handoff_b")
        self.assertEqual((frame, name), ("handoff_station_frame_b", "handoff_b"))

    def test_unknown_station_lists_declared_names(self):
        section = {
            "target_frame": "handoff_station_frame",
            "stations": {"handoff_a": "handoff_station_frame"},
            "default_station": "handoff_a",
        }
        with self.assertRaises(dock.DockDeclarationError) as ctx:
            dock.resolve_dock_station(section, station="handoff_x")
        self.assertIn("未知站位", str(ctx.exception))
        self.assertIn("handoff_a", str(ctx.exception))

    def test_registry_requires_default_station(self):
        section = {"target_frame": "f", "stations": {"handoff_a": "f"}}
        with self.assertRaises(dock.DockDeclarationError) as ctx:
            dock.resolve_dock_station(section)
        self.assertIn("default_station", str(ctx.exception))

    def test_default_station_must_exist(self):
        section = {"target_frame": "f", "stations": {"handoff_a": "f"},
                   "default_station": "handoff_b"}
        with self.assertRaises(dock.DockDeclarationError) as ctx:
            dock.resolve_dock_station(section)
        self.assertIn("不在 stations", str(ctx.exception))

    def test_target_frame_must_match_default_station(self):
        """同一事实不出现两处：写歪 ⇒ 执行前显式失败。"""
        section = {"target_frame": "some_other_frame",
                   "stations": {"handoff_a": "handoff_station_frame"},
                   "default_station": "handoff_a"}
        with self.assertRaises(dock.DockDeclarationError) as ctx:
            dock.resolve_dock_station(section)
        self.assertIn("不一致", str(ctx.exception))

    def test_empty_registry_is_rejected(self):
        with self.assertRaises(dock.DockDeclarationError):
            dock.resolve_dock_station({"target_frame": "f", "stations": {},
                                       "default_station": "handoff_a"})


class _FakeBackend:
    """记录调用参数的假后端：只用来断言"技能层把站名透传下去了、且别的键一律拒绝"。"""

    EVIDENCE = {"simulation": True, "capability": "dock_for_handoff",
                "target_frame": "handoff_station_frame_b", "target_frame_world_fixed": True,
                "settled_at_s": 1.0, "final_translation_error_m": 0.001,
                "final_yaw_error_deg": 0.1, "final_speed_mps": 0.0001}

    def __init__(self):
        self.calls = []

    def dock_acceptance(self):
        return {"position_tolerance_m": 0.03, "yaw_tolerance_rad": 0.03490658503988659,
                "max_final_speed_mps": 0.05}

    def dock_for_handoff(self, **kwargs):
        self.calls.append(kwargs)
        return dict(self.EVIDENCE, failure=None)


class DockProviderStationInputTests(unittest.TestCase):
    """技能层：`station` 是唯一允许的输入键（仍是"不得自带数字"）。"""

    def setUp(self):
        self.backend = _FakeBackend()
        self.provider = DockForHandoffProvider(profile=None, backend=self.backend)

    def test_station_name_is_passed_through(self):
        result = self.provider.execute({"station": "handoff_b"}, lease=object())
        self.assertEqual(len(self.backend.calls), 1)
        self.assertEqual(self.backend.calls[0]["station"], "handoff_b")
        self.assertTrue(result["accepted"])
        self.assertEqual(result["evidence"]["target_frame"], "handoff_station_frame_b")

    def test_no_input_means_default_station(self):
        self.provider.execute({}, lease=object())
        self.assertIsNone(self.backend.calls[0]["station"])

    def test_other_keys_are_rejected(self):
        for bad in ({"target_xy": [0.45, 0.45]}, {"station": "handoff_b", "speed": 0.2},
                    {"translation_error_max_m": 0.5}):
            with self.subTest(bad=bad):
                with self.assertRaises(SkillContractError) as ctx:
                    self.provider.execute(bad, lease=object())
                self.assertIn("只接受站位名", str(ctx.exception))
        self.assertEqual(self.backend.calls, [], "被拒的输入不得触达后端")


if __name__ == "__main__":
    unittest.main()
