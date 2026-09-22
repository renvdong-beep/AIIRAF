"""落足点规划契约测试（专项 C / 步骤 01）：`gait.foothold` 的声明校验与模式门禁。

覆盖三类：
1. **正例**：生产声明（`mode=static`，落点不变 = 现行行为）通过；`per_phase` 合法声明通过，
   且规范化结果给出 stride / 环形顺序 / 单位化方向；`per_phase` + `sway.amplitude_m=0` 通过
   （证明互斥门禁不是恒真门禁）。
2. **负例**：缺 `foothold`（wave 必需）、`mode` 缺/非法、`static` 带无定义键、`per_phase` 缺必需键、
   `per_phase` 未知键、`stride_m` 为 0/负、`phase_direction_map` 腿集合不符/零矢量、
   `phase_order` 与相位升序不符/缺腿/重复/含未声明腿、`trot` + `per_phase`、
   `per_phase` + `sway.amplitude_m > 0`（互斥：两个水平位移源叠加后失稳无法归因）。
3. **边界 / 回归**：`trot` + `static` 通过（既有 trot 路线不被新段破坏）。

术语：`static` = 落点恒为中立足端（现行「原地踏步」）；`per_phase` = 逐相位把摆动腿的落点
挪到计划位置（迈步式 crawl 的落地点）。两者互斥于 `sway`（平移机身），见 ADR-0008 决策 1j。
"""

import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import yaml  # noqa: E402

from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree.quadruped import DeclarationError  # noqa: E402

CONFIG = ROOT / "config/go2_loopback.yaml"
PROFILE = ROOT / "profiles/unitree_go2_mujoco.yaml"

#: 相位偏移升序（实测声明：FL 0.0 / FR 0.25 / RR 0.5 / RL 0.75）⇒ 环形落点顺序。
EXPECTED_PHASE_ORDER = ["FL", "FR", "RR", "RL"]

#: 契约测试用的落点方向（**仅供本文件**：契约层只校验「非零 + 单位化 + 腿集合」，
#: 真实方向要由落点规划步骤按实测足迹给出，不得把测试值当声明值使用）。
TEST_DIRECTIONS = {
    "FL": {"x": 0.0, "y": -1.0},
    "FR": {"x": 0.0, "y": 1.0},
    "RR": {"x": 0.0, "y": -1.0},
    "RL": {"x": 0.0, "y": 1.0},
}


def _joints():
    spec = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["spec"]
    return [str(item) for item in spec["joints"]]


def _document():
    return copy.deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))


def _per_phase_document(sway_amplitude_m=0.0):
    """把生产声明改成 per_phase 语义的合法副本（sway 幅度置 0 = 两者互斥门禁的合法侧）。"""
    document = _document()
    document["gait"]["sway"]["amplitude_m"] = sway_amplitude_m
    document["gait"]["foothold"] = {
        "mode": "per_phase",
        "stride_m": 0.08,
        "phase_direction_map": copy.deepcopy(TEST_DIRECTIONS),
        "phase_order": list(EXPECTED_PHASE_ORDER),
        "ramp_s": 0.0,
        "smooth_s": 0.2,
    }
    return document


class LoadFootholdStaticTest(unittest.TestCase):
    """生产声明：`static` = 落点不变（现行为），且不接受任何水平落点键。"""

    def test_production_declaration_is_static(self):
        params = gait.load_gait_declaration(_document(), _joints())
        self.assertEqual({"mode": "static"}, params["foothold"])
        # static 模式下不得出现落点参数（否则是「假声明」：声明了不生效的值）。
        self.assertNotIn("stride_m", params["foothold"])
        self.assertNotIn("directions", params["foothold"])

    def test_missing_section_fails_for_wave(self):
        document = _document()
        document["gait"].pop("foothold")
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        message = str(ctx.exception)
        self.assertIn("gait.foothold", message)
        self.assertIn("-0.000227483", message)

    def test_section_must_be_mapping(self):
        document = _document()
        document["gait"]["foothold"] = ["static"]
        with self.assertRaises(DeclarationError):
            gait.load_gait_declaration(document, _joints())

    def test_mode_is_required(self):
        document = _document()
        document["gait"]["foothold"] = {}
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("缺少必需键", str(ctx.exception))

    def test_unknown_mode_fails(self):
        document = _document()
        document["gait"]["foothold"] = {"mode": "crawl"}
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("mode", str(ctx.exception))

    def test_static_rejects_undefined_keys(self):
        for key, value in (
            ("stride_m", 0.08),
            ("phase_direction_map", copy.deepcopy(TEST_DIRECTIONS)),
            ("phase_order", list(EXPECTED_PHASE_ORDER)),
            ("smooth_s", 0.2),
        ):
            with self.subTest(key=key):
                document = _document()
                document["gait"]["foothold"] = {"mode": "static", key: value}
                with self.assertRaises(DeclarationError) as ctx:
                    gait.load_gait_declaration(document, _joints())
                self.assertIn("无定义的键", str(ctx.exception))


class LoadFootholdPerPhaseTest(unittest.TestCase):
    """`per_phase`：逐相位落点规划（迈步式 crawl 的落地点）。"""

    def test_valid_per_phase_declaration(self):
        params = gait.load_gait_declaration(_per_phase_document(), _joints())
        foothold = params["foothold"]
        self.assertEqual("per_phase", foothold["mode"])
        self.assertAlmostEqual(0.08, foothold["stride_m"], places=12)
        self.assertAlmostEqual(0.0, foothold["ramp_s"], places=12)
        self.assertAlmostEqual(0.2, foothold["smooth_s"], places=12)
        self.assertEqual(EXPECTED_PHASE_ORDER, foothold["phase_order"])
        self.assertEqual(sorted(EXPECTED_PHASE_ORDER), sorted(foothold["directions"]))
        for code, vector in foothold["directions"].items():
            norm = (vector[0] ** 2 + vector[1] ** 2) ** 0.5
            self.assertAlmostEqual(1.0, norm, places=12, msg=code)

    def test_directions_are_normalised(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_direction_map"]["FL"] = {"x": 0.0, "y": -7.5}
        params = gait.load_gait_declaration(document, _joints())
        # 非单位向量被归一化：**方向不变**（数值 7.5 倍的 y 分量仍指向 -y）。
        self.assertAlmostEqual(0.0, params["foothold"]["directions"]["FL"][0], places=12)
        self.assertAlmostEqual(-1.0, params["foothold"]["directions"]["FL"][1], places=12)

    def test_sway_zero_is_allowed(self):
        """互斥门禁的合法侧：sway 段保留但幅度为 0 ⇒ 不是恒真门禁。"""
        params = gait.load_gait_declaration(_per_phase_document(sway_amplitude_m=0.0), _joints())
        self.assertIsNotNone(params["sway"])
        self.assertAlmostEqual(0.0, params["sway"]["amplitude_m"], places=12)
        self.assertEqual("per_phase", params["foothold"]["mode"])

    def test_sway_amplitude_must_be_zero(self):
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(_per_phase_document(sway_amplitude_m=0.06), _joints())
        self.assertIn("互斥", str(ctx.exception))

    def test_missing_required_keys(self):
        for key in gait.FOOTHOLD_MODE_KEYS["per_phase"]:
            with self.subTest(key=key):
                document = _per_phase_document()
                document["gait"]["foothold"].pop(key)
                with self.assertRaises(DeclarationError) as ctx:
                    gait.load_gait_declaration(document, _joints())
                self.assertIn(key, str(ctx.exception))

    def test_unknown_key_fails(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["stride_x_m"] = 0.08
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("无定义的键", str(ctx.exception))

    def test_stride_must_be_positive(self):
        for value in (0.0, -0.08):
            with self.subTest(value=value):
                document = _per_phase_document()
                document["gait"]["foothold"]["stride_m"] = value
                with self.assertRaises(DeclarationError):
                    gait.load_gait_declaration(document, _joints())

    def test_direction_map_legs_must_match(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_direction_map"].pop("RR")
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("腿集合", str(ctx.exception))

    def test_zero_vector_direction_fails(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_direction_map"]["FL"] = {"x": 0.0, "y": 0.0}
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("零矢量", str(ctx.exception))

    def test_phase_order_must_follow_phase_offsets(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_order"] = ["FL", "RR", "FR", "RL"]
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("相位偏移升序", str(ctx.exception))

    def test_phase_order_must_cover_each_leg_once(self):
        for order in (["FL", "FR", "RR"], ["FL", "FR", "RR", "RR"], ["FL", "FR", "RR", "XX"]):
            with self.subTest(order=order):
                document = _per_phase_document()
                document["gait"]["foothold"]["phase_order"] = order
                with self.assertRaises(DeclarationError):
                    gait.load_gait_declaration(document, _joints())

    def test_phase_order_must_be_sequence(self):
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_order"] = "FLFRRRRL"
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("phase_order", str(ctx.exception))

    def test_per_phase_is_wave_only(self):
        document = _per_phase_document()
        document["gait"]["kind"] = "trot"
        document["gait"]["duty_factor"] = 0.5
        document["gait"].pop("sway")
        for code, offset in (("FL", 0.0), ("RR", 0.0), ("FR", 0.5), ("RL", 0.5)):
            document["gait"]["legs"][code]["phase_offset"] = offset
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("per_phase", str(ctx.exception))


class NarrowFootholdEntryTest(unittest.TestCase):
    """`load_foothold_declaration`（门禁 9 用的窄入口）：只消费 `gait` 段自身。"""

    def test_matches_full_parse_on_production_declaration(self):
        document = _document()
        full = gait.load_gait_declaration(document, _joints())["foothold"]
        narrow = gait.load_foothold_declaration(document)
        self.assertEqual(full, narrow)

    def test_matches_full_parse_on_per_phase(self):
        document = _per_phase_document()
        full = gait.load_gait_declaration(document, _joints())["foothold"]
        narrow = gait.load_foothold_declaration(document)
        self.assertEqual(full, narrow)

    def test_does_not_depend_on_joint_bindings(self):
        """关节名与 Profile/模型无关时，窄入口仍能校验落足点声明（门禁 9 不误报）。

        实测踩到：门禁 9 首版用完整解析 + 夹具 Profile 的关节清单 ⇒ 夹具（关节名与真实
        Go2 不同）被判整份声明非法、profile_check 由通过变失败。
        """
        document = _per_phase_document()
        for code, leg in document["gait"]["legs"].items():
            leg["hip_joint"] = "%s_bogus_hip" % code
        # 宽入口会因关节绑定失败…
        with self.assertRaises(DeclarationError):
            gait.load_gait_declaration(document, _joints())
        # …窄入口不受影响，仍给出规范化结果。
        narrow = gait.load_foothold_declaration(document) or {}
        self.assertEqual("per_phase", narrow.get("mode"))
        self.assertEqual(EXPECTED_PHASE_ORDER, narrow.get("phase_order"))

    def test_rejects_invalid_foothold_like_full_parse(self):
        document = _per_phase_document(sway_amplitude_m=0.06)
        with self.assertRaises(DeclarationError) as narrow_ctx:
            gait.load_foothold_declaration(document)
        with self.assertRaises(DeclarationError) as full_ctx:
            gait.load_gait_declaration(document, _joints())
        self.assertIn("互斥", str(narrow_ctx.exception))
        self.assertIn("互斥", str(full_ctx.exception))

    def test_missing_gait_section_fails(self):
        with self.assertRaises(DeclarationError):
            gait.load_foothold_declaration({"robot": {"id": "x"}})

    def test_legs_must_carry_phase_offset(self):
        document = _document()
        document["gait"]["legs"]["FL"].pop("phase_offset")
        with self.assertRaises(DeclarationError) as ctx:
            gait.load_foothold_declaration(document)
        self.assertIn("phase_offset", str(ctx.exception))


class TrotRegressionTest(unittest.TestCase):
    """既有 trot 路线（二期性能优化）不得被新段破坏。"""

    def test_trot_with_static_foothold(self):
        document = _document()
        document["gait"]["kind"] = "trot"
        document["gait"]["duty_factor"] = 0.5
        document["gait"].pop("sway")
        for code, offset in (("FL", 0.0), ("RR", 0.0), ("FR", 0.5), ("RL", 0.5)):
            document["gait"]["legs"][code]["phase_offset"] = offset
        params = gait.load_gait_declaration(document, _joints())
        self.assertEqual("trot", params["kind"])
        self.assertEqual({"mode": "static"}, params["foothold"])

    def test_trot_without_foothold_is_allowed(self):
        document = _document()
        document["gait"]["kind"] = "trot"
        document["gait"]["duty_factor"] = 0.5
        document["gait"].pop("sway")
        document["gait"].pop("foothold")
        for code, offset in (("FL", 0.0), ("RR", 0.0), ("FR", 0.5), ("RL", 0.5)):
            document["gait"]["legs"][code]["phase_offset"] = offset
        params = gait.load_gait_declaration(document, _joints())
        self.assertIsNone(params["foothold"])


if __name__ == "__main__":
    unittest.main()
