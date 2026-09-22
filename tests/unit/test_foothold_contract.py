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
import math
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

    def test_stride_must_not_be_negative_and_zero_is_isolation_control(self):
        """`stride_m = 0` 合法：它是**隔离对照**（per_phase 通路但零位移 ⇒ 应与 static 逐位一致），
        与 `sway.amplitude_m` 允许 0 的取向一致；负值仍然失败。"""
        document = _per_phase_document()
        document["gait"]["foothold"]["stride_m"] = 0.0
        params = gait.load_gait_declaration(document, _joints())
        self.assertAlmostEqual(0.0, params["foothold"]["stride_m"], places=12)
        document = _per_phase_document()
        document["gait"]["foothold"]["stride_m"] = -0.08
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


class FootholdTrajectoryTest(unittest.TestCase):
    """落点轨迹（步骤 02）：`static` 逐位一致；`per_phase` 的交替/过渡/斜坡/闭环。"""

    L1 = L2 = 0.213
    NEUTRAL_Z = -2.0 * 0.213 * math.cos(0.9)

    def _geometry(self):
        geometry = {}
        for index, code in enumerate(("FL", "FR", "RR", "RL")):
            geometry[code] = {
                "l1_m": self.L1,
                "l2_m": self.L2,
                "neutral_x_m": 0.1805,
                "neutral_z_m": self.NEUTRAL_Z,
                "trunk_rel_m": [0.1805, 0.047, self.NEUTRAL_Z],
                "joints": {
                    "hip_joint": code + "_hip_joint",
                    "thigh_joint": code + "_thigh_joint",
                    "calf_joint": code + "_calf_joint",
                },
            }
        return geometry

    def _home(self):
        names = []
        for code in ("FL", "FR", "RR", "RL"):
            names.extend([code + "_hip_joint", code + "_thigh_joint", code + "_calf_joint"])
        return {name: 0.0 for name in names}

    def _limits(self):
        return {name: (-3.0, 3.0) for name in self._home()}

    def _params(self, per_phase=True):
        document = _per_phase_document() if per_phase else _document()
        return gait.load_gait_declaration(document, _joints())

    def _time_for(self, params, code, cycle, u_local):
        """`cycle` 周期内**局部相位** `u_local` 对应的时刻（严格按 `leg_phase` 的相位约定）。

        为什么不让各用例自己拼时间：相位约定是 `phase = elapsed/period + offset`，
        自己拼 `(cycle + u + offset)` 会在 offset ≠ 0 的腿上算错半个周期 —— 首版落点实现
        就是在这个约定上写反了符号（实测见 build/iraf-24h-3/step03/reachability.txt）。
        """
        offset = params["legs"][code]["phase_offset"]
        return (cycle + u_local - offset) * params["period_s"]

    # ---- static：精确 0 与「无落足点规划」逐位一致 ----

    def test_static_offset_is_exact_zero(self):
        params = self._params(per_phase=False)
        period = params["period_s"]
        for code in ("FL", "FR", "RR", "RL"):
            for step in range(0, 40):
                value = gait.foothold_offset_m(params, code, step * period / 8.0)
                self.assertEqual((0.0, 0.0), value)

    def test_static_joint_targets_are_bit_identical_to_previous_formula(self):
        """逐位一致（不是近似）：`static` 下的目标角 == 改动前的表达式。"""
        params = self._params(per_phase=False)
        geometry = self._geometry()
        home = self._home()
        limits = self._limits()
        clearance = float(params["stance_clearance_m"])
        for code, geom in geometry.items():
            for step in range(0, 40):
                elapsed = step * params["period_s"] / 8.0
                # 改动前的表达式（本测试内独立复算）；sway 与相位相关 ⇒ 必须逐时刻取
                sway = gait.sway_offset_m(params, elapsed)
                phase = gait.leg_phase(params, code, elapsed)
                dx, dz = gait.foot_offset(phase, params, 1.0)
                damping = gait.stabilization_offset(params, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                                    geom["trunk_rel_m"])
                hip, thigh, calf = gait.leg_solve(
                    geom["neutral_x_m"] + dx + damping[0] - float(sway[0]),
                    damping[1] - float(sway[1]),
                    geom["neutral_z_m"] + clearance + dz,
                    geom["l1_m"],
                    geom["l2_m"],
                )
                expected = {
                    geom["joints"]["hip_joint"]: float(home[geom["joints"]["hip_joint"]]) + hip,
                    geom["joints"]["thigh_joint"]: thigh,
                    geom["joints"]["calf_joint"]: calf,
                }
                actual = gait.gait_joint_targets(params, geometry, home, limits, elapsed)
                for joint, value in expected.items():
                    self.assertEqual(
                        value, actual[joint],
                        "关节 %s 在 t=%r 不是逐位一致（%r vs %r）" % (joint, elapsed, value, actual[joint]),
                    )

    # ---- per_phase：交替 / 过渡 / 斜坡 / 闭环 ----

    def test_per_phase_alternates_between_cycles(self):
        params = self._params()
        stride = params["foothold"]["stride_m"]
        period = params["period_s"]
        duty = params["duty_factor"]
        for code in ("FL", "FR", "RR", "RL"):
            offset = params["legs"][code]["phase_offset"]
            # 第 1、2 个周期**支撑相中段**的落点：等级相反、模长相同
            samples = []
            for cycle in (1, 2):
                u = 0.5 * duty
                elapsed = self._time_for(params, code, cycle, u)
                samples.append(gait.foothold_offset_m(params, code, elapsed))
            first, second = samples
            # 落点等级逐周期交替 ⇒ 两个相邻周期的落点向量**互为相反数**
            for axis in (0, 1):
                self.assertAlmostEqual(-first[axis], second[axis], places=12,
                                       msg="%s 第 %d 轴（%r vs %r）" % (code, axis, first, second))
            norm_first = math.hypot(*first)
            norm_second = math.hypot(*second)
            self.assertAlmostEqual(stride, norm_first, places=12, msg=code)
            self.assertAlmostEqual(norm_first, norm_second, places=12, msg=code)

    def test_per_phase_is_continuous_at_cycle_boundary(self):
        """周期边界（支撑相起点）不得跳变：摆动结束落下的等级 = 下一周期支撑相保持的等级。"""
        params = self._params()
        period = params["period_s"]
        for code in ("FL", "FR", "RR", "RL"):
            offset = params["legs"][code]["phase_offset"]
            for cycle in (1, 2, 3):
                end_of_swing = self._time_for(params, code, cycle, 1.0 - 1.0e-9)
                start_of_stance = self._time_for(params, code, cycle + 1, 0.0)
                a = gait.foothold_offset_m(params, code, end_of_swing)
                b = gait.foothold_offset_m(params, code, start_of_stance)
                for axis in (0, 1):
                    self.assertAlmostEqual(a[axis], b[axis], places=6,
                                           msg="%s cycle %d" % (code, cycle))

    def test_per_phase_swing_transition_is_smooth(self):
        """摆动窗内落点从「上周期等级」单调过渡到「本周期等级」，两端与支撑相衔接。"""
        params = self._params()
        period = params["period_s"]
        duty = params["duty_factor"]
        stride = params["foothold"]["stride_m"]
        code = "FL"  # 方向 (0, −1) ⇒ y 分量承载落点等级
        offset = params["legs"][code]["phase_offset"]
        values = []
        for step in range(0, 21):
            u = duty + (1.0 - duty) * step / 20.0
            values.append(gait.foothold_offset_m(params, code, self._time_for(params, code, 1, u))[1])
        # 单调（相邻差**同号**，允许浮点噪声）——不是"近似不变"：余弦过渡的相邻差可达 0.0125
        deltas = [values[index + 1] - values[index] for index in range(len(values) - 1)]
        self.assertTrue(
            all(delta >= -1.0e-12 for delta in deltas) or all(delta <= 1.0e-12 for delta in deltas),
            "摆动窗内落点不单调: %r" % (deltas,),
        )
        # 两端与支撑相衔接：第 1 周期 sign = −1 ⇒ 从 +stride 等级落到 −stride 等级
        # （FL 的方向是 (0, −1)，故 y 分量的符号与等级相反）。
        self.assertAlmostEqual(-stride, values[0], places=12)
        self.assertAlmostEqual(stride, values[-1], places=12)

    def test_per_phase_ramp_scales_amplitude(self):
        """`ramp_s` 内幅度线性建立（`ramp_s = 0` ⇒ 首次抬腿前就满幅）。

        取 FL（`phase_offset = 0`，周期 0.8 s）：`t = 0.5 s` 时该腿处于第 0 周期的支撑相
        （u = 0.625 < duty 0.75）⇒ 落点等级已满幅，幅度只剩 ramp 的 0.5 倍。
        """
        document = _per_phase_document()
        document["gait"]["foothold"]["ramp_s"] = 1.0
        params = gait.load_gait_declaration(document, _joints())
        stride = params["foothold"]["stride_m"]
        # 取 FL 第 0 周期**支撑相**内 u_local = 0.625 的时刻（= 0.5 s）：幅度只由 ramp 决定，
        # `ramp_s = 1.0` ⇒ 该时刻建立到 0.5 倍满幅。
        ramped = gait.foothold_offset_m(params, "FL", self._time_for(params, "FL", 0, 0.625))
        full = gait.foothold_offset_m(params, "FL", self._time_for(params, "FL", 5, 0.0))
        self.assertAlmostEqual(stride, math.hypot(*full), places=12)
        self.assertAlmostEqual(0.5 * stride, math.hypot(*ramped), places=12)

    def test_per_phase_footprint_closes_every_two_cycles(self):
        """两周期后落点回到同一等级 ⇒ 足迹闭环、无累积漂移（"原地"语义）。"""
        params = self._params()
        period = params["period_s"]
        for code in ("FL", "FR", "RR", "RL"):
            offset = params["legs"][code]["phase_offset"]
            for u in (0.0, 0.25, 0.5, 0.9):
                a = gait.foothold_offset_m(params, code, self._time_for(params, code, 2, u))
                b = gait.foothold_offset_m(params, code, self._time_for(params, code, 4, u))
                for axis in (0, 1):
                    self.assertAlmostEqual(a[axis], b[axis], places=12, msg=code)

    def test_foothold_enters_the_joint_targets(self):
        """落点偏移确实进入目标角：x 分量改变大腿角，y 分量改变髋（外展）角。

        机制核对（不是"看起来合理"）：`leg_solve` 里 `q_hip = atan2(py, −pz)` ⇒ 侧向落点只
        由髋角承载；`px` 走矢状面 IK ⇒ 改变大腿/小腿角。两个轴各测一条，避免「改了参数但
        目标角没动」的静默失效。
        """
        home = self._home()
        limits = self._limits()
        geometry = self._geometry()
        # (a) x 方向落点 ⇒ 大腿角变化
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_direction_map"]["FL"] = {"x": 1.0, "y": 0.0}
        params = gait.load_gait_declaration(document, _joints())
        period = params["period_s"]
        u_stance = params["duty_factor"] * 0.5
        t_moved = self._time_for(params, "FL", 1, u_stance)
        base = gait.gait_joint_targets(params, geometry, home, limits, 0.0)
        moved = gait.gait_joint_targets(params, geometry, home, limits, t_moved)
        self.assertNotAlmostEqual(base["FL_thigh_joint"], moved["FL_thigh_joint"], places=9)
        # (b) y 方向落点 ⇒ 髋（外展）角变化
        document = _per_phase_document()
        document["gait"]["foothold"]["phase_direction_map"]["FL"] = {"x": 0.0, "y": 1.0}
        params = gait.load_gait_declaration(document, _joints())
        base = gait.gait_joint_targets(params, geometry, home, limits, 0.0)
        moved = gait.gait_joint_targets(
            params, geometry, home, limits, self._time_for(params, "FL", 1, u_stance)
        )
        self.assertNotAlmostEqual(base["FL_hip_joint"], moved["FL_hip_joint"], places=9)


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
