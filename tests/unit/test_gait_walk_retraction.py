"""支撑足退让契约测试（A6a-④「能走路」这一步）：`gait.walk_foot_offset_m`。

要证明的三件事（每一件都对应一个已实测的坑）：

1. **零影响**：`gait.walk` 未声明 / `enabled: false` / 零指令 ⇒ 偏移精确为 `(0.0, 0.0)`，
   且 `gait_joint_targets` 的输出与「不加该项」**逐位相同**（既有「原地踏步」路径不许被污染）。
2. **退让方向与量值**：机身要以 `v` 前进 ⇒ 支撑足在机身系里必须以 `−v` 退让（世界系钉住足端），
   支撑相内与时间成正比、摆动相按声明形状归零；转向项用该腿自身的 `r`（前后腿横向退让反向）。
   > 依据（2026-09-23 实测，仅改 `locomote.stance_position_weight`，同 `vx=+0.2`/4 s 工况）：
   > 1.0 ⇒ 位移 −0.4604 m（**反向**）；0.2 ⇒ +0.6090 m（方向对）但倾角 179.29°（翻倒）。
3. **门禁不退化**：缺键 / 未知键 / 非法 `return_profile` / 非有限指令各自显式失败；
   退让把足端推出可达范围时由 `leg_ik` 显式拒绝（不截断、不静默兜底）。
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
from iraf_adapters.unitree.mpc import gait_trot  # noqa: E402
from iraf_adapters.unitree.quadruped import (  # noqa: E402
    CommandRejectedError,
    DeclarationError,
)

CONFIG = ROOT / "config/go2_loopback.yaml"
LOCOMOTE_CONFIG = ROOT / "config/go2_locomote.yaml"
PROFILE = ROOT / "profiles/unitree_go2_mujoco.yaml"

L1 = L2 = 0.213
NEUTRAL_Z = -2.0 * L1 * math.cos(0.9)
HOME_THIGH = 0.9
HOME_CALF = -1.8

LEG_CODES = ("FL", "FR", "RL", "RR")
#: 实测中立足端在躯干系的位置（与 test_gait_contract._geometry 同源）。
FOOT_XY = {
    "FL": (0.1934, 0.142),
    "FR": (0.1934, -0.142),
    "RL": (-0.1934, 0.142),
    "RR": (-0.1934, -0.142),
}


def _profile_joints():
    return [str(item) for item in
            yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["spec"]["joints"]]


def _document():
    return copy.deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))


def _walk_params(enabled=True, return_profile="cosine", anchor="command_ramp"):
    """生产链路：`config/go2_locomote.yaml` 的 mpc_gait（含 walk 覆盖）合并出的 trot 声明。

    用生产链路而不是手写参数字典 —— 这样「白名单是否接受 walk」「声明的值是否被真正消费」
    两件事一起被钉住。
    """
    document = yaml.safe_load(LOCOMOTE_CONFIG.read_text(encoding="utf-8")) or {}
    fragment = copy.deepcopy(document["mpc_gait"])
    if not enabled:
        fragment["overrides"]["walk"] = {"enabled": False, "return_profile": return_profile,
                                         "anchor": anchor, "stride_scale": 1.0}
    if return_profile != "cosine":
        fragment["overrides"]["walk"]["return_profile"] = return_profile
    if anchor != "command_ramp":
        fragment["overrides"]["walk"]["anchor"] = anchor
    base = yaml.safe_load((ROOT / fragment["base_declaration"]).read_text(encoding="utf-8"))
    return gait_trot.load_trot_gait(base, fragment, _profile_joints(),
                                    mpc_model=document["mpc_model"])


def _no_walk_params():
    """同一声明但**不含** `walk` 覆盖（= A/B 基准侧）：从生产覆盖里删掉 walk 键。"""
    document = yaml.safe_load(LOCOMOTE_CONFIG.read_text(encoding="utf-8")) or {}
    fragment = copy.deepcopy(document["mpc_gait"])
    fragment["overrides"].pop("walk", None)
    base = yaml.safe_load((ROOT / fragment["base_declaration"]).read_text(encoding="utf-8"))
    return gait_trot.load_trot_gait(base, fragment, _profile_joints(),
                                    mpc_model=document["mpc_model"])


def _geometry():
    geometry = {}
    for code in LEG_CODES:
        rx, ry = FOOT_XY[code]
        geometry[code] = {
            "l1_m": L1,
            "l2_m": L2,
            "neutral_x_m": 0.0,
            "neutral_z_m": NEUTRAL_Z,
            "trunk_rel_m": [rx, ry, NEUTRAL_Z],
            "foot_body": 0,
            "contact_geom": 0,
            "joints": {
                "hip_joint": "%s_hip_joint" % code,
                "thigh_joint": "%s_thigh_joint" % code,
                "calf_joint": "%s_calf_joint" % code,
            },
        }
    return geometry


def _home():
    home = {joint: 0.0 for joint in _profile_joints()}
    for code in LEG_CODES:
        home["%s_thigh_joint" % code] = HOME_THIGH
        home["%s_calf_joint" % code] = HOME_CALF
    return home


def _limits():
    """与 test_gait_contract 同一口径的宽限位（本文件只关心「有没有越界」，不关心边界值）。"""
    return {"%s_%s_joint" % (code, part): (-3.0, 3.0)
            for code in LEG_CODES for part in ("hip", "thigh", "calf")}


def _u_of(params, code, elapsed_s):
    return gait.leg_phase(params, code, elapsed_s) % 1.0


class WalkDeclarationTest(unittest.TestCase):
    """`gait.walk` 经生产链路（mpc_gait 覆盖白名单）被接受，且值被真正消费。"""

    def test_production_declaration_is_accepted_and_parsed(self):
        params = _walk_params()
        self.assertEqual(params["walk"], {"enabled": True, "return_profile": "cosine",
                                          "anchor": "command_ramp", "stride_scale": 1.0})
        # 与 mpc_model 的自洽门禁同时成立（period = 1/gait_hz）
        self.assertAlmostEqual(params["period_s"], 1.0 / 3.0, places=15)

    def test_walk_key_is_in_override_whitelist(self):
        self.assertIn("walk", gait_trot.ALLOWED_OVERRIDE_KEYS)

    def test_unknown_override_key_still_fails(self):
        """门禁未退化：白名单之外的键仍然显式失败。"""
        document = yaml.safe_load(LOCOMOTE_CONFIG.read_text(encoding="utf-8")) or {}
        fragment = copy.deepcopy(document["mpc_gait"])
        fragment["overrides"]["walkk"] = {"enabled": True, "return_profile": "cosine"}
        base = yaml.safe_load((ROOT / fragment["base_declaration"]).read_text(encoding="utf-8"))
        with self.assertRaises(DeclarationError):
            gait_trot.load_trot_gait(base, fragment, _profile_joints(),
                                     mpc_model=document["mpc_model"])

    def test_base_declaration_has_no_walk_block(self):
        """walk 只属于 MPC 路径：基准（wave/loopback）声明里不得出现它。"""
        self.assertNotIn("walk", _document()["gait"])


class WalkZeroImpactTest(unittest.TestCase):
    """未声明 / 关闭 / 零指令 ⇒ 精确 (0.0, 0.0)，且 `gait_joint_targets` 逐位不变。"""

    ELAPSED = 0.937

    def test_not_declared_returns_exact_zero(self):
        params = _no_walk_params()
        self.assertIsNone(params["walk"], "未声明时规范化结果必须是 None（而非静默丢弃键）")
        for code in LEG_CODES:
            self.assertEqual(
                gait.walk_foot_offset_m(params, code, self.ELAPSED, (0.2, 0.0, 0.0),
                                        FOOT_XY[code]), (0.0, 0.0))

    def test_disabled_returns_exact_zero(self):
        params = _walk_params(enabled=False)
        for code in LEG_CODES:
            self.assertEqual(
                gait.walk_foot_offset_m(params, code, self.ELAPSED, (0.2, 0.3, 0.5),
                                        FOOT_XY[code]), (0.0, 0.0))

    def test_zero_command_returns_exact_zero(self):
        params = _walk_params()
        for elapsed in (0.0, 0.05, 0.2, 0.5, 1.3):
            for code in LEG_CODES:
                self.assertEqual(
                    gait.walk_foot_offset_m(params, code, elapsed, (0.0, 0.0, 0.0),
                                            FOOT_XY[code]), (0.0, 0.0))

    def test_joint_targets_are_bit_identical_for_zero_command(self):
        """带 walk 声明但零指令 ⇒ 与不带 walk 声明的目标角**逐位相同**（`x + 0.0` 不改值）。"""
        with_walk = _walk_params()
        without = _no_walk_params()
        geometry = _geometry()
        for elapsed in (0.0, 0.017, 0.2, 0.31, 0.3333333333333333, 0.75, 1.4):
            base = gait.gait_joint_targets(without, geometry, _home(), _limits(), elapsed,
                                           1.0, (0.01, -0.02, 0.03), (0.0, 0.0, 0.1))
            same = gait.gait_joint_targets(with_walk, geometry, _home(), _limits(), elapsed,
                                           1.0, (0.01, -0.02, 0.03), (0.0, 0.0, 0.1))
            self.assertEqual(base, same, "elapsed=%r 处两路径目标角不同" % elapsed)

    def test_joint_targets_unchanged_when_walk_disabled_with_command(self):
        """`enabled: false` + 非零指令 ⇒ 目标角同样逐位不变（开关真能关掉）。"""
        off = _walk_params(enabled=False)
        without = _no_walk_params()
        geometry = _geometry()
        for elapsed in (0.1, 0.25, 0.6, 1.05):
            base = gait.gait_joint_targets(without, geometry, _home(), _limits(), elapsed,
                                           1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            off_targets = gait.gait_joint_targets(
                off, geometry, _home(), _limits(), elapsed, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                walk_command_mps_rad_s=(0.2, 0.0, 0.0))
            self.assertEqual(base, off_targets, "elapsed=%r 处关闭态未被尊重" % elapsed)


class WalkRetractionTest(unittest.TestCase):
    """退让的方向、量值、单调性与相位连续性。"""

    def setUp(self):
        self.params = _walk_params()
        self.period = float(self.params["period_s"])
        self.duty = float(self.params["duty_factor"])

    def test_straight_walk_retracts_backwards_at_command_speed(self):
        """`vx=+0.2` ⇒ 支撑腿 `dx` **负**（足端相对机身退让），支撑末值 = −vx·duty·period。"""
        vx = 0.2
        # FR 的 phase_offset = 0.0 ⇒ u = elapsed/period，便于解析定位。
        end_elapsed = self.duty * self.period
        dx_mid, dy_mid = gait.walk_foot_offset_m(
            self.params, "FR", end_elapsed * 0.5, (vx, 0.0, 0.0), FOOT_XY["FR"])
        dx_end, dy_end = gait.walk_foot_offset_m(
            self.params, "FR", end_elapsed, (vx, 0.0, 0.0), FOOT_XY["FR"])
        self.assertLess(dx_mid, 0.0)
        self.assertAlmostEqual(dx_mid, -vx * 0.5 * end_elapsed, places=15)
        self.assertAlmostEqual(dx_end, -vx * self.duty * self.period, places=15)
        self.assertAlmostEqual(dx_end, -0.2 * 0.6 / 3.0, places=15)
        self.assertAlmostEqual(dy_mid, 0.0, places=15)
        self.assertAlmostEqual(dy_end, 0.0, places=15)

    def test_retraction_is_monotone_through_stance(self):
        vx = 0.2
        previous = 0.0
        for step in range(1, 21):
            elapsed = self.duty * self.period * step / 20.0
            dx, _dy = gait.walk_foot_offset_m(self.params, "FR", elapsed, (vx, 0.0, 0.0),
                                              FOOT_XY["FR"])
            self.assertLessEqual(dx, previous + 1.0e-15)
            self.assertGreaterEqual(dx, -vx * self.duty * self.period - 1.0e-15)
            previous = dx

    def test_swing_returns_to_zero_before_next_stance(self):
        """摆动相内位移单调回到 0：`u → 1` 时精确 0，且全程不被拉回超过支撑末值。"""
        vx = 0.2
        end_value = -vx * self.duty * self.period
        previous = end_value
        for step in range(0, 21):
            u = self.duty + (1.0 - self.duty) * step / 20.0
            if u >= 1.0:
                continue
            elapsed = u * self.period
            dx, _dy = gait.walk_foot_offset_m(self.params, "FR", elapsed, (vx, 0.0, 0.0),
                                              FOOT_XY["FR"])
            self.assertGreaterEqual(dx, previous - 1.0e-15)
            self.assertLessEqual(dx, 0.0 + 1.0e-15)
            previous = dx
        # u → 1 的极限（用 1−1e-12 逼近，避免与下一周期 u=0 混叠）
        dx_last, _dy = gait.walk_foot_offset_m(self.params, "FR", (1.0 - 1.0e-12) * self.period,
                                               (vx, 0.0, 0.0), FOOT_XY["FR"])
        self.assertAlmostEqual(dx_last, 0.0, places=10)

    def test_seam_continuity_at_stance_to_swing(self):
        """`u = duty` 两侧的目标角连续（不得在落地/抬腿处跳变）。"""
        geometry = _geometry()
        vx = 0.2
        eps = 1.0e-9
        before = gait.gait_joint_targets(
            self.params, geometry, _home(), _limits(), (self.duty - eps) * self.period, 1.0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), walk_command_mps_rad_s=(vx, 0.0, 0.0))
        after = gait.gait_joint_targets(
            self.params, geometry, _home(), _limits(), (self.duty + eps) * self.period, 1.0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), walk_command_mps_rad_s=(vx, 0.0, 0.0))
        for joint in before:
            self.assertAlmostEqual(before[joint], after[joint], places=6,
                                   msg="关节 %s 在 u=duty 处不连续" % joint)

    def test_yaw_retraction_uses_each_leg_own_radius(self):
        """`wz=+0.5`（左转）：退让量 = |ω|·|r|·(该腿支撑内已走时长)，前后腿横向方向相反。

        取 `elapsed = 0.55·period`：FR/RL（偏移 0）u=0.55、FL/RR（偏移 0.5）u=0.05 —— 四条腿
        都在支撑相（duty 0.6）内，故解析量为 `u·period`（各自的 u 不同，必须逐腿算，不能用同一数）。
        """
        wz = 0.5
        elapsed = 0.55 * self.period
        signs = {}
        for code in LEG_CODES:
            rx, ry = FOOT_XY[code]
            u = _u_of(self.params, code, elapsed)
            self.assertLess(u, self.duty, "测试前提：%s 应处于支撑相" % code)
            amount = u * self.period
            dx, dy = gait.walk_foot_offset_m(self.params, code, elapsed, (0.0, 0.0, wz),
                                             FOOT_XY[code])
            self.assertAlmostEqual(dx, amount * wz * ry, places=15)
            self.assertAlmostEqual(dy, -amount * wz * rx, places=15)
            self.assertAlmostEqual(math.hypot(dx, dy), abs(wz) * math.hypot(rx, ry) * amount,
                                   places=15)
            signs[code] = dy
        # 左转（ω>0，机身逆时针）⇒ 世界静止的足端在机身系里**顺时针**退让：
        # 前腿（x>0）向 −y、后腿（x<0）向 +y；左腿（y>0）向 +x、右腿（y<0）向 −x。
        self.assertLess(signs["FL"], 0.0)
        self.assertLess(signs["FR"], 0.0)
        self.assertGreater(signs["RL"], 0.0)
        self.assertGreater(signs["RR"], 0.0)

    def test_lateral_command_retracts_sideways(self):
        """`vy=+0.3`（向左侧移）：四条腿一致向 −y 退让（量按各自支撑内时长）。"""
        vy = 0.3
        elapsed = 0.55 * self.period
        for code in LEG_CODES:
            u = _u_of(self.params, code, elapsed)
            self.assertLess(u, self.duty, "测试前提：%s 应处于支撑相" % code)
            amount = u * self.period
            dx, dy = gait.walk_foot_offset_m(self.params, code, elapsed, (0.0, vy, 0.0),
                                             FOOT_XY[code])
            self.assertAlmostEqual(dx, 0.0, places=15)
            self.assertAlmostEqual(dy, -vy * amount, places=15)

    def test_joint_targets_change_only_when_command_nonzero(self):
        """带 walk 声明：非零指令确实改变目标角（否则「声明了但没人消费」）。"""
        geometry = _geometry()
        elapsed = 0.5 * self.duty * self.period
        still = gait.gait_joint_targets(
            self.params, geometry, _home(), _limits(), elapsed, 1.0, (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), walk_command_mps_rad_s=(0.0, 0.0, 0.0))
        moving = gait.gait_joint_targets(
            self.params, geometry, _home(), _limits(), elapsed, 1.0, (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), walk_command_mps_rad_s=(0.2, 0.0, 0.0))
        changed = [joint for joint in still if still[joint] != moving[joint]]
        self.assertTrue(changed, "非零指令未改变任何关节目标角")
        # 此刻处于支撑相的腿必须都在变化集合里（FR/RL 的 phase_offset=0.0 ⇒ u 相同）
        for code in ("FR", "RL"):
            u = _u_of(self.params, code, elapsed)
            if u < self.duty:
                self.assertIn("%s_thigh_joint" % code, changed)

    def test_reach_guard_rejects_oversized_retraction(self):
        """退让把足端推出可达范围 ⇒ `leg_ik` 显式拒绝（不截断、不静默兜底）。"""
        geometry = _geometry()
        with self.assertRaises(CommandRejectedError):
            gait.gait_joint_targets(self.params, geometry, _home(), _limits(), 0.1, 1.0,
                                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                    walk_command_mps_rad_s=(50.0, 0.0, 0.0))


class WalkGateTest(unittest.TestCase):
    """声明门禁：缺键 / 未知键 / 非法形状 / 非有限指令必须各自显式失败。"""

    def _params_with(self, block):
        params = _walk_params()
        params = copy.deepcopy(params)
        params["walk"] = block
        return params

    def test_missing_keys_fail(self):
        for block in ({"return_profile": "cosine"}, {"enabled": True}):
            with self.assertRaises(DeclarationError):
                gait.walk_foot_offset_m(self._params_with(block), "FR", 0.1, (0.2, 0.0, 0.0),
                                        FOOT_XY["FR"])

    def test_unknown_key_fails(self):
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(
                self._params_with({"enabled": True, "return_profile": "cosine",
                                   "stride_m": 0.04}), "FR", 0.1, (0.2, 0.0, 0.0), FOOT_XY["FR"])

    def test_non_mapping_fails(self):
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(self._params_with(True), "FR", 0.1, (0.2, 0.0, 0.0),
                                    FOOT_XY["FR"])

    def test_bad_return_profile_fails(self):
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(
                self._params_with({"enabled": True, "return_profile": "triangle"}),
                "FR", 0.1, (0.2, 0.0, 0.0), FOOT_XY["FR"])

    def test_non_finite_command_fails(self):
        block = {"enabled": True, "return_profile": "cosine", "anchor": "command_ramp",
                 "stride_scale": 1.0}
        for command in ((float("inf"), 0.0, 0.0), (0.0, float("nan"), 0.0),
                        (0.0, 0.0, float("-inf"))):
            with self.assertRaises(CommandRejectedError):
                gait.walk_foot_offset_m(self._params_with(block), "FR", 0.1, command,
                                        FOOT_XY["FR"])

    def test_disabled_block_still_validates_shape(self):
        """`enabled: false` 不是「跳过校验」的借口：形状错仍要失败（假声明不许静默）。"""
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(
                self._params_with({"enabled": False, "return_profile": "sine"}), "FR", 0.1,
                (0.2, 0.0, 0.0), FOOT_XY["FR"])


class WalkMeasuredPoseTest(unittest.TestCase):
    """`anchor: measured_pose`（世界系锚定）：不产生推进量、按实测位姿换算、缺位姿显式失败。"""

    def setUp(self):
        self.params = _walk_params(anchor="measured_pose")
        self.period = float(self.params["period_s"])
        self.duty = float(self.params["duty_factor"])
        self.foot = FOOT_XY["FR"]
        # u = 0.1 < duty ⇒ FR 处于支撑相（phase_offset = 0.0 ⇒ u 随 elapsed 线性）
        self.stance_elapsed = 0.1 * self.period
        self.swing_elapsed = 0.95 * self.period

    def _pose(self, body_xy, yaw, origin):
        return {"body_xy_m": body_xy, "body_yaw_rad": yaw, "stance_foot_world_m": origin}

    def test_body_at_rest_yields_exact_zero(self):
        """机身不动 + 零指令 ⇒ 精确 (0.0, 0.0)（锚点 = 当前中立位）。"""
        pose = self._pose((0.0, 0.0), 0.0, None)
        offset = gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed,
                                         (0.0, 0.0, 0.0), self.foot, pose)
        self.assertEqual(offset, (0.0, 0.0))

    def test_no_self_drive_when_body_does_not_move(self):
        """指令非零但机身实测未动 ⇒ 支撑相偏移仍为 0（**不产生开环推进量**，与 command_ramp 档的本质区别）。"""
        pose = self._pose((0.0, 0.0), 0.0, (self.foot[0], self.foot[1]))
        offset = gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed,
                                         (0.2, 0.0, 0.0), self.foot, pose)
        self.assertEqual(offset, (0.0, 0.0))

    def test_body_displacement_keeps_foot_in_world(self):
        """机身被推动 Δ ⇒ 支撑腿偏移 = −Δ（足端世界系不动，不再把机身钉回站立点）。"""
        origin = (self.foot[0], self.foot[1])
        for delta in ((0.5, 0.0), (0.0, -0.3), (0.2, 0.25)):
            pose = self._pose(delta, 0.0, origin)
            offset = gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed,
                                             (0.0, 0.0, 0.0), self.foot, pose)
            self.assertAlmostEqual(offset[0], -delta[0], places=15)
            self.assertAlmostEqual(offset[1], -delta[1], places=15)

    def test_world_to_body_rotation_uses_measured_yaw(self):
        """机身偏航 θ ⇒ 偏移按 R(θ)ᵀ 换算（含转向时的前后腿方向差异）。"""
        theta = math.pi / 2.0
        body = (0.1, 0.05)
        origin = (0.3, 0.4)
        pose = self._pose(body, theta, origin)
        offset = gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed,
                                         (0.0, 0.0, 0.0), self.foot, pose)
        dx, dy = origin[0] - body[0], origin[1] - body[1]
        expected = (math.cos(theta) * dx + math.sin(theta) * dy - self.foot[0],
                    -math.sin(theta) * dx + math.cos(theta) * dy - self.foot[1])
        self.assertAlmostEqual(offset[0], expected[0], places=15)
        self.assertAlmostEqual(offset[1], expected[1], places=15)

    def test_swing_returns_to_neutral(self):
        """摆动相末端目标回到「当前机身位姿下的中立位」⇒ 偏移 → 0，且中途单调收敛。"""
        origin = (self.foot[0] - 0.1, self.foot[1] + 0.05)
        previous = None
        for step in range(0, 21):
            u = self.duty + (1.0 - self.duty) * step / 20.0
            if u >= 1.0:
                continue
            pose = self._pose((0.0, 0.0), 0.0, origin)
            offset = gait.walk_foot_offset_m(self.params, "FR", u * self.period,
                                             (0.0, 0.0, 0.0), self.foot, pose)
            norm = math.hypot(offset[0], offset[1])
            if previous is not None:
                self.assertLessEqual(norm, previous + 1.0e-12)
            previous = norm
        pose = self._pose((0.0, 0.0), 0.0, origin)
        offset = gait.walk_foot_offset_m(self.params, "FR", (1.0 - 1.0e-12) * self.period,
                                         (0.0, 0.0, 0.0), self.foot, pose)
        self.assertAlmostEqual(offset[0], 0.0, places=10)
        self.assertAlmostEqual(offset[1], 0.0, places=10)

    def test_zero_impact_when_body_at_rest_via_joint_targets(self):
        """锚定档在「机身不动 + 零指令」下与「不带 walk 声明」**逐位相同**。"""
        without = _no_walk_params()
        geometry = _geometry()
        pose = {code: self._pose((0.0, 0.0), 0.0, None) for code in LEG_CODES}
        for elapsed in (0.0, 0.11, 0.2, 0.31, 0.3333333333333333, 0.75, 1.4):
            base = gait.gait_joint_targets(without, geometry, _home(), _limits(), elapsed, 1.0,
                                           (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            anchored = gait.gait_joint_targets(
                self.params, geometry, _home(), _limits(), elapsed, 1.0,
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                walk_command_mps_rad_s=(0.0, 0.0, 0.0), walk_pose=pose)
            self.assertEqual(base, anchored, "elapsed=%r 处两路径目标角不同" % elapsed)

    def test_missing_pose_is_explicit(self):
        """`measured_pose` 档缺实测位姿 ⇒ 显式失败（不允许默默退回开环或零）。"""
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed, (0.2, 0.0, 0.0),
                                    self.foot, None)
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(self.params, "FR", self.stance_elapsed, (0.2, 0.0, 0.0),
                                    self.foot, {"body_xy_m": (0.0, 0.0)})

    def test_bad_stride_scale_fails(self):
        """`stride_scale` 必须是正有限数（0 / 负 / NaN 一律显式失败，不做兜底）。"""
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            params = copy.deepcopy(self.params)
            params["walk"]["stride_scale"] = bad
            with self.assertRaises(DeclarationError):
                gait.walk_foot_offset_m(params, "FR", self.stance_elapsed, (0.2, 0.0, 0.0),
                                        self.foot, self._pose((0.0, 0.0), 0.0, None))

    def test_bad_anchor_value_fails(self):
        """`anchor` 只允许白名单两值（写错名字必须失败，不许静默当成某一档）。"""
        params = copy.deepcopy(self.params)
        params["walk"]["anchor"] = "world_frame"
        with self.assertRaises(DeclarationError):
            gait.walk_foot_offset_m(params, "FR", self.stance_elapsed, (0.2, 0.0, 0.0),
                                    self.foot, self._pose((0.0, 0.0), 0.0, None))


if __name__ == "__main__":
    unittest.main()
