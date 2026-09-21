"""步态契约层测试（步骤 02）：声明校验、IK、相位→目标角、阻尼偏移、验收判定。

覆盖三类：
1. **正例**：真实声明（首期 = wave 四相位）能通过校验；IK 与实测中立位形逐位一致；
   合成「合格采样序列」能判为通过（没有正例对照就无法区分「门禁严格」与「门禁恒失败」）。
2. **负例**：步频为 0、占空比越界（trot <0.5 / wave <0.75 / ≥1.0）、步高为负、相位不自洽
   （trot 不对角、wave 非等间隔或两条腿共用相位）、缺键、未知轨迹形状、越出 Profile 限位、
   判据不达标（饱和/不离地/相位结构错/两条腿同时离地/漂移/高度波动）。
3. **边界**：阻尼偏移的截断、机身零平动但滚转时的横向偏移（ω × r 项）、斜坡。

两条路线都在覆盖内：`wave` 是首期步态（真实声明），`trot` 是二期性能优化（声明键保留，
由 `_trot_params()` 在本文件内构造，避免「二期复用」只停留在注释里）。

全部用例不依赖 MuJoCo 与本机是否跑过场景构建（几何由参数直接给出）。
"""

import copy
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree.quadruped import (  # noqa: E402
    CommandRejectedError,
    DeclarationError,
)

CONFIG = ROOT / "config/go2_loopback.yaml"
PROFILE = ROOT / "profiles/unitree_go2_mujoco.yaml"

#: 实测几何（本机 /usr/bin/python3 + 场景产物）：大腿长 = 小腿长 = 0.213 m。
#: 中立位形（关键帧 `home`：thigh=0.9, calf=-1.8）下足端相对大腿锚点的位移可由该位形**解析**给出：
#: `pz = -(L1·cos q1 + L2·cos(q1+q2)) = -2·L1·cos(0.9)`（两条连杆对称折叠），前 8 位小数
#: 即实测值 0.26480585 m。这里用解析式而不是截断的十进制常数：截断值会让 IK 往返差 1.05e-8 rad，
#: 把「常数少写几位」伪装成「IK 有精度损失」。
L1 = L2 = 0.213
NEUTRAL_Z = -2.0 * L1 * math.cos(0.9)
HOME_THIGH = 0.9
HOME_CALF = -1.8


def _profile_joints():
    return [str(item) for item in yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["spec"]["joints"]]


def _declaration():
    return copy.deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))


def _params(document=None, joints=None):
    return gait.load_gait_declaration(document or _declaration(), joints or _profile_joints())


def _trot_params():
    """trot 路线的声明（对角小跑）：在真实声明的副本上改回两组、组间半周期。"""
    document = _declaration()
    document["gait"]["kind"] = "trot"
    document["gait"]["duty_factor"] = 0.5
    for code, offset in (("FL", 0.0), ("RR", 0.0), ("FR", 0.5), ("RL", 0.5)):
        document["gait"]["legs"][code]["phase_offset"] = offset
    return gait.load_gait_declaration(document, _profile_joints())


def _geometry():
    """与实测同形状的几何（不依赖 mujoco）：四条腿都直接在大腿锚点下方。"""
    geometry = {}
    for code in ("FL", "RR", "FR", "RL"):
        geometry[code] = {
            "l1_m": L1,
            "l2_m": L2,
            "neutral_x_m": 0.0,
            "neutral_z_m": NEUTRAL_Z,
            "trunk_rel_m": [0.1934 if code.startswith(("F",)) else -0.1934,
                            0.142 if code.endswith("L") else -0.142,
                            NEUTRAL_Z],
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
    for code in ("FL", "RR", "FR", "RL"):
        home["%s_thigh_joint" % code] = HOME_THIGH
        home["%s_calf_joint" % code] = HOME_CALF
    return home


def _limits():
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))["spec"]
    return {joint: [float(v[0]), float(v[1])] for joint, v in profile["joint_limits"].items()}


def _samples(params, *, lift_height_m=0.03, period_hz=None, stance_frac=None, height=0.278,
             height_ripple=0.002, displacement=0.001, saturated=0, tracking=0.09,
             broken_leg=None, duration_s=10.0, tilt=1.0):
    """合成采样序列：按声明的相位结构给出各腿接触力（支撑 60 N / 摆动 0 N）。

    `stance_frac` 缺省取**声明**的 duty（不是写死的 0.5）：否则声明一改（trot 0.5 → wave 0.75），
    正例对照就会因为「合成序列与声明不符」整条翻红，把「声明变更」误报成「实现回归」。
    """
    frequency = float(period_hz or params["frequency_hz"])
    period = 1.0 / frequency
    stance_frac = float(params["duty_factor"]) if stance_frac is None else float(stance_frac)
    steps = int(duration_s * 100)
    samples = []
    for index in range(steps):
        t = index / 100.0
        contact = {}
        for code in sorted(params["legs"]):
            phase = ((t / period) + params["legs"][code]["phase_offset"]) % 1.0
            # 边界归属必须**确定性**：wave duty = 0.75 时每 0.04 s 就有一个采样点正好落在
            # 支撑/摆动边界上，而 `1.25*t` 与 `(1.25*t + 0.25) % 1` 的浮点结果会一个落在
            # 边界内、一个落在外（实测 t=0.6：0.7499999999999999 判支撑、0.9999999999999999 判摆动），
            # 于是合成序列出现「同一边界、两条腿归属相反」的浮点假象，把相位结构与离地判据
            # 一起打红。夹具按边距归属（真实模型不存在「恰好等于边界」的瞬时归属）。
            # 这不是放宽判据：判据阈值一字未改，改的是夹具在边界上的确定性。
            stance = phase < stance_frac - 1.0e-9
            if code == broken_leg:
                stance = True
            contact[code] = 60.0 if stance else 0.0
        samples.append(
            {
                "time_s": t,
                "base_height_m": height + height_ripple * math.sin(2.0 * math.pi * t / period),
                "base_position_xy_m": [displacement * t / duration_s, 0.0],
                "base_linear_speed_mps": displacement / duration_s,
                "tilt_deg": tilt,
                "roll_deg": tilt * 0.7,
                "pitch_deg": tilt * 0.7,
                "stab_offset_m": {"FL": [0.0, 0.0], "RR": [0.0, 0.0], "FR": [0.0, 0.0], "RL": [0.0, 0.0]},
                "contact_n": contact,
                "ctrl_saturated": saturated,
                "tracking_error_rad": tracking,
            }
        )
    return samples


class GaitDeclarationTests(unittest.TestCase):
    def test_real_declaration_is_valid_and_self_consistent(self):
        params = _params()
        self.assertEqual("wave", params["kind"])
        self.assertEqual(0.75, params["duty_factor"])
        self.assertAlmostEqual(1.0 / params["frequency_hz"], params["period_s"], places=12)
        # 相位组的约定：按相位偏移**升序**给出，每组带自己的偏移。wave 是四组各一条腿
        # （0 / 0.25 / 0.5 / 0.75）；判据按该结构逐组比较，不再假定「只有两组」。
        self.assertEqual(
            [
                (0.0, ["FL"]),
                (0.25, ["RR"]),
                (0.5, ["FR"]),
                (0.75, ["RL"]),
            ],
            [(group["offset"], group["legs"]) for group in params["phase_groups"]],
        )
        self.assertEqual(4, len(params["legs"]))

    def test_trot_route_is_retained_for_phase_two(self):
        """二期性能优化的 trot 路线必须仍然可加载（否则「保留」只是注释里的说法）。"""
        params = _trot_params()
        self.assertEqual("trot", params["kind"])
        self.assertEqual(0.5, params["duty_factor"])
        self.assertEqual(
            [(0.0, ["FL", "RR"]), (0.5, ["FR", "RL"])],
            [(group["offset"], group["legs"]) for group in params["phase_groups"]],
        )

    def test_wave_duty_below_static_stability_fails(self):
        """wave 的静态稳定前提「任意时刻 ≥ n−1 条腿支撑」⇒ duty ≥ (n−1)/n = 0.75，更小即拒绝。"""
        document = _declaration()
        document["gait"]["duty_factor"] = 0.70
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        message = str(caught.exception)
        self.assertIn("duty_factor", message)
        self.assertIn("0.750000", message)

    def test_wave_requires_one_leg_per_phase(self):
        """两条腿共用同一相位偏移 ⇒ 同一时刻会抬起两条腿，必须显式失败。"""
        document = _declaration()
        document["gait"]["legs"]["FR"]["phase_offset"] = 0.25
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("wave", str(caught.exception))

    def test_wave_requires_even_phase_spacing(self):
        document = _declaration()
        document["gait"]["legs"]["RL"]["phase_offset"] = 0.7
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("等间隔", str(caught.exception))

    def test_missing_section_fails(self):
        document = _declaration()
        document.pop("gait")
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("gait", str(caught.exception))

    def test_missing_key_fails(self):
        for key in ("frequency_hz", "step_height_m", "duty_factor", "swing_profile", "ramp_s",
                    "stabilization", "legs", "verification"):
            with self.subTest(key=key):
                document = _declaration()
                document["gait"].pop(key)
                with self.assertRaises(DeclarationError) as caught:
                    gait.load_gait_declaration(document, _profile_joints())
                self.assertIn(key, str(caught.exception))

    def test_missing_verification_key_fails(self):
        document = _declaration()
        document["gait"]["verification"].pop("max_displacement_m")
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("max_displacement_m", str(caught.exception))

    def test_missing_stabilization_key_fails(self):
        document = _declaration()
        document["gait"]["stabilization"].pop("linear_damping_s")
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("linear_damping_s", str(caught.exception))

    def test_zero_frequency_fails(self):
        document = _declaration()
        document["gait"]["frequency_hz"] = 0.0
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("frequency_hz", str(caught.exception))

    def test_negative_step_height_fails(self):
        document = _declaration()
        document["gait"]["step_height_m"] = -0.01
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("step_height_m", str(caught.exception))

    def test_zero_step_height_fails(self):
        document = _declaration()
        document["gait"]["step_height_m"] = 0.0
        with self.assertRaises(DeclarationError):
            gait.load_gait_declaration(document, _profile_joints())

    def test_duty_factor_out_of_range_fails(self):
        for value in (0.3, 1.0, 1.2, 0.0):
            with self.subTest(duty=value):
                document = _declaration()
                document["gait"]["duty_factor"] = value
                with self.assertRaises(DeclarationError) as caught:
                    gait.load_gait_declaration(document, _profile_joints())
                self.assertIn("duty_factor", str(caught.exception))

    def test_unknown_kind_fails(self):
        document = _declaration()
        document["gait"]["kind"] = "pace"
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("kind", str(caught.exception))

    def test_unknown_swing_profile_fails(self):
        document = _declaration()
        document["gait"]["swing_profile"] = "trapezoid"
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("swing_profile", str(caught.exception))

    def test_non_diagonal_phase_offsets_fail(self):
        document = _declaration()
        for leg in document["gait"]["legs"].values():
            leg["phase_offset"] = 0.0
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("相位", str(caught.exception))

    def test_quarter_period_offset_fails(self):
        document = _declaration()
        document["gait"]["legs"]["FR"]["phase_offset"] = 0.25
        with self.assertRaises(DeclarationError):
            gait.load_gait_declaration(document, _profile_joints())

    def test_leg_count_must_be_four(self):
        document = _declaration()
        document["gait"]["legs"].pop("RL")
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("4", str(caught.exception))

    def test_joint_must_come_from_profile(self):
        document = _declaration()
        document["gait"]["legs"]["FL"]["thigh_joint"] = "FL_thigh"
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("Profile", str(caught.exception))

    def test_odd_profile_bins_fail(self):
        document = _declaration()
        document["gait"]["verification"]["profile_bins"] = 5
        with self.assertRaises(DeclarationError) as caught:
            gait.load_gait_declaration(document, _profile_joints())
        self.assertIn("profile_bins", str(caught.exception))


class InverseKinematicsTests(unittest.TestCase):
    def test_home_pose_round_trip_matches_measured_keyframe(self):
        q1, q2 = gait.leg_ik(0.0, NEUTRAL_Z, L1, L2)
        self.assertAlmostEqual(HOME_THIGH, q1, places=9)
        self.assertAlmostEqual(HOME_CALF, q2, places=9)

    def test_forward_kinematics_round_trip(self):
        for px, pz in ((0.0, -0.24), (0.03, -0.25), (-0.03, -0.22), (0.0, -0.42)):
            with self.subTest(px=px, pz=pz):
                q1, q2 = gait.leg_ik(px, pz, L1, L2)
                fx, fz = gait.leg_forward_kinematics(q1, q2, L1, L2)
                self.assertAlmostEqual(px, fx, places=9)
                self.assertAlmostEqual(pz, fz, places=9)

    def test_unreachable_target_fails(self):
        with self.assertRaises(CommandRejectedError):
            gait.leg_ik(0.0, -0.9, L1, L2)

    def test_leg_solve_neutral_equals_home(self):
        hip, thigh, calf = gait.leg_solve(0.0, 0.0, NEUTRAL_Z, L1, L2)
        self.assertAlmostEqual(0.0, hip, places=12)
        self.assertAlmostEqual(HOME_THIGH, thigh, places=9)
        self.assertAlmostEqual(HOME_CALF, calf, places=9)

    def test_leg_solve_lateral_offset_is_exact(self):
        """横向偏移必须精确落在目标上（q_hip = atan2(dy, −pz)，不用小角近似）。"""
        dy = 0.02
        hip, _, _ = gait.leg_solve(0.0, dy, NEUTRAL_Z, L1, L2)
        self.assertAlmostEqual(math.atan2(dy, -NEUTRAL_Z), hip, places=12)

    def test_leg_solve_rejects_target_above_anchor(self):
        with self.assertRaises(CommandRejectedError):
            gait.leg_solve(0.0, 0.0, 0.01, L1, L2)


class PhaseAndTargetTests(unittest.TestCase):
    def test_stance_target_is_neutral(self):
        params = _params()
        self.assertEqual((0.0, 0.0), gait.foot_offset(0.0, params))

    def test_swing_target_lifts_to_declared_height(self):
        params = _params()
        mid = params["duty_factor"] + (1.0 - params["duty_factor"]) * 0.5
        _, dz = gait.foot_offset(mid, params)
        self.assertAlmostEqual(params["step_height_m"], dz, places=12)

    def test_cosine_profile_has_zero_height_and_slope_at_both_ends(self):
        params = _params()
        params["swing_profile"] = "cosine"
        duty = params["duty_factor"]
        for progress in (duty, duty + (1.0 - duty) * 1e-6, duty + (1.0 - duty) * (1 - 1e-6)):
            with self.subTest(progress=progress):
                _, dz = gait.foot_offset(progress, params)
                self.assertLess(abs(dz), 1e-6)

    def test_unknown_profile_in_params_fails(self):
        params = _params()
        params["swing_profile"] = "unknown"
        # 必须取**摆动相**相位：支撑相（u < duty）在检查形状之前就返回中立偏移，
        # 用 0.25 这类支撑相相位会让「未知形状」永远走不到校验分支（恒不触发的负向用例）。
        with self.assertRaises(DeclarationError):
            gait.foot_offset(
                params["duty_factor"] + (1.0 - params["duty_factor"]) * 0.5, params
            )

    def test_amplitude_ramp(self):
        params = _params()
        self.assertAlmostEqual(0.0, gait.amplitude_at(params, 0.0), places=12)
        self.assertAlmostEqual(0.5, gait.amplitude_at(params, params["ramp_s"] * 0.5), places=12)
        self.assertAlmostEqual(1.0, gait.amplitude_at(params, params["ramp_s"] * 2.0), places=12)

    def test_targets_respect_profile_limits(self):
        params = _params()
        targets = gait.trot_joint_targets(
            params, _geometry(), _home(), _limits(), 0.0, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )
        self.assertEqual(12, len(targets))
        limits = _limits()
        for joint, value in targets.items():
            lower, upper = limits[joint]
            self.assertGreaterEqual(value, lower - 1e-9)
            self.assertLessEqual(value, upper + 1e-9)

    def test_diagonal_legs_share_the_same_target_at_same_phase(self):
        """trot：同相位的两条腿（对角）在同一时刻必须给出相同的关节角——步态结构的直接检查。"""
        params = _trot_params()
        targets = gait.trot_joint_targets(
            params, _geometry(), _home(), _limits(), 0.123, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )
        for joint in ("thigh_joint", "calf_joint"):
            self.assertAlmostEqual(
                targets["FL_%s" % joint], targets["RR_%s" % joint], places=12
            )
            self.assertAlmostEqual(
                targets["FR_%s" % joint], targets["RL_%s" % joint], places=12
            )

    def test_wave_swing_window_moves_exactly_one_leg(self):
        """wave：同一时刻只有一条腿处在摆动相 ⇒ 该相位上只有一条腿的目标角与其他腿不同。"""
        params = _params()
        targets = gait.gait_joint_targets(
            params, _geometry(), _home(), _limits(), 0.123, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )
        # 相位 0.123 时：FL(0.123) / RR(0.373) / FR(0.623) 支撑，RL(0.873) 摆动（duty 0.75）。
        for joint in ("thigh_joint", "calf_joint"):
            stance_value = targets["FL_%s" % joint]
            for code in ("RR", "FR"):
                self.assertAlmostEqual(
                    stance_value, targets["%s_%s" % (code, joint)], places=12,
                    msg="相位 0.123 时 %s 与 FL 同处支撑相，目标角必须相同" % code,
                )
            self.assertGreater(
                abs(targets["RL_%s" % joint] - stance_value), 1.0e-6,
                msg="相位 0.123 时 RL 处在摆动相，目标角必须与支撑腿不同（否则摆动相没生效）",
            )

    def test_legacy_names_are_thin_wrappers(self):
        """旧名（trot_joint_targets / assess_trot）必须与泛化后的名字等价，不是第二套实现。"""
        params = _params()
        self.assertEqual(
            gait.assess_gait(_samples(params), params, 15.0),
            gait.assess_trot(_samples(params), params, 15.0),
        )
        self.assertEqual(
            gait.gait_joint_targets(
                params, _geometry(), _home(), _limits(), 0.5, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
            ),
            gait.trot_joint_targets(
                params, _geometry(), _home(), _limits(), 0.5, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
            ),
        )

    def test_out_of_limit_target_fails_instead_of_clipping(self):
        params = _params()
        limits = _limits()
        limits["FL_thigh_joint"] = [0.0, 0.001]
        with self.assertRaises(CommandRejectedError):
            gait.trot_joint_targets(
                params, _geometry(), _home(), limits, 0.0, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
            )


class StabilizationTests(unittest.TestCase):
    """阻尼偏移的实现层契约。

    注意：这几条用例测的是 `stabilization_offset` 的**数学**（平动项、转动项、截断），
    与「当前声明是否启用」无关 —— 真实声明里 `enabled: false`（实测本形式净失稳，见
    `config/go2_loopback.yaml` 注释与 `build/iraf-24h-2/02/scan7-run1.txt`）。因此除
    「关闭 ⇒ 零偏移」那条外，各用例都显式把 `enabled` 置 True 再断言，否则测的就不是这段数学。
    """

    def _enabled_params(self):
        params = _params()
        params["stabilization"]["enabled"] = True
        return params

    def test_disabled_returns_zero(self):
        params = _params()
        params["stabilization"]["enabled"] = False
        self.assertEqual((0.0, 0.0), gait.stabilization_offset(params, (0.4, 0.2, 0.0), (0, 0, 0), (0, 0, -0.26)))

    def test_zero_gain_returns_zero(self):
        params = self._enabled_params()
        params["stabilization"]["linear_damping_s"] = 0.0
        self.assertEqual((0.0, 0.0), gait.stabilization_offset(params, (0.4, 0.0, 0.0), (0, 0, 0), (0, 0, -0.26)))

    def test_translation_follows_body_velocity(self):
        params = self._enabled_params()
        gain = params["stabilization"]["linear_damping_s"]
        dx, dy = gait.stabilization_offset(params, (0.1, -0.05, 0.0), (0, 0, 0), (0.19, 0.14, -0.26))
        self.assertAlmostEqual(gain * 0.1, dx, places=12)
        self.assertAlmostEqual(gain * -0.05, dy, places=12)

    def test_rotation_term_acts_even_without_translation(self):
        """机身零平动、只有滚转角速度时，外/内侧腿必须得到**同向**横向偏移（阻尼滚转）。"""
        params = self._enabled_params()
        gain = params["stabilization"]["linear_damping_s"]
        omega = (0.5, 0.0, 0.0)
        left = gait.stabilization_offset(params, (0.0, 0.0, 0.0), omega, (0.19, 0.14, -0.26))
        right = gait.stabilization_offset(params, (0.0, 0.0, 0.0), omega, (0.19, -0.14, -0.26))
        expected = gain * (-omega[0] * -0.26)
        self.assertAlmostEqual(expected, left[1], places=12)
        self.assertAlmostEqual(expected, right[1], places=12)
        self.assertGreater(abs(expected), 0.0)

    def test_offset_is_clamped_to_declared_limit(self):
        params = self._enabled_params()
        limit = params["stabilization"]["max_linear_offset_m"]
        dx, dy = gait.stabilization_offset(params, (10.0, -10.0, 0.0), (0, 0, 0), (0, 0, -0.26))
        self.assertAlmostEqual(limit, dx, places=12)
        self.assertAlmostEqual(-limit, dy, places=12)


class AssessmentTests(unittest.TestCase):
    """验收判定：正例必须是**通过**的（没有正例对照无法区分严格与恒失败）。"""

    def test_conforming_series_passes(self):
        params = _params()
        result = gait.assess_gait(_samples(params), params, 15.0)
        self.assertEqual([], result["failed_checks"])
        metrics = result["metrics"]
        self.assertGreater(metrics["samples"], 900)
        for leg in ("FL", "FR", "RL", "RR"):
            self.assertGreaterEqual(metrics["per_leg"][leg]["clear_swing_cycles"], 8)
        # 正例对照必须同时覆盖新增的支撑腿数判据。断言**门禁口径**而不是 3.0：
        # 合成序列在相位边界上有一格是部分支撑（实测 min = 2.9），写成 `== 3.0` 会把
        # 夹具的采样量化误报成实现回归。门禁仍是「≥ duty × 腿数 − duty_tolerance × 腿数 = 2.6」。
        self.assertGreaterEqual(metrics["support_legs"]["min"], 2.6)
        self.assertLessEqual(metrics["support_legs"]["max"], 4.0)

    def test_two_legs_airborne_together_fails_support_gate(self):
        """wave 的静态稳定前提被破坏（同一时刻两条腿离地）⇒ 支撑腿数判据必须失败。"""
        params = _params()
        samples = _samples(params)
        period = params["period_s"]
        for sample in samples:
            phase = (float(sample["time_s"]) / period) % 1.0
            # 人为把 RR / FR 的支撑相压到 1/4 周期（原本 duty = 0.75）⇒ 支撑腿数必然低于声明目标。
            for code in ("RR", "FR"):
                own = (phase + params["legs"][code]["phase_offset"]) % 1.0
                sample["contact_n"][code] = 60.0 if own < 0.25 else 0.0
        result = gait.assess_gait(samples, params, 15.0)
        self.assertIn("support_legs_profile", result["failed_checks"])

    def test_saturation_fails(self):
        params = _params()
        result = gait.assess_trot(_samples(params, saturated=1), params, 15.0)
        self.assertIn("ctrl_saturated_samples", result["failed_checks"])

    def test_leg_that_never_lifts_fails(self):
        params = _params()
        result = gait.assess_trot(_samples(params, broken_leg="FR"), params, 15.0)
        self.assertIn("leg_FR_clear_swing_cycles", result["failed_checks"])

    def test_excessive_tilt_fails(self):
        params = _params()
        result = gait.assess_trot(_samples(params, tilt=20.0), params, 15.0)
        self.assertIn("max_tilt_deg", result["failed_checks"])

    def test_drift_fails(self):
        params = _params()
        result = gait.assess_trot(_samples(params, displacement=0.5), params, 15.0)
        self.assertIn("max_displacement_m", result["failed_checks"])

    def test_bouncing_fails(self):
        params = _params()
        result = gait.assess_trot(_samples(params, height_ripple=0.05), params, 15.0)
        self.assertIn("height_std_m", result["failed_checks"])

    def test_wrong_phase_structure_fails(self):
        """把一条腿的相位搞错（同侧两条腿同相）⇒ 对角结构判据必须失败。

        注意**不能**用 RR 自己的 `phase_offset` 重写接触序列：那与 `_samples` 的生成方式逐位相同，
        是一次空操作，负向用例会变成恒不触发。必须让 RR 改用**同侧腿 RL** 的相位（即跨到另一组），
        此时同组 FL/RR 的支撑相相位环应完全相反。
        """
        params = _params()
        samples = _samples(params)
        period = params["period_s"]
        wrong_offset = params["legs"]["RL"]["phase_offset"]
        self.assertNotAlmostEqual(
            wrong_offset, params["legs"]["RR"]["phase_offset"], places=9,
            msg="负向用例的前提是两个相位偏移必须不同",
        )
        for sample in samples:
            t = float(sample["time_s"])
            wrong = ((t / period) + wrong_offset) % 1.0
            sample["contact_n"]["RR"] = 60.0 if wrong < params["duty_factor"] else 0.0
        result = gait.assess_trot(samples, params, 15.0)
        self.assertIn("phase_sequence_structure", result["failed_checks"])

    def test_empty_samples_fail_loudly(self):
        params = _params()
        with self.assertRaises(CommandRejectedError):
            gait.assess_trot([], params, 15.0)

    def test_collapse_fails(self):
        params = _params()
        samples = _samples(params, height=0.10)
        result = gait.assess_trot(samples, params, 15.0)
        self.assertIn("min_base_height_m", result["failed_checks"])
        self.assertIn("height_mean_m", result["failed_checks"])


if __name__ == "__main__":
    unittest.main()
