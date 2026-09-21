"""四足**力矩级平衡器**声明契约与纯函数单元测试（步骤 02b，战役 iraf-24h-2）。

覆盖（每条负向用例都配正向对照，否则分不清「门禁严格」与「门禁恒失败」）：

1. `load_balance_declaration`：真实声明可加载（**正向对照**，且逐项与 YAML 对齐）；缺段、
   缺任一必需键（逐键遍历）、权重同时为 0、`stance_classification` 取值不在白名单（决策 1e：
   `contact_only` / `declared_and_contact`）、`min_stance_legs > 4`、`normal_force_floor_n` 为正
   或超出法向力上限、`axes` 非法、`amplitudes_m` 空/含负值/不含 `no_fall_amplitude_m`、
   倾角上限写成 45° 以上 —— 全部必须被拒；
2. `desired_wrench`（纯函数，不需要仿真）：直立且达标高度 + 零速度 ⇒ 零纠正量；
   低于目标高度 ⇒ 法向纠正为正（向上推）；水平速度 ⇒ 阻尼力反向；滚转/俯仰 ⇒ 回正力矩反向；
   各项按声明上限截断；`include_gravity_support=true` 时才出现 `mg` 支撑项；
3. `allocate_foot_forces`：对称足形 + 对称力旋量 ⇒ 各腿均分（**正向对照**）；力旋量残差 ≈ 0；
   支撑腿数不足 ⇒ 拒绝；足形退化（共线）⇒ 拒绝；超出逐腿上限 ⇒ `clamped_legs` 如实记录；
4. `leg_joint_torques`：符号约定 `τ = −Jᵀf`（**实测踩过**：漏负号会把高度纠正与水平阻尼变成正反馈）；
   雅可比形状非法 ⇒ 拒绝。
5. 步态验收报告的 `balance` 证据段（`scripts/verify_go2_gait_in_place.py`，调试记录 §18.5 缺口修复）：
   **正向对照**直接用适配器自己的 `_balance_summary` 产出真实形状（生产代码路径，不是手抄夹具），
   断言本入口声明的每一条必需键路径都真的被强制（逐条删键 ⇒ 逐条被拒）；缺段/形状非法/未跑力控周期
   时 `fallback_fraction` 写 `null` 而不是 0。

夹具纪律：复制真实声明到临时目录后按需改坏（真实声明就是正向对照）；不读 `build/` 产物。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_balance_contract -v`
"""

import copy
import sys
import types
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import verify_go2_gait_in_place  # noqa: E402
from iraf_adapters.unitree import balance  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_adapters.unitree.quadruped import DeclarationError  # noqa: E402

DECLARATION = ROOT / "config" / "go2_loopback.yaml"


def _declaration():
    return yaml.safe_load(DECLARATION.read_text(encoding="utf-8"))


def _params(document=None):
    return balance.load_balance_declaration(document or _declaration())


def _mutate(path, value):
    document = _declaration()
    node = document
    parts = path.split(".")
    for part in parts[:-1]:
        node = node[part]
    if value is None:
        del node[parts[-1]]
    else:
        node[parts[-1]] = value
    return document


class BalanceDeclarationTests(unittest.TestCase):
    def test_real_declaration_loads_and_matches_yaml(self):
        """正向对照：真实声明必须能加载，且返回的数字能逐项在 YAML 里找到。"""
        document = _declaration()
        params = _params(document)
        section = document["balance"]
        # 开关值**只与声明对齐**、不硬编码：本轮把 enabled 由 true 改为 false 后，
        # 这条断言曾在本机全量单测里失败（"断言写成快照"而不是"断言契约"）。
        self.assertEqual(params["enabled"], bool(section["enabled"]))
        self.assertEqual(params["weight_position"], float(section["weight_position"]))
        self.assertEqual(params["weight_balance"], float(section["weight_balance"]))
        self.assertIs(params["include_gravity_support"], bool(section["include_gravity_support"]))
        self.assertEqual(
            params["attitude"]["kp_nm_per_rad"], float(section["attitude"]["kp_nm_per_rad"])
        )
        self.assertEqual(
            params["allocation"]["min_stance_legs"], int(section["allocation"]["min_stance_legs"])
        )
        self.assertEqual(
            params["watchdog"]["max_consecutive_no_stance_cycles"],
            int(section["watchdog"]["max_consecutive_no_stance_cycles"]),
        )
        self.assertEqual(
            params["verification"]["report"], str(section["verification"]["report"])
        )
        self.assertEqual(
            params["verification"]["isolation"]["amplitudes_m"],
            [float(item) for item in section["verification"]["isolation"]["amplitudes_m"]],
        )
        self.assertEqual(
            params["verification"]["static_disturbance"]["axes"],
            [str(item) for item in section["verification"]["static_disturbance"]["axes"]],
        )

    def test_stance_classification_declared_and_validated(self):
        """决策 1e：支撑集判定口径必须显式声明，且只允许白名单取值。

        缺键由 `test_every_required_key_rejected_when_missing` 逐键覆盖（该键已进
        `REQUIRED_BALANCE_KEYS`）；本用例补的是「**拼错/自造模式必须被拒**」与「另一个合法
        模式必须能加载」这对正/负对照 —— 否则分不清「门禁严格」与「门禁恒失败」。
        """
        document = _declaration()
        params = _params(document)
        self.assertEqual(
            params["stance_classification"],
            str(document["balance"]["stance_classification"]),
        )
        self.assertIn(params["stance_classification"], balance.STANCE_CLASSIFICATION_MODES)
        for bad in ("declared", "contact", "declared_and_measured", "", 1, None):
            with self.assertRaises(DeclarationError):
                _params(_mutate("balance.stance_classification", bad))
        with self.assertRaises(DeclarationError) as ctx:
            _params(_mutate("balance.stance_classification", "declared"))
        self.assertIn("stance_classification", str(ctx.exception))
        # 正例对照：两个已定义模式都必须能加载（含"声明相位 ∧ 实测接触"）。
        for mode in balance.STANCE_CLASSIFICATION_MODES:
            loaded = _params(_mutate("balance.stance_classification", mode))
            self.assertEqual(loaded["stance_classification"], mode)

    def test_missing_section_rejected(self):
        with self.assertRaises(DeclarationError) as ctx:
            _params(_mutate("balance", None))
        self.assertIn("balance", str(ctx.exception))

    def test_every_required_key_rejected_when_missing(self):
        """逐键遍历：缺任一必需键都必须显式失败（不允许实现层默认值兜底）。"""
        for key in balance.REQUIRED_BALANCE_KEYS:
            with self.subTest(key=key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.%s" % key, None))
        for key in balance.REQUIRED_ATTITUDE_KEYS:
            with self.subTest(key="attitude.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.attitude.%s" % key, None))
        for key in balance.REQUIRED_HEIGHT_KEYS:
            with self.subTest(key="height.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.height.%s" % key, None))
        for key in balance.REQUIRED_VELOCITY_KEYS:
            with self.subTest(key="velocity.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.velocity.%s" % key, None))
        for key in balance.REQUIRED_ALLOCATION_KEYS:
            with self.subTest(key="allocation.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.allocation.%s" % key, None))
        for key in balance.REQUIRED_WATCHDOG_KEYS:
            with self.subTest(key="watchdog.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.watchdog.%s" % key, None))
        for key in balance.REQUIRED_VERIFICATION_KEYS:
            with self.subTest(key="verification.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.verification.%s" % key, None))
        for key in balance.REQUIRED_DISTURBANCE_KEYS:
            with self.subTest(key="static_disturbance.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.verification.static_disturbance.%s" % key, None))
        for key in balance.REQUIRED_ISOLATION_KEYS:
            with self.subTest(key="isolation.%s" % key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.verification.isolation.%s" % key, None))

    def test_zero_weights_rejected(self):
        document = _mutate("balance.weight_position", 0.0)
        document["balance"]["weight_balance"] = 0.0
        with self.assertRaises(DeclarationError) as ctx:
            _params(document)
        self.assertIn("同时为 0", str(ctx.exception))

    def test_zero_weight_alone_is_allowed(self):
        """正向对照：只关掉一侧权重是合法的（不是「恒失败门禁」）。

        前提变更（B1，2026-09-21）：支撑腿的位置级权重 `stance_weight_position` 取 0 后，
        「关掉 weight_balance」不再是合法声明（支撑腿会拿到零控制量 ⇒ 塌陷，见下一条负向用例），
        因此本用例把 pair 的另一半显式设回非 0 —— 改的是**前提**，不是门禁。
        """
        document = _mutate("balance.weight_balance", 0.0)
        document["balance"]["stance_weight_position"] = 1.0
        params = _params(document)
        self.assertEqual(params["weight_balance"], 0.0)
        self.assertEqual(params["weight_position"], 1.0)

    def test_stance_weight_requires_balance_torque(self):
        """B1 负向用例：支撑腿位置权重为 0 时，力矩级权重也必须非 0（否则支撑腿零控制量）。"""
        document = _mutate("balance.stance_weight_position", 0.0)
        document["balance"]["weight_balance"] = 0.0
        with self.assertRaises(DeclarationError) as ctx:
            _params(document)
        self.assertIn("stance_weight_position", str(ctx.exception))

    def test_stance_weight_position_declared_and_matches_yaml(self):
        """B1 核心键必须存在、且与声明文件逐字一致（支撑腿力控的开关就在这里）。"""
        document = _declaration()
        section = document["balance"]
        self.assertIn("stance_weight_position", section)
        params = _params(document)
        self.assertEqual(params["stance_weight_position"], float(section["stance_weight_position"]))

    def test_weights_are_bounded_to_unit_interval(self):
        """权重是 τ_pd 的乘子：>1 是第二份增益（增益已由 control.* 声明）⇒ 实现层强制上界。"""
        for key in ("weight_position", "stance_weight_position", "weight_balance"):
            with self.subTest(key=key):
                with self.assertRaises(DeclarationError):
                    _params(_mutate("balance.%s" % key, 1.5))

    def test_min_stance_legs_must_be_reachable(self):
        with self.assertRaises(DeclarationError) as ctx:
            _params(_mutate("balance.allocation.min_stance_legs", 5))
        self.assertIn("四足", str(ctx.exception))

    def test_normal_floor_semantics(self):
        """`normal_force_floor_n`：必须 ≤ 0（纠正量可为负）且绝对值不超过法向力上限。"""
        params = _params()
        self.assertLessEqual(params["allocation"]["normal_force_floor_n"], 0.0)
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.allocation.normal_force_floor_n", 1.0))
        with self.assertRaises(DeclarationError) as ctx:
            _params(
                _mutate(
                    "balance.allocation.normal_force_floor_n",
                    -float(_params()["allocation"]["max_normal_force_n"]) - 1.0,
                )
            )
        self.assertIn("绝对值", str(ctx.exception))

    def test_axes_and_amplitudes_validation(self):
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.static_disturbance.axes", ["z"]))
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.static_disturbance.axes", []))
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.isolation.amplitudes_m", []))
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.isolation.amplitudes_m", [0.0, -0.001]))
        with self.assertRaises(DeclarationError) as ctx:
            _params(_mutate("balance.verification.isolation.no_fall_amplitude_m", 0.033))
        self.assertIn("amplitudes_m", str(ctx.exception))

    def test_tilt_limits_are_bounded(self):
        """倾角门禁上限由实现层强制：写成 45° 以上即恒真门禁。"""
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.isolation.max_tilt_deg", 90.0))
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.verification.static_disturbance.max_tilt_deg", 46.0))

    def test_enabled_must_be_boolean(self):
        with self.assertRaises(DeclarationError):
            _params(_mutate("balance.enabled", "true"))


class DesiredWrenchTests(unittest.TestCase):
    def _wrench(self, **kwargs):
        params = _params()
        base = {
            "mass_kg": 15.596408,
            "gravity_mps2": 9.81,
            "height_m": 0.27,
            "height_target_m": 0.27,
            "vertical_velocity_mps": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "horizontal_velocity_world_mps": (0.0, 0.0),
            "angular_velocity_world_rad_s": (0.0, 0.0, 0.0),
        }
        base.update(kwargs)
        return params, balance.desired_wrench(params, **base)

    def test_upright_at_target_is_zero_correction(self):
        """正向对照：达标且静止 ⇒ **纠正量**为零；总法向力 = 声明的重力支撑项。

        口径（B1，2026-09-21）：`force_n[2] = gravity_support_n + height_correction_n`，
        所以"是否为零"必须断在**分量**上，总力单独与声明的 `include_gravity_support` 对齐 ——
        断总力为零等于把断言写成上一版声明（`include_gravity_support: false`）的快照。
        """
        params, wrench = self._wrench()
        components = wrench["components"]
        self.assertEqual(components["height_correction_n"], 0.0)
        np.testing.assert_allclose(components["horizontal_damping_n"], np.zeros(2), atol=1e-12)
        np.testing.assert_allclose(wrench["torque_nm"], np.zeros(3), atol=1e-12)
        expected_support = 15.596408 * 9.81 if params["include_gravity_support"] else 0.0
        self.assertAlmostEqual(components["gravity_support_n"], expected_support, places=9)
        self.assertAlmostEqual(wrench["force_n"][2], expected_support, places=9)

    def test_gravity_support_only_when_declared(self):
        """开关两个取值都要覆盖（B1 的核心开关，不能只测声明里当前那一个值）。"""
        base = {
            "mass_kg": 15.596408,
            "gravity_mps2": 9.81,
            "height_m": 0.27,
            "height_target_m": 0.27,
            "vertical_velocity_mps": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "horizontal_velocity_world_mps": (0.0, 0.0),
            "angular_velocity_world_rad_s": (0.0, 0.0, 0.0),
        }
        for declared, expected in ((True, 15.596408 * 9.81), (False, 0.0)):
            with self.subTest(include_gravity_support=declared):
                params = balance.load_balance_declaration(
                    _mutate("balance.include_gravity_support", declared)
                )
                self.assertIs(params["include_gravity_support"], declared)
                wrench = balance.desired_wrench(params, **base)
                self.assertAlmostEqual(wrench["components"]["gravity_support_n"], expected, places=9)
                self.assertAlmostEqual(wrench["force_n"][2], expected, places=9)

    def test_height_error_pushes_up(self):
        params, wrench = self._wrench(height_m=0.25)
        limit = params["height"]["max_force_n"]
        correction = wrench["components"]["height_correction_n"]
        self.assertGreater(correction, 0.0)
        # 截断只作用在**纠正量**上；总法向力 = 纠正量 + 声明的重力支撑项（B1 下含 mg）。
        self.assertLessEqual(correction, limit)
        self.assertAlmostEqual(
            wrench["force_n"][2],
            correction + wrench["components"]["gravity_support_n"],
            places=9,
        )

    def test_height_correction_is_clamped(self):
        params, wrench = self._wrench(height_m=0.0)
        self.assertAlmostEqual(
            wrench["components"]["height_correction_n"], params["height"]["max_force_n"], places=9
        )

    def test_velocity_damping_opposes_motion(self):
        # 取值必须让 |阻尼力| 落在声明的 max_force_n 之内，否则量的是截断而非增益
        # （截断本身另有 test_damping_is_clamped 覆盖）。
        params, wrench = self._wrench(horizontal_velocity_world_mps=(0.1, -0.05))
        gain = params["velocity"]["linear_gain_ns_per_m"]
        self.assertLess(gain * 0.1, params["velocity"]["max_force_n"])
        np.testing.assert_allclose(
            wrench["components"]["horizontal_damping_n"], [-gain * 0.1, gain * 0.05], rtol=1e-12
        )

    def test_damping_is_clamped(self):
        params, wrench = self._wrench(horizontal_velocity_world_mps=(50.0, 0.0))
        limit = params["velocity"]["max_force_n"]
        self.assertAlmostEqual(wrench["force_n"][0], -limit, places=9)

    def test_attitude_torque_restores_upright(self):
        params, wrench = self._wrench(roll_rad=0.1, pitch_rad=0.0)
        self.assertLess(wrench["torque_nm"][0], 0.0)
        self.assertLessEqual(abs(wrench["torque_nm"][0]), params["attitude"]["max_torque_nm"])
        params2, wrench2 = self._wrench(pitch_rad=0.1)
        self.assertLess(wrench2["torque_nm"][1], 0.0)

    def test_yaw_is_damped_not_restored(self):
        """偏航只阻尼、不做回正：原地踏步的偏航目标由步态给。"""
        params, wrench = self._wrench(angular_velocity_world_rad_s=(0.0, 0.0, 0.05))
        self.assertLess(wrench["torque_nm"][2], 0.0)
        self.assertAlmostEqual(
            wrench["components"]["yaw_damping_torque_nm"],
            -float(params["velocity"]["angular_gain_nms_per_rad"]) * 0.05,
            places=12,
        )

    def test_requires_valid_mass_and_gravity(self):
        with self.assertRaises(DeclarationError):
            self._wrench(mass_kg=0.0)
        with self.assertRaises(DeclarationError):
            self._wrench(gravity_mps2=0.0)


class FootForceAllocationTests(unittest.TestCase):
    def setUp(self):
        self.params = _params()
        self.center = np.array([0.0, 0.0, 0.27])
        self.feet = {
            "FL": np.array([0.1934, 0.1420, 0.0]),
            "FR": np.array([0.1934, -0.1420, 0.0]),
            "RL": np.array([-0.1934, 0.1420, 0.0]),
            "RR": np.array([-0.1934, -0.1420, 0.0]),
        }

    def _wrench(self, force, torque):
        return {"force_n": np.asarray(force, dtype=float), "torque_nm": np.asarray(torque, dtype=float)}

    def test_symmetric_wrench_splits_evenly(self):
        """正向对照：对称足形 + 纯竖直**向上**力旋量 ⇒ 四腿均分，且残差 ≈ 0。

        方向口径（B1，2026-09-21）：法向力含 mg 支撑 ⇒ 运行时恒为正；地面无粘附，负法向力
        不可兑现，故正例取向上。负向情形的覆盖见下面两条（显式声明下界，不依赖当前声明值）。
        """
        result = balance.allocate_foot_forces(
            self.params, self._wrench([0.0, 0.0, 8.0], [0.0, 0.0, 0.0]), self.feet, self.center
        )
        verticals = [result["forces"][code][2] for code in sorted(self.feet)]
        for value in verticals:
            self.assertAlmostEqual(value, 2.0, places=9)
        np.testing.assert_allclose(result["residual"]["force_n"], np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(result["residual"]["torque_nm"], np.zeros(3), atol=1e-9)
        self.assertEqual(result["clamped_legs"], [])

    def test_negative_normal_realized_when_floor_allows(self):
        """下界为负时负向纠正必须能被兑现（旧结构的覆盖保留：不依赖当前声明值）。"""
        params = balance.load_balance_declaration(
            _mutate("balance.allocation.normal_force_floor_n", -30.0)
        )
        result = balance.allocate_foot_forces(
            params, self._wrench([0.0, 0.0, -8.0], [0.0, 0.0, 0.0]), self.feet, self.center
        )
        for code in sorted(self.feet):
            self.assertAlmostEqual(result["forces"][code][2], -2.0, places=9)
        self.assertEqual(result["clamped_legs"], [])

    def test_zero_floor_clamps_negative_request(self):
        """下界为 0（B1 口径：地面无粘附）时负向请求必须被**如实截断并记录**，不是静默给 0。"""
        params = balance.load_balance_declaration(
            _mutate("balance.allocation.normal_force_floor_n", 0.0)
        )
        result = balance.allocate_foot_forces(
            params, self._wrench([0.0, 0.0, -8.0], [0.0, 0.0, 0.0]), self.feet, self.center
        )
        for code in sorted(self.feet):
            self.assertAlmostEqual(result["forces"][code][2], 0.0, places=9)
        self.assertEqual(sorted(result["clamped_legs"]), ["FL", "FR", "RL", "RR"])
        self.assertAlmostEqual(result["residual"]["force_n"][2], -8.0, places=9)

    def test_moment_request_is_realized(self):
        """姿态力矩必须真的兑现（首跑踩过：法向力下界取 0 会把负向纠正整段截掉，力矩残差 == 目标力矩）。

        口径（B1）：正例带**向上**的竖直偏置（运行时真实情形：法向力 = mg 分摊 + 纠正量），
        此时两侧腿的差动力全为正、力矩可兑现；净竖直力为 0 的"纯纠正量"情形需要负下界，
        由 `test_negative_normal_realized_when_floor_allows` 覆盖。
        """
        result = balance.allocate_foot_forces(
            self.params, self._wrench([0.0, 0.0, 8.0], [1.0, 0.0, 0.0]), self.feet, self.center
        )
        np.testing.assert_allclose(result["residual"]["torque_nm"], np.zeros(3), atol=1e-9)
        self.assertNotEqual(result["forces"]["FL"][2], result["forces"]["FR"][2])
        self.assertEqual(result["clamped_legs"], [])

    def test_too_few_stance_legs_rejected(self):
        feet = {"FL": self.feet["FL"], "FR": self.feet["FR"]}
        with self.assertRaises(DeclarationError) as ctx:
            balance.allocate_foot_forces(
                self.params, self._wrench([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]), feet, self.center
            )
        self.assertIn("min_stance_legs", str(ctx.exception))

    def test_degenerate_footprint_rejected(self):
        feet = {
            "FL": np.array([0.0, 0.0, 0.0]),
            "FR": np.array([0.1, 0.0, 0.0]),
            "RL": np.array([0.2, 0.0, 0.0]),
            "RR": np.array([0.3, 0.0, 0.0]),
        }
        with self.assertRaises(DeclarationError) as ctx:
            balance.allocate_foot_forces(
                self.params, self._wrench([0.0, 0.0, -4.0], [0.0, 0.0, 0.0]), feet, self.center
            )
        self.assertIn("秩", str(ctx.exception))

    def test_clamping_is_recorded(self):
        """超出逐腿上限时如实记录（「要求了但没兑现」必须能与「本来没要求」区分开）。"""
        result = balance.allocate_foot_forces(
            self.params, self._wrench([0.0, 0.0, -400.0], [0.0, 0.0, 0.0]), self.feet, self.center
        )
        limit = self.params["allocation"]["max_normal_force_n"]
        floor = self.params["allocation"]["normal_force_floor_n"]
        self.assertTrue(result["clamped_legs"])
        for code, force in result["forces"].items():
            self.assertLessEqual(force[2], limit)
            self.assertGreaterEqual(force[2], floor)
            self.assertLessEqual(
                abs(force[0]), self.params["allocation"]["max_horizontal_force_n"]
            )
        self.assertGreater(abs(result["residual"]["force_n"][2]), 0.0)

    def test_wrench_matrix_shape(self):
        matrix = balance.wrench_matrix([item for item in self.feet.values()])
        self.assertEqual(matrix.shape, (6, 12))


class LegJointTorqueTests(unittest.TestCase):
    def test_sign_convention_is_minus_jacobian_transpose(self):
        """符号约定 `τ = −Jᵀf`：漏负号会把高度纠正与水平阻尼变成正反馈（实测）。"""
        jacobian = np.eye(3, dtype=float)
        torque = balance.leg_joint_torques(
            [0.0, 0.0, 2.0], jacobian, ["hip", "thigh", "calf"]
        )
        self.assertEqual(torque, {"hip": 0.0, "thigh": 0.0, "calf": -2.0})

    def test_joint_names_are_preserved(self):
        torque = balance.leg_joint_torques(
            [1.0, 2.0, 3.0], np.eye(3, dtype=float), ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"]
        )
        self.assertEqual(sorted(torque), ["FL_calf_joint", "FL_hip_joint", "FL_thigh_joint"])
        self.assertAlmostEqual(torque["FL_hip_joint"], -1.0, places=12)

    def test_bad_jacobian_shape_rejected(self):
        with self.assertRaises(DeclarationError):
            balance.leg_joint_torques([0.0, 0.0, 1.0], np.zeros((3, 4)), ["a", "b", "c"])


class GaitReportBalanceEvidenceTests(unittest.TestCase):
    """步态验收报告里的 `balance` 证据段（§18.5 缺口修复）。

    契约来自**生产侧**：正向对照直接调用适配器自己的 `_balance_summary`（不是手抄形状），
    负向侧逐条删除本入口声明的必需键路径 —— 若某条路径写了却没人检查，这条用例会立刻暴露
    （「门禁绿、计数 0」的同族缺陷）。
    """

    @staticmethod
    def _produced_segment():
        """**正向对照**：用真实声明 + 适配器 `_balance_summary` 产出 `balance` 段的真实形状。"""
        params = _params()
        # 只借 `_balance_summary` 的产出形状：`_new_balance_stats` 直接取**生产实现**（静态方法），
        # 不走真实装配（本用例不需要模型/仿真）。
        fake = types.SimpleNamespace(
            _balance_stats=None,  # 未跑控制循环 ⇒ 用 `_new_balance_stats()` 的零值统计
            _new_balance_stats=unitree_go2.UnitreeGo2Adapter._new_balance_stats,
            robot_mass_kg=lambda: 15.596408,
            gravity_mps2=lambda: 9.80665,
        )
        return unitree_go2.UnitreeGo2Adapter._balance_summary(fake, params)

    def test_real_producer_satisfies_required_paths(self):
        """真实生产形状必须通过必需键路径校验（否则门禁恒失败而不是严格）。"""
        segment = self._produced_segment()
        self.assertIs(verify_go2_gait_in_place._balance_segment({"balance": segment}), segment)

    def test_report_summary_aligns_with_declaration(self):
        """报告摘要的开关/口径必须与声明对齐（不硬编码取值：`enabled` 曾被改过一次）。"""
        section = _declaration()["balance"]
        summary = verify_go2_gait_in_place._balance_report_summary(self._produced_segment())
        self.assertEqual(summary["enabled"], bool(section["enabled"]))
        self.assertEqual(summary["stance_classification"], str(section["stance_classification"]))
        self.assertEqual(summary["weight_position"], float(section["weight_position"]))
        self.assertEqual(summary["stance_weight_position"], float(section["stance_weight_position"]))
        self.assertEqual(
            sorted(summary["stats"]),
            [
                "cycles",
                "declared_swing_in_contact_cycles",
                "declared_swing_in_contact_samples",
                "fallback_cycles",
                "fallback_fraction",
                "fallback_fraction_defined",
                "force_control_cycles",
                "max_abs_torque_nm",
                "no_stance_cycles",
                "stance_legs_histogram",
                "watchdog_triggered",
            ],
        )

    def test_missing_segment_rejected(self):
        """执行记录没有 `balance` 段 ⇒ 显式失败（不得静默省略、不得补默认值）。"""
        with self.assertRaises(DeclarationError):
            verify_go2_gait_in_place._balance_segment({"samples": []})

    def test_non_mapping_segment_rejected(self):
        with self.assertRaises(DeclarationError):
            verify_go2_gait_in_place._balance_segment({"balance": [1, 2, 3]})

    def test_every_required_path_is_really_enforced(self):
        """逐条删掉 `BALANCE_REQUIRED_PATHS` 里的键路径都必须被拒（防止声明了却没人检查）。"""
        for dotted in verify_go2_gait_in_place.BALANCE_REQUIRED_PATHS:
            with self.subTest(path=dotted):
                segment = copy.deepcopy(self._produced_segment())
                parts = dotted.split(".")
                node = segment
                for part in parts[:-1]:
                    node = node[part]
                del node[parts[-1]]
                with self.assertRaises(DeclarationError):
                    verify_go2_gait_in_place._balance_segment({"balance": segment})

    def test_fallback_fraction_undefined_without_cycles(self):
        """未跑力控周期时 fallback 比值**不可定义** ⇒ null + 标志位 false（不是 0%）。"""
        segment = self._produced_segment()
        self.assertEqual(int(segment["stats"]["cycles"]), 0)
        stats = verify_go2_gait_in_place._balance_report_summary(segment)["stats"]
        self.assertIsNone(stats["fallback_fraction"])
        self.assertFalse(stats["fallback_fraction_defined"])

    def test_fallback_fraction_is_ratio_of_cycles(self):
        """分母是力控周期数：981/1000 ⇒ 0.981（本轮用它取代序列重建口径的 98.1%）。"""
        segment = copy.deepcopy(self._produced_segment())
        segment["stats"]["cycles"] = 1000
        segment["stats"]["position_weight"]["force_control_cycles"] = 1000
        segment["stats"]["position_weight"]["fallback_cycles"] = 981
        stats = verify_go2_gait_in_place._balance_report_summary(segment)["stats"]
        self.assertTrue(stats["fallback_fraction_defined"])
        self.assertAlmostEqual(stats["fallback_fraction"], 0.981, places=12)


class DeclarationFixtureIsolation(unittest.TestCase):
    def test_load_does_not_mutate_input(self):
        """解析器不得改写传入的声明（否则同一进程内后续读到的数字会被污染）。"""
        document = _declaration()
        snapshot = copy.deepcopy(document)
        balance.load_balance_declaration(document)
        self.assertEqual(document, snapshot)


if __name__ == "__main__":
    unittest.main()
