"""四足**移动语义**边界与停止语义矩阵的单元测试（步骤 01，战役 iraf-24h-2）。

覆盖（每条负向用例都配正向对照，否则分不清"门禁严格"与"门禁恒失败"）：
  1. 移动语义边界 `load_movement_limits`：真实声明可加载（正向对照）；缺
     `max_accel_mps2` / `watchdog_timeout_ms` / `allow_in_place_turn` / 工作空间边界、
     `allow_in_place_turn` 不是布尔、矩形 min ≥ max —— 全部必须被拒；
  2. 停止语义表 `load_stop_modes`：两种语义并列且写明适用路径（正向）；缺 `damped_hold`、
     缺适用路径、出现未登记语义 / 未知键 —— 全部必须被拒；
  3. 速度门禁的移动增量：加速度上限（由声明的 `ramp_s` 推算）、原地转弯开关；
  4. 看门狗门禁 `enforce_state_freshness`：新鲜通过、过期拒绝；
  5. 移动工作空间门禁 `check_workspace_moving`：矩形内通过、越界/倾角超限失败；
  6. 停止语义解析 `resolve_stop_mode`：移动 → `damped_hold`（正向）；**常规 stop 请求
     `torque_zero_release` 必须被拒**（负向）；急停 → 失能停机（正向）、急停请求受控停止被拒（负向）、
     未知模式被拒（负向）、未移动 → 机型声明的站立停机语义；
  7. 机型声明 `locomotion` 段：真实声明可加载且来源路径可解析（正向）；缺键、来源路径解析不到、
     看门狗动作声明成失能停机、斜坡不足以到达最高速度、心跳周期大于看门狗超时 —— 全部必须被拒。

夹具纪律：复制真实声明到临时目录后按需改坏（真实声明就是正向对照）；不读 `build/` 产物、
不依赖本机是否跑过构建。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_movement_limits -v`
"""

import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_skills import quadruped as skills  # noqa: E402

SAFETY_POLICY = ROOT / "profiles" / "safety" / "quadruped_lab.yaml"
LOCOMOTION_DECLARATION = ROOT / "config" / "go2_loopback.yaml"


def _state(x, y, quaternion=(1.0, 0.0, 0.0, 0.0)):
    return {"base_position_m": [x, y, 0.27], "base_quaternion_wxyz": list(quaternion)}


def _pitch_quaternion(degrees):
    half = math.radians(degrees) / 2.0
    return (math.cos(half), 0.0, math.sin(half), 0.0)


class _TempCopyFixture(unittest.TestCase):
    """把真实声明复制到临时目录后改坏：真实声明即正向对照。"""

    SOURCE = None

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / Path(self.SOURCE).name
        shutil.copyfile(self.SOURCE, self.path)

    def load(self):
        return yaml.safe_load(self.path.read_text(encoding="utf-8"))

    def mutate(self, mutate):
        document = self.load()
        mutate(document)
        self.path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
        return document


class MovementLimitsDeclarationTests(_TempCopyFixture):
    SOURCE = SAFETY_POLICY

    def setUp(self):
        super().setUp()
        self.real = skills.load_movement_limits(self.path)

    def test_real_movement_limits_load_with_derived_values(self):
        """正向对照：真实声明必须能加载，且与机型声明的斜坡自洽。"""
        self.assertEqual(
            sorted(
                key
                for key in self.real
                if key in skills.MOVEMENT_NUMERIC_KEYS
            ),
            sorted(skills.MOVEMENT_NUMERIC_KEYS),
        )
        self.assertIs(self.real["allow_in_place_turn"], True)
        self.assertEqual(sorted(self.real["workspace_m"]), sorted(skills.WORKSPACE_RECT_KEYS))
        self.assertLess(self.real["workspace_m"]["x_min"], self.real["workspace_m"]["x_max"])
        self.assertGreater(self.real["watchdog_timeout_ms"], 0.0)
        for mode in skills.REGISTERED_STOP_MODES:
            self.assertIn(mode, self.real["stop_modes"])

    def test_every_movement_key_is_mandatory(self):
        for key in skills.MOVEMENT_NUMERIC_KEYS + ("allow_in_place_turn", "workspace_m"):
            with self.subTest(key=key):
                super().setUp()
                self.mutate(lambda doc, key=key: doc["spec"]["quadruped_limits"].pop(key))
                with self.assertRaises(skills.SkillContractError) as caught:
                    skills.load_movement_limits(self.path)
                self.assertIn(key, str(caught.exception))

    def test_missing_workspace_edge_is_rejected(self):
        self.mutate(lambda doc: doc["spec"]["quadruped_limits"]["workspace_m"].pop("y_min"))
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_movement_limits(self.path)
        self.assertIn("y_min", str(caught.exception))

    def test_self_contradictory_rectangle_is_rejected(self):
        self.mutate(lambda doc: doc["spec"]["quadruped_limits"]["workspace_m"].__setitem__("x_min", 0.6))
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_movement_limits(self.path)
        self.assertIn("自相矛盾", str(caught.exception))

    def test_non_boolean_in_place_turn_is_rejected(self):
        """字符串 'false' 不是 false：布尔语义必须真布尔，禁止 truthy 兜底。"""
        self.mutate(
            lambda doc: doc["spec"]["quadruped_limits"].__setitem__("allow_in_place_turn", "false")
        )
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_movement_limits(self.path)
        self.assertIn("布尔值", str(caught.exception))


class StopModesDeclarationTests(_TempCopyFixture):
    SOURCE = SAFETY_POLICY

    def test_real_stop_modes_load_with_applicability(self):
        """正向对照：两种语义并列，且各自写明适用路径。"""
        modes = skills.load_stop_modes(self.path)
        self.assertEqual(sorted(modes), sorted(skills.REGISTERED_STOP_MODES))
        for mode, entry in modes.items():
            self.assertTrue(entry["applies_to"], mode)
            self.assertTrue(entry["detail"], mode)
        self.assertIn("locomote_stop", modes["damped_hold"]["applies_to"])
        self.assertIn("emergency_stop", modes["torque_zero_release"]["applies_to"])

    def test_missing_damped_hold_is_rejected(self):
        self.mutate(lambda doc: doc["spec"]["stop_modes"].pop("damped_hold"))
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_stop_modes(self.path)
        self.assertIn("damped_hold", str(caught.exception))

    def test_missing_applies_to_is_rejected(self):
        self.mutate(lambda doc: doc["spec"]["stop_modes"]["damped_hold"].pop("applies_to"))
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_stop_modes(self.path)
        self.assertIn("applies_to", str(caught.exception))

    def test_unregistered_mode_is_rejected(self):
        self.mutate(
            lambda doc: doc["spec"]["stop_modes"].__setitem__(
                "gentle_slowdown", {"applies_to": ["x"], "detail": "未登记的语义"}
            )
        )
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_stop_modes(self.path)
        self.assertIn("未登记的停止语义", str(caught.exception))

    def test_missing_stop_modes_section_is_rejected(self):
        self.mutate(lambda doc: doc["spec"].pop("stop_modes"))
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.load_stop_modes(self.path)
        self.assertIn("stop_modes", str(caught.exception))


class VelocityGateMovementTests(unittest.TestCase):
    def setUp(self):
        self.limits = skills.load_quadruped_limits(SAFETY_POLICY)
        self.movement = skills.load_movement_limits(SAFETY_POLICY)
        self.ramp_s = 1.0

    def test_within_accel_and_in_place_turn_passes(self):
        """正向对照：声明允许原地转弯，且速度增量落在加速度上限内。"""
        measured = skills.enforce_velocity_limits(
            {"vx_mps": 0.2, "vy_mps": 0.0, "wz_rad_s": 0.5},
            self.limits,
            movement=self.movement,
            current_velocity={"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.0},
            ramp_s=self.ramp_s,
        )
        self.assertAlmostEqual(measured["accel_mps2"], 0.2, places=12)
        self.assertTrue(measured["in_place_turn"] is False)
        self.assertAlmostEqual(measured["ramp_s"], self.ramp_s, places=12)

    def test_in_place_turn_is_flagged_when_only_yaw_is_commanded(self):
        measured = skills.enforce_velocity_limits(
            {"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.5},
            self.limits,
            movement=self.movement,
            current_velocity={"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.0},
            ramp_s=self.ramp_s,
        )
        self.assertTrue(measured["in_place_turn"])

    def test_accel_over_limit_is_rejected(self):
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.enforce_velocity_limits(
                {"vx_mps": self.limits["max_speed_mps"], "vy_mps": 0.0, "wz_rad_s": 0.0},
                self.limits,
                movement=self.movement,
                current_velocity={"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.0},
                ramp_s=0.1,
            )
        self.assertIn("加速度", str(caught.exception))

    def test_in_place_turn_is_rejected_when_declaration_forbids_it(self):
        movement = dict(self.movement)
        movement["allow_in_place_turn"] = False
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.enforce_velocity_limits(
                {"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.5},
                self.limits,
                movement=movement,
                current_velocity={"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.0},
                ramp_s=self.ramp_s,
            )
        self.assertIn("不允许原地转弯", str(caught.exception))

    def test_movement_gate_requires_explicit_current_velocity_and_ramp(self):
        """缺 current_velocity/ramp_s ⇒ 显式失败：不得假定"从零起步"。"""
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.enforce_velocity_limits(
                {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0},
                self.limits,
                movement=self.movement,
            )
        self.assertIn("current_velocity", str(caught.exception))

    def test_movement_boundary_is_required_when_ramp_is_given(self):
        with self.assertRaises(skills.SkillContractError):
            skills.enforce_velocity_limits(
                {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0},
                self.limits,
                ramp_s=1.0,
            )


class WatchdogGateTests(unittest.TestCase):
    def setUp(self):
        self.movement = skills.load_movement_limits(SAFETY_POLICY)

    def test_fresh_state_passes(self):
        measured = skills.enforce_state_freshness(0.01, self.movement)
        self.assertTrue(measured["fresh"])
        self.assertAlmostEqual(
            measured["watchdog_timeout_s"], self.movement["watchdog_timeout_ms"] / 1000.0, places=12
        )

    def test_stale_state_is_rejected(self):
        stale = self.movement["watchdog_timeout_ms"] / 1000.0 + 1e-6
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.enforce_state_freshness(stale, self.movement)
        self.assertIn("看门狗", str(caught.exception))

    def test_negative_age_is_rejected(self):
        with self.assertRaises(skills.SkillContractError):
            skills.enforce_state_freshness(-0.001, self.movement)


class MovingWorkspaceGateTests(unittest.TestCase):
    def setUp(self):
        self.movement = skills.load_movement_limits(SAFETY_POLICY)

    def test_inside_rectangle_passes(self):
        result = skills.check_workspace_moving(_state(0.0, 0.0), _state(0.2, -0.3), self.movement)
        self.assertTrue(result["passed"], result["checks"])
        self.assertAlmostEqual(result["travel_m"], math.hypot(0.2, 0.3), places=12)

    def test_leaving_rectangle_fails(self):
        high = self.movement["workspace_m"]["x_max"]
        result = skills.check_workspace_moving(_state(0.0, 0.0), _state(high + 0.01, 0.0), self.movement)
        self.assertFalse(result["passed"])
        failed = {item["name"] for item in result["checks"] if not item["passed"]}
        self.assertEqual(failed, {"workspace.x_max"})

    def test_moving_tilt_over_limit_fails(self):
        result = skills.check_workspace_moving(
            _state(0.0, 0.0),
            _state(0.0, 0.0, _pitch_quaternion(self.movement["max_tilt_moving_deg"] + 1.0)),
            self.movement,
        )
        self.assertFalse(result["passed"])
        failed = {item["name"] for item in result["checks"] if not item["passed"]}
        self.assertEqual(failed, {"workspace.tilt_moving_deg"})


class StopModeResolutionTests(unittest.TestCase):
    def setUp(self):
        self.modes = skills.load_stop_modes(SAFETY_POLICY)
        self.standing_mode = yaml.safe_load(
            LOCOMOTION_DECLARATION.read_text(encoding="utf-8")
        )["stop"]["mode"]

    def test_moving_stop_resolves_to_damped_hold(self):
        self.assertEqual(
            skills.resolve_stop_mode(self.modes, True, self.standing_mode), "damped_hold"
        )

    def test_standing_stop_uses_declared_standing_mode(self):
        """站立停机语义来自机型声明（既有验收事实），本步不改变。"""
        self.assertEqual(
            skills.resolve_stop_mode(self.modes, False, self.standing_mode), self.standing_mode
        )

    def test_emergency_resolves_to_torque_zero_release(self):
        self.assertEqual(
            skills.resolve_stop_mode(self.modes, True, self.standing_mode, emergency=True),
            "torque_zero_release",
        )

    def test_routine_stop_requesting_torque_zero_release_is_rejected(self):
        """负向：常规 stop（非急停）不得请求失能停机——语义混用必须被拒。"""
        for moving in (True, False):
            with self.subTest(moving=moving):
                with self.assertRaises(skills.SkillContractError) as caught:
                    skills.resolve_stop_mode(
                        self.modes, moving, self.standing_mode, requested="torque_zero_release"
                    )
                self.assertIn("torque_zero_release", str(caught.exception))
                self.assertIn("急停", str(caught.exception))

    def test_emergency_may_not_be_downgraded_to_damped_hold(self):
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.resolve_stop_mode(
                self.modes, True, self.standing_mode, emergency=True, requested="damped_hold"
            )
        self.assertIn("不得降级", str(caught.exception))

    def test_unknown_requested_mode_is_rejected(self):
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.resolve_stop_mode(
                self.modes, True, self.standing_mode, requested="gentle_slowdown"
            )
        self.assertIn("未知停止模式", str(caught.exception))

    def test_unregistered_standing_mode_is_rejected(self):
        with self.assertRaises(skills.SkillContractError) as caught:
            skills.resolve_stop_mode(self.modes, False, "gentle_slowdown")
        self.assertIn("未在 stop_modes 登记", str(caught.exception))

    def test_incomplete_stop_modes_table_is_rejected(self):
        with self.assertRaises(skills.SkillContractError):
            skills.resolve_stop_mode(
                {"damped_hold": {"applies_to": ["locomote_stop"]}}, True, "damped_hold"
            )


class LocomotionDeclarationTests(_TempCopyFixture):
    SOURCE = LOCOMOTION_DECLARATION

    def setUp(self):
        super().setUp()
        self.movement = skills.load_movement_limits(SAFETY_POLICY)
        self.standing = skills.load_quadruped_limits(SAFETY_POLICY)

    def load_locomotion(self):
        return skills.load_locomotion_declaration(
            self.load(), self.movement, self.standing
        )

    def test_real_locomotion_section_loads_and_sources_resolve(self):
        """正向对照：来源路径必须解析到本声明内的真实值（不是字符串装饰）。"""
        locomotion = self.load_locomotion()
        declaration = self.load()
        self.assertAlmostEqual(
            locomotion["control_frequency_hz"], float(declaration["control"]["frequency_hz"]), places=12
        )
        self.assertEqual(locomotion["watchdog_action"], "damped_hold")
        self.assertEqual(
            locomotion["damped_hold"]["deceleration_source"]["value"], declaration["locomotion"]["ramp_s"]
        )
        self.assertEqual(
            locomotion["damped_hold"]["hold_pose_source"]["value"], declaration["stand"]["pose_source"]
        )

    def test_every_locomotion_key_is_mandatory(self):
        for key in skills.LOCOMOTION_REQUIRED_KEYS:
            with self.subTest(key=key):
                super().setUp()
                self.mutate(lambda doc, key=key: doc["locomotion"].pop(key))
                with self.assertRaises(skills.SkillContractError) as caught:
                    self.load_locomotion()
                self.assertIn(key, str(caught.exception))

    def test_missing_locomotion_section_is_rejected(self):
        self.mutate(lambda doc: doc.pop("locomotion"))
        with self.assertRaises(skills.SkillContractError) as caught:
            self.load_locomotion()
        self.assertIn("locomotion", str(caught.exception))

    def test_unresolvable_source_path_is_rejected(self):
        """来源路径写错 ⇒ 显式失败（禁止静默回退到默认值）。"""
        self.mutate(
            lambda doc: doc["locomotion"].__setitem__("control_frequency_source", "control.missing_key")
        )
        with self.assertRaises(skills.SkillContractError) as caught:
            self.load_locomotion()
        self.assertIn("missing_key", str(caught.exception))

    def test_watchdog_action_must_be_damped_hold(self):
        self.mutate(lambda doc: doc["locomotion"].__setitem__("watchdog_action", "torque_zero_release"))
        with self.assertRaises(skills.SkillContractError) as caught:
            self.load_locomotion()
        self.assertIn("damped_hold", str(caught.exception))

    def test_ramp_too_short_for_max_speed_is_rejected(self):
        """跨文件自洽：ramp_s × max_accel_mps2 必须够到达声明的最高速度。"""
        self.mutate(lambda doc: doc["locomotion"].__setitem__("ramp_s", 0.5))
        with self.assertRaises(skills.SkillContractError) as caught:
            self.load_locomotion()
        self.assertIn("自相矛盾", str(caught.exception))

    def test_heartbeat_longer_than_watchdog_is_rejected(self):
        self.mutate(lambda doc: doc["locomotion"].__setitem__("heartbeat_period_ms", 150.0))
        with self.assertRaises(skills.SkillContractError) as caught:
            self.load_locomotion()
        self.assertIn("heartbeat_period_ms", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
