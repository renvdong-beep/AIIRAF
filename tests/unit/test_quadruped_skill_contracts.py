"""四足技能契约与声明边界门禁的单元测试（步骤 17）。

覆盖（每条负向用例旁边都配正向对照，否则分不清"门禁严格"与"门禁恒失败"）：
  1. Skill 清单契约：三个 Skill 都能被 `SkillRegistry` 加载，且必需字段（输入/输出 schema、
     能力依赖、前置条件、超时、取消、恢复、安全等级、版本）齐备；
  2. 能力声明方向：`requires ⊆ profile.capabilities ⊆ adapter.IMPLEMENTED_CAPABILITIES`；
     `locomote` **不在** profile.capabilities（首期无步态控制器，声明它就是声明做不到的能力）；
  3. Provider ↔ 输出 schema：`StandProvider` 的证据必须满足 stand.output.json（少键必须失败）；
  4. 声明边界门禁：`spec.quadruped_limits` 缺段/缺键/非正数/错 kind 一律显式失败（fail-closed）；
  5. 速度门禁：线速度与偏航角速度超限各自可拒绝，上限内通过（正例对照）；
  6. 工作空间门禁：位移与倾角各一条正反用例；倾角判据必须**不含偏航**（原地转向不算"站不直"）；
  7. `LocomoteProvider`：能力未实现时显式拒绝；若后端"成功返回"必须拒绝伪造成功；
  8. 场景包一致性：`scenes/handoff_lab/scene.yaml` 里 unitree_go2 的能力声明不超出 Profile，
     且 profile 引用已闭合为路径（不存在"一处待交付、一处已交付"的静默缺口）。

夹具纪律：不读 `build/` 产物、不依赖本机是否跑过构建；需要改坏声明时复制到临时目录再改。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_skill_contracts -v`
"""

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from jsonschema import Draft202012Validator  # noqa: E402

from iraf_adapters.unitree import quadruped as quadruped_contract  # noqa: E402
from iraf_adapters.unitree.unitree_go2 import UnitreeGo2Adapter  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_sdk import KNOWN_ERROR_CODES  # noqa: E402
from iraf_skills import quadruped as quadruped_skills  # noqa: E402

SAFETY_POLICY = ROOT / "profiles" / "safety" / "quadruped_lab.yaml"
PROFILE = ROOT / "profiles" / "unitree_go2_mujoco.yaml"
SCENE = ROOT / "scenes" / "handoff_lab" / "scene.yaml"
SKILL_ROOT = ROOT / "skills"

MISSING_KEY_CASES = (
    "max_speed_mps",
    "max_yaw_rate_rad_s",
    "max_base_translation_m",
    "max_tilt_deg",
)


def _state(x, y, quaternion=(1.0, 0.0, 0.0, 0.0)):
    return {"base_position_m": [x, y, 0.27], "base_quaternion_wxyz": list(quaternion)}


def _yaw_quaternion(degrees):
    import math

    half = math.radians(degrees) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _pitch_quaternion(degrees):
    import math

    half = math.radians(degrees) / 2.0
    return (math.cos(half), 0.0, math.sin(half), 0.0)


class DeclarationLimitsFixture(unittest.TestCase):
    """复制真实安全策略到临时目录后按需改坏——真实声明就是正向对照。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "quadruped_lab.yaml"
        shutil.copyfile(SAFETY_POLICY, self.path)

    def mutate(self, mutate):
        document = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        mutate(document)
        self.path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")


class LimitsDeclarationTests(DeclarationLimitsFixture):
    def test_real_declaration_loads(self):
        limits = quadruped_skills.load_quadruped_limits(self.path)
        self.assertEqual(sorted(limits), sorted(MISSING_KEY_CASES))
        for value in limits.values():
            self.assertGreater(value, 0.0)

    def test_missing_section_fails_closed(self):
        self.mutate(lambda doc: doc["spec"].pop("quadruped_limits"))
        with self.assertRaises(quadruped_skills.SkillContractError) as caught:
            quadruped_skills.load_quadruped_limits(self.path)
        self.assertIn("quadruped_limits", str(caught.exception))

    def test_every_required_key_is_mandatory(self):
        for key in MISSING_KEY_CASES:
            with self.subTest(key=key):
                self.setUp()  # 每个子用例从干净副本开始
                self.mutate(lambda doc, key=key: doc["spec"]["quadruped_limits"].pop(key))
                with self.assertRaises(quadruped_skills.SkillContractError) as caught:
                    quadruped_skills.load_quadruped_limits(self.path)
                self.assertIn(key, str(caught.exception))

    def test_non_positive_limit_is_rejected(self):
        self.mutate(lambda doc: doc["spec"]["quadruped_limits"].__setitem__("max_speed_mps", 0.0))
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.load_quadruped_limits(self.path)

    def test_wrong_kind_is_rejected(self):
        self.mutate(lambda doc: doc.__setitem__("kind", "RobotProfile"))
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.load_quadruped_limits(self.path)

    def test_missing_file_is_rejected(self):
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.load_quadruped_limits(Path(self._tmp.name) / "absent.yaml")


class VelocityGateTests(unittest.TestCase):
    def setUp(self):
        self.limits = quadruped_skills.load_quadruped_limits(SAFETY_POLICY)

    def test_within_limits_passes(self):
        measured = quadruped_skills.enforce_velocity_limits(
            {"vx_mps": 0.2, "vy_mps": 0.1, "wz_rad_s": 0.5}, self.limits
        )
        self.assertAlmostEqual(measured["speed_mps"], (0.2 ** 2 + 0.1 ** 2) ** 0.5, places=12)
        self.assertAlmostEqual(measured["yaw_rate_rad_s"], 0.5, places=12)

    def test_speed_over_limit_is_rejected_with_registered_code(self):
        with self.assertRaises(quadruped_skills.SkillContractError) as caught:
            quadruped_skills.enforce_velocity_limits(
                {"vx_mps": self.limits["max_speed_mps"] + 0.01, "vy_mps": 0.0, "wz_rad_s": 0.0},
                self.limits,
            )
        self.assertEqual(caught.exception.code, quadruped_skills.COMMAND_REJECTED_CODE)
        self.assertIn(quadruped_skills.COMMAND_REJECTED_CODE, KNOWN_ERROR_CODES)

    def test_yaw_rate_over_limit_is_rejected(self):
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.enforce_velocity_limits(
                {"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": self.limits["max_yaw_rate_rad_s"] + 0.01},
                self.limits,
            )

    def test_negative_direction_is_measured_by_magnitude(self):
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.enforce_velocity_limits(
                {"vx_mps": -(self.limits["max_speed_mps"] + 0.01), "vy_mps": 0.0, "wz_rad_s": 0.0},
                self.limits,
            )


class WorkspaceGateTests(unittest.TestCase):
    def setUp(self):
        self.limits = quadruped_skills.load_quadruped_limits(SAFETY_POLICY)

    def test_small_motion_passes(self):
        result = quadruped_skills.check_workspace(_state(0.0, 0.0), _state(0.004, 0.003), self.limits)
        self.assertTrue(result["passed"])
        self.assertLess(result["checks"][0]["measured"], self.limits["max_base_translation_m"])

    def test_translation_over_limit_fails(self):
        limit = self.limits["max_base_translation_m"]
        result = quadruped_skills.check_workspace(_state(0.0, 0.0), _state(limit + 0.001, 0.0), self.limits)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"][0]["passed"])
        self.assertTrue(result["checks"][1]["passed"])

    def test_tilt_over_limit_fails(self):
        result = quadruped_skills.check_workspace(
            _state(0.0, 0.0), _state(0.0, 0.0, _pitch_quaternion(self.limits["max_tilt_deg"] + 2.0)), self.limits
        )
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"][1]["passed"])
        self.assertTrue(result["checks"][0]["passed"])

    def test_yaw_is_not_counted_as_tilt(self):
        """原地转向不得被判成"站不直"（步骤 15 的实测教训：姿态判据要取相对竖直的倾斜角）。"""
        result = quadruped_skills.check_workspace(
            _state(0.0, 0.0), _state(0.0, 0.0, _yaw_quaternion(80.0)), self.limits
        )
        self.assertAlmostEqual(result["tilt_deg"], 0.0, places=9)
        self.assertTrue(result["passed"])


class ProviderContractTests(unittest.TestCase):
    def test_stand_provider_evidence_matches_output_schema(self):
        schema = json.loads((SKILL_ROOT / "stand" / "stand.output.json").read_text(encoding="utf-8"))
        report = {
            "simulation": True,
            "capability": "stand",
            "execution_id": "exec-0001",
            "fencing_token": 1,
            "duration_ms": 8000.0,
            "control_cycles": 800,
            "substeps_per_control": 5,
            "ctrl_saturated_samples": 0,
            "target_source": "profile_home",
            "torque_limit_source": "model",
            "gravity_feedforward": True,
            "joint_targets_rad": {"FL_hip_joint": 0.0},
            "final_state": {"simulation": True, "time_s": 8.0, "base_position_m": [0.0, 0.0, 0.28],
                            "base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0], "joint_positions_rad": {}},
        }
        output = quadruped_skills.StandProvider(profile=None, backend=_FakeBackend(report)).execute({}, lease=None)
        Draft202012Validator(schema).validate(output)
        # 负向对照：少一个必需键必须失败，否则"schema 通过"没有意义。
        broken = json.loads(json.dumps(output))
        broken["evidence"].pop("duration_ms")
        self.assertTrue(sorted(Draft202012Validator(schema).iter_errors(broken)))

    def test_stand_provider_forwards_declaration_inputs(self):
        backend = _FakeBackend(_stand_report())
        quadruped_skills.StandProvider(profile=None, backend=backend).execute(
            {"joint_targets": {"FL_hip_joint": 0.1}, "duration_ms": 1500}, lease=None
        )
        self.assertEqual(backend.calls[0]["targets"], {"FL_hip_joint": 0.1})
        self.assertEqual(backend.calls[0]["duration_ms"], 1500)

    def test_stand_provider_rejects_report_missing_required_key(self):
        report = _stand_report()
        report.pop("target_source")
        with self.assertRaises(quadruped_skills.SkillContractError) as caught:
            quadruped_skills.StandProvider(profile=None, backend=_FakeBackend(report)).execute({}, lease=None)
        self.assertIn("target_source", str(caught.exception))

    def test_locomote_provider_refuses_when_capability_not_implemented(self):
        backend = _FakeBackend()
        with self.assertRaises(quadruped_contract.UnsupportedCapabilityError):
            quadruped_skills.LocomoteProvider(profile=None, backend=backend).execute(
                {"velocity": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}}, lease=None
            )
        self.assertEqual(backend.locomote_calls, 1)

    def test_locomote_provider_refuses_a_report_without_gait_evidence(self):
        """后端"成功返回"但**缺步态证据** ⇒ 必须拒绝（铁律 1.5 禁止伪造成功）。

        ⚠ 语义变更（2026-09-23）：locomote 实现后本 Provider 改为**消费报告**，
        拒绝的判据从"只要返回就拒"变为"**缺输出 schema 必需键就拒**"（`_evidence` 显式失败，
        不静默丢字段）—— 旧的"返回即契约破裂"守卫会把**成功判成失败**，已删除。
        """
        backend = _FakeBackend(locomote_returns={"capability": "locomote"})
        with self.assertRaises(quadruped_skills.SkillContractError) as caught:
            quadruped_skills.LocomoteProvider(profile=None, backend=backend).execute(
                {"velocity": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}}, lease=None
            )
        self.assertIn("缺少输出必需键", str(caught.exception))

    def test_locomote_provider_consumes_a_complete_report(self):
        """完整报告 ⇒ 输出满足 locomote 输出 schema；且**时长单位**必须是毫秒透传。

        回归依据（2026-09-23 实测）：Provider 曾先调 `resolve_duration_ms`（返回**秒**）
        再把它当**毫秒**传下去 ⇒ `duration_ms=2000` 变成 2 ms ⇒「控制周期数不足 1」。
        """
        report = {"simulation": True, "capability": "locomote", "duration_ms": 2000.0,
                  "command": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}}
        backend = _FakeBackend(locomote_returns=report)
        out = quadruped_skills.LocomoteProvider(profile=None, backend=backend).execute(
            {"velocity": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0},
             "duration_ms": 2000}, lease=None)
        self.assertEqual(out["skill"], "locomote")
        self.assertTrue(out["accepted"])
        self.assertEqual(out["evidence"]["simulation"], True)
        self.assertEqual(out["evidence"]["capability"], "locomote")
        self.assertEqual(out["evidence"]["duration_ms"], 2000.0)
        self.assertEqual(out["evidence"]["velocity"],
                         {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0})
        self.assertEqual(backend.locomote_args[1], 2000.0,
                         "Provider 必须把毫秒透传给适配器（不得自行换算）")

    def test_locomote_provider_rejects_unknown_velocity_field(self):
        backend = _FakeBackend()
        with self.assertRaises(quadruped_contract.CommandRejectedError):
            quadruped_skills.LocomoteProvider(profile=None, backend=backend).execute(
                {"velocity": {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0, "dds_domain": 0}}, lease=None
            )


class SkillManifestTests(unittest.TestCase):
    def setUp(self):
        self.registry = SkillRegistry().load_directory(SKILL_ROOT)
        self.profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))

    def test_three_quadruped_skills_are_loadable_with_full_contract(self):
        for name in ("stand", "stop", "locomote"):
            with self.subTest(skill=name):
                skill = self.registry.resolve(name, "1.0.0")
                self.assertIsNotNone(skill, "Skill 未登记: " + name)
                manifest = skill.manifest
                self.assertTrue(manifest.input_schema)
                self.assertTrue(manifest.output_schema)
                self.assertTrue(manifest.requires)
                self.assertTrue(manifest.recovery)
                self.assertTrue(manifest.intent_examples)
                self.assertTrue(manifest.safety_class)
                self.assertGreater(manifest.timeout_seconds, 0)
                self.assertTrue(manifest.cancellation)
                self.assertRegex(manifest.version, r"^\d+\.\d+\.\d+$")

    def test_capability_direction_is_declared_subset_of_implemented(self):
        """方向门禁：`declared ⊆ implemented`。

        `locomote` 于 2026-09-23 **两层达标后**才入列（适配器层四工况两向误差 0.0659/0.0571 ≤0.10、
        技能层 stand/locomote/stop 三步全 SUCCEEDED；证据 build/acceptance/go2-locomote/report.json
        与 build/iraf-a6a12/skill-layer-run.json）。旧守卫 `assertNotIn("locomote", declared)` 的意图
        （"不得声明做不到的能力"）现由下面的 subset 断言接管。
        """
        declared = {str(item) for item in (self.profile["spec"].get("capabilities") or [])}
        # 2026-09-24：`dock_for_handoff` 在适配器层 9/9 + 技能层 10/10 后入列
        self.assertEqual(declared, {"stand", "stop", "locomote", "dock_for_handoff"})
        implemented = {str(item) for item in UnitreeGo2Adapter.IMPLEMENTED_CAPABILITIES}
        self.assertTrue(declared <= implemented, "声明了未实现的能力: %s" % sorted(declared - implemented))

    def test_each_skill_requires_a_declared_capability(self):
        declared = {str(item) for item in (self.profile["spec"].get("capabilities") or [])}
        for name in ("stand", "stop"):
            manifest = self.registry.resolve(name, "1.0.0").manifest
            self.assertTrue(set(manifest.requires) <= declared, "%s 依赖未声明的能力" % name)
        # `locomote` 已声明 ⇒ 旧的"未声明能力"负向载体失效；换成**真实存在且仍会拒绝**的那条：
        # 超过安全策略上限的速度指令必须被拒（同一门禁家族，行为与代码都没变）。
        locomote = self.registry.resolve("locomote", "1.0.0").manifest
        self.assertTrue(set(locomote.requires) <= declared,
                        "locomote 已声明，其 requires 必须落在 declared 内")
        with self.assertRaises(quadruped_skills.SkillContractError):
            quadruped_skills.enforce_velocity_limits(
                {"vx_mps": 99.0, "vy_mps": 0.0, "wz_rad_s": 0.0},
                {"max_speed_mps": 0.5, "max_yaw_rate_rad_s": 1.0},
            )

    def test_scene_declaration_matches_profile_and_is_closed(self):
        scene = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
        entry = next(item for item in scene["robots"] if item["id"] == "unitree_go2")
        self.assertIsInstance(entry["profile"], str, "Go2 的 profile 引用应已闭合为路径")
        declared = {str(item) for item in (self.profile["spec"].get("capabilities") or [])}
        self.assertTrue(
            {str(item) for item in entry["capabilities"]} <= declared,
            "场景声明超出 Profile（铁律 1.3）",
        )


class _FakeBackend:
    """替身后端：只证明 Provider 的翻译与守卫逻辑，不代表仿真器行为。"""

    def __init__(self, report=None, locomote_returns=None):
        self.report = report or {}
        self.locomote_returns = locomote_returns
        self.calls = []
        self.locomote_calls = 0
        self.locomote_args = None

    def stand(self, lease, targets=None, duration_ms=None, execution_id=None):
        self.calls.append({"targets": targets, "duration_ms": duration_ms, "lease": lease})
        return self.report

    def resolve_velocity(self, velocity):
        return quadruped_contract.QuadrupedAdapter.resolve_velocity(self, velocity)

    def resolve_duration_ms(self, duration_ms, declared_seconds):
        return quadruped_contract.QuadrupedAdapter.resolve_duration_ms(duration_ms, declared_seconds)

    def locomote(self, velocity, duration_ms, lease, execution_id=None):
        self.locomote_calls += 1
        self.locomote_args = (velocity, duration_ms)
        if self.locomote_returns is None:
            raise quadruped_contract.UnsupportedCapabilityError(
                "能力 locomote 未在本后端实现（替身）"
            )
        return self.locomote_returns


def _stand_report():
    return {
        "simulation": True,
        "capability": "stand",
        "execution_id": "exec-0001",
        "fencing_token": 1,
        "duration_ms": 8000.0,
        "control_cycles": 800,
        "substeps_per_control": 5,
        "ctrl_saturated_samples": 0,
        "target_source": "profile_home",
        "torque_limit_source": "model",
        "gravity_feedforward": True,
        "joint_targets_rad": {"FL_hip_joint": 0.0},
        "final_state": {"simulation": True, "time_s": 8.0, "base_position_m": [0.0, 0.0, 0.28],
                        "base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0], "joint_positions_rad": {}},
    }


if __name__ == "__main__":
    unittest.main()
