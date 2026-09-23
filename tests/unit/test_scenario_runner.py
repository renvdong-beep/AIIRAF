"""S2 脚本化场景执行器契约测试（步骤 18）。

被测契约：`scripts/scenario.py`（`list` / `run` 两个子命令 + 五级退出码）与
`scenes/handoff_lab/scenario.yaml`（判据、故障注入项与待交付登记）。

设计要点（fail-closed，每条负向都配正向对照）：
  1. 词表只有一个来源（`config/scene.schema.json`）：执行器不得自带第二份判据/动作/终态清单；
  2. 判据方向必须正确（`min_*` 越大越好、`max_*` 越小越好），且**没有评测依据时必须失败**——
     否则就是"命令已发出 = 成功"的假判据；
  3. 会被真正下发的步骤：判据必须可评测、参数必须符合技能输入契约（否则声明合法但运行期才炸）；
  4. 待交付步骤（有 `pending_closed_by`）显式跳过并进 `pending_steps`，其不可评测判据进登记，
     不判整条场景非法——"未交付"是登记，不是通过；
  5. 能力未声明且无登记 ⇒ 退出码 3；本体未声明 `robot.backend` 绑定 ⇒ 退出码 3（不猜后端入口）；
     未交付的故障类型 ⇒ 退出码 2；
  6. 故障注入：注入点拒绝下发（带登记的错误码、绝不 SUCCEEDED），其后只允许 `safetyAction` 步骤继续；
     注入点不可执行时**登记为未注入**，`--require-injected-faults` 才把它变成硬失败（占位不是通过）。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_scenario_runner -v`

需要真实仿真的用例（`test_real_*`）在生成模型不存在时显式 skip 并说明原因（先跑
`scripts/build_scene.py`）——跳过不是通过，证据由步骤 18 的验收入口落盘。
"""

import contextlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import scenario  # noqa: E402

PACKAGE_DIR = REPO_ROOT / "scenes" / "handoff_lab"
MODEL = REPO_ROOT / "build" / "scenes" / "handoff_lab" / "handoff_lab.xml"
REAL_DECLARATION = REPO_ROOT / "config" / "go2_loopback.yaml"


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def dump_yaml(path, document):
    Path(path).write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


class RunnerFixture(unittest.TestCase):
    """把真实场景包复制到临时目录后按需改坏——真实声明就是正向对照。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.package = Path(self._tmp.name) / "scenes" / "handoff_lab"
        shutil.copytree(PACKAGE_DIR, self.package)
        self.scenario_path = self.package / "scenario.yaml"
        self.scene_path = self.package / "scene.yaml"
        self.baseline_path = self.package / "baseline.yaml"
        self.report_dir = Path(self._tmp.name) / "reports"

    def mutate_scenario(self, mutate):
        document = load_yaml(self.scenario_path)
        mutate(document)
        dump_yaml(self.scenario_path, document)

    def mutate_scene(self, mutate):
        document = load_yaml(self.scene_path)
        mutate(document)
        dump_yaml(self.scene_path, document)

    def run_main(self, *argv):
        """执行入口层并捕获 stdout（否则 JSON 摘要会污染单测输出）。"""
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            code = scenario.main(list(argv))
        return code, captured.getvalue()

    def run_scenario(self, name, *extra):
        report = self.report_dir / ("%s.json" % name)
        code, stdout = self.run_main(
            "run", "--scene", str(self.package), "--scenario", name, "--report", str(report), *extra
        )
        return code, stdout, report

    def error_message(self, *argv):
        captured = io.StringIO()
        with contextlib.redirect_stderr(captured):
            code, _ = self.run_main(*argv)
        return code, captured.getvalue()


class VocabularyTests(unittest.TestCase):
    """词表的唯一来源是场景契约：执行器不得自带第二份。"""

    def test_criteria_vocabulary_comes_from_schema(self):
        contract = scenario.scenario_contract()
        self.assertTrue(contract["criteria"])
        for name in scenario.CRITERION_SPEC:
            self.assertIn(name, contract["criteria"], msg="判据 %s 不在契约词表里" % name)
        self.assertIn(scenario.SAFE_HOLD, contract["terminal_states"])
        self.assertIn("sensor_unavailable", contract["fault_kinds"])

    def test_supported_fault_kinds_subset_of_contract(self):
        contract = scenario.scenario_contract()
        for kind in scenario.SUPPORTED_FAULT_KINDS:
            self.assertIn(kind, contract["fault_kinds"])

    def test_default_report_path_is_scene_and_scenario_namespaced(self):
        path = scenario.default_report_path("handoff_lab", "stand_stop")
        self.assertEqual(
            scenario._rel(path), "build/acceptance/handoff_lab/stand_stop/report.json"
        )


class CriterionEvaluationTests(unittest.TestCase):
    """判据方向与"无依据即失败"是本类交付最容易写反的地方。"""

    def test_min_criterion_requires_at_least(self):
        checks = scenario.evaluate_criteria(
            {"min_stable_hold_s": 3.0}, {"sim_time_advance_s": 3.0}
        )
        self.assertTrue(checks[0]["passed"], msg=checks)
        checks = scenario.evaluate_criteria(
            {"min_stable_hold_s": 3.0}, {"sim_time_advance_s": 2.999}
        )
        self.assertFalse(checks[0]["passed"], msg=checks)

    def test_max_criterion_requires_at_most(self):
        self.assertTrue(
            scenario.evaluate_criteria({"max_speed_m_s": 0.05}, {"final_speed_mps": 0.05})[0]["passed"]
        )
        self.assertFalse(
            scenario.evaluate_criteria({"max_speed_m_s": 0.05}, {"final_speed_mps": 0.06})[0]["passed"]
        )

    def test_timeout_criterion_uses_wall_clock(self):
        self.assertTrue(
            scenario.evaluate_criteria({"timeout_s": 5.0}, {"wall_seconds": 4.9})[0]["passed"]
        )
        self.assertFalse(
            scenario.evaluate_criteria({"timeout_s": 5.0}, {"wall_seconds": 5.1})[0]["passed"]
        )

    def test_missing_basis_fails_closed(self):
        checks = scenario.evaluate_criteria({"max_speed_m_s": 1.0}, {})
        self.assertFalse(checks[0]["passed"])
        self.assertIsNone(checks[0]["measured"])
        self.assertIn("评测依据不可用", checks[0]["detail"])

    def test_measure_step_reports_only_available_measurements(self):
        before = {"time_s": 1.0, "base_linear_velocity_mps": [0.0, 0.0, 0.0]}
        after = {"time_s": 4.5, "base_linear_velocity_mps": [0.003, 0.004, 0.0]}
        measured = scenario.measure_step(before, after, {"duration_ms": 3500}, 0.42)
        self.assertAlmostEqual(measured["sim_time_advance_s"], 3.5)
        self.assertAlmostEqual(measured["final_speed_mps"], 0.005)
        self.assertAlmostEqual(measured["evidence_duration_s"], 3.5)
        self.assertAlmostEqual(measured["wall_seconds"], 0.42)

    def test_measure_step_without_backend_state_has_no_speed_basis(self):
        measured = scenario.measure_step(None, None, {}, 0.1)
        self.assertNotIn("final_speed_mps", measured)
        self.assertNotIn("sim_time_advance_s", measured)

    def test_dock_criteria_take_yaw_magnitude(self):
        # 停靠结果量直接取自技能 evidence；偏航必须取**绝对值**（带符号量会骗过判据）
        evidence = {"final_translation_error_m": 3.6323864375270122e-05,
                    "final_yaw_error_deg": -4.9650, "duration_ms": 3000.0}
        measured = scenario.measure_step({"time_s": 0.0}, {"time_s": 1.0}, evidence, 0.5)
        self.assertAlmostEqual(measured["dock_translation_error_m"], 3.6323864375270122e-05)
        self.assertAlmostEqual(measured["dock_yaw_error_deg"], 4.9650)
        checks = scenario.evaluate_criteria(
            {"translation_error_max_m": 0.03, "yaw_error_max_deg": 2.0}, measured)
        self.assertFalse(checks[1]["passed"], msg="|−4.9650°| > 2.0° ⇒ 必须判失败")
        self.assertTrue(checks[0]["passed"], msg="36 µm ≤ 30 mm ⇒ 通过")

    def test_dock_criteria_without_evidence_fail_explicitly(self):
        # 技能没给出结果量 ⇒ 依据缺失 ⇒ 判失败（绝不静默通过）
        measured = scenario.measure_step({"time_s": 0.0}, {"time_s": 1.0}, {}, 0.5)
        checks = scenario.evaluate_criteria(
            {"translation_error_max_m": 0.03, "yaw_error_max_deg": 2.0}, measured)
        for check in checks:
            self.assertFalse(check["passed"])
            self.assertIsNone(check["measured"])
            self.assertIn("评测依据不可用", check["detail"])


class PlanAndPreflightTests(RunnerFixture):
    """声明分类：待交付跳过、未登记即失败。"""

    def test_pending_step_criteria_are_registered_not_blocking(self):
        contract = scenario.scenario_contract()
        index = scenario.capability_index(load_yaml(self.scene_path))
        entry = load_yaml(self.scenario_path)["scenarios"]["fault_sensor_loss"]
        plan = scenario.plan_steps(entry, index, contract)
        pending = [item for item in plan if item["kind"] == scenario.STEP_SKIPPED_PENDING]
        self.assertEqual([item["id"] for item in pending], ["f02_dock"])
        # 2026-09-24 起 `translation_error_max_m` / `yaw_error_max_deg` **可评测**（测量量取
        # 技能 evidence 的停靠结果量，并在 `measure_step` 里对偏航取绝对值）⇒ 待交付登记里
        # 不再把它们列为"判不了"；该步待交付的原因只剩**能力未声明**。
        self.assertEqual(pending[0]["registration"]["unevaluable_criteria"], [])
        # 待交付步骤不得让整条场景变成"判据无依据"的非法声明。
        scenario.check_evaluable_criteria(plan)

    def test_dispatched_step_with_unevaluable_criteria_fails_preflight(self):
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["stand_stop"]["steps"][0].__setitem__(
                "criteria", {"pose_tolerance_m": 0.005}
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "stand_stop"
        )
        self.assertEqual(code, scenario.EXIT_DECLARATION, msg=message)
        self.assertIn("没有评测依据的判据", message)
        self.assertIn("pose_tolerance_m", message)

    def test_unregistered_capability_fails_with_reference_code(self):
        def drop_registration(doc):
            step = doc["scenarios"]["nominal"]["steps"][1]
            step.pop("pending_closed_by")
            step.pop("pending_reason")

        self.mutate_scenario(drop_registration)
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "nominal"
        )
        self.assertEqual(code, scenario.EXIT_REFERENCE, msg=message)
        self.assertIn("未登记待交付", message)

    def test_params_violating_skill_input_schema_fail_preflight(self):
        # 真实缺陷复现（步骤 18 发现）：stop 的输入 schema 是 additionalProperties:false，
        # 原先声明的 params.reason 会被 Policy 拒；必须在预检阶段就失败，而不是跑一半才炸。
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["stand_stop"]["steps"][1].__setitem__(
                "params", {"reason": "sensor_unavailable"}
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "stand_stop"
        )
        self.assertEqual(code, scenario.EXIT_DECLARATION, msg=message)
        self.assertIn("参数不符合技能", message)


class FaultDeclarationTests(RunnerFixture):
    """故障声明的三类负向 + 一条正向对照。"""

    def test_unsupported_fault_kind_is_explicit(self):
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["fault_sensor_unavailable"]["faults"][0].__setitem__(
                "kind", "estop"
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "fault_sensor_unavailable"
        )
        self.assertEqual(code, scenario.EXIT_DECLARATION, msg=message)
        self.assertIn("尚未交付", message)

    def test_succeeded_terminal_state_with_fake_success_forbidden_is_rejected(self):
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["fault_sensor_unavailable"]["faults"][0]["expect"].__setitem__(
                "terminal_state", "SUCCEEDED"
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "fault_sensor_unavailable"
        )
        self.assertEqual(code, scenario.EXIT_DECLARATION, msg=message)
        self.assertIn("自相矛盾", message)

    def test_fault_target_must_belong_to_step_robot(self):
        # overhead_camera 归属 piper，而注入点是四足步骤 ⇒ 无法判定该故障会阻止本步。
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["fault_sensor_unavailable"]["faults"][0].__setitem__(
                "target", "overhead_camera"
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "fault_sensor_unavailable"
        )
        self.assertEqual(code, scenario.EXIT_REFERENCE, msg=message)
        self.assertIn("归属本体", message)

    def test_unknown_fault_target_is_reference_failure(self):
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["fault_sensor_unavailable"]["faults"][0].__setitem__(
                "target", "no_such_sensor"
            )
        )
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "fault_sensor_unavailable"
        )
        self.assertEqual(code, scenario.EXIT_REFERENCE, msg=message)
        # 目标传感器不存在时先被 scene_check 的引用层拦下（同一退出码 3），原因里必须出现该名字。
        self.assertIn("no_such_sensor", message)


class BindingTests(RunnerFixture):
    """后端绑定必须来自机型声明：缺声明即显式失败，绝不猜入口。"""

    def test_unbound_robot_fails_before_motion(self):
        """缺 robot.backend 的机型声明必须在运动前失败（不猜入口）。

        2026-09-21 更新：此前用 piper 做这个负例，是因为 piper 当时未接入本机制；
        A1 之后 piper 已在 `config/machines/piper.yaml` 声明 `robot.backend: mujoco_arm`，
        因此本用例改为**构造一份缺 robot.backend 的声明**来证伪——意图与判据不变，前提更新。
        """
        document = load_yaml(REAL_DECLARATION)
        document = json.loads(json.dumps(document))
        document["robot"].pop("backend", None)
        tmp = Path(self._tmp.name) / "no-backend.yaml"
        dump_yaml(tmp, document)
        index = {"piper": {"profile": document["robot"]["profile"]}}
        with self.assertRaises(scenario.ScenarioError) as ctx:
            scenario.machine_declaration("piper", index, {"robots": {"piper": str(tmp)}})
        self.assertEqual(ctx.exception.code, scenario.EXIT_REFERENCE)
        self.assertIn("robot.backend", str(ctx.exception))

    def test_declared_binding_matches_registry(self):
        index = scenario.capability_index(load_yaml(self.scene_path))
        binding = scenario.machine_declaration(
            "unitree_go2", index, load_yaml(self.baseline_path)
        )
        self.assertIn(binding["backend"], scenario.KNOWN_BACKENDS)
        self.assertEqual(binding["profile"], "profiles/unitree_go2_mujoco.yaml")
        self.assertEqual(binding["safety_policy"], "profiles/safety/quadruped_lab.yaml")

    def test_profile_reference_mismatch_is_declaration_error(self):
        document = load_yaml(REAL_DECLARATION)
        altered = dict(document)
        altered["robot"] = dict(document["robot"])
        altered["robot"]["profile"] = "profiles/piper_mujoco.yaml"
        tmp = Path(self._tmp.name) / "mismatched.yaml"
        dump_yaml(tmp, altered)
        index = scenario.capability_index(load_yaml(self.scene_path))
        baseline = {"robots": {"unitree_go2": scenario._rel(tmp)}}
        with self.assertRaises(scenario.ScenarioError) as caught:
            scenario.machine_declaration("unitree_go2", index, baseline)
        self.assertEqual(caught.exception.code, scenario.EXIT_DECLARATION)
        self.assertIn("引用不一致", str(caught.exception))

    def test_missing_model_fails_as_reference_failure(self):
        with self.assertRaises(scenario.ScenarioError) as caught:
            scenario.check_model_available(
                {"model": {"file": "build/scenes/never-built/absent.xml"}}, REPO_ROOT
            )
        self.assertEqual(caught.exception.code, scenario.EXIT_REFERENCE)
        # 正向对照：已存在的文件不得被判为缺失（否则这条门禁是恒失败）。
        scenario.check_model_available({"model": {"file": "AGENTS.md"}}, REPO_ROOT)
        # 未声明 model.file 的机型不在本门禁范围（由适配器在装配期失败，不在这里猜）。
        self.assertIsNone(scenario.check_model_available({}, REPO_ROOT))


class CliContractTests(RunnerFixture):
    """退出码契约与 list 的可用性。"""

    def test_list_reports_scenarios_and_bindings(self):
        code, stdout = self.run_main("list")
        self.assertEqual(code, scenario.EXIT_OK)
        payload = json.loads(stdout)
        scenes = {item["id"]: item for item in payload["scenes"]}
        self.assertIn("handoff_lab", scenes)
        robots = {item["id"]: item for item in scenes["handoff_lab"]["robots"]}
        self.assertEqual(robots["unitree_go2"]["runner_backend"], "unitree_go2_mujoco")
        # 2026-09-21（A1）：piper 已在 config/machines/piper.yaml 声明运行时绑定（后端配置取自场景报告）
        self.assertEqual(robots["piper"]["runner_backend"], "mujoco_arm")
        self.assertFalse(robots["piper"]["runner_binding_reason"])
        # 仍未绑定的本体继续给出原因（fail-closed 的另一半证据）
        self.assertIsNone(robots["humanoid_static"]["runner_backend"])
        self.assertTrue(robots["humanoid_static"]["runner_binding_reason"])
        names = {item["name"] for item in scenes["handoff_lab"]["scenarios"]}
        self.assertIn("stand_stop", names)
        self.assertIn("fault_sensor_unavailable", names)
        pending = {
            item["name"]: item["pending_steps"] for item in scenes["handoff_lab"]["scenarios"]
        }
        self.assertEqual(pending["stand_stop"], [])
        self.assertEqual(pending["nominal"], ["s02_dock", "s04_place_in_tray", "s05_confirm_payload"])

    def test_unknown_scenario_is_reference_failure(self):
        code, message = self.error_message(
            "run", "--scene", str(self.package), "--scenario", "nope"
        )
        self.assertEqual(code, scenario.EXIT_REFERENCE)
        self.assertIn("不在", message)

    def test_missing_scene_directory_is_usage_error(self):
        code, message = self.error_message(
            "run", "--scene", str(self.package) + "-absent", "--scenario", "stand_stop"
        )
        self.assertEqual(code, scenario.EXIT_USAGE)
        self.assertIn("场景目录不存在", message)

    def test_missing_arguments_are_usage_error(self):
        captured = io.StringIO()
        with contextlib.redirect_stderr(captured):
            with self.assertRaises(SystemExit) as caught:
                scenario.main(["run", "--scene", str(self.package)])
        self.assertEqual(caught.exception.code, 2)

    def test_no_subcommand_is_usage_error(self):
        captured = io.StringIO()
        with contextlib.redirect_stderr(captured):
            code = scenario.main([])
        self.assertEqual(code, scenario.EXIT_USAGE)


class RealSimulationTests(RunnerFixture):
    """需要真实 MuJoCo 的用例：模型不存在时显式 skip（跳过不是通过）。"""

    def setUp(self):
        super().setUp()
        if not MODEL.is_file():
            self.skipTest(
                "生成模型不存在（%s）：先跑 scripts/build_scene.py --scene scenes/handoff_lab "
                "--robot unitree_go2 才能验证真实执行链（证据由步骤 18 验收入口落盘）" % scenario._rel(MODEL)
            )

    def test_real_stand_stop_passes_with_measured_numbers(self):
        code, stdout, report_path = self.run_scenario("stand_stop")
        summary = json.loads(stdout)
        self.assertEqual(code, scenario.EXIT_OK, msg=json.dumps(summary, ensure_ascii=False))
        self.assertTrue(summary["passed"])
        report = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertTrue(report["simulation"])
        self.assertEqual(report["counts"]["executed"], 2)
        self.assertEqual(report["failed_checks"], [])
        first = report["steps"][0]
        self.assertEqual(first["status"], "SUCCEEDED")
        self.assertGreaterEqual(first["measured"]["sim_time_advance_s"], 3.0)
        self.assertLessEqual(
            first["measured"]["final_speed_mps"], first["criteria"]["max_speed_m_s"]
        )
        self.assertTrue(all(item["passed"] for item in first["checks"]))
        self.assertEqual(report["robots"][0]["backend_entry"], "unitree_go2_mujoco")
        self.assertEqual(report["package"]["scene_check_exit_code"], 0)

    def test_real_tightened_criterion_fails(self):
        """收紧判据必须真的失败：否则说明判据没有接线（恒通过）。"""
        self.mutate_scenario(
            lambda doc: doc["scenarios"]["stand_stop"]["steps"][0]["criteria"].__setitem__(
                "max_speed_m_s", 1.0e-9
            )
        )
        code, stdout, _ = self.run_scenario("stand_stop")
        summary = json.loads(stdout)
        self.assertEqual(code, scenario.EXIT_CRITERIA, msg=json.dumps(summary, ensure_ascii=False))
        self.assertTrue(any("判据 max_speed_m_s 未满足" in item for item in summary["failed_checks"]))

    def test_real_fault_injection_refuses_and_holds(self):
        code, stdout, report_path = self.run_scenario("fault_sensor_unavailable")
        summary = json.loads(stdout)
        self.assertEqual(code, scenario.EXIT_OK, msg=json.dumps(summary, ensure_ascii=False))
        report = json.loads(report_path.read_text(encoding="utf-8"))
        refused = [item for item in report["steps"] if item["kind"] == scenario.STEP_REFUSED_FAULT]
        self.assertEqual([item["id"] for item in refused], ["u01_stand"])
        self.assertEqual(refused[0]["status"], "FAILED")
        self.assertEqual(refused[0]["error_code"], scenario.PRECONDITION_FAILED_CODE)
        self.assertNotEqual(refused[0]["status"], "SUCCEEDED")
        safety = [
            item for item in report["steps"] if item["kind"] == scenario.STEP_SAFETY_ACTION_AFTER_FAULT
        ]
        self.assertEqual([item["id"] for item in safety], ["u02_safe_hold"])
        self.assertEqual(safety[0]["safety_class"], "safety_action")
        fault = report["faults"][0]
        self.assertTrue(fault["injected"])
        self.assertTrue(fault["verified"])
        self.assertFalse(fault["fake_success"])
        self.assertEqual(fault["refused_step"], "u01_stand")
        self.assertEqual(fault["safety_actions_executed"], ["u02_safe_hold"])
        self.assertEqual(fault["scenario_state_after_fault"], scenario.SAFE_HOLD)
        self.assertEqual(report["counts"]["injected_faults"], 1)

    def test_real_uninjectable_fault_is_registered_and_gated(self):
        """注入点在待交付步骤上 ⇒ 登记为未注入；--require-injected-faults 才变成硬失败。"""
        code, stdout, report_path = self.run_scenario("fault_sensor_loss")
        self.assertEqual(code, scenario.EXIT_OK, msg=stdout)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        fault = report["faults"][0]
        self.assertFalse(fault["injected"])
        self.assertFalse(fault["verified"])
        self.assertIn("未执行", fault["not_injected_reason"])
        self.assertEqual(report["unverified_faults"], ["lidar_loss"])
        self.assertEqual(report["counts"]["skipped_pending"], 1)
        code, stdout, _ = self.run_scenario("fault_sensor_loss", "--require-injected-faults")
        self.assertEqual(code, scenario.EXIT_CRITERIA, msg=stdout)
        summary = json.loads(stdout)
        self.assertTrue(
            any("--require-injected-faults" in item for item in summary["failed_checks"])
        )


if __name__ == "__main__":
    unittest.main()
