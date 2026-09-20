"""场景包契约与引用完整性测试（步骤 11）。

被测契约：`config/scene.schema.json`（根结构 = scene.yaml / `definitions.scene_baseline` /
`definitions.scenario_catalog`）与 `scripts/scene_check.py`（三层校验 + 退出码）。

设计要点（fail-closed，每条负向都配正向对照）：
  1. 场景声明缺字段/未知字段/模糊占位一律契约层失败（退出码 2）；
  2. 引用的本体 profile、机型基线、道具 mesh 必须真实存在，缺即引用层失败（退出码 1）；
  3. `baseline.robots` / `initial_state` 的键集合必须与 `scene.robots[].id` 完全一致——
     少写一个本体就是静默缺口；
  4. 未交付的引用只能显式登记为 `{state: unverified, closed_by, reason}`，进 `pending_refs`；
     `--require-resolved-refs` 把它变成硬失败（退出码 5），占位不是"通过"；
  5. 静态模型本体（人形，决策 4.B）不得出现在任何 Skill 步骤或运动能力声明里；
  6. 模型层校验（传感器锚点 / 道具 body 是否真的在生成模型里）只在有模型时执行；
     `--require-model` 缺模型即失败（退出码 4），不能假装校验过。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_scene_schema -v`
"""

import copy
import contextlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from jsonschema import Draft7Validator

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import scene_check  # noqa: E402

SCHEMA_PATH = REPO_ROOT / "config" / "scene.schema.json"
PACKAGE_DIR = REPO_ROOT / "scenes" / "handoff_lab"
CONTRACT_KEYS = ("scene_baseline", "scenario_catalog")


def load_schema():
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def dump_yaml(path, document):
    Path(path).write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


class SchemaContractTests(unittest.TestCase):
    """先证明契约本身是合法 draft-07，避免"恒失败/恒通过"的假门禁。"""

    def test_schema_documents_are_valid_draft7(self):
        schema = load_schema()
        Draft7Validator.check_schema(schema)
        for key in CONTRACT_KEYS:
            sub = copy.deepcopy(schema["definitions"][key])
            sub["$schema"] = schema["$schema"]
            Draft7Validator.check_schema(sub)

    def test_sub_contracts_are_self_contained(self):
        """子契约内部只用相对 $ref，且辅助定义嵌套在自己名下（可整体提取后单独校验）。"""
        schema = load_schema()
        for key in CONTRACT_KEYS:
            sub = schema["definitions"][key]
            refs = _collect_refs(sub)
            self.assertTrue(refs, msg="%s 没有任何 $ref，子契约可能被写空了" % key)
            for ref in refs:
                self.assertTrue(
                    ref.startswith("#/definitions/"),
                    msg="%s 使用了非相对引用 %s（整体提取后会解析失败）" % (key, ref),
                )
                name = ref.split("/")[-1]
                self.assertIn(
                    name,
                    sub.get("definitions", {}),
                    msg="%s 的 %s 未嵌套在自己的 definitions 下" % (key, name),
                )

    def test_scene_contract_requires_simulation_true(self):
        """simulation 必须是常量 true：缺失或 false 都不合法（AGENTS.md 1.7）。"""
        schema = load_schema()
        self.assertEqual(schema["properties"]["simulation"]["const"], True)
        self.assertIn("simulation", schema["required"])


def _collect_refs(node, found=None):
    found = [] if found is None else found
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "$ref" and isinstance(value, str):
                found.append(value)
            else:
                _collect_refs(value, found)
    elif isinstance(node, list):
        for item in node:
            _collect_refs(item, found)
    return found


class ScenePackageFixture(unittest.TestCase):
    """把真实场景包复制到临时目录后按需改坏——真实声明就是正向对照。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.package = Path(self._tmp.name) / "scenes" / "handoff_lab"
        shutil.copytree(PACKAGE_DIR, self.package)
        self.scene_path = self.package / "scene.yaml"
        self.baseline_path = self.package / "baseline.yaml"
        self.scenario_path = self.package / "scenario.yaml"

    def run_check(self, **kwargs):
        return scene_check.check(self.package, **kwargs)

    def mutate(self, path, mutate):
        document = load_yaml(path)
        mutate(document)
        dump_yaml(path, document)


class PositiveControlTests(ScenePackageFixture):
    def test_real_package_passes(self):
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, 0, msg=json.dumps(report, ensure_ascii=False, indent=2))
        self.assertTrue(report["passed"])
        self.assertEqual(report["simulation"], True)
        self.assertEqual(report["id"], "handoff_lab")

    def test_pending_gaps_are_declared_not_silent(self):
        """未交付的引用必须出现在报告里：孔洞是显式登记的，不是被跳过。"""
        report, _ = self.run_check()
        refs = {item["ref"] for item in report["pending_refs"]}
        self.assertIn("robots.go2.profile", refs)
        self.assertIn("robots.humanoid_static.profile", refs)
        self.assertIn("baseline.robots.go2", refs)
        self.assertIn("baseline.initial_state.go2", refs)
        for item in report["pending_refs"]:
            self.assertTrue(item["closed_by"], msg=item)
            self.assertTrue(item["reason"], msg=item)
        steps = {(item["scenario"], item["step"]) for item in report["pending_steps"]}
        self.assertIn(("nominal", "s01_verify_ready"), steps)
        self.assertIn(("nominal", "s04_place_in_tray"), steps)

    def test_model_layer_is_pending_not_checked_when_model_absent(self):
        report, _ = self.run_check()
        self.assertEqual(report["model_check"]["status"], "pending_generation")
        self.assertTrue(report["model_check"]["reason"])


class NegativeContractTests(ScenePackageFixture):
    def test_missing_simulation_is_rejected(self):
        self.mutate(self.scene_path, lambda doc: doc.pop("simulation"))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("simulation" in item for item in report["schema_failures"]))

    def test_empty_lights_is_rejected(self):
        self.mutate(self.scene_path, lambda doc: doc.__setitem__("lights", []))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("lights" in item for item in report["schema_failures"]))

    def test_unknown_field_is_rejected(self):
        self.mutate(self.scene_path, lambda doc: doc.__setitem__("magic_offset_m", 0.01))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("magic_offset_m" in item for item in report["schema_failures"]))

    def test_missing_readme_is_rejected(self):
        (self.package / "README.md").unlink()
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("README.md" in item for item in report["schema_failures"]))

    def test_vague_placeholder_is_rejected(self):
        """未知只能写结构化的 unverified 占位：tbd/unknown 之类的模糊字符串不是路径。"""

        def mutate(doc):
            doc["model"]["builder"] = "tbd"

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("builder" in item for item in report["schema_failures"]))

    def test_placeholder_without_reason_is_rejected(self):
        """占位缺 reason 等于没有登记（说不清为什么未交付）。"""

        def mutate(doc):
            doc["model"]["builder"] = {"state": "unverified", "closed_by": "步骤 13"}

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)
        self.assertTrue(any("builder" in item for item in report["schema_failures"]))


class NegativeReferenceTests(ScenePackageFixture):
    def test_missing_robot_profile_is_rejected(self):
        def mutate(doc):
            doc["robots"][0]["profile"] = "profiles/does_not_exist_mujoco.yaml"

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("profile 不存在" in item for item in report["reference_failures"]))

    def test_profile_must_be_a_robot_profile(self):
        def mutate(doc):
            doc["robots"][0]["profile"] = "config/piper_simulation_baseline.yaml"

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("不是 RobotProfile" in item for item in report["reference_failures"]))

    def test_capability_beyond_profile_is_rejected(self):
        """声明不得超出 profile：场景里写 pick_object 而 profile 没有，即失败（铁律 1.3）。"""

        def mutate(doc):
            doc["robots"][0]["capabilities"] = ["move_joint", "teleport"]

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("teleport" in item for item in report["reference_failures"]))

    def test_baseline_robot_keys_must_match_scene(self):
        self.mutate(self.baseline_path, lambda doc: doc["robots"].pop("humanoid_static"))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(
            any("baseline.robots" in item and "不一致" in item for item in report["reference_failures"])
        )

    def test_placeholder_state_must_agree_across_files(self):
        """一处待交付、一处已交付：两处声明不一致，必须失败。"""

        def mutate(doc):
            doc["robots"]["go2"] = "config/piper_simulation_baseline.yaml"

        self.mutate(self.baseline_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("两处声明不一致" in item for item in report["reference_failures"]))

    def test_missing_mesh_file_is_rejected(self):
        def mutate(doc):
            doc["props"][0]["geometry"] = {"type": "mesh", "mesh": "vendor/nope/box.STL"}

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("mesh 不存在" in item for item in report["reference_failures"]))

    def test_duplicate_entity_id_is_rejected(self):
        def mutate(doc):
            doc["props"][1]["id"] = doc["props"][0]["id"]

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("重复 id" in item for item in report["reference_failures"]))

    def test_sensor_anchor_entity_must_exist(self):
        def mutate(doc):
            doc["sensors"][1]["anchor"]["entity"] = "ghost_robot"

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(
            any("anchor.entity 引用的实体不存在" in item for item in report["reference_failures"])
        )

    def test_scene_ref_must_point_to_own_package(self):
        self.mutate(self.baseline_path, lambda doc: doc.__setitem__("scene", "other/scene.yaml"))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("不是本场景包的 scene.yaml" in item for item in report["reference_failures"]))

    def test_scene_id_must_equal_directory_name(self):
        self.mutate(self.scene_path, lambda doc: doc.__setitem__("id", "somewhere_else"))
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("与目录名" in item for item in report["reference_failures"]))


class NegativeScenarioTests(ScenePackageFixture):
    def test_unknown_robot_in_step_is_rejected(self):
        def mutate(doc):
            doc["scenarios"]["nominal"]["steps"][0]["robot"] = "ghost_robot"

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("robot 引用的本体不存在" in item for item in report["reference_failures"]))

    def test_unregistered_capability_gap_is_rejected(self):
        """用未声明能力却不登记待交付 = 静默声明了不存在的能力，必须失败。"""

        def mutate(doc):
            doc["scenarios"]["nominal"]["steps"][3].pop("pending_closed_by")
            doc["scenarios"]["nominal"]["steps"][3].pop("pending_reason")

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(
            any("未声明具备的能力" in item for item in report["reference_failures"])
        )

    def test_static_model_entity_cannot_run_skills(self):
        """决策 4.B：仅模型实体不得出现在任何 Skill 步骤里。"""

        def mutate(doc):
            doc["scenarios"]["nominal"]["steps"][1]["robot"] = "humanoid_static"

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(
            any("禁止给仅模型实体注册运动/操作能力" in item for item in report["reference_failures"])
        )

    def test_humanoid_cannot_claim_motion_capability(self):
        """人形声明运动能力：契约层直接拒绝（schema 的 model_only 分支）。"""

        def mutate(doc):
            doc["robots"][2]["capabilities"] = ["stand", "locomote"]

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)

    def test_fault_step_reference_is_checked(self):
        def mutate(doc):
            doc["scenarios"]["fault_sensor_loss"]["faults"][0]["at_step"] = "f99_nowhere"

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("at_step" in item for item in report["reference_failures"]))

    def test_fault_target_must_exist(self):
        def mutate(doc):
            doc["scenarios"]["fault_sensor_loss"]["faults"][0]["target"] = "ghost_sensor"

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("target 引用的对象不存在" in item for item in report["reference_failures"]))

    def test_fake_success_forbidden_is_const_true(self):
        def mutate(doc):
            doc["scenarios"]["fault_sensor_loss"]["faults"][0]["expect"][
                "fake_success_forbidden"
            ] = False

        self.mutate(self.scenario_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_SCHEMA)


class ModelLayerTests(ScenePackageFixture):
    def write_model(self, drop=None):
        """写一份含 include 的最小生成模型：证明锚点校验真的在看生成后的模型。"""
        child = self.package / "vendor_part.xml"
        child.write_text(
            "<?xml version='1.0'?>\n<mujoco>\n"
            '  <site name="imu" pos="0 0 0.05"/>\n'
            '  <body name="box_01"/>\n'
            '  <body name="tray_01"/>\n'
            "</mujoco>\n",
            encoding="utf-8",
        )
        parent = self.package / "generated.xml"
        pieces = [
            "<?xml version='1.0'?>\n<mujoco>\n",
            '  <include file="vendor_part.xml"/>\n',
        ]
        if drop != "camera":
            pieces.append('  <camera name="overhead_camera" pos="0 0 1"/>\n')
        if drop != "site":
            pieces.append('  <site name="payload_lidar_site" pos="0 0 0.12"/>\n')
        pieces.append("</mujoco>\n")
        parent.write_text("".join(pieces), encoding="utf-8")
        return parent

    def test_model_layer_passes_when_anchors_exist(self):
        model = self.write_model()
        report, exit_code = self.run_check(model_path=model, require_model=True)
        self.assertEqual(exit_code, 0, msg=json.dumps(report, ensure_ascii=False, indent=2))
        self.assertEqual(report["model_check"]["status"], "checked")
        anchors = {item["name"]: item["present"] for item in report["model_check"]["anchors"]}
        self.assertEqual(anchors["imu"], True)
        self.assertEqual(anchors["overhead_camera"], True)
        self.assertEqual(anchors["payload_lidar_site"], True)

    def test_missing_camera_anchor_is_rejected(self):
        model = self.write_model(drop="camera")
        report, exit_code = self.run_check(model_path=model, require_model=True)
        self.assertEqual(exit_code, scene_check.EXIT_MODEL)
        self.assertTrue(any("overhead_camera" in item for item in report["model_failures"]))

    def test_missing_site_anchor_is_rejected(self):
        model = self.write_model(drop="site")
        report, exit_code = self.run_check(model_path=model, require_model=True)
        self.assertEqual(exit_code, scene_check.EXIT_MODEL)
        self.assertTrue(any("payload_lidar_site" in item for item in report["model_failures"]))

    def test_missing_prop_body_is_rejected(self):
        """道具 body 不在生成模型里 = 抓取判据命中不到目标（body 在 include 进来的子文件里）。"""
        model = self.write_model()
        child = self.package / "vendor_part.xml"
        child.write_text(
            child.read_text(encoding="utf-8").replace('<body name="tray_01"/>', ""),
            encoding="utf-8",
        )
        report, exit_code = self.run_check(model_path=model, require_model=True)
        self.assertEqual(exit_code, scene_check.EXIT_MODEL)
        self.assertTrue(any("tray_01" in item for item in report["model_failures"]))

    def test_missing_builder_path_is_rejected(self):
        """构建器写成路径时必须是真入口；不存在的路径不得当作"已交付"。"""

        def mutate(doc):
            doc["model"]["builder"] = "scripts/does_not_exist_builder.py"

        self.mutate(self.scene_path, mutate)
        report, exit_code = self.run_check()
        self.assertEqual(exit_code, scene_check.EXIT_REFERENCE)
        self.assertTrue(any("model.builder" in item for item in report["reference_failures"]))

    def test_builder_placeholder_is_reported(self):
        """占位必须出现在 pending_refs 里，不能只在 YAML 里躺着。"""
        report, _ = self.run_check()
        self.assertIn("model.builder", {item["ref"] for item in report["pending_refs"]})

    def test_require_model_without_model_fails(self):
        report, exit_code = self.run_check(require_model=True)
        self.assertEqual(exit_code, scene_check.EXIT_MODEL_REQUIRED)
        self.assertTrue(any("--require-model" in item for item in report["gate_failures"]))

    def test_require_resolved_refs_rejects_pending(self):
        """占位不是通过：严格模式下同一份声明必须失败（证明门禁不是恒绿）。"""
        report, exit_code = self.run_check(require_resolved_refs=True)
        self.assertEqual(exit_code, scene_check.EXIT_PENDING)
        self.assertTrue(report["pending_refs"])
        self.assertTrue(report["gate_failures"])


class ClosedPackagePositiveControlTests(unittest.TestCase):
    """正向对照：所有引用都已交付的场景包在严格模式下必须通过。

    没有这条对照就无法区分"门禁严格"与"门禁恒失败"——真实场景包仍有待交付项，
    严格模式在它上面永远失败，因此必须另建一份"已闭合"的包来证明门禁可通过。
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.package = Path(self._tmp.name) / "scenes" / "closed_lab"
        self.package.mkdir(parents=True)
        scene = {
            "schema_version": "iraf.scene/v1",
            "id": "closed_lab",
            "description": "正向对照：所有引用都已交付的场景包。",
            "simulation": True,
            "seed": {"base": 1, "randomization": False},
            "terrain": {
                "kind": "flat",
                "friction": "1.0 0.02 0.001",
                "workbench": {
                    "top_z_m": 0.0,
                    "half_size_m": 0.8,
                    "half_thickness_m": 0.025,
                    "friction": "1.0 0.02 0.001",
                },
            },
            "robots": [
                {
                    "id": "piper",
                    "kind": "arm",
                    "role": "manipulator",
                    "profile": "profiles/piper_mujoco.yaml",
                    "capabilities": ["move_joint", "pick_object", "visual_pick", "stop"],
                }
            ],
            "props": [
                {
                    "id": "box_01",
                    "body": "box_01",
                    "kind": "box",
                    "geometry": {"type": "box", "size_m": [0.025, 0.025, 0.025]},
                    "mass_kg": 0.04,
                    "friction": "2.0 0.05 0.001",
                    "rgba": [0.82, 0.22, 0.12, 1.0],
                    "pose": {"pos_m": [0.19, 0.0, 0.025], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                }
            ],
            "sensors": [
                {
                    "id": "overhead_camera",
                    "kind": "camera",
                    "source": "scene",
                    "anchor": {"object_kind": "camera", "name": "overhead_camera", "entity": "piper"},
                    "pos_m": [0.28, -0.72, 0.72],
                    "look_at_m": [0.19, 0.0, 0.025],
                    "fovy_deg": 78,
                }
            ],
            "lights": [
                {
                    "name": "key",
                    "directional": True,
                    "dir": [-0.16, 0.16, -1.0],
                    "diffuse": [0.85, 0.84, 0.82],
                    "specular": [0.25, 0.25, 0.25],
                    "castshadow": False,
                }
            ],
            "model": {
                "output": "build/scenes/closed_lab/closed_lab.xml",
                "builder": "scripts/build_piper_pick_scene.py",
            },
        }
        baseline = {
            "schema_version": "iraf.scene-baseline/v1",
            "scene": "scene.yaml",
            "robots": {"piper": "config/piper_simulation_baseline.yaml"},
            "initial_state": {"piper": {"pose_source": "profile"}},
            "acceptance": {
                "pose_tolerance_m": 0.005,
                "min_lift_delta_m": 0.02,
                "min_stable_hold_s": 3.0,
                "dock_translation_error_max_m": 0.03,
                "dock_yaw_error_max_deg": 2.0,
                "max_speed_m_s": 0.0,
            },
        }
        scenario = {
            "schema_version": "iraf.scenario-catalog/v1",
            "scene": "scene.yaml",
            "simulation": True,
            "scenarios": {
                "nominal": {
                    "description": "正向对照序列（只用已交付能力）",
                    "steps": [
                        {
                            "id": "s01_pick",
                            "action": "pick_object",
                            "robot": "piper",
                            "params": {"target": "box_01"},
                            "criteria": {"pose_tolerance_m": 0.005, "min_lift_delta_m": 0.02},
                        }
                    ],
                    "faults": [],
                }
            },
        }
        dump_yaml(self.package / "scene.yaml", scene)
        dump_yaml(self.package / "baseline.yaml", baseline)
        dump_yaml(self.package / "scenario.yaml", scenario)
        (self.package / "README.md").write_text("# closed_lab\n", encoding="utf-8")

    def test_closed_package_passes_strict_mode(self):
        report, exit_code = scene_check.check(self.package, require_resolved_refs=True)
        self.assertEqual(exit_code, 0, msg=json.dumps(report, ensure_ascii=False, indent=2))
        self.assertEqual(report["pending_refs"], [])
        self.assertEqual(report["pending_steps"], [])

    def test_closed_package_still_fails_when_model_is_required(self):
        """严格模式通过不等于模型层做过校验：缺模型时 --require-model 仍必须失败。"""
        report, exit_code = scene_check.check(self.package, require_model=True)
        self.assertEqual(exit_code, scene_check.EXIT_MODEL_REQUIRED)


class CliTests(ScenePackageFixture):
    """CLI 层只在退出码上断言；stdout 摘要重定向掉，避免污染全量单测输出。"""

    def call_main(self, argv):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
            return scene_check.main(argv)

    def test_main_reports_zero_for_real_package(self):
        self.assertEqual(self.call_main(["--scene", str(PACKAGE_DIR)]), 0)

    def test_main_reports_reference_failure_code(self):
        def mutate(doc):
            doc["robots"][0]["profile"] = "profiles/does_not_exist_mujoco.yaml"

        self.mutate(self.scene_path, mutate)
        self.assertEqual(
            self.call_main(["--scene", str(self.package)]), scene_check.EXIT_REFERENCE
        )

    def test_missing_scene_directory_is_usage_error(self):
        self.assertEqual(
            self.call_main(["--scene", str(self.package / "nope")]), scene_check.EXIT_USAGE
        )


if __name__ == "__main__":
    unittest.main()
