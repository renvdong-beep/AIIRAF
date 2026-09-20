"""场景构建器注入测试（步骤 13）。

被测契约：`src/iraf_adapters/unitree/scene_builder.py` + `scripts/build_scene.py`。

断言的是"注入后模型里能按名字找到每个声明对象"，且判定以**MuJoCo 编译结果**为准，
不看 XML 文本（文本里有名字但编译报错/被 include 覆盖都算没生效）。

覆盖：
  1. 正向：真实场景包（`scenes/handoff_lab` + `profiles/unitree_go2_mujoco.yaml`）生成成功，
     相机/雷达 site/托盘 body/挂载参考系/工作台/光源逐项能在模型里按名字找到；
  2. 正向对照：厂商资产哈希与锁一致（证明锁校验不是恒失败的门禁）；
  3. 负向（引用完整性，退出码 3）：传感器锚点 site 不存在、道具挂载参考系未声明、
     跨本体挂载、本体 id 不在场景里；
  4. 负向（厂商锁，退出码 4）：厂商文件哈希与锁不一致、资产未在锁内登记；
  5. 负向（声明，退出码 2）：Profile 缺 spec.model、场景 id 与目录名不一致、
     场景声明不符合 schema；
  6. 厂商文件只读：构建前后厂商模型 SHA-256 不变，且构建过程不写 vendor/；
  7. 报告契约：键集合与既有场景报告一致（gripper/target_id 键存在），
     `scripts/emit_backend_config.py` 能直接消费（四足场景 manipulation 值为 null，不据此装配）。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_scene_builder_injection -v`
"""

import hashlib
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from iraf_adapters.unitree import scene_builder  # noqa: E402

import emit_backend_config  # noqa: E402

PACKAGE_DIR = REPO_ROOT / "scenes" / "handoff_lab"
PROFILE = REPO_ROOT / "profiles" / "unitree_go2_mujoco.yaml"
SCHEMA = REPO_ROOT / "config" / "scene.schema.json"
LOCK = REPO_ROOT / "vendor" / "unitree_go2" / "source-lock.json"
VENDOR_MODEL = REPO_ROOT / "vendor" / "unitree_go2" / "unitree_robots" / "go2" / "go2.xml"


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


def _dump(path, document):
    Path(path).write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SceneBuilderFixture(unittest.TestCase):
    """在临时仓库根上构建：声明可改写，真实仓库与真实厂商资产不被触碰。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.scene_dir = self.root / "scenes" / "handoff_lab"
        shutil.copytree(PACKAGE_DIR, self.scene_dir)
        (self.root / "profiles").mkdir(parents=True, exist_ok=True)
        shutil.copy2(PROFILE, self.root / "profiles" / PROFILE.name)
        (self.root / "config").mkdir(parents=True, exist_ok=True)
        shutil.copy2(SCHEMA, self.root / "config" / SCHEMA.name)
        self.scene_path = self.scene_dir / "scene.yaml"
        self.baseline_path = self.scene_dir / "baseline.yaml"
        self.profile_path = self.root / "profiles" / PROFILE.name
        # 厂商资产按锁真实存在（含 mesh assets），这样正向用例能真正编译。
        vendor_src = REPO_ROOT / "vendor" / "unitree_go2"
        shutil.copytree(vendor_src, self.root / "vendor" / "unitree_go2")
        self.output = self.root / "build" / "scenes" / "handoff_lab" / "handoff_lab.xml"

    def mutate_scene(self, mutate):
        document = _load(self.scene_path)
        mutate(document)
        _dump(self.scene_path, document)

    def mutate_profile(self, mutate):
        document = _load(self.profile_path)
        mutate(document)
        _dump(self.profile_path, document)

    def mutate_lock(self, mutate):
        document = json.loads(LOCK.read_text(encoding="utf-8"))
        mutate(document)
        (self.root / "vendor" / "unitree_go2" / "source-lock.json").write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def build(self, robot="unitree_go2"):
        return scene_builder.build_scene_model(
            self.scene_dir, robot, root=self.root, output=self.output
        )

    def assertBuildFails(self, code, fragment, robot="unitree_go2"):
        with self.assertRaises(scene_builder.SceneBuildError) as context:
            self.build(robot=robot)
        self.assertEqual(context.exception.code, code, msg=str(context.exception))
        self.assertIn(fragment, str(context.exception))
        return context.exception


class InjectionTests(SceneBuilderFixture):
    def test_all_declared_objects_are_found_in_compiled_model(self):
        report = self.build()
        facts = report["model_facts"]
        scene = _load(self.scene_path)
        # 传感器：相机 → world camera；雷达/IMU → site。
        for sensor in scene["sensors"]:
            anchor = sensor["anchor"]
            bucket = facts["cameras"] if anchor["object_kind"] == "camera" else facts["sites"]
            self.assertIn(anchor["name"], bucket, msg=sensor["id"])
        # 道具 body 与挂载参考系。
        for prop in scene["props"]:
            self.assertIn(prop["body"], facts["bodies"], msg=prop["id"])
        self.assertIn("tray_frame", facts["sites"])
        # 注入计数（厂商 nbody=18 含 world、ncam=0、nsite=1）：
        # 注入后 nbody=20（+box_01 +tray_01）、ncam=1、nsite=3（+tray_frame +payload_lidar_site）。
        self.assertEqual(facts["ncam"], 1)
        self.assertEqual(facts["nsite"], 3)
        self.assertEqual(facts["nbody"], 20)
        self.assertEqual(facts["nu"], 12)  # 四足 12 个力矩型 motor，未被改写
        self.assertEqual([item["name"] for item in report["sensors"]["injected"]], ["overhead_camera", "payload_lidar_site"])
        self.assertEqual(
            [item["name"] for item in report["sensors"]["vendor_referenced"]], ["imu"]
        )

    def test_workbench_lights_and_props_follow_declaration(self):
        report = self.build()
        scene = _load(self.scene_path)
        self.assertEqual(report["injections"]["lights"], [item["name"] for item in scene["lights"]])
        self.assertEqual(
            report["injections"]["workbench"]["top_z_m"], scene["terrain"]["workbench"]["top_z_m"]
        )
        positions = {item["id"]: item["position_m"] for item in report["injections"]["props"]}
        self.assertEqual(positions["box_01"], scene["props"][0]["pose"]["pos_m"])
        # 托盘按挂载参考系注入（位置来自 Profile 的 mount_frames，不是场景绝对坐标）。
        self.assertEqual(report["injections"]["props"][1]["pose_source"], "mount_frame")
        self.assertEqual(report["injections"]["props"][1]["position_m"], [0.0, 0.0, 0.057])

    def test_report_key_set_stays_consumable_by_backend_config(self):
        """键集合兼容：既有消费者不抛错；四足场景的 manipulation 值为 null。"""
        report = self.build()
        for key in ("target_id", "gripper", "vision", "workbench_top_z_m", "schema_version"):
            self.assertIn(key, report)
        self.assertIsNone(report["target_id"])
        self.assertIsNone(report["gripper"])
        self.assertEqual(report["report_kind"], "scene_model")
        # 既有消费入口（scripts/emit_backend_config.py）在本报告上仍可运行。
        config = emit_backend_config.build_config(report, self.output)
        self.assertEqual(config["model_path"], str(self.output.resolve()))
        # 既有消费者把 target_id/gripper **原样搬运**（不补默认值）：本机型为四足，
        # 因此搬出来的是显式 null，绝不会被当成一个有效的操作目标。
        self.assertIsNone(config["manipulation"]["gripper"])
        self.assertEqual(list(config["manipulation"]["targets"]), [None])
        self.assertIn("manipulation_absent_reason", report)

    def test_profile_identity_is_recorded(self):
        report = self.build()
        identity = report["profile_identity"]
        self.assertEqual(identity["name"], "unitree_go2")
        self.assertEqual(len(identity["joints"]), 12)
        self.assertEqual(identity["capabilities"], [])  # 能力未验收：空是事实
        self.assertEqual(report["robot"]["profile_source"], "declared_identity_lookup")
        self.assertTrue(report["simulation"])

    def test_vendor_model_is_read_only(self):
        before = _sha256(self.root / "vendor" / "unitree_go2" / "unitree_robots" / "go2" / "go2.xml")
        report = self.build()
        after = _sha256(self.root / "vendor" / "unitree_go2" / "unitree_robots" / "go2" / "go2.xml")
        self.assertEqual(before, after)
        self.assertTrue(report["vendor_source"]["match"])
        self.assertEqual(report["vendor_source"]["sha256"], report["vendor_source"]["locked_sha256"])
        # XML 里的注入只发生在内存副本上：厂商文件的字节数与锁一致。
        lock = json.loads(
            (self.root / "vendor" / "unitree_go2" / "source-lock.json").read_text(encoding="utf-8")
        )
        entry = next(item for item in lock["files"] if item["path"] == "unitree_robots/go2/go2.xml")
        self.assertEqual(entry["sha256"], after)

    def test_generated_model_is_loadable_and_keyframe_dimension_fixed(self):
        """注入自由关节后关键帧必须补齐，且自由道具回到声明位姿（不是世界原点）。"""
        report = self.build()
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(self.output))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        # xpos 是 mj_forward 的结果量：不 forward 的话读到的是零初始化缓冲区。
        mujoco.mj_forward(model, data)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box_01")
        self.assertGreaterEqual(body_id, 0)
        for axis, expected in enumerate([0.19, 0.0, 0.025]):
            self.assertAlmostEqual(float(data.xpos[body_id][axis]), expected, places=6)
        # 厂商 12 关节 + 躯干 freejoint + 注入自由道具 box_01 的 freejoint = 14
        self.assertEqual(report["model_facts"]["njnt"], 14)


class NegativeReferenceTests(SceneBuilderFixture):
    def test_unknown_sensor_anchor_site_fails(self):
        """厂商来源传感器的锚点必须真的存在于厂商模型里（改名字即失败）。"""

        def mutate(document):
            document["sensors"][2]["anchor"]["name"] = "no_such_site"

        self.mutate_scene(mutate)
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "no_such_site")

    def test_unknown_mount_frame_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"]["mount_frames"].pop("tray_frame")
        )
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "mount_frames")

    def test_cross_entity_mount_fails(self):
        def mutate(document):
            document["props"][1]["pose"]["mount"]["entity"] = "piper"

        self.mutate_scene(mutate)
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "不是本次构建的本体")

    def test_unknown_robot_fails(self):
        self.assertBuildFails(
            scene_builder.EXIT_REFERENCE, "没有本体", robot="unitree_h1"
        )

    def test_sensor_anchored_to_foreign_entity_site_fails(self):
        """site 类锚点挂在别的主体上：无法注入，必须显式失败（相机例外，见注入规则）。"""

        def mutate(document):
            document["sensors"][1]["anchor"]["entity"] = "piper"

        self.mutate_scene(mutate)
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "无法把 site 挂到它的几何上")


class VendorLockTests(SceneBuilderFixture):
    def test_hash_mismatch_fails(self):
        def mutate(document):
            for item in document["files"]:
                if item["path"] == "unitree_robots/go2/go2.xml":
                    item["sha256"] = "0" * 64

        self.mutate_lock(mutate)
        self.assertBuildFails(scene_builder.EXIT_VENDOR_LOCK, "哈希与锁不一致")

    def test_unregistered_vendor_asset_fails(self):
        self.mutate_lock(lambda document: document.__setitem__("files", []))
        self.assertBuildFails(scene_builder.EXIT_VENDOR_LOCK, "未在锁内登记")

    def test_vendor_path_outside_lock_prefix_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"].__setitem__(
                "file", "config/piper_simulation_baseline.yaml"
            )
        )
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "不在 vendor/")


class DeclarationTests(SceneBuilderFixture):
    def test_profile_without_model_section_fails(self):
        self.mutate_profile(lambda document: document["spec"].pop("model"))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "缺少 spec.model")

    def test_invalid_profile_is_rejected_by_core_validator(self):
        self.mutate_profile(lambda document: document["spec"].pop("joint_limits"))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "核心校验")

    def test_scene_id_must_match_directory(self):
        self.mutate_scene(lambda document: document.__setitem__("id", "other_lab"))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "与目录名")

    def test_declaration_must_satisfy_schema(self):
        self.mutate_scene(lambda document: document.pop("simulation"))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "scene.schema.json")

    def test_trunk_body_must_exist_in_vendor_model(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"].__setitem__("trunk_body", "no_such_body")
        )
        self.assertBuildFails(scene_builder.EXIT_REFERENCE, "no_such_body")

    def test_missing_scene_directory_fails(self):
        with self.assertRaises(scene_builder.SceneBuildError) as context:
            scene_builder.build_scene_model(
                self.root / "scenes" / "nope", "unitree_go2", root=self.root
            )
        self.assertEqual(context.exception.code, scene_builder.EXIT_REFERENCE)


if __name__ == "__main__":
    unittest.main()
