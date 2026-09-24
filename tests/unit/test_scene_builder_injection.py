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


class WorldFixedFramesTests(SceneBuilderFixture):
    """世界固定帧（`frames:`）—— 停靠目标帧的正确形态（2026-09-24 新增）。

    要证明的四件事：
      1. 声明 → 注入为 **worldbody 下的 site**，且所属 body 是 worldbody（body 0）
         ⇒ 满足停靠对目标帧的要求"世界固定、不与机器人刚性相连"；
      2. 位姿按声明逐位生效（编译结果为准，不看 XML 文本）；
      3. 报告登记 `injected_as: world_site` + `pose_source: declaration`（可追溯）；
      4. 负向：重名 / `kind` 非 site / 缺 `pose` 一律**显式失败**（退出码 2），不静默跳过。
    """

    FRAME = {"id": "handoff_station_frame", "kind": "site",
             "pose": {"pos_m": [0.55, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
             "note": "夹具：站位帧"}

    def _build_with(self, frames):
        self.mutate_scene(lambda doc: doc.__setitem__("frames", frames))
        return self.build()

    def test_declared_frame_is_injected_as_world_fixed_site(self):
        import mujoco

        report = self._build_with([dict(self.FRAME)])
        records = report["injections"]["world_frames"]
        self.assertEqual([item["id"] for item in records], ["handoff_station_frame"])
        self.assertEqual(records[0]["injected_as"], "world_site")
        self.assertEqual(records[0]["pose_source"], "declaration")

        model = mujoco.MjModel.from_xml_path(report["output"])
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "handoff_station_frame")
        self.assertGreaterEqual(site_id, 0, msg="声明帧必须出现在**编译后**模型里")
        # 关键属性：所属 body = worldbody(0) ⇒ 世界固定（停靠门禁要求的那一条）
        self.assertEqual(int(model.site_bodyid[site_id]), 0)
        self.assertAlmostEqual(float(model.site_pos[site_id][0]), 0.55, places=9)
        self.assertAlmostEqual(float(model.site_pos[site_id][1]), 0.0, places=9)
        self.assertAlmostEqual(float(model.site_pos[site_id][2]), 0.0, places=9)

    def test_frame_pose_is_bit_exact_from_declaration(self):
        # 声明值 → 模型值逐位一致（避免"看着差不多"）
        report = self._build_with([dict(self.FRAME,
                                       pose={"pos_m": [0.4321, -0.1234, 0.0567],
                                             "quat_wxyz": [0.7071067811865476, 0.0, 0.0,
                                                           0.7071067811865475]})])
        import mujoco

        model = mujoco.MjModel.from_xml_path(report["output"])
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "handoff_station_frame")
        self.assertEqual(float(model.site_pos[site_id][0]), 0.4321)
        self.assertEqual(float(model.site_pos[site_id][1]), -0.1234)
        self.assertEqual(float(model.site_pos[site_id][2]), 0.0567)

    def test_absent_frames_leave_injection_empty(self):
        # 向后兼容：场景**不声明** frames 时，注入记录为空且不影响其他注入。
        # ⚠ 注意本用例必须**显式去掉** frames —— 生产场景（scenes/handoff_lab）现在已声明站位帧，
        # 直接 build() 会拿到 1 条记录（这正是我第一版写错的地方：断言与生产声明互相矛盾）。
        self.mutate_scene(lambda doc: doc.pop("frames", None))
        report = self.build()
        self.assertEqual(report["injections"]["world_frames"], [])
        self.assertTrue(report["injections"]["mount_frames"])

    def test_production_scene_declares_the_station_frame(self):
        # 生产声明侧：站位帧确实在场景里，且注入为 world site（与上面的"无声明"路径成对）
        report = self.build()
        self.assertEqual([item["id"] for item in report["injections"]["world_frames"]],
                         ["handoff_station_frame"])
        self.assertIn("handoff_station_frame", report["model_facts"]["sites"])

    def test_duplicate_name_fails_explicitly(self):
        # 与既有 site（托盘挂载参考系）重名 ⇒ 退出码 2，不静默改名
        self.mutate_scene(lambda doc: doc.__setitem__(
            "frames", [{"id": "tray_frame", "kind": "site",
                        "pose": {"pos_m": [0.0, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}}]))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "重名")

    def test_non_site_kind_fails_explicitly(self):
        # 两层防线：schema（`kind: const site`）先拒；构建器里还有第二层同判定（防"绕过 schema 的
        # 调用路径"）。这里断言第一层的实际消息 —— 判据是"显式失败"，不是某句固定文案。
        self.mutate_scene(lambda doc: doc.__setitem__(
            "frames", [{"id": "station_body", "kind": "body",
                        "pose": {"pos_m": [0.5, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}}]))
        error = self.assertBuildFails(scene_builder.EXIT_DECLARATION, "frames/0/kind")
        self.assertIn("site", str(error))

    def test_missing_pose_fails_explicitly(self):
        self.mutate_scene(lambda doc: doc.__setitem__(
            "frames", [{"id": "station_frame", "kind": "site"}]))
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "pose")


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
        # 场景声明的**世界固定帧**（`frames:`）也必须出现在编译后模型里
        for frame in scene.get("frames") or []:
            self.assertIn(frame["id"], facts["sites"], msg=frame["id"])
        # 注入计数（厂商 nbody=18 含 world、ncam=0、nsite=1）：
        # 注入后 nbody=20（+box_01 +tray_01）、ncam=1、nsite=4
        #（+tray_frame +payload_lidar_site +handoff_station_frame）。
        self.assertEqual(facts["ncam"], 1)
        self.assertEqual(facts["nsite"], 4)
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
        # 步骤 17 后：stand/stop 已经过技能层全链路验收并回填进 Profile
        # （证据 build/acceptance/go2-skills/report.json）；locomote 仍未声明
        # （首期无步态控制器）。这里的断言跟着**事实**走，不是放宽门禁。
        self.assertEqual(identity["capabilities"], ["stand", "stop", "locomote"])
        # 步骤 17 后 scene.robots[].profile 已闭合为路径 ⇒ 构建器走"声明路径"这条来源，
        # 而不是按声明身份回退查找（两条都合法，报告必须写明走了哪条，绝不静默）。
        self.assertEqual(report["robot"]["profile_source"], "scene.robots[].profile")
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


class InitialAlignmentTests(SceneBuilderFixture):
    """初始位姿对齐（A′ ①）：抬升量按**实测**最低可碰撞几何点算，并独立复核。"""

    def test_alignment_lifts_lowest_collision_geom_onto_support_plane(self):
        """正例：抬升量 = 实测最低点与支撑面的间隙；编译产物独立复核后残差为 0。

        回归重点（本步实测缺陷）：最低点必须落在**可碰撞几何**上。首版把视觉网格的包围球下界
        当最低点，抬升量从 18.372 mm 变成 96.406 mm —— 那会让四足悬空 78 mm 起步。
        """
        import mujoco

        report = self.build()
        section = report["initial_alignment"]
        self.assertTrue(section["enabled"])
        self.assertTrue(section["applied"])
        self.assertEqual(section["geometry_scope"], "collision")
        entry = section["keyframes"][0]
        self.assertEqual(entry["name"], "home")
        self.assertIn(entry["lowest_geom"], ("FL", "FR", "RL", "RR"))
        self.assertEqual(entry["lowest_geom_type"], "mjGEOM_SPHERE")
        self.assertAlmostEqual(entry["lift_m"], -entry["lowest_z_before_m"], places=9)
        self.assertAlmostEqual(entry["base_z_after_m"] - entry["base_z_before_m"], entry["lift_m"], places=9)
        self.assertLessEqual(abs(entry["residual_after_m"]), section["tolerance_m"])
        # 独立复核（不读报告）：编译产物 + 关键帧 → 四足足端球最低点落在支撑面（z=0）上。
        model = mujoco.MjModel.from_xml_path(str(self.output))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        for leg in ("FL", "FR", "RL", "RR"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
            lowest = float(data.geom_xpos[gid][2]) - float(model.geom_size[gid][0])
            self.assertLess(abs(lowest), 1.0e-6, msg="%s 的最低点 %.9f 未落在支撑面" % (leg, lowest))

    def test_absent_section_leaves_initial_pose_untouched(self):
        """正向对照（fail-closed 的另一半）：没有该声明段 ⇒ 不做对齐，厂商关键帧原样保留。"""
        self.mutate_profile(lambda document: document["spec"]["model"].pop("initial_alignment"))
        report = self.build()
        self.assertIsNone(report["initial_alignment"])

    def test_unknown_mode_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"]["initial_alignment"].__setitem__(
                "mode", "snap_to_ground"
            )
        )
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "mode")

    def test_missing_support_plane_source_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"]["initial_alignment"].pop(
                "support_plane_source"
            )
        )
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "support_plane_source")

    def test_unresolvable_support_plane_path_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"]["initial_alignment"].__setitem__(
                "support_plane_source", "terrain.no_such_layer.top_z_m"
            )
        )
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "terrain.no_such_layer.top_z_m")

    def test_invalid_geometry_scope_fails(self):
        self.mutate_profile(
            lambda document: document["spec"]["model"]["initial_alignment"].__setitem__(
                "geometry_scope", "visual"
            )
        )
        self.assertBuildFails(scene_builder.EXIT_DECLARATION, "geometry_scope")


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
