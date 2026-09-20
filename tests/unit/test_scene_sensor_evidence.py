"""场景传感器验收测试（步骤 14）。

被测对象：`scripts/verify_scene_sensors.py`（入口层）+ `iraf_adapters.unitree.scene_sensor_evidence`
（实现层）+ `config/scene.schema.json` 的 `definitions.scene_baseline.sensor_acceptance`（声明契约）。

设计要点（fail-closed，每条负向都配正向对照）：
  1. 阈值**只能**来自 `scenes/<id>/baseline.yaml` 的 `sensor_acceptance`：缺段/缺键即失败（退出码 2），
     脚本内不得有数字默认值；契约层同样把该段列为必需（schema `required`）。
  2. 被测产物是步骤 13 生成的 MJCF；缺失即引用层失败（退出码 3），**不**隐式重建。
  3. 每条判据都要能失败：点数/台面占比/工作台几何/采样漂移各有一条负向用例，
     用"把阈值调到必然不满足"或"改坏夹具产物"来证明门禁不是恒真。
  4. 离屏渲染不可用时相机部分标 BLOCKED（退出码 4），雷达与 IMU 仍必须完成——
     用**必然抛错的渲染替身**确定性复现该分支（不依赖本机 GL 状态）。
  5. 单测全部离线：夹具是自建的**合成小仓库**（合成 MJCF + 合成 profile + 合成场景包），
     不依赖 `vendor/` 厂商资产、`build/` 证据区，也不做真实渲染（真实渲染由步骤 14 的
     验收脚本入口 + `mujoco.Renderer` 证明，见 build/iraf-24h/14/sensors.txt）。
     渲染替身会在报告的 `camera.render.implementation` 里留痕（写成 stub 名字），
     避免把替身证据误读为真实渲染证据。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_scene_sensor_evidence -v`
"""

import hashlib
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml
from jsonschema import Draft7Validator

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src"))

from iraf_adapters.unitree import scene_sensor_evidence as sse  # noqa: E402

SCHEMA_PATH = REPO_ROOT / "config" / "scene.schema.json"
REAL_PACKAGE = REPO_ROOT / "scenes" / "handoff_lab"

STUB_RENDERER_NAME = "stub_renderer_for_tests"

#: 夹具工作台：half_size 取 5.0 m（真实场景是 0.8 m）——夹具雷达挂在 0.65 m 高，
#: 0.8 m 台面只有 -45° 一圈能打到（实测 -25° 起就落空），会让"点数下限 120"
#: 这条**生产阈值**在夹具里恒失败。放大台面（几何事实，不是放宽阈值）后
#: -45/-25/-10 三圈都能打到台面，夹具因此能复用生产阈值。
FIXTURE_WORKBENCH_HALF_SIZE_M = 5.0


class _StubRenderer:
    """离屏渲染替身：只回一帧固定灰图，用于离线验证判据链路。"""

    def __init__(self, height, width):
        self.height = int(height)
        self.width = int(width)
        self.closed = False

    def update_scene(self, data, camera=None):
        self.camera = camera

    def render(self):
        # 大部分像素为黑 + 一块 32x32 亮块：非全黑占比 0.333，
        # 既能通过生产的 min_nonblack_fraction（0.02），也留出"调高到 0.9 必然失败"的负向空间。
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[: min(32, self.height), : min(32, self.width), :] = 255
        return frame

    def close(self):
        self.closed = True


def stub_renderer_for_tests(model, height, width):  # noqa: N802 - 名字要出现在报告里
    return _StubRenderer(height, width)


class _RaisingRenderer:
    """必然抛错的渲染替身：确定性复现 EGL/GL 不可用分支。"""

    def __init__(self, *args, **kwargs):
        raise RuntimeError("模拟 EGL 初始化失败（单测替换真实渲染器）")


class SensorEvidenceFixture(unittest.TestCase):
    """在临时目录里搭一个**合成小仓库**：合成场景包 + 合成 profile + 合成 MJCF 产物。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name) / "repo"
        self.package_dir = self.root / "scenes" / "tests_lab"
        self.model_path = self.root / "build" / "scenes" / "tests_lab" / "tests_lab.xml"
        self.calibration_path = self.root / "build" / "calibration" / "testbot-camera.json"
        self.profile_path = self.root / "profiles" / "testbot_mujoco.yaml"
        self.machine_baseline = self.root / "config" / "testbot_simulation_baseline.yaml"
        (self.root / "config").mkdir(parents=True)
        shutil.copyfile(SCHEMA_PATH, self.root / "config" / "scene.schema.json")
        self.write_declarations()
        self.write_model()

    # ---- 夹具拼装 ----
    def write_declarations(self):
        real_scene = yaml.safe_load((REAL_PACKAGE / "scene.yaml").read_text(encoding="utf-8"))
        real_baseline = yaml.safe_load((REAL_PACKAGE / "baseline.yaml").read_text(encoding="utf-8"))

        scene = dict(real_scene)
        scene["id"] = "tests_lab"
        scene["description"] = "传感器验收单测夹具（合成场景，不代表任何真机能力）"
        scene["robots"] = [
            {
                "id": "testbot",
                "kind": "arm",
                "role": "manipulator",
                "profile": "profiles/testbot_mujoco.yaml",
                "capabilities": ["move_joint"],
                "note": "单测夹具本体：只声明 move_joint。",
            }
        ]
        scene["props"] = [
            {
                "id": "box_01",
                "body": "box_01",
                "kind": "box",
                "geometry": {"type": "box", "size_m": [0.025, 0.025, 0.025]},
                "mass_kg": 0.04,
                "friction": "2.0 0.05 0.001",
                "rgba": [0.82, 0.22, 0.12, 1.0],
                "pose": {"pos_m": [0.19, 0.0, 0.025], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                "note": "夹具道具。",
            }
        ]
        scene["sensors"] = [
            {
                "id": "overhead_camera",
                "kind": "camera",
                "source": "scene",
                "anchor": {"object_kind": "camera", "name": "overhead_camera", "entity": "testbot"},
                "pos_m": [0.28, -0.72, 0.72],
                "look_at_m": [0.19, 0.0, 0.025],
                "fovy_deg": 78,
                "note": "夹具相机。",
            },
            {
                "id": "payload_lidar",
                "kind": "lidar",
                "source": "scene",
                "anchor": {"object_kind": "site", "name": "lidar_site", "entity": "testbot"},
                "num_rays": 360,
                "range_m": 8.0,
                "mount_height_m": 0.1,
                "note": "夹具雷达。",
            },
            {
                "id": "base_imu",
                "kind": "imu",
                "source": "vendor",
                "anchor": {"object_kind": "site", "name": "imu_site", "entity": "testbot"},
                "rate_hz": 100,
                "note": "夹具 IMU（合成模型自带）。",
            },
        ]
        scene["terrain"] = dict(real_scene["terrain"])
        scene["terrain"]["workbench"] = {
            "top_z_m": 0.0,
            "half_size_m": FIXTURE_WORKBENCH_HALF_SIZE_M,
            "half_thickness_m": 0.025,
            "friction": "1.0 0.02 0.001",
        }
        scene["model"] = {
            "output": "build/scenes/tests_lab/tests_lab.xml",
            "builder": "scripts/build_scene.py",
        }
        self.write_yaml(self.package_dir / "scene.yaml", scene)
        (self.package_dir / "README.md").write_text("# tests_lab（单测夹具）\n", encoding="utf-8")

        baseline = dict(real_baseline)
        baseline["robots"] = {"testbot": "config/testbot_simulation_baseline.yaml"}
        baseline["initial_state"] = {"testbot": {"pose_source": "profile"}}
        self.write_yaml(self.package_dir / "baseline.yaml", baseline)

        self.write_yaml(
            self.profile_path,
            {
                "apiVersion": "iraf.intewell.io/v1",
                "kind": "RobotProfile",
                "metadata": {"name": "testbot", "version": "0.1.0"},
                "spec": {
                    "simulation": True,
                    "verification": "unverified",
                    "control_frequency_hz": 100,
                    "joints": ["arm_joint"],
                    "joint_limits": {"arm_joint": [-1.0, 1.0]},
                    "capabilities": ["move_joint"],
                    "model": {
                        "vendor": "testbot",
                        "file": "vendor/testbot/testbot.xml",
                        "trunk_body": "trunk",
                        "imu_site": "imu_site",
                        "mount_frames": {},
                    },
                },
            },
        )
        self.write_yaml(
            self.machine_baseline,
            {
                "schema_version": "iraf.simulation-baseline/v1",
                "vision": {"detector": {"calibration_file": "build/calibration/testbot-camera.json"}},
            },
        )
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        self.calibration_path.write_text(
            json.dumps(
                {
                    "schema_version": "iraf.camera-extrinsics/v1",
                    "camera": "overhead_camera",
                    "focal_px": 29.6,
                    "principal_point_px": [32.0, 24.0],
                    "image_size_px": [64, 48],
                }
            ),
            encoding="utf-8",
        )

    def write_model(self, ground_z=None, ground_half_size=None):
        ground_top = 0.0 if ground_z is None else float(ground_z)
        half = FIXTURE_WORKBENCH_HALF_SIZE_M if ground_half_size is None else float(ground_half_size)
        thickness = 0.025
        xml = """<mujoco model="tests_lab">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="workbench" type="box" pos="0 0 %.9f" size="%.6f %.6f %.6f" rgba="0.35 0.35 0.38 1"/>
    <camera name="overhead_camera" pos="0.28 -0.72 0.72" quat="1 0 0 0" fovy="78" mode="fixed"/>
    <body name="trunk" pos="0 0 0.5">
      <freejoint name="trunk_free"/>
      <geom name="trunk_geom" type="box" size="0.05 0.05 0.05" mass="1.0"/>
      <site name="imu_site" pos="0 0 0.02"/>
      <site name="lidar_site" pos="0 0 0.15"/>
      <body name="arm" pos="0.1 0 0">
        <joint name="arm_joint" type="hinge" axis="0 0 1" range="-1 1"/>
        <geom name="arm_geom" type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
      </body>
    </body>
    <body name="box_01" pos="0.19 0 0.025">
      <freejoint name="box_01_free"/>
      <geom name="box_01_geom" type="box" size="0.025 0.025 0.025" mass="0.04"/>
    </body>
  </worldbody>
  <actuator><motor name="arm" joint="arm_joint" ctrlrange="-10 10"/></actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
    <jointactuatorfrc name="arm_torque" joint="arm_joint"/>
  </sensor>
  <keyframe><key name="home" qpos="0 0 0.5 1 0 0 0 0 0.19 0 0.025 1 0 0 0" ctrl="0.7"/></keyframe>
</mujoco>
""" % (ground_top - thickness, half, half, thickness)
        self.model_path.parent.mkdir(parents=True, exist_ok=True)
        self.model_path.write_text(xml, encoding="utf-8")

    # ---- 工具 ----
    @staticmethod
    def write_yaml(path, document):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )

    def load_yaml(self, path):
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def mutate_yaml(self, path, mutate):
        document = self.load_yaml(path)
        mutate(document)
        self.write_yaml(path, document)

    def verify(self, **kwargs):
        kwargs.setdefault("renderer", stub_renderer_for_tests)
        return sse.verify_scene_sensors(self.package_dir, root=self.root, write_report=False, **kwargs)


class PositiveControlTests(SensorEvidenceFixture):
    def test_synthetic_scene_passes_all_checks(self):
        report, exit_code = self.verify()
        self.assertEqual(exit_code, 0, msg=json.dumps(report, ensure_ascii=False, indent=2))
        self.assertTrue(report["passed"])
        self.assertEqual(report["failed_checks"], [])
        self.assertEqual(report["simulation"], True)
        ids = {item["id"] for item in report["checks"]}
        self.assertEqual(
            ids,
            {
                "state.base_drift_after_settle",
                "camera.fovy_model_matches_declaration",
                "camera.calibration_fovy_consistency",
                "camera.calibration_focal_consistency",
                "camera.render_resolution_matches_declaration",
                "camera.render_nonblack_fraction",
                "lidar.workbench_geometry_matches_declaration",
                "lidar.points_min",
                "lidar.miss_fraction_max",
                "lidar.ground_fraction_min",
                "lidar.ray_vs_declared_box",
                "imu.quat_norm",
                "imu.gyro_matches_model_angular_velocity",
                "imu.acc_matches_model_linear_acceleration",
                "imu.torque_sensors_match_control",
            },
        )
        # 相机：声明 → 模型 回环必须逐位一致；标定内参一致性有两档（fovy/focal）。
        self.assertEqual(report["camera"]["model_vs_declaration_rel_error"], 0.0)
        self.assertLessEqual(report["camera"]["fovy_rel_error"], 0.05)
        self.assertLessEqual(report["camera"]["focal_rel_error"], 0.05)
        self.assertEqual(report["camera"]["render"]["resolution_diff_px"], 0)
        # 雷达：点数 > 0 且满足声明的下限；台面点与解析求交一致。
        self.assertGreaterEqual(report["lidar"]["points"], report["lidar"]["rays"] // 4)
        self.assertGreater(report["lidar"]["ground_points"], 0)
        self.assertEqual(report["lidar"]["ray_box_comparison"]["max_deviation_m"], 0.0)
        self.assertGreater(report["lidar"]["ray_box_comparison"]["compared"], 0)
        # IMU：读数必须与独立计算的模型量一致（不是与硬编码常数比对）。
        self.assertLessEqual(report["imu"]["quat"]["norm_error"], 1e-9)
        self.assertLessEqual(report["imu"]["gyro"]["max_abs_error_rad_s"], 1e-9)
        self.assertLessEqual(report["imu"]["acc"]["rel_error"], 0.01)
        self.assertEqual(report["imu"]["torque"]["max_abs_error_nm"], 0.0)
        # ctrl=0.7 必须真的体现在力矩传感器读数上（否则这条判据是空转的）。
        self.assertAlmostEqual(report["imu"]["torque"]["rows"][0]["measured_nm"], 0.7, places=9)

    def test_render_double_is_marked_in_report(self):
        """渲染替身必须在报告里留痕：替身证据不得被误读为真实渲染证据。"""
        report, _ = self.verify()
        self.assertEqual(report["camera"]["render"]["implementation"], STUB_RENDERER_NAME)

    def test_report_binds_evidence_to_artifact(self):
        report_path = self.package_dir.parent.parent / "build" / "acceptance" / "tests-lab-scene-sensors" / "report.json"
        report, exit_code = sse.verify_scene_sensors(
            self.package_dir, root=self.root, write_report=True, renderer=stub_renderer_for_tests
        )
        self.assertEqual(exit_code, 0)
        self.assertTrue(report_path.is_file())
        written = json.loads(report_path.read_text(encoding="utf-8"))
        digest = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        self.assertEqual(written["scene"]["model_sha256"], digest)
        self.assertEqual(written["scene"]["baseline"], "scenes/tests_lab/baseline.yaml")
        self.assertEqual(written["declarations"]["source"], "scenes/tests_lab/baseline.yaml#sensor_acceptance")
        self.assertTrue(written["not_proved"])
        self.assertEqual(written["environment"]["mujoco"], sse.mujoco.__version__)
        self.assertTrue(written["report_path"].endswith("tests-lab-scene-sensors/report.json"))

    def test_real_package_thresholds_are_schema_valid(self):
        """正向对照：真实场景包的 baseline.yaml 必须同时通过 schema 与实现层门禁。"""
        baseline = self.load_yaml(REAL_PACKAGE / "baseline.yaml")
        schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        validator = Draft7Validator(schema["definitions"]["scene_baseline"])
        errors = sorted(validator.iter_errors(baseline), key=lambda item: list(item.absolute_path))
        self.assertEqual([item.message for item in errors], [])


class DeclarationGateTests(SensorEvidenceFixture):
    def test_missing_sensor_acceptance_section_fails(self):
        self.mutate_yaml(self.package_dir / "baseline.yaml", lambda doc: doc.pop("sensor_acceptance"))
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("sensor_acceptance", str(ctx.exception))

    def test_missing_threshold_key_fails(self):
        self.mutate_yaml(
            self.package_dir / "baseline.yaml",
            lambda doc: doc["sensor_acceptance"]["camera"].pop("min_nonblack_fraction"),
        )
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("min_nonblack_fraction", str(ctx.exception))

    def test_schema_requires_sensor_acceptance_section(self):
        """契约层门禁：缺段即 schema 非法（不是只有实现层在拦）。"""
        baseline = self.load_yaml(REAL_PACKAGE / "baseline.yaml")
        baseline.pop("sensor_acceptance")
        schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        validator = Draft7Validator(schema["definitions"]["scene_baseline"])
        messages = [item.message for item in validator.iter_errors(baseline)]
        self.assertTrue(any("sensor_acceptance" in message for message in messages), msg=messages)

    def test_schema_rejects_impossible_threshold_values(self):
        baseline = self.load_yaml(REAL_PACKAGE / "baseline.yaml")
        baseline["sensor_acceptance"]["lidar"]["min_points"] = 0
        baseline["sensor_acceptance"]["lidar"]["max_miss_fraction"] = 1.5
        baseline["sensor_acceptance"]["camera"]["resolution_tolerance_px"] = -1
        baseline["sensor_acceptance"]["imu"]["acc_rel_error_max"] = 0.0
        schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        validator = Draft7Validator(schema["definitions"]["scene_baseline"])
        errors = sorted(validator.iter_errors(baseline), key=lambda item: list(item.absolute_path))
        locations = {"/".join(str(part) for part in item.absolute_path) for item in errors}
        self.assertIn("sensor_acceptance/lidar/min_points", locations)
        self.assertIn("sensor_acceptance/lidar/max_miss_fraction", locations)
        self.assertIn("sensor_acceptance/camera/resolution_tolerance_px", locations)
        self.assertIn("sensor_acceptance/imu/acc_rel_error_max", locations)

    def test_multiple_lidar_sensors_fail(self):
        def mutate(doc):
            duplicate = json.loads(json.dumps(doc["sensors"][1]))
            duplicate["id"] = "payload_lidar_backup"
            doc["sensors"].append(duplicate)

        self.mutate_yaml(self.package_dir / "scene.yaml", mutate)
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("payload_lidar_backup", str(ctx.exception))

    def test_missing_camera_sensor_fails(self):
        self.mutate_yaml(
            self.package_dir / "scene.yaml",
            lambda doc: doc.__setitem__(
                "sensors", [item for item in doc["sensors"] if item["kind"] != "camera"]
            ),
        )
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("camera", str(ctx.exception))

    def test_num_rays_not_divisible_fails(self):
        def mutate(doc):
            for sensor in doc["sensors"]:
                if sensor["kind"] == "lidar":
                    sensor["num_rays"] = 361

        self.mutate_yaml(self.package_dir / "scene.yaml", mutate)
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("361", str(ctx.exception))

    def test_unknown_robot_fails(self):
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify(robot="no_such_robot")
        self.assertEqual(ctx.exception.code, sse.EXIT_REFERENCE)
        self.assertIn("no_such_robot", str(ctx.exception))

    def test_non_increasing_histogram_edges_fail(self):
        self.mutate_yaml(
            self.package_dir / "baseline.yaml",
            lambda doc: doc["sensor_acceptance"]["lidar"].__setitem__(
                "range_histogram_edges_m", [0.0, 0.5, 0.5, 8.0]
            ),
        )
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)
        self.assertIn("range_histogram_edges_m", str(ctx.exception))


class ReferenceGateTests(SensorEvidenceFixture):
    def test_missing_model_fails_with_hint(self):
        self.model_path.unlink()
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_REFERENCE)
        self.assertIn("build_scene.py", str(ctx.exception))

    def test_missing_calibration_file_fails(self):
        self.calibration_path.unlink()
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_REFERENCE)
        self.assertIn("testbot-camera.json", str(ctx.exception))

    def test_missing_camera_in_model_fails(self):
        self.model_path.write_text(
            self.model_path.read_text(encoding="utf-8").replace(
                '<camera name="overhead_camera"', '<camera name="other_camera"'
            ),
            encoding="utf-8",
        )
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_REFERENCE)
        self.assertIn("overhead_camera", str(ctx.exception))

    def test_profile_imu_site_mismatch_fails(self):
        self.mutate_yaml(
            self.profile_path,
            lambda doc: doc["spec"]["model"].__setitem__("imu_site", "absent_site"),
        )
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            self.verify()
        self.assertEqual(ctx.exception.code, sse.EXIT_REFERENCE)
        self.assertIn("absent_site", str(ctx.exception))


class GateCanFailTests(SensorEvidenceFixture):
    """证明判据会真的失败（恒不触发的负向用例等于没有用例）。"""

    def test_points_gate_fails(self):
        def mutate(doc):
            doc["sensor_acceptance"]["lidar"]["min_points"] = 361

        self.mutate_yaml(self.package_dir / "baseline.yaml", mutate)
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("lidar.points_min", report["failed_checks"])
        check = next(item for item in report["checks"] if item["id"] == "lidar.points_min")
        self.assertLess(check["measured"], 361)
        self.assertFalse(report["passed"])

    def test_ground_fraction_gate_fails(self):
        self.mutate_yaml(
            self.package_dir / "baseline.yaml",
            lambda doc: doc["sensor_acceptance"]["lidar"].__setitem__("min_ground_fraction", 0.99),
        )
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("lidar.ground_fraction_min", report["failed_checks"])

    def test_workbench_geometry_mismatch_is_caught(self):
        """把注入的地面 geom 下移 1 mm：声明→模型 回环与解析求交两条判据都必须报警。"""
        self.write_model(ground_z=-0.001)
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("lidar.workbench_geometry_matches_declaration", report["failed_checks"])
        self.assertIn("lidar.ray_vs_declared_box", report["failed_checks"])
        deviation = report["lidar"]["ray_box_comparison"]["max_deviation_m"]
        self.assertGreater(deviation, 1e-6)

    def test_settle_drift_gate_fails(self):
        def mutate(doc):
            doc["sensor_acceptance"]["settle"]["steps"] = 200

        self.mutate_yaml(self.package_dir / "baseline.yaml", mutate)
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("state.base_drift_after_settle", report["failed_checks"])
        self.assertGreater(report["state"]["base_drift_m"], 0.01)

    def test_low_nonblack_fraction_gate_fails(self):
        def mutate(doc):
            doc["sensor_acceptance"]["camera"]["min_nonblack_fraction"] = 0.9

        self.mutate_yaml(self.package_dir / "baseline.yaml", mutate)
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("camera.render_nonblack_fraction", report["failed_checks"])

    def test_calibration_mismatch_gate_fails(self):
        """把标定焦距改成另一台相机的值：内参一致性判据必须失败。"""
        calibration = json.loads(self.calibration_path.read_text(encoding="utf-8"))
        calibration["focal_px"] = 60.0
        self.calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
        report, exit_code = self.verify()
        self.assertEqual(exit_code, sse.EXIT_FAILED)
        self.assertIn("camera.calibration_fovy_consistency", report["failed_checks"])
        self.assertIn("camera.calibration_focal_consistency", report["failed_checks"])


class RenderBlockedTests(SensorEvidenceFixture):
    def test_render_failure_marks_camera_blocked_and_keeps_others(self):
        report, exit_code = self.verify(renderer=_RaisingRenderer)
        self.assertEqual(exit_code, sse.EXIT_RENDER)
        self.assertFalse(report["passed"])
        self.assertEqual(report["camera"]["status"], "blocked")
        self.assertEqual(report["camera"]["render"]["status"], "blocked")
        self.assertIn("EGL", report["camera"]["render"]["error"])
        self.assertIn("camera.offscreen_render", report["failed_checks"])
        # 雷达与 IMU 必须仍然完成（步骤 14「失败/阻塞处理」的硬要求）。
        self.assertGreater(report["lidar"]["points"], 0)
        self.assertEqual(report["imu"]["torque"]["joints"], 1)
        self.assertNotEqual(report["lidar"]["status"], "blocked")

    def test_no_render_flag_reports_disabled_and_exits_render(self):
        report, exit_code = self.verify(render=False, renderer=None)
        self.assertEqual(exit_code, sse.EXIT_RENDER)
        self.assertEqual(report["camera"]["render"]["status"], "disabled_by_usage")
        self.assertFalse(report["passed"])
        self.assertGreater(report["lidar"]["points"], 0)


class PureFunctionTests(unittest.TestCase):
    def test_ray_box_distance_hit_and_miss(self):
        center = [0.0, 0.0, -0.025]
        half = [0.8, 0.8, 0.025]
        hit = sse._ray_box_distance([0.0, 0.0, 0.5], [0.0, 0.0, -1.0], center, half)
        self.assertAlmostEqual(hit, 0.5, places=12)
        inside = sse._ray_box_distance([0.0, 0.0, -0.025], [0.0, 0.0, -1.0], center, half)
        self.assertAlmostEqual(inside, 0.025, places=12)
        self.assertIsNone(sse._ray_box_distance([5.0, 0.0, 0.5], [0.0, 0.0, -1.0], center, half))
        # 与盒面平行且横向在盒外：必须判为无交，而不是 inf/nan。
        self.assertIsNone(sse._ray_box_distance([1.0, 0.0, 0.5], [1.0, 0.0, 0.0], center, half))
        # 方向朝上：正向无交。
        self.assertIsNone(sse._ray_box_distance([0.0, 0.0, 0.5], [0.0, 0.0, 1.0], center, half))

    def test_relative_error_rejects_zero_reference(self):
        with self.assertRaises(sse.SensorEvidenceError) as ctx:
            sse._rel_error(1.0, 0.0)
        self.assertEqual(ctx.exception.code, sse.EXIT_DECLARATION)

    def test_sensor_type_mapping_matches_mujoco(self):
        self.assertEqual(sse.SENSOR_TYPES["quat"], sse.mujoco.mjtSensor.mjSENS_FRAMEQUAT)
        self.assertEqual(sse.SENSOR_TYPES["gyro"], sse.mujoco.mjtSensor.mjSENS_GYRO)
        self.assertEqual(
            sse.SENSOR_TYPES["accelerometer"], sse.mujoco.mjtSensor.mjSENS_ACCELEROMETER
        )
        self.assertEqual(
            sse.SENSOR_TYPES["joint_torque"], sse.mujoco.mjtSensor.mjSENS_JOINTACTFRC
        )


if __name__ == "__main__":
    unittest.main()
