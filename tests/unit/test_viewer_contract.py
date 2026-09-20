"""契约与 profile 结构化扩展的单元测试。

覆盖：
- RobotBackend 契约方法存在性（新增方法不得缺失）
- 既有 4 个方法签名不被改动
- RobotProfile 结构化字段的向后兼容（缺失回退、位置参数构造）
- 结构化字段的显式校验（非法引用必须报错）
- viewer_runner 纯函数（相机读取、容差读取、请求组装）
"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.backend import RobotBackend  # noqa: E402
from iraf_core.core import RobotProfile  # noqa: E402
from iraf_core.profile import ProfileError, load_robot_profile  # noqa: E402


class RobotBackendContractTests(unittest.TestCase):
    """契约补全后，可视化与生命周期方法必须齐全。"""

    REQUIRED_VIEWER_METHODS = (
        "start_continuous",
        "stop_continuous",
        "is_continuous",
        "render_frames",
        "home_pose",
        "hold_current_pose",
        "simulation_status",
        "display_lock",
    )

    def test_viewer_contract_methods_present(self):
        for name in self.REQUIRED_VIEWER_METHODS:
            self.assertTrue(
                hasattr(RobotBackend, name),
                "RobotBackend 缺少可视化契约方法: " + name,
            )

    def test_original_methods_preserved(self):
        for name in ("move_joint", "pick_object", "stop", "step"):
            self.assertTrue(
                hasattr(RobotBackend, name), "既有契约方法丢失: " + name
            )

    def test_move_joint_signature_unchanged(self):
        import inspect

        params = list(inspect.signature(RobotBackend.move_joint).parameters)
        self.assertEqual(params, ["self", "positions", "duration_ms", "lease"])

    def test_pick_object_signature_unchanged(self):
        import inspect

        params = list(inspect.signature(RobotBackend.pick_object).parameters)
        self.assertEqual(
            params, ["self", "target_id", "grasp_pose", "duration_ms", "lease"]
        )


class RobotProfileCompatTests(unittest.TestCase):
    """结构化字段必须向后兼容。"""

    def test_positional_construction_still_works(self):
        profile = RobotProfile(
            "robot", "1.0.0", True, 100, ("j1",), {"j1": (-1, 1)},
            frozenset({"move_joint"}), "development", "digest",
        )
        self.assertEqual(profile.name, "robot")
        self.assertIsNone(profile.joint_roles)
        self.assertIsNone(profile.gripper)
        self.assertIsNone(profile.home)
        self.assertIsNone(profile.camera)
        self.assertIsNone(profile.manipulation)

    def test_structured_fields_accepted(self):
        profile = RobotProfile(
            "robot", "1.0.0", True, 100, ("j1", "j2"),
            {"j1": (-1, 1), "j2": (-1, 1)}, frozenset(), "development", "digest",
            joint_roles={"j1": "arm", "j2": "gripper_drive_left"},
            gripper={"drive_joints": ["j1", "j2"]},
            home={"j1": 0.5},
            camera={"lookat_m": [0.0, 0.0, 0.1]},
            manipulation={"targets": {}},
        )
        self.assertEqual(profile.home, {"j1": 0.5})
        self.assertEqual(profile.gripper, {"drive_joints": ["j1", "j2"]})

    def test_piper_profile_parses_structured_sections(self):
        profile = load_robot_profile(ROOT / "profiles/piper_mujoco.yaml")
        self.assertIsNotNone(profile.joint_roles)
        self.assertIsNotNone(profile.gripper)
        self.assertIsNotNone(profile.home)
        self.assertIsNotNone(profile.camera)
        self.assertIsNotNone(profile.manipulation)

    def test_piper_gripper_declares_drive_joints(self):
        profile = load_robot_profile(ROOT / "profiles/piper_mujoco.yaml")
        self.assertEqual(profile.gripper["drive_joints"], ["joint7", "joint8"])
        self.assertIn("max_tilt_deg", profile.gripper)

    def test_joint_roles_cover_all_joints(self):
        profile = load_robot_profile(ROOT / "profiles/piper_mujoco.yaml")
        self.assertEqual(set(profile.joint_roles), set(profile.joints))
        arm = [j for j, role in profile.joint_roles.items() if role == "arm"]
        self.assertEqual(len(arm), 6)

    def test_home_references_only_declared_joints(self):
        profile = load_robot_profile(ROOT / "profiles/piper_mujoco.yaml")
        self.assertTrue(set(profile.home).issubset(set(profile.joints)))


class StructuredSectionValidationTests(unittest.TestCase):
    """非法结构必须显式拒绝，不得静默兜底。"""

    def _write(self, spec_extra, name):
        path = ROOT / "build" / ("tmp_%s.yaml" % name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            "apiVersion: iraf.intewell.io/v1\n"
            "kind: RobotProfile\n"
            "metadata: {name: tmp_%s, version: 1.0.0}\n"
            "spec:\n"
            "  simulation: true\n"
            "  control_frequency_hz: 100\n"
            "  joints: [j1, j2]\n"
            "  joint_limits: {j1: [0, 1], j2: [0, 1]}\n" % (name,)
            + spec_extra,
            encoding="utf-8",
        )
        return path

    def test_unknown_joint_role_rejected(self):
        path = self._write("  joint_roles: {j9: arm}\n", "badrole")
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_unknown_role_name_rejected(self):
        path = self._write("  joint_roles: {j1: wizard}\n", "badrolename")
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_gripper_accepts_single_tendon_drive(self):
        """单驱动关节合法：Robotiq 2F-85 为单 tendon 驱动，不存在两个独立关节。

        契约由"恰好两个"放宽为 1..N，缺省类型按键数推断为 tendon。
        """
        path = self._write(
            "  gripper:\n"
            "    drive_joints: [j1]\n"
            "    open_positions: {j1: 0.0}\n"
            "    closed_positions: {j1: 255.0}\n",
            "tendondrive",
        )
        profile = load_robot_profile(path)
        self.assertEqual(["j1"], profile.gripper["drive_joints"])
        self.assertEqual("tendon", profile.gripper["type"])
        # 单驱动下左右索引都指向该通道。
        self.assertEqual(0, profile.gripper["left_index"])
        self.assertEqual(0, profile.gripper["right_index"])
        # 归一化开合度（0..255）必须原样保留，不在解析层做量纲裁剪。
        self.assertEqual(255.0, profile.gripper["closed_positions"]["j1"])

    def test_gripper_parallel_requires_exactly_two_drives(self):
        """显式声明 parallel 时仍必须恰好两个驱动关节。"""
        path = self._write(
            "  gripper:\n"
            "    type: parallel\n"
            "    drive_joints: [j1]\n"
            "    open_positions: {j1: 1.0}\n"
            "    closed_positions: {j1: 0.0}\n",
            "badparallel",
        )
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_gripper_unknown_type_rejected(self):
        path = self._write(
            "  gripper:\n"
            "    type: magic\n"
            "    drive_joints: [j1, j2]\n"
            "    open_positions: {j1: 1.0, j2: 0.0}\n"
            "    closed_positions: {j1: 0.0, j2: 1.0}\n",
            "badtype",
        )
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_gripper_duplicate_drive_joints_rejected(self):
        path = self._write(
            "  gripper:\n"
            "    drive_joints: [j1, j1]\n"
            "    open_positions: {j1: 1.0}\n"
            "    closed_positions: {j1: 0.0}\n",
            "dupdrive",
        )
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_gripper_index_out_of_range_rejected(self):
        path = self._write(
            "  gripper:\n"
            "    drive_joints: [j1]\n"
            "    left_index: 3\n"
            "    open_positions: {j1: 1.0}\n"
            "    closed_positions: {j1: 0.0}\n",
            "badindex",
        )
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_gripper_positions_must_match_drive_joints(self):
        path = self._write(
            "  gripper:\n"
            "    drive_joints: [j1, j2]\n"
            "    open_positions: {j1: 1.0}\n"
            "    closed_positions: {j1: 0.0}\n",
            "badpositions",
        )
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_home_with_unknown_joint_rejected(self):
        path = self._write("  home: {j9: 0.5}\n", "badhome")
        with self.assertRaises(ProfileError):
            load_robot_profile(path)

    def test_camera_lookat_must_be_three_numbers(self):
        path = self._write("  camera: {lookat_m: [0.0, 0.1], distance_m: 1.0}\n", "badcam")
        with self.assertRaises(ProfileError):
            load_robot_profile(path)


class ViewerRunnerPureFunctionTests(unittest.TestCase):
    """显示入口的纯函数行为（不触碰图形会话）。"""

    def setUp(self):
        from iraf_adapters.mujoco import viewer_runner

        self.viewer_runner = viewer_runner
        self.profile = load_robot_profile(ROOT / "profiles/piper_mujoco.yaml")

    def test_camera_settings_from_profile(self):
        camera = self.viewer_runner.camera_settings(self.profile)
        self.assertEqual(camera["lookat_m"], [0.06, 0.0, 0.12])
        self.assertEqual(camera["distance_m"], 1.05)
        self.assertEqual(camera["azimuth_deg"], 180.0)
        self.assertEqual(camera["elevation_deg"], -8.0)

    def test_camera_settings_fallback_when_missing(self):
        bare = RobotProfile("r", "1.0.0", True, 100, ("j1",), {"j1": (0, 1)})
        camera = self.viewer_runner.camera_settings(
            bare, {"lookat_m": [1.0, 2.0, 3.0], "distance_m": 2.0}
        )
        self.assertEqual(camera["lookat_m"], [1.0, 2.0, 3.0])
        self.assertEqual(camera["distance_m"], 2.0)

    def test_target_tolerance_from_profile(self):
        tolerance = self.viewer_runner.target_tolerance(self.profile, "box_01")
        self.assertAlmostEqual(tolerance, 0.005)

    def test_target_tolerance_default_when_unknown(self):
        tolerance = self.viewer_runner.target_tolerance(self.profile, "nope", 0.02)
        self.assertAlmostEqual(tolerance, 0.02)

    def test_move_request_uses_move_joint_skill(self):
        from iraf_core.core import SafetyPolicy

        safety = SafetyPolicy(
            "lab", "1.0.0", "development", True,
            frozenset({"move_joint"}), 30000, "digest",
        )
        request = self.viewer_runner.build_move_request(
            self.profile, safety, {"joint1": 0.1}, 1000, "corr"
        )
        self.assertEqual(request["skill"], "move_joint")
        self.assertEqual(request["parameters"]["positions"], {"joint1": 0.1})
        self.assertEqual(request["profile_digest"], self.profile.digest)

    def test_pick_request_shape(self):
        from iraf_core.core import SafetyPolicy

        safety = SafetyPolicy(
            "lab", "1.0.0", "development", True,
            frozenset({"pick_object"}), 30000, "digest",
        )
        request = self.viewer_runner.build_pick_request(
            self.profile, safety, "box_01", [0.19, 0.0, 0.025], 12000, "corr"
        )
        self.assertEqual(request["skill"], "pick_object")
        pose = request["parameters"]["grasp_pose"]
        self.assertEqual(pose["frame_id"], "world")
        self.assertAlmostEqual(pose["position"]["x"], 0.19)
        self.assertEqual(
            set(pose["orientation"]), {"x", "y", "z", "w"}
        )


if __name__ == "__main__":
    unittest.main()
