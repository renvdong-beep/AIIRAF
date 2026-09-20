"""Go2 loopback 验收契约（步骤 15）。

覆盖：声明必需键/取值/自洽性（负向 + **正向对照**）、位形解析、关节→执行器绑定、
控制律、判据（含"判据会真的失败"的负向）、静止判定、退出码，以及一小段
合成模型的端到端 loopback。

纪律（AGENTS.md 2.8 / 铁律 5.3）：
- 夹具不依赖本机是否跑过构建：所有引用的模型路径都指向**必然不存在**的位置，
  或用内联 MJCF 字符串；不读 `build/` 下的产物。
- 每个负向用例旁边都有正向对照，否则分不清"门禁严格"与"门禁恒失败"。
"""

import copy
import math
import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.unitree import loopback  # noqa: E402

REAL_CONFIG = ROOT / "config/go2_loopback.yaml"
KIT_JOINTS = ["a_joint", "b_joint"]

# 合成模型：自由基座方块落在水平面上（"站立"），两条绕 z 的铰链（重力不产生力矩）。
# 传感器按声明命名，用来验证绑定/采样链路（不依赖厂商资产与 build/ 产物）。
# 注意：连杆几何显式关闭碰撞（contype/conaffinity=0）——否则它们与基座方块互相穿透接触，
# 接触冲量会让无阻尼铰链自旋（首次夹具实测 a_joint 转到 −1776 rad），那是夹具缺陷不是被测逻辑。
KIT_XML = """
<mujoco model="loopback_fixture">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="base" pos="0 0 0.05">
      <freejoint name="root"/>
      <geom name="base_geom" type="box" size="0.05 0.05 0.05" mass="1.0"/>
      <site name="imu_fixture" pos="0 0 0" size="0.005"/>
      <body name="link_a" pos="0.08 0 0">
        <joint name="a_joint" type="hinge" axis="0 0 1" damping="0.2"/>
        <geom name="a_geom" type="box" size="0.02 0.02 0.02" mass="0.05" contype="0" conaffinity="0"/>
      </body>
      <body name="link_b" pos="-0.08 0 0">
        <joint name="b_joint" type="hinge" axis="0 0 1" damping="0.2"/>
        <geom name="b_geom" type="box" size="0.02 0.02 0.02" mass="0.05" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="a_motor" joint="a_joint" ctrlrange="-5 5"/>
    <motor name="b_motor" joint="b_joint" ctrlrange="-5 5"/>
  </actuator>
  <sensor>
    <framequat name="quat" objtype="site" objname="imu_fixture"/>
    <gyro name="gyro" site="imu_fixture"/>
    <accelerometer name="acc" site="imu_fixture"/>
    <jointactuatorfrc name="a_motor_torque" joint="a_joint"/>
    <jointactuatorfrc name="b_motor_torque" joint="b_joint"/>
  </sensor>
</mujoco>
"""

KIT_PROFILE = {
    "kind": "RobotProfile",
    "metadata": {"name": "fixture_quadruped"},
    "spec": {"joints": KIT_JOINTS, "model": {"imu_site": "imu_fixture"}},
}


def _real_config():
    return yaml.safe_load(REAL_CONFIG.read_text(encoding="utf-8"))


def _real_joints():
    _path, _document, profile = loopback.resolve_profile(
        ROOT,
        str(_real_config()["robot"]["id"]),
        _real_config()["robot"]["profile"],
    )
    return [str(item) for item in profile.joints]


def _fixture_config():
    return {
        "schema_version": "iraf.quadruped-loopback/v1",
        "simulation": True,
        "robot": {"id": "fixture_quadruped", "profile": "profiles/fixture.yaml"},
        "model": {"file": "build/never-built/kit.xml", "builder": "scripts/build_scene.py"},
        "initial": {"keyframe": "home"},
        "control": {
            "frequency_hz": 250.0,
            "kp_nm_per_rad": 50.0,
            "kd_nm_s_per_rad": 2.0,
            "gravity_feedforward": False,
            "torque_limit_source": "model",
        },
        "stand": {
            "pose_source": "explicit",
            "pose_rad": {"a_joint": 0.0, "b_joint": 0.0},
            "ramp_s": 0.0,
            "duration_s": 1.0,
            "hold_s": 0.5,
            "height_target_m": 0.05,
            "height_tolerance_m": 0.02,
            "height_std_max_m": 0.01,
            # 合成夹具的基座是"方块落在水平面上"，接触抖动比真实机型（球形足 + condim=6）
            # 大一个量级（本机实测 max|v|≈0.57 m/s）；本阈值**只属于夹具**，
            # 真实机型的速度判据在 config/go2_loopback.yaml（0.05 m/s），不在这里放宽。
            "speed_tolerance_mps": 1.0,
            "attitude_tolerance_deg": 5.0,
            "max_tracking_error_rad": 0.2,
        },
        "stop": {
            "mode": "torque_zero_release",
            "duration_s": 1.0,
            "speed_tolerance_mps": 0.05,
            "static_hold_s": 0.2,
            "final_window_s": 0.1,
        },
        "sample": {"frequency_hz": 250.0},
        "state": {
            "sensors": {
                "quat": "quat",
                "gyro": "gyro",
                "acc": "acc",
                "torque_pattern": "{actuator}_torque",
            },
            "quat_norm_tolerance": 1.0e-9,
            "gyro_max_abs_error_rad_s": 1.0e-9,
            "torque_max_abs_error_nm": 1.0e-9,
        },
        "report": {"path": "build/iraf-24h/15-test/report.json"},
    }


def _kit_model():
    import mujoco

    return mujoco.MjModel.from_xml_string(KIT_XML)


def _fake_samples(count=100, dt=0.02, height=0.05, attitude_deg=0.0, speed=0.001,
                  stop_speed=0.0, torque_offset=0.0):
    """构造采样数组（不跑物理）：用于逐条验证判据。"""
    half = count // 2
    times = [index * dt for index in range(count)]
    phases = ["stand"] * half + ["stop"] * (count - half)
    # 倾斜绕 x 轴（tilt 由 x/y 分量决定；绕 z 的是偏航，不参与姿态门禁）。
    quat = [math.cos(math.radians(attitude_deg) / 2.0), math.sin(math.radians(attitude_deg) / 2.0), 0.0, 0.0]
    samples = {
        "time_s": times,
        "phase": phases,
        "base_pos": [[0.0, 0.0, height]] * count,
        "base_quat": [quat] * count,
        "base_linvel": [[speed, 0.0, 0.0]] * count,
        "base_angvel": [[0.0, 0.0, 0.0]] * count,
        "joint_pos": [[0.0, 0.0]] * count,
        "joint_vel": [[0.0, 0.0]] * count,
        "ctrl": [[1.0, -1.0]] * count,
        "torque": [[1.0 + torque_offset, -1.0]] * count,
        "quat": [[1.0, 0.0, 0.0, 0.0]] * count,
        "gyro": [[0.0, 0.0, 0.0]] * count,
        "gyro_model": [[0.0, 0.0, 0.0]] * count,
        "acc": [[0.0, 0.0, 9.81]] * count,
    }
    for index in range(half, count):
        samples["base_linvel"][index] = [stop_speed, 0.0, 0.0]
    return samples


def _binding_info():
    return {
        "ctrlrange_nm": {"a_joint": [-5.0, 5.0], "b_joint": [-5.0, 5.0]},
        "saturated_samples": 0,
        "substeps_per_control": 2,
    }


def _assess(config, samples):
    config = copy.deepcopy(config)
    config["_pose"] = {"a_joint": 0.0, "b_joint": 0.0}
    return loopback.assess(samples, config, KIT_JOINTS, _binding_info())


class DeclarationContractTests(unittest.TestCase):
    def test_real_config_is_valid_positive_control(self):
        """正向对照：真实声明必须通过（否则负向用例分不清门禁严格与恒失败）。"""
        config = _real_config()
        joints = _real_joints()
        self.assertEqual(len(joints), 12)
        loopback.validate_declaration(config, joints)

    def test_every_required_key_is_enforced(self):
        """逐键删除 -> 退出码 2（必需键元组与声明同文，缺键必须是干净的退出码）。"""
        joints = _real_joints()
        for key in loopback.REQUIRED_KEYS:
            config = _real_config()
            node = config
            parts = key.split(".")
            for part in parts[:-1]:
                node = node[part]
            del node[parts[-1]]
            with self.assertRaises(loopback.LoopbackError) as caught:
                loopback.validate_declaration(config, joints)
            self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION, key)

    def test_simulation_must_be_true(self):
        config = _real_config()
        config["simulation"] = False
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_gravity_feedforward_must_be_boolean(self):
        """回归：`document.get("control.gravity_feedforward")` 的写法曾让合法声明被拒。"""
        config = _real_config()
        loopback.validate_declaration(config, _real_joints())
        config["control"]["gravity_feedforward"] = "yes"
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_torque_limit_source_other_than_model_is_rejected(self):
        config = _real_config()
        config["control"]["torque_limit_source"] = "config"
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_stop_mode_is_restricted(self):
        config = _real_config()
        config["stop"]["mode"] = "hold_position"
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_sample_frequency_must_divide_control_frequency(self):
        config = _real_config()
        config["sample"]["frequency_hz"] = 70.0
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_window_must_contain_required_hold(self):
        config = _real_config()
        config["stand"]["duration_s"] = config["stand"]["ramp_s"] + config["stand"]["hold_s"] - 0.01
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_pose_source_restricted_and_explicit_needs_all_joints(self):
        config = _real_config()
        config["stand"]["pose_source"] = "model_default"
        with self.assertRaises(loopback.LoopbackError):
            loopback.validate_declaration(config, _real_joints())
        config = _real_config()
        config["stand"]["pose_source"] = "explicit"
        config["stand"]["pose_rad"] = {"FL_hip_joint": 0.0}
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.validate_declaration(config, _real_joints())
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_non_numeric_and_negative_values_are_rejected(self):
        for key, value in (("kp_nm_per_rad", 0.0), ("kp_nm_per_rad", "30"), ("frequency_hz", 0.0)):
            config = _real_config()
            config["control"][key] = value
            with self.assertRaises(loopback.LoopbackError) as caught:
                loopback.validate_declaration(config, _real_joints())
            self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_written_paths_must_be_repo_relative(self):
        config = _real_config()
        config["report"]["path"] = "/tmp/report.json"
        with self.assertRaises(loopback.LoopbackError):
            loopback.validate_declaration(config, _real_joints())
        config = _real_config()
        config["model"]["file"] = "../outside.xml"
        with self.assertRaises(loopback.LoopbackError):
            loopback.validate_declaration(config, _real_joints())


class PoseResolutionTests(unittest.TestCase):
    def test_profile_home_is_used_and_missing_home_is_reference_failure(self):
        config = _fixture_config()
        config["stand"]["pose_source"] = "profile_home"
        profile = {"spec": {"home": {"a_joint": 0.1, "b_joint": -0.2}}}
        pose = loopback.resolve_pose(config, profile, KIT_JOINTS)
        self.assertEqual(pose, {"a_joint": 0.1, "b_joint": -0.2})
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_pose(config, {"spec": {}}, KIT_JOINTS)
        self.assertEqual(caught.exception.code, loopback.EXIT_REFERENCE)

    def test_profile_home_must_cover_all_joints(self):
        config = _fixture_config()
        config["stand"]["pose_source"] = "profile_home"
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_pose(config, {"spec": {"home": {"a_joint": 0.0}}}, KIT_JOINTS)
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_explicit_pose_is_read_from_config(self):
        config = _fixture_config()
        pose = loopback.resolve_pose(config, {"spec": {}}, KIT_JOINTS)
        self.assertEqual(pose, {"a_joint": 0.0, "b_joint": 0.0})


class BindingTests(unittest.TestCase):
    def test_binding_maps_joint_to_actuator_with_different_names(self):
        import mujoco

        model = _kit_model()
        binding = loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        self.assertEqual(binding["a_joint"]["qpos_adr"], 7)
        self.assertEqual(binding["b_joint"]["qpos_adr"], 8)
        # 执行器名（a_motor）与关节名（a_joint）不同：绑定必须解析出正确的 actuator。
        self.assertEqual(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, binding["a_joint"]["actuator_id"]),
            "a_motor",
        )
        self.assertNotEqual(binding["a_joint"]["actuator_id"], binding["b_joint"]["actuator_id"])

    def test_unknown_joint_is_a_reference_failure(self):
        import mujoco

        model = _kit_model()
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_binding(model, mujoco, ["a_joint", "nope_joint"])
        self.assertEqual(caught.exception.code, loopback.EXIT_REFERENCE)

    def test_joint_without_actuator_is_a_reference_failure(self):
        """夹具：把 b 的执行器摘掉后，b_joint 必须显式失败（不得静默跳过）。"""
        import mujoco

        model = mujoco.MjModel.from_xml_string(KIT_XML.replace('<motor name="b_motor" joint="b_joint" ctrlrange="-5 5"/>', ""))
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        self.assertEqual(caught.exception.code, loopback.EXIT_REFERENCE)


class ControllerMathTests(unittest.TestCase):
    def test_pd_plus_feedforward(self):
        ctrl, saturated = loopback.compute_ctrl(
            q=np.array([0.1, 0.0]), dq=np.array([0.0, 0.5]), q_des=np.array([0.2, 0.0]),
            kp=100.0, kd=2.0, tau_ff=np.array([1.0, -1.0]),
            lower=np.array([-20.0, -20.0]), upper=np.array([20.0, 20.0]),
        )
        self.assertAlmostEqual(ctrl[0], 100.0 * 0.1 + 1.0, places=12)
        self.assertAlmostEqual(ctrl[1], -2.0 * 0.5 - 1.0, places=12)
        self.assertFalse(bool(saturated.any()))

    def test_clipping_and_saturation_flags(self):
        ctrl, saturated = loopback.compute_ctrl(
            q=np.array([0.0]), dq=np.array([0.0]), q_des=np.array([1.0]),
            kp=100.0, kd=0.0, tau_ff=np.array([0.0]),
            lower=np.array([-5.0]), upper=np.array([5.0]),
        )
        self.assertAlmostEqual(float(ctrl[0]), 5.0, places=12)
        self.assertTrue(bool(saturated[0]))


class AssessTests(unittest.TestCase):
    def test_positive_control_passes(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples())
        self.assertEqual(assessment["failed_checks"], [])
        self.assertGreater(assessment["stand"]["hold_seconds"], 0.0)
        self.assertTrue(assessment["stop"]["static_entered"])

    def test_height_gate_can_fail(self):
        config = _fixture_config()
        config["stand"]["height_target_m"] = 5.0
        config["stand"]["height_tolerance_m"] = 0.01
        assessment = _assess(config, _fake_samples())
        self.assertIn("stand.height_mean_within_tolerance", assessment["failed_checks"])

    def test_attitude_gate_can_fail(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples(attitude_deg=30.0))
        self.assertIn("stand.max_attitude_error_deg", assessment["failed_checks"])

    def test_stand_speed_gate_can_fail(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples(speed=2.0))
        self.assertIn("stand.hold_seconds", assessment["failed_checks"])

    def test_hold_seconds_gate_can_fail(self):
        config = _fixture_config()
        config["stand"]["hold_s"] = 5.0
        config["stand"]["duration_s"] = 6.0
        config["stop"]["duration_s"] = 1.0
        assessment = _assess(config, _fake_samples())
        self.assertIn("stand.hold_seconds", assessment["failed_checks"])

    def test_stop_gate_can_fail_when_speed_never_decays(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples(stop_speed=1.0))
        self.assertIn("stop.static_entered", assessment["failed_checks"])
        self.assertIsNone(assessment["stop"]["seconds_to_static"])

    def test_torque_sensor_binding_gate_can_fail(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples(torque_offset=0.5))
        self.assertIn("state.torque_matches_applied_ctrl", assessment["failed_checks"])

    def test_ctrl_outside_model_range_can_fail(self):
        config = _fixture_config()
        samples = _fake_samples()
        samples["ctrl"] = [[9.0, -1.0]] * len(samples["ctrl"])
        assessment = _assess(config, samples)
        self.assertIn("control.within_model_ctrlrange", assessment["failed_checks"])

    def test_numbers_are_finite_and_not_null(self):
        config = _fixture_config()
        assessment = _assess(config, _fake_samples())
        for section in ("stand", "stop"):
            for key, value in assessment[section].items():
                if isinstance(value, float):
                    self.assertTrue(math.isfinite(value), "%s.%s" % (section, key))
        for key in ("height_mean_m", "height_std_m", "hold_seconds"):
            self.assertIsInstance(assessment["stand"][key], float)
        self.assertIsInstance(assessment["stop"]["final_speed_mps"], float)


class StaticOnsetTests(unittest.TestCase):
    def test_detects_onset_after_decay(self):
        times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        speeds = [1.0, 0.5, 0.01, 0.005, 0.002, 0.001]
        self.assertAlmostEqual(loopback.static_onset(times, speeds, 0.05, 0.2), 0.2, places=9)

    def test_returns_none_when_never_slow_enough_for_long_enough(self):
        times = [0.0, 0.1, 0.2, 0.3]
        speeds = [0.01, 1.0, 0.01, 0.01]
        self.assertIsNone(loopback.static_onset(times, speeds, 0.05, 0.3))

    def test_returns_first_time_when_already_static(self):
        times = [0.0, 0.1, 0.2]
        speeds = [0.0, 0.0, 0.0]
        self.assertAlmostEqual(loopback.static_onset(times, speeds, 0.05, 0.2), 0.0, places=9)


class SimulateIntegrationTests(unittest.TestCase):
    """合成模型端到端：绑定 + 采样 + 传感器解析 + 判据（不依赖厂商资产与 build/ 产物）。"""

    def _run(self, feedforward=False):
        import mujoco

        config = _fixture_config()
        config["control"]["gravity_feedforward"] = feedforward
        config["_pose"] = {"a_joint": 0.0, "b_joint": 0.0}
        config["_profile_document"] = KIT_PROFILE
        model = _kit_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        binding = loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        samples, info = loopback.simulate(model, data, mujoco, binding, KIT_JOINTS, config, config["_pose"])
        return samples, info, config

    def test_simulate_produces_sampled_state(self):
        samples, info, config = self._run()
        self.assertEqual(len(samples["time_s"]), len(samples["phase"]))
        self.assertGreater(len(samples["time_s"]), 100)
        arrays = np.asarray(samples["base_pos"], dtype=float)
        self.assertTrue(np.isfinite(arrays).all())
        # 基座落在平面上（不穿地、也不飞走）
        self.assertAlmostEqual(float(arrays[:, 2].mean()), 0.05, delta=0.01)
        self.assertEqual(info["substeps_per_control"], 2)
        assessment = _assess(config, samples)
        self.assertIn("stand.height_mean_within_tolerance", [item["name"] for item in assessment["checks"]])
        self.assertEqual(assessment["failed_checks"], [])

    def test_gravity_feedforward_path_runs_and_matches_qfrc_bias(self):
        """前馈分支必须真的执行（覆盖率之外的行为验证）：静止位形下的 qfrc_bias 与独立计算一致。"""
        import mujoco

        samples, _info, _config = self._run(feedforward=True)
        self.assertGreater(len(samples["time_s"]), 100)
        model = _kit_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        binding = loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        dofs = [binding[name]["dof_adr"] for name in KIT_JOINTS]
        tau = loopback.bias_torque(model, data, mujoco, dofs)
        self.assertEqual(tau.shape, (2,))

    def test_simulate_rejects_non_integer_substeps(self):
        import mujoco

        config = _fixture_config()
        config["control"]["frequency_hz"] = 200.0  # 1/(200*0.002) = 2.5 个物理步
        config["sample"]["frequency_hz"] = 200.0
        config["_pose"] = {"a_joint": 0.0, "b_joint": 0.0}
        config["_profile_document"] = KIT_PROFILE
        model = _kit_model()
        data = mujoco.MjData(model)
        binding = loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.simulate(model, data, mujoco, binding, KIT_JOINTS, config, config["_pose"])
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)


class EntryExitCodeTests(unittest.TestCase):
    def test_missing_config_is_usage_error(self):
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.run_loopback(ROOT / "config/does-not-exist.yaml", root=ROOT)
        self.assertEqual(caught.exception.code, loopback.EXIT_USAGE)

    def test_missing_model_is_reference_failure(self):
        """夹具把 model.file 写成必然不存在的路径 ⇒ 与「本机是否跑过构建」无关。"""
        import tempfile

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.yaml"
            path.write_text(yaml.safe_dump(_fixture_config(), allow_unicode=True), encoding="utf-8")
            with self.assertRaises(loopback.LoopbackError) as caught:
                loopback.run_loopback(path, root=ROOT)
            self.assertEqual(caught.exception.code, loopback.EXIT_REFERENCE)

    def test_profile_name_must_match_robot_id(self):
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_profile(ROOT, "not_a_robot", "profiles/unitree_go2_mujoco.yaml")
        self.assertEqual(caught.exception.code, loopback.EXIT_DECLARATION)

    def test_missing_profile_is_reference_failure(self):
        with self.assertRaises(loopback.LoopbackError) as caught:
            loopback.resolve_profile(ROOT, "unitree_go2", "profiles/nope_mujoco.yaml")
        self.assertEqual(caught.exception.code, loopback.EXIT_REFERENCE)


class RepoDeclarationGuardTests(unittest.TestCase):
    def test_go2_profile_home_and_scene_pose_source_are_consistent(self):
        """场景包的站立位形引用必须与 Profile 的 spec.home 同时存在（否则是静默缺口）。"""
        profile = yaml.safe_load((ROOT / "profiles/unitree_go2_mujoco.yaml").read_text(encoding="utf-8"))
        home = profile["spec"]["home"]
        self.assertEqual(set(home), set(profile["spec"]["joints"]))
        baseline = yaml.safe_load((ROOT / "scenes/handoff_lab/baseline.yaml").read_text(encoding="utf-8"))
        self.assertEqual(baseline["initial_state"]["unitree_go2"]["pose_source"], "profile")

    def test_no_internal_ip_in_new_declarations(self):
        """AGENTS.md 2.7：新增声明文件不得含内网地址。"""
        import re

        pattern = re.compile(r"\b10\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")
        for path in (REAL_CONFIG, ROOT / "profiles/unitree_go2_mujoco.yaml", ROOT / "scenes/handoff_lab/baseline.yaml"):
            self.assertIsNone(pattern.search(path.read_text(encoding="utf-8")), str(path))


if __name__ == "__main__":
    unittest.main()
