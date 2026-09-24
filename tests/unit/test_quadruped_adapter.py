"""四足通用契约与 Unitree Go2 适配器的契约测试（步骤 16）。

覆盖（每条负向用例旁边都配正向对照，否则分不清"门禁严格"与"门禁恒失败"）：
  1. 通用契约面平台无关：能力词表/公开方法参数/标准反馈键都不得含厂家内部标识；
  2. 能力契约 `declared ⊆ implemented`：方法存在 ≠ 能力已实现（Go2 首期无步态控制器，
     `locomote` 必须显式拒绝，且不得被写进能力声明）；
  3. 控制权：无租约、旧 fencing token、终态执行的控制请求一律拒绝；执行台账终态不可回写；
  4. 指令校验：未知关节名、越界目标、非有限数值、非法时长、未知速度字段一律拒绝；
  5. 装配 fail-closed：声明缺键/身份不一致/模型或关键帧不存在/限位放宽/控制频率不整除；
  6. 安全闭锁：急停不需租约、闭锁期间拒绝重新调度运动、停机不被闭锁阻塞、授权复位；
  7. 反漂移：`pd_torque` / `gravity_bias_torque` / 关节绑定与验收路径 `loopback` 逐位一致；
  8. `profile_check --quadruped` 的四向一致门禁（正反用例）。

夹具纪律：不读 `build/` 产物、不依赖本机是否跑过构建；模型用内联 MJCF 写到临时目录，
声明由仓库真实声明复制后覆盖路径（既覆盖"必需键集合"的漂移，又不依赖厂商资产）。

运行：`PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_adapter -v`
"""

import contextlib
import copy
import inspect
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import profile_check  # noqa: E402

from iraf_adapters.unitree import loopback, quadruped  # noqa: E402
from iraf_adapters.unitree.unitree_go2 import (  # noqa: E402
    UnitreeGo2Adapter,
    load_declaration,
    resolve_joint_bindings,
)
from iraf_core.authority import ControlAuthorityManager, LeaseConflict  # noqa: E402

REAL_DECLARATION = ROOT / "config/go2_loopback.yaml"

# 夹具关节：名字刻意保留"关节名 ≠ 执行器名"的厂家命名习惯（FL_hip_joint / FL_hip），
# 这是真实 go2.xml 的情况；Piper 恰好同名会掩盖这个差异。
KIT_JOINTS = ["FL_hip_joint", "FL_thigh_joint", "RR_hip_joint", "RR_thigh_joint"]
KIT_ACTUATORS = ["FL_hip", "FL_thigh", "RR_hip", "RR_thigh"]
#: 模型里声明的关节限位（Profile 只允许收紧到不超过它）。
KIT_MODEL_RANGE = (-1.0, 1.0)

# 合成四足：自由基座方块落在水平面上，四条绕 z 的铰链（重力不产生力矩 ⇒ 不与地面接触耦合）。
# 装饰连杆关闭碰撞（contype/conaffinity=0）：否则与基座互相穿透产生持续冲量，无阻尼铰链会自旋
# （步骤 15 夹具实测 a_joint 转到 −1776 rad），那是夹具缺陷不是被测逻辑。
KIT_XML = """<mujoco model="quadruped_adapter_fixture">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="base" pos="0 0 0.05">
      <freejoint name="root"/>
      <geom name="base_geom" type="box" size="0.05 0.05 0.05" mass="1.0"/>
      <site name="imu_fixture" pos="0 0 0" size="0.005"/>
      <body name="fl_link" pos="0.08 0 0">
        <joint name="FL_hip_joint" type="hinge" axis="0 0 1" range="-1 1" damping="0.2"/>
        <geom name="fl_hip_geom" type="box" size="0.02 0.02 0.02" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="fl_thigh_link" pos="-0.08 0 0">
        <joint name="FL_thigh_joint" type="hinge" axis="0 0 1" range="-1 1" damping="0.2"/>
        <geom name="fl_thigh_geom" type="box" size="0.02 0.02 0.02" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="rr_hip_link" pos="0 0.08 0">
        <joint name="RR_hip_joint" type="hinge" axis="0 0 1" range="-1 1" damping="0.2"/>
        <geom name="rr_hip_geom" type="box" size="0.02 0.02 0.02" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="rr_thigh_link" pos="0 -0.08 0">
        <joint name="RR_thigh_joint" type="hinge" axis="0 0 1" range="-1 1" damping="0.2"/>
        <geom name="rr_thigh_geom" type="box" size="0.02 0.02 0.02" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="FL_hip" joint="FL_hip_joint" ctrlrange="-5 5"/>
    <motor name="FL_thigh" joint="FL_thigh_joint" ctrlrange="-4 4"/>
    <motor name="RR_hip" joint="RR_hip_joint" ctrlrange="-5 5"/>
    <motor name="RR_thigh" joint="RR_thigh_joint" ctrlrange="-4 4"/>
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_fixture"/>
    <gyro name="imu_gyro" site="imu_fixture"/>
    <accelerometer name="imu_acc" site="imu_fixture"/>
    <jointactuatorfrc name="FL_hip_torque" joint="FL_hip_joint"/>
    <jointactuatorfrc name="FL_thigh_torque" joint="FL_thigh_joint"/>
    <jointactuatorfrc name="RR_hip_torque" joint="RR_hip_joint"/>
    <jointactuatorfrc name="RR_thigh_torque" joint="RR_thigh_joint"/>
  </sensor>
  <keyframe>
    <key name="home" qpos="0 0 0.05 1 0 0 0 0 0 0 0"/>
  </keyframe>
</mujoco>
"""


def _fixture_profile(capabilities=None, limited=None):
    limits = {joint: list(KIT_MODEL_RANGE) for joint in KIT_JOINTS}
    if limited:
        limits.update(limited)
    return {
        "apiVersion": "iraf.intewell.io/v1",
        "kind": "RobotProfile",
        "metadata": {"name": "fixture_quadruped", "version": "0.0.1"},
        "spec": {
            "simulation": True,
            "verification": "unverified",
            "control_frequency_hz": 100,
            "joints": list(KIT_JOINTS),
            "joint_limits": limits,
            "home": {joint: 0.0 for joint in KIT_JOINTS},
            "capabilities": list(capabilities or []),
        },
    }


def _set(document, dotted, value):
    """按点号路径写值（含"删除该键"用 _drop）。"""
    node = document
    parts = dotted.split(".")
    for part in parts[:-1]:
        node = node[part]
    node[parts[-1]] = value
    return document


def _drop(document, dotted):
    node = document
    parts = dotted.split(".")
    for part in parts[:-1]:
        node = node[part]
    node.pop(parts[-1], None)
    return document


class QuadrupedAdapterCases(unittest.TestCase):
    """夹具：临时目录里的模型 + Profile + 声明；每个用例都用新的 authority/lease。"""

    @classmethod
    def setUpClass(cls):
        cls._temp = tempfile.TemporaryDirectory()
        cls.tmp = Path(cls._temp.name)
        cls.model_path = cls.tmp / "fixture.xml"
        cls.model_path.write_text(KIT_XML, encoding="utf-8")
        cls.profile_path = cls.tmp / "fixture_profile.yaml"
        cls.profile_path.write_text(
            yaml.safe_dump(_fixture_profile(), allow_unicode=True), encoding="utf-8"
        )
        cls.declaration_path = cls.tmp / "fixture_declaration.yaml"
        cls._write_declaration(cls.declaration_path, _fixture_declaration(cls.tmp))
        cls.profile = _load_profile(cls.profile_path)

    @classmethod
    def tearDownClass(cls):
        cls._temp.cleanup()

    @staticmethod
    def _write_declaration(path, declaration):
        Path(path).write_text(
            yaml.safe_dump(declaration, allow_unicode=True), encoding="utf-8"
        )
        return Path(path)

    # ---- 夹具工具 ----
    def make_adapter(self, declaration_path=None):
        authority = ControlAuthorityManager()
        lease = authority.acquire("robot:fixture_quadruped", "test-owner", ttl_seconds=60.0)
        adapter = UnitreeGo2Adapter.from_config(
            declaration_path or self.declaration_path, self.profile, authority
        )
        return adapter, authority, lease

    def declaration_variant(self, mutate):
        """复制真实声明并做一处改动（负向用例），写进临时目录。"""
        declaration = _fixture_declaration(self.tmp)
        mutate(declaration)
        self.__class__._variant_seq = getattr(self.__class__, "_variant_seq", 0) + 1
        path = self.tmp / ("variant_%02d.yaml" % self.__class__._variant_seq)
        return self._write_declaration(path, declaration)

    # ---- 1. 通用契约面平台无关 ----
    def test_capability_vocabulary_is_portable(self):
        self.assertTrue(
            quadruped.assert_portable_surface("能力词表", quadruped.CAPABILITIES)
        )
        self.assertEqual(
            ("dock_for_handoff", "emergency_stop", "locomote", "read_state", "stand", "stop"),
            quadruped.CAPABILITIES,
        )

    def test_vendor_tokens_are_detected(self):
        """正向对照：门禁必须能真的命中厂家标识，否则它只是装饰。"""
        self.assertTrue(quadruped.vendor_tokens("FL_hip_joint"))
        self.assertTrue(quadruped.vendor_tokens("dds_domain"))
        self.assertTrue(quadruped.vendor_tokens("crc32"))
        self.assertEqual([], quadruped.vendor_tokens("joint_positions_rad"))
        with self.assertRaises(quadruped.DeclarationError):
            quadruped.assert_portable_surface("能力词表", ["stand", "FR_hip"])

    def test_public_method_signatures_avoid_vendor_tokens(self):
        names = []
        for method in ("stand", "stop", "locomote", "dock_for_handoff", "read_state",
                       "emergency_stop"):
            signature = inspect.signature(getattr(quadruped.QuadrupedAdapter, method))
            names.extend(signature.parameters)
        quadruped.assert_portable_surface("公开方法参数名", names)

    def test_state_keys_are_portable_and_complete(self):
        quadruped.assert_portable_surface("反馈顶层键", quadruped.STATE_KEYS)
        adapter, _authority, _lease = self.make_adapter()
        state = adapter.read_state()
        self.assertEqual(set(quadruped.STATE_KEYS), set(state))
        self.assertEqual(set(quadruped.IMU_KEYS), set(state["imu"]))

    def test_validate_state_rejects_missing_extra_and_vendor_keys(self):
        adapter, _authority, _lease = self.make_adapter()
        state = adapter.read_state()
        for mutate in (
            lambda s: s.pop("joint_torque_nm"),
            lambda s: s.update({"extra": 1}),
            lambda s: s.update({"FL_hip_joint": 0.0}),
            lambda s: s["imu"].pop("angular_velocity_rad_s"),
        ):
            broken = copy.deepcopy(state)
            mutate(broken)
            with self.assertRaises(quadruped.DeclarationError):
                quadruped.validate_state(broken)

    # ---- 2. 能力契约 ----
    def test_go2_implements_only_verified_capabilities(self):
        # `locomote` 于 2026-09-23 两层验收达标后入列（技能层报告 build/iraf-a6a12/skill-layer-run.json）
        self.assertEqual(
            {"emergency_stop", "read_state", "stand", "stop", "locomote", "dock_for_handoff"},
            set(UnitreeGo2Adapter.IMPLEMENTED_CAPABILITIES),
        )

    def test_capability_contract_accepts_declared_subset(self):
        report = quadruped.verify_capabilities(
            ["stand", "stop", "read_state"], UnitreeGo2Adapter
        )
        self.assertTrue(report["passed"])
        self.assertEqual(
            ["dock_for_handoff", "emergency_stop", "locomote"],
            report["undeclared_implemented_capabilities"]
        )

    def test_capability_contract_rejects_unimplemented_and_unknown(self):
        class _StubAdapter:
            """只实现 `stand` 的桩：Go2 已把词表内能力全部实现，用它保留"声明未实现能力必须被拒"的载体。"""

            IMPLEMENTED_CAPABILITIES = frozenset({"stand"})

        with self.assertRaises(quadruped.CapabilityContractError) as context:
            quadruped.verify_capabilities(["stand", "locomote"], _StubAdapter)
        self.assertIn("locomote", str(context.exception))
        with self.assertRaises(quadruped.CapabilityContractError) as context:
            quadruped.verify_capabilities(["teleport"], UnitreeGo2Adapter)
        self.assertIn("teleport", str(context.exception))

    def test_factory_contract_one_way_semantics_unchanged(self):
        """装配期"声明必须实现"的单向语义不变；本步骤只是把四足能力登记进词表。"""
        from iraf_adapters.factory import (
            BackendContractError,
            CAPABILITY_METHODS,
            KNOWN_BACKENDS,
            verify_backend_contract,
        )

        for capability in quadruped.CAPABILITIES:
            self.assertIn(capability, CAPABILITY_METHODS)
        self.assertEqual(
            "iraf_adapters.unitree.unitree_go2:UnitreeGo2Adapter",
            KNOWN_BACKENDS["unitree_go2_mujoco"],
        )
        profile = _load_profile(self.profile_path, capabilities=["stand", "stop"])
        report = verify_backend_contract(UnitreeGo2Adapter, profile)
        self.assertTrue(report["passed"])
        arm_profile = _load_profile(self.profile_path, capabilities=["pick_object"])
        with self.assertRaises(BackendContractError):
            verify_backend_contract(UnitreeGo2Adapter, arm_profile)

    # ---- 3. 控制权与执行台账 ----
    def test_stand_with_lease_succeeds(self):
        adapter, _authority, lease = self.make_adapter()
        report = adapter.stand(lease, duration_ms=100)
        self.assertEqual("stand", report["capability"])
        self.assertTrue(report["simulation"])
        self.assertGreater(report["control_cycles"], 0)
        self.assertEqual("SUCCEEDED", adapter.ledger.entry(report["execution_id"])["state"])
        self.assertEqual("model", report["torque_limit_source"])
        self.assertEqual(set(quadruped.STATE_KEYS), set(report["final_state"]))
        # 目标位形来自 Profile 的 home（单一事实来源），不是代码里的默认值。
        self.assertEqual("profile_home", report["target_source"])

    def test_stand_without_lease_is_rejected(self):
        adapter, _authority, _lease = self.make_adapter()
        with self.assertRaises(quadruped.ControlAuthorityError):
            adapter.stand(None, duration_ms=20)

    def test_stale_fencing_token_is_rejected(self):
        """旧 token 必须被拒绝：另一个控制源接管后，旧租约不得再驱动执行器。"""
        adapter, authority, lease = self.make_adapter()
        authority.release(lease)
        authority.acquire("robot:fixture_quadruped", "other-owner", ttl_seconds=60.0)
        with self.assertRaises(quadruped.ControlAuthorityError):
            adapter.stand(lease, duration_ms=20)

    def test_terminal_execution_rejects_more_control(self):
        adapter, _authority, lease = self.make_adapter()
        report = adapter.stand(lease, duration_ms=20)
        with self.assertRaises(quadruped.TerminalExecutionError):
            adapter.stand(lease, duration_ms=20, execution_id=report["execution_id"])

    def test_wrong_capability_resumption_is_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        report = adapter.stand(lease, duration_ms=20)
        with self.assertRaises(quadruped.CommandRejectedError):
            adapter.stop(lease, execution_id=report["execution_id"])

    def test_unknown_execution_id_is_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        with self.assertRaises(quadruped.CommandRejectedError):
            adapter.stop(lease, execution_id="exec-9999")

    def test_ledger_terminal_state_cannot_be_rewritten(self):
        ledger = quadruped.ExecutionLedger()
        execution_id = ledger.begin("stand", 3)
        ledger.finish(execution_id, "SUCCEEDED")
        with self.assertRaises(quadruped.TerminalExecutionError):
            ledger.finish(execution_id, "SUCCEEDED")
        with self.assertRaises(quadruped.TerminalExecutionError):
            ledger.assert_active(execution_id)
        with self.assertRaises(quadruped.DeclarationError):
            quadruped.ExecutionLedger().finish("exec-0001", "RUNNING")

    def test_execution_sequence_is_monotonic(self):
        adapter, _authority, lease = self.make_adapter()
        first = adapter.stand(lease, duration_ms=20)
        second = adapter.stand(lease, duration_ms=20)
        self.assertEqual(1, adapter.ledger.entry(first["execution_id"])["sequence"])
        self.assertEqual(2, adapter.ledger.entry(second["execution_id"])["sequence"])

    def test_authority_manager_rejects_second_owner(self):
        """控制权唯一性由 ControlAuthorityManager 保证（适配器不另造一套）。"""
        authority = ControlAuthorityManager()
        authority.acquire("robot:fixture_quadruped", "owner-a", ttl_seconds=60.0)
        with self.assertRaises(LeaseConflict):
            authority.acquire("robot:fixture_quadruped", "owner-b", ttl_seconds=60.0)

    # ---- 4. 指令校验 ----
    def test_unknown_joint_name_is_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        with self.assertRaises(quadruped.CommandRejectedError) as context:
            adapter.stand(lease, targets={"FL_tail_joint": 0.1}, duration_ms=20)
        self.assertIn("未知关节名", str(context.exception))

    def test_out_of_range_target_is_rejected_but_boundary_is_accepted(self):
        """目标只能收紧不能放宽：越界拒绝；恰好取到限位边界必须通过（正向对照）。"""
        adapter, _authority, lease = self.make_adapter()
        with self.assertRaises(quadruped.CommandRejectedError):
            adapter.stand(lease, targets={"FL_hip_joint": 1.5}, duration_ms=20)
        boundary = adapter.stand(
            lease, targets={"FL_hip_joint": 1.0}, duration_ms=20
        )
        self.assertEqual(1.0, boundary["joint_targets_rad"]["FL_hip_joint"])

    def test_non_finite_target_is_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        for value in (float("nan"), float("inf"), "abc"):
            with self.assertRaises(quadruped.CommandRejectedError):
                adapter.stand(lease, targets={"FL_hip_joint": value}, duration_ms=20)

    def test_partial_targets_fall_back_to_profile_home(self):
        """部分目标：未指定的关节回落到 Profile 的 spec.home（声明来源），不是当前位置。"""
        adapter, _authority, lease = self.make_adapter()
        report = adapter.stand(lease, targets={"FL_hip_joint": 0.4}, duration_ms=20)
        self.assertEqual("explicit_partial", report["target_source"])
        self.assertEqual(0.4, report["joint_targets_rad"]["FL_hip_joint"])
        for joint in KIT_JOINTS:
            if joint == "FL_hip_joint":
                continue
            self.assertEqual(0.0, report["joint_targets_rad"][joint])

    def test_velocity_command_shape_is_validated(self):
        adapter, _authority, _lease = self.make_adapter()
        parsed = adapter.resolve_velocity({"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0})
        self.assertEqual({"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}, parsed)
        for bad in (
            {"vx_mps": 0.1, "vy_mps": 0.0},
            {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0, "yaw_rate": 1.0},
            {"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": float("nan")},
            {},
        ):
            with self.assertRaises(quadruped.CommandRejectedError):
                adapter.resolve_velocity(bad)

    def test_duration_validation(self):
        adapter, _authority, lease = self.make_adapter()
        for bad in (0, -1, float("nan")):
            with self.assertRaises(quadruped.CommandRejectedError):
                adapter.stand(lease, duration_ms=bad)
        defaulted = adapter.stand(lease)
        self.assertAlmostEqual(
            float(adapter.declaration["stand"]["duration_s"]) * 1000.0,
            defaulted["duration_ms"],
            places=6,
        )

    # ---- 5. locomote：已接 MPC 路径，接线输入必须来自声明 ----
    def test_locomote_requires_declared_wiring(self):
        """`locomote` 不再是"显式拒绝"：它走 MPC Provider 正式路径（A6a-④ ⑤）。

        本 fixture 的声明里**没有** `locomote` 段 ⇒ 必须**显式失败**（Provider 配置与默认时长
        只能来自声明，缺项不得静默取默认值）。
        能力面不变：Profile 的 `capabilities` 仍只有 [stand, stop]（能力回填须过本机判据 +
        aarch64 板复测，铁律 6.8）⇒ "未声明能力却调用"仍由能力契约层拦下。
        """
        adapter, _authority, lease = self.make_adapter()
        # 本 fixture 的 Profile 关节名与真实机型不同 ⇒ 接线在 trot 声明合并/校验处**显式失败**
        # （关节身份只能来自 Profile；这正是"缺声明即失败、不静默兜底"的表现）。
        with self.assertRaises(quadruped.DeclarationError):
            adapter.locomote({"vx_mps": 0.1, "vy_mps": 0.0, "wz_rad_s": 0.0}, 200, lease)
        # 指令形状非法时按"指令被拒绝"返回：可区分"乱下指令"与"接线未声明"。
        with self.assertRaises(quadruped.CommandRejectedError):
            adapter.locomote({"vx_mps": 0.1}, 200, lease)

    def test_base_class_rejects_everything_it_did_not_implement(self):
        authority = ControlAuthorityManager()

        class _Empty(quadruped.QuadrupedAdapter):
            pass

        adapter = _Empty({"simulation": True, "control": {"torque_limit_source": "model"}},
                         self.profile, authority)
        for call in (
            lambda: adapter.read_state(),
            lambda: adapter.stand(None),
            lambda: adapter.locomote({}, 1, None),
        ):
            with self.assertRaises(quadruped.UnsupportedCapabilityError):
                call()

    # ---- 6. 急停与安全闭锁 ----
    def test_emergency_stop_needs_no_lease_and_latches(self):
        adapter, _authority, lease = self.make_adapter()
        ack = adapter.emergency_stop("测试急停")
        self.assertFalse(ack["lease_required"])
        self.assertTrue(ack["torque_released"])
        self.assertFalse(ack["stepped"])
        self.assertEqual("adapter_state_only", ack["evidence_scope"])
        self.assertTrue(adapter.emergency_stop_confirmed()["confirmed"])
        with self.assertRaises(quadruped.EmergencyStopLatchedError):
            adapter.stand(lease, duration_ms=20)

    def test_emergency_stop_requires_reason(self):
        adapter, _authority, _lease = self.make_adapter()
        for reason in (None, "", "   "):
            with self.assertRaises(quadruped.DeclarationError):
                adapter.emergency_stop(reason)

    def test_stop_is_not_blocked_by_latch_but_does_not_clear_it(self):
        """停机是安全方向的动作，不被闭锁阻塞；但它不解除闭锁（不重新放行运动）。"""
        adapter, _authority, lease = self.make_adapter()
        adapter.emergency_stop("测试急停")
        report = adapter.stop(lease)
        self.assertEqual("STOPPED", adapter.ledger.entry(report["execution_id"])["state"])
        self.assertTrue(adapter.estop.engaged)
        with self.assertRaises(quadruped.EmergencyStopLatchedError):
            adapter.stand(lease, duration_ms=20)

    def test_clear_emergency_stop_requires_lease_and_authorization(self):
        adapter, _authority, lease = self.make_adapter()
        adapter.emergency_stop("测试急停")
        with self.assertRaises(quadruped.ControlAuthorityError):
            adapter.clear_emergency_stop(None, "控制器已确认 safe state")
        with self.assertRaises(quadruped.ControlAuthorityError):
            adapter.clear_emergency_stop(lease, "")
        cleared = adapter.clear_emergency_stop(lease, "控制器已确认 safe state")
        self.assertTrue(cleared["cleared"])
        self.assertFalse(adapter.estop.engaged)
        resumed = adapter.stand(lease, duration_ms=20)
        self.assertEqual("stand", resumed["capability"])

    def test_clear_without_latch_is_rejected(self):
        adapter, _authority, lease = self.make_adapter()
        with self.assertRaises(quadruped.CommandRejectedError):
            adapter.clear_emergency_stop(lease, "无闭锁时的复位请求")

    # ---- 7. 装配 fail-closed ----
    def test_missing_declaration_key_fails_closed(self):
        path = self.declaration_variant(lambda d: _drop(d, "control.kp_nm_per_rad"))
        authority = ControlAuthorityManager()
        with self.assertRaises(quadruped.DeclarationError) as context:
            UnitreeGo2Adapter.from_config(path, self.profile, authority)
        self.assertIn("control.kp_nm_per_rad", str(context.exception))

    def test_robot_id_profile_mismatch_is_rejected(self):
        path = self.declaration_variant(lambda d: _set(d, "robot.id", "other_robot"))
        with self.assertRaises(quadruped.DeclarationError) as context:
            UnitreeGo2Adapter.from_config(path, self.profile, ControlAuthorityManager())
        self.assertIn("robot.id", str(context.exception))

    def test_missing_model_is_rejected(self):
        path = self.declaration_variant(
            lambda d: _set(d, "model.file", str(self.tmp / "not_built" / "scene.xml"))
        )
        with self.assertRaises(quadruped.ModelUnavailableError) as context:
            UnitreeGo2Adapter.from_config(path, self.profile, ControlAuthorityManager())
        self.assertIn("被测模型不存在", str(context.exception))

    def test_missing_keyframe_is_rejected(self):
        path = self.declaration_variant(lambda d: _set(d, "initial.keyframe", "nope"))
        with self.assertRaises(quadruped.ModelUnavailableError) as context:
            UnitreeGo2Adapter.from_config(path, self.profile, ControlAuthorityManager())
        self.assertIn("initial.keyframe", str(context.exception))

    def test_profiler_limits_may_not_exceed_model(self):
        profile = _load_profile(
            self.profile_path, limited={"FL_hip_joint": [-2.0, 2.0]}
        )
        with self.assertRaises(quadruped.DeclarationError) as context:
            UnitreeGo2Adapter.from_config(
                self.declaration_path, profile, ControlAuthorityManager()
            )
        self.assertIn("只能收紧", str(context.exception))

    def test_control_frequency_must_divide_timestep(self):
        path = self.declaration_variant(lambda d: _set(d, "control.frequency_hz", 300.0))
        with self.assertRaises(quadruped.DeclarationError) as context:
            UnitreeGo2Adapter.from_config(path, self.profile, ControlAuthorityManager())
        self.assertIn("不整除", str(context.exception))

    def test_simulation_and_torque_source_are_enforced(self):
        for mutate, needle in (
            (lambda d: _set(d, "simulation", False), "simulation"),
            (lambda d: _set(d, "control.torque_limit_source", "config"), "torque_limit_source"),
            (lambda d: _set(d, "stop.mode", "hold_position"), "stop.mode"),
            (lambda d: _set(d, "stand.pose_source", "guessed"), "stand.pose_source"),
        ):
            path = self.declaration_variant(mutate)
            with self.assertRaises(quadruped.DeclarationError) as context:
                UnitreeGo2Adapter.from_config(path, self.profile, ControlAuthorityManager())
            self.assertIn(needle, str(context.exception))

    def test_config_accepts_declaration_mapping(self):
        """部署侧传进来的是 JSON dict：含 declaration 键时必须能装配（与 bootstrap 一致）。"""
        adapter, _authority, _lease = self.make_adapter(
            {"declaration": str(self.declaration_path)}
        )
        self.assertEqual("fixture_quadruped", adapter.profile.name)
        with self.assertRaises(quadruped.DeclarationError):
            load_declaration({"unknown": 1})

    # ---- 8. 反漂移：与验收路径逐位一致 ----
    def test_joint_binding_matches_acceptance_path(self):
        adapter, _authority, _lease = self.make_adapter()
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(self.model_path))
        reference = loopback.resolve_binding(model, mujoco, KIT_JOINTS)
        self.assertEqual(reference, adapter.bindings)
        legacy = resolve_joint_bindings(model, mujoco, KIT_JOINTS)
        self.assertEqual(reference, legacy)

    def test_pd_torque_matches_acceptance_path(self):
        rng = np.random.default_rng(20260920)
        lower = np.array([-5.0, -4.0])
        upper = np.array([5.0, 4.0])
        for _ in range(50):
            q = rng.normal(size=2)
            dq = rng.normal(size=2)
            q_des = rng.normal(size=2)
            tau_ff = rng.normal(size=2)
            expected, expected_saturated = loopback.compute_ctrl(
                q, dq, q_des, 150.0, 4.0, tau_ff, lower, upper
            )
            actual, actual_saturated = quadruped.pd_torque(
                q, dq, q_des, 150.0, 4.0, tau_ff, lower, upper
            )
            self.assertTrue(np.array_equal(expected, actual))
            self.assertTrue(np.array_equal(expected_saturated, actual_saturated))

    def test_gravity_bias_matches_acceptance_path(self):
        adapter, _authority, _lease = self.make_adapter()
        dofs = [adapter.bindings[joint]["dof_adr"] for joint in adapter.joint_order]
        expected = loopback.bias_torque(adapter.model, adapter.data, adapter.mujoco, dofs)
        actual = quadruped.gravity_bias_torque(
            adapter.model, adapter.data, adapter.mujoco, dofs
        )
        self.assertTrue(np.array_equal(expected, actual))

    def test_torque_limits_come_from_model(self):
        adapter, _authority, _lease = self.make_adapter()
        summary = adapter.describe()
        for index, joint in enumerate(adapter.joint_order):
            actuator_id = adapter.actuator_ids[index]
            expected = [
                float(adapter.model.actuator_ctrlrange[actuator_id][0]),
                float(adapter.model.actuator_ctrlrange[actuator_id][1]),
            ]
            self.assertEqual(expected, summary["torque_limits_nm"][joint])
        self.assertEqual("model", summary["control"]["torque_limit_source"])

    def test_joint_torque_feedback_matches_applied_ctrl(self):
        """力矩反馈与施加的控制量一致（motor 执行器 gear=1），不是与硬编码常数比。"""
        adapter, _authority, lease = self.make_adapter()
        adapter.stand(lease, targets={"FL_hip_joint": 0.5}, duration_ms=50)
        state = adapter.read_state()
        for index, joint in enumerate(adapter.joint_order):
            actuator_id = adapter.actuator_ids[index]
            expected = float(adapter.data.ctrl[actuator_id]) * float(
                adapter.model.actuator_gear[actuator_id][0]
            )
            self.assertAlmostEqual(expected, state["joint_torque_nm"][joint], places=9)

    def test_describe_reports_contract_and_control_source(self):
        adapter, _authority, lease = self.make_adapter()
        adapter.stand(lease, duration_ms=20)
        summary = adapter.describe()
        self.assertEqual("fixture_quadruped", summary["robot"])
        self.assertEqual(list(quadruped.CAPABILITIES), summary["capability_vocabulary"])
        self.assertEqual("test-owner", summary["control_source"]["owner"])
        self.assertEqual(1, summary["control_source"]["fencing_token"])
        self.assertEqual(5, summary["control"]["substeps_per_control"])


class ProfileCheckQuadrupedCases(unittest.TestCase):
    """`profile_check --quadruped` 的四向一致门禁（与适配器共用同一夹具）。"""

    @classmethod
    def setUpClass(cls):
        cls._temp = tempfile.TemporaryDirectory()
        cls.tmp = Path(cls._temp.name)
        cls.model_path = cls.tmp / "fixture.xml"
        cls.model_path.write_text(KIT_XML, encoding="utf-8")
        cls.profile_path = cls.tmp / "fixture_profile.yaml"
        cls.profile_path.write_text(
            yaml.safe_dump(_fixture_profile(), allow_unicode=True), encoding="utf-8"
        )
        cls.declaration_path = cls.tmp / "fixture_declaration.yaml"
        cls.declaration_path.write_text(
            yaml.safe_dump(_fixture_declaration(cls.tmp), allow_unicode=True),
            encoding="utf-8",
        )

    @classmethod
    def tearDownClass(cls):
        cls._temp.cleanup()

    def test_positive_control(self):
        report = profile_check.check_quadruped(self.declaration_path)
        self.assertTrue(report["passed"], report["failures"])
        self.assertEqual("fixture_quadruped", report["robot"])
        self.assertEqual(4, len(report["joints"]))
        self.assertEqual([], report["capabilities"]["declared"])
        self.assertTrue(report["backend_contract"]["passed"])
        self.assertTrue(report["capability_contract"]["passed"])

    def test_cli_json_contract(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = profile_check._main_quadruped(
                type("Args", (), {"quadruped": self.declaration_path})()
            )
        self.assertEqual(0, code)
        self.assertIn("QUADRUPED_PROFILE_CHECK_PASSED", stderr.getvalue())
        payload = json.loads(stdout.getvalue())
        self.assertTrue(payload["passed"])

    def test_profile_capabilities_must_be_implemented(self):
        profile_path = self.tmp / "fixture_profile_locomote.yaml"
        profile_path.write_text(
            yaml.safe_dump(
                _fixture_profile(capabilities=["stand", "teleport"]), allow_unicode=True
            ),
            encoding="utf-8",
        )
        declaration = _fixture_declaration(self.tmp)
        declaration["robot"]["profile"] = str(profile_path)
        path = self.tmp / "fixture_declaration_locomote.yaml"
        path.write_text(yaml.safe_dump(declaration, allow_unicode=True), encoding="utf-8")
        report = profile_check.check_quadruped(path)
        self.assertFalse(report["passed"])
        self.assertTrue(
            any("teleport" in message for message in report["failures"]),
            report["failures"],
        )

    def test_missing_model_and_missing_profile_fail_closed(self):
        declaration = _fixture_declaration(self.tmp)
        declaration["model"]["file"] = str(self.tmp / "absent" / "scene.xml")
        path = self.tmp / "fixture_declaration_no_model.yaml"
        path.write_text(yaml.safe_dump(declaration, allow_unicode=True), encoding="utf-8")
        report = profile_check.check_quadruped(path)
        self.assertFalse(report["passed"])
        self.assertTrue(any("模型" in message for message in report["failures"]))

        declaration = _fixture_declaration(self.tmp)
        declaration["robot"]["profile"] = str(self.tmp / "absent_profile.yaml")
        path = self.tmp / "fixture_declaration_no_profile.yaml"
        path.write_text(yaml.safe_dump(declaration, allow_unicode=True), encoding="utf-8")
        report = profile_check.check_quadruped(path)
        self.assertFalse(report["passed"])
        self.assertTrue(any("Profile 不存在" in message for message in report["failures"]))

    def test_missing_declaration_is_one_failure_not_a_traceback(self):
        report = profile_check.check_quadruped(self.tmp / "nope.yaml")
        self.assertFalse(report["passed"])
        self.assertEqual(1, len(report["failures"]))


def _fixture_declaration(tmp):
    """真实声明复制 + 夹具覆盖：既保证必需键集合随仓库声明同步，又不依赖厂商资产。"""
    declaration = copy.deepcopy(
        yaml.safe_load(REAL_DECLARATION.read_text(encoding="utf-8"))
    )
    declaration["robot"] = {
        "id": "fixture_quadruped",
        "profile": str(Path(tmp) / "fixture_profile.yaml"),
    }
    declaration["model"] = {
        "file": str(Path(tmp) / "fixture.xml"),
        "builder": str(Path(tmp) / "fixture.xml"),
    }
    declaration["stand"]["pose_source"] = "profile_home"
    declaration["stand"]["ramp_s"] = 0.05
    declaration["stand"]["duration_s"] = 0.2
    declaration["stop"]["duration_s"] = 0.05
    return declaration


def _load_profile(path, capabilities=None, limited=None):
    from iraf_core.profile import load_robot_profile

    if capabilities is None and limited is None:
        return load_robot_profile(path)
    document = copy.deepcopy(_fixture_profile(capabilities=capabilities, limited=limited))
    tmp = Path(tempfile.mkdtemp(prefix="iraf-fixture-profile-"))
    variant = tmp / "variant_profile.yaml"
    variant.write_text(yaml.safe_dump(document, allow_unicode=True), encoding="utf-8")
    return load_robot_profile(variant)


if __name__ == "__main__":
    unittest.main()
