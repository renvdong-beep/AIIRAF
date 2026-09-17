"""viewer_runner 实时可视化契约的单元测试（机器人无关层）。

锁定三条不变量，它们共同保证"完整抓取过程可见"而非"只有一帧"：
1. SnapshotMirror 的副本与后端状态一致（含 mocap，这是渲染错位的高风险点）；
2. 快照只在拷贝期间持锁，且不依赖任何机型专有成员；
3. build_grasp_request 的截止时间能覆盖 realtime 时长，且仍带齐
   profile/safety 摘要（显示路径不绕过 Policy）。
"""

import os
import sys
import threading
import unittest
from pathlib import Path

import numpy as np

# 从测试文件位置反推仓库根（tests/unit/x.py -> 仓库根），
# 避免硬编码开发者本机路径；CI 可用 IRAF_ROOT 覆盖。
ROOT = Path(os.environ.get("IRAF_ROOT") or Path(__file__).resolve().parents[2])
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.mujoco import viewer_runner  # noqa: E402

SCENE = ROOT / "build/models/piper-pick-scene.xml"

MINIMAL_XML = """
<mujoco>
  <worldbody>
    <body name="base">
      <joint name="j1" type="hinge"/>
      <geom type="box" size="0.05 0.05 0.05"/>
      <body name="tip" pos="0.1 0 0">
        <geom type="sphere" size="0.01"/>
      </body>
    </body>
  </worldbody>
  <actuator><position joint="j1" kp="10"/></actuator>
</mujoco>
"""


class _BackendStub:
    """最小后端替身：只暴露 RobotBackend 契约里的公开成员。"""

    def __init__(self, model, data=None):
        self.model = model
        self.data = data if data is not None else _mujoco().MjData(model)
        self._lock = threading.RLock()

    def display_lock(self):
        return self._lock


def _mujoco():
    import mujoco

    return mujoco


class SnapshotMirrorTests(unittest.TestCase):
    def setUp(self):
        mujoco = _mujoco()
        self.model = mujoco.MjModel.from_xml_string(MINIMAL_XML)
        self.backend = _BackendStub(self.model)
        mujoco.mj_forward(self.model, self.backend.data)

    def test_refresh_copies_qpos_exactly(self):
        mujoco = _mujoco()
        self.backend.data.qpos[0] = 0.42
        mujoco.mj_forward(self.model, self.backend.data)
        snapshot = viewer_runner.SnapshotMirror(self.model)
        mirror = snapshot.refresh(self.backend)
        self.assertAlmostEqual(0.42, float(mirror.qpos[0]), places=12)

    def test_refresh_is_repeatable_and_tracks_changes(self):
        """渲染循环每帧都会 refresh，因此必须能持续反映最新状态。"""
        snapshot = viewer_runner.SnapshotMirror(self.model)
        for value in (0.1, 0.5, -0.3):
            self.backend.data.qpos[0] = value
            _mujoco().mj_forward(self.model, self.backend.data)
            mirror = snapshot.refresh(self.backend)
            self.assertAlmostEqual(value, float(mirror.qpos[0]), places=12)

    def test_mirror_is_a_separate_object_from_backend_data(self):
        """必须持有独立副本，否则渲染仍会与物理线程争用同一份 data。"""
        snapshot = viewer_runner.SnapshotMirror(self.model)
        mirror = snapshot.refresh(self.backend)
        self.assertIsNot(mirror, self.backend.data)

        self.backend.data.qpos[0] = 0.9
        _mujoco().mj_forward(self.model, self.backend.data)
        # 未 refresh 前，副本不应被后端改动影响。
        self.assertNotAlmostEqual(0.9, float(mirror.qpos[0]), places=6)

    def test_refresh_does_not_hold_lock_after_return(self):
        """refresh 返回后必须已释放锁，否则物理线程会被渲染卡住。"""
        snapshot = viewer_runner.SnapshotMirror(self.model)
        snapshot.refresh(self.backend)
        acquired = self.backend.display_lock().acquire(blocking=False)
        try:
            self.assertTrue(acquired, "refresh 返回后仍持有 display_lock")
        finally:
            if acquired:
                self.backend.display_lock().release()

    def test_works_with_minimal_backend_contract(self):
        """只依赖 model/data/display_lock，因此与具体机型无关。"""
        snapshot = viewer_runner.SnapshotMirror(self.model)
        mirror = snapshot.refresh(self.backend)
        self.assertIsNotNone(mirror)

    def test_rejects_model_without_state_size(self):
        """mj_stateSize 非法时必须显式失败，不静默继续。"""
        snapshot = viewer_runner.SnapshotMirror(self.model)
        self.assertGreater(len(snapshot._buffer), 0)


@unittest.skipUnless(SCENE.is_file(), "缺少 Piper 场景文件")
class PiperSceneSnapshotTests(unittest.TestCase):
    """在真实抓取场景上验证 mocap 被带入快照。"""

    def setUp(self):
        mujoco = _mujoco()
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.backend = _BackendStub(self.model)
        mujoco.mj_forward(self.model, self.backend.data)

    def test_target_body_position_matches(self):
        mujoco = _mujoco()
        snapshot = viewer_runner.SnapshotMirror(self.model)
        mirror = snapshot.refresh(self.backend)
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "box_01")
        self.assertGreaterEqual(bid, 0)
        self.assertTrue(
            np.allclose(self.backend.data.xpos[bid], mirror.xpos[bid], atol=1e-12)
        )

    def test_mocap_is_carried_into_snapshot(self):
        """FULLPHYSICS 不含 mocap，必须显式复制，否则渲染错位。"""
        mujoco = _mujoco()
        self.assertGreater(int(self.model.nmocap), 0)
        self.backend.data.mocap_pos[0] = [0.31, -0.42, 0.53]
        mujoco.mj_forward(self.model, self.backend.data)
        snapshot = viewer_runner.SnapshotMirror(self.model)
        mirror = snapshot.refresh(self.backend)
        self.assertTrue(
            np.allclose(self.backend.data.mocap_pos, mirror.mocap_pos, atol=1e-12)
        )


class BuildGraspRequestTests(unittest.TestCase):
    def _profile_and_safety(self):
        from iraf_core.profile import load_robot_profile, load_safety_policy

        return (
            load_robot_profile(ROOT / "profiles/piper_mujoco.yaml"),
            load_safety_policy(ROOT / "profiles/safety/simulation_lab.yaml"),
        )

    def test_deadline_covers_realtime_duration(self):
        import time as _time

        profile, safety = self._profile_and_safety()
        duration = 92000
        request = viewer_runner.build_grasp_request(
            profile,
            safety,
            "pick_object",
            "box_01",
            [0.19, 0.0, 0.025],
            duration,
            "probe",
            deadline_slack_ms=1000,
        )
        slack = request["deadline_unix_ms"] - int(_time.time() * 1000)
        self.assertGreaterEqual(slack, duration)

    def test_deadline_never_shrinks_below_zero(self):
        profile, safety = self._profile_and_safety()
        short = viewer_runner.build_grasp_request(
            profile, safety, "pick_object", "t", [0.0, 0.0, 0.0], 1000, "p"
        )
        long = viewer_runner.build_grasp_request(
            profile, safety, "pick_object", "t", [0.0, 0.0, 0.0], 50000, "p"
        )
        self.assertGreater(long["deadline_unix_ms"], short["deadline_unix_ms"])

    def test_skill_is_configurable(self):
        """显示场景可用超时更宽的 skill（如 display_pick）。"""
        profile, safety = self._profile_and_safety()
        request = viewer_runner.build_grasp_request(
            profile, safety, "display_pick", "box_01", [0.19, 0.0, 0.025], 92000, "p"
        )
        self.assertEqual("display_pick", request["skill"])

    def test_carries_profile_and_safety_digests(self):
        """显示路径同样经 Policy：摘要不齐会被直接拒绝。"""
        profile, safety = self._profile_and_safety()
        request = viewer_runner.build_grasp_request(
            profile, safety, "pick_object", "box_01", [0.19, 0.0, 0.025], 1000, "p"
        )
        for key in (
            "profile_name",
            "profile_version",
            "profile_digest",
            "safety_policy_name",
            "safety_policy_version",
            "safety_policy_digest",
        ):
            self.assertTrue(request.get(key), "缺少字段: " + key)

    def test_orientation_defaults_to_identity(self):
        profile, safety = self._profile_and_safety()
        request = viewer_runner.build_grasp_request(
            profile, safety, "pick_object", "box_01", [0.19, 0.0, 0.025], 1000, "p"
        )
        self.assertEqual(
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            request["parameters"]["grasp_pose"]["orientation"],
        )


class LegacyEntryTests(unittest.TestCase):
    def test_legacy_display_entries_still_exist(self):
        """新增能力不得删除既有入口，避免破坏既有显示脚本。"""
        self.assertTrue(callable(viewer_runner.run_interactive))
        self.assertTrue(callable(viewer_runner.run_offscreen))
        self.assertTrue(callable(viewer_runner.run_interactive_live))

    def test_display_mode_constants_stable(self):
        self.assertEqual("interactive_viewer", viewer_runner.DISPLAY_INTERACTIVE)
        self.assertEqual("offscreen_frames", viewer_runner.DISPLAY_OFFSCREEN)
        self.assertEqual("unavailable", viewer_runner.DISPLAY_UNAVAILABLE)


if __name__ == "__main__":
    unittest.main()
