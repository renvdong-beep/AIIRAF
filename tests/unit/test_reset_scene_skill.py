# -*- coding: utf-8 -*-
"""reset_scene 技能：契约、Provider 转发、能力登记、profile 声明。

为什么要有这条技能（2026-10-23 使用者口径）：
  「场景复位是不是也应该有一个 IRAF 的 skill，用来清理当前的某一个 mujoco 或者所有 mujoco」。
  以前只能重启 provider 进程复位（窗口也关了），开发时看不到画面。现在做成技能：
  走 TaskFlow → Skill Runtime → Policy → Provider，持租约、进事件库，且**进程内复位**（窗口保留）。
"""
import json
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator

from iraf_core.registry import SkillRegistry, load_skill_manifest
from iraf_adapters.remote.provider_service import _NO_LEASE, _POSITIONAL
from iraf_skills.common.motion import SkillRejected
from iraf_skills.common.scene import ResetSceneProvider

REPO = Path(__file__).resolve().parents[2]
SKILL_DIR = REPO / "skills" / "reset_scene"


class ResetSceneManifestTest(unittest.TestCase):
    def setUp(self):
        self.manifest = load_skill_manifest(SKILL_DIR / "skill.yaml")

    def test_required_fields_declared(self):
        self.assertEqual(("reset_scene",), self.manifest.requires)
        self.assertIn("safety.estop == false", self.manifest.preconditions)
        self.assertEqual("maintenance", self.manifest.safety_class)
        self.assertEqual(30, self.manifest.timeout_seconds)

    def test_natural_language_trigger(self):
        self.assertTrue(self.manifest.intent_trigger.get("natural_language"))

    def test_input_schema_scope(self):
        schema = json.loads((SKILL_DIR / "reset_scene.input.json").read_text(encoding="utf-8"))
        ok = Draft202012Validator(schema).is_valid({"scope": "all"})
        self.assertTrue(ok)
        self.assertTrue(Draft202012Validator(schema).is_valid(
            {"scope": "target", "target_id": "box_01"}))
        self.assertFalse(Draft202012Validator(schema).is_valid({"scope": "everything"}))

    def test_registry_loads_it(self):
        registry = SkillRegistry().load_directory(str(REPO / "skills"))
        self.assertIn("reset_scene", registry.names())
        trigger = registry.intent_contracts()["reset_scene"]["trigger"]
        self.assertTrue(trigger.get("natural_language"))


class ResetSceneProviderTest(unittest.TestCase):
    class _Backend:
        def __init__(self, result):
            self.result = result
            self.calls = []

        def reset_scene(self, scope="all", target_id=None, lease=None):
            self.calls.append((scope, target_id, lease))
            return self.result

    def test_forwards_to_backend(self):
        backend = self._Backend({"status": "SUCCEEDED", "scope": "all"})
        out = ResetSceneProvider(None, backend).execute({"scope": "all"}, lease="L1")
        self.assertEqual("SUCCEEDED", out["status"])
        self.assertEqual([("all", None, "L1")], backend.calls)

    def test_rejects_unconfirmed_result(self):
        """Backend 说没成功就不能当成功（不伪造）。"""
        backend = self._Backend({"status": "FAILED"})
        with self.assertRaises(SkillRejected):
            ResetSceneProvider(None, backend).execute({"scope": "all"}, lease="L1")

    def test_rejects_target_without_id(self):
        backend = self._Backend({"status": "SUCCEEDED", "scope": "target"})
        with self.assertRaises(SkillRejected):
            ResetSceneProvider(None, backend).execute({"scope": "target"}, lease="L1")

    def test_backend_without_capability_is_rejected(self):
        class _NoCap:
            pass

        with self.assertRaises(SkillRejected):
            ResetSceneProvider(None, _NoCap()).execute({"scope": "all"}, lease="L1")


class NothingButTaskLevelTest(unittest.TestCase):
    def test_capability_registered_with_lease(self):
        """reset_scene 必须持租约：与运行中任务并发会互相破坏。"""
        self.assertIn("reset_scene", _POSITIONAL)
        self.assertNotIn("reset_scene", _NO_LEASE)

    def test_profile_declares_capability(self):
        """装配期契约是 declared ⊆ implemented：profile 不声明就不会被派发。"""
        for name in ("piper_mujoco.yaml", "ur5_mujoco.yaml"):
            text = (REPO / "profiles" / name).read_text(encoding="utf-8")
            self.assertIn("reset_scene", text, "%s 没声明 reset_scene" % name)

    def test_profile_loads_with_dev_section(self):
        """profile 加了 dev 段后仍必须能被 load_robot_profile 正常解析。"""
        from iraf_core.profile import load_robot_profile
        profile = load_robot_profile(str(REPO / "profiles" / "piper_mujoco.yaml"))
        self.assertIn("reset_scene", tuple(profile.capabilities))

    def test_remote_backend_forwards(self):
        from iraf_adapters.remote.provider_backend import RemoteProviderBackend
        self.assertTrue(callable(getattr(RemoteProviderBackend, "reset_scene", None)))

    def test_all_profile_capabilities_are_registered(self):
        """profile 声明的每个 capability 都必须在 CAPABILITY_METHODS 里登记。

        0.3.15 第一次上板就是漏了这一步：profile 加了 reset_scene、backend 也实现了，
        但 factory 的"能力登记表"没登记 ⇒ 装配期直接
        `BackendContractError: ... reset_scene(需 <未登记的 capability>)`，板上 CLI 起不来。
        这条测试把"声明了却没人认"的悬空能力挡在提交前。
        """
        from iraf_adapters.factory import CAPABILITY_METHODS
        from iraf_core.profile import load_robot_profile

        checked = 0
        for path in sorted((REPO / "profiles").glob("*_mujoco.yaml")):
            profile = load_robot_profile(str(path))
            missing = [c for c in profile.capabilities if c not in CAPABILITY_METHODS]
            self.assertEqual([], missing, "%s 声明的能力未在 CAPABILITY_METHODS 登记：%s"
                             % (path.name, missing))
            checked += 1
        self.assertGreaterEqual(checked, 2, "没扫到 profile，测试失效")


class BaselineFallbackTest(unittest.TestCase):
    """无 keyframe 的场景必须用初始化快照复位。

    实测（AgentOS 会话 execution cd606a2d）：piper 抓取场景 nkey=0，
    第一版 reset_scene 只认 keyframe ⇒ 永远报"场景没有 keyframe，拒绝猜测初始姿态"。
    现在基准顺序是 keyframe → 初始化快照（世界刚加载时的 qpos/qvel）。
    """

    def _backend(self, nq=7):
        import numpy as np
        from iraf_adapters.mujoco.mujoco_backend import MujocoBackend

        backend = object.__new__(MujocoBackend)   # 绕过 __init__（不加载 MJCF）

        class _Model:
            nkey = 0
            njnt = 0

        class _Data:
            def __init__(self):
                self.qpos = np.arange(nq, dtype=float)
                self.qvel = np.zeros(nq)

        class _Auth:
            def validate(self, lease):
                self.lease = lease

        backend.model = _Model()
        backend.data = _Data()
        backend._initial_qpos = backend.data.qpos.copy()
        backend._initial_qvel = backend.data.qvel.copy()
        backend._plant_injected = False
        backend.authority = _Auth()
        backend._forward_readonly = lambda: None
        backend._reset_metrics = lambda: None
        return backend

    def test_all_uses_initial_snapshot_when_no_keyframe(self):
        import numpy as np
        backend = self._backend()
        expected = backend._initial_qpos.copy()
        backend.data.qpos[:] = 9.9          # 模拟抓取后世界已改变
        out = backend.reset_scene(scope="all", lease="L1")
        self.assertEqual("SUCCEEDED", out["status"])
        self.assertEqual("initial-snapshot", out["baseline"])
        self.assertTrue(np.allclose(backend.data.qpos, expected))

    def test_refuses_when_no_baseline_at_all(self):
        backend = self._backend()
        backend._initial_qpos = None
        with self.assertRaises(RuntimeError):
            backend.reset_scene(scope="all", lease="L1")

    def test_guest_backend_refuses(self):
        """共享世界里是 guest ⇒ 不许复位别人的世界。"""
        backend = self._backend()
        backend._plant_injected = True
        with self.assertRaises(RuntimeError):
            backend.reset_scene(scope="all", lease="L1")

    def test_safety_policy_allows_reset_scene(self):
        """default: deny 的策略必须显式放行，否则任务到不了 provider。"""
        policy = (REPO / "profiles/safety/simulation_lab.yaml").read_text(encoding="utf-8")
        self.assertIn("reset_scene", policy)


if __name__ == "__main__":
    unittest.main()
