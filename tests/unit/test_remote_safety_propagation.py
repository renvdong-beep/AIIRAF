# -*- coding: utf-8 -*-
"""跨机安全事件传导的用例（铁律 1.6 / 1.9）。

要证明的事：Provider/适配层报告的安全事件**必须在本侧也变成隔离**，
否则会出现"远端已隔离、板上还在继续调度"的静默不一致 ——
这类不一致比直接失败危险，因为它不报警。

三层各有用例：
  ① 服务端判定（`safety_state_from_inventory` 纯函数）
  ② 隔离锁存（`ExecutionRegistry.quarantine/quarantined`）
  ③ runtime 传导（backend 抛 `SafetyEventReported` ⇒ 本侧 QUARANTINED，且后续执行被拒）
"""
from __future__ import annotations

import time
import unittest
from pathlib import Path

from iraf_core.authority import ControlAuthorityManager
from iraf_core.core import RobotProfile, SafetyPolicy
from iraf_core.policy import AuthenticatedContext
from iraf_core.registry import SkillRegistry
from iraf_core.runtime import SkillRuntime
from iraf_core.safety import SafetyEventReported, SafetyState
from iraf_core.store import SqliteExecutionStore
from iraf_adapters.remote.provider_service import ExecutionRegistry, safety_state_from_inventory

RESOURCE = "robot-arm"


# ---------------- ① 服务端判定（纯函数） ----------------

class SafetyStateFromInventoryTest(unittest.TestCase):
    def test_estop_true_is_quarantined(self):
        state = safety_state_from_inventory({"safety": {"estop": True}, "mode": "simulation"})
        self.assertTrue(state["quarantined"])
        self.assertEqual(state["kind"], "ESTOP")

    def test_explicit_quarantine_flag_is_quarantined(self):
        state = safety_state_from_inventory({"safety": {"quarantined": True, "reason": "碰撞"}})
        self.assertTrue(state["quarantined"])
        self.assertEqual(state["reason"], "碰撞")

    def test_healthy_inventory_is_operational(self):
        state = safety_state_from_inventory({"safety": {"estop": False}, "mode": "simulation"})
        self.assertFalse(state["quarantined"])

    def test_inventory_without_safety_section_is_operational(self):
        """清单存在但没有 safety 段：不臆造隔离（真隔离必须由被控对象给出）。"""
        state = safety_state_from_inventory({"mode": "simulation"})
        self.assertFalse(state["quarantined"])


# ---------------- ② 隔离锁存 ----------------

class QuarantineLatchTest(unittest.TestCase):
    def setUp(self):
        self.registry = ExecutionRegistry(ControlAuthorityManager(), RESOURCE)

    def test_latch_survives_healthy_reads(self):
        """锁存是刻意的：不因'下一次读到好状态'自动解除（铁律 1.9 要求授权复位）。"""
        self.assertIsNone(self.registry.quarantined(RESOURCE))
        self.registry.quarantine(RESOURCE, "ESTOP", "急停已触发")
        self.assertIsNotNone(self.registry.quarantined(RESOURCE))
        self.assertEqual(self.registry.quarantined(RESOURCE)["kind"], "ESTOP")

    def test_quarantine_is_per_resource(self):
        self.registry.quarantine(RESOURCE, "ESTOP", "急停已触发")
        self.assertIsNone(self.registry.quarantined("other-arm"))


# ---------------- ③ runtime 传导（端到端，含真实 Skill/Policy/Store） ----------------

class ReportedSafetyEventBackend:
    """第一次调用抛远端安全事件；资源已被隔离后不应再被调用。"""

    def __init__(self, authority):
        self.authority = authority
        self.calls = 0

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "mode": "simulation"}

    def move_joint(self, positions, duration_ms, lease):
        self.calls += 1
        self.authority.validate(lease)
        raise SafetyEventReported(RESOURCE, "远端控制器未确认安全停止", "CANNOT_STOP")

    def stop(self, lease):
        return None


class SafetyEventPropagationTest(unittest.TestCase):
    def build(self):
        authority = ControlAuthorityManager()
        profile = RobotProfile("robot", "1.0.0", True, 100, ("j1",), {"j1": (-1, 1)},
                               frozenset({"move_joint", "stop"}), "development", "profile-digest")
        safety = SafetyPolicy("lab", "1.0.0", "development", True,
                              frozenset({"move_joint", "stop"}), 30000, "safety-digest")
        registry = SkillRegistry().load_directory(Path(__file__).resolve().parents[2] / "skills")
        backend = ReportedSafetyEventBackend(authority)
        runtime = SkillRuntime(profile, safety, backend, registry, authority,
                               SqliteExecutionStore(":memory:"))
        return runtime, backend

    def task(self, key):
        return {"request_id": "r-" + key, "correlation_id": "c-" + key, "idempotency_key": key,
                "skill": "move_joint", "skill_version_constraint": "1.0.0",
                "parameters": {"positions": {"j1": 0.2}, "duration_ms": 1000},
                "deadline_unix_ms": int(time.time() * 1000) + 5000,
                "resource_id": RESOURCE, "controller": "remote-provider",
                "profile_name": "robot", "profile_version": "1.0.0", "profile_digest": "profile-digest",
                "safety_policy_name": "lab", "safety_policy_version": "1.0.0",
                "safety_policy_digest": "safety-digest"}

    def test_reported_event_quarantines_local_resource(self):
        runtime, backend = self.build()
        owner = AuthenticatedContext("board", frozenset({"task.submit"}), "local")
        result = runtime.execute(self.task("k1"), owner)
        self.assertEqual(result["error_code"], "IRAF-SAFETY-QUARANTINED", result)
        self.assertEqual(backend.calls, 1)
        # 本侧资源确实进了隔离（不是只回了个错误码）
        self.assertEqual(runtime.safety.state(RESOURCE), SafetyState.QUARANTINED)

    def test_subsequent_task_is_refused_after_quarantine(self):
        """隔离之后再发任务必须被拒 —— 且**不得**再触达 Provider（backend.calls 不再增长）。"""
        runtime, backend = self.build()
        owner = AuthenticatedContext("board", frozenset({"task.submit"}), "local")
        runtime.execute(self.task("k1"), owner)
        again = runtime.execute(self.task("k2"), owner)
        self.assertEqual(again["error_code"], "IRAF-SAFETY-QUARANTINED", again)
        self.assertEqual(backend.calls, 1, "隔离后不得再驱动 Provider")


# ---------------- ④ servicer 级（Invoke 完整路径） ----------------
# 为什么必须有这一层：③ 只覆盖 runtime 侧；servicer 的 Invoke 曾经因为
# `envelope` 在赋值前被使用而整条调用报 NameError（实测踩到），单测当时没能拦下。

try:  # 桩是生成物：没跑过 gen_proto_stubs 的环境跳过这一层
    from google.protobuf.json_format import ParseDict
    from google.protobuf.timestamp_pb2 import Timestamp

    from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2
    _STUBS = True
except ImportError:  # pragma: no cover
    _STUBS = False


class _FakeBackend:
    def __init__(self, estop=False):
        self._estop = estop
        self.calls = 0

    def runtime_inventory(self):
        return {"safety": {"estop": self._estop}, "mode": "simulation"}

    def move_joint(self, positions, duration_ms, lease):
        self.calls += 1
        return {"ok": True}


class _FakeContext:
    def invocation_metadata(self):
        return (("authorization", "Bearer tok"),)

    def abort(self, code, message):
        raise RuntimeError("aborted: %s" % message)


def _call(capability="move_joint", correlation="c-1", fencing=1):
    call = provider_pb2.CapabilityCall(capability=capability)
    ParseDict({"positions": {"j1": 0.1}, "duration_ms": 100}, call.inputs)
    stamp = Timestamp()
    stamp.GetCurrentTime()
    stamp.seconds += 60
    call.lease.correlation_id = correlation
    call.lease.idempotency_key = correlation
    call.lease.resource_id = RESOURCE
    call.lease.fencing_token = fencing
    call.lease.deadline.CopyFrom(stamp)
    return call


@unittest.skipUnless(_STUBS, "需要 proto 桩（先跑 scripts/gen_proto_stubs.sh）")
class ServicerInvokeTest(unittest.TestCase):
    def build(self, estop=False):
        import types

        from iraf_adapters.remote.provider_service import build_servicer_class

        backend = _FakeBackend(estop)
        runtime = types.SimpleNamespace(
            backend=backend,
            authority=ControlAuthorityManager(),
            profile=types.SimpleNamespace(capabilities={"move_joint"}),
        )
        servicer_cls = build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2)
        return servicer_cls(runtime, "tok", subject="board-runtime", display_state={}), backend

    def test_invoke_ok_when_healthy(self):
        servicer, backend = self.build(estop=False)
        resp = servicer.Invoke(_call(), _FakeContext())
        self.assertTrue(resp.accepted, resp.error.message_zh)
        self.assertEqual(backend.calls, 1)

    def test_invoke_refused_when_estop_latched(self):
        """被控对象急停时：服务端必须拒绝，并回 IRAF-SAFETY-QUARANTINED（含 resource/kind/reason）。"""
        servicer, backend = self.build(estop=True)
        resp = servicer.Invoke(_call(), _FakeContext())
        self.assertFalse(resp.accepted)
        self.assertEqual(resp.error.code, "IRAF-SAFETY-QUARANTINED")
        self.assertEqual(dict(resp.error.details).get("resource"), RESOURCE)
        self.assertEqual(backend.calls, 0, "隔离时不得驱动被控对象")

    def test_quarantine_latches_even_after_estop_clears(self):
        """锁存语义：急停解除后**也不自动放行**（须显式复位，铁律 1.9）。"""
        servicer, backend = self.build(estop=True)
        self.assertFalse(servicer.Invoke(_call(), _FakeContext()).accepted)
        backend._estop = False
        resp = servicer.Invoke(_call(correlation="c-2", fencing=2), _FakeContext())
        self.assertFalse(resp.accepted, "锁存后即使状态恢复正常也不得放行")
        self.assertEqual(backend.calls, 0)


if __name__ == "__main__":
    unittest.main()
