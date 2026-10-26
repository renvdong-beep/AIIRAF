# -*- coding: utf-8 -*-
"""Provider 侧**执行域租约**必须在动作结束（成功/拒绝/失败）后放开。

为什么必须有用例（2026-10-26 实测）：`CapabilityProviderServicer.Invoke` 原先只在
`except Exception` 里调 `ExecutionRegistry.release`，**成功路径没有释放** ⇒
`reset_scene` 成功后同一 resource 仍被服务端本地 `ControlAuthorityManager` 占着
`ttl_seconds=120`，紧随其后的 `pick_object` 被判 `IRAF-RESOURCE-BUSY`
（`resource busy: robot-arm`，剩余 120.0 s）。用户口径「复位 → 立刻抓要一步到位」。

本文件走完整 `servicer.Invoke` 路径（真 proto 桩，不启端口、不驱动 MuJoCo）。
没有桩的环境自动跳过（与 test_remote_safety_propagation.py 同一约定）。
"""
from __future__ import annotations

import types
import unittest

from iraf_core.authority import ControlAuthorityManager
from iraf_adapters.remote.provider_service import CapabilityRejected

RESOURCE = "robot-arm"

try:
    from google.protobuf.json_format import ParseDict
    from google.protobuf.timestamp_pb2 import Timestamp

    from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2
    _STUBS = True
except ImportError:  # pragma: no cover
    _STUBS = False


class _Backend:
    """最小 Provider：只实现本用例用到的两个能力。"""

    def __init__(self, fail_on=None):
        self.calls = []
        self._fail_on = fail_on

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "mode": "simulation"}

    def reset_scene(self, scope="all", target_id=None, lease=None):
        self.calls.append("reset_scene")
        return {"ok": True, "reset": True}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease=None):
        self.calls.append("pick_object")
        if self._fail_on == "pick_object":
            raise RuntimeError("夹爪不可用（模拟执行故障）")
        return {"ok": True, "picked": target_id}


class _Context:
    def invocation_metadata(self):
        return (("authorization", "Bearer tok"),)

    def abort(self, code, message):
        raise RuntimeError("aborted: %s" % message)


def _call(capability, inputs, key, fencing=0, resource=RESOURCE):
    call = provider_pb2.CapabilityCall(capability=capability)
    ParseDict(inputs, call.inputs)
    stamp = Timestamp()
    stamp.GetCurrentTime()
    stamp.seconds += 60
    call.lease.correlation_id = key
    call.lease.idempotency_key = key
    call.lease.resource_id = resource
    call.lease.fencing_token = fencing
    call.lease.deadline.CopyFrom(stamp)
    return call


@unittest.skipUnless(_STUBS, "需要 proto 桩（先跑 scripts/gen_proto_stubs.sh）")
class LeaseReleasedPerExecutionTest(unittest.TestCase):
    def build(self, fail_on=None):
        from iraf_adapters.remote.provider_service import build_servicer_class

        backend = _Backend(fail_on)
        authority = ControlAuthorityManager()
        runtime = types.SimpleNamespace(
            backend=backend,
            authority=authority,
            profile=types.SimpleNamespace(capabilities={"reset_scene", "pick_object"}),
        )
        servicer = build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2)(
            runtime, "tok", subject="board-runtime", display_state={})
        return servicer, backend, authority

    def test_reset_then_pick_immediately_is_accepted(self):
        """复位成功后立刻抓取必须被接受，且两次调用后资源表为空（本次缺陷的直接回归）。"""
        servicer, backend, authority = self.build()
        first = servicer.Invoke(_call("reset_scene", {"scope": "all", "target_id": None}, "k-reset"),
                                _Context())
        self.assertTrue(first.accepted, first.error.message_zh)
        self.assertEqual(authority._leases, {}, "复位成功后不得继续占用资源")

        second = servicer.Invoke(_call("pick_object",
                                       {"target_id": "cube", "grasp_pose": {}, "duration_ms": 100},
                                       "k-pick", fencing=1),
                                 _Context())
        self.assertTrue(second.accepted, "%s: %s" % (second.error.code, second.error.message_zh))
        self.assertEqual(backend.calls, ["reset_scene", "pick_object"])
        self.assertEqual(authority._leases, {}, "抓取结束后也不得残留租约")
        self.assertEqual(servicer._registry._leases, {}, "登记表不得累积执行域租约")

    def test_fencing_token_increases_across_calls(self):
        """释放不等于放弃护栏：下一次动作重新 acquire，fencing token 单调上升。"""
        servicer, _backend, _authority = self.build()
        first = servicer.Invoke(_call("reset_scene", {"scope": "all", "target_id": None}, "k1"),
                                _Context())
        second = servicer.Invoke(_call("reset_scene", {"scope": "all", "target_id": None}, "k2"),
                                 _Context())
        self.assertTrue(first.accepted and second.accepted)
        self.assertGreater(second.fencing_token, first.fencing_token)

    def test_rejection_after_acquire_leaves_no_lease(self):
        """取到租约之后才被拒（能力未实现 / 参数形状未登记）也必须放开。"""
        servicer, _backend, authority = self.build()

        def _boom(*_args, **_kwargs):
            raise CapabilityRejected("IRAF-CAPABILITY-UNKNOWN", "参数形状未登记（模拟）")

        servicer._capability_call = _boom
        resp = servicer.Invoke(_call("reset_scene", {"scope": "all", "target_id": None}, "k-rej"),
                               _Context())
        self.assertFalse(resp.accepted)
        self.assertEqual(resp.error.code, "IRAF-CAPABILITY-UNKNOWN")
        self.assertEqual(authority._leases, {}, "拒绝路径不得把资源占到 TTL 到期")

    def test_execution_failure_leaves_no_lease(self):
        """Provider 抛异常 ⇒ FAILED，且不得残留租约（下次动作仍可执行）。"""
        servicer, _backend, authority = self.build(fail_on="pick_object")
        resp = servicer.Invoke(_call("pick_object",
                                     {"target_id": "cube", "grasp_pose": {}, "duration_ms": 100},
                                     "k-fail"),
                               _Context())
        self.assertFalse(resp.accepted)
        self.assertEqual(resp.error.code, "IRAF-PROVIDER-ERROR")
        self.assertEqual(authority._leases, {})


@unittest.skipUnless(_STUBS, "需要 proto 桩")
class ReleaseWithoutLeaseTest(unittest.TestCase):
    """拒绝路径（deadline / BUSY）没取到租约：release 必须**不写终态**。"""

    def test_release_without_lease_is_noop(self):
        from iraf_adapters.remote.provider_service import ExecutionRegistry

        reg = ExecutionRegistry(ControlAuthorityManager(), RESOURCE)
        self.assertFalse(reg.release("exec-none", "SUCCEEDED"))
        self.assertEqual(reg._leases, {})
        self.assertNotIn("", reg._terminal, "空 execution_id 不得进终态表")
        self.assertNotIn("exec-none", reg._terminal)


if __name__ == "__main__":
    unittest.main()
