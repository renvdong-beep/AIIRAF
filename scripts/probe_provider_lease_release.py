# -*- coding: utf-8 -*-
"""探针：x86 Provider 侧租约是否在成功返回后释放（复位→立刻抓 卡 I RAF-RESOURCE-BUSY）。

现象（用户报）：reset_scene 成功返回后立刻调用 pick_object，x86 provider 回
`[IRAF-RESOURCE-BUSY]`。block 侧 runtime 的 release 在 finally 里做，正常。

本探针在**进程内**走完整 servicer.Invoke 路径（不启端口、不驱动 MuJoCo）：
  1) 第一次 Invoke: reset_scene（不同 idempotency_key）
  2) 第二次 Invoke: pick_object（同一 resource_id，不同 idempotency_key）
打印两次的 accepted / error.code / fencing_token，以及 authority 里残留的租约与剩余寿命。
预期（修复前）：第二次 IRAF-RESOURCE-BUSY，authority 里还留着 reset_scene 的租约。
"""
from __future__ import annotations

import sys
import time
import types

from google.protobuf.json_format import ParseDict
from google.protobuf.timestamp_pb2 import Timestamp

from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2
from iraf_core.authority import ControlAuthorityManager
from iraf_adapters.remote.provider_service import build_servicer_class

RESOURCE = "robot-arm"


class FakeBackend:
    def __init__(self):
        self.calls = []

    def runtime_inventory(self):
        return {"safety": {"estop": False}, "mode": "simulation"}

    def reset_scene(self, scope="all", target_id=None, lease=None):
        self.calls.append(("reset_scene", scope))
        return {"ok": True, "reset": True}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease=None):
        self.calls.append(("pick_object", target_id))
        return {"ok": True, "picked": target_id}


class FakeContext:
    def invocation_metadata(self):
        return (("authorization", "Bearer tok"),)

    def abort(self, code, message):
        raise RuntimeError("aborted: %s" % message)


def make_call(capability, inputs, key, fencing=0):
    call = provider_pb2.CapabilityCall(capability=capability)
    ParseDict(inputs, call.inputs)
    stamp = Timestamp()
    stamp.GetCurrentTime()
    stamp.seconds += 60
    call.lease.correlation_id = key
    call.lease.idempotency_key = key
    call.lease.resource_id = RESOURCE
    call.lease.fencing_token = fencing
    call.lease.deadline.CopyFrom(stamp)
    return call


def main():
    backend = FakeBackend()
    authority = ControlAuthorityManager()
    runtime = types.SimpleNamespace(
        backend=backend,
        authority=authority,
        profile=types.SimpleNamespace(capabilities={"reset_scene", "pick_object"}),
    )
    servicer = build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2)(
        runtime, "tok", subject="board-runtime", display_state={})

    r1 = servicer.Invoke(make_call("reset_scene", {"scope": "all", "target_id": None}, "k-reset"),
                         FakeContext())
    print("[1] reset_scene   accepted=%s code=%s token=%d"
          % (r1.accepted, r1.error.code or "-", r1.fencing_token))
    print("    authority 残留租约: %s"
          % sorted((k, v.owner, v.fencing_token) for k, v in authority._leases.items()))
    print("    registry._leases: %d 条" % len(servicer._registry._leases))

    r2 = servicer.Invoke(make_call("pick_object",
                                   {"target_id": "cube", "grasp_pose": {}, "duration_ms": 100},
                                   "k-pick", fencing=1),
                         FakeContext())
    print("[2] pick_object   accepted=%s code=%s token=%d msg=%s"
          % (r2.accepted, r2.error.code or "-", r2.fencing_token, r2.error.message_zh or "-"))

    leftover = sorted((k, v.owner, v.fencing_token,
                       round((v.expires_at_ns - time.monotonic_ns()) / 1e9, 1))
                      for k, v in authority._leases.items())
    print("    authority 残留租约(owner, token, 剩余秒): %s" % leftover)
    print("    backend 实际执行次数: %d %s" % (len(backend.calls), backend.calls))
    print("结论: %s" % ("RESOURCE-BUSY 复现" if r2.error.code == "IRAF-RESOURCE-BUSY" else "未复现"))
    return 0 if r2.accepted else 1


if __name__ == "__main__":
    sys.exit(main())
