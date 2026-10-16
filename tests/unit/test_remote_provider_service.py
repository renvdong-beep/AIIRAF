# -*- coding: utf-8 -*-
"""远端 Capability Provider 的**拒绝路径**与租约语义单测（铁律 2.8 / 2.12）。

为什么必须有用例：跨机 Provider 是"物理动作的最后一跳"。它若在缺 deadline、
旧 fencing token、终态执行等情况下"尽力而为"，就会出现
「板上以为被拒、仿真侧却动了」这种静默不一致 —— 比直接失败危险得多。

本文件**不 import grpc / proto 桩**（只用 stdlib + iraf_core），
因此桩尚未生成、或环境没装 grpcio 时也能跑。
"""
from __future__ import annotations

import time

import pytest

from iraf_core.authority import ControlAuthorityManager
from iraf_adapters.remote.provider_service import (
    CODE_DEADLINE_EXCEEDED,
    CODE_DEADLINE_MISSING,
    CODE_FENCING_REGRESSION,
    CODE_TERMINAL_REUSE,
    CapabilityRejected,
    ExecutionRegistry,
)

RESOURCE = "robot-arm"
MOTION = "move_joint"          # 控制类能力（在 factory.MOTION_CAPABILITIES 内）
MONITOR = "accept_payload"     # 监控类能力


class _Stamp:
    """google.protobuf.Timestamp 的最小替身。"""

    def __init__(self, epoch_s):
        self.seconds = int(epoch_s)
        self.nanos = int((epoch_s - int(epoch_s)) * 1e9)


class _Envelope:
    """LeaseEnvelope 的最小替身（只提供 ExecutionRegistry 真正读的字段）。"""

    def __init__(self, *, execution_id="", resource_id=RESOURCE, fencing_token=0,
                 idempotency_key="", deadline_s=None):
        self.execution_id = execution_id
        self.resource_id = resource_id
        self.fencing_token = fencing_token
        self.idempotency_key = idempotency_key
        self.deadline = _Stamp(deadline_s) if deadline_s is not None else _Stamp(0)
        self._has_deadline = deadline_s is not None

    def HasField(self, name):
        if name == "deadline":
            return self._has_deadline
        raise ValueError(name)


def _registry(ttl=120.0):
    return ExecutionRegistry(ControlAuthorityManager(), RESOURCE, ttl_seconds=ttl)


# ---------- 拒绝路径 ----------

def test_reject_missing_deadline():
    """无 deadline 的调用必须被拒（铁律 2.3：禁止无超时的等待）。"""
    reg = _registry()
    with pytest.raises(CapabilityRejected) as exc:
        reg.resolve(_Envelope(deadline_s=None), MOTION)
    assert exc.value.code == CODE_DEADLINE_MISSING
    assert exc.value.retryable is False


def test_reject_expired_deadline():
    """deadline 已过必须被拒，且**不得**留下租约（拒绝不产生副作用）。"""
    reg = _registry()
    with pytest.raises(CapabilityRejected) as exc:
        reg.resolve(_Envelope(deadline_s=time.time() - 1.0), MOTION)
    assert exc.value.code == CODE_DEADLINE_EXCEEDED
    assert reg._leases == {}, "拒绝路径不应登记租约"


def test_reject_fencing_token_regression():
    """旧 fencing token 必须被拒（旧控制源的请求不得驱动执行器）。"""
    reg = _registry()
    reg.resolve(_Envelope(execution_id="exec-a", fencing_token=7, deadline_s=time.time() + 60), MOTION)
    with pytest.raises(CapabilityRejected) as exc:
        reg.resolve(_Envelope(execution_id="exec-b", fencing_token=3, deadline_s=time.time() + 60), MOTION)
    assert exc.value.code == CODE_FENCING_REGRESSION
    assert reg._last_token[RESOURCE] == 7, "回退的 token 不得降低已见水位"


def test_reject_terminal_reuse_for_motion_capability():
    """终态执行再发**控制类**请求必须被拒（铁律 2.12）。"""
    reg = _registry()
    envelope = _Envelope(execution_id="exec-done", deadline_s=time.time() + 60)
    reg.resolve(envelope, MOTION)
    reg.release("exec-done", "SUCCEEDED")
    with pytest.raises(CapabilityRejected) as exc:
        reg.resolve(_Envelope(execution_id="exec-done", deadline_s=time.time() + 60), MOTION)
    assert exc.value.code == CODE_TERMINAL_REUSE


def test_terminal_execution_still_allows_monitor_capability():
    """监控类能力（不产生物理动作）在终态后仍可读 —— 语义边界要精确。"""
    reg = _registry()
    reg.resolve(_Envelope(execution_id="exec-done2", deadline_s=time.time() + 60), MOTION)
    reg.release("exec-done2", "SUCCEEDED")
    execution_id, lease = reg.resolve(
        _Envelope(execution_id="exec-done2", deadline_s=time.time() + 60), MONITOR)
    assert execution_id == "exec-done2"
    assert lease is not None


# ---------- 成功路径与租约语义 ----------

def test_lease_acquired_once_and_reused_within_execution():
    """同一 execution 的多次调用复用同一租约（fencing token 不因重复调用漂移）。"""
    reg = _registry()
    first = _Envelope(execution_id="exec-x", deadline_s=time.time() + 60)
    _id1, lease1 = reg.resolve(first, MOTION)
    _id2, lease2 = reg.resolve(first, MOTION)
    assert lease1 is lease2
    assert lease1.fencing_token == 1


def test_release_frees_resource_for_next_execution():
    """释放后资源可被下一次执行获取，且 fencing token 单调上升。"""
    reg = _registry()
    _id, lease1 = reg.resolve(_Envelope(execution_id="exec-1", deadline_s=time.time() + 60), MOTION)
    reg.release("exec-1", "SUCCEEDED")
    _id, lease2 = reg.resolve(_Envelope(execution_id="exec-2", deadline_s=time.time() + 60), MOTION)
    assert lease2.fencing_token > lease1.fencing_token


def test_execution_id_generated_by_server_when_absent():
    """execution_id 缺省时由**服务端**生成（铁律 2.12：不是调用方给）。"""
    reg = _registry()
    execution_id, _lease = reg.resolve(
        _Envelope(execution_id="", deadline_s=time.time() + 60), MOTION)
    assert execution_id.startswith("exec-")
    assert execution_id in reg._leases


def test_step_and_monitor_capabilities_need_no_lease():
    """纯查询/步进类能力不占资源租约（不产生资源所有权）。"""
    reg = _registry()
    execution_id, lease = reg.resolve(
        _Envelope(execution_id="exec-s", deadline_s=time.time() + 60), "step")
    assert lease is None
    assert reg._leases.get(execution_id) is None


def test_idempotency_cache_roundtrip():
    """同 idempotency_key 可命中缓存（幂等：不重复执行同一逻辑动作）。"""
    reg = _registry()
    assert reg.cached("key-1") is None
    reg.remember("key-1", {"accepted": True, "capability": MOTION})
    assert reg.cached("key-1") == {"accepted": True, "capability": MOTION}
    assert reg.cached("") is None, "空键不参与幂等（否则不同调用会互相污染）"


def test_cancel_is_honest_about_not_hard_stopping():
    """取消只置标志并把执行标记为 CANCEL_REQUESTED —— 不假装"已经停住了"。"""
    reg = _registry()
    event = reg.cancel_event("exec-c")
    assert not event.is_set()
    event.set()
    assert reg.requested_cancel("exec-c")
