# -*- coding: utf-8 -*-
"""LeaseStore 单元测试（阶段 D · D1）"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_core.leases import (  # noqa: E402
    E_RESOURCE_BUSY,
    LeaseConflict,
    LeaseStore,
    capacity_state,
)


def store(tmp_path, enforce=True) -> LeaseStore:
    return LeaseStore(str(tmp_path / "leases.db"), enforce=enforce)


def test_acquire_returns_active_lease(tmp_path):
    s = store(tmp_path)
    lease = s.acquire("piper-pick", "piper", "alice", ttl_s=60)
    assert lease.lease_id
    assert s.get_active("piper-pick", "piper").lease_id == lease.lease_id
    assert lease.remaining_s() > 0


def test_conflict_when_held(tmp_path):
    s = store(tmp_path)
    s.acquire("piper-pick", "piper", "alice", ttl_s=60)
    with pytest.raises(LeaseConflict) as ei:
        s.acquire("piper-pick", "piper", "bob", ttl_s=60)
    assert ei.value.lease.holder == "alice"


def test_different_resource_not_conflicting(tmp_path):
    s = store(tmp_path)
    s.acquire("joint-handoff", "joint-piper", "alice")
    s.acquire("joint-handoff", "joint-ur5", "bob")   # 同 world 不同资源可分别持有
    assert len(s.list_active()) == 2


def test_release_allows_reacquire(tmp_path):
    s = store(tmp_path)
    lease = s.acquire("piper-pick", "piper", "alice")
    assert s.release(lease.lease_id, "done") is True
    assert s.get_active("piper-pick", "piper") is None
    s.acquire("piper-pick", "piper", "bob")


def test_ttl_expiry_frees_resource(tmp_path):
    s = store(tmp_path)
    lease = s.acquire("piper-pick", "piper", "alice", ttl_s=1)
    time.sleep(1.2)
    assert lease.expired()
    # 下次 acquire 会先 expire_stale
    s.acquire("piper-pick", "piper", "bob", ttl_s=60)
    assert s.get_active("piper-pick", "piper").holder == "bob"


def test_renew_extends_ttl(tmp_path):
    s = store(tmp_path)
    lease = s.acquire("piper-pick", "piper", "alice", ttl_s=1)
    time.sleep(1.2)
    renewed = s.renew(lease.lease_id, ttl_s=60)
    assert renewed is not None
    assert renewed.remaining_s() > 50
    assert renewed.expired() is False


def test_persists_across_reopen(tmp_path):
    db = tmp_path / "leases.db"
    s1 = LeaseStore(str(db))
    lease = s1.acquire("piper-pick", "piper", "alice", ttl_s=60)
    s2 = LeaseStore(str(db))          # 模拟 MCP 重启
    again = s2.get_active("piper-pick", "piper")
    assert again is not None and again.lease_id == lease.lease_id


def test_enforce_off_records_without_blocking(tmp_path):
    s = store(tmp_path, enforce=False)
    a = s.acquire("piper-pick", "piper", "alice")
    b = s.acquire("piper-pick", "piper", "bob")
    assert a.lease_id != b.lease_id
    assert len(s.list_active()) == 2


def test_release_by_execution(tmp_path):
    s = store(tmp_path)
    lease = s.acquire("piper-pick", "piper", "alice")
    s.renew(lease.lease_id, execution_id="exec-1")
    assert s.release_by_execution("exec-1", "done") == 1
    assert s.get_active("piper-pick", "piper") is None


def test_conflict_message_has_code(tmp_path):
    s = store(tmp_path)
    s.acquire("piper-pick", "piper", "alice")
    with pytest.raises(LeaseConflict) as ei:
        s.acquire("piper-pick", "piper", "bob")
    assert E_RESOURCE_BUSY in str(ei.value)


class _Host:
    def __init__(self, hid, prio):
        self.id = hid
        self.priority = prio


def test_capacity_state_single_host(tmp_path):
    s = store(tmp_path)
    hosts = [_Host("x86-1", 10)]
    free, held = capacity_state(hosts, s)
    assert [h.id for h in free] == ["x86-1"] and not held
    s.acquire("host:x86-1", "__world__", "alice")
    free, held = capacity_state(hosts, s)
    assert not free and [h.id for h, _ in held] == ["x86-1"]


def test_capacity_state_multi_host_independent(tmp_path):
    """D2：不同 host 的 world 租约互不影响（跨 world 可并行）。"""
    s = store(tmp_path)
    hosts = [_Host("x86-1", 10), _Host("x86-2", 20)]
    s.acquire("host:x86-1", "__world__", "alice")          # 只占第一台
    free, held = capacity_state(hosts, s)
    assert [h.id for h in free] == ["x86-2"], "第二台应可用"
    assert [h.id for h, _ in held] == ["x86-1"]


def test_capacity_state_exhausted(tmp_path):
    s = store(tmp_path)
    hosts = [_Host("x86-1", 10), _Host("x86-2", 20)]
    s.acquire("host:x86-1", "__world__", "alice")
    s.acquire("host:x86-2", "__world__", "bob")
    free, held = capacity_state(hosts, s)
    assert free == [] and len(held) == 2
