# -*- coding: utf-8 -*-
"""执行租约（阶段 D · D1）

目的：把"同一 world 同一时刻只能有一个控制者"从**口头约定**变成**可持久化、可查询、可过期**的租约。

设计
----
* 粒度：`(scene_id, resource_id)`；同一 world 的多个 resource 可分别持有（joint 场景三资源独立）。
* 存储：板卡事件库（`IRAF_EVENT_STORE`，SQLite）的 `leases` 表 —— MCP 重启后租约仍在。
* TTL：默认取 `scenes.lease_ttl_s`；执行期可 `renew`；过期且无活动执行 → 自动回收。
* 冲突：返回 `IRAF-RESOURCE-BUSY`（附 holder 与剩余 TTL），**不排队、不静默重试**。
* 开关：`IRAF_LEASE_ENFORCE=0` 时只记录不拦（回滚路径）。

只做租约本身：不判断技能、不执行动作、不做健康检查。
"""

from __future__ import annotations

import sqlite3
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

E_RESOURCE_BUSY = "IRAF-RESOURCE-BUSY"
E_LEASE_INVALID = "IRAF-INPUT-INVALID"

SCHEMA = """
CREATE TABLE IF NOT EXISTS leases (
    lease_id     TEXT PRIMARY KEY,
    scene_id     TEXT NOT NULL,
    resource_id  TEXT NOT NULL,
    holder       TEXT NOT NULL,
    execution_id TEXT NOT NULL DEFAULT '',
    acquired_at  REAL NOT NULL,
    renewed_at   REAL NOT NULL,
    ttl_s        INTEGER NOT NULL,
    released_at  REAL,
    release_reason TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_leases_key ON leases(scene_id, resource_id, released_at);
CREATE INDEX IF NOT EXISTS idx_leases_exec ON leases(execution_id);
"""


@dataclass(frozen=True)
class Lease:
    lease_id: str
    scene_id: str
    resource_id: str
    holder: str
    execution_id: str
    acquired_at: float
    renewed_at: float
    ttl_s: int
    released_at: Optional[float]
    release_reason: str

    @property
    def expires_at(self) -> float:
        return self.renewed_at + self.ttl_s

    def remaining_s(self, now: Optional[float] = None) -> float:
        return max(0.0, self.expires_at - (now if now is not None else time.time()))

    def expired(self, now: Optional[float] = None) -> bool:
        return self.remaining_s(now) <= 0

    def as_dict(self) -> dict:
        return {
            "lease_id": self.lease_id,
            "scene_id": self.scene_id,
            "resource_id": self.resource_id,
            "holder": self.holder,
            "execution_id": self.execution_id,
            "acquired_at": self.acquired_at,
            "renewed_at": self.renewed_at,
            "ttl_s": self.ttl_s,
            "expires_at": self.expires_at,
            "remaining_s": round(self.remaining_s(), 1),
            "released_at": self.released_at,
            "release_reason": self.release_reason,
        }


class LeaseConflict(Exception):
    """同 (scene, resource) 已有未过期租约。"""

    def __init__(self, lease: Lease):
        self.lease = lease
        super().__init__("%s: %s/%s 被 %s 持有（剩余 %.1fs）"
                         % (E_RESOURCE_BUSY, lease.scene_id, lease.resource_id,
                            lease.holder, lease.remaining_s()))


class LeaseStore:
    def __init__(self, db_path: str, enforce: bool = True):
        self.path = str(Path(db_path))
        self.enforce = enforce
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        with self._conn() as c:
            c.executescript(SCHEMA)

    @contextmanager
    def _conn(self):
        conn = sqlite3.connect(self.path, timeout=10)
        try:
            conn.row_factory = sqlite3.Row
            yield conn
            conn.commit()
        finally:
            conn.close()

    @staticmethod
    def _row(row) -> Lease:
        return Lease(
            lease_id=row["lease_id"], scene_id=row["scene_id"], resource_id=row["resource_id"],
            holder=row["holder"], execution_id=row["execution_id"],
            acquired_at=row["acquired_at"], renewed_at=row["renewed_at"],
            ttl_s=row["ttl_s"], released_at=row["released_at"],
            release_reason=row["release_reason"])

    # -- 查询 ---------------------------------------------------------
    def get_active(self, scene_id: str, resource_id: str) -> Optional[Lease]:
        with self._conn() as c:
            row = c.execute(
                "SELECT * FROM leases WHERE scene_id=? AND resource_id=? AND released_at IS NULL "
                "ORDER BY renewed_at DESC LIMIT 1", (scene_id, resource_id)).fetchone()
        return self._row(row) if row else None

    def list_active(self) -> List[Lease]:
        with self._conn() as c:
            rows = c.execute("SELECT * FROM leases WHERE released_at IS NULL "
                             "ORDER BY scene_id, resource_id").fetchall()
        return [self._row(r) for r in rows]

    # -- 生命周期 -----------------------------------------------------
    def expire_stale(self, now: Optional[float] = None) -> int:
        """把已过期且未释放的租约标记为 expired（不物理删除，留痕）。"""
        now = now if now is not None else time.time()
        with self._conn() as c:
            cur = c.execute(
                "UPDATE leases SET released_at=?, release_reason='expired' "
                "WHERE released_at IS NULL AND (renewed_at + ttl_s) <= ?", (now, now))
            return cur.rowcount

    def acquire(self, scene_id: str, resource_id: str, holder: str,
                ttl_s: int = 60, execution_id: str = "") -> Lease:
        """获取租约；冲突抛 LeaseConflict。enforce=False 时只记录（返回新租约，不拦）。"""
        ttl_s = max(1, int(ttl_s))
        self.expire_stale()
        now = time.time()
        with self._conn() as c:
            if self.enforce:
                row = c.execute(
                    "SELECT * FROM leases WHERE scene_id=? AND resource_id=? AND released_at IS NULL "
                    "ORDER BY renewed_at DESC LIMIT 1", (scene_id, resource_id)).fetchone()
                if row is not None:
                    raise LeaseConflict(self._row(row))
            lease = Lease(lease_id=uuid.uuid4().hex, scene_id=scene_id, resource_id=resource_id,
                          holder=holder, execution_id=execution_id, acquired_at=now,
                          renewed_at=now, ttl_s=ttl_s, released_at=None, release_reason="")
            c.execute(
                "INSERT INTO leases(lease_id, scene_id, resource_id, holder, execution_id, "
                "acquired_at, renewed_at, ttl_s, released_at, release_reason) "
                "VALUES(?,?,?,?,?,?,?,?,NULL,'')",
                (lease.lease_id, lease.scene_id, lease.resource_id, lease.holder,
                 lease.execution_id, lease.acquired_at, lease.renewed_at, lease.ttl_s))
        return lease

    def renew(self, lease_id: str, ttl_s: Optional[int] = None,
              execution_id: Optional[str] = None) -> Optional[Lease]:
        with self._conn() as c:
            row = c.execute("SELECT * FROM leases WHERE lease_id=?", (lease_id,)).fetchone()
            if row is None or row["released_at"] is not None:
                return None
            ttl = int(ttl_s if ttl_s is not None else row["ttl_s"])
            exec_id = row["execution_id"] if execution_id is None else execution_id
            c.execute("UPDATE leases SET renewed_at=?, ttl_s=?, execution_id=? WHERE lease_id=?",
                      (time.time(), ttl, exec_id, lease_id))
        return self.get_by_id(lease_id)

    def get_by_id(self, lease_id: str) -> Optional[Lease]:
        with self._conn() as c:
            row = c.execute("SELECT * FROM leases WHERE lease_id=?", (lease_id,)).fetchone()
        return self._row(row) if row else None

    def release(self, lease_id: str, reason: str = "done") -> bool:
        with self._conn() as c:
            cur = c.execute(
                "UPDATE leases SET released_at=?, release_reason=? "
                "WHERE lease_id=? AND released_at IS NULL", (time.time(), reason, lease_id))
            return cur.rowcount > 0

    def release_by_execution(self, execution_id: str, reason: str = "done") -> int:
        with self._conn() as c:
            cur = c.execute(
                "UPDATE leases SET released_at=?, release_reason=? "
                "WHERE execution_id=? AND released_at IS NULL", (time.time(), reason, execution_id))
            return cur.rowcount
