# -*- coding: utf-8 -*-
"""异步运行记录（阶段 E1）

背景：`/scenario` 是同步阻塞调用（最长 900s），chat 侧看起来"卡住"，客户端也容易超时。
本模块给"后台跑、按 run_id 查"提供持久化状态：MCP 重启后仍能查到历史，不会假装成功。

状态机：`queued → running → succeeded | failed`；进程重启时把未完成的 run 标为
`failed/error="interrupted by restart"`（诚实失败，不伪造成功）。

只存运行事实：不执行链路、不判断技能。
"""

from __future__ import annotations

import json
import sqlite3
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id      TEXT PRIMARY KEY,
    scenario    TEXT NOT NULL,
    display     TEXT NOT NULL DEFAULT 'window',
    hold_s      INTEGER NOT NULL DEFAULT 60,
    status      TEXT NOT NULL,
    created_at  REAL NOT NULL,
    started_at  REAL,
    finished_at REAL,
    passed      INTEGER,
    rc          INTEGER,
    wall_s      REAL,
    steps_json  TEXT NOT NULL DEFAULT '[]',
    error       TEXT NOT NULL DEFAULT '',
    via         TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_runs_created ON runs(created_at DESC);
"""

TERMINAL = ("succeeded", "failed")


class RunStore:
    def __init__(self, db_path: str, mark_interrupted: bool = True):
        self.path = str(Path(db_path))
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        with self._conn() as c:
            c.executescript(SCHEMA)
        if mark_interrupted:
            self.mark_interrupted()

    @contextmanager
    def _conn(self):
        conn = sqlite3.connect(self.path, timeout=10)
        try:
            conn.row_factory = sqlite3.Row
            yield conn
            conn.commit()
        finally:
            conn.close()

    # -- 写 -----------------------------------------------------------
    def create(self, scenario: str, display: str = "window", hold_s: int = 60) -> str:
        run_id = uuid.uuid4().hex
        with self._conn() as c:
            c.execute("INSERT INTO runs(run_id, scenario, display, hold_s, status, created_at) "
                      "VALUES(?,?,?,?, 'queued', ?)",
                      (run_id, scenario, display, int(hold_s), time.time()))
        return run_id

    def start(self, run_id: str) -> bool:
        with self._conn() as c:
            cur = c.execute("UPDATE runs SET status='running', started_at=? "
                            "WHERE run_id=? AND status='queued'", (time.time(), run_id))
            return cur.rowcount > 0

    def finish(self, run_id: str, *, status: str, passed: Optional[bool] = None,
               rc: Optional[int] = None, wall_s: Optional[float] = None,
               steps: Optional[list] = None, error: str = "", via: str = "") -> bool:
        if status not in TERMINAL:
            raise ValueError("status 必须是 %s 之一：%r" % (TERMINAL, status))
        with self._conn() as c:
            cur = c.execute(
                "UPDATE runs SET status=?, finished_at=?, passed=?, rc=?, wall_s=?, "
                "steps_json=?, error=?, via=? WHERE run_id=?",
                (status, time.time(), None if passed is None else (1 if passed else 0),
                 rc, wall_s, json.dumps(steps or [], ensure_ascii=False), error, via, run_id))
            return cur.rowcount > 0

    def mark_interrupted(self) -> int:
        """进程重启：把 queued/running 的旧记录标为 failed（诚实失败）。"""
        with self._conn() as c:
            cur = c.execute(
                "UPDATE runs SET status='failed', finished_at=?, "
                "error='interrupted by restart', rc=-1 "
                "WHERE status IN ('queued','running')", (time.time(),))
            return cur.rowcount

    # -- 读 -----------------------------------------------------------
    @staticmethod
    def _row(row) -> dict:
        return {
            "run_id": row["run_id"],
            "scenario": row["scenario"],
            "display": row["display"],
            "hold_s": row["hold_s"],
            "status": row["status"],
            "created_at": row["created_at"],
            "started_at": row["started_at"],
            "finished_at": row["finished_at"],
            "passed": None if row["passed"] is None else bool(row["passed"]),
            "rc": row["rc"],
            "wall_s": row["wall_s"],
            "steps": json.loads(row["steps_json"] or "[]"),
            "error": row["error"],
            "via": row["via"],
        }

    def get(self, run_id: str) -> Optional[dict]:
        with self._conn() as c:
            row = c.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
        return self._row(row) if row else None

    def list_recent(self, limit: int = 20) -> List[dict]:
        with self._conn() as c:
            rows = c.execute("SELECT * FROM runs ORDER BY created_at DESC LIMIT ?",
                             (max(1, int(limit)),)).fetchall()
        return [self._row(r) for r in rows]

    def active(self) -> List[dict]:
        with self._conn() as c:
            rows = c.execute("SELECT * FROM runs WHERE status IN ('queued','running') "
                             "ORDER BY created_at").fetchall()
        return [self._row(r) for r in rows]
