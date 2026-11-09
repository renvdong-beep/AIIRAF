# -*- coding: utf-8 -*-
"""RunStore 单元测试（阶段 E1）"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from iraf_core.runs import RunStore  # noqa: E402


def store(tmp_path, mark_interrupted=True) -> RunStore:
    return RunStore(str(tmp_path / "runs.db"), mark_interrupted=mark_interrupted)


def test_create_is_queued(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal", "window", 60)
    run = s.get(rid)
    assert run["status"] == "queued"
    assert run["scenario"] == "nominal"
    assert run["started_at"] is None


def test_lifecycle_success(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal", "none", 30)
    assert s.start(rid) is True
    assert s.get(rid)["status"] == "running"
    assert s.finish(rid, status="succeeded", passed=True, rc=0, wall_s=12.5,
                    steps=[{"id": "s01", "status": "SUCCEEDED"}], via="supervisor") is True
    run = s.get(rid)
    assert run["status"] == "succeeded"
    assert run["passed"] is True
    assert run["wall_s"] == 12.5
    assert run["steps"][0]["id"] == "s01"
    assert run["finished_at"] is not None


def test_lifecycle_failure(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal")
    s.start(rid)
    s.finish(rid, status="failed", passed=False, rc=1, error="provider unavailable")
    run = s.get(rid)
    assert run["status"] == "failed" and run["passed"] is False
    assert "provider" in run["error"]


def test_start_only_from_queued(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal")
    s.start(rid)
    assert s.start(rid) is False       # 不能从 running 再 start


def test_finish_rejects_unknown_status(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal")
    with pytest.raises(ValueError):
        s.finish(rid, status="weird")


def test_restart_marks_interrupted(tmp_path):
    db = tmp_path / "runs.db"
    s1 = RunStore(str(db), mark_interrupted=False)
    rid = s1.create("nominal")
    s1.start(rid)
    s2 = RunStore(str(db))              # 模拟重启
    run = s2.get(rid)
    assert run["status"] == "failed"
    assert run["error"] == "interrupted by restart"
    assert run["rc"] == -1


def test_list_recent_and_active(tmp_path):
    s = store(tmp_path)
    a = s.create("a")
    b = s.create("b")
    s.start(b)
    assert len(s.list_recent()) == 2
    # active = queued + running（a 还没开始，也算在飞）
    assert {r["run_id"] for r in s.active()} == {a, b}
    s.finish(b, status="succeeded", passed=True)
    s.finish(a, status="failed", rc=-1, error="cancelled")
    assert s.active() == []
    assert s.get(a)["status"] == "failed"


def test_steps_roundtrip_unicode(tmp_path):
    s = store(tmp_path)
    rid = s.create("nominal")
    steps = [{"id": "s02_dock", "status": "SUCCEEDED", "note": "停靠到位"}]
    s.finish(rid, status="succeeded", passed=True, steps=steps)
    assert s.get(rid)["steps"][0]["note"] == "停靠到位"
