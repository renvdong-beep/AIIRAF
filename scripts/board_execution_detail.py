#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读：把板上事件库最近几次执行的结果解析成人话（status / skill / reason / 耗时）。

用法（开发机，经 ssh stdin）：
  sshpass -e ssh root@<板> "python3 - --limit 5" < scripts/board_execution_detail.py
"""
import json
import sqlite3
import sys
from pathlib import Path

DB = Path("/var/lib/iraf/iraf-runtime.db")
LIMIT = 5
if "--limit" in sys.argv:
    try:
        LIMIT = int(sys.argv[sys.argv.index("--limit") + 1])
    except (IndexError, ValueError):
        pass

con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
cur = con.cursor()
rows = list(cur.execute(
    "select execution_id, result_json, updated_at_ms from executions order by rowid desc limit %d"
    % LIMIT))

for execution_id, raw, updated in rows:
    try:
        result = json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        result = {"_unparsable": str(raw)[:120]}
    status = result.get("status") or "-"
    skill = result.get("resolved_skill") or result.get("skill") or "-"
    code = result.get("error_code") or "-"
    reason = (result.get("reason") or "-")
    mode = result.get("robot_mode") or "(未回传)"
    steps = len(result.get("next_steps") or [])
    print("%s\n  status=%s skill=%s robot_mode=%s next_steps=%d error=%s"
          % (execution_id, status, skill, mode, steps, code))
    if reason and reason != "-":
        print("  reason=%s" % str(reason)[:160])
    # 状态迁移序列（便于判断是并发/场景/鉴权哪一类）
    seq = list(cur.execute(
        "select status, reason from execution_events where execution_id=? order by sequence",
        (execution_id,)))
    print("  events=" + " → ".join(
        "%s%s" % (s, ("(%s)" % str(r)[:40]) if r else "") for s, r in seq))
con.close()
