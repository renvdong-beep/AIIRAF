#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读：查板上 IRAF 事件库最近的执行事实（execution_id / status / skill / 时间）。

用法（开发机，经 ssh stdin 执行；板上无需 sqlite3 CLI）：
  sshpass -e ssh root@<板> "python3 -" < scripts/board_query_executions.py
  sshpass -e ssh root@<板> "python3 - --limit 10" < scripts/board_query_executions.py
"""
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

if not DB.is_file():
    raise SystemExit("事件库不存在：%s" % DB)

con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
cur = con.cursor()
tables = [r[0] for r in cur.execute("select name from sqlite_master where type='table'")]
print("表：", tables)

candidates = [t for t in tables if "exec" in t.lower()]
if not candidates:
    raise SystemExit("没找到执行相关表")

for table in candidates:
    cols = [r[1] for r in cur.execute('pragma table_info("%s")' % table)]
    print("\n== %s ==" % table)
    print("列：", cols)
    pick = [c for c in ("execution_id", "status", "skill", "skill_name", "created_at",
                        "updated_at", "error_code", "reason") if c in cols]
    if not pick:
        pick = cols[:6]
    sql = 'select %s from "%s" order by rowid desc limit %d' % (", ".join(pick), table, LIMIT)
    for row in cur.execute(sql):
        print("  " + " | ".join(str(v)[:60] if v is not None else "-" for v in row))
con.close()
