#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读：验证“技能选择收口”在板上真的生效（iraf_robot_mode 返回 skill_triggers）。

判据（2026-10-23）：
  · 返回里必须有 skill_triggers，且 visual_pick 的 natural_language=false
  · place_object 的 requires/note 指向"接收体/tray_01"
  · pick_object 允许自然语言（否则连"抓取 box_01"都会被挡）

用法（板上，经 ssh stdin）：
  sshpass -e ssh root@<板> "PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
      python3 - --env /etc/iraf/edge-intel.env" < scripts/probe_skill_triggers.py
"""
import json
import os
import sys

env_file = None
if "--env" in sys.argv:
    env_file = sys.argv[sys.argv.index("--env") + 1]

if env_file and os.path.isfile(env_file):
    for line in open(env_file, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        os.environ.setdefault(key.strip(), value.strip().strip("'\""))

os.environ.setdefault("IRAF_VERSION_DIR", "/opt/iraf-sdk/current")

try:
    from iraf_adapters.mcp import server
except ModuleNotFoundError as exc:
    raise SystemExit("缺依赖：%s（用板上 python3 + PYTHONPATH 指向 SDK）" % exc)

data = json.loads(server.iraf_robot_mode("status"))
triggers = data.get("skill_triggers") or {}
print("mode =", data.get("mode"), "| skills =", len(data.get("skills") or []),
      "| skill_triggers =", len(triggers))
if not triggers:
    raise SystemExit("✗ 返回里没有 skill_triggers（版本没升上去？）")

for name in sorted(triggers):
    t = triggers[name] or {}
    flag = "自然语言" if t.get("natural_language") else "需显式参数"
    extra = ""
    if t.get("requires"):
        extra = " requires=%s" % t["requires"]
    if t.get("note"):
        extra += " note=%s" % t["note"]
    print("  %-28s %s%s" % (name, flag, extra))

bad = []
if (triggers.get("pick_object") or {}).get("natural_language") is not True:
    bad.append("pick_object 应允许自然语言")
if (triggers.get("visual_pick") or {}).get("natural_language") is not False:
    bad.append("visual_pick 应禁止自然语言")
if "tray_01" not in str((triggers.get("place_object") or {}).get("note", "")):
    bad.append("place_object 的 note 没提 tray_01")

print()
if bad:
    print("✗ 不合格：" + "；".join(bad))
    raise SystemExit(1)
print("✓ 收口生效：pick_object=自然语言，visual_pick=需 vision_file，place_object=需接收体")
