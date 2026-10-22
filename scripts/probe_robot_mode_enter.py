#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验收：iraf_robot_mode(enter) 只做盘点、返回 next_steps，且**不产生 execution_id**。

为什么单独有这个探针（2026-10-22 dashboard /chat 实机）：
  平台把"进入机器人开发模式"自行补成"只读自检 + 冒烟提交"两段。冒烟那次会真的驱动
  MuJoCo 抓一次，把目标搬走 ⇒ 本次会话再没有干净场景可演示（必现 distance≈0.374435m）。
  我们只能在**自己这边**收口：enter 的返回必须给出固定顺序（先盘点、等真实需求），
  并且 enter 本身不得产生任何执行。

用法（需 mcp 包，本仓库用 build/mcp-venv/bin/python）：
  PYTHONPATH=src build/mcp-venv/bin/python scripts/probe_robot_mode_enter.py
  # 板上：PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src python3 ...
"""
import json
import os
import sys


def main() -> int:
    try:
        from iraf_adapters.mcp import server
    except ModuleNotFoundError as exc:
        print("✗ 缺少依赖：%s（用 build/mcp-venv/bin/python 或板上 python3）" % exc)
        return 2

    out = json.loads(server.iraf_robot_mode("enter"))
    steps = out.get("next_steps") or []
    print("mode =", out.get("mode"), "| active =", out.get("active"),
          "| remaining_s =", out.get("remaining_s"))
    print("skills =", len(out.get("skills") or []), "| intents =", len(out.get("intents") or {}))
    print("simulation =", out.get("simulation"), "| profile =", out.get("profile"))
    print("examples =", sorted((out.get("examples") or {}).keys()),
          "| robot_task.purpose =", (out.get("examples") or {}).get("robot_task", {}).get("purpose"))
    print("next_steps =", len(steps))
    for s in steps:
        print("  -", s)

    bad = []
    if len(steps) < 4:
        bad.append("next_steps 少于 4 条（平台会自己设计下一步）")
    if not any("盘点" in s for s in steps):
        bad.append("缺\"先盘点\"这一步")
    if not any("不要为了验证链路而提交任务" in s for s in steps):
        bad.append("缺\"禁止冒烟提交\"这一条")
    # enter 是只读的：返回里出现 execution_id 说明它真的执行了
    for k in ("execution_id", "resolved_skill", "status_executed"):
        if out.get(k):
            bad.append("enter 返回里出现了 %s（enter 必须是只读盘点）" % k)

    print()
    if bad:
        print("✗ 不合格：")
        for b in bad:
            print("  -", b)
        return 1
    print("✓ enter 只读盘点合规：无 execution_id，next_steps %d 条" % len(steps))
    return 0


if __name__ == "__main__":
    sys.exit(main())
