#!/usr/bin/env python3
"""探针：放置航点里夹爪通道的取值来自哪一层（臂报告 or 联合场景装配）。

背景（2026-10-08）：s07「Backend 未确认载荷已放下」根因是四段放置航点
(place_transit/above/descend/retreat)_positions 夹带了**闭合值 163.0**，
导致释放相位发出的张开(0.0)被紧随的撤退段位置指令写回 163 ⇒ 指腹夹住载荷。

疑问：当前构建器 scripts/build_robot_baseline.py:670-696 写的是 **open** 值，
      所以产物里的 163 要么来自**过期的臂报告**，要么来自**联合装配阶段**的覆盖。

本探针只读不写，把三层产物里的夹爪通道取值并排列出：
  1) 臂报告：build/models/ur5-pick-scene.json
  2) 臂报告：build/models/piper-pick-scene.json
  3) 联合场景：build/scenes/handoff_lab/handoff_lab_joint.json

用法：python3 scripts/probe_place_waypoint_gripper.py
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
WAYPOINT_RE = re.compile(r"^place_.*_positions$")
GRIPPER_HINT = ("rq2f85", "gripper", "finger", "claw", "jaw", "joint7", "joint8")


def _walk(node, path, out):
    if isinstance(node, dict):
        for key, value in node.items():
            here = f"{path}.{key}" if path else str(key)
            if isinstance(value, dict) and WAYPOINT_RE.match(str(key)):
                out.append((here, value))
                continue
            _walk(value, here, out)
    elif isinstance(node, list):
        for idx, value in enumerate(node):
            _walk(value, f"{path}[{idx}]", out)


def _report(label, path):
    if not path.exists():
        print(f"[{label}] 缺失：{path}")
        return
    doc = json.loads(path.read_text())
    hits = []
    _walk(doc, "", hits)
    print(f"[{label}] {path}  (mtime={path.stat().st_mtime:.0f})")
    if not hits:
        print("  未找到 place_*_positions 字典")
        return
    for where, positions in hits:
        grip = {k: v for k, v in positions.items() if any(h in str(k) for h in GRIPPER_HINT)}
        others = [k for k in positions if k not in grip]
        print(f"  {where}: 夹爪通道={grip or '{}'}  臂关节数={len(others)}")
    print()


def main():
    _report("ur5e-report", REPO / "build/models/ur5-pick-scene.json")
    _report("piper-report", REPO / "build/models/piper-pick-scene.json")
    _report("joint-scene", REPO / "build/scenes/handoff_lab/handoff_lab_joint.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
