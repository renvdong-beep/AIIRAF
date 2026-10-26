# -*- coding: utf-8 -*-
"""探针：把 piper 抓取场景的方块挪到**机械臂左侧**，并给报告补上抓取段纠偏声明。

用途：验证"piper 能否按实测目标重解 IK 抓左侧方块"（机制验证）。产物落在 build/models/probe/，
**不是交付物**（交付物必须由构建器生成；本探针只回答"这条路通不通"）。
"""
from __future__ import annotations

import json
import re
import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC_XML = REPO / "build/models/piper-pick-scene.xml"
SRC_JSON = REPO / "build/models/piper-pick-scene.json"
OUT = REPO / "build/models/probe"
LEFT_Y = 0.15           # +y = 面向 +x 时的左侧
LEFT_X = 0.19

OUT.mkdir(parents=True, exist_ok=True)
xml = SRC_XML.read_text(encoding="utf-8")
report = json.loads(SRC_JSON.read_text(encoding="utf-8"))

# 1) 方块 body 位置（XML）——只改 box_01 那一行，其余逐字不动
pattern = re.compile(r'(<body name="box_01"[^>]*?pos=")([^"]+)(")', re.S)
match = pattern.search(xml)
if not match:
    raise SystemExit("XML 里找不到 box_01 的 pos 属性")
old_pos = match.group(2)
new_pos = "%.9f %.9f 0.025" % (LEFT_X, LEFT_Y)
xml = pattern.sub(lambda m: m.group(1) + new_pos + m.group(3), xml, count=1)

# 2) 报告：目标位置 + 抓取段纠偏声明（pad_boxes / grasp_pose_correction）
report["target_position"] = {"x": LEFT_X, "y": LEFT_Y, "z": 0.025}
for target in report.get("targets") or []:
    if target.get("id") == report.get("target_id"):
        target["position_m"] = [LEFT_X, LEFT_Y, 0.025]
gripper = report["gripper"]
gripper["pad_boxes"] = ["piper_left_finger", "piper_right_finger"]
gripper["grasp_pose_correction"] = {
    "mode": "resolved",
    "residual_tolerance_m": 0.002,
    "max_correction_m": 0.07,
    "ik_iterations": 400,
    "ik_step": 0.5,
    "ik_tolerance_m": 0.0005,
    "align_max_attempts": 3,
    "descend_splits": 5,
    "max_axis_deg": 8.0,
}

(OUT / "piper-left-pick-scene.xml").write_text(xml, encoding="utf-8")
(OUT / "piper-left-pick-scene.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
if (REPO / "build/models/meshes").is_dir():
    (OUT / "meshes").symlink_to(REPO / "build/models/meshes", target_is_directory=True)

print("[probe] 方块 %s → %s（左侧 y=%.3f）" % (old_pos, new_pos, LEFT_Y))
print("[probe] 产物：%s/piper-left-pick-scene.{xml,json}" % OUT)
print("[probe] 报告已补：pad_boxes=%s，grasp_pose_correction.mode=%s"
      % (gripper["pad_boxes"], gripper["grasp_pose_correction"]["mode"]))
