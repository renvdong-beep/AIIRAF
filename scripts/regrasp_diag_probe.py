#!/usr/bin/env python3
"""regrasp 诊断探针（**不改仓库**）：把后端 `pick_object` 的完整证据打出来（失败时也要看到）。

为什么要它（2026-09-29 §11.23(48)）：regrasp 未通过腰部握力判据时，**Provider 会抛错**
（"Backend 未确认目标已抓取"）⇒ 证据根本不进 scene 报告 ⇒ 调参完全没有输入。
本探针 monkeypatch 后端方法，先打印 `evidence`（含 regrasp 的力与夹持几何），再原样返回。
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import iraf_adapters.mujoco.mujoco_backend as mb  # noqa: E402

_original = mb.MujocoBackend.pick_object


def _wrapped(self, *args, **kwargs):
    report = _original(self, *args, **kwargs)
    try:
        ev = (report or {}).get("evidence") or {}
        print("\n================ PICK/REGRASP 摘要（探针）================", flush=True)
        print("grasped=%s confirmation=%s lifted=%s lift_delta_m=%s" % (
            report.get("grasped"), report.get("confirmation"), ev.get("lifted"),
            ev.get("lift_delta_m")), flush=True)
        print("双侧接触=%s 左力=%s 右力=%s 不平衡比=%s force_ok=%s" % (
            ev.get("bilateral_contact"), ev.get("left_normal_force_n"),
            ev.get("right_normal_force_n"), ev.get("force_imbalance_ratio"),
            ev.get("force_ok")), flush=True)
        print("align center_distance_m=%s pad_offset_m=%s" % (
            (ev.get("grasp_alignment") or {}).get("center_distance_m"),
            (ev.get("grasp_alignment") or {}).get("pad_offset_m")), flush=True)
        for label, value in (("regrasp", ev.get("regrasp")),):
            print("%s = %s" % (label, json.dumps(value, ensure_ascii=False, indent=1)), flush=True)
        print("=================================================\n", flush=True)
    except Exception as error:  # noqa: BLE001
        print("证据序列化失败:", error, flush=True)
    return report


mb.MujocoBackend.pick_object = _wrapped

import scenario  # noqa: E402

sys.argv = ["scenario.py", "run", "--scene", "scenes/handoff_lab", "--scenario", "nominal",
            "--world", "joint"]
raise SystemExit(scenario.main())
