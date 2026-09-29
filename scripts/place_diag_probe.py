#!/usr/bin/env python3
"""放置段诊断探针（**不改仓库**，2026-09-29 §11.23(48) 稳定性排查）。

为什么要它：稳定性实测 6 次里 2 次失败，模式都是 s04 报「Backend 未确认载荷已放下」——
而 Provider 一旦拒绝，证据就**进不了 scene 报告** ⇒ 失败瞬间的数字（落位间隙、末速、载荷位姿、接触对）
全部看不到。本探针包住后端 `place_object`，把关键证据**无论成败**都打印出来。

用法：python3 build/place_diag_probe.py   （跑一次 nominal --world joint）
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import iraf_adapters.mujoco.mujoco_backend as mb  # noqa: E402

_original = mb.MujocoBackend.place_object


def _wrapped(self, *args, **kwargs):
    report = _original(self, *args, **kwargs)
    try:
        evidence = (report or {}).get("evidence") or {}
        print("\n================ PLACE 摘要（探针）================", flush=True)
        print("released=%s payload_in_tray=%s place_target=%s" % (
            report.get("released"), evidence.get("payload_in_tray"),
            report.get("place_target_id")), flush=True)
        print("落稳: settle_ms=%s gap_m=%s speed_mps=%s on_target=%s" % (
            evidence.get("place_settle_ms"), evidence.get("place_settled_gap_m"),
            evidence.get("place_settled_speed_mps"), evidence.get("place_settled_on_target")), flush=True)
        print("对齐: %s" % json.dumps(evidence.get("place_alignment"), ensure_ascii=False), flush=True)
        print("纠偏: %s" % json.dumps(
            {k: v for k, v in (evidence.get("place_pose_correction") or {}).items()
             if k in ("mode", "applied", "lateral_m", "vertical_m")}, ensure_ascii=False), flush=True)
        print("抬离余量 retreat_delta_m=%s carry_activated=%s" % (
            evidence.get("retreat_delta_m"), evidence.get("carry_constraint_activated")), flush=True)
        print("=================================================\n", flush=True)
    except Exception as error:  # noqa: BLE001
        print("证据序列化失败:", error, flush=True)
    return report


mb.MujocoBackend.place_object = _wrapped

import scenario  # noqa: E402

sys.argv = ["scenario.py", "run", "--scene", "scenes/handoff_lab", "--scenario", "nominal",
            "--world", "joint"]
raise SystemExit(scenario.main())
