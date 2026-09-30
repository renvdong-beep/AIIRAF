#!/usr/bin/env python3
"""候选**放置落点**的可解性核实（I4-a 前置，2026-09-30 §11.48）。

为什么必须：`nominal --world joint` 已整链 8/8（含 s06 卸载抓取），但"卸载后把方块放到 B 站旁台面"
尚未接上。落点必须先证明**在该臂上可解**（位置可达 **且** 工具指向可用），否则声明写下去也只会在
构建期/运行期以显式失败收场。

口径（与既有全部一致性要求同源）：
  · 用**同一个求解器**（声明的 `robots[].reference_solver.entry` = `build_reference_poses`），
    输入是"**臂基座系**的 xy + 世界 z"（场景把世界目标换算到基座系的同一条公式）；
  · 工具点 = 声明的 `pad_boxes` 中点（与参考姿态证据/运行期门禁同一几何口径）；
  · 输出每相位的 `position_error_m`（残差）与 `orientation_error_deg`（姿态偏差）。

用法：
  PYTHONPATH=src python3 scripts/probe_ur5e_place_point_feasibility.py \\
      --point 0.70,0.60,0.025 [--base 0.45,0.90 --yaw-deg 90] [--output build/diagnostics/...json]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import pathlib
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
SOLVER = REPO / "scripts/build_robot_baseline.py"
BASELINE = REPO / "config/ur5_simulation_baseline.yaml"


def _load_module():
    spec = importlib.util.spec_from_file_location("build_robot_baseline_probe", SOLVER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--point", required=True, help="候选落点（世界系 xyz，逗号分隔）")
    parser.add_argument("--base", default="0.45,0.90", help="臂基座世界 xy")
    parser.add_argument("--yaw-deg", type=float, default=90.0, help="臂基座偏航（绕 z）")
    parser.add_argument("--output", default="build/diagnostics/ur5e-place-point-feasibility.json")
    args = parser.parse_args(argv)

    world = [float(v) for v in args.point.split(",")]
    base = [float(v) for v in args.base.split(",")]
    yaw = math.radians(float(args.yaw_deg))
    # 世界 → **臂基座系**：local = Rᵀ (world − base)（与场景构建期同一条公式；此处基座在 z=0 平面）
    dx, dy, dz = world[0] - base[0], world[1] - base[1], world[2]
    local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
    module = _load_module()
    baseline_doc = yaml.safe_load(BASELINE.read_text(encoding="utf-8"))
    result = {"point_world_m": world, "base_world_xy_m": base, "yaw_deg": args.yaw_deg,
              "local_target_m": [round(local_x, 9), round(local_y, 9), round(dz, 9)],
              "local_xy_radius_m": round(math.hypot(local_x, local_y), 9)}
    try:
        reference = module.build_reference_poses(REPO, baseline_doc,
                                                 target_xy_override_m=[local_x, local_y],
                                                 target_z_override_m=dz)
    except Exception as error:  # noqa: BLE001 —— 求解器门禁失败本身就是判据
        result["solver_error"] = "%s: %s" % (type(error).__name__, error)
        print(json.dumps(result, ensure_ascii=False, indent=2)[:1200])
        return 0
    phases = {}
    for phase in ("home", "approach", "grasp", "lift"):
        packed = reference.get(phase) or {}
        if phase == "home":
            packed = reference.get("home_solved") or {}
        phases[phase] = {"position_error_m": packed.get("position_error_m"),
                         "target_m": packed.get("target_m"),
                         "finger_center_m": packed.get("finger_center_m")}
    result["phases"] = phases
    result["orientation_error_deg"] = reference.get("orientation_error_deg")
    result["pad_offset_m_piper_style"] = reference.get("finger_height_correction_m")
    result["pass"] = all((phases[phase]["position_error_m"] or 9.9) <= 1e-5 for phase in phases)
    out = REPO / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2)[:1600])
    print("证据: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
