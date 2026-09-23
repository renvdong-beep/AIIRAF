#!/usr/bin/env python3
"""停靠自指门禁的验收入口：对**真实场景模型**执行 `dock_for_handoff`，必须在**执行前**显式失败。

为什么需要这个入口（2026-09-24 实测）：本场景把托盘挂在四足背上
（`scene.yaml` 的 `props.tray_01.pose.mount.entity: unitree_go2`），而 `dock_for_handoff` 曾把这个
**背上的帧**当停靠目标 ⇒ 目标帧与机身同一刚体，误差恒为自指量、第 0 拍即"在位"，
白跑 61.5 s 仿真后给出「36 µm / 偏航 0.0°」这种**看起来完美但什么都没做**的结果。
本入口把这条错误钉成**可复跑的判据**（详见 docs/debug/2026-09-24-dock-target-self-frame.md）。

判据（全部实测，缺一条即失败）：
  ① 抛 `DockDeclarationError`（不是返回一个好看的误差数字）；
  ② 报错含目标帧名、所属 body、"刚性挂在机器人 … 上"的判定与可操作方向（世界固定站位）；
  ③ **零物理步进**：失败发生在任何 MPC/控制回路之前（`data.time` 与 `ncon` 前后不变）。

用法（仓库根目录）：
    PYTHONPATH=src MUJOCO_GL=glfw /usr/bin/python3 scripts/verify_dock_target_frame.py
退出码：0 = 全部通过；1 = 有判据失败。
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import yaml  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree.dock import DockDeclarationError  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

CONFIG = ROOT / "config/go2_loopback.yaml"
REPORT = ROOT / "build/acceptance/go2-dock-guard/report.json"


def main():
    cfg = yaml.safe_load(open(CONFIG))
    declaration = cfg.get("dock_for_handoff") or {}
    target_frame = str(declaration.get("target_frame"))
    if not declaration:
        print("声明缺少 dock_for_handoff 段：本入口无法判定（不猜默认值）", file=sys.stderr)
        return 1

    profile = load_robot_profile(ROOT / cfg["robot"]["profile"])
    authority = ControlAuthorityManager()
    backend = load_backend(KNOWN_BACKENDS[cfg["robot"]["backend"]], str(CONFIG), profile, authority)

    before = (float(backend.data.time), int(backend.data.ncon))
    lease = authority.acquire(profile.name + "-mujoco", "dock-target-frame-verify", ttl_seconds=60.0)
    started = time.perf_counter()
    error = None
    try:
        backend.dock_for_handoff(lease=lease, position_tolerance_m=0.03,
                                 yaw_tolerance_rad=math.radians(2.0), max_final_speed_mps=0.05)
    except DockDeclarationError as exc:
        error = exc
    finally:
        authority.release(lease)
    wall_s = time.perf_counter() - started
    after = (float(backend.data.time), int(backend.data.ncon))
    message = str(error) if error is not None else ""

    checks = [
        ("显式失败（DockDeclarationError）", error is not None),
        ("报错含目标帧名 %s" % target_frame, target_frame in message),
        ("报错含所属 body 名", "base_link" in message),
        ("报错含「刚性挂在机器人」判定", "刚性挂在机器人" in message),
        ("报错含可操作方向「世界固定」", "世界固定" in message),
        ("零物理步进（data.time 前后不变）", before[0] == after[0]),
        ("零接触计算（ncon 前后不变）", before[1] == after[1]),
    ]
    report = {
        "capability": "dock_for_handoff",
        "simulation": True,
        "criterion": "停靠目标帧必须世界固定：挂在机器人身上的帧必须在执行前显式失败",
        "declaration_config": str(CONFIG.relative_to(ROOT)),
        "model": str(backend.model_path) if hasattr(backend, "model_path") else None,
        "declared_target_frame": target_frame,
        "raised": type(error).__name__ if error is not None else None,
        "message": message,
        "wall_seconds": wall_s,
        "data_time_before_s": before[0],
        "data_time_after_s": after[0],
        "ncon_before": before[1],
        "ncon_after": after[1],
        "checks": [{"name": name, "passed": bool(passed)} for name, passed in checks],
        "passed": all(passed for _name, passed in checks),
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    for name, passed in checks:
        print("%s %s" % ("通过" if passed else "失败", name))
    print("\n墙钟 %.3f s（对照：自指目标曾白跑 61.5 s 仿真 / 6150 个控制拍）" % wall_s)
    if message:
        print("报错原文：%s" % message)
    print("\n报告：%s" % REPORT.relative_to(ROOT))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
