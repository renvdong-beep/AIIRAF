#!/usr/bin/env python3
"""Go2 原地踏步的**漂移判据**（可复跑；当前状态：失败）。

背景（2026-09-23 实测，见 docs/debug/2026-09-23-go2-gait-drift.md）：
使用者反馈"狗闪一下就走开了/不见了"。实测真因不是显示，而是**原地踏步在漂移**：
单次 10 s 步态漂移 4.9376 m 并走出台面掉进虚空；声明判据 `gait.verification.max_drift_m = 0.15`
被超 6.5 倍（3.6 s 时已 0.97 m）。历史验收 `build/acceptance/go2-trot-in-place/report.json`
（2026-09-22 09:50）本就 `passed: False`（含 `max_displacement_m`）⇒ 这个演示从未达标。

本脚本做三件事（全部来自声明，不写数字）：
  1. 跑一次步态，按固定间隔采样基座水平位移、高度、倾角、水平速度，打印**逐周期漂移表**；
  2. 与声明判据比较（`max_drift_m`、`min_base_height_m`、`max_tilt_deg`），给出通过/失败；
  3. 报告落盘 `build/acceptance/go2-gait-drift/report.json`，退出码 0=通过 / 1=失败。

用法：
  PYTHONPATH=src DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw \
  /usr/bin/python3 scripts/verify_go2_gait_drift.py --config config/go2_loopback.yaml
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree import gait  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_DECLARATION, EXIT_BACKEND, EXIT_REJECTED, EXIT_FAILED = 0, 1, 2, 3, 4, 5
REPORT_SCHEMA = "iraf.go2-gait-drift/v1"
#: 采样间隔（s，墙钟）：步态在墙钟上比仿真快 ⇒ 用仿真时间对齐，这里只是轮询节流。
POLL_S = 0.15


def _tilt_deg(quat_wxyz):
    """躯干倾角（度，不含偏航）：与 loopback/balance 同一套公式（R 的第三行）。"""
    w, x, y, z = (float(v) for v in np.asarray(quat_wxyz, dtype=float).reshape(4))
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2 原地踏步漂移判据（不是显示）")
    parser.add_argument("--config", type=Path, default=Path("config/go2_loopback.yaml"))
    parser.add_argument("--root", type=Path, default=None)
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--cycles", type=float, default=1.0,
                        help="跑几轮步态（每轮 = 声明 verification.duration_s）")
    args = parser.parse_args(argv)

    root = args.root or unitree_go2.repo_root()
    config_path = args.config if args.config.is_absolute() else root / args.config
    if not config_path.is_file():
        print("用法错误：声明文件不存在: %s" % config_path, file=sys.stderr)
        return EXIT_USAGE

    import yaml

    declaration = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    profile = load_robot_profile(root / declaration["robot"]["profile"])
    params = gait.load_gait_declaration(declaration, profile.joints)
    verification = params["verification"]
    # 判据键名必须与声明一致（我第一次写成 balance 段的 max_drift_m ⇒ KeyError；
    # 原地踏步的位移判据是 gait.verification.max_displacement_m，注释写明"位移目标恒为 0"）。
    max_displacement = float(verification["max_displacement_m"])
    fall_height = float(verification["fall_base_height_m"])
    height_target = float(verification["height_target_m"])
    height_tol = float(verification["height_mean_tolerance_m"])
    height_std_max = float(verification["height_std_max_m"])

    authority = ControlAuthorityManager()
    try:
        backend = load_backend(KNOWN_BACKENDS[str(declaration["robot"]["backend"])], str(config_path),
                               profile, authority)
    except Exception as exc:  # noqa: BLE001
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    duration_s = float(verification["duration_s"])
    lease = authority.acquire(profile.name + "-mujoco", "gait-drift-check",
                              ttl_seconds=max(60.0, args.cycles * duration_s + 60.0))

    holder = {"result": None, "error": None}

    def run():
        try:
            for _ in range(max(1, int(round(args.cycles)))):
                holder["result"] = backend.gait_in_place(lease)
        except Exception as exc:  # noqa: BLE001
            holder["error"] = type(exc).__name__ + ": " + str(exc)

    thread = threading.Thread(target=run, name="gait-drift-run", daemon=True)
    thread.start()

    samples = []
    while thread.is_alive():
        with backend.display_lock():
            qpos = np.asarray(backend.data.qpos[0:7], dtype=float).copy()
            qvel = np.asarray(backend.data.qvel[0:3], dtype=float).copy()
            sim_t = float(backend.data.time)
        samples.append({
            "time_s": sim_t,
            "x_m": float(qpos[0]), "y_m": float(qpos[1]), "z_m": float(qpos[2]),
            "tilt_deg": _tilt_deg(qpos[3:7]),
            "speed_mps": float(np.linalg.norm(qvel)),
        })
        time.sleep(POLL_S)
    thread.join(timeout=30.0)

    with backend.display_lock():
        qpos = np.asarray(backend.data.qpos[0:7], dtype=float).copy()
        sim_t = float(backend.data.time)
    samples.append({"time_s": sim_t, "x_m": float(qpos[0]), "y_m": float(qpos[1]),
                    "z_m": float(qpos[2]), "tilt_deg": _tilt_deg(qpos[3:7]), "speed_mps": 0.0})

    x0, y0 = samples[0]["x_m"], samples[0]["y_m"]
    for row in samples:
        row["drift_m"] = float(((row["x_m"] - x0) ** 2 + (row["y_m"] - y0) ** 2) ** 0.5)
    max_drift_seen = max(row["drift_m"] for row in samples)
    min_height_seen = min(row["z_m"] for row in samples)
    max_tilt_seen = max(row["tilt_deg"] for row in samples)
    # 稳态窗 = 后半段（与既有验收同口径：扣掉 1 s 幅度斜坡后的稳态）
    steady = samples[len(samples) // 2:] or samples
    steady_height = [row["z_m"] for row in steady]
    height_mean = float(np.mean(steady_height))
    height_std = float(np.std(steady_height))

    checks = [
        {"name": "max_displacement_m", "value": max_drift_seen,
         "expectation": "<= %g（全程机身位移峰值；位移目标恒为 0）" % max_displacement,
         "passed": bool(max_drift_seen <= max_displacement)},
        {"name": "fall_base_height_m", "value": min_height_seen,
         "expectation": ">= %g（低于即判翻倒/掉出）" % fall_height,
         "passed": bool(min_height_seen >= fall_height)},
        {"name": "height_mean_m", "value": height_mean,
         "expectation": "|均值 − %g| <= %g" % (height_target, height_tol),
         "passed": bool(abs(height_mean - height_target) <= height_tol)},
        {"name": "height_std_m", "value": height_std,
         "expectation": "<= %g" % height_std_max,
         "passed": bool(height_std <= height_std_max)},
    ]
    failed = [c["name"] for c in checks if not c["passed"]]

    print("逐周期漂移表（仿真时间 / 基座 x,y,z / 漂移 / 倾角 / 速度）：")
    step = max(1, len(samples) // 20)
    for row in samples[::step]:
        print("  t=%6.2f  x=%8.4f y=%8.4f z=%8.4f  漂移=%8.4f m  倾角=%6.2f°  |v|=%6.3f m/s"
              % (row["time_s"], row["x_m"], row["y_m"], row["z_m"], row["drift_m"],
                 row["tilt_deg"], row["speed_mps"]))
    print("\n判据（来自声明 gait.verification）：")
    for check in checks:
        print("  %-20s 实测 %-14.6g 判据 %-12s %s"
              % (check["name"], check["value"], check["expectation"],
                 "通过" if check["passed"] else "**失败**"))
    print("  步态错误: %s" % holder["error"])

    report = {
        "schema_version": REPORT_SCHEMA,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "config": str(config_path),
        "gait": {"kind": params["kind"], "frequency_hz": params["frequency_hz"],
                 "duty_factor": params["duty_factor"], "step_height_m": params["step_height_m"],
                 "duration_s": duration_s, "cycles": args.cycles},
        "criteria": {"max_displacement_m": max_displacement, "fall_base_height_m": fall_height, "height_target_m": height_target, "height_mean_tolerance_m": height_tol, "height_std_max_m": height_std_max, "max_tilt_deg_informational": max_tilt_seen,
"max_tilt_deg": max_tilt_seen},
        "measured": {"max_displacement_m": max_drift_seen, "min_base_height_m": min_height_seen, "max_tilt_deg": max_tilt_seen, "height_mean_m": height_mean, "height_std_m": height_std,
                     "max_tilt_deg": max_tilt_seen},
        "passed": not failed,
        "failed_checks": failed,
        "gait_error": holder["error"],
        "samples": samples,
    }
    report_path = args.report or (root / "build/acceptance/go2-gait-drift/report.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print("\n报告：%s" % report_path)
    if holder["error"]:
        return EXIT_REJECTED
    return EXIT_OK if not failed else EXIT_FAILED


if __name__ == "__main__":
    raise SystemExit(main())
