#!/usr/bin/env python3
"""`locomote` 的正式验收入口（A6a-④ ⑥）：直行 / 倒退 / 左转 / 右转 四个工况。

判据来源（不写数字）：
  · `config/go2_locomote.yaml`：`velocity_tracking.steady_window_start_s`、`max_mean_rel_error`、
    `velocity_tracking.applies_when`（只适用于 vx≠0 的直行/倒退；转向工况按声明**仅记录**）
  · `profiles/safety/quadruped_lab.yaml`：`max_speed_mps` / `max_yaw_rate_rad_s` / `max_tilt_moving_deg`

诚实边界（写在报告里，避免结论超出证据）：
  · 本脚本直接调适配器入口，**不声明能力**（Profile 的 capabilities 仍 [stand, stop]）；
    "能跑通"不等于"能力已验收"——能力回填还要 aarch64 板复测（铁律 6.8）。
  · 每个工况结束后会打印"段内均值/位移/偏航变化/倾角/失败原因"，失败按原样记录。

用法：
  PYTHONPATH=src DISPLAY=:0 XAUTHORITY=... MUJOCO_GL=glfw /usr/bin/python3 \
      scripts/verify_go2_locomote.py [--only forward] [--duration-ms 3000]
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from iraf_adapters.factory import KNOWN_BACKENDS, load_backend  # noqa: E402
from iraf_adapters.unitree import unitree_go2  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_DECLARATION, EXIT_BACKEND, EXIT_REJECTED, EXIT_FAILED = 0, 1, 2, 3, 4, 5

SCENARIOS = {
    "forward": {"vx_mps": 0.2, "vy_mps": 0.0, "wz_rad_s": 0.0},
    "backward": {"vx_mps": -0.2, "vy_mps": 0.0, "wz_rad_s": 0.0},
    "turn_left": {"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": 0.5},
    "turn_right": {"vx_mps": 0.0, "vy_mps": 0.0, "wz_rad_s": -0.5},
}


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _steady(samples, start_s):
    return [s for s in samples if float(s["time_s"]) >= start_s] or samples


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2 locomote 验收（四个工况）")
    parser.add_argument("--config", type=Path, default=Path("config/go2_locomote.yaml"))
    parser.add_argument("--robot-config", type=Path, default=Path("config/go2_loopback.yaml"))
    parser.add_argument("--only", choices=sorted(SCENARIOS), default=None)
    parser.add_argument("--duration-ms", type=float, default=3000.0)
    parser.add_argument("--report", type=Path,
                        default=Path("build/acceptance/go2-locomote/report.json"))
    args = parser.parse_args(argv)

    acceptance = yaml.safe_load((ROOT / args.config).read_text(encoding="utf-8"))
    robot_config_path = ROOT / args.robot_config
    profile = load_robot_profile(ROOT / acceptance["robot"]["profile"])
    safety = load_safety_policy(ROOT / acceptance["safety_policy"])
    tracking = acceptance["velocity_tracking"]
    steady_start = float(tracking["steady_window_start_s"])
    max_rel = float(tracking["max_mean_rel_error"])
    limits = (safety.as_dict() if hasattr(safety, "as_dict") else {})
    max_speed = float(getattr(safety, "max_speed_mps", 0.0) or 0.0)
    max_yaw = float(getattr(safety, "max_yaw_rate_rad_s", 0.0) or 0.0)
    max_tilt = float(getattr(safety, "max_tilt_moving_deg", 15.0) or 15.0)
    budget = getattr(safety, "budget", None) or {}
    if isinstance(budget, dict):
        max_speed = float(budget.get("max_speed_mps", max_speed) or max_speed)
        max_yaw = float(budget.get("max_yaw_rate_rad_s", max_yaw) or max_yaw)

    authority = ControlAuthorityManager()
    try:
        backend = load_backend(KNOWN_BACKENDS[str(robot_config_path and
                                                  yaml.safe_load(robot_config_path.read_text(encoding="utf-8"))
                                                  ["robot"]["backend"])],
                               str(robot_config_path), profile, authority)
    except Exception as exc:  # noqa: BLE001
        print("后端装配失败：%s" % exc, file=sys.stderr)
        return EXIT_BACKEND

    results = []
    for name in ([args.only] if args.only else sorted(SCENARIOS)):
        velocity = dict(SCENARIOS[name])
        if max_speed > 0.0:
            assert abs(velocity["vx_mps"]) <= max_speed
        if max_yaw > 0.0:
            assert abs(velocity["wz_rad_s"]) <= max_yaw
        lease = authority.acquire(profile.name + "-mujoco", "locomote-%s" % name,
                                  ttl_seconds=max(60.0, args.duration_ms / 1000.0 * 6.0))
        entry = {"scenario": name, "command": velocity}
        try:
            report = backend.locomote(velocity, args.duration_ms, lease)
        except Exception as exc:  # noqa: BLE001 —— 显式失败按原样记录，不当成功
            entry.update({"status": "RAISED", "error": "%s: %s" % (type(exc).__name__, exc),
                          "checks": [], "passed": False})
            results.append(entry)
            print("  %-11s **异常** %s: %s" % (name, type(exc).__name__, exc))
            continue
        samples = report.get("samples") or []
        steady = _steady(samples, steady_start + float(samples[0]["time_s"]) if samples else 0.0)
        speeds = np.array([float(s["base_linear_speed_mps"]) for s in steady]) if steady else np.array([0.0])
        tilts = np.array([float(s["tilt_deg"]) for s in steady]) if steady else np.array([0.0])
        yaw_rates = np.array([float(s["base_yaw_rate_rad_s"]) for s in steady]) if steady else np.array([0.0])
        yaw0 = float(samples[0]["base_yaw_deg"]) if samples else 0.0
        yaw1 = float(samples[-1]["base_yaw_deg"]) if samples else 0.0
        xy0 = np.array(samples[0]["base_position_xy_m"], dtype=float) if samples else np.zeros(2)
        xy1 = np.array(samples[-1]["base_position_xy_m"], dtype=float) if samples else np.zeros(2)
        displacement = float(np.linalg.norm(xy1 - xy0))
        mean_speed = float(np.mean(speeds))
        cmd_speed = abs(float(velocity["vx_mps"]))
        cmd_yaw = abs(float(velocity["wz_rad_s"]))
        checks = []
        entry.update({
            "status": "SUCCEEDED" if report.get("failure") is None else report["failure"]["decision"],
            "failure": report.get("failure"),
            "displacement_m": displacement,
            "mean_speed_mps": mean_speed,
            "yaw_change_deg": yaw1 - yaw0,
            "mean_yaw_rate_rad_s": float(np.mean(yaw_rates)),
            "max_tilt_deg": float(np.max(tilts)),
            "control_cycles": report.get("control_cycles"),
            "ctrl_saturated_samples": report.get("ctrl_saturated_samples"),
            "provider_stats": (report.get("provider") or {}).get("stats"),
            "client_stats": report.get("client"),
            # 诊断序列（**不参与判据**）：每 10 拍取一个 + 末 5 拍，用于定位"从第几拍开始失控"。
            # 完整 samples 太大不入库；本序列含 倾角/支撑集/峰力矩/速度/位置。
            "diagnostics": ([{k: s.get(k) for k in ("time_s", "tilt_deg", "stance_legs",
                                                    "max_abs_ctrl_nm", "base_linear_speed_mps",
                                                    "base_position_xy_m", "base_yaw_deg",
                                                    "ctrl_saturated")}
                             for s in samples[::10]] + [s for s in samples[-5:]]),
            "samples_count": len(samples),
            "warmup": report.get("warmup"),
        })
        if cmd_speed > 0.0:                     # 直行/倒退：硬判据（声明 applies_when）
            rel = abs(mean_speed - cmd_speed) / cmd_speed
            checks.append({"name": "mean_rel_error", "value": rel,
                           "expectation": "<= %g" % max_rel, "passed": bool(rel <= max_rel)})
        else:                                   # 转向：按声明**仅记录**
            entry["yaw_rate_record_only"] = {
                "mean_yaw_rate_rad_s": float(np.mean(yaw_rates)), "commanded": velocity["wz_rad_s"]}
        checks.append({"name": "max_tilt_deg", "value": float(np.max(tilts)),
                       "expectation": "<= %g" % max_tilt, "passed": bool(np.max(tilts) <= max_tilt)})
        entry["checks"] = checks
        entry["passed"] = all(c["passed"] for c in checks) and report.get("failure") is None
        results.append(entry)
        print("  %-11s 位移=%.4f m 段内均速=%.4f m/s 偏航变化=%+.2f° 均偏航率=%+.4f rad/s "
              "最大倾角=%.2f° 失败=%s %s"
              % (name, displacement, mean_speed, yaw1 - yaw0, float(np.mean(yaw_rates)),
                 float(np.max(tilts)), report.get("failure"), "通过" if entry["passed"] else "**未通过**"))

    failed = [r["scenario"] for r in results if not r["passed"]]
    report = {
        "schema_version": "iraf.go2-locomote-verify/v1",
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "declaration": str(args.config),
        "criteria": {"steady_window_start_s": steady_start, "max_mean_rel_error": max_rel,
                     "max_tilt_moving_deg": max_tilt},
        "evidence": {"profile_sha256": _sha256(ROOT / acceptance["robot"]["profile"]),
                     "safety_policy_sha256": _sha256(ROOT / acceptance["safety_policy"]),
                     "control_frequency_hz": float(profile.control_frequency_hz),
                     "friction_pairing": "我们的 mu=0.4 与厂商 MJCF 足端摩擦 0.4 成对（见 mpc_model.mu）",
                     "qp_overrun_count": sum(
                         int(((r.get("provider_stats") or {}).get("overran_cycles")) or 0)
                         for r in results)},
        "scenarios": results,
        "passed": not failed,
        "failed_scenarios": failed,
        "note": "本入口直接调适配器（未声明能力）；能力回填还须 aarch64 板复测（铁律 6.8）",
    }
    path = ROOT / args.report
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("报告：%s" % path)
    return EXIT_OK if not failed else EXIT_FAILED


if __name__ == "__main__":
    raise SystemExit(main())
