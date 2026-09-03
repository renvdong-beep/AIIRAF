#!/usr/bin/env python3
"""验证 MuJoCo 连续步进、指标、故障锁存和显式恢复基线。"""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

from iraf_adapters.factory import load_backend
from iraf_adapters.mujoco.supervisor import MujocoSimulationSupervisor
from iraf_core.authority import ControlAuthorityManager
from iraf_core.profile import load_robot_profile


REQUIRED_ENV = ("IRAF_PROFILE", "IRAF_BACKEND_ENTRYPOINT", "IRAF_BACKEND_CONFIG")


def _load_runtime_environment():
    if all(os.environ.get(key) for key in REQUIRED_ENV):
        return
    try:
        pids = subprocess.check_output(
            ["pgrep", "-f", "iraf_adapters.http.runtime_http"], text=True
        ).split()
    except (OSError, subprocess.CalledProcessError):
        pids = []
    for pid in reversed(pids):
        try:
            values = {}
            for item in Path("/proc", pid, "environ").read_bytes().split(b"\0"):
                if b"=" in item:
                    key, value = item.split(b"=", 1)
                    values[key.decode()] = value.decode()
            if all(values.get(key) for key in REQUIRED_ENV):
                os.environ.update({key: values[key] for key in REQUIRED_ENV})
                return
        except (OSError, UnicodeDecodeError):
            continue
    missing = [key for key in REQUIRED_ENV if not os.environ.get(key)]
    raise RuntimeError("缺少 Runtime 环境配置: " + ", ".join(missing))


def _wait_until(predicate, timeout_seconds, reason):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise RuntimeError("等待超时: " + reason)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-seconds", type=float, default=0.5)
    parser.add_argument("--timeout-seconds", type=float, default=3.0)
    parser.add_argument("--min-frequency-ratio", type=float, default=0.5)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/acceptance/simulation-baseline.json"),
    )
    args = parser.parse_args(argv)
    if args.sample_seconds <= 0 or args.timeout_seconds <= 0:
        parser.error("采样和超时时间必须为正数")
    if not 0 < args.min_frequency_ratio <= 1:
        parser.error("min-frequency-ratio 必须在 0 到 1 之间")

    _load_runtime_environment()
    profile = load_robot_profile(os.environ["IRAF_PROFILE"])
    if not profile.simulation:
        raise RuntimeError("仿真基线仅允许 simulation=true 的 RobotProfile")
    config = json.loads(os.environ["IRAF_BACKEND_CONFIG"])
    config["fault_injection_enabled"] = True
    backend = load_backend(
        os.environ["IRAF_BACKEND_ENTRYPOINT"],
        config,
        profile,
        ControlAuthorityManager(),
    )
    supervisor = MujocoSimulationSupervisor(backend)
    report = {
        "schema_version": "iraf.mujoco.simulation-baseline/v1",
        "simulation_only": True,
        "profile": {
            "name": profile.name,
            "version": profile.version,
            "digest": profile.digest,
        },
        "checks": {},
    }
    exit_code = 1
    try:
        supervisor.start()
        _wait_until(
            lambda: supervisor.diagnostics()["step_count"] >= 3,
            args.timeout_seconds,
            "连续步进启动",
        )
        time.sleep(args.sample_seconds)
        steady = supervisor.diagnostics()
        frequency_ratio = (
            steady["effective_frequency_hz"] / steady["target_frequency_hz"]
            if steady["target_frequency_hz"] > 0
            else 0.0
        )
        report["checks"]["continuous_step"] = {
            "passed": bool(
                steady["healthy"]
                and steady["step_count"] >= 3
                and frequency_ratio >= args.min_frequency_ratio
            ),
            "frequency_ratio": round(frequency_ratio, 3),
            "metrics": steady,
        }

        delay_ms = max(20.0, 4000.0 / steady["target_frequency_hz"])
        overruns_before = steady["step_overrun_count"]
        supervisor.inject_fault("step_delay", duration_ms=delay_ms)
        _wait_until(
            lambda: supervisor.diagnostics()["step_overrun_count"]
            > overruns_before,
            args.timeout_seconds,
            "步进延迟故障形成超限指标",
        )
        delayed = supervisor.diagnostics()
        report["checks"]["step_delay"] = {
            "passed": delayed["healthy"]
            and delayed["step_overrun_count"] > overruns_before,
            "metrics": delayed,
        }

        supervisor.inject_fault("step_failure")
        _wait_until(
            lambda: supervisor.diagnostics()["last_error"] is not None,
            args.timeout_seconds,
            "步进故障锁存",
        )
        failed = supervisor.diagnostics()
        report["checks"]["fail_closed"] = {
            "passed": bool(
                not failed["healthy"]
                and not failed["running"]
                and failed["step_failure_count"] == 1
                and backend.stopped
            ),
            "controls_stopped": bool(backend.stopped),
            "metrics": failed,
        }

        restart_rejected = False
        try:
            supervisor.start()
        except RuntimeError:
            restart_rejected = True
        supervisor.stop()
        supervisor.clear_faults()
        supervisor.start()
        _wait_until(
            lambda: supervisor.diagnostics()["step_count"] >= 3,
            args.timeout_seconds,
            "清除故障后恢复连续步进",
        )
        recovered = supervisor.diagnostics()
        report["checks"]["explicit_recovery"] = {
            "passed": bool(restart_rejected and recovered["healthy"]),
            "restart_rejected_before_clear": restart_rejected,
            "metrics": recovered,
        }
        exit_code = (
            0
            if all(check["passed"] for check in report["checks"].values())
            else 1
        )
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        supervisor.stop()
        report["passed"] = exit_code == 0
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=True, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=True, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
