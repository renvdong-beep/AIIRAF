#!/usr/bin/env python3
"""生成 development simulation 的可追溯一键验收证据包。"""

import argparse
import datetime as dt
import hashlib
import importlib.metadata
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ENV_KEYS = (
    "IRAF_PROFILE",
    "IRAF_SAFETY_POLICY",
    "IRAF_SKILL_ROOT",
    "IRAF_BACKEND_ENTRYPOINT",
    "IRAF_BACKEND_CONFIG",
    "IRAF_EVENT_STORE",
)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _display_path(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def _is_evidence_path(path):
    path = Path(path).resolve()
    allowed_roots = (PROJECT_ROOT / "build", PROJECT_ROOT / "artifacts")
    return any(path == root or root in path.parents for root in allowed_roots)


def _text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _run_command(name, argv, output_dir, timeout_seconds, env=None):
    log_path = Path(output_dir) / "logs" / f"{name}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    exit_code = None
    timed_out = False
    try:
        completed = subprocess.run(
            [str(item) for item in argv],
            cwd=PROJECT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
        exit_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        stdout = _text(exc.stdout)
        stderr = _text(exc.stderr) + f"\n验收命令超过 {timeout_seconds} 秒并已终止\n"
    duration_ms = round((time.monotonic() - started) * 1000.0, 3)
    log_path.write_text(
        "[stdout]\n" + _text(stdout) + "\n[stderr]\n" + _text(stderr),
        encoding="utf-8",
    )
    return {
        "name": name,
        "passed": bool(not timed_out and exit_code == 0),
        "exit_code": exit_code,
        "timed_out": timed_out,
        "duration_ms": duration_ms,
        "log": _display_path(log_path),
        "log_sha256": _sha256(log_path),
    }


def _read_json(path):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def _runtime_environment():
    env = dict(os.environ)
    if all(env.get(key) for key in RUNTIME_ENV_KEYS):
        _append_project_python_paths(env)
        return env, []
    try:
        pids = subprocess.check_output(
            ["pgrep", "-f", "iraf_adapters.http.runtime_http"],
            cwd=PROJECT_ROOT,
            text=True,
        ).split()
    except (OSError, subprocess.CalledProcessError):
        pids = []
    for pid in reversed(pids):
        try:
            process_env = {}
            for item in Path("/proc", pid, "environ").read_bytes().split(b"\0"):
                if b"=" in item:
                    key, value = item.split(b"=", 1)
                    process_env[key.decode()] = value.decode()
            if all(process_env.get(key) for key in RUNTIME_ENV_KEYS):
                env.update(process_env)
                break
        except (OSError, UnicodeDecodeError):
            continue
    missing = [key for key in RUNTIME_ENV_KEYS if not env.get(key)]
    _append_project_python_paths(env)
    return env, missing


def _append_project_python_paths(env):
    """补齐受控脚本兼容入口，保留 Runtime 进程的其余环境。"""
    tools_path = str(PROJECT_ROOT / "tools")
    paths = [
        value for value in env.get("PYTHONPATH", "").split(os.pathsep) if value
    ]
    if tools_path not in paths:
        paths.append(tools_path)
    env["PYTHONPATH"] = os.pathsep.join(paths)


def _runtime_health(base_url, timeout_seconds=5.0):
    url = base_url.rstrip("/") + "/health"
    try:
        with urllib.request.urlopen(url, timeout=timeout_seconds) as response:
            body = json.loads(response.read().decode("utf-8"))
            return {"http_status": response.status, "body": body}
    except urllib.error.HTTPError as exc:
        try:
            body = json.loads(exc.read().decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            body = {"reason": str(exc)}
        return {"http_status": exc.code, "body": body}
    except Exception as exc:
        return {
            "http_status": 0,
            "body": {"reason": f"{type(exc).__name__}: {exc}"},
        }


def _git_output(*args):
    try:
        return subprocess.check_output(
            ["git", *args], cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def _source_state():
    changed = _git_output("status", "--porcelain")
    changed_paths = [] if changed in {"", "unavailable"} else changed.splitlines()
    return {
        "commit": _git_output("rev-parse", "HEAD"),
        "branch": _git_output("branch", "--show-current"),
        "dirty": bool(changed_paths),
        "changed_path_count": len(changed_paths),
    }


def _dependency_versions():
    versions = {"python": sys.version.split()[0]}
    for package in ("mujoco", "protobuf", "PyYAML", "jsonschema", "grpcio"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "unavailable"
    return versions


def _artifact(path, kind):
    path = Path(path)
    if not path.is_file():
        return None
    return {
        "kind": kind,
        "path": _display_path(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _contract_artifacts(env):
    artifacts = []
    for key, kind in (
        ("IRAF_PROFILE", "robot_profile"),
        ("IRAF_SAFETY_POLICY", "safety_policy"),
    ):
        item = _artifact(env.get(key, ""), kind)
        if item:
            artifacts.append(item)
    skill_root = Path(env.get("IRAF_SKILL_ROOT", "missing"))
    if skill_root.is_dir():
        for pattern in ("skill.yaml", "input.schema.json", "output.schema.json"):
            for path in sorted(skill_root.glob(f"*/{pattern}")):
                artifacts.append(_artifact(path, "skill_contract"))
    return [item for item in artifacts if item]


def _evidence_artifacts(paths):
    return [item for item in (_artifact(path, "verification_report") for path in paths) if item]


def _successful_replay_checks(report, execution_id, require_intent=False):
    report = report or {}
    checks = {
        "schema": report.get("schema_version") == "iraf.execution-replay/v1",
        "execution": bool(execution_id) and report.get("execution_id") == execution_id,
        "terminal": report.get("terminal_status") == "SUCCEEDED",
        "events": [item.get("status") for item in report.get("events", [])]
        == ["PENDING", "VALIDATING", "RUNNING", "SUCCEEDED"],
        "simulation_boundary": report.get("simulation") is True,
    }
    if require_intent:
        provider = report.get("intent_provider") or {}
        checks.update(
            {
                "adapter": report.get("adapter") == "agentos-intent",
                "intent_provider": bool(provider.get("name") and provider.get("model")),
                "intent_request_digest": len(report.get("intent_request_digest", ""))
                == 64,
                "resolved_skill": bool(report.get("resolved_skill")),
            }
        )
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/acceptance/development-simulation"),
    )
    parser.add_argument("--command-timeout-seconds", type=float, default=60.0)
    parser.add_argument(
        "--base-url", default=os.environ.get("IRAF_RUNTIME_URL", "http://127.0.0.1:8765")
    )
    args = parser.parse_args(argv)
    if args.command_timeout_seconds <= 0:
        parser.error("command-timeout-seconds 必须为正数")
    output_dir = args.output.resolve()
    if not _is_evidence_path(output_dir):
        parser.error("output 必须位于仓库的 build/ 或 artifacts/ 目录")
    output_dir.mkdir(parents=True, exist_ok=True)
    env, missing_env = _runtime_environment()

    simulation_report_path = output_dir / "simulation-baseline.json"
    intent_failure_report_path = output_dir / "intent-failure-replay.json"
    minimal_report_path = output_dir / "minimal-chain.json"
    replay_report_path = output_dir / "execution-replay.json"
    intent_replay_report_path = output_dir / "intent-execution-replay.json"
    commands = [
        _run_command(
            "unit-tests",
            [sys.executable, "-m", "unittest", "discover", "-s", "tests/unit", "-v"],
            output_dir,
            args.command_timeout_seconds,
            env,
        ),
        _run_command(
            "simulation-baseline",
            [
                sys.executable,
                PROJECT_ROOT / "scripts" / "verify_simulation_baseline.py",
                "--output",
                simulation_report_path,
            ],
            output_dir,
            args.command_timeout_seconds,
            env,
        ),
        _run_command(
            "intent-failure-replay",
            [
                sys.executable,
                PROJECT_ROOT / "scripts" / "verify_intent_failure_replay.py",
                "--output",
                intent_failure_report_path,
            ],
            output_dir,
            args.command_timeout_seconds,
            env,
        ),
    ]
    commands.append(
        _run_command(
            "minimal-direct-and-intent-chain",
            [
                sys.executable,
                PROJECT_ROOT / "scripts" / "verify_minimal_chain.py",
                "--mode",
                "both",
                "--output",
                minimal_report_path,
            ],
            output_dir,
            args.command_timeout_seconds,
            env,
        )
    )
    health = _runtime_health(args.base_url)
    simulation_report = _read_json(simulation_report_path)
    intent_failure_report = _read_json(intent_failure_report_path)
    minimal_report = _read_json(minimal_report_path)
    direct_result = (minimal_report or {}).get("results", {}).get("direct_task", {})
    direct_status = direct_result.get("status")
    direct_execution_id = direct_result.get("execution_id", "")
    intent_result = (minimal_report or {}).get("results", {}).get(
        "agentos_intent", {}
    )
    intent_status = intent_result.get("status")
    intent_execution_id = intent_result.get("execution_id", "")
    commands.append(
        _run_command(
            "execution-replay",
            [
                sys.executable,
                PROJECT_ROOT / "scripts" / "export_replay_manifest.py",
                "--execution-id",
                direct_execution_id,
                "--output",
                replay_report_path,
            ],
            output_dir,
            args.command_timeout_seconds,
            env,
        )
    )
    commands.append(
        _run_command(
            "intent-execution-replay",
            [
                sys.executable,
                PROJECT_ROOT / "scripts" / "export_replay_manifest.py",
                "--execution-id",
                intent_execution_id,
                "--output",
                intent_replay_report_path,
            ],
            output_dir,
            args.command_timeout_seconds,
            env,
        )
    )
    replay_report = _read_json(replay_report_path)
    intent_replay_report = _read_json(intent_replay_report_path)
    direct_replay_checks = _successful_replay_checks(
        replay_report, direct_execution_id
    )
    intent_replay_checks = _successful_replay_checks(
        intent_replay_report, intent_execution_id, require_intent=True
    )
    health_body = health.get("body", {})
    health_simulation = health_body.get("simulation") or {}
    checks = {
        "runtime_environment": {"passed": not missing_env, "missing": missing_env},
        "unit_tests": {"passed": commands[0]["passed"]},
        "simulation_baseline": {
            "passed": bool(commands[1]["passed"] and (simulation_report or {}).get("passed"))
        },
        "intent_failure_replay": {
            "passed": bool(
                commands[2]["passed"]
                and (intent_failure_report or {}).get("passed")
            )
        },
        "minimal_direct_chain": {
            "passed": bool(commands[3]["passed"] and direct_status == "SUCCEEDED"),
            "terminal_status": direct_status,
        },
        "agentos_intent_chain": {
            "passed": bool(
                commands[3]["passed"]
                and intent_status == "SUCCEEDED"
                and (intent_result.get("intent_provider") or {}).get("model")
            ),
            "terminal_status": intent_status,
            "model": (intent_result.get("intent_provider") or {}).get("model", ""),
        },
        "execution_replay": {
            "passed": bool(
                commands[4]["passed"]
                and all(direct_replay_checks.values())
            ),
            "execution_id": direct_execution_id,
            "event_count": len((replay_report or {}).get("events", [])),
            "checks": direct_replay_checks,
        },
        "intent_execution_replay": {
            "passed": bool(
                commands[5]["passed"]
                and all(intent_replay_checks.values())
            ),
            "execution_id": intent_execution_id,
            "event_count": len((intent_replay_report or {}).get("events", [])),
            "checks": intent_replay_checks,
        },
        "runtime_health": {
            "passed": bool(
                health.get("http_status") == 200
                and health_body.get("status") == "ok"
                and health_simulation.get("healthy") is True
            ),
            "http_status": health.get("http_status"),
        },
    }
    manifest = {
        "schema_version": "iraf.development-simulation-evidence/v1",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "development-simulation",
        "simulation_only": True,
        "passed": all(item["passed"] for item in checks.values()),
        "source": _source_state(),
        "dependencies": _dependency_versions(),
        "checks": checks,
        "commands": commands,
        "runtime_health": health,
        "contract_artifacts": _contract_artifacts(env),
        "evidence_artifacts": _evidence_artifacts(
            [
                simulation_report_path,
                intent_failure_report_path,
                minimal_report_path,
                replay_report_path,
                intent_replay_report_path,
            ]
        ),
        "limitations": [
            "本证据包不证明真机、HIL、Linux-RT deadline 或 RTOS/现场总线能力。",
            "源码工作区为 dirty 时仅可作为开发证据，不可作为冻结发布制品。",
        ],
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    manifest_digest = _sha256(manifest_path)
    digest_path = output_dir / "manifest.sha256"
    digest_path.write_text(
        f"{manifest_digest}  {_display_path(manifest_path)}\n", encoding="ascii"
    )
    print(
        json.dumps(
            {
                "passed": manifest["passed"],
                "manifest": _display_path(manifest_path),
                "manifest_sha256": manifest_digest,
            },
            ensure_ascii=True,
            indent=2,
        )
    )
    return 0 if manifest["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
