#!/usr/bin/env python3
"""受控的 IRAF 24 小时验证调度器；不执行任意远程代码修改。"""
from __future__ import annotations
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shlex
import ssl
import subprocess
import time
import urllib.error
import urllib.request
import uuid

ROOT = Path(__file__).resolve().parents[1]
ALLOWED_KINDS = {"verify_development_simulation", "edge_model_health"}

def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()

def git_value(*args):
    try:
        return subprocess.check_output(["git", *args], cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"

def load_config(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("version") != 1:
        raise ValueError("调度配置 version 必须为 1")
    tasks = value.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("调度配置必须包含 tasks")
    for task in tasks:
        if task.get("kind") not in ALLOWED_KINDS:
            raise ValueError("禁止的任务类型: " + str(task.get("kind")))
    return value

def target_from_env(spec, label):
    env_name = spec.get("target_env", "")
    target = os.environ.get(env_name, "") if env_name else ""
    if not target:
        raise RuntimeError(f"{label} 未配置 SSH target 环境变量 {env_name}")
    return target

def run_ssh(target, command, timeout):
    started = time.monotonic()
    try:
        result = subprocess.run(
            ["ssh", target, command],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "passed": result.returncode == 0,
            "exit_code": result.returncode,
            "duration_ms": round((time.monotonic() - started) * 1000, 3),
            "stdout": result.stdout[-12000:],
            "stderr": result.stderr[-4000:],
            "command": "ssh " + shlex.quote(target) + " <controlled-command>",
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "passed": False,
            "exit_code": None,
            "duration_ms": round((time.monotonic() - started) * 1000, 3),
            "stdout": str(exc.stdout or "")[-12000:],
            "stderr": "SSH/远程验收超时",
            "command": "ssh " + shlex.quote(target) + " <controlled-command>",
        }

def runtime_task(config, task, cycle_id):
    runtime = config.get("runtime") or {}
    target = target_from_env(runtime, "247.145 Runtime")
    repo = runtime.get("repo", "")
    python = runtime.get("python", "python3")
    if not repo.startswith("/"):
        raise ValueError("Runtime repo 必须为绝对路径")
    remote_output = f"build/acceptance/development-simulation/orchestrator-{cycle_id}"
    command = (
        f"cd {shlex.quote(repo)} && "
        "test -z "$(git status --porcelain)" && "
        f"PYTHONPATH=src:build/generated/python:adapters:adapters/agentos:adapters/grpc:"
        f"adapters/http:adapters/mujoco:adapters/model:skills/common:skills/piper:tools "
        f"{shlex.quote(python)} scripts/verify_development_simulation.py "
        f"--output {shlex.quote(remote_output)} --command-timeout-seconds 120"
    )
    result = run_ssh(target, command, 240)
    result.update({"task_id": task["id"], "kind": task["kind"], "target": "runtime", "remote_output": remote_output})
    if not result["passed"] and result["exit_code"] == 1 and "git status" in result["stderr"]:
        result["reason"] = "247.145 工作区 dirty，已阻止验收"
    return result

def edge_task(config, task):
    edge = config.get("edge") or {}
    url = os.environ.get(edge.get("models_url_env", ""), "")
    token = os.environ.get(edge.get("token_env", ""), "")
    if not url or not token:
        return {
            "passed": False,
            "blocked": True,
            "task_id": task["id"],
            "kind": task["kind"],
            "target": "edge",
            "reason": "未提供边缘模型 URL 或 Token；不在调度日志中请求或保存凭据",
        }
    request = urllib.request.Request(
        url.rstrip("/") + "/models",
        headers={"Authorization": "Bearer " + token},
    )
    context = ssl._create_unverified_context() if edge.get("verify_tls") is False else None
    try:
        with urllib.request.urlopen(request, timeout=15, context=context) as response:
            payload = json.loads(response.read().decode("utf-8"))
            models = payload.get("data") if isinstance(payload, dict) else None
            return {
                "passed": response.status == 200 and isinstance(models, list),
                "task_id": task["id"],
                "kind": task["kind"],
                "target": "edge",
                "http_status": response.status,
                "model_count": len(models) if isinstance(models, list) else 0,
            }
    except (urllib.error.URLError, ValueError) as exc:
        return {
            "passed": False,
            "task_id": task["id"],
            "kind": task["kind"],
            "target": "edge",
            "reason": f"{type(exc).__name__}: {exc}",
        }

def run_cycle(config, cycle_id):
    results = []
    for task in config["tasks"]:
        if task.get("enabled", True) is False:
            continue
        if task["kind"] == "verify_development_simulation":
            results.append(runtime_task(config, task, cycle_id))
        elif task["kind"] == "edge_model_health":
            results.append(edge_task(config, task))
    return results

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("build/orchestrator"))
    parser.add_argument("--duration-hours", type=float, default=0)
    parser.add_argument("--interval-seconds", type=float, default=None)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    if args.duration_hours < 0:
        parser.error("duration-hours 不能为负数")
    config = load_config(args.config)
    interval = args.interval_seconds if args.interval_seconds is not None else float(config.get("interval_seconds", 3600))
    if interval <= 0:
        parser.error("interval-seconds 必须为正数")
    output = args.output.resolve()
    if not any(part in {"build", "artifacts"} for part in output.parts):
        parser.error("output 必须位于 build/ 或 artifacts/ 目录")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    deadline = started + args.duration_hours * 3600 if args.duration_hours else started
    cycles = []
    while True:
        cycle_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
        cycle = {"cycle_id": cycle_id, "started_at": now(), "results": run_cycle(config, cycle_id)}
        cycle["passed"] = bool(cycle["results"]) and all(item.get("passed") for item in cycle["results"])
        cycles.append(cycle)
        if args.once or not args.duration_hours or time.monotonic() >= deadline:
            break
        if not cycle["passed"]:
            cycle["halted"] = True
            break
        time.sleep(min(interval, max(0, deadline - time.monotonic())))
    manifest = {
        "schema_version": "iraf.orchestrator-run/v1",
        "created_at": now(),
        "source": {"commit": git_value("rev-parse", "HEAD"), "branch": git_value("branch", "--show-current"), "dirty": bool(git_value("status", "--porcelain"))},
        "scope": "development-simulation",
        "cycles": cycles,
        "passed": bool(cycles) and all(cycle.get("passed") for cycle in cycles),
        "policy": {"allowed_kinds": sorted(ALLOWED_KINDS), "code_mutation": "forbidden"},
    }
    path = output / "manifest.json"
    path.write_text(json.dumps(manifest, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / "manifest.sha256").write_text(f"{digest}  manifest.json\n", encoding="ascii")
    print(json.dumps({"passed": manifest["passed"], "manifest": str(path), "manifest_sha256": digest, "cycles": len(cycles)}, ensure_ascii=True))
    return 0 if manifest["passed"] else 1

if __name__ == "__main__":
    raise SystemExit(main())
