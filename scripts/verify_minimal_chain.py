#!/usr/bin/env python3
"""Verify the development AgentOS -> IRAF -> Piper MuJoCo chain."""
import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import urllib.error
import urllib.request
from pathlib import Path


def _process_env():
    token = os.environ.get("IRAF_RUNTIME_TOKEN")
    if token:
        return dict(os.environ)
    try:
        pids = subprocess.check_output(["pgrep", "-f", "iraf_adapters.http.runtime_http"], text=True).split()
    except (OSError, subprocess.CalledProcessError):
        pids = []
    for pid in reversed(pids):
        try:
            values = {}
            for item in Path("/proc", pid, "environ").read_bytes().split(b"\0"):
                if b"=" in item:
                    key, value = item.split(b"=", 1)
                    values[key.decode()] = value.decode()
            if values.get("IRAF_RUNTIME_TOKEN"):
                return values
        except (OSError, UnicodeDecodeError):
            continue
    return dict(os.environ)


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _post(url, token, body):
    request = urllib.request.Request(url, data=json.dumps(body, ensure_ascii=False).encode(), headers={"Content-Type": "application/json", "Authorization": "Bearer " + token}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=35) as response:
            return response.status, json.loads(response.read().decode())
    except urllib.error.HTTPError as exc:
        try:
            payload = json.loads(exc.read().decode())
        except (ValueError, UnicodeDecodeError):
            payload = {"reason": str(exc)}
        return exc.code, payload
    except Exception as exc:
        return 0, {"error_code": "IRAF-TRANSPORT-FAILED", "reason": str(exc)}


def _request_base():
    return {
        "request_id": "minimal-chain-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S"),
        "idempotency_key": "minimal-chain-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S%f"),
        "correlation_id": "minimal-chain-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d"),
        "deadline": (dt.datetime.now(dt.timezone.utc) + dt.timedelta(seconds=30)).isoformat().replace("+00:00", "Z"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default=os.environ.get("IRAF_RUNTIME_URL", "http://127.0.0.1:8765"))
    parser.add_argument("--mode", choices=("direct", "intent", "both"), default="both")
    parser.add_argument("--text", default="将关节1移动到0.2")
    parser.add_argument("--output", type=Path, default=Path("build/acceptance/minimal-chain.json"))
    args = parser.parse_args(argv)
    env = _process_env()
    token = env.get("IRAF_RUNTIME_TOKEN", "")
    if not token:
        parser.error("IRAF_RUNTIME_TOKEN is not available; export it or run as the runtime user")
    profile_path = env.get("IRAF_PROFILE", "profiles/piper_mujoco.yaml")
    safety_path = env.get("IRAF_SAFETY_POLICY", "profiles/safety/simulation_lab.yaml")
    profile = {"name": "piper_mujoco", "version": "0.1.0", "digest": _digest(profile_path)}
    safety = {"name": "simulation_lab", "version": "1.0.0", "digest": _digest(safety_path)}
    base = _request_base()
    results = {}
    if args.mode in {"direct", "both"}:
        direct = {"request_id": base["request_id"] + "-direct", "idempotency_key": base["idempotency_key"] + "-direct"}
        direct["goal"] = {"correlation_id": base["correlation_id"], "skill_name": "move_joint", "skill_version_constraint": "1.0.0", "inputs": {"positions": {"joint1": 0.2}, "duration_ms": 100}, "deadline": base["deadline"], "robot_profile": profile, "safety_policy": safety, "labels": {"resource_id": "piper_mujoco", "controller": "minimal-chain"}}
        status, payload = _post(args.base_url.rstrip("/") + "/v1/tasks", token, direct)
        results["direct_task"] = {"http_status": status, "status": payload.get("status", ""), "error_code": payload.get("error_code", ""), "reason": payload.get("reason", ""), "skill": payload.get("skill", {}), "provider": payload.get("provider", {}), "execution_id": payload.get("execution_id", "")}
    if args.mode in {"intent", "both"}:
        intent = dict(base)
        intent["request_id"] += "-intent"
        intent["idempotency_key"] += "-intent"
        intent["text"] = args.text
        intent["resource_id"] = "piper_mujoco"
        intent["controller"] = "minimal-chain"
        status, payload = _post(args.base_url.rstrip("/") + "/v1/intents", token, intent)
        results["agentos_intent"] = {"http_status": status, "status": payload.get("status", ""), "error_code": payload.get("error_code", ""), "reason": payload.get("reason", ""), "resolved_skill": payload.get("resolved_skill", ""), "provider": payload.get("provider", {}), "execution_id": payload.get("execution_id", ""), "intent_provider": payload.get("intent_provider", {})}
    report = {"schema_version": "iraf.minimal-chain.verification/v1", "mode": args.mode, "base_url": args.base_url, "simulation_only": True, "results": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True, indent=2))
    direct_ok = results.get("direct_task", {}).get("status") == "SUCCEEDED"
    intent_ok = results.get("agentos_intent", {}).get("status") == "SUCCEEDED"
    if args.mode == "direct":
        return 0 if direct_ok else 1
    if args.mode == "intent":
        return 0 if intent_ok else 2
    return 0 if direct_ok and intent_ok else (1 if not direct_ok else 2)


if __name__ == "__main__":
    raise SystemExit(main())
