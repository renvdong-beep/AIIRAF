#!/usr/bin/env python3
"""验证 AgentOS intent Provider 失联时无动作并生成可回放失败。"""

import argparse
import datetime as dt
import json
import os
import subprocess
from pathlib import Path

from iraf_adapters.agentos.bridge import AgentOSBridge
from iraf_adapters.agentos.dispatcher import TaskDispatcher
from iraf_adapters.bootstrap import build_runtime_from_env
from iraf_adapters.model.openai_intent import IntentProviderError
from iraf_core.policy import AuthenticatedContext


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_ENV = (
    "IRAF_PROFILE",
    "IRAF_SAFETY_POLICY",
    "IRAF_SKILL_ROOT",
    "IRAF_BACKEND_ENTRYPOINT",
    "IRAF_BACKEND_CONFIG",
    "IRAF_EVENT_STORE",
)


class UnavailableIntentProvider:
    name = "fault-injection"
    version = "1.0.0"
    model_id = "unavailable-model"

    def parse(self, text, allowed):
        raise IntentProviderError("注入的 intent Provider 不可用")


class FailOnBackendAccess:
    """Provider 失败路径不应读取库存，更不应调用任何动作。"""

    def __init__(self):
        self.accesses = []

    def __getattr__(self, name):
        self.accesses.append(name)
        raise AssertionError("意图解析失败后不应访问动作后端: " + name)


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


def _is_output_path(path):
    path = Path(path).resolve()
    roots = (PROJECT_ROOT / "build", PROJECT_ROOT / "artifacts")
    return any(path == root or root in path.parents for root in roots)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/acceptance/intent-failure-replay.json"),
    )
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not _is_output_path(output):
        parser.error("output 必须位于仓库的 build/ 或 artifacts/ 目录")
    _load_runtime_environment()
    runtime = build_runtime_from_env()
    backend_guard = FailOnBackendAccess()
    runtime.backend = backend_guard
    bridge = AgentOSBridge(UnavailableIntentProvider(), TaskDispatcher(runtime))
    suffix = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d%H%M%S%f")
    request = {
        "request_id": "intent-failure-" + suffix,
        "correlation_id": "intent-failure-" + suffix,
        "idempotency_key": "intent-failure-" + suffix,
        "text": "该文本不得进入回放 manifest",
        "deadline_unix_ms": int(
            (dt.datetime.now(dt.timezone.utc) + dt.timedelta(seconds=10)).timestamp()
            * 1000
        ),
        "resource_id": runtime.resource_id,
        "controller": "intent-failure-verification",
    }
    context = AuthenticatedContext(
        "intent-failure-verifier", frozenset({"task.submit", "task.read"}), "local"
    )
    result = bridge.execute_intent(request, context)
    replay = runtime.store.get_replay_manifest(result["execution_id"])
    statuses = [event["status"] for event in (replay or {}).get("events", [])]
    replay_text = json.dumps(replay, ensure_ascii=False, sort_keys=True)
    checks = {
        "explicit_failure": result.get("error_code") == "IRAF-INTENT-PARSE-FAILED",
        "no_backend_access": backend_guard.accesses == [],
        "terminal_events": statuses == ["PENDING", "VALIDATING", "FAILED"],
        "intent_identity": (replay or {}).get("intent_provider", {}).get("model")
        == "unavailable-model",
        "no_raw_intent": request["text"] not in replay_text,
        "simulation_boundary": (replay or {}).get("simulation") is True,
    }
    report = {
        "schema_version": "iraf.intent-failure-replay/v1",
        "simulation_only": True,
        "passed": all(checks.values()),
        "checks": checks,
        "result": result,
        "replay": replay,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
