#!/usr/bin/env python3
"""验证 IRAF 24 小时调度运行的完整性，不修改任何源码或证据。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


def fail(checks, name, reason):
    checks[name] = {"passed": False, "reason": reason}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args(argv)
    manifest_path = args.manifest.resolve()
    checks = {}
    if not manifest_path.is_file():
        print(json.dumps({"passed": False, "reason": "manifest 不存在"}, ensure_ascii=True))
        return 1
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print(json.dumps({"passed": False, "reason": f"manifest 无法读取: {exc}"}, ensure_ascii=True))
        return 1

    checks["schema"] = {"passed": manifest.get("schema_version") == "iraf.orchestrator-run/v1"}
    source = manifest.get("source") or {}
    checks["source"] = {
        "passed": bool(source.get("commit")) and source.get("dirty") is False,
        "commit": source.get("commit"),
        "dirty": source.get("dirty"),
    }
    required = float(manifest.get("required_duration_seconds", 0))
    elapsed = float(manifest.get("duration_seconds", 0))
    checks["duration"] = {"passed": required > 0 and elapsed >= required, "required": required, "actual": elapsed}
    minimum = int(manifest.get("minimum_cycles", 0))
    cycles = manifest.get("cycles")
    checks["cycles"] = {
        "passed": isinstance(cycles, list) and len(cycles) >= minimum and minimum > 0,
        "required": minimum,
        "actual": len(cycles) if isinstance(cycles, list) else 0,
    }
    if not isinstance(cycles, list) or not cycles:
        fail(checks, "cycle_results", "manifest 没有周期结果")
    else:
        failed = [item.get("cycle_id", "unknown") for item in cycles if item.get("passed") is not True]
        checks["cycle_results"] = {"passed": not failed, "failed_cycle_ids": failed}

    heartbeat = manifest_path.parent / "heartbeat.json"
    if not heartbeat.is_file():
        fail(checks, "heartbeat", "heartbeat.json 不存在")
    else:
        try:
            heartbeat_data = json.loads(heartbeat.read_text(encoding="utf-8"))
            checks["heartbeat"] = {
                "passed": heartbeat_data.get("cycles_completed") == len(cycles or [])
                and heartbeat_data.get("last_cycle_passed") is True,
                "cycles_completed": heartbeat_data.get("cycles_completed"),
            }
        except (OSError, ValueError) as exc:
            fail(checks, "heartbeat", f"heartbeat 无法读取: {exc}")

    cycle_dir = manifest_path.parent / "cycles"
    missing = []
    if isinstance(cycles, list):
        for cycle in cycles:
            cycle_id = cycle.get("cycle_id")
            if not cycle_id or not (cycle_dir / f"{cycle_id}.json").is_file():
                missing.append(cycle_id or "unknown")
    checks["cycle_evidence"] = {"passed": not missing, "missing_cycle_ids": missing}

    digest_path = manifest_path.parent / "manifest.sha256"
    expected = ""
    if digest_path.is_file():
        expected = digest_path.read_text(encoding="ascii").split()[0]
    actual_digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    checks["manifest_digest"] = {"passed": expected == actual_digest, "sha256": actual_digest}
    passed = bool(manifest.get("passed")) and all(item.get("passed") is True for item in checks.values())
    result = {"schema_version": "iraf.24h-verification/v1", "manifest": str(manifest_path), "passed": passed, "checks": checks}
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
