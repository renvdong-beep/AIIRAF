#!/usr/bin/env python3
"""从 canonical EventStore 导出脱敏的 execution replay manifest。"""

import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

from iraf_core.store import SqliteExecutionStore


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _display_path(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def _is_output_path(path):
    path = Path(path).resolve()
    roots = (PROJECT_ROOT / "build", PROJECT_ROOT / "artifacts")
    return any(path == root or root in path.parents for root in roots)


def _runtime_event_store():
    configured = os.environ.get("IRAF_EVENT_STORE")
    if configured:
        return configured
    try:
        pids = subprocess.check_output(
            ["pgrep", "-f", "iraf_adapters.http.runtime_http"], text=True
        ).split()
    except (OSError, subprocess.CalledProcessError):
        pids = []
    for pid in reversed(pids):
        try:
            for item in Path("/proc", pid, "environ").read_bytes().split(b"\0"):
                if item.startswith(b"IRAF_EVENT_STORE="):
                    return item.split(b"=", 1)[1].decode()
        except (OSError, UnicodeDecodeError):
            continue
    return ""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execution-id", required=True)
    parser.add_argument("--event-store", default="")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    event_store = args.event_store or _runtime_event_store()
    if not event_store:
        parser.error("缺少 IRAF_EVENT_STORE，无法导出回放索引")
    output = (
        args.output
        if args.output is not None
        else PROJECT_ROOT / "build" / "replay" / f"{args.execution_id}.json"
    ).resolve()
    if not _is_output_path(output):
        parser.error("output 必须位于仓库的 build/ 或 artifacts/ 目录")

    manifest = SqliteExecutionStore(event_store).get_replay_manifest(
        args.execution_id
    )
    if manifest is None:
        print(
            json.dumps(
                {
                    "status": "NOT_FOUND",
                    "execution_id": args.execution_id,
                    "reason": "未找到可回放的持久化执行",
                },
                ensure_ascii=True,
            )
        )
        return 2
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(manifest, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    digest = _sha256(output)
    digest_path = output.with_suffix(output.suffix + ".sha256")
    digest_path.write_text(
        f"{digest}  {_display_path(output)}\n", encoding="ascii"
    )
    print(
        json.dumps(
            {
                "status": "EXPORTED",
                "execution_id": args.execution_id,
                "manifest": _display_path(output),
                "sha256": digest,
            },
            ensure_ascii=True,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
