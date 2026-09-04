#!/usr/bin/env python3
"""IRAF 受控任务规划器：把版本化计划拆成可验证的 CodingTask。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import time

TASK_ID = re.compile(r"^[a-z0-9][a-z0-9._-]{2,63}$")
PROFILES = {"unit", "development_simulation"}


def load_plan(path: Path) -> dict:
    plan = json.loads(path.read_text(encoding="utf-8"))
    if plan.get("version") != 1 or not isinstance(plan.get("tasks"), list):
        raise ValueError("计划必须包含 version=1 和 tasks 列表")
    ids = set()
    for item in plan["tasks"]:
        task_id = item.get("id")
        if not isinstance(task_id, str) or not TASK_ID.fullmatch(task_id) or task_id in ids:
            raise ValueError(f"任务 id 无效或重复: {task_id}")
        ids.add(task_id)
        if not isinstance(item.get("title"), str) or not item["title"].strip():
            raise ValueError(f"任务 title 不能为空: {task_id}")
        if item.get("verification_profile") not in PROFILES:
            raise ValueError(f"任务验证 profile 不在白名单: {task_id}")
        deps = item.get("depends_on", [])
        if not isinstance(deps, list) or any(not isinstance(dep, str) for dep in deps):
            raise ValueError(f"depends_on 无效: {task_id}")
    for item in plan["tasks"]:
        unknown = set(item.get("depends_on", [])) - ids
        if unknown:
            raise ValueError(f"{item['id']} 依赖未知任务: {sorted(unknown)}")
    _check_acyclic(plan["tasks"])
    return plan


def _check_acyclic(tasks: list[dict]) -> None:
    graph = {item["id"]: set(item.get("depends_on", [])) for item in tasks}
    resolved = set()
    while graph:
        ready = {task_id for task_id, deps in graph.items() if not deps - resolved}
        if not ready:
            raise ValueError("任务依赖存在环")
        resolved.update(ready)
        for task_id in ready:
            graph.pop(task_id)


def passed_ids(evidence: Path) -> set[str]:
    result = set()
    for item in evidence.glob("*.json"):
        try:
            data = json.loads(item.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if data.get("passed") is True:
            result.add(data.get("task_id", item.stem))
    return result


def emit_ready(plan: dict, plan_path: Path, queue: Path, evidence: Path) -> list[str]:
    queue.mkdir(parents=True, exist_ok=True)
    done = queue.parent / "done"
    done.mkdir(parents=True, exist_ok=True)
    known = {p.stem for p in queue.glob("*.json")} | {p.stem for p in done.glob("*.json")}
    completed = passed_ids(evidence)
    emitted = []
    for item in plan["tasks"]:
        task_id = item["id"]
        if item.get("status", "ready") != "ready":
            continue
        if task_id in known or not set(item.get("depends_on", [])) <= completed:
            continue
        source_patch = (plan_path.parent / item["patch"]).resolve()
        if not source_patch.is_file() or plan_path.parent.resolve() not in source_patch.parents:
            raise ValueError(f"patch 必须存在于计划目录内: {item['patch']}")
        patch_name = f"{task_id}.patch"
        shutil.copyfile(source_patch, queue / patch_name)
        task = {"version": 1, "id": task_id, "title": item["title"],
                "verification_profile": item["verification_profile"], "patch": patch_name}
        (queue / f"{task_id}.json").write_text(json.dumps(task, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        emitted.append(task_id)
    return emitted


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--queue", type=Path, default=Path("/etc/iraf/coding-queue/inbox"))
    parser.add_argument("--evidence", type=Path, default=Path("/home/coretek/AIIRAF/build/coding-worker"))
    parser.add_argument("--poll-seconds", type=int, default=1800)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    plan = load_plan(args.plan)
    while True:
        emitted = emit_ready(plan, args.plan, args.queue, args.evidence)
        print(json.dumps({"plan_id": plan.get("plan_id", args.plan.stem), "emitted": emitted}, ensure_ascii=False), flush=True)
        if args.once:
            return 0
        time.sleep(max(30, args.poll_seconds))


if __name__ == "__main__":
    raise SystemExit(main())
