#!/usr/bin/env python3
"""受控 IRAF 编码任务 worker：只消费任务包，不执行任意命令。"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

TASK_ID = re.compile(r"^[a-z0-9][a-z0-9._-]{2,63}$")
PYTHON = "/home/coretek/miniconda3/envs/mujoco_graspnet/bin/python"
PYTHONPATH = "src:build/generated/python:adapters:adapters/agentos:adapters/grpc:adapters/http:adapters/mujoco:adapters/model:skills/common:skills/piper:tools"


def stamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def run(cmd, cwd, timeout=900):
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, timeout=timeout, check=False)


def verify_task(task):
    if task.get("version") != 1 or not isinstance(task.get("id"), str) or not TASK_ID.fullmatch(task["id"]):
        raise ValueError("任务 version 或 id 无效")
    if not isinstance(task.get("title"), str) or not task["title"].strip():
        raise ValueError("任务 title 不能为空")
    if task.get("verification_profile") not in {"unit", "development_simulation"}:
        raise ValueError("verification_profile 不在白名单")
    patch = task.get("patch")
    if not isinstance(patch, str) or Path(patch).is_absolute() or ".." in Path(patch).parts:
        raise ValueError("patch 必须是任务目录内的相对路径")
    return task


def verify_command(profile, cwd):
    env = dict(os.environ)
    env["PYTHONPATH"] = PYTHONPATH
    if profile == "unit":
        return [PYTHON, "-m", "unittest", "discover", "-s", "tests/unit", "-p", "test_*.py"], env
    return [PYTHON, "scripts/verify_development_simulation.py", "--output", "build/coding-worker-verification"], env


def process_task(repo, queue, worktree, task_path, evidence_root):
    started = stamp()
    task = verify_task(json.loads(task_path.read_text(encoding="utf-8")))
    task_dir = task_path.parent
    patch_path = task_dir / task["patch"]
    if not patch_path.is_file():
        raise FileNotFoundError(f"patch 不存在: {patch_path}")
    if subprocess.run(["git", "status", "--porcelain"], cwd=repo, capture_output=True, text=True, check=False).stdout:
        raise RuntimeError("主仓库工作区 dirty，拒绝领取编码任务")
    base = run(["git", "rev-parse", "origin/main"], repo, 30)
    if base.returncode != 0:
        raise RuntimeError("无法解析 origin/main")
    branch = f"codex/{task['id']}"
    if run(["git", "show-ref", "--verify", f"refs/heads/{branch}"], repo, 30).returncode == 0:
        raise RuntimeError(f"分支已存在: {branch}")
    if worktree.exists():
        if any(worktree.iterdir()):
            raise RuntimeError(f"编码 worktree 已存在: {worktree}")
        worktree.rmdir()
    result = {"task_id": task["id"], "branch": branch, "started_at": started, "passed": False}
    try:
        added = run(["git", "worktree", "add", "-b", branch, str(worktree), base.stdout.strip()], repo, 60)
        if added.returncode != 0:
            raise RuntimeError(added.stderr[-2000:])
        checked = run(["git", "apply", "--check", str(patch_path)], worktree, 60)
        if checked.returncode != 0:
            raise RuntimeError("patch 校验失败: " + checked.stderr[-2000:])
        applied = run(["git", "apply", str(patch_path)], worktree, 60)
        if applied.returncode != 0:
            raise RuntimeError("patch 应用失败: " + applied.stderr[-2000:])
        command, env = verify_command(task["verification_profile"], worktree)
        tested = subprocess.run(command, cwd=worktree, env=env, text=True, capture_output=True, timeout=1800, check=False)
        result["verification"] = {"profile": task["verification_profile"], "exit_code": tested.returncode, "stdout": tested.stdout[-6000:], "stderr": tested.stderr[-3000:]}
        if tested.returncode != 0:
            raise RuntimeError("白名单验证失败")
        changed = run(["git", "status", "--porcelain"], worktree, 30).stdout.strip()
        if not changed:
            raise RuntimeError("任务没有产生实际代码变更，拒绝空提交")
        staged = run(["git", "add", "-A"], worktree, 30)
        if staged.returncode != 0:
            raise RuntimeError(staged.stderr[-2000:])
        committed = run(["git", "-c", "user.name=IRAF Coding Worker", "-c", "user.email=iraf-worker@localhost", "commit", "-m", task["title"].strip()], worktree, 120)
        if committed.returncode != 0:
            raise RuntimeError(committed.stderr[-2000:])
        pushed = run(["git", "push", "--set-upstream", "origin", branch], worktree, 180)
        if pushed.returncode != 0:
            raise RuntimeError("push 失败: " + pushed.stderr[-2000:])
        result.update({"passed": True, "commit": run(["git", "rev-parse", "HEAD"], worktree, 30).stdout.strip()})
    finally:
        if worktree.exists():
            run(["git", "worktree", "remove", "--force", str(worktree)], repo, 120)
        result["ended_at"] = stamp()
        evidence_root.mkdir(parents=True, exist_ok=True)
        (evidence_root / f"{task['id']}.json").write_text(json.dumps(result, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("/home/coretek/AIIRAF"))
    parser.add_argument("--queue", type=Path, default=Path("/etc/iraf/coding-queue/inbox"))
    parser.add_argument("--worktree", type=Path, default=Path("/home/coretek/AIIRAF-coding-worktree"))
    parser.add_argument("--evidence", type=Path, default=Path("/home/coretek/AIIRAF/build/coding-worker"))
    parser.add_argument("--poll-seconds", type=int, default=1800)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    args.queue.mkdir(parents=True, exist_ok=True)
    (args.queue.parent / "done").mkdir(parents=True, exist_ok=True)
    while True:
        tasks = sorted(args.queue.glob("*.json"))
        for task_path in tasks:
            try:
                result = process_task(args.repo, args.queue, args.worktree, task_path, args.evidence)
                print(json.dumps(result, ensure_ascii=True), flush=True)
                if result.get("passed"):
                    shutil.move(str(task_path), str(args.queue.parent / "done" / task_path.name))
            except Exception as exc:
                print(json.dumps({"task_id": task_path.stem, "passed": False, "reason": str(exc), "ended_at": stamp()}, ensure_ascii=True), flush=True)
        if args.once:
            return 0
        time.sleep(max(30, args.poll_seconds))


if __name__ == "__main__":
    raise SystemExit(main())
