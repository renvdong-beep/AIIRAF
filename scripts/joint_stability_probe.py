#!/usr/bin/env python3
"""联合世界**稳定性探针**：同一份产物连跑 N 次 nominal，统计通过率与失败模式。

为什么必须（2026-09-29 实测）：联合链路的失败是**偶发**的，单次跑绿不能证明"已交付"。
本次实测遇到两种不同失败模式：
  · 已知：s05 `max_accept_speed_mps` 超限（载荷放下后仍在爬行，settle 窗口不够）；
  · 新见：**载荷根本没进托盘**（s04 `Backend 未确认载荷已放下`，s05 报载荷最低点 ≈ 0 m ⇒ 掉到台面）。
两者都在**同一份 piper 模型**上出现（该模型与 HEAD 版构建器产物逐字节一致，sha256
e04e708cb7697ced4caadf0ecf9f54c17a90b90171530b5848d47c5a9db45085）⇒ 不是构建器回归，
而是运行期/物理的偶发项 ⇒ 只能靠**连跑统计**说话。

口径：
  · 每次运行的"通过"以 **scenario.py 进程退出码**为权威（0 = passed），报告文件只作证据来源；
  · 每次运行结束**立刻**取走报告（下一次运行会覆盖同一路径）；
  · 失败模式按"失败步骤 id + 首个失败判据名"归类计数；
  · 数值范围按达成运行的实测列出（不四舍五入成"大约"）。

用法：
  PYTHONPATH=src python3 scripts/joint_stability_probe.py --runs 5
  → build/joint-stability.json + build/joint-stability/run_<i>.log，退出码 0/1（是否全过）
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_REPORT = REPO / "build/acceptance/handoff_lab/nominal/report.json"


def _run_once(index, scene, scenario, world, workdir, log_dir):
    """跑一次场景；返回 (exit_code, report_or_None, log_path)。"""
    log_path = log_dir / ("run_%d.log" % index)
    env = os.environ.copy()
    env["PYTHONPATH"] = "src"
    with log_path.open("w", encoding="utf-8") as handle:
        proc = subprocess.run(
            [sys.executable, "scripts/scenario.py", "run", "--scene", scene,
             "--scenario", scenario, "--world", world],
            cwd=str(workdir), env=env, stdout=handle, stderr=subprocess.STDOUT,
        )
    report = None
    if DEFAULT_REPORT.is_file():
        try:
            report = json.loads(DEFAULT_REPORT.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            report = None
    return proc.returncode, report, log_path


def _failure_mode(report):
    """把一次失败归类成 (步骤 id, 归类键)；无失败返回 (None, None)。

    ⚠ 归类键的口径（2026-09-29 实测踩点）：**步骤级 FAILED 必须用 `reason`**，不能拿它
    "第一个未通过的判据名" —— 步骤级失败时所有判据都是未测量（`measured=null`、`passed=false`），
    于是会稳定地取到**列表里第一个判据名**，把"横向纠偏守卫拦下"这类真实原因伪装成
    `require_release`（本轮据此误判过一次"触地纠偏导致释放失败"，见 §11.25(f-6)/(f-7)）。
    判据级失败（步骤 SUCCEEDED 但某判据不达标）才用判据名。
    """
    failed = report.get("failed_checks") or []
    if not failed:
        return None, None
    for step in (report.get("steps") or []):
        reason = str(step.get("reason") or "").strip()
        if step.get("status") != "SUCCEEDED":
            if reason:
                return str(step.get("id")), "reason:" + reason[:60]
            for check in (step.get("checks") or []):
                if not check.get("passed", True):
                    return str(step.get("id")), str(check.get("name"))
            return str(step.get("id")), None
        for check in (step.get("checks") or []):
            if not check.get("passed", True):
                return str(step.get("id")), str(check.get("name"))
    return "<report>", None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--scene", default="scenes/handoff_lab")
    parser.add_argument("--scenario", default="nominal")
    parser.add_argument("--world", default="joint")
    parser.add_argument("--output", default="build/joint-stability.json")
    args = parser.parse_args()

    log_dir = REPO / "build/joint-stability"
    log_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for index in range(1, args.runs + 1):
        code, report, log_path = _run_once(index, args.scene, args.scenario, args.world,
                                          REPO, log_dir)
        passed = (code == 0)
        mode = _failure_mode(report) if report else ("<no-report>", None)
        steps = {}
        for step in (report.get("steps") or []) if report else []:
            steps[str(step.get("id"))] = {
                "status": str(step.get("status")),
                "checks": {str(c.get("name")): c.get("measured")
                           for c in (step.get("checks") or [])},
                "reason": str(step.get("reason") or "")[:200],
            }
        records.append({"run": index, "exit_code": code, "passed": passed,
                        "failed_step": mode[0], "failed_criterion": mode[1],
                        "failed_checks": (report.get("failed_checks") or []) if report else [],
                        "steps": steps, "log": str(log_path.relative_to(REPO))})
        print("run %d/%d: exit=%s passed=%s%s" % (
            index, args.runs, code, passed,
            "" if passed else " | %s / %s" % mode))
        sys.stdout.flush()

    passed_count = sum(1 for item in records if item["passed"])
    modes = {}
    for item in records:
        if not item["passed"]:
            key = "%s::%s" % (item["failed_step"], item["failed_criterion"])
            modes[key] = modes.get(key, 0) + 1
    numeric = {}
    for item in records:
        if not item["passed"]:
            continue
        for step_id, step in item["steps"].items():
            for name, value in step["checks"].items():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    numeric.setdefault("%s::%s" % (step_id, name), []).append(value)
    summary = {
        "runs": args.runs,
        "passed": passed_count,
        "pass_rate": round(passed_count / args.runs, 4),
        "failure_modes": modes,
        "numeric_ranges": {key: {"min": min(values), "max": max(values), "n": len(values)}
                           for key, values in sorted(numeric.items())},
        "records": records,
        "note": ("通过口径 = scenario.py 进程退出码；报告文件仅作证据来源。"
                 "失败模式按（失败步骤 id::首个未通过判据名）归类。"),
    }
    (REPO / args.output).write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n",
                                    encoding="utf-8")
    print("通过 %d/%d = %.1f%%；失败模式 %s" % (passed_count, args.runs,
                                           100.0 * passed_count / args.runs, modes or "无"))
    print("证据: %s" % args.output)
    return 0 if passed_count == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
