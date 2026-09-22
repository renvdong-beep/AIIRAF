"""Go2 `damped_hold` 验收（真实场景入口）：`--config` + `--profile` + `--safety` → 报告 + 退出码。

与 `verify_go2_loopback.py` 同构（薄 CLI：解析参数 → 调库/适配器 → 摘要 → 退出码），
但被测对象是**适配器的 `stop(mode="damped_hold")`**（真实声明 + 真实模型 + 真实安全策略），
不是 loopback 的独立控制循环。**不修改既有 loopback 任何代码。**

判据（全部来自声明，不在本脚本写数字）：
  · `stop` 段：`duration_s`（超时预算）/ `speed_tolerance_mps` / `static_hold_s` / `final_window_s`
  · 安全策略：`quadruped_limits.max_tilt_moving_deg`（**唯一事实来源**，经 `load_movement_limits` 读入后传给适配器）

两档：
  · 正路径：期望 `succeeded: true`、末速 ≤ 容差、连续 `static_hold_s` 静止、终态 `STOPPED`
  · 超时档：把 `stop.static_hold_s` 调到大于总时长（写临时声明副本）⇒ 期望终态 `FAILED` 且带实测量

退出码：0 全部通过 ｜ 1 用法错误 ｜ 2 声明非法 ｜ 3 引用缺失 ｜ 5 判据未过。
诚实边界：全部结论属**仿真**（报告 `simulation: true`）。
"""

import argparse
import datetime
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import yaml  # noqa: E402

from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.profile import load_robot_profile as load_profile  # noqa: E402
from iraf_adapters.unitree.unitree_go2 import UnitreeGo2Adapter  # noqa: E402
from iraf_skills.quadruped import load_movement_limits  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_DECLARATION, EXIT_REFERENCE, EXIT_CHECKS = 0, 1, 2, 3, 5


def _run_case(declaration_path, profile, safety_path, label):
    limits = load_movement_limits(safety_path)
    tilt_limit = float(limits["max_tilt_moving_deg"])
    authority = ControlAuthorityManager()
    lease = authority.acquire("robot:unitree_go2", "verify-damped-hold", ttl_seconds=120.0)
    adapter = UnitreeGo2Adapter.from_config(declaration_path, profile, authority)
    report = adapter.stop(lease, mode="damped_hold", tilt_limit_deg=tilt_limit)
    checks = []

    def add(name, value, expectation, passed):
        checks.append({"name": name, "value": value, "expectation": expectation,
                       "passed": bool(passed)})

    if label == "positive":
        add("succeeded", report["succeeded"], "== true", report["succeeded"] is True)
        add("final_speed_mps", report["final_speed_mps"],
            "<= %.3f" % report["speed_tolerance_mps"],
            report["final_speed_mps"] <= report["speed_tolerance_mps"])
        add("max_tilt_deg", report["max_tilt_deg"], "<= %.1f" % tilt_limit,
            report["max_tilt_deg"] <= tilt_limit)
        add("stop_mode", report["stop_mode"], "== damped_hold", report["stop_mode"] == "damped_hold")
        add("tilt_limit_source", report["tilt_limit_source"], "== caller",
            report["tilt_limit_source"] == "caller")
    else:
        add("succeeded", report["succeeded"], "== false", report["succeeded"] is False)
        add("failure_reason_nonempty", bool(report["failure_reason"]), "非空",
            bool(report["failure_reason"]))

    terminal = None
    for entry in adapter.ledger.snapshot():
        if str(entry.get("execution_id")) == str(report["execution_id"]):
            terminal = str(entry.get("state"))
    expected_terminal = "STOPPED" if label == "positive" else "FAILED"
    add("terminal_state", terminal, "== %s" % expected_terminal, terminal == expected_terminal)
    return {"label": label, "report": report, "checks": checks,
            "failed_checks": [c["name"] for c in checks if not c["passed"]]}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2 damped_hold 验收（真实场景，仿真）")
    parser.add_argument("--config", type=Path, default=Path("config/go2_loopback.yaml"))
    parser.add_argument("--profile", type=Path, default=Path("profiles/unitree_go2_mujoco.yaml"))
    parser.add_argument("--safety", type=Path,
                        default=Path("profiles/safety/quadruped_lab.yaml"))
    parser.add_argument("--report", type=Path,
                        default=Path("build/acceptance/go2-damped-hold/report.json"))
    parser.add_argument("--root", type=Path, default=None)
    args = parser.parse_args(argv)

    root = args.root or Path(__file__).resolve().parents[1]
    for path, label in ((args.config, "config"), (args.profile, "profile"), (args.safety, "safety")):
        if not (root / path).is_file():
            print("用法错误：%s 文件不存在: %s" % (label, root / path), file=sys.stderr)
            return EXIT_USAGE

    try:
        profile = load_profile(root / args.profile)
    except Exception as exc:  # noqa: BLE001
        print("Profile 非法（退出码 %d）：%s" % (EXIT_DECLARATION, exc), file=sys.stderr)
        return EXIT_DECLARATION

    cases = []
    with tempfile.TemporaryDirectory() as tmp:
        # 正路径：真实声明
        cases.append(_run_case(root / args.config, profile, root / args.safety, "positive"))
        # 超时档：把 static_hold_s 调到大于总时长（临时副本，不改真实声明）
        raw = yaml.safe_load((root / args.config).read_text(encoding="utf-8"))
        raw.setdefault("stop", {})
        raw["stop"]["static_hold_s"] = max(999.0, float(raw["stop"].get("static_hold_s", 0.5)))
        variant = Path(tmp) / "damped_hold_timeout.yaml"
        variant.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
        cases.append(_run_case(variant, profile, root / args.safety, "timeout"))

    failed = [c["label"] + ":" + name for c in cases for name in c["failed_checks"]]
    report = {
        "schema_version": "iraf.acceptance.damped_hold/v1",
        "kind": "acceptance_report",
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "simulation": True,
        "config": str(args.config),
        "profile": str(args.profile),
        "safety_policy": str(args.safety),
        "cases": [{c["label"]: c["report"]} for c in cases],
        "checks": sum(len(c["checks"]) for c in cases),
        "failed_checks": failed,
        "passed": not failed,
    }
    out = root / args.report
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    summary = {
        "passed": report["passed"],
        "exit_code": EXIT_OK if report["passed"] else EXIT_CHECKS,
        "report_path": str(out),
        "simulation": True,
        "checks": report["checks"],
        "failed_checks": failed,
        "positive": {k: cases[0]["report"][k] for k in
                     ("succeeded", "final_speed_mps", "max_tilt_deg", "stop_mode", "failure_reason")},
        "timeout": {k: cases[1]["report"][k] for k in
                    ("succeeded", "final_speed_mps", "failure_reason")},
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return EXIT_OK if report["passed"] else EXIT_CHECKS


if __name__ == "__main__":
    raise SystemExit(main())
