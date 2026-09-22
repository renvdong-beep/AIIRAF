"""② 速度跟踪判据的**复算脚本**（可复跑，铁律 5.2）：从既有序列复算均值/P95/max。

为什么单独写脚本：② 的判定必须能由**任何人用同一份原始序列复算**，不能靠一次运行的口头结论。
判据口径（已定，见 .hermes/plans/2026-09-23-continuous-execution-queue.md 的 A3）：
  · **硬判据**：稳态窗口（`t ≥ --window-start-s`，默认 2.5 s）内，相对指令速度的
    **误差均值** ≤ `--max-mean-rel-error`（默认 0.10）；
  · **仅记录**：`|误差|` 均值、P95、max（不设阈，等工况足够再定）。

只读 `build/research/mpc-repo/eval_trot_23-*.json`（证据区），不产生新仿真。

用法::

    PYTHONPATH=src python3 scripts/recompute_locomote_tracking.py --tag vendor-mu0.4-vx0.50
    PYTHONPATH=src python3 scripts/recompute_locomote_tracking.py --all
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

DEFAULT_DIR = Path("build/research/mpc-repo")


def _pct(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100.0 * (len(ordered) - 1))))]


def compute(series, window_start_s=2.5):
    win = [r for r in series if float(r["t"]) >= float(window_start_s)]
    if not win:
        return None
    errs = [float(r["vx"]) - float(r["vx_cmd"]) for r in win]
    cmd = float(win[-1]["vx_cmd"])
    abs_errs = [abs(e) for e in errs]
    mean_err = sum(errs) / len(errs)
    return {
        "samples": len(win),
        "window_start_s": float(window_start_s),
        "vx_cmd": cmd,
        "vx_mean": sum(float(r["vx"]) for r in win) / len(win),
        "err_mean": mean_err,
        "mean_rel_error": (mean_err / cmd) if cmd != 0.0 else None,
        "abs_err_mean": sum(abs_errs) / len(abs_errs),
        "abs_err_p95": _pct(abs_errs, 95),
        "abs_err_max": max(abs_errs),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="复算 ② 速度跟踪判据（只读既有序列）")
    parser.add_argument("--dir", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--tag", default=None, help="单个 tag（对应 eval_trot_23-<tag>.json）")
    parser.add_argument("--all", action="store_true", help="复算目录下全部序列")
    parser.add_argument("--window-start-s", type=float, default=2.5)
    parser.add_argument("--max-mean-rel-error", type=float, default=0.10)
    args = parser.parse_args(argv)

    if args.tag:
        paths = [args.dir / ("eval_trot_23-%s.json" % args.tag)]
    elif args.all:
        paths = sorted(args.dir.glob("eval_trot_23-*.json"))
    else:
        print("用法错误：需要 --tag 或 --all", file=sys.stderr)
        return 1

    rows = []
    for path in paths:
        if not path.is_file():
            print("跳过（不存在）：%s" % path, file=sys.stderr)
            continue
        payload = json.loads(path.read_text(encoding="utf-8"))
        stats = compute(payload.get("series", []), args.window_start_s)
        if stats is None:
            print("跳过（窗口为空）：%s" % path, file=sys.stderr)
            continue
        # 口径：本判据只适用于**直行/倒退**（指令 vx ≠ 0）。转弯（yaw）与侧行（vy）工况的
        # vx 指令为 0 ⇒ 该判据**不适用**（记 NA），不得判成"不通过"（首版即犯此错，已更正）。
        applicable = stats["vx_cmd"] != 0.0
        passed = (applicable and stats["mean_rel_error"] is not None
                  and abs(stats["mean_rel_error"]) <= args.max_mean_rel_error)
        rows.append({"file": path.name, **stats, "applicable": bool(applicable),
                     "passed_hard_criterion": bool(passed)})
        verdict = ("硬判据通过" if passed else "硬判据不通过") if applicable else "不适用（vx 指令为 0）"
        rel = ("%+.2f%%" % (100.0 * stats["mean_rel_error"])) if applicable else "n/a"
        print("%-34s | 窗口 %4d 样本 ｜ 指令 %+.3f ｜ 实测 %+.4f ｜ 误差均值 %+.4f（%s）"
              " ｜ |误差| 均值 %.4f ｜ P95 %.4f ｜ max %.4f ⇒ %s"
              % (path.name, stats["samples"], stats["vx_cmd"], stats["vx_mean"],
                 stats["err_mean"], rel,
                 stats["abs_err_mean"], stats["abs_err_p95"], stats["abs_err_max"], verdict))
    if not rows:
        print("没有任何序列被复算", file=sys.stderr)
        return 1
    applicable = [r for r in rows if r["applicable"]]
    print("\n硬判据（直行/倒退，误差均值 ≤ %.0f%%）：%d/%d 适用工况通过 ｜ 不适用（vx 指令为 0）%d 条"
          " ｜ P95/max 仅记录（不设阈）"
          % (100.0 * args.max_mean_rel_error,
             sum(1 for r in applicable if r["passed_hard_criterion"]), len(applicable),
             len(rows) - len(applicable)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
