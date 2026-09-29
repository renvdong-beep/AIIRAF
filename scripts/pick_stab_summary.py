#!/usr/bin/env python3
"""汇总 `build/pick_stab_*.log`（抓取侧稳定性量化）：逐轮抽出 pick/regrasp 关键量并给通过率。

用法：python3 build/pick_stab_summary.py [日志前缀数]（默认 8）
"""

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ("grasped", "lifted", "lift_delta_m", "force_ok", "bilateral_contact",
          "left_normal_force_n", "right_normal_force_n")
REGRASP_FIELDS = ("pre_lift_delta_m", "pre_lift_declared_m", "pad_minus_payload_z_m",
                  "payload_pad_lateral_m", "bilateral_after_regrasp", "force_ok_after_regrasp")


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    rows = []
    for index in range(1, count + 1):
        path = ROOT / ("build/pick_stab_%d.log" % index)
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        row = {"run": index}
        # ⚠ 判"整轮通过"**只能看顶层**：报告里每个步骤都有 `"passed": true/false`，
        # 用 `"passed": true` 会抓错（本轮实测：run 3 步骤 s03 失败却仍被判 "passed=True"）。
        # 顶层判据：`"failed_checks": []`（空数组 = 无失败项）。
        # ⚠ 三态而不是两态（2026-09-29 实测教训）：日志里既没有 `"failed_checks": []` 也没有失败项，
        # 说明**这一轮的日志被逐样本追踪打印淹没/截断**（不是物理失败）⇒ 记 None，绝不当作失败。
        # 权威判据是**进程退出码**（run 8 实测 exit=0 = 通过，却被旧脚本算成失败 ⇒ 通过率被低估）。
        if re.search(r'"failed_checks": \[\]', text):
            row["passed"] = True
        elif re.search(r'"failed_checks": \[\n', text):
            row["passed"] = False
        else:
            row["passed"] = None
        head = re.search(r"PICK/REGRASP 摘要", text)
        row["summary"] = bool(head)
        m = re.search(r"grasped=(\S+) confirmation=(\S+) lifted=(\S+) lift_delta_m=(\S+)", text)
        if m:
            row["grasped"], row["lifted"], row["lift_delta_m"] = m.group(1), m.group(3), m.group(4)
        m = re.search(r"双侧接触=(\S+) 左力=(\S+) 右力=(\S+) 不平衡比=(\S+) force_ok=(\S+)", text)
        if m:
            row["bilateral"], row["left_force"], row["right_force"], row["force_ok"] = (
                m.group(1), m.group(2), m.group(3), m.group(5))
        for name in REGRASP_FIELDS:
            found = re.search(r'"%s": ([^,\n]+)' % name, text)
            row[name] = found.group(1).strip().strip('"') if found else None
        fail = re.search(r'"failed_checks": \[\n\s+"([^"]{0,160})', text)
        row["failed_check"] = fail.group(1) if fail else ""
        rows.append(row)

    print("run  passed  grasped lifted lift_delta  force_ok  双侧|左/右力            pre_lift_delta/声明  pad−payload_z")
    for row in rows:
        print("%-4s %-7s %-7s %-6s %-10s %-9s %-20s %-20s %s" % (
            row["run"], row.get("passed"), row.get("grasped"), row.get("lifted"),
            row.get("lift_delta_m"), row.get("force_ok"),
            "%s|%s/%s" % (row.get("bilateral"), row.get("left_force"), row.get("right_force")),
            "%s/%s" % (row.get("pre_lift_delta_m"), row.get("pre_lift_declared_m")),
            row.get("pad_minus_payload_z_m")))
        if not row.get("passed"):
            print("       FAIL: %s" % (row.get("failed_check") or "(未捕获)"))
    ok = [row for row in rows if row.get("passed") is True]
    failed = [row for row in rows if row.get("passed") is False]
    unknown = [row for row in rows if row.get("passed") is None]
    print("\n通过 %d/%d（失败 %d，日志不完整 %d —— 后者以进程退出码为准）"
          % (len(ok), len(rows), len(failed), len(unknown)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
