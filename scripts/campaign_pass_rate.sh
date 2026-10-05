#!/usr/bin/env bash
# 连跑 N 轮统计**整链通过率**（2026-09-30）。不早停，逐轮记录关键步骤状态并汇总。
#
# 为什么（AGENTS/使用者纪律）：单轮通过不算验收；s06 的到位残差与 s02b 的停靠都是**间歇**的
# ⇒ 必须看通过率与最坏残差分布，而不是"跑到一次绿就宣布成功"。
#
# 用法（仓库根）：bash scripts/campaign_pass_rate.sh [轮数，缺省 5]
# 输出：build/diagnostics/passrate-round<N>.log；汇总打印在 stdout，并写 build/diagnostics/passrate.json
set -euo pipefail
cd "$(dirname "$0")/.."

ROUNDS="${1:-5}"
REPORT="build/acceptance/handoff_lab/nominal/report.json"
OUT="build/diagnostics/passrate.json"

for i in $(seq 1 "$ROUNDS"); do
  log="build/diagnostics/passrate-round${i}.log"
  echo "[轮 ${i}/${ROUNDS}] 开始 -> ${log}"
  PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab \
    --scenario nominal --world joint --display none > "$log" 2>&1 || true
  python3 - "$REPORT" "$i" "$OUT" <<'PY'
import json, sys, pathlib
report_path, index, out_path = pathlib.Path(sys.argv[1]), int(sys.argv[2]), pathlib.Path(sys.argv[3])
doc = json.load(open(report_path)) if report_path.exists() else {}
rows = {}
for step in doc.get("steps", []):
    measured = step.get("measured") or {}
    rows[step.get("id")] = {
        "status": step.get("status"),
        "residual_m": measured.get("grasp_center_distance_m"),
        "offset_m": measured.get("place_offset_from_tray_center_m"),
        "yaw_deg": measured.get("dock_yaw_error_deg"),
        "reason": (step.get("reason") or "")[:160],
    }
sink = json.loads(out_path.read_text()) if out_path.exists() else {"rounds": []}
sink["rounds"].append({"round": index, "passed": doc.get("passed"), "steps": rows})
out_path.write_text(json.dumps(sink, ensure_ascii=False, indent=2))
def show(key):
    row = rows.get(key) or {}
    extra = ""
    if row.get("residual_m") is not None:
        extra = " residual=%.9f" % row["residual_m"]
    elif row.get("offset_m") is not None:
        extra = " offset=%.9f" % row["offset_m"]
    elif row.get("yaw_deg") is not None:
        extra = " yaw=%.6f°" % row["yaw_deg"]
    print("  %-26s %-9s%s" % (key, row.get("status"), extra))
print("[轮 %d] passed=%s" % (index, doc.get("passed")))
for key in ("s01_verify_ready", "s02_dock", "s03_pick", "s04_place_in_tray",
            "s05_confirm_payload", "s02b_dock_station_b", "s05b_confirm_payload_at_b",
            "s06_unload_at_b", "s07_place_at_b_table"):
    show(key)
PY
done

python3 - "$OUT" <<'PY'
import json, pathlib, sys
sink = json.loads(pathlib.Path(sys.argv[1]).read_text())
rounds = sink["rounds"]
print("=" * 60)
print("[汇总] %d 轮" % len(rounds))
for key in ("s02b_dock_station_b", "s06_unload_at_b", "s07_place_at_b_table"):
    ok = sum(1 for item in rounds if (item["steps"].get(key) or {}).get("status") == "SUCCEEDED")
    print("  %-26s 通过 %d/%d" % (key, ok, len(rounds)))
print("  整链 passed=true      %d/%d" % (sum(1 for item in rounds if item.get("passed")), len(rounds)))
res = [item["steps"]["s06_unload_at_b"]["residual_m"] for item in rounds
       if (item["steps"].get("s06_unload_at_b") or {}).get("residual_m") is not None]
if res:
    print("  s06 残差: min=%.9f max=%.9f" % (min(res), max(res)))
off = [item["steps"]["s07_place_at_b_table"]["offset_m"] for item in rounds
       if (item["steps"].get("s07_place_at_b_table") or {}).get("offset_m") is not None]
if off:
    print("  s07 落点: min=%.9f max=%.9f（验收 0.065）" % (min(off), max(off)))
PY
