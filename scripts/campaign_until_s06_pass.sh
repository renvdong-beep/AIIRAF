#!/usr/bin/env bash
# 连跑到 **s06 通过**（为 §11.72/§11.73 的 above 段取证准备数据）。2026-09-30
#
# 为什么需要：`s07` 的放置段只在 s06 成功后才执行；而 s06 的到位残差是**间歇**的
# （0.00068 ~ 0.0073 m，判据 0.005）⇒ 必须连跑到一轮通过，才拿得到 `above` 段的逐拍数据。
#
# 用法（仓库根）：bash scripts/campaign_until_s06_pass.sh [最多轮数]
# 输出：build/diagnostics/above-campaign-round<N>.log；命中即停并打印日志名。
set -euo pipefail
cd "$(dirname "$0")/.."

MAX="${1:-4}"
REPORT="build/acceptance/handoff_lab/nominal/report.json"

for i in $(seq 1 "$MAX"); do
  log="build/diagnostics/above-campaign-round${i}.log"
  echo "[轮 ${i}/${MAX}] 开始 -> ${log}"
  IRAF_DEBUG_PLACE=1 IRAF_DEBUG_TRACE_CMD=1 PYTHONPATH=src \
    python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario nominal \
      --world joint --display none > "$log" 2>&1 || true
  status="$(python3 - "$REPORT" <<'PY'
import json, sys
doc = json.load(open(sys.argv[1]))
print(next((s.get("status", "") for s in doc.get("steps", []) if s.get("id") == "s06_unload_at_b"), "MISSING"))
PY
)"
  s07="$(python3 - "$REPORT" <<'PY'
import json, sys
doc = json.load(open(sys.argv[1]))
print(next((s.get("status", "") for s in doc.get("steps", []) if s.get("id") == "s07_place_at_b_table"), "MISSING"))
PY
)"
  echo "[轮 ${i}] s06=${status} s07=${s07}"
  if [ "$status" = "SUCCEEDED" ]; then
    echo "[命中] s06 通过（s07=${s07}）⇒ 逐拍数据在 ${log}"
    exit 0
  fi
done

echo "[结束] ${MAX} 轮内 s06 未通过；按需加大轮数"
exit 0
