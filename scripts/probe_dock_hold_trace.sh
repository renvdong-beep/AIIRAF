#!/usr/bin/env bash
# 采集停靠**保持窗**偏航轨迹（2026-10-05 §11.86）。
#
# 为什么（纪律）：单轮通过不算验收，而 B 站停靠是**间歇**的（2/5）。要看的是
# 保持窗的偏航形状（单调漂移 / 等幅振荡 / 收敛），因此需要多轮样本。
#
# 用法（仓库根）：bash scripts/probe_dock_hold_trace.sh [轮数，缺省 2] [标签，缺省时间戳]
# 输出：build/diagnostics/dockhold-<标签>-round<N>.log（含 DOCK_TRACE，覆盖保持窗）
# ⚠ 文件名必须带标签：2026-10-05 实测踩过——不带标签会**覆盖**上一批（当时刚采完的 baseline 被
#    诊断批次盖掉，只能靠文档里的数复原）。标签缺省取时间戳 ⇒ 不同批次永不互相覆盖。
# 判读：python3 scripts/probe_dock_hold_yaw.py build/diagnostics/dockhold-<标签>-round*.log
set -euo pipefail
cd "$(dirname "$0")/.."

ROUNDS="${1:-2}"
TAG="${2:-$(date +%Y%m%d-%H%M%S)}"
OUTDIR="build/diagnostics"

for i in $(seq 1 "$ROUNDS"); do
  log="${OUTDIR}/dockhold-${TAG}-round${i}.log"
  echo "[轮 ${i}/${ROUNDS}] -> ${log}"
  # IRAF_DEBUG_DOCK=1 只开内存缓冲留证，不影响控制（逐拍通路零 print，段末一次性落盘）
  IRAF_DEBUG_DOCK=1 PYTHONPATH=src python3 scripts/scenario.py run \
    --scene scenes/handoff_lab --scenario nominal --world joint --display none \
    > "$log" 2>&1 || true
  if grep -q "DOCK_TRACE" "$log"; then
    echo "[轮 ${i}/${ROUNDS}] 已捕获 DOCK_TRACE（含保持窗）"
  else
    echo "[轮 ${i}/${ROUNDS}] 警告：未捕获 DOCK_TRACE（检查 IRAF_DEBUG_DOCK 是否生效）"
  fi
done

echo "完成。判读：python3 scripts/probe_dock_hold_yaw.py ${OUTDIR}/dockhold-${TAG}-round*.log"
