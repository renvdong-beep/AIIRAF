#!/usr/bin/env bash
# 需求闸门（§11.87）隔离验证：owner 自由推进 vs guest 精确步数。
#
# 背景：joint 世界里 guest（臂）请求 `count` 步，而 owner 的**植物驻留线程**在等待期间并发
# mj_step ⇒ 实际推进 `count + 挂钟决定的超出量`（§11.56 实测 17×）；两拍之间 owner 也在推进
# ⇒ 臂的控制 dt 逐轮变化 ⇒ 步骤结果不可复现（A 站逐位可复现、B 站 1.8785~5.2865°）。
#
# 用法（仓库根）：bash scripts/probe_demand_gate.sh <on|off> [轮数=3] [标签=时间戳]
# 输出：build/diagnostics/dockhold-gate<on|off>-<标签>-round<N>.log
# 判读：python3 scripts/probe_dock_hold_yaw.py build/diagnostics/dockhold-gate<..>-<标签>-round*.log
#   · 闸门 off：A 站末态应与已知逐位一致（0.028569271967332333 / −0.42469473055728896）⇒ 证明"逐位不变"
#   · 闸门 on ：`PICK_STEP_ACCT` 的 requested 应等于 actual（超出量 0），且多轮之间 A/B 末态逐位一致
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:?用法: probe_demand_gate.sh <on|off> [轮数] [标签]}"
case "$MODE" in
  on)  GATE=1 ;;
  off) GATE=0 ;;
  *)   echo "第一个参数必须是 on 或 off，实际 ${MODE}" >&2; exit 2 ;;
esac
ROUNDS="${2:-3}"
TAG="${3:-$(date +%Y%m%d-%H%M%S)}"
OUTDIR="build/diagnostics"

for i in $(seq 1 "$ROUNDS"); do
  log="${OUTDIR}/dockhold-gate${MODE}-${TAG}-round${i}.log"
  echo "[闸门 ${MODE} 轮 ${i}/${ROUNDS}] -> ${log}"
  IRAF_DEBUG_DOCK=1 IRAF_DEBUG_GUEST_STEPS=1 IRAF_PLANT_DEMAND_GATE_OVERRIDE="$GATE" \
    PYTHONPATH=src python3 scripts/scenario.py run \
    --scene scenes/handoff_lab --scenario nominal --world joint --display none \
    > "$log" 2>&1 || true
  if grep -q "DOCK_HALT" "$log"; then
    echo "  DOCK_HALT ok；PICK_STEP_ACCT 行数=$(grep -c 'PICK_STEP_ACCT' "$log" || true)"
  else
    echo "  ⚠ 未捕获 DOCK_HALT（查看日志）"
  fi
done

echo "完成。判读：python3 scripts/probe_dock_hold_yaw.py ${OUTDIR}/dockhold-gate${MODE}-${TAG}-round*.log"
