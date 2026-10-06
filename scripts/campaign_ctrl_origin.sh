#!/usr/bin/env bash
# 采集「hold 调用起点 + 控制量分量指纹」的连跑批次（2026-10-06 §11.88 量测⑳）。
#
# 目的：判定 s02b 离散分支的成因是
#   A —— 狗的 hold 调用边界未锚到植物步号（`q0`/`alpha` 相位不同）⇒ 修法＝按步号起调用；
#   B —— 植物状态先被外部改掉（窗口内狗没写 ctrl，只可能是另一台臂）⇒ 转查臂侧。
# 两者修法完全不同，故必须二选一而不是"顺手都改"。
#
# 采集项（全部是**内存累计 + 全 run 一行**，不开逐拍 I/O —— 本仓三次踩过观测者效应）：
#   IRAF_DEBUG_DOCK=1            ⇒ DOCK_HALT（判读脚本用它取 final_yaw_deg 分常态/备选）
#   IRAF_DEBUG_PLANT_SPAN=1      ⇒ PLANT_STEP_SPANS（逐步 `state_before`/`ctrl_digest`）
#   IRAF_DEBUG_CTRL_ALIGN=1      ⇒ `_ctrl_writes` 留证（逐拍 (步号, 取值摘要)）
#   IRAF_DEBUG_CTRL_DETAIL=s03_pick ⇒ 只带**目标步**的逐拍明细（避免整 run 明细膨胀）
#   IRAF_DEBUG_CTRL_CALLS=1      ⇒ PLANT_CTRL_CALLS（每次 `_run_control` 的起点步号/仿真钟/周期数/通路）
#   IRAF_DEBUG_CTRL_COMPONENTS=1 ⇒ 明细由 2 元组升为 5 元组（步号, ctrl, desired, q, dq）
#
# 用法（仓库根）：bash scripts/campaign_ctrl_origin.sh [轮数=30] [标签=时间戳]
set -euo pipefail
cd "$(dirname "$0")/.."

ROUNDS="${1:-30}"
TAG="${2:-$(date +%Y%m%d-%H%M%S)}"
OUTDIR="build/diagnostics"
# 每轮墙钟上限：正常一轮约 100 s；**必须带 timeout**（本仓踩过静默停住把整批拖死 15 小时）。
PER_ROUND_TIMEOUT_S="${PER_ROUND_TIMEOUT_S:-900}"

for i in $(seq 1 "$ROUNDS"); do
  log="${OUTDIR}/calls-${TAG}-round${i}.log"
  # ⚠ `rc` 必须显式取：原先写 `printf ... "$?"` 取到的是 `|| true` 的状态 ⇒ **每轮都假报 exit=0**，
  # 真实失败会被掩盖（2026-10-06 实测发现）。
  rc=0
  timeout "$PER_ROUND_TIMEOUT_S" env \
    IRAF_DEBUG_DOCK=1 IRAF_DEBUG_PLANT_SPAN=1 IRAF_DEBUG_CTRL_ALIGN=1 \
    IRAF_DEBUG_CTRL_DETAIL=s03_pick IRAF_DEBUG_CTRL_CALLS=1 IRAF_DEBUG_CTRL_COMPONENTS=1 \
    IRAF_DEBUG_CTRL_EXACT=1 IRAF_DEBUG_ARM_CTRL=1 \
    PYTHONPATH=src python3 scripts/scenario.py run \
    --scene scenes/handoff_lab --scenario nominal --world joint --display none \
    > "$log" 2>&1 || rc=$?
  printf "[%2d/%d] exit=%s  %s  -> %s\n" "$i" "$ROUNDS" "$rc" "$(date +%H:%M:%S)" "$log"
done

echo "=== 判读①：分叉进在「状态」还是「目标相位」（分量指纹）==="
python3 scripts/probe_span_divergence.py "${OUTDIR}"/calls-"${TAG}"-round*.log || true
echo
echo "=== 判读②：hold 调用起点表是否逐轮一致 ==="
python3 scripts/probe_ctrl_calls.py "${OUTDIR}"/calls-"${TAG}"-round*.log || true
echo
echo "=== 判读③：臂侧首个取值不同的拍（判定 B 的源头）==="
python3 scripts/probe_arm_divergence.py "${OUTDIR}"/calls-"${TAG}"-round*.log || true
