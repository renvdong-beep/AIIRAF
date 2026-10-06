#!/usr/bin/env bash
# 停靠偶发分叉的连跑采集 + 判读（2026-10-05 §11.88 量测⑩）。
#
# 用途：联合世界里 s02b 有一条约 17% 的**离散备选分支**（B 站末态 -2.10311916511876° vs 常态
# -2.38582468189734°）。步数、控制更新对齐、MPC 计数三层都已证明逐轮相同 ⇒ 只剩"控制量的**取值**"。
# 本脚本连跑 N 轮，采集 `DOCK_HALT.ctrl_align.seq`（每次 ctrl 写入的「控制量取值摘要」），
# 再用 `probe_ctrl_divergence.py` 定位**第一次分叉的那一拍**。
#
# 用法（仓库根）：bash scripts/campaign_dock_divergence.sh [轮数=24] [标签=时间戳]
set -euo pipefail
cd "$(dirname "$0")/.."

ROUNDS="${1:-24}"
TAG="${2:-$(date +%Y%m%d-%H%M%S)}"
OUTDIR="build/diagnostics"
# 每轮墙钟上限：正常一轮约 2 min；**必须带 timeout** —— 本仓踩过"静默停住把整批拖死"
# （2026-10-05：一个死锁的子进程活了 15 小时，只因当年 kill 了外层 wrapper 而没 kill 孩子）。
PER_ROUND_TIMEOUT_S="${PER_ROUND_TIMEOUT_S:-900}"

for i in $(seq 1 "$ROUNDS"); do
  log="${OUTDIR}/cval-${TAG}-round${i}.log"
  timeout "$PER_ROUND_TIMEOUT_S" env IRAF_DEBUG_DOCK=1 IRAF_DEBUG_CTRL_ALIGN=1 \
    PYTHONPATH=src python3 scripts/scenario.py run \
    --scene scenes/handoff_lab --scenario nominal --world joint --display none \
    > "$log" 2>&1 || true
  printf "[%2d/%d] exit=%s  %s  -> %s\n" "$i" "$ROUNDS" "$?" "$(date +%H:%M:%S)" "$log"
done

echo "=== 判读（常态/异常 与 第一次分叉拍）==="
python3 scripts/probe_ctrl_divergence.py "${OUTDIR}"/cval-"${TAG}"-round*.log

echo
echo "⚠ 运维提醒：本脚本用 timeout 兜底，但若某轮被 timeout 杀掉，"
echo "   请**按 PID**确认没有残留（不要用 pgrep 模式匹配 —— 本仓踩过："
echo "   pgrep -f 'scenario.py run' 会匹配到**发起清理的那条命令自己**，把 shell 一起杀掉）。"
