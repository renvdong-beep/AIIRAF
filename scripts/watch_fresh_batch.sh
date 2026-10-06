#!/usr/bin/env bash
# 等 fresh 批次跑完并自动判读（2026-10-06 §11.88 量测⑲）。
#
# 为什么要独立看住：批次的 shell 由 Hermes 后台会话发起，但**上下文压缩会重置进程登记表**
# ⇒ `process list` 看不到它（本轮已踩：误判"批次已死"）。因此用"日志停更"作完成判据。
#
# ⚠ 不用 `pgrep -f "scenario.py run"`：本仓踩过——它会匹配到**发起清理/发起查看的那条命令自己**
#   （`pgrep -f` 匹配整条命令行，检查脚本自己的命令行里就含该字符串）。
#
# 用法（仓库根）：bash scripts/watch_fresh_batch.sh [末轮号=24]
set -euo pipefail
cd "$(dirname "$0")/.."

LAST="${1:-24}"
OUT="build/diagnostics"
LOG="${OUT}/fresh-round${LAST}.log"

while [ ! -f "$LOG" ]; do sleep 30; done          # 1) 等末轮日志出现
while :; do                                        # 2) 等它停止增长（连续 30 s 体积不变）
  a=$(stat -c%s "$LOG")
  sleep 30
  b=$(stat -c%s "$LOG")
  [ "$a" = "$b" ] && break
done
sleep 5

echo "fresh 批次结束（${LOG} 已停更）"
echo "=== 判读：逐步 MPC 新鲜度决策计数 ==="
python3 scripts/probe_fresh_decisions.py --glob "${OUT}/fresh-round*.log" || true
echo
echo "=== 判读：分叉进在「状态」还是「控制取值」 ==="
python3 scripts/probe_span_divergence.py "${OUT}"/fresh-round*.log || true
