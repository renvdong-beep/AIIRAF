#!/usr/bin/env bash
# 等一个连跑批次跑完并自动判读（2026-10-06）。
#
# 为什么需要独立看住：批次的 shell 由后台会话发起，而**上下文压缩会重置进程登记表**
# ⇒ 看不到它（本仓踩过：误判"批次已死"，其实还在跑）。因此用"**日志停更**"当完成判据。
#
# ⚠ 不用 `pgrep -f "scenario.py run"`：本仓踩过——它匹配整条命令行，会命中**发起查看/清理
#   的那条命令自身**（于是要么自杀、要么永不退出）。
#
# 用法（仓库根）：bash scripts/watch_batch.sh <日志名前缀> <末轮号>
#   例： bash scripts/watch_batch.sh fresh-round 24
#        bash scripts/watch_batch.sh calls-origin1-round 30
set -euo pipefail
cd "$(dirname "$0")/.."

PREFIX="${1:?需要日志名前缀，例如 fresh-round}"
LAST="${2:?需要末轮号，例如 24}"
OUT="build/diagnostics"
LOG="${OUT}/${PREFIX}${LAST}.log"

while [ ! -f "$LOG" ]; do sleep 30; done          # 1) 等末轮日志出现
while :; do                                        # 2) 等它停止增长（连续 30 s 体积不变）
  a=$(stat -c%s "$LOG" 2>/dev/null || echo -1)
  sleep 30
  b=$(stat -c%s "$LOG" 2>/dev/null || echo -1)
  [ "$a" = "$b" ] && [ "$a" != "-1" ] && break
done
sleep 5

GLOB="${OUT}/${PREFIX}*.log"
echo "批次结束（${LOG} 已停更）"
echo "=== 判读①：逐步账（状态指纹 / 控制取值指纹 / MPC 新鲜度四元组）==="
python3 scripts/probe_span_divergence.py "${GLOB}" || true
echo
echo "=== 判读②：逐步 MPC 新鲜度决策计数 ==="
python3 scripts/probe_fresh_decisions.py --glob "${GLOB}" || true
echo
echo "=== 判读③：hold 调用起点表是否逐轮一致（判定 A/B）==="
python3 scripts/probe_ctrl_calls.py "${GLOB}" || true
