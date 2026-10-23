#!/usr/bin/env bash
# 盯 AgentOS 的 MCP 调用（用户手动操作时用）：一旦 agent 真的调用工具就落一行证据。
# 只读板上日志，不写板卡任何文件；输出到本地 build/logs/mcp-watch-<ts>.log。
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
OUT="${1:-/home/coretek/AIIRAF/build/logs/mcp-watch-$(date +%Y%m%d-%H%M%S).log}"
mkdir -p "$(dirname "$OUT")"
echo "baseline=$(sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$H" 'grep -c CallToolRequest /root/.intewell/logs/mcp-stderr.log 2>/dev/null || echo 0' | tr -d '\r')" | tee -a "$OUT"
echo "watching → $OUT"
LAST=0
for _ in $(seq 1 240); do   # 20 分钟上限
  NOW=$(sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$H" \
        'grep -c CallToolRequest /root/.intewell/logs/mcp-stderr.log 2>/dev/null || echo 0' 2>/dev/null | tr -d '\r')
  NOW="${NOW:-0}"
  if [ "$NOW" -gt "$LAST" ] 2>/dev/null; then
    echo "=== $(date +%H:%M:%S) CallToolRequest 累计=$NOW（+$((NOW-LAST))）===" | tee -a "$OUT"
    sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$H" \
      'grep -A3 "CallToolRequest" /root/.intewell/logs/mcp-stderr.log | tail -16' 2>/dev/null | tee -a "$OUT"
    # 顺带把最新一次执行事实捞出来（事件库，权威）
    sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$H" \
      'sqlite3 /var/lib/iraf/iraf-runtime.db "select execution_id,status,skill from executions order by rowid desc limit 2;" 2>/dev/null || echo "(无 sqlite3 或无表)"' \
      2>/dev/null | tee -a "$OUT"
    LAST="$NOW"
  fi
  sleep 5
done
echo "watch 结束（20 分钟）最终累计=$LAST" | tee -a "$OUT"
