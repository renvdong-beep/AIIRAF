#!/usr/bin/env bash
# 只读：谁把 config.yaml 改回去了 + 当前 dashboard 进程与启动方式
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. config.yaml 现状（时间/备份/内容尾）==="
run 'ls -la --time-style=+%F_%T /root/.intewell/config.yaml*; echo ---; grep -nE "^(model|dashboard|_config_version|mcp_)" /root/.intewell/config.yaml; echo ---; date'

echo "=== 2. 当前 dashboard 进程 ==="
run 'ss -lntp 2>/dev/null | grep ":9119 "; PID=$(ss -lntp 2>/dev/null | awk "/:9119 /{print \$NF}" | sed "s/.*pid=\([0-9]*\).*/\1/" | head -1); echo "pid=$PID"; ps -o pid,lstart,cmd -p $PID; echo "cwd=$(readlink -f /proc/$PID/cwd)"'

echo "=== 3. 该进程 env（全部键名）==="
run 'PID=$(ss -lntp 2>/dev/null | awk "/:9119 /{print \$NF}" | sed "s/.*pid=\([0-9]*\).*/\1/" | head -1); tr "\0" "\n" < /proc/$PID/environ | cut -d= -f1 | sort | tr "\n" " "; echo'

echo "=== 4. 谁在重启/守护 dashboard ==="
run 'ps -eo pid,cmd | grep -E "restart_dashboard|watchdog|supervis" | grep -v grep | head; echo ---; ls -la --time-style=+%F_%T /root/.intewell/logs/gui.log; grep -c "" /root/.intewell/logs/gui.log'

echo "=== 5. .env 是否还在 ==="
run 'ls -la --time-style=+%F_%T /root/.intewell/.env 2>&1'
