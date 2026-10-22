#!/usr/bin/env bash
# 只读探针：AgentOS / dashboard / 容器 / 配置位置（.147 智能域）
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$H" "$1" 2>&1; }

echo "=== 1. docker ps ==="
run 'docker ps --format "{{.Names}}|{{.Image}}|{{.Status}}" 2>&1 | head -12'

echo "=== 2. AgentOS 进程 ==="
run 'ps -eo pid,cmd | grep -Ei "agentos|intewell-agent" | grep -v grep | head -12'

echo "=== 3. 监听端口 ==="
run 'ss -lntp 2>/dev/null | grep -E ":9119|:8765|:50051|:808" | head -10'

echo "=== 4. 配置候选文件 ==="
run 'ls -la /root/.intewell/ 2>&1 | head -10; echo ---; ls -la /opt/agentos 2>&1 | head -10; echo ---; ls /root/*.py 2>&1 | head'

echo "=== 5. 容器内 home ==="
run 'docker exec agentos-worker-1 sh -lc "ls -la /home/agentos/.intewell/ 2>&1 | head; echo ---; ls /home/agentos/.intewell/skills 2>&1" 2>&1 | head -25'

echo "=== 6. dashboard 存活 ==="
run 'curl -s -o /dev/null -w "%{http_code}\n" --max-time 5 http://127.0.0.1:9119/ 2>&1'
