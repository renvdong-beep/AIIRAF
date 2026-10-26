#!/usr/bin/env bash
# 只读：新 AgentOS 部署（bundle d20588cd）的现状：容器、deploy/.env、key 位置
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. 现存 bundle 目录 ==="
run 'ls -la --time-style=+%F_%T /root/IntewellAgent/artifacts/arm64/ | tail -6'

echo "=== 2. 容器（镜像/启动时间）==="
run 'docker ps --format "{{.Names}}|{{.Image}}|{{.Status}}" | head -10'

echo "=== 3. 新 bundle 的 deploy/.env（key 行长/头）==="
run 'E=/root/IntewellAgent/artifacts/arm64/agentos-bundle-d20588cd-arm64/deploy/.env; ls -l --time-style=+%F_%T $E 2>&1; grep -nE "^CUSTOM_API_(KEY|URL|PROTOCOL)=" $E 2>/dev/null | while IFS= read -r l; do k=${l%%=*}; v=${l#*=}; case "$k" in *KEY) echo "  $k len=${#v} head=$(printf %s "$v" | cut -c1-6)";; *) echo "  $k=$v";; esac; done'

echo "=== 4. worker 容器 env 里的 key ==="
run 'docker inspect agentos-worker-1 --format "{{range .Config.Env}}{{println .}}{{end}}" 2>/dev/null | grep -E "^(CUSTOM_API|AGENTOS_MODEL)" | while IFS= read -r l; do k=${l%%=*}; v=${l#*=}; case "$k" in *KEY) echo "  $k len=${#v} head=$(printf %s "$v" | cut -c1-6)";; *) echo "  $k=$v";; esac; done'

echo "=== 5. /etc/iraf/agentos.key 是否仍在且有效 ==="
run 'v=$(cat /etc/iraf/agentos.key 2>/dev/null); echo "  len=${#v} head=$(printf %s "$v" | cut -c1-6)"; c=$(curl -s -o /dev/null -w "%{http_code}" --max-time 8 -H "Authorization: Bearer $v" http://192.168.100.89:3000/v1/models); echo "  -> 网关 http=$c"'
