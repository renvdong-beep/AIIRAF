#!/usr/bin/env bash
# 只读校验：重启后的 dashboard 是否真拿到 key，且配置 60 秒内未被改写
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. dashboard 进程 env（key 只报长度/头）==="
run 'PID=$(ss -lntp 2>/dev/null | awk "/:9119 /{print \$NF}" | sed "s/.*pid=\([0-9]*\).*/\1/" | head -1); echo "  pid=$PID started=$(ps -o lstart= -p $PID)"; echo "  cwd=$(readlink -f /proc/$PID/cwd)"; tr "\0" "\n" < /proc/$PID/environ | grep -E "^(CUSTOM_API_KEY|OPENAI_BASE_URL|OPENAI_API_KEY|INTEWELL_AGENT_HOME)=" | while IFS= read -r l; do k=${l%%=*}; v=${l#*=}; case "$k" in *KEY) echo "  $k len=${#v} head=$(printf %s "$v" | cut -c1-6)";; *) echo "  $k=$v";; esac; done'

echo "=== 2. 配置等 60 秒后是否仍是 model+key（防被重写）==="
run 'sleep 60; echo "  config mtime=$(stat -c %y /root/.intewell/config.yaml)"; grep -cE "^(model|mcp_discovery_timeout):" /root/.intewell/config.yaml; echo "  .env=$(ls -l /root/.intewell/.env 2>&1 | awk "{print \$5\" bytes\"}")"'

echo "=== 3. 用 dashboard 同环境跑一次 CLI（同二进制、同 key）==="
run 'cd /opt/iraf-dev-workspace && KEY=$(cat /etc/iraf/agentos.key) && INTEWELL_AGENT_HOME=/opt/IntewellAgent OPENAI_BASE_URL=http://192.168.100.89:3000/v1 OPENAI_API_KEY="$KEY" CUSTOM_API_KEY="$KEY" timeout 110 python3 /usr/local/bin/intewell-agent-cli -z "只回两个字：收到" 2>&1 | tail -6'

echo "=== 4. 新增的 worker 容器侧 key（d20588cd 镜像）==="
run 'docker exec agentos-worker-1 sh -lc "echo len=\${#CUSTOM_API_KEY} head=\$(printf %s \"\$CUSTOM_API_KEY\" | cut -c1-6) url=\$CUSTOM_API_URL model=\$AGENTOS_MODEL"'
