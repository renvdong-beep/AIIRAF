#!/usr/bin/env bash
# 只读验证：dashboard 的 /api/model/info + 与 pty 同环境的 CLI 直连模型
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. /api/model/info（公开路由，不需登录）==="
run 'curl -s --max-time 8 http://127.0.0.1:9119/api/model/info | head -c 600; echo'

echo "=== 2. dashboard 进程当前 model 相关 env ==="
run 'PID=$(ss -lntp 2>/dev/null | awk "/:9119 /{print \$NF}" | sed "s/.*pid=\([0-9]*\).*/\1/" | head -1); tr "\0" "\n" < /proc/$PID/environ | grep -E "^(BODY|AGENTOS_MODEL|INTEWELL_AGENT_HOME|CUSTOM_|OPENAI_)" | while IFS= read -r l; do k=${l%%=*}; v=${l#*=}; case "$k" in *KEY) echo "  $k len=${#v} head=$(printf %s "$v" | cut -c1-6)";; *) echo "  $k=$v";; esac; done'

echo "=== 3. 与 pty 同环境的 CLI 直连（120 秒上限）==="
run 'cd /opt/iraf-dev-workspace && KEY=$(cat /etc/iraf/agentos.key) && INTEWELL_AGENT_HOME=/opt/IntewellAgent OPENAI_BASE_URL=http://192.168.100.89:3000/v1 OPENAI_API_KEY="$KEY" CUSTOM_API_KEY="$KEY" timeout 110 python3 /usr/local/bin/intewell-agent-cli -z "只回一个字：好" 2>&1 | tail -12'

echo "=== 4. config.yaml 现状（model 段）==="
run 'grep -A4 "^model:" /root/.intewell/config.yaml'
