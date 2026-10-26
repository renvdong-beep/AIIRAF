#!/usr/bin/env bash
# 只读：dashboard/worker 日志里的模型/provider 报错 + 平台侧 provider 设置的真实读取点
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

echo "=== 1. dashboard gui.log 尾部 ==="
run 'tail -25 /root/.intewell/logs/gui.log 2>&1'

echo "=== 2. dashboard agent.log 尾部 ==="
run 'tail -15 /root/.intewell/logs/agent.log 2>&1'

echo "=== 3. worker-1 最近日志里的 provider/凭据相关行 ==="
run 'docker logs --tail 400 agentos-worker-1 2>&1 | grep -iE "provider|credential|api key|unauthor|401|openrouter|custom" | tail -15'

echo "=== 4. 平台侧 provider 配置读取点（只列文件:行）==="
run 'D=/usr/local/lib/python3.11/site-packages/intewell_cli; grep -rn "api_key_env_vars\|No usable credentials" $D --include=*.py | head -8'

echo "=== 5. dashboard 是否有 provider 设置路由 ==="
run 'D=/usr/local/lib/python3.11/site-packages/intewell_cli; grep -rnoE "\"/api/[a-z/_-]*(provider|model|config)[a-z/_-]*\"" $D --include=*.py | sort -u | head -15'
