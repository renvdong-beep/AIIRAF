#!/usr/bin/env bash
# 把 dashboard 层（/root/.intewell）的 provider/key 配好并重启 dashboard
# 用法：bash scripts/apply_board_dashboard_apikey.sh [--apply]
set -uo pipefail
H=root@10.203.247.147
export SSHPASS=1
run() { sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$H" "$1" 2>&1; }

apply=0
[ "${1:-}" = "--apply" ] && apply=1

echo "=== 0. 前置检查 ==="
run 'echo "  key=/etc/iraf/agentos.key $(ls -l /etc/iraf/agentos.key | awk "{print \$1,\$3,\$5}")"; echo "  workspace=$(ls -d /opt/iraf-dev-workspace 2>&1)"; echo "  restore_script=$(ls -l /root/restore_agentos_config.py 2>&1 | awk "{print \$5}")"; echo "  restart_script=$(ls -l /root/restart_dashboard_ws.sh 2>&1 | awk "{print \$5}")"'

if [ "$apply" -ne 1 ]; then echo "=== 未带 --apply，未做任何改动 ==="; exit 0; fi

echo "=== 1. 写 provider 配置（restore_agentos_config.py，保留 dashboard 段）==="
run 'python3 /root/restore_agentos_config.py; echo "rc=$?"'

echo "=== 2. 校验 config.yaml 的 model 段与 dashboard 段共存 ==="
run 'python3 - <<PY
import yaml
c = yaml.safe_load(open("/root/.intewell/config.yaml"))
print("  keys:", sorted(c))
print("  model:", c.get("model"))
print("  mcp_discovery_timeout:", c.get("mcp_discovery_timeout"))
print("  dashboard.basic_auth.username:", (c.get("dashboard") or {}).get("basic_auth", {}).get("username"))
PY'

echo "=== 3. 校验 /root/.intewell/.env ==="
run 'ls -l /root/.intewell/.env; grep -c . /root/.intewell/.env; grep -oE "^(OPENAI_BASE_URL|OPENAI_API_KEY|CUSTOM_API_KEY)=" /root/.intewell/.env'

echo "=== 4. 重启 dashboard（cwd=/opt/iraf-dev-workspace，注入 key）==="
run 'bash /root/restart_dashboard_ws.sh; echo "rc=$?"'
