#!/usr/bin/env bash
# 把本机网关 key 推上板卡并写入 AgentOS deploy/.env（只改配置，不动厂商代码）
# 用法：bash scripts/apply_board_apikey.sh [--apply]
#   不带 --apply 只做：连通性/来源核对，不写任何文件
set -uo pipefail

H=root@10.203.247.147
export SSHPASS=1
SSH="sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10"
SCP="sshpass -e scp -o StrictHostKeyChecking=no -o ConnectTimeout=10"
ENVFILE=/root/IntewellAgent/artifacts/arm64/agentos-bundle-081ff92f-arm64/deploy/.env
KEYSRC=/home/coretek/.hermes/.env
LOCALKEY=/tmp/iraf_newkey.txt

apply=0
[ "${1:-}" = "--apply" ] && apply=1

# 1) 取本机 key（不回显）
KEY=$(grep -E '^NEWAPI_KEY=' "$KEYSRC" | head -1 | cut -d= -f2-)
if [ -z "${KEY:-}" ]; then echo "FAIL 本机 $KEYSRC 里没有 NEWAPI_KEY"; exit 2; fi
echo "[1] 本机 key: len=${#KEY}"

# 2) 本机验证这把 key 有效
code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 8 -H "Authorization: Bearer ${KEY}" \
  http://192.168.100.89:3000/v1/models)
echo "[2] 本机 -> 网关 /v1/models http=${code}"
if [ "$code" != "200" ]; then echo "FAIL 本机 key 无效，停止"; exit 3; fi

# 3) 板上旧 key 现状
echo "[3] 板上旧 key："
$SSH "$H" "E=$ENVFILE; v=\$(grep -E '^CUSTOM_API_KEY=' \$E | cut -d= -f2-); echo \"    len=\${#v} head=\$(printf %s \"\$v\" | cut -c1-6)***\"; c=\$(curl -s -o /dev/null -w '%{http_code}' --max-time 8 -H \"Authorization: Bearer \$v\" http://192.168.100.89:3000/v1/models); echo \"    板上 -> 网关 http=\$c\""

if [ "$apply" -ne 1 ]; then
  echo "[4] 未带 --apply，未写入任何文件"
  exit 0
fi

# 4) 推送 key 与写入脚本
umask 077
printf '%s' "$KEY" > "$LOCALKEY"
chmod 600 "$LOCALKEY"
$SCP "$LOCALKEY" "$H:/root/.iraf-newkey" >/dev/null || { echo "FAIL scp key"; exit 4; }
$SCP scripts/board/apply_custom_api_key.py "$H:/root/apply_custom_api_key.py" >/dev/null || { echo "FAIL scp script"; exit 5; }
echo "[4] 已推送 key 与写入脚本"

# 5) 写入并备份
echo "[5] 写入 deploy/.env："
$SSH "$H" "python3 /root/apply_custom_api_key.py /root/.iraf-newkey $ENVFILE; rc=\$?; shred -u /root/.iraf-newkey 2>/dev/null || rm -f /root/.iraf-newkey; exit \$rc"
rc=$?
if [ "$rc" -ne 0 ]; then echo "FAIL 写入失败 rc=$rc"; exit 6; fi

# 6) 用新值验证板上到网关
echo "[6] 板上新 key -> 网关："
$SSH "$H" "E=$ENVFILE; v=\$(grep -E '^CUSTOM_API_KEY=' \$E | cut -d= -f2-); c=\$(curl -s -o /tmp/gw_check.json -w '%{http_code}' --max-time 8 -H \"Authorization: Bearer \$v\" http://192.168.100.89:3000/v1/models); echo \"    http=\$c\"; head -c 80 /tmp/gw_check.json; echo"

echo "[7] 下一步：容器仍持旧 key（env 在创建时注入），需重建 worker 才生效"
