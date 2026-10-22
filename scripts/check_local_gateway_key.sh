#!/usr/bin/env bash
# 校验本机 Hermes 用的网关 key 是否有效（只打印 HTTP 码，不打印 key）
set -uo pipefail
set +x
ENVF=/home/coretek/.hermes/.env
if [ ! -f "$ENVF" ]; then echo "NO_ENV_FILE"; exit 2; fi
# shellcheck disable=SC1090
KEY=$(grep -E '^NEWAPI_KEY=' "$ENVF" | head -1 | cut -d= -f2-)
if [ -z "${KEY:-}" ]; then echo "NO_NEWAPI_KEY_IN_ENV"; exit 3; fi
echo "local_key_len=${#KEY}"
code=$(curl -s -o /tmp/gw_local_models.json -w '%{http_code}' --max-time 8 \
  -H "Authorization: Bearer ${KEY}" http://192.168.100.89:3000/v1/models)
echo "local_http=${code}"
echo "resp_head=$(head -c 120 /tmp/gw_local_models.json)"
