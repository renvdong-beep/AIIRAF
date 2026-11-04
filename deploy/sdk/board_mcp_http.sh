#!/usr/bin/env bash
# 在板上把任务级 MCP server 以 **streamable-http** 起起来（跨机/语音宿主用）。
#
# 为什么要有它：stdio 只能被**本机**宿主当子进程拉起；语音/语义前端常常不在板上
# （智控域实测 microphones=0、无 /dev/snd）。跨机就要靠 HTTP 传输。
#
# 端点与传输方式来自装配单 /etc/iraf/edge-intel.env 的 IRAF_MCP_*（默认 127.0.0.1:8765）。
# 本脚本只覆盖 host（0.0.0.0），因为要跨机访问；生产环境必须先做网络隔离/鉴权再放开。
#
# 用法（板上 root）：
#   bash start_mcp_http.sh              # 后台起，日志 /var/log/iraf/mcp-http.log
#   bash start_mcp_http.sh --stop
set -euo pipefail

ENV_FILE="${IRAF_ENV_FILE:-/etc/iraf/edge-intel.env}"
VERSION_DIR="${IRAF_VERSION_DIR:-/opt/iraf-sdk/current}"
LOG="${IRAF_MCP_HTTP_LOG:-/var/log/iraf/mcp-http.log}"
PORT="${IRAF_MCP_PORT:-8765}"
HOST="${IRAF_MCP_HOST_BIND:-0.0.0.0}"
TOKEN_FILE="${IRAF_MCP_TOKEN_FILE:-/etc/iraf/mcp-tokens.json}"
# 跨机访问必须显式给出允许的 Host（MCP SDK 默认只认 loopback，否则 421）。
# 不在这里猜板卡 IP（AGENTS 5.3）：缺省保持 loopback，跨机由调用方 export。
ALLOWED_HOSTS="${IRAF_MCP_ALLOWED_HOSTS:-127.0.0.1:${PORT},localhost:${PORT}}"
if [ "$HOST" != "127.0.0.1" ] && [ "$HOST" != "localhost" ]; then
  case "$ALLOWED_HOSTS" in
    "127.0.0.1:${PORT},localhost:${PORT}"|"localhost:${PORT},127.0.0.1:${PORT}")
      echo "[警告] HOST=$HOST 但 IRAF_MCP_ALLOWED_HOSTS 只有 loopback：" >&2
      echo "       跨机请求会被 421 拒绝；请 export IRAF_MCP_ALLOWED_HOSTS=127.0.0.1:$PORT,<板卡IP>:$PORT" >&2
      ;;
  esac
fi

if [ "${1:-}" = "--stop" ]; then
  pkill -f "iraf_adapters.mcp.server" || true
  echo "已停止（若本来没跑，忽略）"
  exit 0
fi

[ -f "$ENV_FILE" ] || { echo "缺装配单 $ENV_FILE" >&2; exit 2; }
mkdir -p "$(dirname "$LOG")"

# token 与模型网关 key 都从环境/文件来，不写进本脚本（铁律 5.3/2.7）
: "${IRAF_PROVIDER_TOKEN:?需要 IRAF_PROVIDER_TOKEN（跨机 Provider 鉴权）}"

IRAF_MCP_TRANSPORT=http \
IRAF_MCP_HOST="$HOST" \
IRAF_MCP_PORT="$PORT" \
IRAF_MCP_ALLOWED_HOSTS="$ALLOWED_HOSTS" \
IRAF_MCP_TOKEN_FILE="$TOKEN_FILE" \
IRAF_MCP_TLS_CERT="${IRAF_MCP_TLS_CERT:-}" \
IRAF_MCP_TLS_KEY="${IRAF_MCP_TLS_KEY:-}" \
IRAF_MCP_TLS_CA="${IRAF_MCP_TLS_CA:-}" \
IRAF_MCP_TLS_REQUIRE_CLIENT="${IRAF_MCP_TLS_REQUIRE_CLIENT:-}" \
IRAF_MCP_AUDIT_LOG="${IRAF_MCP_AUDIT_LOG:-}" \
IRAF_ENV_FILE="$ENV_FILE" \
PYTHONPATH="$VERSION_DIR/lib:$VERSION_DIR/src" \
IRAF_AGENTOS_MODEL_URL="${IRAF_AGENTOS_MODEL_URL:-http://192.168.100.89:3000/v1}" \
IRAF_AGENTOS_MODEL="${IRAF_AGENTOS_MODEL:-deepseek-flash}" \
nohup /usr/bin/python3 -m iraf_adapters.mcp.server >>"$LOG" 2>&1 &

sleep 3
echo "已启动：transport=streamable-http host=$HOST port=$PORT path=/mcp"
echo "  日志：$LOG"
echo "  自检：ss -lntp | grep $PORT"
tail -3 "$LOG" || true
