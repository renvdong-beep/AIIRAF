#!/usr/bin/env bash
# 在边缘板（edge-intel）上启用北向 MCP HTTP 常驻服务：签发 token（首次）→ enable/start → 自检。
#
# 用法（板上 root，安装完 SDK 后）：
#   bash /opt/iraf-sdk/current/deploy/sdk/board_mcp_enable.sh
#   bash /opt/iraf-sdk/current/deploy/sdk/board_mcp_enable.sh --status
#
# 边界：
#   · 只操作 iraf-mcp-http.service 与 /etc/iraf/mcp-tokens.json / mcp-client.token；
#   · 不打印 token 到日志（只在首次签发时写入 600 文件，由操作者取走）；
#   · 服务缺 token 会 fail-closed，不会以无鉴权方式监听 8765。
set -euo pipefail

CONFIG_DIR="${IRAF_CONFIG_DIR:-/etc/iraf}"
VERSION_DIR="${IRAF_VERSION_DIR:-/opt/iraf-sdk/current}"
TOKEN_FILE="${IRAF_MCP_TOKEN_FILE:-$CONFIG_DIR/mcp-tokens.json}"
CLIENT_TOKEN="${IRAF_MCP_CLIENT_TOKEN:-$CONFIG_DIR/mcp-client.token}"
UNIT="iraf-mcp-http.service"
PORT="${IRAF_MCP_PORT:-8765}"

if [ "${1:-}" = "--status" ]; then
  systemctl --no-pager status "$UNIT" 2>&1 | head -12 || true
  echo ---
  ss -lntp 2>/dev/null | grep ":$PORT" || echo "8765 未监听"
  [ -f "$CLIENT_TOKEN" ] && echo "客户端 token：$CLIENT_TOKEN（600，内容不打印）" || true
  exit 0
fi

command -v systemctl >/dev/null 2>&1 || { echo "[mcp-enable][错误] 没有 systemctl" >&2; exit 2; }
[ -d "$VERSION_DIR/src" ] || { echo "[mcp-enable][错误] 版本目录不存在：$VERSION_DIR" >&2; exit 2; }

if [ ! -s "$TOKEN_FILE" ]; then
  echo "[mcp-enable] 首次签发 MCP token（明文只写入 $CLIENT_TOKEN，不回显）"
  umask 077
  ISSUE_OUT="$(mktemp)"
  if ! PYTHONPATH="$VERSION_DIR/lib:$VERSION_DIR/src" /usr/bin/python3 \
      -m iraf_tools.mcp_tokens --file "$TOKEN_FILE" issue \
      --subject "${IRAF_MCP_CLIENT_SUBJECT:-upper-sdk}" \
      --tenant "${IRAF_MCP_CLIENT_TENANT:-default}" \
      --scopes task.submit,task.read --resources '*' --expires-days 90 \
      > "$ISSUE_OUT" 2>&1; then
    echo "[mcp-enable][错误] 签发 token 失败：" >&2
    tail -5 "$ISSUE_OUT" >&2
    rm -f "$ISSUE_OUT"
    exit 3
  fi
  tail -1 "$ISSUE_OUT" > "$CLIENT_TOKEN"
  chmod 600 "$CLIENT_TOKEN"
  rm -f "$ISSUE_OUT"
  [ -s "$CLIENT_TOKEN" ] || { echo "[mcp-enable][错误] token 文件为空" >&2; exit 3; }
fi

echo "[mcp-enable] 启用并启动 $UNIT"
systemctl daemon-reload
systemctl enable --now "$UNIT"

for _ in $(seq 1 15); do
  if ss -lntp 2>/dev/null | grep -q ":$PORT"; then
    echo "[mcp-enable] 8765 已监听"
    systemctl --no-pager --lines=0 status "$UNIT" 2>&1 | head -3 || true
    echo "[mcp-enable] 客户端 token：$CLIENT_TOKEN（600）"
    exit 0
  fi
  sleep 1
done

echo "[mcp-enable][错误] 8765 未监听，最近日志：" >&2
journalctl -u "$UNIT" -n 20 --no-pager 2>&1 | tail -20 >&2 || true
exit 4
