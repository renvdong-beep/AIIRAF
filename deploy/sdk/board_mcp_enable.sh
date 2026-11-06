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
# 多场景：ur5 固定 8765，piper 固定 8766（与 install_profiles.yaml 的 services[].ports 同源）。
# 可用 IRAF_MCP_UNITS="name:port name:port" 覆盖，但默认值必须与声明一致。
UNITS_SPEC="${IRAF_MCP_UNITS:-iraf-mcp-http.service:8765 iraf-mcp-piper.service:8766 iraf-mcp-joint.service:8767}"

if [ "${1:-}" = "--status" ]; then
  for spec in $UNITS_SPEC; do
    UNIT="${spec%%:*}"; PORT="${spec##*:}"
    systemctl --no-pager status "$UNIT" 2>&1 | head -6 || true
    ss -lntp 2>/dev/null | grep ":$PORT" || echo "[mcp-enable] $PORT 未监听"
  done
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

systemctl daemon-reload
for spec in $UNITS_SPEC; do
  UNIT="${spec%%:*}"; PORT="${spec##*:}"
  echo "[mcp-enable] 启用并启动 $UNIT（$PORT）"
  if ! systemctl enable --now "$UNIT"; then
    echo "[mcp-enable][错误] $UNIT 启动失败，最近日志：" >&2
    journalctl -u "$UNIT" -n 20 --no-pager 2>&1 | tail -20 >&2 || true
    exit 4
  fi
  for _ in $(seq 1 15); do
    if ss -lntp 2>/dev/null | grep -q ":$PORT"; then
      echo "[mcp-enable] $PORT 已监听"
      break
    fi
    sleep 1
  done
  if ! ss -lntp 2>/dev/null | grep -q ":$PORT"; then
    echo "[mcp-enable][错误] $PORT 未监听，最近日志：" >&2
    journalctl -u "$UNIT" -n 20 --no-pager 2>&1 | tail -20 >&2 || true
    exit 4
  fi
done

echo "[mcp-enable] 客户端 token：$CLIENT_TOKEN（600）"
