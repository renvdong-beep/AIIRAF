#!/usr/bin/env bash
# 板上入口：把 IRAF MCP 绑定进 AgentOS worker 的共享 config.yaml。
# 实际逻辑在 agentos_mcp_bind.py；这里只拼版本目录与 PYTHONPATH。
#
# 用法（板上 root）：
#   bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh --status
#   bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh --apply \
#        --url http://10.203.247.147:8765/mcp
#   bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh --revert
set -euo pipefail

VERSION_DIR="${IRAF_VERSION_DIR:-/opt/iraf-sdk/current}"
TOOL="${VERSION_DIR}/deploy/sdk/agentos_mcp_bind.py"
[ -f "$TOOL" ] || { echo "[agentos-mcp][错误] 缺少 ${TOOL}" >&2; exit 2; }

export PYTHONPATH="${VERSION_DIR}/lib:${VERSION_DIR}/src:${VERSION_DIR}/deploy/sdk${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$TOOL" "$@"
