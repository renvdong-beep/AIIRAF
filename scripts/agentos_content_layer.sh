#!/usr/bin/env bash
# AgentOS 内容层处置 + 任务级 MCP 登记（开发机入口；不往平台容器/框架里写任何东西）
#
# 为什么要有这个脚本（2026-10-23 实机）：厂商换包（d20588cd → 8820f5a8）把
# /root/.intewell 清成空壳，手工在板上做的"停旁路 skill / 写 SOUL 守则 / mcp 登记"
# 连同"当时用的脚本"一起消失。所以这三步必须是**仓库里的可复跑动作**。
#
# 用法（开发机，root 通过 SSHPASS 给密码）：
#   IRAF_BOARD_HOST=10.203.247.147 SSHPASS=1 bash scripts/agentos_content_layer.sh --status
#   IRAF_BOARD_HOST=10.203.247.147 SSHPASS=1 bash scripts/agentos_content_layer.sh --apply
#   IRAF_BOARD_HOST=10.203.247.147 SSHPASS=1 bash scripts/agentos_content_layer.sh --register-mcp
#   IRAF_BOARD_HOST=10.203.247.147 SSHPASS=1 bash scripts/agentos_content_layer.sh --revert
#
# 边界（AGENTS 铁律 / 用户口径）：
#   · 只改"内容层"：skills 目录、SOUL.md、~/.intewell/config.yaml 的 mcp_servers 段；
#     不装包、不改容器、不打补丁。全部可 --revert。
#   · 凭据不进命令行值：MCP 子进程用 IRAF_PROVIDER_TOKEN_FILE 指向板上的 600 文件。
set -uo pipefail

HOST="${IRAF_BOARD_HOST:?需要 IRAF_BOARD_HOST（板卡 IP）}"
USER_NAME="${IRAF_BOARD_USER:-root}"
: "${SSHPASS:?需要 SSHPASS（板卡密码；只从环境变量传）}"
export SSHPASS
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PAYLOAD="${SCRIPT_DIR}/agentos_content_layer.py"
[ -f "${PAYLOAD}" ] || { echo "缺 ${PAYLOAD}" >&2; exit 2; }

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=10)
TARGET="${USER_NAME}@${HOST}"
run() { sshpass -e ssh "${SSH_OPTS[@]}" "${TARGET}" "$1" 2>&1; }
py()  { sshpass -e ssh "${SSH_OPTS[@]}" "${TARGET}" "python3 - $1" < "${PAYLOAD}" 2>&1; }

case "${1:-}" in
  --status)       py --status ;;
  --apply)        py --apply ;;
  --revert)       py --revert ;;
  --register-mcp)
    # 模型网关取值从**板上既有事实**读（config.yaml 的 model 段），不在脚本里写死地址
    MODEL_URL="${IRAF_AGENTOS_MODEL_URL:-$(run "grep -m1 'base_url:' /root/.intewell/config.yaml | awk '{print \$2}'")}"
    MODEL="${IRAF_AGENTOS_MODEL:-$(run "grep -m1 'default:' /root/.intewell/config.yaml | awk '{print \$2}'")}"
    [ -n "${MODEL_URL}" ] || { echo "读不到 model.base_url（请先配 key：scripts/apply_board_dashboard_apikey.sh --apply）" >&2; exit 2; }
    [ -n "${MODEL}" ]     || { echo "读不到 model.default（同上）" >&2; exit 2; }
    if run "intewell-agent-cli mcp list 2>/dev/null | grep -q iraf-task" >/dev/null; then
      echo "iraf-task 已登记（跳过；要重登记先 --unregister-mcp）"
      exit 0
    fi
    echo "登记 iraf-task（stdio / -m iraf_adapters.mcp.server）…"
    run "echo y | timeout 90 intewell-agent-cli mcp add iraf-task \
      --command /usr/bin/python3 \
      --env IRAF_ENV_FILE=/etc/iraf/edge-intel.env \
            PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
            IRAF_PROVIDER_TOKEN_FILE=/etc/iraf/runtime.secrets \
            IRAF_AGENTOS_MODEL_URL=${MODEL_URL} \
            IRAF_AGENTOS_MODEL=${MODEL} \
      --args -m iraf_adapters.mcp.server" | tail -6
    ;;
  --unregister-mcp)
    run "timeout 60 intewell-agent-cli mcp remove iraf-task" | tail -4 ;;
  *)
    cat >&2 <<'USAGE'
用法：bash scripts/agentos_content_layer.sh <动作>
  --status          只读体检（旁路 skill / SOUL 守则 / 备份区）
  --apply           停用旁路 skill + 写入路由守则（幂等，可 --revert）
  --register-mcp    登记任务级 MCP（iraf-task，stdio）；已登记则跳过
  --unregister-mcp  反登记（重登记前用）
  --revert          还原 SOUL.md 与旁路 skill
环境变量：IRAF_BOARD_HOST（必填）、IRAF_BOARD_USER（默认 root）、SSHPASS（必填）
          IRAF_AGENTOS_MODEL_URL / IRAF_AGENTOS_MODEL（可选，覆盖从板上读到的事实）
USAGE
    exit 1 ;;
esac
