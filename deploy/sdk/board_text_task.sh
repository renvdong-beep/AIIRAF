#!/usr/bin/env bash
# 板上"一句话机器人任务"入口（IRAF 自带 CLI，不经 AgentOS 的工具装载）。
#
# 为什么要这个入口：AgentOS 的几种表面（-z 单轮 / ACP / dashboard）加载 MCP 工具的时机
# 各不相同，有的结构上看不到工具（见 docs/voice-to-task-path.md §5）。IRAF 自己的 CLI
# 直接走 TaskFlow，用于"只验证机器人侧语义→技能→执行"时最稳。
#
# 用法（板上 root）：
#   bash board_text_task.sh "抓取 box_01"
#   bash board_text_task.sh --list-skills
set -euo pipefail

TEXT="${1:?用法: board_text_task.sh \"抓取 box_01\"（或 --list-skills）}"
VERSION_DIR="${IRAF_VERSION_DIR:-/opt/iraf-sdk/current}"
ENV_FILE="${IRAF_ENV_FILE:-/etc/iraf/edge-intel.env}"

[ -f "$ENV_FILE" ] || { echo "缺装配单 $ENV_FILE" >&2; exit 2; }

cd "$VERSION_DIR"
set -a
# shellcheck disable=SC1090
. "$ENV_FILE"
set +a

# 模型网关 key 从 600 文件读，不落命令行、不打印（凭据零落盘，AGENTS 2.7）
if [ -z "${IRAF_AGENTOS_TOKEN:-}" ] && [ -r /etc/iraf/agentos.key ]; then
  IRAF_AGENTOS_TOKEN="$(cat /etc/iraf/agentos.key)"
  export IRAF_AGENTOS_TOKEN
fi
# 跨机 Provider 凭据：从 600 文件读（装配单按设计不写凭据），不落命令行、不打印
if [ -z "${IRAF_PROVIDER_TOKEN:-}" ] && [ -r /etc/iraf/runtime.secrets ]; then
  IRAF_PROVIDER_TOKEN="$(sed -n 's/^IRAF_PROVIDER_TOKEN=//p' /etc/iraf/runtime.secrets | head -1 | tr -d "'\"")"
  export IRAF_PROVIDER_TOKEN
fi
# 意图模型网关：优先环境，其次板上 dashboard 的既有事实（config.yaml 的 model 段）
if [ -z "${IRAF_AGENTOS_MODEL_URL:-}" ] && [ -r /root/.intewell/config.yaml ]; then
  IRAF_AGENTOS_MODEL_URL="$(sed -n 's/^  base_url: *//p' /root/.intewell/config.yaml | head -1 | tr -d "'\"")"
  export IRAF_AGENTOS_MODEL_URL
fi
if [ -z "${IRAF_AGENTOS_MODEL:-}" ] && [ -r /root/.intewell/config.yaml ]; then
  IRAF_AGENTOS_MODEL="$(sed -n 's/^  default: *//p' /root/.intewell/config.yaml | head -1 | tr -d "'\"")"
  export IRAF_AGENTOS_MODEL
fi
: "${IRAF_PROVIDER_TOKEN:?需要 IRAF_PROVIDER_TOKEN（跨机 Provider 鉴权）：写 /etc/iraf/runtime.secrets 或 export}"
: "${IRAF_AGENTOS_MODEL_URL:?需要 IRAF_AGENTOS_MODEL_URL（意图模型网关）：配 dashboard 的 model.base_url 或 export}"
: "${IRAF_AGENTOS_MODEL:?需要 IRAF_AGENTOS_MODEL（同上，model.default）}"

export PYTHONPATH="$VERSION_DIR/lib:$VERSION_DIR/src"

if [ "$TEXT" = "--list-skills" ]; then
  exec python3 -m iraf_adapters.agentos.cli --list-skills
fi
exec python3 -m iraf_adapters.agentos.cli --text "$TEXT"
