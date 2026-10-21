#!/usr/bin/env bash
# 机器人开发模式 REPL（IRAF 侧）：每一句输入都直接进 TaskFlow，由 IRAF 判定是不是机器人任务。
#
# 与 board_text_task.sh 的区别：这个是**连续会话**（--robot-mode），不是单次 --text。
# 演示/联调时用它证明：平台只要把话转过来，机器人语义的筛选发生在 IRAF 内部。
#
# 用法（板上 root）：
#   bash board_robot_mode.sh                  # 交互
#   bash board_robot_mode.sh < 输入文件        # 管道（每行一句，:q 退出）
set -euo pipefail

VERSION_DIR="${IRAF_VERSION_DIR:-/opt/iraf-sdk/current}"
ENV_FILE="${IRAF_ENV_FILE:-/etc/iraf/edge-intel.env}"

[ -f "$ENV_FILE" ] || { echo "缺装配单 $ENV_FILE" >&2; exit 2; }
cd "$VERSION_DIR"
set -a
# shellcheck disable=SC1090
. "$ENV_FILE"
set +a

if [ -z "${IRAF_AGENTOS_TOKEN:-}" ] && [ -r /etc/iraf/agentos.key ]; then
  IRAF_AGENTOS_TOKEN="$(cat /etc/iraf/agentos.key)"
  export IRAF_AGENTOS_TOKEN
fi
: "${IRAF_PROVIDER_TOKEN:?需要 IRAF_PROVIDER_TOKEN（跨机 Provider 鉴权）}"
: "${IRAF_AGENTOS_MODEL_URL:?需要 IRAF_AGENTOS_MODEL_URL（意图模型网关）}"
: "${IRAF_AGENTOS_MODEL:?需要 IRAF_AGENTOS_MODEL}"
export PYTHONPATH="$VERSION_DIR/lib:$VERSION_DIR/src"

exec python3 -m iraf_adapters.agentos.cli --robot-mode
