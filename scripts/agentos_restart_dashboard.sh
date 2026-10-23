#!/usr/bin/env bash
# 重启 AgentOS dashboard（web 终端 / chat），让它从 IRAF 开发者工作区启动并带上模型 key。
#
# 为什么必须走这一条路（2026-10-23 实机两次踩到）：
#   · dashboard 启动时会**重写** ~/.intewell/config.yaml，把 model 段整段丢掉
#     （实测 09:00:48 重启 → 09:01:05 config 只剩 dashboard 段 + _config_version）。
#     只有本脚本这条"带 key 起 + 指定工作区"的路径能保住 model 段与进程环境里的 key。
#   · cwd 影响文件工具的工作视野：从 /opt/iraf-dev-workspace 起，模型先看到 README/案例，
#     而不是去读 /opt/iraf-sdk/current 源码然后"自己重写一遍抓取"。
#   · node 必须在 PATH 里（内嵌 chat 前端要它）。
#
# 用法（开发机，经 ssh stdin 执行，不在板上留脚本）：
#   sshpass -e ssh root@<板> "bash -s" < scripts/agentos_restart_dashboard.sh
# 或走包装：bash scripts/agentos_content_layer.sh --restart
#
# 可覆盖的环境变量：AGENTOS_DASHBOARD_PORT / AGENTOS_DASHBOARD_LOG /
#                   IRAF_AGENTOS_KEY_FILE / IRAF_AGENTOS_MODEL_URL /
#                   INTEWELL_NODE_DIR / IRAF_DEV_WORKSPACE
set -euo pipefail

PORT="${AGENTOS_DASHBOARD_PORT:-9119}"
LOG="${AGENTOS_DASHBOARD_LOG:-/root/.intewell/logs/gui.log}"
KEY_FILE="${IRAF_AGENTOS_KEY_FILE:-/etc/iraf/agentos.key}"
BASE_URL="${IRAF_AGENTOS_MODEL_URL:-http://192.168.100.89:3000/v1}"
NODE_DIR="${INTEWELL_NODE_DIR:-/opt/IntewellAgent/node-v22.23.2-linux-arm64/bin}"
WORKSPACE="${IRAF_DEV_WORKSPACE:-/opt/iraf-dev-workspace}"
CLI="${INTEWELL_CLI:-/usr/local/bin/intewell-agent-cli}"

[ -r "${KEY_FILE}" ] || { echo "读不到模型网关 key：${KEY_FILE}" >&2; exit 2; }
[ -x "${CLI}" ]      || { echo "找不到 AgentOS CLI：${CLI}（换包没装完？）" >&2; exit 2; }
[ -d "${WORKSPACE}" ] || echo "[警告] 缺开发者工作区 ${WORKSPACE}，改用 /root 启动" >&2
[ -d "${WORKSPACE}" ] || WORKSPACE=/root
KEY="$(cat "${KEY_FILE}")"

PID="$(ss -lntp 2>/dev/null | awk '/:'"${PORT}"' /{print $NF}' | sed 's/.*pid=\([0-9]*\).*/\1/' | head -1 || true)"
if [ -n "${PID:-}" ]; then
  echo "停止旧 dashboard pid=${PID}（cwd=$(readlink -f "/proc/${PID}/cwd" 2>/dev/null || echo '?')）"
  kill "${PID}" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 "${PID}" 2>/dev/null || break; sleep 0.5; done
  kill -9 "${PID}" 2>/dev/null || true
fi

echo "启动 dashboard：cwd=${WORKSPACE}，provider=custom ${BASE_URL}，node=${NODE_DIR}"
setsid env INTEWELL_AGENT_HOME="${INTEWELL_AGENT_HOME:-/opt/IntewellAgent}" \
  OPENAI_BASE_URL="${BASE_URL}" OPENAI_API_KEY="${KEY}" CUSTOM_API_KEY="${KEY}" \
  PATH="${NODE_DIR}:${PATH}" INTEWELL_NODE="${NODE_DIR}/node" \
  nohup bash -c "cd '${WORKSPACE}' && exec /usr/bin/python3 '${CLI}' dashboard --host 0.0.0.0 --port ${PORT}" \
  >>"${LOG}" 2>&1 </dev/null &

for _ in $(seq 1 40); do
  if ss -lntp 2>/dev/null | grep -q ":${PORT} "; then
    NEWPID="$(ss -lntp | awk '/:'"${PORT}"' /{print $NF}' | sed 's/.*pid=\([0-9]*\).*/\1/' | head -1)"
    echo "已监听 :${PORT}（pid=${NEWPID}）"
    echo "进程 cwd = $(readlink -f "/proc/${NEWPID}/cwd")"
    echo "node 在 PATH = $(PATH="${NODE_DIR}:${PATH}" command -v node || echo 缺)"
    exit 0
  fi
  sleep 0.5
done

echo "未起来，最近日志：" >&2
tail -20 "${LOG}" >&2
exit 2
