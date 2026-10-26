#!/usr/bin/env bash
# 本机（x86 仿真侧，10.203.247.121）拉起 MuJoCo Capability Provider。
#
# 与 scripts/dev_provider_supervisor.py --ensure 的关系：那个按 profile 的 dev 段守护，
# 但装配环境（IRAF_* 六个必填项）需要有 env 文件或已在跑的进程可继承；本脚本把装配
# 六项**从仓库现状显式生成**（场景报告 → emit_backend_config.py），零隐式默认值。
#
# 用法（仓库根，前台；Ctrl-C 退出）：
#   bash scripts/dev_provider_up.sh                 # 带窗口（默认，演示口径）
#   HEADLESS=1 bash scripts/dev_provider_up.sh      # 无窗口（只跑能力）
#   PORT=50099 bash scripts/dev_provider_up.sh      # 换端口
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

PORT="${PORT:-50052}"
HOST="${HOST:-0.0.0.0}"
SCENE="${SCENE:-build/models/piper-pick-scene.xml}"
OUT="$REPO/build/provider-up"
mkdir -p "$OUT"

TOKEN="$(PYTHONPATH=src python3 -c \
  'from iraf_adapters.remote.provider_backend import read_token_file; print(read_token_file("'"$HOME"'/.iraf/provider.token"))')"
if [ -z "$TOKEN" ]; then
  echo "[provider-up][错误] 缺 token：~/.iraf/provider.token（600）。跨机调用必须有鉴权，不编造默认值" >&2
  exit 2
fi
if [ ! -f "$SCENE" ]; then
  echo "[provider-up][错误] 场景不存在：$SCENE" >&2
  exit 2
fi

PYTHONPATH=src python3 scripts/emit_backend_config.py --scene "$SCENE" --compact > "$OUT/backend_config.json"

export PYTHONPATH=src:build/generated/python
export IRAF_PROFILE="${IRAF_PROFILE:-profiles/piper_mujoco.yaml}"
export IRAF_SAFETY_POLICY="${IRAF_SAFETY_POLICY:-profiles/safety/simulation_lab.yaml}"
export IRAF_SKILL_ROOT=skills
export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend
export IRAF_BACKEND_CONFIG="$(cat "$OUT/backend_config.json")"
export IRAF_EVENT_STORE="${IRAF_EVENT_STORE:-$OUT/events.db}"
export IRAF_PROVIDER_TOKEN="$TOKEN"

ARGS=(--host "$HOST" --port "$PORT")
if [ "${HEADLESS:-0}" = "1" ]; then
  ARGS+=(--no-window)
else
  # 不能沿用继承来的 DISPLAY：本机经 SSH 运行时它常是 localhost:10.0（X 转发），
  # 窗口会以 "X11: Failed to open display" 直接失败。默认死给本机桌面 :0。
  export DISPLAY="${PROVIDER_DISPLAY:-:0}"
  export XAUTHORITY="${PROVIDER_XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
  export MUJOCO_GL=glfw
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export IRAF_VIEWER_DISPLAY_MODE=interactive
fi

echo "[provider-up] profile=$IRAF_PROFILE scene=$SCENE"
echo "[provider-up] listen=$HOST:$PORT mode=$([ "${HEADLESS:-0}" = "1" ] && echo headless || echo window) event_store=$IRAF_EVENT_STORE"
exec build/mcp-venv/bin/python scripts/iraf_provider_serve.py "${ARGS[@]}"
