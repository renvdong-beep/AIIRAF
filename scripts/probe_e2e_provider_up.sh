#!/usr/bin/env bash
# 探针用：起一个**独立端口**的 x86 MuJoCo Provider（headless），供「复位→立刻动」跨进程验证。
# 不动用户的 50052/窗口实例；端口、事件库、日志都落在 build/e2e/ 下。
# 用法：bash scripts/probe_e2e_provider_up.sh            # 前台起（Ctrl-C 退出）
#       PORT=50099 bash scripts/probe_e2e_provider_up.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

PORT="${PORT:-50099}"
OUT="$REPO/build/e2e"
mkdir -p "$OUT"

TOKEN="$(PYTHONPATH=src python3 -c \
  'from iraf_adapters.remote.provider_backend import read_token_file; print(read_token_file("'"$HOME"'/.iraf/provider.token"))')"
if [ -z "$TOKEN" ]; then
  echo "[probe-e2e][错误] 缺 token：先建 ~/.iraf/provider.token（600），不编造默认值" >&2
  exit 2
fi

PYTHONPATH=src python3 scripts/emit_backend_config.py \
  --scene build/models/piper-pick-scene.xml --compact > "$OUT/backend_config.json"
echo "[probe-e2e] backend_config=$OUT/backend_config.json ($(wc -c < "$OUT/backend_config.json") 字节)"

export PYTHONPATH=src:build/generated/python
export IRAF_PROFILE=profiles/piper_mujoco.yaml
export IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml
export IRAF_SKILL_ROOT=skills
export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend
export IRAF_BACKEND_CONFIG="$(cat "$OUT/backend_config.json")"
export IRAF_EVENT_STORE="$OUT/events.db"
export IRAF_PROVIDER_TOKEN="$TOKEN"
export MUJOCO_GL="${MUJOCO_GL:-glfw}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

echo "[probe-e2e] 启动 provider：127.0.0.1:$PORT（headless）"
exec build/mcp-venv/bin/python scripts/iraf_provider_serve.py \
  --host 127.0.0.1 --port "$PORT" --no-window
