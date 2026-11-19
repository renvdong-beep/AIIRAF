#!/usr/bin/env bash
# 无板卡本地 MCP（HTTP Streamable）启动器：只在本机 127.0.0.1 监听。
set -euo pipefail
REPO=/home/coretek/AIIRAF
STATE=/home/coretek/.config/iraf
mkdir -p "$STATE"
TOKEN_FILE="$STATE/mcp-local-tokens.json"
TOKEN="local-$(head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n')"
python3 - "$TOKEN_FILE" "$TOKEN" <<'PY'
import hashlib, json, sys, os
from pathlib import Path
path, token = Path(sys.argv[1]), sys.argv[2]
doc = {"version": 1, "tokens": [{
    "id": "local-1", "subject": "local-agent", "tenant": "local",
    "scopes": ["task.submit", "task.read"], "resources": ["*"],
    "sha256": hashlib.sha256(token.encode()).hexdigest(),
    "enabled": True,
}]}
path.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
os.chmod(path, 0o600)
PY
printf '%s\n' "$TOKEN" > "$STATE/mcp-local.token"
chmod 600 "$STATE/mcp-local.token"
cd "$REPO"
echo "token file: $TOKEN_FILE"
echo "token: $TOKEN"
exec env \
  PYTHONPATH="$REPO/src:/home/coretek/.local/opt/iraf-sdk/0.4.4/lib" \
  IRAF_MCP_TRANSPORT=http IRAF_MCP_HOST=127.0.0.1 IRAF_MCP_PORT=8766 \
  IRAF_MCP_TOKEN_FILE="$TOKEN_FILE" \
  IRAF_MCP_AUDIT_LOG=/home/coretek/.local/state/iraf/mcp-local-audit.jsonl \
  IRAF_EVENT_STORE=/home/coretek/.local/state/iraf/local-mcp.db \
  IRAF_INTENT_KEYWORD=1 \
  IRAF_PROFILE=profiles/unitree_g1_mujoco.yaml \
  IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml \
  IRAF_SKILL_ROOT=/home/coretek/AIIRAF/skills \
  IRAF_BACKEND_ENTRYPOINT=iraf_adapters.unitree_g1.backend:UnitreeG1Backend \
  'IRAF_BACKEND_CONFIG={"declaration":"config/unitree_g1_loopback.yaml"}' \
  IRAF_SCENE_REGISTRY_MODE=off \
  IRAF_REPO_ROOT="$REPO" \
  python3 -m iraf_adapters.mcp.server
