#!/usr/bin/env bash
# 钉 MCP 依赖闭包（edge-intel）
#
# 为什么需要它：`fetch_wheelhouse.sh` 只保证"按声明抓到了"，传递依赖仍可能漂到最新
# （实测：mcp 1.26 会连带抓 starlette 1.10，而板卡实测可用的是 1.0.1）。本脚本把
# **板卡验收过的那一组**钉进 wheelhouse，并删除同包的其它版本，保证离线安装可复现。
#
# 用法（仓库根）：bash deploy/sdk/pin_wheelhouse.sh [target-id] [dest]
set -euo pipefail

TARGET="${1:-aarch64-manylinux_2_28-cp311}"
DEST="${2:-build/wheelhouse/${TARGET}}"
PYTHON="${PYTHON:-python3}"

[ -d "$DEST" ] || { echo "[pin] 目标目录不存在：$DEST（先跑 fetch_wheelhouse.sh）" >&2; exit 2; }
mkdir -p "$DEST"

echo "[pin] 下载钉版闭包 → $DEST"
"$PYTHON" -m pip download --dest "$DEST" \
  --platform manylinux_2_28_aarch64 --platform manylinux2014_aarch64 \
  --python-version 3.11 --implementation cp \
  --abi cp311 --abi abi3 --abi none --only-binary=:all: \
  'mcp==1.26.0' 'starlette==1.0.1' 'sse-starlette==3.5.0' 'uvicorn==0.41.0' \
  'pydantic==2.13.4' 'anyio==4.15.1' 'python-multipart==0.0.27' 'typing-inspection==0.4.4' \
  'mujoco==3.14.0' 'grpcio==1.84.0' 'glfw==2.10.2' 'pydantic-core==2.46.4' >/dev/null

echo "[pin] 按钉版清理同包其它版本"
"$PYTHON" - "$DEST" <<'PY'
import os, re, sys
dest = sys.argv[1]
PINNED = {
    "mcp": "1.26.0",
    "starlette": "1.0.1",
    "sse-starlette": "3.5.0",
    "uvicorn": "0.41.0",
    "pydantic": "2.13.4",
    "pydantic-core": "2.46.4",
    "anyio": "4.15.1",
    "python-multipart": "0.0.27",
    "typing-inspection": "0.4.4",
    "mujoco": "3.14.0",
    "grpcio": "1.84.0",
    "glfw": "2.10.2",
}
files = [n for n in os.listdir(dest) if n.endswith(".whl")]
removed = []
kept_pinned = []
groups = {}
for name in files:
    head = name.rsplit(".whl", 1)[0]
    parts = head.split("-")
    pkg = parts[0].lower().replace("_", "-")
    version = parts[1]
    groups.setdefault(pkg, []).append((version, name))

for pkg, items in groups.items():
    if pkg in PINNED:
        want = PINNED[pkg]
        for version, name in items:
            if version != want:
                os.remove(os.path.join(dest, name)); removed.append(name)
            else:
                kept_pinned.append(name)
    elif len(items) > 1:
        # 非钉版包：保留版本号最高的一个（pip 不允许同包多版本共存）
        def key(item):
            return [int(p) if p.isdigit() else 0 for p in re.split(r"[.\-]", item[0])]
        items.sort(key=key)
        for _, name in items[:-1]:
            os.remove(os.path.join(dest, name)); removed.append(name)

print("[pin] 删除：%s" % (removed or "无"))
print("[pin] 钉版保留：%s" % sorted(kept_pinned))
missing = [f"{p}=={v}" for p, v in PINNED.items()
           if not any(n.startswith(p.replace("-", "_")) or n.startswith(p)
                      for n in os.listdir(dest))]
print("[pin] 缺失钉版：%s" % (missing or "无"))
PY

echo "[pin] 结果："
ls "$DEST" | grep -iE '^(mcp|starlette|sse_starlette|uvicorn|pydantic|anyio|python_multipart|typing_inspection)' | sort
