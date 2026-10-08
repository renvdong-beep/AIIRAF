#!/usr/bin/env bash
# 生成 iraf.v1 的 Python proto stub（**消息层 + grpc 层**）。
#
# 为什么需要这个脚本（而不是直接用 deploy/sdk/build_sdk.sh）：
#   `deploy/sdk/build_sdk.sh` 的步骤 2 只跑 `protoc --python_out`，并在用法说明里
#   自己写明「stub 的 grpc 层本机没有 grpc_python_plugin，无法复现，脚本只登记不伪造」。
#   本脚本用 `python3 -m grpc_tools.protoc`（grpcio-tools 自带 protoc 与 grpc 插件，
#   不依赖系统 protoc、也不引入 protoc 版本偏斜）把**这一层补上**，
#   供双域容器化验证的两端（控制域服务端 / 智能域客户端）加载。
#   ⇒ 两者是互补关系：build_sdk.sh 管打包与清单，本脚本管"能 import 的 stub"。
#
# 用法（仓库根）：
#   bash scripts/gen_proto_stubs.sh [输出目录]
#   输出目录缺省 build/generated/python（与 SDK 报错提示里的目录一致；build/ 已 gitignore）。
#
# 退出码：0 成功 / 1 参数或环境错误 / 2 生成失败 / 3 自检失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROTO_ROOT="${REPO_ROOT}/api/proto"
OUT_DIR="${1:-${REPO_ROOT}/build/generated/python}"
PYTHON="${PYTHON:-python3}"

err() { printf '[gen-proto][错误] %s\n' "$*" >&2; }
log() { printf '[gen-proto] %s\n' "$*"; }

command -v "${PYTHON}" >/dev/null 2>&1 || { err "找不到解释器 ${PYTHON}（可用 PYTHON=... 指定）"; exit 1; }

if ! "${PYTHON}" -c "import grpc_tools.protoc" >/dev/null 2>&1; then
  err "缺少 grpcio-tools：请先安装（pip3 install grpcio-tools）。"
  err "它同时提供 protoc 与 grpc 插件，是本脚本不依赖系统 protoc 的前提。"
  exit 1
fi

PROTO_FILES=(
  "${PROTO_ROOT}/iraf/v1/common.proto"
  "${PROTO_ROOT}/iraf/v1/skill.proto"
  "${PROTO_ROOT}/iraf/v1/runtime.proto"
  "${PROTO_ROOT}/iraf/v1/events.proto"
)
for item in "${PROTO_FILES[@]}"; do
  [ -f "${item}" ] || { err "proto 契约不存在：${item}（契约是唯一事实来源，缺一不可）"; exit 1; }
done

mkdir -p "${OUT_DIR}"
log "仓库根：${REPO_ROOT}"
log "proto 源：${PROTO_ROOT}"
log "输出目录：${OUT_DIR}"

if ! "${PYTHON}" -m grpc_tools.protoc \
      -I "${PROTO_ROOT}" \
      --python_out="${OUT_DIR}" \
      --grpc_python_out="${OUT_DIR}" \
      "${PROTO_FILES[@]}"; then
  err "protoc 生成失败（检查 proto 语法与 import 路径）"
  exit 2
fi

GENERATED=(
  "iraf/v1/common_pb2.py"
  "iraf/v1/skill_pb2.py"
  "iraf/v1/runtime_pb2.py"
  "iraf/v1/runtime_pb2_grpc.py"
  "iraf/v1/events_pb2.py"
  "iraf/v1/events_pb2_grpc.py"
)
for item in "${GENERATED[@]}"; do
  [ -f "${OUT_DIR}/${item}" ] || { err "生成物缺失：${item}"; exit 2; }
done

# 自检：真正 import 一次（只列文件不算数——namespace package 缺 __init__.py 也能 import，
# 但 sys.path 写错就 import 不了，必须实测）。
if ! PYTHONPATH="${OUT_DIR}" "${PYTHON}" - <<'PYCHECK'
import iraf.v1.runtime_pb2_grpc as rg
import iraf.v1.events_pb2_grpc as eg
import iraf.v1.skill_pb2 as sk
names = [n for n in dir(rg) if n.endswith("Stub")]
services = [n for n in dir(eg) if n.endswith("Stub")]
assert "SkillRuntimeServiceStub" in names, names
assert "EventServiceStub" in services, services
assert sk.SkillState.SKILL_STATE_RUNNING == 3, sk.SkillState.SKILL_STATE_RUNNING
print("[gen-proto] import 自检通过：%s | %s" % (names, services))
PYCHECK
then
  err "生成目录 import 自检失败：stub 不能被 Python 加载"
  exit 3
fi

log "完成：$(printf '%s ' "${GENERATED[@]}")"
log "使用方式：把 ${OUT_DIR} 加入 PYTHONPATH（服务端与客户端都要）"
