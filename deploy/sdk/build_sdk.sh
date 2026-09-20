#!/usr/bin/env bash
# SDK 打包入口（x86 开发端；设计 §4/§5，步骤 06）
#
# 职责：读声明矩阵 → 生成 proto stub（既有流程）→ 构建 SDK wheel 与 runtime bundle
#       → 写 manifest.json / sbom.spdx.json / sha256sums.txt（全部可复算）。
#
# 用法：
#   bash deploy/sdk/build_sdk.sh --help
#   bash deploy/sdk/build_sdk.sh --dry-run      # 只打印计划，不写任何文件
#   bash deploy/sdk/build_sdk.sh                # 完整构建 + 自校验
#   bash deploy/sdk/build_sdk.sh --verify       # 只做校验（产物被改一个字节即退出 4）
#   bash deploy/sdk/build_sdk.sh --target <id>  # 矩阵声明多个构建目标时必填
#
# 退出码（设计 §5 全脚本统一约定）：
#   0 成功 / 1 参数错误 / 2 预检失败 / 3 构建失败 / 4 校验失败
#
# 声明来源（禁止在本脚本硬编码版本、标签、包清单、镜像源）：config/sdk/package_matrix.yaml
# 解释器：默认取 PATH 中的 python3（需要 PyYAML + jsonschema）；可用 PYTHON=/usr/bin/python3 覆盖。
#   注：cron 环境里默认 python3 是 Hermes venv（3.11.15，有 PyYAML/jsonschema、无 mujoco），
#   打包入口两者都可运行；仿真相关脚本才必须用 /usr/bin/python3（见 plans/iraf-24h/00-执行规则.md §5.6）。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
REPO_ROOT_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON="${PYTHON:-python3}"
REPO_ROOT="${REPO_ROOT_DEFAULT}"
MATRIX="${MATRIX:-config/sdk/package_matrix.yaml}"
OUT="${OUT:-build/sdk}"
TARGET_ID=""
MANIFEST=""
PROTOC="${PROTOC:-}"
PYSTUB_OUT="build/sdk/pystub-local"
REQUIRE_STUBS="0"
MODE="build"
JSON_OUT=""

usage() {
  cat <<'USAGE'
SDK 打包入口（x86 开发端；IRAF iraf-24h 步骤 06）

用法：bash deploy/sdk/build_sdk.sh [选项]

选项：
  -h, --help            显示本帮助并退出（退出码 0）
  -n, --dry-run         只打印构建计划（声明值、预期产物名、成员数），不写任何文件
  --verify              只校验已有 manifest 与产物；任何不符即退出码 4
  --repo-root <dir>     仓库根目录（默认：本脚本上两级）
  --matrix <path>       产物矩阵（默认 config/sdk/package_matrix.yaml）
  --out <dir>           产物输出目录（默认 build/sdk）
  --target <id>         构建目标 id（矩阵声明多个目标时必填）
  --manifest <path>     --verify 用的 manifest（默认 <out>/manifest.json）
  --protoc <path>       protoc 可执行文件（默认取 PATH）
  --pystub-out <dir>    本机 stub 生成目录（默认 build/sdk/pystub-local）
  --require-stubs       既有 stub 目录不完整即失败（默认只警告并写进 manifest）
  --json-out <path>     preflight 摘要 JSON 落盘路径

环境变量：PYTHON（解释器，默认 python3）、MATRIX、OUT、PROTOC

退出码：0 成功 / 1 参数错误 / 2 预检失败（声明缺失、版本无法确定）/ 3 构建失败 / 4 校验失败

说明：目标端安装与 /health 健康检查不在本脚本职责内（板卡不在场 → DEFERRED）；
      stub 的 grpc 层本机没有 grpc_python_plugin，无法复现，脚本只登记不伪造。
USAGE
}

log() { printf '[build_sdk] %s\n' "$*"; }
err() { printf '[build_sdk][错误] %s\n' "$*" >&2; }

# step <中文标签> <命令...>：按计划执行，透传退出码；--dry-run 时只打印不执行。
step() {
  local label="$1"; shift
  log "== ${label}"
  if [ "${MODE}" = "dry-run" ]; then
    log "[dry-run] 跳过：$*"
    return 0
  fi
  if "$@"; then
    return 0
  else
    local rc=$?
    err "${label} 失败（退出码 ${rc}）"
    exit "${rc}"
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -n|--dry-run)
      MODE="dry-run"
      ;;
    --verify)
      MODE="verify"
      ;;
    --repo-root)
      [ "$#" -ge 2 ] || { err "--repo-root 缺少取值"; exit 1; }
      REPO_ROOT="$2"; shift
      ;;
    --matrix)
      [ "$#" -ge 2 ] || { err "--matrix 缺少取值"; exit 1; }
      MATRIX="$2"; shift
      ;;
    --out)
      [ "$#" -ge 2 ] || { err "--out 缺少取值"; exit 1; }
      OUT="$2"; shift
      ;;
    --target)
      [ "$#" -ge 2 ] || { err "--target 缺少取值"; exit 1; }
      TARGET_ID="$2"; shift
      ;;
    --manifest)
      [ "$#" -ge 2 ] || { err "--manifest 缺少取值"; exit 1; }
      MANIFEST="$2"; shift
      ;;
    --protoc)
      [ "$#" -ge 2 ] || { err "--protoc 缺少取值"; exit 1; }
      PROTOC="$2"; shift
      ;;
    --pystub-out)
      [ "$#" -ge 2 ] || { err "--pystub-out 缺少取值"; exit 1; }
      PYSTUB_OUT="$2"; shift
      ;;
    --require-stubs)
      REQUIRE_STUBS="1"
      ;;
    --json-out)
      [ "$#" -ge 2 ] || { err "--json-out 缺少取值"; exit 1; }
      JSON_OUT="$2"; shift
      ;;
    *)
      err "无法识别的参数：$1（用 --help 查看用法）"
      exit 1
      ;;
  esac
  shift
done

# 相对路径统一按仓库根解析，避免依赖调用方 cwd（服务/CI 调用时 cwd 不可假设）。
abspath() {
  case "$1" in
    /*) printf '%s' "$1" ;;
    *) printf '%s' "${REPO_ROOT}/$1" ;;
  esac
}
MATRIX="$(abspath "${MATRIX}")"
OUT="$(abspath "${OUT}")"
PYSTUB_OUT="$(abspath "${PYSTUB_OUT}")"
[ -n "${MANIFEST}" ] && MANIFEST="$(abspath "${MANIFEST}")"
[ -n "${JSON_OUT}" ] && JSON_OUT="$(abspath "${JSON_OUT}")"

LIB="${SCRIPT_DIR}/lib_manifest.py"
if [ ! -f "${LIB}" ]; then
  err "缺少实现层 ${LIB}"
  exit 3
fi
if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
fi

fail_with_code() {
  local rc="$1"
  err "$2"
  exit "${rc}"
}

PY_OPTS=(--repo-root "${REPO_ROOT}" --matrix "${MATRIX}" --out "${OUT}")
if [ -n "${TARGET_ID}" ]; then PY_OPTS+=(--target "${TARGET_ID}"); fi
if [ -n "${PROTOC}" ]; then PY_OPTS+=(--protoc "${PROTOC}"); fi

run_py() {
  "${PYTHON}" "${LIB}" "$@"
}

if [ "${MODE}" = "verify" ]; then
  # verify 只依赖 manifest + 产物，不读声明矩阵（面板/目标端上可以只有产物与清单）。
  log "模式：verify（只校验，不构建）"
  VERIFY_OPTS=(--repo-root "${REPO_ROOT}" --out "${OUT}")
  if [ -n "${MANIFEST}" ]; then VERIFY_OPTS+=(--manifest "${MANIFEST}"); fi
  if run_py verify "${VERIFY_OPTS[@]}"; then
    exit 0
  else
    fail_with_code "$?" "产物校验失败"
  fi
fi

if [ ! -f "${MATRIX}" ]; then
  err "声明矩阵不存在：${MATRIX}"
  exit 2
fi

log "仓库根：${REPO_ROOT}"
log "解释器：$(command -v "${PYTHON}")　矩阵：${MATRIX#"${REPO_ROOT}"/}　输出：${OUT#"${REPO_ROOT}"/}"
if [ "${MODE}" = "dry-run" ]; then
  log "模式：dry-run（只打印计划，不写任何文件）"
else
  log "模式：build（产物写入 ${OUT#"${REPO_ROOT}"/}，该目录已 gitignore，属证据区）"
fi

if [ "${MODE}" = "dry-run" ]; then
  # dry-run 不写盘：先打印完整计划，再逐步骤打印"将被执行"的命令，最后静态核对不落盘。
  run_py plan "${PY_OPTS[@]}"
  log "== 将按序执行的步骤（均不落盘）"
  log "  [dry-run] 预检：${PYTHON} ${LIB} preflight ${PY_OPTS[*]}"
  log "  [dry-run] 步骤 2 生成 stub：${PYTHON} ${LIB} gen-stubs ${PY_OPTS[*]} --pystub-out ${PYSTUB_OUT#"${REPO_ROOT}"/}"
  log "  [dry-run] 步骤 3 构建 wheel：${PYTHON} ${LIB} wheel ${PY_OPTS[*]}"
  log "  [dry-run] 步骤 4 组装 bundle：${PYTHON} ${LIB} bundle ${PY_OPTS[*]}"
  log "  [dry-run] 步骤 5 写 manifest：${PYTHON} ${LIB} manifest ${PY_OPTS[*]} --pystub-out ${PYSTUB_OUT#"${REPO_ROOT}"/}"
  log "  [dry-run] 步骤 6 自校验：${PYTHON} ${LIB} verify --repo-root ${REPO_ROOT} --out ${OUT#"${REPO_ROOT}"/}"
  log "dry-run 结束：未写入任何产物（退出码 0）"
  exit 0
fi

PREFLIGHT_OPTS=("${PY_OPTS[@]}")
if [ -n "${JSON_OUT}" ]; then PREFLIGHT_OPTS+=(--json-out "${JSON_OUT}"); fi
step "步骤 1 预检（矩阵契约 / 版本单一来源 / git / protoc / 既有 stub 状态）" \
  run_py preflight "${PREFLIGHT_OPTS[@]}"

STUB_OPTS=("${PY_OPTS[@]}" --pystub-out "${PYSTUB_OUT}")
if [ "${REQUIRE_STUBS}" = "1" ]; then STUB_OPTS+=(--require-stubs); fi
step "步骤 2 生成 proto stub（既有流程：protoc --python_out；不改 protoc 版本）" \
  run_py gen-stubs "${STUB_OPTS[@]}"

step "步骤 3 构建 SDK wheel（纯 Python，py3-none-any）" \
  run_py wheel "${PY_OPTS[@]}"

step "步骤 4 组装 runtime bundle（src + skills + profiles + config + deploy）" \
  run_py bundle "${PY_OPTS[@]}"

step "步骤 5 写 manifest.json + sbom.spdx.json + sha256sums.txt" \
  run_py manifest "${PY_OPTS[@]}" --pystub-out "${PYSTUB_OUT}"

step "步骤 6 自校验（manifest × 产物逐文件 SHA-256；篡改即退出 4）" \
  run_py verify "--repo-root" "${REPO_ROOT}" "--out" "${OUT}"

log "构建结束：产物与清单均在 ${OUT#"${REPO_ROOT}"/}（退出码 0）"
exit 0
