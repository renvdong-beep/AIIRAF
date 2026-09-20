#!/usr/bin/env bash
# 离线 wheelhouse 抓取入口（x86 开发端；设计 §4/§5，步骤 07）
#
# 职责：读声明矩阵 → 解析声明的镜像索引 → 按 python_tag/platform_tag 抓取 wheel
#       → 逐个校验平台标签 → 写 wheelhouse.json（供步骤 08 组装板级 bundle）。
#
# 用法：
#   bash deploy/sdk/fetch_wheelhouse.sh --help
#   bash deploy/sdk/fetch_wheelhouse.sh --dry-run              # 只打印计划（含解析结果），不下载不写盘
#   bash deploy/sdk/fetch_wheelhouse.sh --dry-run --no-resolve # 纯声明计划，连索引也不查询
#   bash deploy/sdk/fetch_wheelhouse.sh                        # 抓取到 build/wheelhouse/<target-id>/
#   bash deploy/sdk/fetch_wheelhouse.sh --verify               # 只校验已抓取目录内的平台标签
#   bash deploy/sdk/fetch_wheelhouse.sh --target <id> --dest <dir>
#
# 退出码（设计 §5 统一约定）：
#   0 成功 / 1 参数错误 / 2 预检失败（声明缺失、index_url 为空、目标标签无可用 wheel）/ 3 抓取失败 / 4 校验失败
#
# 声明来源（禁止在本脚本硬编码版本、标签、包清单、镜像源）：config/sdk/package_matrix.yaml
# 实现层：deploy/sdk/check_wheel_tags.py（标签匹配 + PEP 503 索引解析 + 下载 + 清单）。
#   注：本脚本**不使用** `pip download`：实测非 PyPI 源会让无人值守审批门拦截（见
#   docs/debug/2026-09-20-wheelhouse-fetch-index-api.md）。声明语义不变：换源只改矩阵一行。
# 解释器：默认取 PATH 中的 python3（只用标准库 + PyYAML）；可用 PYTHON=/usr/bin/python3 覆盖。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON="${PYTHON:-python3}"
REPO_ROOT="${REPO_ROOT_DEFAULT}"
MATRIX="config/sdk/package_matrix.yaml"
DEST="build/wheelhouse"
TARGET_ID=""
TIMEOUT="120"
DEADLINE="900"
RETRIES="2"
JSON_OUT=""
MODE="fetch"
NO_RESOLVE="0"

usage() {
  cat <<'USAGE'
离线 wheelhouse 抓取入口（x86 开发端；IRAF iraf-24h 步骤 07）

用法：bash deploy/sdk/fetch_wheelhouse.sh [选项]

选项：
  -h, --help             显示本帮助并退出（退出码 0）
  -n, --dry-run          只打印抓取计划（含索引解析结果），不下载、不写任何文件
  --no-resolve           dry-run 时连索引也不查询（纯声明计划，完全离线）
  --verify               只校验已有目录内的 wheel 平台标签，不抓取
  --repo-root <dir>      仓库根目录（默认：本脚本上两级）
  --matrix <path>        声明矩阵（默认 config/sdk/package_matrix.yaml）
  --target <id>          构建目标 id（矩阵声明多个目标时必填）
  --dest <dir>           下载目录（默认 build/wheelhouse/<target-id>；已 gitignore，属证据区）
  --timeout <秒>         单次请求超时（默认 120）
  --deadline <秒>        本次抓取总截止时间（默认 900，超时即退出码 3）
  --retries <次>         单文件最大重试次数（默认 2，有界，禁止无限重试）
  --json-out <path>      本次结果 JSON 落盘路径

环境变量：PYTHON（解释器，默认 python3）、MATRIX、DEST

退出码：0 成功 / 1 参数错误 / 2 预检失败 / 3 抓取失败 / 4 校验失败

说明：本脚本产出的 wheelhouse 只证明"开发端按声明标签抓到了正确的 wheel"，
     不构成目标端安装或运行证据（板卡不在场 ⇒ 目标端验收 DEFERRED）。
USAGE
}

log() { printf '[fetch_wheelhouse] %s\n' "$*"; }
err() { printf '[fetch_wheelhouse][错误] %s\n' "$*" >&2; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -n|--dry-run) MODE="dry-run" ;;
    --no-resolve) NO_RESOLVE="1" ;;
    --verify) MODE="verify" ;;
    --repo-root) [ "$#" -ge 2 ] || { err "--repo-root 缺少取值"; exit 1; }; REPO_ROOT="$2"; shift ;;
    --matrix) [ "$#" -ge 2 ] || { err "--matrix 缺少取值"; exit 1; }; MATRIX="$2"; shift ;;
    --target) [ "$#" -ge 2 ] || { err "--target 缺少取值"; exit 1; }; TARGET_ID="$2"; shift ;;
    --dest) [ "$#" -ge 2 ] || { err "--dest 缺少取值"; exit 1; }; DEST="$2"; shift ;;
    --timeout) [ "$#" -ge 2 ] || { err "--timeout 缺少取值"; exit 1; }; TIMEOUT="$2"; shift ;;
    --deadline) [ "$#" -ge 2 ] || { err "--deadline 缺少取值"; exit 1; }; DEADLINE="$2"; shift ;;
    --retries) [ "$#" -ge 2 ] || { err "--retries 缺少取值"; exit 1; }; RETRIES="$2"; shift ;;
    --json-out) [ "$#" -ge 2 ] || { err "--json-out 缺少取值"; exit 1; }; JSON_OUT="$2"; shift ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

if [ "${NO_RESOLVE}" = "1" ] && [ "${MODE}" != "dry-run" ]; then
  err "--no-resolve 只能与 --dry-run 一起使用"
  exit 1
fi

# 相对路径统一按仓库根解析，避免依赖调用方 cwd（CI/服务调用时 cwd 不可假设）。
abspath() {
  case "$1" in
    /*) printf '%s' "$1" ;;
    *) printf '%s' "${REPO_ROOT}/$1" ;;
  esac
}
MATRIX="$(abspath "${MATRIX}")"
[ -n "${JSON_OUT}" ] && JSON_OUT="$(abspath "${JSON_OUT}")"

LIB="${SCRIPT_DIR}/check_wheel_tags.py"
if [ ! -f "${LIB}" ]; then
  err "缺少实现层 ${LIB}"
  exit 3
fi
if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
fi

COMMON=(--repo-root "${REPO_ROOT}" --matrix "${MATRIX}")
[ -n "${TARGET_ID}" ] && COMMON+=(--target "${TARGET_ID}")
[ -n "${JSON_OUT}" ] && COMMON+=(--json-out "${JSON_OUT}")

log "仓库根：${REPO_ROOT}"
log "解释器：$(command -v "${PYTHON}")　矩阵：${MATRIX#"${REPO_ROOT}"/}"
log "模式：${MODE}$([ "${NO_RESOLVE}" = "1" ] && printf '（--no-resolve）')"

rc=0
case "${MODE}" in
  dry-run)
    # dry-run 只解析与打印：不建目录、不下载、不写 wheelhouse.json。
    PLAN=(fetch --dest "${DEST}" --dry-run)
    [ "${NO_RESOLVE}" = "1" ] && PLAN+=(--no-resolve)
    # 注意：argparse 的父级选项必须写在子命令**之前**，顺序不能调换。
    "${PYTHON}" "${LIB}" "${COMMON[@]}" "${PLAN[@]}" || rc=$?
    if [ "${rc}" -eq 0 ]; then
      log "dry-run 结束：未下载任何文件（退出码 0）"
    else
      err "dry-run 失败（退出码 ${rc}）：见上方中文原因"
    fi
    ;;
  verify)
    DEST_ABS="$(abspath "${DEST}")"
    if [ "${DEST}" = "build/wheelhouse" ] && [ -n "${TARGET_ID}" ]; then
      DEST_ABS="${DEST_ABS}/${TARGET_ID}"
    fi
    [ -d "${DEST_ABS}" ] || { err "待校验目录不存在：${DEST_ABS}"; exit 2; }
    if "${PYTHON}" "${LIB}" "${COMMON[@]}" verify --dir "${DEST_ABS}"; then
      log "标签校验通过：${DEST_ABS#"${REPO_ROOT}"/}"
    else
      rc=$?
      err "标签校验失败（退出码 ${rc}）"
    fi
    ;;
  fetch)
    "${PYTHON}" "${LIB}" "${COMMON[@]}" fetch --dest "${DEST}" --timeout "${TIMEOUT}" \
      --deadline "${DEADLINE}" --retries "${RETRIES}" || rc=$?
    if [ "${rc}" -eq 0 ]; then
      log "抓取结束：产物在 ${DEST#"${REPO_ROOT}"/}（已 gitignore，属证据区；退出码 0）"
    else
      err "抓取失败（退出码 ${rc}）：见上方中文原因"
    fi
    ;;
  *)
    err "未知模式：${MODE}"
    exit 1
    ;;
esac

exit "${rc}"
