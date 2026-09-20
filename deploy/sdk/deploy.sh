#!/usr/bin/env bash
# SDK 部署入口（x86 开发端；IRAF iraf-24h 步骤 10）：双通道传输 —— ssh 直连 / media（隔离网人工拷贝）。
#
# 用法：
#   bash deploy/sdk/deploy.sh --help
#   bash deploy/sdk/deploy.sh --transport media --output build/iraf-24h/10
#       → 产出 bundle 副本 + sha256sum.txt + README-安装.md + deploy-report.json，退出码 0
#   bash deploy/sdk/deploy.sh --transport media --output <dir> --dry-run
#       → 只打印计划，不写任何文件
#   bash deploy/sdk/deploy.sh --transport ssh --target <user@host> --port <n> --dry-run
#       → 可达性探测 + 打印将要执行的远端命令；目标不可达退出码 2（当前板卡不在场：DEFERRED）
#   bash deploy/sdk/deploy.sh --transport ssh --target <user@host> --allow-unverified
#       → 真实部署：scp → 远端 sha256 复核 → 远端 install.sh → 取回 install-report.json
#
# 退出码（与设计 §5 / 08/09 脚本统一约定）：
#   0 成功 / 1 参数错误 / 2 预检失败（bundle 缺校验和或多义、目标不可达、缺 ssh/scp）
#   3 传输或远端执行失败 / 4 校验失败（SHA-256 不符）/ 5 远端后置校验失败
#
# 硬边界（不得违反）：
#   * 主机地址与凭据只来自参数或环境变量（IRAF_DEPLOY_TARGET / IRAF_DEPLOY_PORT /
#     IRAF_DEPLOY_IDENTITY）；本脚本与本实现层不含任何地址常量（步骤 10 门禁）。
#   * ssh 通道一律 BatchMode=yes：无人值守下不做密码交互、不落密码，只支持密钥/agent。
#   * media 通道只产出**人工拷贝交接物**；演练只产出计划。二者都**不构成**目标端证据。
#   * 目标端真实安装、/health、事件库检查一律 DEFERRED（x86-first 战役，板卡不在场）；
#     禁止把 --dry-run 结果写成"已部署/已安装"。
#
# 实现层：deploy/sdk/lib_deploy.py（**只用标准库**，不依赖 PyYAML/jsonschema）。
# 解释器：默认取 PATH 中的 python3；可用 PYTHON=/usr/bin/python3 覆盖。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
REPO_ROOT_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON="${PYTHON:-python3}"
REPO_ROOT="${REPO_ROOT:-${REPO_ROOT_DEFAULT}}"
LIB="${SCRIPT_DIR}/lib_deploy.py"
TRANSPORT=""
OUTPUT=""
DRY_RUN="0"
EXTRA=()

usage() {
  cat <<'USAGE'
SDK 部署入口（IRAF iraf-24h 步骤 10）

用法：bash deploy/sdk/deploy.sh --transport <media|ssh> [选项]

选项：
  -h, --help              显示本帮助并退出（退出码 0）
  -n, --dry-run           只打印计划，不做任何传输与写盘
  --transport <通道>      必填：media（隔离网人工拷贝）| ssh（直连部署）
  --output <dir>          media：交接物输出目录（必填）；ssh：证据落盘目录（默认 build/deploy）
  --bundle <tar.gz>       板级 bundle（默认取 --bundle-dir 下唯一带伴随 *.sha256 的 tar.gz；
                          0 个或多个都显式失败，不自动挑第一个）
  --bundle-dir <dir>      bundle 所在目录（默认 build/sdk）
  --target <user@host>    ssh 目标（也可用环境变量 IRAF_DEPLOY_TARGET）；必须含 user@host
  --port <n>              ssh 端口（也可用 IRAF_DEPLOY_PORT；默认 22）
  --identity <key>        ssh 私钥（也可用 IRAF_DEPLOY_IDENTITY）
  --remote-dir <dir>      远端暂存目录（默认 ~/iraf-deploy）
  --allow-unverified      转发给远端 install.sh（默认不转发：fail-closed）
  --python <exe>          远端解释器（转发给 install.sh，默认 python3）
  --timeout-s <n>         可达性探测超时秒数（默认 4）
  --json-out <path>       额外落一份部署报告 JSON
  --repo-root <dir>       仓库根目录（默认：本脚本上两级）

环境变量：PYTHON、REPO_ROOT、IRAF_DEPLOY_TARGET、IRAF_DEPLOY_PORT、IRAF_DEPLOY_IDENTITY

退出码：0 成功 / 1 参数错误 / 2 预检失败 / 3 传输或远端执行失败 / 4 校验失败 / 5 远端后置校验失败

说明：media 通道产出的是**交接物**（人工拷贝用），不构成目标端安装证据；目标端真实安装与
      /health 验收 DEFERRED（板卡不在场）。演练结果的措辞只能是"计划/演练完成"，不是"已部署"。
USAGE
}

log() { printf '[deploy.sh] %s\n' "$*"; }
err() { printf '[deploy.sh][错误] %s\n' "$*" >&2; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -n|--dry-run)
      DRY_RUN="1"
      ;;
    --transport)
      [ "$#" -ge 2 ] || { err "--transport 缺少取值（media|ssh）"; exit 1; }
      TRANSPORT="$2"; shift
      ;;
    --output)
      [ "$#" -ge 2 ] || { err "--output 缺少取值"; exit 1; }
      OUTPUT="$2"; shift
      ;;
    *)
      # 其余取值型/开关参数直接转发给实现层，避免两处各写一份校验。
      case "$1" in
        --bundle|--bundle-dir|--target|--port|--identity|--remote-dir|--python|--timeout-s|--json-out|--repo-root)
          [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
          EXTRA+=("$1" "$2"); shift
          ;;
        --allow-unverified)
          EXTRA+=("$1")
          ;;
        *)
          err "无法识别的参数：$1（用 --help 查看用法）"
          exit 1
          ;;
      esac
      ;;
  esac
  shift
done

case "${TRANSPORT}" in
  media|ssh) ;;
  "")
    err "缺少 --transport（media|ssh）"
    exit 1
    ;;
  *)
    err "未知传输通道：${TRANSPORT}（只支持 media|ssh）"
    exit 1
    ;;
esac

if [ "${TRANSPORT}" = "media" ] && [ -z "${OUTPUT}" ]; then
  err "media 通道必须显式给 --output <dir>（交接物输出目录）"
  exit 1
fi

[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}（本脚本必须与 lib_deploy.py 同目录）"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || {
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
}

ARGS=(--repo-root "${REPO_ROOT}")
if [ -n "${OUTPUT}" ]; then ARGS+=(--output "${OUTPUT}"); fi
if [ "${DRY_RUN}" = "1" ]; then ARGS+=(--dry-run); fi
if [ "${#EXTRA[@]}" -gt 0 ]; then ARGS+=("${EXTRA[@]}"); fi

cd "${REPO_ROOT}"
if [ "${DRY_RUN}" = "1" ]; then
  log "模式：dry-run（只打印计划；不传输、不写盘）"
else
  case "${TRANSPORT}" in
    media) log "模式：media 通道（产出人工拷贝交接物；不构成目标端证据）" ;;
    ssh) log "模式：ssh 通道（真实部署；目标端验收仍为 DEFERRED）" ;;
  esac
fi

# 注意（踩过的坑）：不能用 `if ! cmd; then rc=$?` —— 取反后的 $? 恒为 0，退出码会被吞掉。
set +e
"${PYTHON}" "${LIB}" "${TRANSPORT}" "${ARGS[@]}"
rc=$?
set -e
if [ "${rc}" -ne 0 ]; then
  case "${rc}" in
    2) err "预检失败（退出码 2）：bundle 缺校验和/多义、目标不可达或缺 ssh 工具" ;;
    4) err "校验失败（退出码 4）：SHA-256 不符，禁止继续" ;;
    5) err "远端后置校验失败（退出码 5）：不得表述为部署成功" ;;
    *) err "部署未通过（退出码 ${rc}）" ;;
  esac
  exit "${rc}"
fi
log "完成（退出码 0）"
exit 0
