#!/usr/bin/env bash
# 目标端自检入口（步骤 09）：bundle 校验和 + 逐成员 SHA-256 → 板卡声明门禁 → /health(HTTP/gRPC)
# → 事件库写入检查 → 输出证据 JSON。
#
# 用法：
#   bash deploy/sdk/verify.sh --help
#   bash deploy/sdk/verify.sh --bundle build/sdk/iraf-board-e300-0.2.0-aarch64.tar.gz --dry-run \
#        --allow-unverified --json-out build/iraf-24h/09/verify-evidence.json
#   bash deploy/sdk/verify.sh --root <沙箱或 />          # 安装后置校验（install.sh 会自动调用本入口）
#
# 退出码（设计 §5 统一约定）：
#   0 成功/演练完成   1 参数错误   2 声明缺失或契约不符
#   4 校验失败（SHA-256 不符、bundle 成员被篡改、已装树漂移）
#   5 后置判定失败（服务未启动、/health 或 gRPC 探针不通过、事件库不可写）
#
# 硬边界（不得违反）：
#   * **服务未启动 = 判定失败（退出 5），不是通过**；只有显式 --dry-run 才降级为"演练完成"，
#     且证据里 service_state= SERVICE_NOT_RUNNING、evidence_scope=rehearsal_only、
#     verified=false。目标端 /health 验收 DEFERRED（x86-first 战役，板卡不在场）。
#   * 本脚本不启动、不重启任何服务；不修改已装文件（只写安装记录与证据）。
#   * 声明（端点/超时/事件库路径/安装记录名）全部来自 config/sdk/package_matrix.yaml，
#     缺声明即退出 2，脚本里不写死（铁律 5.3）。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"

PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_target_verify.py"
INSTALLED_DIR=""
PASSTHROUGH=()

usage() {
  cat <<'USAGE'
目标端自检入口（IRAF iraf-24h 步骤 09）

用法：bash verify.sh [--bundle <tar.gz>] [--root <dir>] [--dry-run] [选项]

选项：
  -h, --help             显示本帮助并退出（退出码 0）
  --bundle <tar.gz>      板级 bundle（需同目录存在 <bundle>.sha256）；做校验和与逐成员复算
  --root <dir>           文件系统根：真实自检用 /（默认），沙箱演练给沙箱目录
  -n, --dry-run          演练：不做目标端判定，服务项记为 DEFERRED（仍如实记录未启动状态）
  --allow-unverified     显式放行"板卡声明未实测"（不放行契约层失败与事实不符）
  --installed-dir <dir>  已装版本目录（默认由脚本自身位置推断：<version>/scripts/verify.sh）
  --health-url <url>     /health 覆盖（默认取声明；记录 overridden 标记，仅用于本机解析验证）
  --grpc-target <host:p> gRPC 覆盖（同上）
  --timeout-s <秒>       探针超时覆盖
  --event-store <path>   事件库路径覆盖（同上）
  --python <exe>         解释器（默认 python3）
  --json-out <path>      额外落一份证据 JSON（本机路径，不入库）

退出码：0 成功/演练完成 / 1 参数错误 / 2 声明缺失或契约不符
        4 校验失败 / 5 后置判定失败（服务未启动等）

说明：本机是 x86_64 开发端、板卡不在场（x86-first）。真实自检在本机会明确失败
      （服务未启动、架构不符），这是 fail-closed 的预期行为；目标端 /health 验收 DEFERRED。
USAGE
}

log() { printf '[verify] %s\n' "$*" >&2; }
err() { printf '[verify][错误] %s\n' "$*" >&2; }

# 已装版本目录：脚本位于 <version_dir>/scripts/verify.sh 时自动推断（不做全盘搜索）。
CANDIDATE="$(cd "${SCRIPT_DIR}/.." 2>/dev/null && pwd || true)"
if [ -n "${CANDIDATE}" ] && [ -f "${CANDIDATE}/config/sdk/package_matrix.yaml" ]; then
  INSTALLED_DIR="${CANDIDATE}"
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -n|--dry-run|--allow-unverified)
      PASSTHROUGH+=("$1")
      ;;
    --bundle|--root|--health-url|--grpc-target|--timeout-s|--event-store|--python|--json-out|--installed-dir)
      [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
      if [ "$1" = "--python" ]; then
        PYTHON="$2"
      else
        PASSTHROUGH+=("$1" "$2")
      fi
      shift
      ;;
    *)
      err "无法识别的参数：$1（用 --help 查看用法）"
      exit 1
      ;;
  esac
  shift
done

[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}（本脚本必须与 lib_target_verify.py 同目录）"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || {
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
}

ARGS=(verify --python "${PYTHON}")
if [ -n "${INSTALLED_DIR}" ]; then
  ARGS+=(--installed-dir "${INSTALLED_DIR}")
  log "已装版本目录（由脚本位置推断）：${INSTALLED_DIR}"
elif [ -f "${SCRIPT_DIR}/../pyproject.toml" ]; then
  log "仓库开发端模式（未检测到已装版本目录）"
fi
if [ "${#PASSTHROUGH[@]}" -gt 0 ]; then
  ARGS+=("${PASSTHROUGH[@]}")
fi

# `if ! cmd; then rc=$?` 取反后 $? 恒为 0，会吞掉退出码：这里显式保存。
set +e
OUT="$("${PYTHON}" "${LIB}" "${ARGS[@]}")"
rc=$?
set -e
printf '%s\n' "${OUT}"
if [ "${rc}" -ne 0 ]; then
  err "自检未通过（退出码 ${rc}）：详见上面的 JSON 证据（steps / failure）"
  exit "${rc}"
fi
log "自检完成（退出码 0）：演练结果不等于目标端证据，/health 验收仍为 DEFERRED"
exit 0
