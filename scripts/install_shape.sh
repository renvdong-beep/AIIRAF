#!/usr/bin/env bash
# IRAF 安装档位入口 —— 只做"按档位组装参数并串起三步"，不复制任何路径/端口事实。
#
# 四个档位（与 config/install/install_profiles.yaml 的角色一一对应）：
#   x86-all        本机全装（single-ubuntu2204：仿真 + 运行同机；仅 x86_64）
#   intel          板卡只装智控域（edge-intel）
#   motion-split   两域分机：智控域一台 + 运控域另一台（edge-intel / edge-motion）
#   motion-cohost  两域同机：同一块板装 edge-intel + edge-motion（路径已隔离，见声明注释）
#
# 默认只打印计划（dry-run）；真装必须显式 --apply。
# 退出码：0 成功/演练完成 / 1 参数错误 / 2 本机预检失败 / 3 构建或组装失败
#         4 远端安装失败 / 5 后置判定失败（与 board_deploy.sh 保持一致）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON="${PYTHON:-python3}"

SHAPE=""
BOARD="profiles/boards/e300.yaml"
TARGET=""
HOST="${IRAF_BOARD_HOST:-}"
MOTION_HOST="${IRAF_MOTION_HOST:-}"
VALUES=""
ENV_SETS=()
APPLY="0"
EXTRA_BOARD_ARGS=()

log() { printf '[install-shape] %s\n' "$*" >&2; }
err() { printf '[install-shape][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
用法：bash scripts/install_shape.sh --shape <档位> [选项]

必填：
  --shape <id>        x86-all | intel | motion-split | motion-cohost

选项：
  -h, --help          显示本帮助并退出
  --apply             真装（默认只打印计划）
  --board <yaml>      BoardProfile 路径（默认 profiles/boards/e300.yaml）
  --host <ip>         板卡地址（也可用 IRAF_BOARD_HOST）
  --motion-host <ip>  运控域所在机器（motion-split 必填；也可用 IRAF_MOTION_HOST）
  --values <yaml>     部署期取值文件（透传给 board_deploy.sh）
  --env-set KEY=VALUE 部署期取值（可多次，透传）
  --target <id>       构建目标（一般不需要：会按 BoardProfile 推断）

环境变量：SSHPASS（板卡密码，只从环境变量传；x86-all 不需要）

档位对应的角色（来自声明，不在此处重复路径/端口）：
  x86-all        single-ubuntu2204（本机，user scope）
  intel          edge-intel
  motion-split   edge-intel @host + edge-motion @motion-host
  motion-cohost  edge-intel + edge-motion @host
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --shape) [ "$#" -ge 2 ] || { err "--shape 缺少取值"; exit 1; }; SHAPE="$2"; shift ;;
    --board) [ "$#" -ge 2 ] || { err "--board 缺少取值"; exit 1; }; BOARD="$2"; shift ;;
    --host) [ "$#" -ge 2 ] || { err "--host 缺少取值"; exit 1; }; HOST="$2"; shift ;;
    --motion-host) [ "$#" -ge 2 ] || { err "--motion-host 缺少取值"; exit 1; }; MOTION_HOST="$2"; shift ;;
    --values) [ "$#" -ge 2 ] || { err "--values 缺少取值"; exit 1; }; VALUES="$2"; shift ;;
    --env-set) [ "$#" -ge 2 ] || { err "--env-set 缺少取值"; exit 1; }; ENV_SETS+=("$2"); shift ;;
    --target) [ "$#" -ge 2 ] || { err "--target 缺少取值"; exit 1; }; TARGET="$2"; shift ;;
    --apply) APPLY="1" ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

[ -n "${SHAPE}" ] || { err "缺少 --shape"; exit 1; }
case "${SHAPE}" in
  x86-all|intel|motion-split|motion-cohost) ;;
  *) err "未知档位：${SHAPE}（只支持 x86-all / intel / motion-split / motion-cohost）"; exit 1 ;;
esac
if [ "${SHAPE}" != "x86-all" ]; then
  [ -n "${HOST}" ] || { err "缺少 --host（或 IRAF_BOARD_HOST）"; exit 1; }
fi
if [ "${SHAPE}" = "motion-split" ]; then
  [ -n "${MOTION_HOST}" ] || { err "motion-split 需要 --motion-host（运控域所在机器）"; exit 1; }
  [ "${MOTION_HOST}" != "${HOST}" ] || { err "motion-split 的两个地址相同 ⇒ 同机部署请用 motion-cohost"; exit 1; }
fi
if [ "${SHAPE}" != "x86-all" ]; then
  [ -n "${SSHPASS:-}" ] || { err "缺少 SSHPASS（板卡密码；只从环境变量传）"; exit 1; }
fi

# ── 版本与产物路径：全部从声明/矩阵读，不在本脚本里写死 ──────────────────────────
VERSION="$("${PYTHON}" -c "import sys;sys.path.insert(0,'${REPO_ROOT}/deploy/sdk');import lib_manifest as m;from pathlib import Path;print(m.read_declared_version(Path('${REPO_ROOT}'),m.load_matrix(Path('${REPO_ROOT}'),Path('${REPO_ROOT}/config/sdk/package_matrix.yaml'))['sdk']['version_ref']))")"
SDK_WHEEL="${REPO_ROOT}/build/sdk/iraf_sdk-${VERSION}-py3-none-any.whl"
RUNTIME_BUNDLE="${REPO_ROOT}/build/sdk/iraf-runtime-${VERSION}-aarch64.tar.gz"

TARGET_ARGS=()
[ -n "${TARGET}" ] && TARGET_ARGS=(--target "${TARGET}")

# 构建目标 id：优先用 --target，否则按 BoardProfile 推断（与 package_board_bundle 同一套逻辑）
board_target_id() {
  local board_rel="$1"
  if [ -n "${TARGET}" ]; then printf '%s\n' "${TARGET}"; return 0; fi
  "${PYTHON}" -c "
import sys
from pathlib import Path
sys.path.insert(0, '${REPO_ROOT}/deploy/sdk')
import lib_manifest as m
import lib_board_bundle as b
root = Path('${REPO_ROOT}')
matrix = m.load_matrix(root, root / 'config/sdk/package_matrix.yaml')
board = b.read_board_profile(root / '${board_rel}')
print(m.select_target(matrix, None, board_profile=board)['id'])
" 2>/dev/null || { err "按板卡声明推断构建目标失败（可显式给 --target）"; exit 2; }
}

run_step() {
  local label="$1"; shift
  if [ "${APPLY}" = "1" ]; then
    log "▶ ${label}"
    "$@"
  else
    log "（dry-run）${label}：$(printf '%q ' "$@")"
  fi
}

# ── 步骤 1：构建 SDK 产物（幂等；产物存在且比 src 新时跳过由 build_sdk --verify 负责）──
need_build="0"
[ -f "${SDK_WHEEL}" ] && [ -f "${RUNTIME_BUNDLE}" ] || need_build="1"
if [ "${need_build}" = "1" ]; then
  run_step "构建 SDK 与 runtime 包（target=$( [ -n "${TARGET}" ] && echo "${TARGET}" || echo 自动推断 )）" \
    bash "${REPO_ROOT}/deploy/sdk/build_sdk.sh" ${TARGET_ARGS[@]:-}
else
  log "产物已存在：$(basename "${SDK_WHEEL}") / $(basename "${RUNTIME_BUNDLE}")（如需重建请先删除或手动跑 build_sdk.sh）"
fi

# ── 步骤 2：按档位执行 ────────────────────────────────────────────────────────
deploy_role() {
  local role="$1" host="$2"; shift 2
  local args=(--profile "${role}" --sdk-wheel "${SDK_WHEEL}" --runtime-bundle "${RUNTIME_BUNDLE}")
  args+=(--wheelhouse "${REPO_ROOT}/build/wheelhouse/${WHEELHOUSE_TARGET}")
  [ -n "${VALUES}" ] && args+=(--values "${VALUES}")
  for kv in "${ENV_SETS[@]:-}"; do [ -n "${kv}" ] && args+=(--env-set "${kv}"); done
  [ "${APPLY}" = "1" ] && args+=(--apply)
  log "角色 ${role} → ${host}"
  IRAF_BOARD_HOST="${host}" bash "${REPO_ROOT}/deploy/sdk/board_deploy.sh" "${args[@]}"
}

case "${SHAPE}" in
  x86-all)
    # 本机全装：不经过 ssh/board_deploy，直接跑目标端安装器（user scope + 在线索引依赖）。
    log "档位 x86-all：本机（$(uname -m)）安装 single-ubuntu2204（仿真 + 运行同机）"
    x86_args=(--profile single-ubuntu2204 --sdk-wheel "${SDK_WHEEL}" --runtime-bundle "${RUNTIME_BUNDLE}")
    for kv in "${ENV_SETS[@]:-}"; do [ -n "${kv}" ] && x86_args+=(--env-set "${kv}"); done
    run_step "本机安装 single-ubuntu2204" \
      bash "${REPO_ROOT}/deploy/sdk/install-iraf.sh" "${x86_args[@]}"
    ;;
  intel)
    WHEELHOUSE_TARGET="$(board_target_id "${BOARD}")"
    deploy_role edge-intel "${HOST}"
    ;;
  motion-split)
    WHEELHOUSE_TARGET="$(board_target_id "${BOARD}")"
    deploy_role edge-intel "${HOST}"
    deploy_role edge-motion "${MOTION_HOST}"
    ;;
  motion-cohost)
    WHEELHOUSE_TARGET="$(board_target_id "${BOARD}")"
    deploy_role edge-intel "${HOST}"
    deploy_role edge-motion "${HOST}"
    ;;
esac

log "完成（apply=${APPLY}）。验收口径：装得上 / 能干净卸载（uninstall-iraf.sh --dry-run）+ 六路残留核验；"
log "v1 结论一律 simulation=true，不构成真机能力证据。"
