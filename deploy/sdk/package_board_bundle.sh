#!/usr/bin/env bash
# 板级 bundle 组装入口（x86 开发端；设计 §4 产物 3、§5 脚本契约、步骤 08）
#
# 职责：读声明矩阵与 BoardProfile → 过板卡声明门禁（复用 profile_check.py）→ 组装
#       BoardProfile + runtime bundle + SDK wheel + 离线 wheelhouse + 脚本 + systemd 单元
#       + env 模板 → 写 bundle 与伴随 SHA-256。
#
# 用法：
#   bash deploy/sdk/package_board_bundle.sh --help
#   bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml
#       → 声明含 unverified 且未加 --allow-unverified：**默认拒绝**，退出码 2
#   bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml --allow-unverified
#   bash deploy/sdk/package_board_bundle.sh --board profiles/boards/e300.yaml --allow-unverified --dry-run
#
# 退出码（设计 §5 统一约定）：0 成功 / 1 参数错误 / 2 预检失败（声明缺失、未实测、输入产物缺失）
#                             3 构建失败 / 4 校验失败（wheelhouse 标签、bundle 清单被改）
#
# 声明来源（禁止在本脚本硬编码版本、标签、包清单、镜像源、安装路径）：config/sdk/package_matrix.yaml
# 实现层：deploy/sdk/lib_board_bundle.py
# 解释器：默认取 PATH 中的 python3（需要 PyYAML + jsonschema）；可用 PYTHON=/usr/bin/python3 覆盖。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
REPO_ROOT_DEFAULT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON="${PYTHON:-python3}"
REPO_ROOT="${REPO_ROOT_DEFAULT}"
LIB="${SCRIPT_DIR}/lib_board_bundle.py"
BOARD=""
ALLOW_UNVERIFIED="0"
DRY_RUN="0"
EXTRA=()

usage() {
  cat <<'USAGE'
板级 bundle 组装入口（x86 开发端；IRAF iraf-24h 步骤 08）

用法：bash deploy/sdk/package_board_bundle.sh --board <BoardProfile 路径> [选项]

选项：
  -h, --help             显示本帮助并退出（退出码 0）
  -n, --dry-run          只打印组装计划，不写任何文件
  --board <path>         BoardProfile（必填），如 profiles/boards/e300.yaml
  --allow-unverified     显式放行"声明未实测"状态并在产物清单里记录（不放行契约层失败）
  --wheelhouse <dir>     离线 wheelhouse 目录（默认 build/wheelhouse/<target-id>）
  --runtime <path>       runtime bundle（默认 build/sdk/iraf-runtime-<version>-<arch>.tar.gz）
  --sdk-wheel <path>     SDK wheel（默认 build/sdk/<dist>-<version>-py3-none-any.whl）
  --out <dir>            产物输出目录（默认 build/sdk，已 gitignore）
  --matrix <path>        产物矩阵（默认 config/sdk/package_matrix.yaml）
  --target <id>          构建目标 id（矩阵声明多个目标时必填）
  --python <exe>         解释器（默认 python3，用于读声明与调用 profile_check）
  --json-out <path>      组装摘要 JSON 落盘路径
  --repo-root <dir>      仓库根目录（默认：本脚本上两级）

环境变量：PYTHON、REPO_ROOT

退出码：0 成功 / 1 参数错误 / 2 预检失败（声明未实测、输入产物缺失、wheelhouse 缺声明）
        3 构建失败 / 4 校验失败（wheelhouse 平台标签不符、bundle 清单含绝对路径）

说明：bundle 内的 BoardProfile 未实测时（status=unverified）默认拒绝组装；--allow-unverified
      只放行这一类，且会把 allow_unverified_at_build 写进 bundle 清单。
      目标端安装验收 DEFERRED（板卡不在场）：本脚本不产生任何目标端证据。
USAGE
}

log() { printf '[package_board_bundle] %s\n' "$*"; }
err() { printf '[package_board_bundle][错误] %s\n' "$*" >&2; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -n|--dry-run)
      DRY_RUN="1"
      ;;
    --allow-unverified)
      ALLOW_UNVERIFIED="1"
      ;;
    --board)
      [ "$#" -ge 2 ] || { err "--board 缺少取值"; exit 1; }
      BOARD="$2"; shift
      ;;
    --repo-root)
      [ "$#" -ge 2 ] || { err "--repo-root 缺少取值"; exit 1; }
      REPO_ROOT="$2"; shift
      ;;
    *)
      # 其余取值型参数直接转发给实现层，避免在两处各写一份校验。
      case "$1" in
        --wheelhouse|--runtime|--sdk-wheel|--out|--matrix|--target|--python|--json-out)
          [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
          EXTRA+=("$1" "$2"); shift
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

if [ -z "${BOARD}" ]; then
  err "缺少 --board（BoardProfile 路径），例如 profiles/boards/e300.yaml"
  exit 1
fi
[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || {
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
}

ARGS=(--repo-root "${REPO_ROOT}" --board "${BOARD}" --python "${PYTHON}")
if [ "${ALLOW_UNVERIFIED}" = "1" ]; then ARGS+=(--allow-unverified); fi
if [ "${#EXTRA[@]}" -gt 0 ]; then ARGS+=("${EXTRA[@]}"); fi

cd "${REPO_ROOT}"
if [ "${DRY_RUN}" = "1" ]; then
  log "模式：dry-run（只打印计划，不写任何文件）"
  exec "${PYTHON}" "${LIB}" plan "${ARGS[@]}"
fi

log "模式：build（bundle 写入 build/sdk，该目录已 gitignore，属证据区）"
# 注意（踩过的坑）：不能用 `if ! cmd; then rc=$?` —— 取反后的 $? 恒为 0，退出码会被吞掉。
set +e
"${PYTHON}" "${LIB}" build "${ARGS[@]}"
rc=$?
set -e
if [ "${rc}" -ne 0 ]; then
  err "组装失败（退出码 ${rc}）"
  exit "${rc}"
fi
log "组装完成（退出码 0）"
exit 0
