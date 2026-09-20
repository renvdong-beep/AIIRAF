#!/usr/bin/env bash
# 受控卸载入口（步骤 09）：停服务 → 依据安装记录删除本 bundle 的文件 → 保留数据与日志。
#
# 用法：
#   bash deploy/sdk/uninstall.sh --help
#   bash deploy/sdk/uninstall.sh --root <沙箱> --dry-run          # 演练：只做范围比对，不删任何文件
#   bash deploy/sdk/uninstall.sh --root /                          # 目标端真实卸载
#
# 退出码（设计 §5 统一约定）：
#   0 成功/演练完成   1 参数或范围错误（清单外文件、记录缺失、激活链接不符、前缀不符）
#   4 校验失败（文件与安装记录不一致）   5 停服务失败/服务状态未知（未删除任何文件）
#
# 硬边界（不得违反）：
#   * 卸载只依据安装记录（delivery.install.file_record，由 verify.sh 安装后生成）删文件；
#     发现**清单外文件**、记录内文件缺失、SHA-256 不符一律拒绝（退出 1/4），不做"尽力而为"删除。
#   * 日志目录与事件库属数据，卸载保留。
#   * 演练（--dry-run）只比对不删除；沙箱（--root ≠ /）不执行 systemctl。
#   * 安装记录名、路径全部来自 config/sdk/package_matrix.yaml（铁律 5.3），脚本里不写死。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"

PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_target_verify.py"
INSTALLED_DIR=""
PASSTHROUGH=()

usage() {
  cat <<'USAGE'
受控卸载入口（IRAF iraf-24h 步骤 09）

用法：bash uninstall.sh --root <dir> [选项]

选项：
  -h, --help             显示本帮助并退出（退出码 0）
  --root <dir>           文件系统根（必填）：沙箱演练给沙箱目录，目标端给 /。
                         缺失即为参数错误——没有根就无从判断"是否只动本 bundle"。
  -n, --dry-run          演练：只比对安装记录与实际树，不执行 systemctl、不删除文件
  --version <v>          指定待卸载版本（默认取 <prefix>/current 符号链接指向）
  --installed-dir <dir>  直接指定已装版本目录（默认由脚本位置推断）
  --python <exe>         解释器（默认 python3）
  --json-out <path>      额外落一份卸载报告 JSON（本机路径，不入库）

退出码：0 成功/演练完成 / 1 参数或范围错误 / 4 校验失败 / 5 停服务失败或状态未知

说明：本机是 x86_64 开发端、板卡不在场（x86-first）：目标端真实卸载验收 DEFERRED，
      演练结果不得表述为"已在板卡上卸载完成"。
USAGE
}

log() { printf '[uninstall] %s\n' "$*" >&2; }
err() { printf '[uninstall][错误] %s\n' "$*" >&2; }

# 已装版本目录：脚本位于 <version_dir>/scripts/uninstall.sh 时自动推断。
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
    -n|--dry-run)
      PASSTHROUGH+=("$1")
      ;;
    --root|--version|--python|--json-out|--installed-dir)
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

ARGS=(uninstall --python "${PYTHON}")
if [ -n "${INSTALLED_DIR}" ]; then
  ARGS+=(--installed-dir "${INSTALLED_DIR}")
  log "已装版本目录（由脚本位置推断）：${INSTALLED_DIR}"
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
  err "卸载未通过（退出码 ${rc}）：未删除任何文件，详见上面的 JSON 报告"
  exit "${rc}"
fi
log "卸载流程完成（退出码 0）"
exit 0
