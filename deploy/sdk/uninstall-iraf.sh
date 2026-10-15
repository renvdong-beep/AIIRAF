#!/usr/bin/env bash
# 受控卸载入口（IRAF 双域部署 P2b）：**只按安装记录删**本角色的文件，越界一律拒绝。
#
# 硬边界（不得违反）：
#   * 只删安装记录（<版本目录>/<file_record>）里列出的文件，且必须落在声明的安装根之下；
#   * 发现**清单外文件 / 记录内文件缺失 / SHA-256 不符 / 记录属于别的角色** ⇒ 一律拒绝（退出 4），
#     不做"尽力而为"删除；
#   * 数据与日志（声明里的 uninstall.keep）默认**保留**，只有显式 --purge 才删；
#   * 不碰 dpkg/dnf 系统包，不碰 AgentOS 自己的文件（安装边界在声明里固定为禁止）；
#   * 卸载顺序（真机双域）：先智能域（edge-intel，上层）→ 再运控域（edge-motion）。
#     控制域卸载前应确认没有在执行任务；本脚本不代替该状态判断，操作者须先确认。
#
# 用法：
#   bash deploy/sdk/uninstall-iraf.sh --help
#   bash deploy/sdk/uninstall-iraf.sh --profile single-ubuntu2204 --dry-run     # 先看将删清单
#   bash deploy/sdk/uninstall-iraf.sh --profile single-ubuntu2204               # 真卸
#   bash deploy/sdk/uninstall-iraf.sh --profile edge-intel --root <沙箱> --dry-run
#
# 退出码：0 成功/演练完成 / 1 参数错误 / 2 声明失败 / 3 IO 失败 / 4 范围或校验失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_install.py"
PASSTHROUGH=()
DRY_RUN="0"

log() { printf '[uninstall-iraf] %s\n' "$*" >&2; }
err() { printf '[uninstall-iraf][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
受控卸载入口（IRAF 双域部署）

用法：bash uninstall-iraf.sh --profile <角色 id> [选项]

选项：
  -h, --help              显示本帮助并退出（退出码 0）
  --profile <id>          必填：single-ubuntu2204 | sim-x86 | edge-intel | edge-motion
  -n, --dry-run           只打印将删/将保留清单，不删任何文件
  --purge                 连数据与日志一起删（默认保留）
  --force-clean           无安装记录时的兜底清理：只删**声明里列出的**路径（前缀/unit/env），逐项打印。
                          用于"有安装痕迹但记录缺失"的半状态（例如上一次安装中途失败）。
  --root <dir>            文件系统根（默认 /；沙箱演练给沙箱目录）
  --installed-dir <dir>   直接指定已装版本目录
  --json                  输出 JSON
  --repo-root <dir>       仓库根（调试用）

退出码：0 成功/演练完成 / 1 参数错误 / 2 声明失败 / 3 IO 失败 / 4 范围或校验失败

说明：卸载只依据安装记录；记录缺失或树被改动时**拒绝执行**，这是设计而非缺陷。
      先跑 --dry-run 看清清单再真卸。
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -n|--dry-run) DRY_RUN="1"; PASSTHROUGH+=("$1") ;;
    --purge|--json|--force-clean) PASSTHROUGH+=("$1") ;;
    --profile|--root|--installed-dir|--repo-root)
      [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
      PASSTHROUGH+=("$1" "$2"); shift ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || { err "找不到解释器 ${PYTHON}"; exit 2; }

if [ "${DRY_RUN}" = "1" ]; then
  log "模式：演练（只列清单，不删任何文件）"
else
  log "模式：真实卸载（只按安装记录删除）"
fi

# 仓库根通过环境变量传给实现层（避免与调用方传入的 --repo-root 重复导致 argparse 报错）
export IRAF_REPO_ROOT="${REPO_ROOT}"

set +e
"${PYTHON}" "${LIB}" uninstall "${PASSTHROUGH[@]}"
rc=$?
set -e
if [ "${rc}" -eq 0 ]; then
  if [ "${DRY_RUN}" = "1" ]; then
    log "演练完成（退出码 0）：这不代表已卸载；确认清单无误后再去掉 --dry-run"
    log "真卸之后必须跑：bash deploy/sdk/verify_residue.sh --profile <id>"
  else
    log "卸载完成（退出码 0）：请立即跑 verify_residue.sh 做六路残留核验（这是独立证据）"
  fi
else
  err "卸载未执行（退出码 ${rc}）：未删除任何文件，详见上面的中文原因"
fi
exit "${rc}"
