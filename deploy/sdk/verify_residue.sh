#!/usr/bin/env bash
# 残留核验入口（IRAF 双域部署 P2b）：卸载后**独立**回答"IRAF 在这台机器上还剩什么"。
#
# 为什么要有独立入口：卸载脚本自己说"我删干净了"不是证据。本入口不信任任何自述，
# 直接按声明（config/install/install_profiles.yaml 的 residue 判据）做**六路只读扫描**：
#   ① 文件 ② 空目录 ③ systemd 单元（含 enable 软链）④ env 文件 ⑤ 监听端口 ⑥ 相关进程
# 六路全 0 才 exit 0；任一非 0 即 exit 4 并逐项列出路径（不"尽力而为"清理）。
#
# 用法：
#   bash deploy/sdk/verify_residue.sh --help
#   bash deploy/sdk/verify_residue.sh --profile single-ubuntu2204                 # 真机/本机
#   bash deploy/sdk/verify_residue.sh --profile sim-x86 --root <沙箱目录>          # 沙箱演练
#   bash deploy/sdk/verify_residue.sh --profile edge-intel --json
#
# 退出码：0 干净（六路全 0）/ 1 参数错误 / 2 声明失败 / 4 有残留或不一致
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_install.py"
PASSTHROUGH=()

log() { printf '[verify-residue] %s\n' "$*" >&2; }
err() { printf '[verify-residue][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
残留核验入口（IRAF 双域部署）

用法：bash verify_residue.sh --profile <角色 id> [选项]

选项：
  -h, --help              显示本帮助并退出（退出码 0）
  --profile <id>          必填：single-ubuntu2204 | sim-x86 | edge-intel | edge-motion
  --root <dir>            文件系统根（默认 /；沙箱演练给沙箱目录）
  --installed-dir <dir>   直接指定已装版本目录（默认按 current 链接解析）
  --json                  输出 JSON（供机器判读）
  --repo-root <dir>       仓库根（调试用）

退出码：0 干净 / 1 参数错误 / 2 声明失败 / 4 有残留或记录不一致

说明：本脚本**只读**（不删任何文件）；沙箱演练不查 systemd 服务状态，只看单元文件是否存在。
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --profile|--root|--installed-dir|--repo-root)
      [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
      PASSTHROUGH+=("$1" "$2"); shift ;;
    --json) PASSTHROUGH+=("$1") ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || { err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"; exit 2; }

# 仓库根通过环境变量传给实现层（避免与调用方传入的 --repo-root 重复导致 argparse 报错）
export IRAF_REPO_ROOT="${REPO_ROOT}"

set +e
"${PYTHON}" "${LIB}" residue "${PASSTHROUGH[@]}"
rc=$?
set -e
if [ "${rc}" -eq 0 ]; then
  log "残留核验通过：六路全 0"
else
  err "残留核验未通过（退出码 ${rc}）：见上面的逐项清单"
fi
exit "${rc}"
