#!/usr/bin/env bash
# 安装入口（IRAF 双域部署 P2b）：按**声明**把 IRAF 装到一个角色上，失败自动回滚本次改动。
#
# 流水线（全部由 config/install/install_profiles.yaml 的生效值驱动，脚本内不写死路径/端口）：
#   预检（架构 / Python 标签 / glibc / OS / 磁盘 / systemd / 端口空闲 / 声明是否已实测 / wheelhouse）
#   → 版本化目录 <prefix>/<version> → pip 装 SDK wheel（离线 wheelhouse 或声明索引）
#   → 解 runtime 源码包（可选）→ 渲染 env 文件 → 渲染 systemd 单元 → 切换 current 链接
#   → 写安装记录（逐文件 SHA-256，含版本目录外的配置文件与新建目录）→ 后置自检（import iraf_sdk）
#
# 硬边界（不得违反）：
#   * 失败即**回滚本次创建的内容**并还原 current 链接；不碰既有安装与其它角色的文件；
#   * 已存在同名 systemd 单元时**拒绝覆盖**（退出 4）—— 变更要走受控卸载/重装流程；
#   * 声明里未实测的目标端字段（板卡 os/kernel/python_tag/platform_tag/glibc_min）
#     必须显式 --allow-unverified 才放行；放行不构成验收证据（AGENTS 3.2 / 6.4）；
#   * 真机路径不得加载仿真适配器：本脚本只装声明里 components 列出的组件。
#
# 用法：
#   bash deploy/sdk/install-iraf.sh --help
#   # 演练（只跑预检 + 打印计划，不写任何文件）
#   bash deploy/sdk/install-iraf.sh --profile edge-intel --root <沙箱> \
#        --sdk-wheel build/sdk/iraf_sdk-0.2.0-py3-none-any.whl \
#        --wheelhouse build/wheelhouse/aarch64-manylinux_2_28-cp311 --dry-run
#   # 真装
#   bash deploy/sdk/install-iraf.sh --profile single-ubuntu2204 \
#        --sdk-wheel build/sdk/iraf_sdk-0.2.0-py3-none-any.whl \
#        --runtime-bundle build/sdk/iraf-runtime-0.2.0-aarch64.tar.gz
#
# 退出码（设计 §5 统一约定）：0 成功/演练完成 / 1 参数错误 / 2 预检失败 / 3 安装 IO 失败
#                              4 范围或校验失败 / 5 后置自检失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_install.py"
PASSTHROUGH=()
DRY_RUN="0"

log() { printf '[install-iraf] %s\n' "$*" >&2; }
err() { printf '[install-iraf][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
安装入口（IRAF 双域部署）

用法：bash install-iraf.sh --profile <角色 id> --sdk-wheel <wheel> [选项]

必填：
  --profile <id>          single-ubuntu2204 | sim-x86 | edge-intel | edge-motion
  --sdk-wheel <whl>       SDK wheel（由 deploy/sdk/build_sdk.sh 产出）

选项：
  -h, --help              显示本帮助并退出（退出码 0）
  -n, --dry-run           只跑预检并打印计划，不写任何文件
  --root <dir>            文件系统根（默认 /；沙箱演练给沙箱目录）
  --runtime-bundle <tgz>  runtime 源码包（可选；由 build_sdk.sh 产出）
  --wheelhouse <dir>      离线 wheelhouse 目录（依赖来源为 wheelhouse 时必填）
  --python <exe>          目标解释器（默认取声明的 python_bin）
  --run-user <user>       单元运行用户（默认取声明的 run_user；@current@ 表示调用者）
  --allow-unverified      显式放行"声明未实测"（探索用，不构成验收证据）
  --json                  输出 JSON 计划/结果
  --repo-root <dir>       仓库根（调试用）

退出码：0 成功/演练完成 / 1 参数错误 / 2 预检失败 / 3 IO 失败 / 4 范围或校验失败 / 5 后置自检失败

说明：预检未过一律不写系统目录；安装中途失败会回滚本次创建的内容并还原 current 链接。
      装完请跑 verify_residue.sh 看"装了什么"，要验证可干净清除请跑 rehearse_install_chain.sh。
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -n|--dry-run) DRY_RUN="1"; PASSTHROUGH+=("$1") ;;
    --allow-unverified|--rehearsal|--json) PASSTHROUGH+=("$1") ;;
    --profile|--root|--sdk-wheel|--runtime-bundle|--wheelhouse|--python|--run-user|--repo-root)
      [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
      PASSTHROUGH+=("$1" "$2"); shift ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

case " ${PASSTHROUGH[*]} " in
  *" --profile "*) ;;
  *) err "缺少 --profile（角色 id）"; exit 1 ;;
esac
case " ${PASSTHROUGH[*]} " in
  *" --sdk-wheel "*) ;;
  *) err "缺少 --sdk-wheel（SDK wheel 路径；先用 deploy/sdk/build_sdk.sh 产出）"; exit 1 ;;
esac

[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || { err "找不到解释器 ${PYTHON}"; exit 2; }

export IRAF_REPO_ROOT="${REPO_ROOT}"

if [ "${DRY_RUN}" = "1" ]; then
  log "模式：演练（只跑预检与计划，不写任何文件）"
else
  log "模式：真实安装（失败自动回滚）"
fi

set +e
"${PYTHON}" "${LIB}" install "${PASSTHROUGH[@]}"
rc=$?
set -e
if [ "${rc}" -eq 0 ]; then
  if [ "${DRY_RUN}" = "1" ]; then
    log "演练完成（退出码 0）：这不代表已安装"
  else
    log "安装完成（退出码 0）：请接着跑"
    log "  ① bash deploy/sdk/verify_residue.sh --profile <id>      # 看装了什么（此时应报不干净）"
    log "  ② bash deploy/sdk/uninstall-iraf.sh --profile <id> --dry-run   # 看将删清单"
  fi
else
  err "安装未完成（退出码 ${rc}）：未安装或已回滚，详见上面的中文原因"
fi
exit "${rc}"
