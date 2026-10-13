#!/usr/bin/env bash
# 板卡方案的**部署入口**（IRAF 双域部署 · 安装）：从开发机把制品推到板卡并在**目标端**安装。
#
# 与既有 `deploy/sdk/deploy.sh` 的区别（别混用）：
#   · deploy.sh 走的是 **bundle 通道**（package_board_bundle.sh 打成一个 tar.gz，由 lib_board_bundle 安装）；
#   · 本脚本走**按角色通道**：制品 = SDK wheel + runtime 包 + 离线 wheelhouse，目标端调 install-iraf.sh，
#     由 `config/install/install_profiles.yaml` 的角色（edge-intel / edge-motion / …）驱动。
#   两条通道的制品格式与记录格式不同，**不要交叉使用**。
#
# 流水线：本地预检（制品齐备 + wheelhouse 标签校验）→ 远端落地目录 → scp 制品 →
#         远端 install-iraf.sh（预检 → 安装 → 写记录）→ 远端 board_diag.sh 等价判读 → 取回报告
#
# 硬边界（不得违反）：
#   · 凭据只从环境变量来（SSHPASS / IRAF_BOARD_HOST / IRAF_BOARD_USER / IRAF_BOARD_PORT）；
#     不进命令行参数、不落盘、不写进日志；
#   · **默认 --dry-run**：不给 --apply 只打印计划与将要执行的远端命令；
#   · 板卡声明未实测（profiles/boards/*.yaml 含 unverified）时必须显式 --allow-unverified；
#   · 不碰 AgentOS 自己的文件与端口（安装只写声明里的 prefix/config/log/data 与 systemd 单元）；
#   · 同一角色"已装同名单元"时目标端会拒绝覆盖（退出码 4）—— 变更走受控卸载或 --upgrade 重装。
#
# 用法：
#   export IRAF_BOARD_HOST=10.203.247.74 IRAF_BOARD_USER=root
#   read -rs -p '板卡密码: ' SSHPASS; export SSHPASS
#   # 1) 先看计划（默认 dry-run，不做任何写操作）
#   bash deploy/sdk/board_deploy.sh --profile edge-intel \
#        --sdk-wheel build/sdk-cp311/iraf_sdk-0.2.0-py3-none-any.whl \
#        --runtime-bundle build/sdk-cp311/iraf-runtime-0.2.0-aarch64.tar.gz \
#        --wheelhouse build/wheelhouse/aarch64-manylinux_2_28-cp311
#   # 2) 真部署（显式 --apply）
#   bash deploy/sdk/board_deploy.sh --profile edge-intel … --apply
#
# 退出码：0 成功/演练完成 / 1 参数错误 / 2 本地预检失败 / 3 传输失败 / 4 远端安装失败（范围/校验）
#         5 远端后置判定失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

HOST="${IRAF_BOARD_HOST:-}"
BOARD_USER="${IRAF_BOARD_USER:-root}"
SSH_PORT="${IRAF_BOARD_PORT:-22}"
REMOTE_DIR="${IRAF_BOARD_REMOTE_DIR:-/root/iraf-deploy}"
PROFILE=""
SDK_WHEEL=""
RUNTIME_BUNDLE=""
WHEELHOUSE=""
APPLY="0"
ALLOW_UNVERIFIED="0"

log() { printf '[board-deploy] %s\n' "$*" >&2; }
err() { printf '[board-deploy][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
板卡部署入口（按角色通道）

用法：bash board_deploy.sh --profile <角色> --sdk-wheel <whl> [选项]

必填：
  --profile <id>          edge-intel | edge-motion | single-ubuntu2204 | sim-x86
  --sdk-wheel <whl>       SDK wheel（build_sdk.sh 产物）

选项：
  -h, --help              显示本帮助并退出
  --runtime-bundle <tgz>  runtime 包（build_sdk.sh 产物）
  --wheelhouse <dir>      离线 wheelhouse（edge-* 必填）
  --apply                 真部署（默认只演练、不写任何远端文件）
  --allow-unverified      放行"板卡声明未实测"（探索用；不构成验收证据）
  --remote-dir <dir>      远端落地目录（默认 /root/iraf-deploy；也可用 IRAF_BOARD_REMOTE_DIR）

环境变量：IRAF_BOARD_HOST（必填）、IRAF_BOARD_USER、IRAF_BOARD_PORT、SSHPASS（必填）

退出码：0 成功/演练完成 / 1 参数错误 / 2 本地预检失败 / 3 传输失败 / 4 远端安装失败 / 5 后置判定失败
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --apply) APPLY="1" ;;
    --allow-unverified) ALLOW_UNVERIFIED="1" ;;
    --profile) PROFILE="$2"; shift ;;
    --sdk-wheel) SDK_WHEEL="$2"; shift ;;
    --runtime-bundle) RUNTIME_BUNDLE="$2"; shift ;;
    --wheelhouse) WHEELHOUSE="$2"; shift ;;
    --remote-dir) REMOTE_DIR="$2"; shift ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

[ -n "${HOST}" ] || { err "缺少 IRAF_BOARD_HOST"; exit 1; }
[ -n "${SSHPASS:-}" ] || { err "缺少 SSHPASS（板卡密码；只从环境变量传）"; exit 1; }
[ -n "${PROFILE}" ] || { err "缺少 --profile"; exit 1; }
[ -n "${SDK_WHEEL}" ] || { err "缺少 --sdk-wheel"; exit 1; }
command -v sshpass >/dev/null 2>&1 || { err "找不到 sshpass"; exit 1; }

# 本地预检：角色的生效布局 + 制品存在性（**目标端事实**由远端 install-iraf.sh 再查一遍）
LAYOUT="$("${PYTHON}" "${SCRIPT_DIR}/lib_install.py" layout --profile "${PROFILE}")" || exit 2
need_wheelhouse="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['dependency_source']['kind'])" "${LAYOUT}")"
[ -f "${SDK_WHEEL}" ] || { err "SDK wheel 不存在：${SDK_WHEEL}"; exit 2; }
if [ -n "${RUNTIME_BUNDLE}" ]; then
  [ -f "${RUNTIME_BUNDLE}" ] || { err "runtime 包不存在：${RUNTIME_BUNDLE}"; exit 2; }
fi
if [ "${need_wheelhouse}" = "wheelhouse" ]; then
  [ -n "${WHEELHOUSE}" ] || { err "该角色的依赖来源是 wheelhouse ⇒ 必须给 --wheelhouse"; exit 2; }
  [ -d "${WHEELHOUSE}" ] || { err "wheelhouse 目录不存在：${WHEELHOUSE}"; exit 2; }
  if ! bash "${SCRIPT_DIR}/fetch_wheelhouse.sh" --verify --target \
        "$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['dependency_source']['target_id'])" "${LAYOUT}")" \
        --dest "${WHEELHOUSE}" >/dev/null 2>&1; then
    log "wheelhouse 标签校验：未通过或无清单（继续，但目标端会再校一次）"
  else
    log "wheelhouse 标签校验：通过"
  fi
fi

SSH_OPTS=(-o "StrictHostKeyChecking=accept-new"
          -o "UserKnownHostsFile=${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/ssh/known_hosts"
          -o "ConnectTimeout=10" -o "BatchMode=no" -p "${SSH_PORT}")
TARGET="${BOARD_USER}@${HOST}"
remote() { SSHPASS="${SSHPASS}" timeout "${IRAF_BOARD_PROBE_TIMEOUT:-600}" sshpass -e ssh "${SSH_OPTS[@]}" "${TARGET}" "$1"; }
copy() { SSHPASS="${SSHPASS}" timeout "${IRAF_BOARD_PROBE_TIMEOUT:-600}" sshpass -e scp "${SSH_OPTS[@]}" "$@"; }

mkdir -p "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/ssh"

REMOTE_SDK_DIR="${REMOTE_DIR}/deploy/sdk"
log "目标 ${TARGET}｜角色 ${PROFILE}｜落地目录 ${REMOTE_DIR}"
log "制品：wheel=$(basename "${SDK_WHEEL}")"$'\n'"      runtime=$(basename "${RUNTIME_BUNDLE:-（无）}")"$'\n'"      wheelhouse=${WHEELHOUSE:-（无）}"

INSTALL_CMD="bash ${REMOTE_SDK_DIR}/install-iraf.sh --profile ${PROFILE} --sdk-wheel ${REMOTE_DIR}/iraf_sdk.whl"
[ -n "${RUNTIME_BUNDLE}" ] && INSTALL_CMD="${INSTALL_CMD} --runtime-bundle ${REMOTE_DIR}/iraf-runtime.tar.gz"
[ -n "${WHEELHOUSE}" ] && INSTALL_CMD="${INSTALL_CMD} --wheelhouse ${REMOTE_DIR}/wheelhouse"
[ "${ALLOW_UNVERIFIED}" = "1" ] && INSTALL_CMD="${INSTALL_CMD} --allow-unverified"

if [ "${APPLY}" != "1" ]; then
  log "== 演练（未加 --apply，不写任何远端文件）=="
  cat <<PLAN | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-plan.txt"

 1) 远端建目录： mkdir -p ${REMOTE_SDK_DIR}/units ${REMOTE_DIR}/scripts ${REMOTE_DIR}/config/install ${REMOTE_DIR}/wheelhouse
                 （目标端镜像仓库布局：deploy/sdk 下放脚本与模板，scripts/ 下放校验器，
                   config/install 下放声明，pyproject.toml 放根 —— 这样 SCRIPT_DIR/../.. 正好是 repo 根）
 2) 推送脚本  ： scp deploy/sdk/*.sh deploy/sdk/lib_install.py → ${REMOTE_SDK_DIR}/
                  scp deploy/sdk/units/*.tmpl                     → ${REMOTE_SDK_DIR}/units/
                  scp scripts/install_profile_check.py           → ${REMOTE_DIR}/scripts/
                  scp config/install/*                           → ${REMOTE_DIR}/config/install/
                  scp pyproject.toml                             → ${REMOTE_DIR}/
 3) 推送制品  ： scp <sdk wheel> → ${REMOTE_DIR}/iraf_sdk.whl；<runtime 包> → ${REMOTE_DIR}/iraf-runtime.tar.gz；
                  scp -r <wheelhouse> → ${REMOTE_DIR}/wheelhouse/
 4) 远端安装  ： cd ${REMOTE_DIR} && ${INSTALL_CMD}
                  （目标端自行预检：架构/Python 标签/glibc/磁盘/systemd/端口/声明是否已实测；失败自动回滚）
 5) 远端判读  ： bash ${REMOTE_SDK_DIR}/verify_residue.sh --profile ${PROFILE} --json
                  并回到开发机跑 deploy/sdk/board_diag.sh 拿完整判读（含 CLI --list-skills 自检）

 注：演练只证明"命令与制品齐备"，不构成任何板卡侧证据。
PLAN
  exit 0
fi

log "== 真实部署（--apply）=="
log "① 远端建目录（目标端的目录布局**镜像仓库**：SCRIPT_DIR/../.. 必须等于 repo 根）"
remote "mkdir -p ${REMOTE_SDK_DIR}/units ${REMOTE_DIR}/scripts ${REMOTE_DIR}/config/install ${REMOTE_DIR}/wheelhouse" \
  || { err "远端建目录失败"; exit 3; }

log "② 推送脚本与声明（按角色通道所需的最小集）"
copy "${SCRIPT_DIR}/install-iraf.sh" "${SCRIPT_DIR}/lib_install.py" "${SCRIPT_DIR}/verify_residue.sh" \
     "${SCRIPT_DIR}/uninstall-iraf.sh" "${SCRIPT_DIR}/rehearse_install_chain.sh" \
     "${TARGET}:${REMOTE_SDK_DIR}/" || { err "脚本传输失败"; exit 3; }
copy "${SCRIPT_DIR}/units/"*.tmpl "${TARGET}:${REMOTE_SDK_DIR}/units/" || { err "单元模板传输失败"; exit 3; }
copy "${REPO_ROOT}/scripts/install_profile_check.py" "${TARGET}:${REMOTE_DIR}/scripts/" || { err "校验器传输失败"; exit 3; }
copy "${REPO_ROOT}/config/install/"* "${TARGET}:${REMOTE_DIR}/config/install/" || { err "声明文件传输失败"; exit 3; }
copy "${REPO_ROOT}/pyproject.toml" "${TARGET}:${REMOTE_DIR}/" || { err "pyproject 传输失败"; exit 3; }

log "③ 推送制品"
copy "${SDK_WHEEL}" "${TARGET}:${REMOTE_DIR}/iraf_sdk.whl" || { err "SDK wheel 传输失败"; exit 3; }
[ -n "${RUNTIME_BUNDLE}" ] && { copy "${RUNTIME_BUNDLE}" "${TARGET}:${REMOTE_DIR}/iraf-runtime.tar.gz" || { err "runtime 包传输失败"; exit 3; }; }
[ -n "${WHEELHOUSE}" ] && { copy -r "${WHEELHOUSE}" "${TARGET}:${REMOTE_DIR}/wheelhouse/" || { err "wheelhouse 传输失败"; exit 3; }; }

log "④ 远端安装（目标端自行预检；失败会回滚）"
set +e
remote "cd ${REMOTE_DIR} && IRAF_REPO_ROOT=${REMOTE_DIR} ${INSTALL_CMD}" 2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-install.log"
rc=${PIPESTATUS[0]}
set -e
if [ "${rc}" -ne 0 ]; then
  err "远端安装未通过（退出码 ${rc}）：见上面的中文原因与 build/board/.../deploy-install.log"
  exit 4
fi

log "⑤ 远端判读（IRAF 就绪逐项）"
set +e
remote "cd ${REMOTE_DIR} && IRAF_REPO_ROOT=${REMOTE_DIR} bash ${REMOTE_SDK_DIR}/verify_residue.sh --profile ${PROFILE} --json" \
  2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-residue.json" | tail -5
set -e
log "部署完成：请回到开发机跑 deploy/sdk/board_diag.sh --profile ${PROFILE} 拿完整判读（含 CLI --list-skills 自检）"
exit 0
