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
REHEARSAL="0"
REMOTE_ROOT=""
UNINSTALL="0"
PURGE="0"
FORCE_CLEAN="0"

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
  --rehearsal             目标端以**沙箱演练**方式安装（放宽目标端事实检查）；必须配 --remote-root 且非 /
                          用途：替身主机/本地管线上端到端自测。**不构成板卡证据**
  --remote-root <dir>     传给远端 install-iraf.sh 的 --root（默认不传 = 目标端真实根 /）
  --uninstall             只做**受控卸载**：推脚本 → 远端 uninstall-iraf.sh → 残留核验（应六路全 0）
  --purge                 配合 --uninstall：连数据与日志一起删（默认保留）
  --force-clean           配合 --uninstall：无安装记录时的兜底清理（只删声明路径，逐项打印）
  --force-clean           配合 --uninstall：无安装记录时的兜底清理（只删声明路径，逐项打印）

环境变量：IRAF_BOARD_HOST（必填）、IRAF_BOARD_USER、IRAF_BOARD_PORT、SSHPASS（必填）

退出码：0 成功/演练完成 / 1 参数错误 / 2 本地预检失败 / 3 传输失败 / 4 远端安装失败 / 5 后置判定失败
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --apply) APPLY="1" ;;
    --uninstall) UNINSTALL="1" ;;
    --purge) PURGE="1" ;;
    --force-clean) FORCE_CLEAN="1" ;;
    --allow-unverified) ALLOW_UNVERIFIED="1" ;;
    --rehearsal) REHEARSAL="1" ;;
    --remote-root) REMOTE_ROOT="$2"; shift ;;
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
if [ "${UNINSTALL}" != "1" ]; then
  [ -n "${SDK_WHEEL}" ] || { err "缺少 --sdk-wheel（安装/演练必需；--uninstall 时不需要）"; exit 1; }
fi
# 演练语义与目标端 install-iraf.sh 保持一致：必须沙箱根 + 放行未实测
if [ "${REHEARSAL}" = "1" ]; then
  [ -n "${REMOTE_ROOT}" ] || { err "--rehearsal 必须同时给 --remote-root <沙箱目录>（不得在目标端真实根上演练）"; exit 1; }
  [ "${REMOTE_ROOT}" != "/" ] || { err "--rehearsal 的 --remote-root 不得是 /"; exit 1; }
  [ "${ALLOW_UNVERIFIED}" = "1" ] || { err "--rehearsal 会放宽目标端事实检查 ⇒ 必须同时给 --allow-unverified"; exit 1; }
  log "模式：**替身/沙箱演练**（目标端 --root=${REMOTE_ROOT}）；结果不构成板卡证据"
fi
command -v sshpass >/dev/null 2>&1 || { err "找不到 sshpass"; exit 1; }

# 本地预检：角色的生效布局 + 制品存在性（**目标端事实**由远端 install-iraf.sh 再查一遍）
LAYOUT="$("${PYTHON}" "${SCRIPT_DIR}/lib_install.py" layout --profile "${PROFILE}")" || exit 2
need_wheelhouse="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['dependency_source']['kind'])" "${LAYOUT}")"
if [ "${UNINSTALL}" != "1" ]; then   # 卸载不需要制品，跳过制品预检
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
fi   # end if UNINSTALL != 1

# ssh/scp 的端口选项**不同**：ssh 用 -p，scp 用 -P（实测踩到：给 scp 传 -p 会被当成"保留时间戳"，
# 端口变成文件名 → 报 "22: No such file or directory"）。因此基础选项与端口分开拼。
SSH_OPTS=(-o "StrictHostKeyChecking=accept-new"
          -o "UserKnownHostsFile=${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/ssh/known_hosts"
          -o "ConnectTimeout=10" -o "BatchMode=no")
SSH_OPTS+=(-p "${SSH_PORT}")
SCP_OPTS=(-o "StrictHostKeyChecking=accept-new"
          -o "UserKnownHostsFile=${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/ssh/known_hosts"
          -o "ConnectTimeout=10" -o "BatchMode=no")
SCP_OPTS+=(-P "${SSH_PORT}")
TARGET="${BOARD_USER}@${HOST}"
remote() { SSHPASS="${SSHPASS}" timeout "${IRAF_BOARD_PROBE_TIMEOUT:-600}" sshpass -e ssh "${SSH_OPTS[@]}" "${TARGET}" "$1"; }
copy() { SSHPASS="${SSHPASS}" timeout "${IRAF_BOARD_PROBE_TIMEOUT:-600}" sshpass -e scp "${SCP_OPTS[@]}" "$@"; }

mkdir -p "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/ssh"

REMOTE_SDK_DIR="${REMOTE_DIR}/deploy/sdk"
log "目标 ${TARGET}｜角色 ${PROFILE}｜落地目录 ${REMOTE_DIR}"
log "制品：wheel=$(basename "${SDK_WHEEL}")"$'\n'"      runtime=$(basename "${RUNTIME_BUNDLE:-（无）}")"$'\n'"      wheelhouse=${WHEELHOUSE:-（无）}"

SDK_WHEEL_REMOTE="${REMOTE_DIR}/artifacts/$(basename "${SDK_WHEEL}")"
RUNTIME_REMOTE="${RUNTIME_BUNDLE:+${REMOTE_DIR}/artifacts/$(basename "${RUNTIME_BUNDLE}")}"
INSTALL_CMD="bash ${REMOTE_SDK_DIR}/install-iraf.sh --profile ${PROFILE} --sdk-wheel ${SDK_WHEEL_REMOTE}"
[ -n "${RUNTIME_REMOTE}" ] && INSTALL_CMD="${INSTALL_CMD} --runtime-bundle ${RUNTIME_REMOTE}"
[ -n "${WHEELHOUSE}" ] && INSTALL_CMD="${INSTALL_CMD} --wheelhouse ${REMOTE_DIR}/wheelhouse"
[ "${ALLOW_UNVERIFIED}" = "1" ] && INSTALL_CMD="${INSTALL_CMD} --allow-unverified"
[ "${REHEARSAL}" = "1" ] && INSTALL_CMD="${INSTALL_CMD} --rehearsal --root ${REMOTE_ROOT}"
RESIDUE_CMD="bash ${REMOTE_SDK_DIR}/verify_residue.sh --profile ${PROFILE} --json"
UNINSTALL_CMD="bash ${REMOTE_SDK_DIR}/uninstall-iraf.sh --profile ${PROFILE}"
FORCE_CLEAN_ARG=""; [ "${FORCE_CLEAN}" = "1" ] && FORCE_CLEAN_ARG=" --force-clean"
[ -n "${REMOTE_ROOT}" ] && { RESIDUE_CMD="${RESIDUE_CMD} --root ${REMOTE_ROOT}"; UNINSTALL_CMD="${UNINSTALL_CMD} --root ${REMOTE_ROOT}"; }

if [ "${UNINSTALL}" = "1" ]; then
  log "== 受控卸载（只按安装记录删；清单外文件即拒绝）=="
  remote "mkdir -p ${REMOTE_SDK_DIR}/units ${REMOTE_DIR}/scripts ${REMOTE_DIR}/config/install" \
    || { err "远端目录准备失败"; exit 3; }
  copy "${SCRIPT_DIR}/lib_install.py" "${SCRIPT_DIR}/uninstall-iraf.sh" "${SCRIPT_DIR}/verify_residue.sh" \
       "${TARGET}:${REMOTE_SDK_DIR}/" || { err "脚本传输失败"; exit 3; }
  copy "${REPO_ROOT}/scripts/install_profile_check.py" "${TARGET}:${REMOTE_DIR}/scripts/" || { err "校验器传输失败"; exit 3; }
  copy "${REPO_ROOT}/config/install/"* "${TARGET}:${REMOTE_DIR}/config/install/" || { err "声明传输失败"; exit 3; }
  copy "${REPO_ROOT}/pyproject.toml" "${TARGET}:${REMOTE_DIR}/" || { err "pyproject 传输失败"; exit 3; }
# proto 桩：声明里的 python_stubs_dir 是**仓库相对路径**（build/generated/python），
# 目标端 repo 根 = REMOTE_DIR，所以必须推到同样的相对位置，安装器才能按声明找到并拷进 lib/

  log "① 卸载演练（--dry-run：只看将删清单，不删任何文件）"
  set +e
  remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} bash ${REMOTE_SDK_DIR}/uninstall-iraf.sh --profile ${PROFILE} --root ${REMOTE_ROOT:-/}${FORCE_CLEAN_ARG} --dry-run" \
    2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/uninstall-plan.log" | tail -8
  plan_rc=${PIPESTATUS[0]}
  set -e

  log "② 真卸载"
  set +e
  remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} bash ${REMOTE_SDK_DIR}/uninstall-iraf.sh --profile ${PROFILE} --root ${REMOTE_ROOT:-/}$([ \"${PURGE}\" = \"1\" ] && echo ' --purge')${FORCE_CLEAN_ARG}" \
    2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/uninstall.log" | tail -6
  rc=${PIPESTATUS[0]}
  set -e
  if [ "${rc}" -ne 0 ]; then err "卸载未执行（退出码 ${rc}）：未删除任何文件"; exit 4; fi

  log "③ 残留核验（期望：六路全 0）"
  set +e
  remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} bash ${REMOTE_SDK_DIR}/verify_residue.sh --profile ${PROFILE} --root ${REMOTE_ROOT:-/} --json" \
    > "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/residue-after-uninstall.json" 2>&1
  res_rc=$?
  set -e
  if [ "${res_rc}" -eq 0 ]; then
    log "  ✓ 六路残留全 0（IRAF 已从该角色清除；数据与日志按声明保留）"
  else
    err "  ✗ 仍有残留（rc=${res_rc}）：见 build/board/.../residue-after-uninstall.json"
  fi
  log "卸载流程完成（演练退出码 ${plan_rc} / 真卸 ${rc} / 残留核验 ${res_rc}）"
  exit $([ "${res_rc}" -eq 0 ] && echo 0 || echo 4)
fi

if [ "${APPLY}" != "1" ]; then
  log "== 演练（未加 --apply，不写任何远端文件）=="
  cat <<PLAN | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-plan.txt"

 1) 远端建目录： mkdir -p ${REMOTE_SDK_DIR}/units ${REMOTE_DIR}/scripts ${REMOTE_DIR}/config/install \
                          ${REMOTE_DIR}/wheelhouse ${REMOTE_DIR}/artifacts
                 （目标端镜像仓库布局：deploy/sdk 下放脚本与模板，scripts/ 下放校验器，
                   config/install 下放声明，pyproject.toml 放根 —— 这样 SCRIPT_DIR/../.. 正好是 repo 根）
 2) 推送脚本  ： scp deploy/sdk/*.sh deploy/sdk/lib_install.py → ${REMOTE_SDK_DIR}/
                  scp deploy/sdk/units/*.tmpl                     → ${REMOTE_SDK_DIR}/units/
                  scp scripts/install_profile_check.py           → ${REMOTE_DIR}/scripts/
                  scp config/install/*                           → ${REMOTE_DIR}/config/install/
                  scp pyproject.toml                             → ${REMOTE_DIR}/
 3) 推送制品  ： scp <sdk wheel> → ${REMOTE_DIR}/artifacts/$(basename "${SDK_WHEEL}")
                  scp <runtime 包> → ${REMOTE_DIR}/artifacts/$(basename "${RUNTIME_BUNDLE:-<无>}")
                  scp <wheelhouse 内容> → ${REMOTE_DIR}/wheelhouse/
                  ⚠ 制品**保持原始文件名**：pip 按文件名做 PEP 427 校验，改名会被拒
 4) 远端安装  ： cd ${REMOTE_DIR} && ${INSTALL_CMD}
                  （目标端自行预检：架构/Python 标签/glibc/磁盘/systemd/端口/声明是否已实测；失败自动回滚）
 5) 远端判读  ： ${RESIDUE_CMD}
                  并回到开发机跑 deploy/sdk/board_diag.sh 拿完整判读（含 CLI --list-skills 自检）

 注：演练只证明"命令与制品齐备"，不构成任何板卡侧证据。
PLAN
  exit 0
fi

log "== 真实部署（--apply）=="
log "① 远端建目录（目标端的目录布局**镜像仓库**：SCRIPT_DIR/../.. 必须等于 repo 根）"
remote "mkdir -p ${REMOTE_SDK_DIR}/units ${REMOTE_DIR}/scripts ${REMOTE_DIR}/config/install ${REMOTE_DIR}/wheelhouse" \
  || { err "远端建目录失败"; exit 3; }
if [ -n "${REMOTE_ROOT}" ]; then
  # 演练用的沙箱根可能尚不存在；先建出来（真实安装时不设 REMOTE_ROOT ⇒ 不额外建任何目录）
  remote "mkdir -p ${REMOTE_ROOT}" || { err "远端沙箱根创建失败：${REMOTE_ROOT}"; exit 3; }
fi

log "② 推送脚本与声明（按角色通道所需的最小集）"
copy "${SCRIPT_DIR}/install-iraf.sh" "${SCRIPT_DIR}/lib_install.py" "${SCRIPT_DIR}/verify_residue.sh" \
     "${SCRIPT_DIR}/uninstall-iraf.sh" "${SCRIPT_DIR}/rehearse_install_chain.sh" \
     "${TARGET}:${REMOTE_SDK_DIR}/" || { err "脚本传输失败"; exit 3; }
copy "${SCRIPT_DIR}/units/"*.tmpl "${TARGET}:${REMOTE_SDK_DIR}/units/" || { err "单元模板传输失败"; exit 3; }
copy "${REPO_ROOT}/scripts/install_profile_check.py" "${TARGET}:${REMOTE_DIR}/scripts/" || { err "校验器传输失败"; exit 3; }
copy "${REPO_ROOT}/config/install/"* "${TARGET}:${REMOTE_DIR}/config/install/" || { err "声明文件传输失败"; exit 3; }
copy "${REPO_ROOT}/pyproject.toml" "${TARGET}:${REMOTE_DIR}/" || { err "pyproject 传输失败"; exit 3; }
STUBS_DIR="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['python_stubs_dir'])" "${LAYOUT}")"
[ -d "${REPO_ROOT}/${STUBS_DIR}" ] || { err "声明的 proto 桩目录不存在：${REPO_ROOT}/${STUBS_DIR}（先跑 scripts/gen_proto_stubs.sh）"; exit 2; }
# ⚠ 只建到 STUBS_DIR 这一层：scp -r <src>/iraf 到**已存在**的 <dest>/iraf 会再多一层（iraf/iraf/…）
remote "mkdir -p ${REMOTE_DIR}/${STUBS_DIR}" || { err "远端桩目录创建失败"; exit 3; }
copy -r "${REPO_ROOT}/${STUBS_DIR}/iraf" "${TARGET}:${REMOTE_DIR}/${STUBS_DIR}/" || { err "proto 桩传输失败"; exit 3; }
remote "test -d ${REMOTE_DIR}/${STUBS_DIR}/iraf/v1 && echo stubs-ok" \
  | grep -q stubs-ok || { err "proto 桩落地位置不符（期望 ${REMOTE_DIR}/${STUBS_DIR}/iraf/v1）"; exit 3; }

# 仿真档场景 + 生成器：声明里的 sim_scene 是**仓库相对**路径，推到远端同位置；
# 目标端安装器据此复制进版本目录并在板上生成 IRAF_BACKEND_CONFIG（不手写）。
SIM_SCENE="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1]).get('sim_scene') or '')" "${LAYOUT}")"
if [ -n "${SIM_SCENE}" ]; then
  [ -f "${REPO_ROOT}/${SIM_SCENE}" ] || { err "声明的仿真场景不存在：${REPO_ROOT}/${SIM_SCENE}"; exit 2; }
  [ -f "${REPO_ROOT}/${SIM_SCENE%.*}.json" ] || { err "场景旁挂报告缺失：${REPO_ROOT}/${SIM_SCENE%.*}.json（先跑 scripts/build_baseline.py）"; exit 2; }
  remote "mkdir -p ${REMOTE_DIR}/$(dirname "${SIM_SCENE}") ${REMOTE_DIR}/scripts" || { err "远端场景目录创建失败"; exit 3; }
  copy "${REPO_ROOT}/${SIM_SCENE}" "${REPO_ROOT}/${SIM_SCENE%.*}.json" "${TARGET}:${REMOTE_DIR}/$(dirname "${SIM_SCENE}")/" \
    || { err "场景传输失败"; exit 3; }
  copy "${REPO_ROOT}/scripts/emit_backend_config.py" "${TARGET}:${REMOTE_DIR}/scripts/" || { err "生成器传输失败"; exit 3; }
  log "  ✓ 仿真场景与生成器已推送（${SIM_SCENE}）"
fi

log "③ 推送制品（保持原始文件名：pip 按文件名做 PEP 427 校验，改名会被拒）"
remote "mkdir -p ${REMOTE_DIR}/artifacts" || { err "远端制品目录创建失败"; exit 3; }
SDK_WHEEL_REMOTE="${REMOTE_DIR}/artifacts/$(basename "${SDK_WHEEL}")"
copy "${SDK_WHEEL}" "${TARGET}:${REMOTE_DIR}/artifacts/" || { err "SDK wheel 传输失败"; exit 3; }
RUNTIME_REMOTE=""
if [ -n "${RUNTIME_BUNDLE}" ]; then
  RUNTIME_REMOTE="${REMOTE_DIR}/artifacts/$(basename "${RUNTIME_BUNDLE}")"
  copy "${RUNTIME_BUNDLE}" "${TARGET}:${REMOTE_DIR}/artifacts/" || { err "runtime 包传输失败"; exit 3; }
fi
[ -n "${WHEELHOUSE}" ] && {
  # 拷贝**目录内容**而不是目录本身：① 直接 scp -r <dir> 到已存在目录会变成 <dest>/<dir>/…；
  # ② 新版 OpenSSH 的 scp 拒绝 `<dir>/.`（报 "unexpected filename: ."）⇒ 用本地 glob 展开内容。
  copy "${WHEELHOUSE}"/* "${TARGET}:${REMOTE_DIR}/wheelhouse/" || { err "wheelhouse 传输失败"; exit 3; }
}

log "④ 远端安装（目标端自行预检；失败会回滚）"
# 安装器自身的依赖（PyYAML/jsonschema）也要有：目标端系统 python 实测**没有** yaml，
# 直接跑会 `ModuleNotFoundError: No module named 'yaml'`。做法：从**同一个 wheelhouse** 装到
# 独立目录 <remote>/pylib，再用 PYTHONPATH 提供给安装脚本 —— 不碰板卡系统 site-packages，
# 卸载时随 <remote>/pylib 一起删掉（与"干净清除"的边界一致）。
log "④a 目标端安装器依赖（--no-index --target <remote>/pylib；不写系统 site-packages）"
set +e
remote "python3 -m pip install --disable-pip-version-check --no-index --find-links ${REMOTE_DIR}/wheelhouse --target ${REMOTE_DIR}/pylib pyyaml jsonschema >/dev/null 2>&1 || pip3 install --disable-pip-version-check --no-index --find-links ${REMOTE_DIR}/wheelhouse --target ${REMOTE_DIR}/pylib pyyaml jsonschema" 2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-tooling-deps.log" | tail -3
rc_tooling=${PIPESTATUS[0]}
set -e
if [ "${rc_tooling}" -ne 0 ]; then
  err "目标端安装器依赖安装失败（退出码 ${rc_tooling}）：见 build/board/.../deploy-tooling-deps.log"
  exit 3
fi
remote "PYTHONPATH=${REMOTE_DIR}/pylib python3 -c 'import yaml, jsonschema; print(\"tooling-deps-ok\")'" \
  || { err "目标端 PYTHONPATH 自检失败：yaml/jsonschema 不可导入"; exit 3; }
log "  ✓ 目标端安装器依赖就绪（pylib + PYTHONPATH 自检通过）"

set +e
remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} ${INSTALL_CMD}" 2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-install.log"
rc=${PIPESTATUS[0]}
set -e
if [ "${rc}" -ne 0 ]; then
  err "远端安装未通过（退出码 ${rc}）：见上面的中文原因与 build/board/.../deploy-install.log"
  exit 4
fi

log "⑤ 远端判读（残留核验：装完应报不干净 —— 那是「已装即有内容」，不是缺陷）"
set +e
remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} ${RESIDUE_CMD}" \
  2>&1 | tee "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-residue.json" | tail -5
set -e
if [ "${REHEARSAL}" = "1" ]; then
  log "⑥ 替身演练收尾：远端受控卸载 + 残留核验（证明管线可干净回收）"
  set +e
  remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} ${UNINSTALL_CMD}" >/dev/null 2>&1
  remote "cd ${REMOTE_DIR} && PYTHONPATH=${REMOTE_DIR}/pylib IRAF_REPO_ROOT=${REMOTE_DIR} ${RESIDUE_CMD}" \
    > "${REPO_ROOT}/build/board/${IRAF_BOARD_NAME:-board}/deploy-residue-after-uninstall.json" 2>&1
  rc=$?
  set -e
  if [ "${rc}" -eq 0 ]; then
    log "  ✓ 卸载后六路残留全 0（管线可干净回收）"
  else
    log "  ✗ 卸载后仍有残留（rc=${rc}）——见 deploy-residue-after-uninstall.json"
  fi
fi
log "部署完成：请回到开发机跑 deploy/sdk/board_diag.sh --profile ${PROFILE} 拿完整判读（含 CLI --list-skills 自检）"
exit 0
