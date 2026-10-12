#!/usr/bin/env bash
# 安装/卸载/残留核验的**沙箱演练**（IRAF 双域部署 P2b 证据入口）。
#
# 为什么必须演练：真机写入前要先证明"卸载只动本包、卸载后六路全 0"。
# 本脚本在沙箱目录里扮演"文件系统根 /"，造出一棵**像是装过**的树 + 安装记录，
# 然后依次跑四个用例，并把每步的退出码与关键输出落盘成证据。
#
# 用例（全部在沙箱内，不碰真实系统目录、不启服务）：
#   ① 卸载前核验 → 必须报"不干净"（证明核验不是橡皮章）
#   ② 卸载演练（--dry-run）→ 必须 exit 0 且**不删任何文件**（用文件数对账）
#   ③ 真卸载 → 必须 exit 0
#   ④ 卸载后核验 → 必须 exit 0（六路全 0）
#   ⑤ 拒绝路径：往版本目录塞一个"清单外文件"后卸载 → 必须 exit 4（拒绝，不尽力而为）
#
# 用法：
#   bash deploy/sdk/rehearse_install_layout.sh [--profile edge-motion] [--out <证据目录>]
# 退出码：0 全部用例符合预期 / 1 参数错误 / 2 有用例不符合预期
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

PROFILE="edge-motion"
OUT="${REPO_ROOT}/build/acceptance/install-layout"

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      cat <<'USAGE'
沙箱演练（安装记录 / 残留核验 / 受控卸载）

用法：bash rehearse_install_layout.sh [--profile <角色>] [--out <证据目录>]

选项：
  -h, --help        显示本帮助并退出
  --profile <id>    用哪个角色的声明做演练（默认 edge-motion：绝对前缀，便于沙箱）
  --out <dir>       证据输出目录（默认 build/acceptance/install-layout）

全程只在沙箱目录内操作，不写真实系统目录、不启服务。
USAGE
      exit 0
      ;;
    --profile) PROFILE="$2"; shift ;;
    --out) OUT="$2"; shift ;;
    *) printf '[rehearse][错误] 无法识别的参数：%s\n' "$1" >&2; exit 1 ;;
  esac
  shift
done

mkdir -p "${OUT}"
SANDBOX="${OUT}/sandbox"
LOG="${OUT}/rehearsal.log"
: > "${LOG}"

log() { printf '%s\n' "$*" | tee -a "${LOG}"; }
step() { printf '\n=== %s ===\n' "$*" | tee -a "${LOG}"; }
FAILED=0

check() {  # check <期望退出码> <实际退出码> <用例名>
  if [ "$1" = "$2" ]; then
    log "  ✓ ${3}：退出码 ${2}（期望 ${1}）"
  else
    log "  ✗ ${3}：退出码 ${2}，期望 ${1}"
    FAILED=1
  fi
}

count_files() { find "$1" -type f 2>/dev/null | wc -l; }

step "0) 清理并重建沙箱（只动 ${SANDBOX}）"
# 只删本脚本自己在证据目录里建的沙箱，避免误伤
if [ -d "${SANDBOX}" ]; then rm -rf "${SANDBOX}"; fi
mkdir -p "${SANDBOX}"

# 角色的生效布局（从声明解析，脚本里不写死路径）
LAYOUT_JSON="$("${PYTHON}" "${SCRIPT_DIR}/lib_install.py" --repo-root "${REPO_ROOT}" layout --profile "${PROFILE}")"
VERSION="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['version'])" "${LAYOUT_JSON}")"
PREFIX="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['prefix'])" "${LAYOUT_JSON}")"
CONFIG_DIR="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['config_dir'])" "${LAYOUT_JSON}")"
DATA_DIR="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['data_dir'])" "${LAYOUT_JSON}")"
CURRENT="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['current_link'])" "${LAYOUT_JSON}")"
UNIT="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['residue_units'][0])" "${LAYOUT_JSON}")"
log "  角色=${PROFILE} 版本=${VERSION}"
log "  前缀=${PREFIX} 配置=${CONFIG_DIR} 数据=${DATA_DIR} 单元=${UNIT}"

VERSION_DIR="${SANDBOX}${PREFIX}/${VERSION}"
mkdir -p "${VERSION_DIR}/bin" "${VERSION_DIR}/src/iraf_core"
printf '#!/bin/sh\necho iraf\n' > "${VERSION_DIR}/bin/iraf-agentos"
printf 'x = 1\n' > "${VERSION_DIR}/src/iraf_core/runtime.py"
ln -s "${VERSION}" "${SANDBOX}${PREFIX}/${CURRENT}"
mkdir -p "${SANDBOX}/etc/systemd/system" "${SANDBOX}${CONFIG_DIR}" "${SANDBOX}${DATA_DIR}"
printf '[Unit]\nDescription=IRAF\n' > "${SANDBOX}/etc/systemd/system/${UNIT}"
printf 'IRAF_PROFILE=demo\n' > "${SANDBOX}${CONFIG_DIR}/iraf-sdk.env"
printf 'events\n' > "${SANDBOX}${DATA_DIR}/iraf-events.db"

step "1) 写安装记录（版本目录 + 版本目录之外的配置文件与新建目录）"
"${PYTHON}" "${SCRIPT_DIR}/lib_install.py" --repo-root "${REPO_ROOT}" record-write \
  --profile "${PROFILE}" --installed-dir "${VERSION_DIR}" \
  --config-file "${SANDBOX}${CONFIG_DIR}/iraf-sdk.env" \
  --created-dir "${SANDBOX}${PREFIX}" --created-dir "${SANDBOX}${CONFIG_DIR}" \
  --note "沙箱演练" 2>&1 | tee -a "${LOG}"

step "2) 卸载前核验（期望：不干净 ⇒ 退出码 4，证明核验不是橡皮章）"
set +e
bash "${SCRIPT_DIR}/verify_residue.sh" --profile "${PROFILE}" --root "${SANDBOX}" \
  --repo-root "${REPO_ROOT}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 4 "${rc}" "卸载前核验应报不干净"

step "3) 卸载演练 --dry-run（期望 exit 0 且文件数不变）"
before="$(count_files "${VERSION_DIR}")"
set +e
bash "${SCRIPT_DIR}/uninstall-iraf.sh" --profile "${PROFILE}" --root "${SANDBOX}" \
  --repo-root "${REPO_ROOT}" --dry-run 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "演练应成功"
after="$(count_files "${VERSION_DIR}")"
if [ "${before}" = "${after}" ]; then
  log "  ✓ dry-run 未删除任何文件（${before} → ${after}）"
else
  log "  ✗ dry-run 删除了文件（${before} → ${after}）"; FAILED=1
fi

step "4) 真卸载（期望 exit 0）"
set +e
bash "${SCRIPT_DIR}/uninstall-iraf.sh" --profile "${PROFILE}" --root "${SANDBOX}" \
  --repo-root "${REPO_ROOT}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "真卸载应成功"

step "5) 卸载后核验（期望 exit 0：六路全 0）"
set +e
bash "${SCRIPT_DIR}/verify_residue.sh" --profile "${PROFILE}" --root "${SANDBOX}" \
  --repo-root "${REPO_ROOT}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "卸载后应六路全 0"

step "6) 拒绝路径：清单外文件 ⇒ 卸载必须拒绝（exit 4）"
mkdir -p "${VERSION_DIR}/bin"
printf 'x\n' > "${VERSION_DIR}/bin/unknown-binary"
"${PYTHON}" "${SCRIPT_DIR}/lib_install.py" --repo-root "${REPO_ROOT}" record-write \
  --profile "${PROFILE}" --installed-dir "${VERSION_DIR}" --note "拒绝路径用例" >/dev/null 2>&1
printf 'x\n' > "${VERSION_DIR}/bin/unknown-binary"   # 记录之后再放进去 ⇒ 成为清单外文件
set +e
bash "${SCRIPT_DIR}/uninstall-iraf.sh" --profile "${PROFILE}" --root "${SANDBOX}" \
  --repo-root "${REPO_ROOT}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 4 "${rc}" "清单外文件应导致拒绝"

step "结果"
if [ "${FAILED}" = "0" ]; then
  log "全部用例符合预期。证据：${LOG}"
  exit 0
fi
log "**有用例不符合预期**（见上面 ✗）。证据：${LOG}"
exit 2
