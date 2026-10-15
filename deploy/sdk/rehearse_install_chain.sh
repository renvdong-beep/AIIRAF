#!/usr/bin/env bash
# 安装→核验→卸载→核验 的**全链路沙箱演练**（IRAF 双域部署 P2b 证据入口）。
#
# 为什么必须演练：真机写入前要证明三件事 —— ①预检真的会拦 ②安装失败会回滚
# ③装完之后能按记录干净清除（六路残留全 0）。
#
# 做法：在沙箱目录里扮演文件系统根 "/"，造出**离线 wheelhouse + runtime 包**（自造的最小件，
# 不含真实依赖），然后跑完整的装/卸链路。全程不碰真实系统目录、不启服务。
#
# 用例（期望值写在用例名里）：
#   ① --dry-run 安装            → 退出码 0 且**不写任何文件**
#   ② 真安装                    → 退出码 0；版本目录/current/env/unit/安装记录齐备
#   ③ 装完残留核验              → 退出码 4（已装即有内容 ⇒ 证明核验不是橡皮章）
#   ④ 卸载 --dry-run            → 退出码 0 且文件数不变
#   ⑤ 真卸载                    → 退出码 0
#   ⑥ 卸载后核验                → 退出码 0（六路全 0）
#   ⑦ 拒绝覆盖：预置同名 unit 后安装 → 退出码 4，且**回滚**后无版本目录残留
#   ⑧ 拒绝在真实根上演练        → 退出码 1（--rehearsal 必须配沙箱 root）
#
# 用法：bash deploy/sdk/rehearse_install_chain.sh [--profile edge-motion] [--out <证据目录>]
# 退出码：0 全部符合预期 / 1 参数错误 / 2 有用例不符合预期
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

PROFILE="edge-motion"
OUT="${REPO_ROOT}/build/acceptance/install-chain"

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      cat <<'USAGE'
全链路沙箱演练（安装 → 残留核验 → 受控卸载 → 残留核验）

用法：bash rehearse_install_chain.sh [--profile <角色>] [--out <证据目录>]

选项：
  -h, --help        显示本帮助并退出
  --profile <id>    演练用的角色（默认 edge-motion：绝对前缀 + 离线 wheelhouse）
  --out <dir>       证据输出目录（默认 build/acceptance/install-chain）

全程只在沙箱目录内操作：不写真实系统目录、不启服务、不联网。
USAGE
      exit 0
      ;;
    --profile) PROFILE="$2"; shift ;;
    --out) OUT="$2"; shift ;;
    *) printf '[rehearse-chain][错误] 无法识别的参数：%s\n' "$1" >&2; exit 1 ;;
  esac
  shift
done

SANDBOX="${OUT}/sandbox"
ART="${OUT}/artifacts"
LOG="${OUT}/rehearsal-chain.log"
mkdir -p "${OUT}"
: > "${LOG}"

log() { printf '%s\n' "$*" | tee -a "${LOG}"; }
step() { printf '\n=== %s ===\n' "$*" | tee -a "${LOG}"; }
FAILED=0
check() {
  if [ "$1" = "$2" ]; then
    log "  ✓ ${3}：退出码 ${2}（期望 ${1}）"
  else
    log "  ✗ ${3}：退出码 ${2}，期望 ${1}"
    FAILED=1
  fi
}
count_files() { find "$1" -type f 2>/dev/null | wc -l; }

step "0) 重建沙箱与自造件（只动 ${OUT}）"
rm -rf "${SANDBOX}" "${ART}"
mkdir -p "${SANDBOX}" "${ART}/wheelhouse"

# 自造最小 SDK wheel + 离线 wheelhouse 清单（**不含真实依赖**，只用于验证安装链路本身）
WHEEL="$("${PYTHON}" - "${ART}/wheelhouse" <<'PY'
import sys, zipfile
from pathlib import Path

dest = Path(sys.argv[1])
dest.mkdir(parents=True, exist_ok=True)
whl = dest / "iraf_sdk-0.2.0-py3-none-any.whl"
with zipfile.ZipFile(whl, "w") as zf:
    zf.writestr("iraf_sdk/__init__.py", "__version__ = '0.2.0'\n")
    zf.writestr("iraf_sdk-0.2.0.dist-info/METADATA",
                "Metadata-Version: 2.1\nName: iraf_sdk\nVersion: 0.2.0\n")
    zf.writestr("iraf_sdk-0.2.0.dist-info/WHEEL",
                "Wheel-Version: 1.0\nGenerator: rehearsal\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
    zf.writestr("iraf_sdk-0.2.0.dist-info/RECORD", "")
# 清单必须写明**构建目标**：安装预检会比对声明里的 dependency_source.target_id
TARGET_ID = "aarch64-manylinux_2_28-cp311"
for dep in ("pyyaml", "jsonschema", "grpcio", "protobuf"):
    with zipfile.ZipFile(dest / ("%s-1.0.0-py3-none-any.whl" % dep), "w") as zf:
        zf.writestr("%s/__init__.py" % dep, "__version__ = '1.0.0'\n")
        zf.writestr("%s-1.0.0.dist-info/METADATA" % dep,
                    "Metadata-Version: 2.1\nName: %s\nVersion: 1.0.0\n" % dep)
        zf.writestr("%s-1.0.0.dist-info/WHEEL" % dep,
                    "Wheel-Version: 1.0\nGenerator: rehearsal\nRoot-Is-Purelib: true\nTag: py3-none-any\n")
        zf.writestr("%s-1.0.0.dist-info/RECORD" % dep, "")
(dest / "wheelhouse.json").write_text(
    '{"schema_version": "iraf.wheelhouse/v1", "target": "%s",'
    ' "note": "沙箱演练自造，不含真实依赖", "files": [{"name": "%s"}]}\n' % (TARGET_ID, whl.name),
    encoding="utf-8")
print(whl)
PY
)"
# 自造 proto 桩目录（不依赖 protobuf，仅验证"桩会被拷进 lib/ 并纳入安装记录"这条链路）
STUBS="${ART}/stubs"
mkdir -p "${STUBS}/iraf/v1"
: > "${STUBS}/iraf/__init__.py"
: > "${STUBS}/iraf/v1/__init__.py"
printf 'VALUE = 1\n' > "${STUBS}/iraf/v1/skill_pb2.py"
log "  自造 proto 桩：${STUBS}/iraf/v1"

# 自造 runtime 源码包（单顶层目录，验证解包与去顶层逻辑）
RUNTIME="$("${PYTHON}" - "${ART}" <<'PY'
import sys, tarfile
from pathlib import Path

dest = Path(sys.argv[1])
bundle = dest / "iraf-runtime-0.2.0-aarch64.tar.gz"
with tarfile.open(bundle, "w:gz") as tar:
    import io
    payload = "# IRAF runtime (sandbox fixture)\n".encode("utf-8")
    info = tarfile.TarInfo("iraf-runtime-0.2.0/src/iraf_core/__init__.py")
    info.size = len(payload)
    tar.addfile(info, io.BytesIO(payload))
print(bundle)
PY
)"
log "  自造 SDK wheel：${WHEEL}"
log "  自造 runtime  ：${RUNTIME}"

# 角色生效布局（路径全部来自声明，脚本内不写死）
LAYOUT="$("${PYTHON}" "${SCRIPT_DIR}/lib_install.py" layout --profile "${PROFILE}")"
VERSION="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['version'])" "${LAYOUT}")"
PREFIX="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['prefix'])" "${LAYOUT}")"
CONFIG_DIR="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['config_dir'])" "${LAYOUT}")"
UNIT_DIR="${SANDBOX}/etc/systemd/system"
UNIT="${UNIT_DIR}/iraf-control-provider.service"
VERSION_DIR="${SANDBOX}${PREFIX}/${VERSION}"
ENV_FILE="${SANDBOX}${CONFIG_DIR}/${PROFILE}.env"
log "  角色=${PROFILE} 版本=${VERSION} 前缀=${PREFIX}"

INSTALL_ARGS=(--profile "${PROFILE}" --root "${SANDBOX}" --sdk-wheel "${WHEEL}"
              --runtime-bundle "${RUNTIME}" --wheelhouse "${ART}/wheelhouse"
              --stubs-dir "${STUBS}" --allow-unverified --rehearsal)

step "1) 安装演练 --dry-run（期望：退出码 0 且不写任何文件）"
set +e
bash "${SCRIPT_DIR}/install-iraf.sh" "${INSTALL_ARGS[@]}" --dry-run 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "安装演练应成功"
if [ ! -e "${SANDBOX}${PREFIX}" ] && [ ! -e "${SANDBOX}${CONFIG_DIR}" ]; then
  log "  ✓ dry-run 未写任何文件（前缀与配置目录均不存在）"
else
  log "  ✗ dry-run 写了文件：$(ls -d "${SANDBOX}${PREFIX}" "${SANDBOX}${CONFIG_DIR}" 2>/dev/null | tr '\n' ' ')"
  FAILED=1
fi

step "2) 真安装（期望：退出码 0，产物齐备）"
set +e
bash "${SCRIPT_DIR}/install-iraf.sh" "${INSTALL_ARGS[@]}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "安装应成功"
for probe in "${VERSION_DIR}" "${SANDBOX}${PREFIX}/current" "${ENV_FILE}" "${UNIT}" \
             "${VERSION_DIR}/iraf-install-files.json"; do
  if [ -e "${probe}" ]; then
    log "  ✓ 产物存在：${probe}"
  else
    log "  ✗ 产物缺失：${probe}"; FAILED=1
  fi
done
if [ -f "${VERSION_DIR}/lib/iraf_sdk/__init__.py" ]; then
  log "  ✓ SDK 已装入版本目录 lib/"
else
  log "  ✗ SDK 未装入版本目录 lib/"; FAILED=1
fi
if [ -f "${VERSION_DIR}/src/iraf_core/__init__.py" ]; then
  log "  ✓ runtime 已解包（顶层目录已剥离）"
else
  log "  ✗ runtime 未解包"; FAILED=1
fi
if [ -f "${VERSION_DIR}/lib/iraf/v1/skill_pb2.py" ]; then
  log "  ✓ proto 桩已拷入 lib/（运行期 import iraf.v1.* 需要）"
else
  log "  ✗ proto 桩未拷入 lib/"; FAILED=1
fi
if [ -f "${VERSION_DIR}/lib/jsonschema/__init__.py" ]; then
  log "  ✓ 运行期依赖已装入 lib/（按声明的 runtime_dependencies）"
else
  log "  ✗ 运行期依赖缺失"; FAILED=1
fi
if grep -q "IRAF_BACKEND_ENTRYPOINT=" "${ENV_FILE}" 2>/dev/null; then
  log "  ✓ env 文件已渲染（含 IRAF_BACKEND_ENTRYPOINT）"
else
  log "  ✗ env 文件内容异常"; FAILED=1
fi
if grep -q "^User=" "${UNIT}" 2>/dev/null && ! grep -q "@" "${UNIT}"; then
  log "  ✓ 单元已渲染（User 已替换，无残留占位符）"
else
  log "  ✗ 单元渲染异常（可能仍有 @TOKEN@）"; FAILED=1
fi

step "3) 装完残留核验（期望：退出码 4 —— 已装即有内容）"
set +e
bash "${SCRIPT_DIR}/verify_residue.sh" --profile "${PROFILE}" --root "${SANDBOX}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 4 "${rc}" "装完核验应报不干净"

step "4) 卸载演练 --dry-run（期望：退出码 0 且文件数不变）"
before="$(count_files "${VERSION_DIR}")"
set +e
bash "${SCRIPT_DIR}/uninstall-iraf.sh" --profile "${PROFILE}" --root "${SANDBOX}" --dry-run 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "卸载演练应成功"
after="$(count_files "${VERSION_DIR}")"
if [ "${before}" = "${after}" ]; then
  log "  ✓ dry-run 未删除任何文件（${before} → ${after}）"
else
  log "  ✗ dry-run 删除了文件（${before} → ${after}）"; FAILED=1
fi

step "5) 真卸载（期望：退出码 0）"
set +e
bash "${SCRIPT_DIR}/uninstall-iraf.sh" --profile "${PROFILE}" --root "${SANDBOX}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "真卸载应成功"

step "6) 卸载后核验（期望：退出码 0 —— 六路全 0）"
set +e
bash "${SCRIPT_DIR}/verify_residue.sh" --profile "${PROFILE}" --root "${SANDBOX}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 0 "${rc}" "卸载后应六路全 0"

step "7) 拒绝覆盖：预置同名 unit 后安装（期望：退出码 4，且回滚后无版本目录残留）"
mkdir -p "${UNIT_DIR}"
printf '[Unit]\nDescription=占位（模拟"该单元已存在"）\n' > "${UNIT}"
set +e
bash "${SCRIPT_DIR}/install-iraf.sh" "${INSTALL_ARGS[@]}" 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 4 "${rc}" "unit 已存在时安装应被拒绝"
rm -f "${UNIT}"
if [ ! -d "${VERSION_DIR}" ]; then
  log "  ✓ 回滚生效：版本目录未残留（${VERSION_DIR}）"
else
  log "  ✗ 回滚不完整：仍存在 ${VERSION_DIR}"; FAILED=1
fi
if [ ! -e "${ENV_FILE}" ]; then
  log "  ✓ 回滚生效：env 文件未残留"
else
  log "  ✗ 回滚不完整：仍存在 ${ENV_FILE}"; FAILED=1
fi

step "8) 拒绝在真实根上演练（期望：退出码 1）"
set +e
"${PYTHON}" "${SCRIPT_DIR}/lib_install.py" install --profile "${PROFILE}" --root / \
  --sdk-wheel "${WHEEL}" --wheelhouse "${ART}/wheelhouse" \
  --allow-unverified --rehearsal --dry-run 2>&1 | tee -a "${LOG}"
rc=$?
set -e
check 1 "${rc}" "真实根上开演练应被拒绝"

step "结果"
if [ "${FAILED}" = "0" ]; then
  log "全部用例符合预期。证据：${LOG}"
  exit 0
fi
log "**有用例不符合预期**（见上面 ✗）。证据：${LOG}"
exit 2
