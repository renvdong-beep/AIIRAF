#!/usr/bin/env bash
# IRAF SDK 板上升级一条龙：build → 受控卸载 → 装 → 判读（幂等、可反复跑）
#
# 为什么要有它（2026-10-23 实测）：一次升版要手工拼 4 步，而每次都撞同样的坑：
#   ① 预检"声明未实测" → 必须 --allow-unverified（exploratory，不构成验收证据）
#   ② 安装器**拒绝覆盖**已装版本 → 必须先受控卸载
#   ③ 缺 PROVIDER_ENDPOINT → 必须 --values + --env-set，否则安装器显式失败
#   ④ 卸载会把 /etc/iraf/runtime.secrets 一起删（它是安装记录里的文件）
#      → 升级会"顺手把凭据弄丢"，装完 runtime 因缺 token 起不来
# 当天为这几条来回跑了 4 轮、每轮 6 分钟左右。本脚本把它们固化成一次执行。
#
# 用法（开发机）：
#   export IRAF_BOARD_HOST=10.203.247.147 SSHPASS=1
#   bash deploy/sdk/board_upgrade.sh --profile edge-intel                 # 演练：只打印计划
#   bash deploy/sdk/board_upgrade.sh --profile edge-intel --apply         # 真升级
#   bash deploy/sdk/board_upgrade.sh --profile edge-intel --apply --window-logs
# 选项：
#   --version <x.y.z>        指定版本（默认读 pyproject.toml#project.version）
#   --target <id>            构建目标（默认按 profile 的 dependency_source.target_id）
#   --values <file>          部署期取值（默认 config/install/deploy_values/<profile>.yaml）
#   --provider-endpoint <h:p> 覆盖 Provider 端点（默认从板上现有装配单读回）
#   --no-secrets             不保留板上 runtime.secrets（默认保留并复用）
#   --skip-build             复用已有 wheel/bundle
#   --skip-diag              装完不跑 board_diag
#   --apply                  真执行（默认演练）
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SDK_DIR="${REPO_ROOT}/deploy/sdk"
PROFILE=""
VERSION=""
TARGET=""
VALUES=""
PROVIDER_ENDPOINT=""
KEEP_SECRETS="1"
SKIP_BUILD="0"
SKIP_DIAG="0"
APPLY="0"

log() { echo "[upgrade] $*"; }
err() { echo "[upgrade][错误] $*" >&2; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --profile) PROFILE="${2:?}"; shift ;;
    --version) VERSION="${2:?}"; shift ;;
    --target) TARGET="${2:?}"; shift ;;
    --values) VALUES="${2:?}"; shift ;;
    --provider-endpoint) PROVIDER_ENDPOINT="${2:?}"; shift ;;
    --no-secrets) KEEP_SECRETS="0" ;;
    --skip-build) SKIP_BUILD="1" ;;
    --skip-diag) SKIP_DIAG="1" ;;
    --apply) APPLY="1" ;;
    *) err "无法识别的参数：$1"; exit 1 ;;
  esac
  shift
done

[ -n "$PROFILE" ] || { err "缺 --profile（如 edge-intel）"; exit 1; }
: "${IRAF_BOARD_HOST:?需要 IRAF_BOARD_HOST}"
: "${SSHPASS:?需要 SSHPASS}"
export SSHPASS

# 版本：单一来源 pyproject.toml
if [ -z "$VERSION" ]; then
  VERSION="$(sed -n 's/^version = "\(.*\)"$/\1/p' "${REPO_ROOT}/pyproject.toml" | head -1)"
fi
[ -n "$VERSION" ] || { err "读不到版本（pyproject.toml#project.version）"; exit 1; }
VERSION_TAG="$VERSION"
log "目标：${IRAF_BOARD_HOST}｜角色 ${PROFILE}｜版本 ${VERSION}"

# 构建目标与取值文件的默认值都来自声明，不写死
TARGET_ID="$(PYTHONPATH="${REPO_ROOT}/deploy/sdk" python3 "${SDK_DIR}/lib_install.py" layout --profile "$PROFILE" 2>/dev/null \
  | python3 -c 'import json,sys; d=json.load(sys.stdin); print((d.get("dependency_source") or {}).get("target_id") or "")' 2>/dev/null)"
[ -n "$TARGET" ] || TARGET="${TARGET_ID:-aarch64-manylinux_2_28-cp311}"
[ -n "$VALUES" ] || VALUES="${REPO_ROOT}/config/install/deploy_values/${PROFILE}.yaml"

WHEEL="${REPO_ROOT}/build/sdk/iraf_sdk-${VERSION}-py3-none-any.whl"
BUNDLE="${REPO_ROOT}/build/sdk/iraf-runtime-${VERSION}-aarch64.tar.gz"
WHEELHOUSE="${REPO_ROOT}/build/wheelhouse/${TARGET}"

log "构建目标 ${TARGET}｜取值 ${VALUES}｜wheelhouse ${WHEELHOUSE}"

run_remote() {
  sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "root@${IRAF_BOARD_HOST}" "$1" 2>&1
}

# ---------- 1. 构建（缺产物才建） ----------
if [ "$SKIP_BUILD" = "1" ]; then
  log "① 跳过构建（--skip-build）"
  [ -f "$WHEEL" ] || { err "缺 ${WHEEL}"; exit 2; }
else
  if [ -f "$WHEEL" ] && [ -f "$BUNDLE" ]; then
    log "① 产物已存在，复用（要重建请先删 build/sdk）"
  else
    log "① 构建 SDK（deploy/sdk/build_sdk.sh --target ${TARGET}）"
    if [ "$APPLY" != "1" ]; then
      log "   [演练] bash deploy/sdk/build_sdk.sh --target ${TARGET}"
    else
      bash "${SDK_DIR}/build_sdk.sh" --target "$TARGET" 2>&1 | tail -4
      [ -f "$WHEEL" ] || { err "构建后仍缺 ${WHEEL}"; exit 3; }
    fi
  fi
fi

# ---------- 2. 探测板上现状（版本 / 端点 / 凭据） ----------
CURRENT="$(run_remote 'readlink /opt/iraf-sdk/current 2>/dev/null || echo none')"
CURRENT="$(printf '%s' "$CURRENT" | tr -d '\r')"
log "② 板上现状：current=${CURRENT:-none}"

if [ -z "$PROVIDER_ENDPOINT" ]; then
  PROVIDER_ENDPOINT="$(run_remote "grep -ho '\"endpoint\":\"[^\"]*\"' /etc/iraf/${PROFILE}.env 2>/dev/null | head -1 | sed 's/.*:\"//;s/\"//'" | tr -d '\r')"
fi
if [ -n "$PROVIDER_ENDPOINT" ]; then
  log "   Provider 端点（沿用板上现有值）：${PROVIDER_ENDPOINT}"
else
  # 不能"继续但缺占位符"：安装器会在渲染阶段失败，且失败原因不如这里直白。
  err "读不到 Provider 端点（/etc/iraf/${PROFILE}.env 不存在或没有 endpoint）"
  err "处置：加 --provider-endpoint <x86-host>:50052（地址不入库，见 AGENTS 6.10）"
  exit 2
fi

SECRETS_ARG=()
TMP_SECRETS=""
if [ "$KEEP_SECRETS" = "1" ] && [ "$APPLY" = "1" ]; then
  # 卸载会删掉 /etc/iraf/<profile>.env 与 runtime.secrets（都在安装记录里）。
  # 凭据不能靠"远端暂存目录"（board_deploy 的 --secrets-file 只吃**本地**文件，
  # 且远端 artifacts 目录在推送阶段会被覆盖/清理）⇒ 卸载前取回本地临时文件，装完删除。
  if run_remote "test -f /etc/iraf/runtime.secrets" >/dev/null 2>&1; then
    TMP_SECRETS="$(mktemp /tmp/iraf-secrets.XXXXXX)"
    sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "root@${IRAF_BOARD_HOST}" \
      "cat /etc/iraf/runtime.secrets" > "${TMP_SECRETS}" 2>/dev/null
    chmod 600 "${TMP_SECRETS}"
    if [ -s "${TMP_SECRETS}" ]; then
      SECRETS_ARG=(--secrets-file "${TMP_SECRETS}")
      log "   凭据：已从板卡取回（$(wc -c < "${TMP_SECRETS}") 字节，值不回显），装完由安装器重新落地"
    else
      log "   [警告] 取回 runtime.secrets 失败 ⇒ 装完 runtime 会因缺凭据 fail-closed"
    fi
  else
    log "   [警告] 板上没有 /etc/iraf/runtime.secrets ⇒ 装完 runtime 会因缺凭据 fail-closed"
  fi
elif [ "$KEEP_SECRETS" = "1" ]; then
  log "   [演练] 将在卸载前把 /etc/iraf/runtime.secrets 取回本地临时文件，装完推回"
fi
cleanup() { [ -n "${TMP_SECRETS}" ] && [ -f "${TMP_SECRETS}" ] && rm -f "${TMP_SECRETS}"; }
trap cleanup EXIT

COMMON=(--profile "$PROFILE" --sdk-wheel "$WHEEL" --runtime-bundle "$BUNDLE"
        --wheelhouse "$WHEELHOUSE" --allow-unverified --values "$VALUES")
[ -n "$PROVIDER_ENDPOINT" ] && COMMON+=(--env-set "PROVIDER_ENDPOINT=${PROVIDER_ENDPOINT}")
[ "${#SECRETS_ARG[@]}" -gt 0 ] && COMMON+=("${SECRETS_ARG[@]}")

if [ "$APPLY" != "1" ]; then
  log "③ [演练] 受控卸载：board_deploy.sh --profile ${PROFILE} --uninstall"
  log "④ [演练] 安装：board_deploy.sh ${COMMON[*]} --apply"
  log "⑤ [演练] 判读：board_diag.sh --profile ${PROFILE}"
  log "演练结束：未对板卡做任何改动（加 --apply 真升级）"
  exit 0
fi

# ---------- 3. 受控卸载（只按安装记录删） ----------
if [ "${CURRENT}" = "none" ] || [ -z "${CURRENT}" ]; then
  log "③ 板上无已装版本，跳过卸载"
else
  log "③ 受控卸载 ${CURRENT}"
  # 注意：board_deploy.sh 的整体退出码是**残留核验**的结果，而卸载后唯一预期残留是
  # /etc/iraf/agentos.key（模型网关 key，非安装器产物，按约定保留）⇒ 整体 rc=4 是正常语义。
  # 所以这里不看 rc，看**事实**：current 链接是否真的没了。
  IRAF_BOARD_HOST="$IRAF_BOARD_HOST" bash "${SDK_DIR}/board_deploy.sh" \
    --profile "$PROFILE" --sdk-wheel "$WHEEL" --uninstall 2>&1 | grep -E "已删除|卸载完成|仍有残留" | tail -3
  AFTER="$(run_remote 'readlink /opt/iraf-sdk/current 2>/dev/null || echo none' | tr -d '\r')"
  if [ -n "${AFTER}" ] && [ "${AFTER}" != "none" ]; then
    err "卸载未生效（current 仍指向 ${AFTER}）：不要继续安装，先看 build/board/.../uninstall.log"
    exit 4
  fi
  log "   ✓ 已卸载（残留核验 rc=4 属预期：只保留 /etc/iraf/agentos.key）"
fi

# ---------- 4. 安装 ----------
log "④ 安装 ${VERSION}"
IRAF_BOARD_HOST="$IRAF_BOARD_HOST" bash "${SDK_DIR}/board_deploy.sh" "${COMMON[@]}" --apply 2>&1 \
  | grep -E "凭据已落地|安装完成|自检|current|错误|警告" | tail -8
RC=${PIPESTATUS[0]}
[ "$RC" -eq 0 ] || { err "安装失败（rc=${RC}）：见 build/board/.../deploy-install.log"; exit 5; }

# ---------- 5. 判读 ----------
if [ "$SKIP_DIAG" = "1" ]; then
  log "⑤ 跳过判读（--skip-diag）"
else
  log "⑤ 判读（board_diag）"
  IRAF_BOARD_HOST="$IRAF_BOARD_HOST" bash "${SDK_DIR}/board_diag.sh" --profile "$PROFILE" 2>&1 \
    | grep -E "就绪|✗|○|✓ 单元状态|✓ CLI" | head -12
fi

log "升级完成：${CURRENT:-none} → ${VERSION}"
log "提醒：AgentOS 侧若在跑 MCP 子进程，需要新会话/重启 dashboard 才会用到新代码（见 scripts/agentos_content_layer.sh --restart）"
