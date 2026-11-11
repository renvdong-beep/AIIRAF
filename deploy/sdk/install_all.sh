#!/usr/bin/env bash
# IRAF 一键安装（x86 仿真机）：运行时 + Provider + 控制面。
#
# 为什么需要它：`sim-x86` 角色只装运行时与 Provider，控制面是独立组件；
# 对使用者来说"装完还要再跑一个脚本"就是额外心智负担。本脚本把两段合成一条命令，
# 且控制面**优先用 bundle 自带的脚本与注册表**，不强制 SSH 到板卡。
#
# 用法：
#   bash deploy/sdk/install_all.sh --sdk-wheel <whl> --runtime-bundle <tgz> \
#        [--host-id x86-1] [--allow 10.203.247.147] [--registry-dir <dir>] [--from-board <ip>] \
#        [--run-user <user>] [--env-set K=V ...] [--secrets-file <f>] [--dry-run]
#
# 退出码：运行时/Provider 失败 → 原样透传；控制面失败 → 7。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HOST_ID="x86-1"
REGISTRY_DIR=""
FROM_BOARD=""
ALLOW=()
INSTALL_ARGS=()
DRY="0"
TOKEN_FILE="${HOME}/.config/iraf/supervisor.token"

while [ $# -gt 0 ]; do
  case "$1" in
    --host-id) HOST_ID="$2"; shift 2 ;;
    --registry-dir) REGISTRY_DIR="$2"; shift 2 ;;
    --from-board) FROM_BOARD="$2"; shift 2 ;;
    --allow) ALLOW+=("$2"); shift 2 ;;
    --dry-run) DRY="1"; INSTALL_ARGS+=("$1"); shift ;;
    --sdk-wheel|--runtime-bundle|--wheelhouse|--run-user|--env-set|--secrets-file|--profile|--root|--repo-root)
      INSTALL_ARGS+=("$1" "$2"); shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 1 ;;
  esac
done

echo "== 第 1 段：运行时 + Provider（install-iraf.sh）"
PROFILE_ID="sim-x86"
HAS_PROFILE="0"
for ((i=0; i<${#INSTALL_ARGS[@]}; i++)); do
  if [ "${INSTALL_ARGS[$i]}" = "--profile" ]; then
    PROFILE_ID="${INSTALL_ARGS[$((i+1))]}"; HAS_PROFILE="1"
  fi
done
[ "${HAS_PROFILE}" = "1" ] || INSTALL_ARGS+=(--profile "${PROFILE_ID}")
bash "${SCRIPT_DIR}/install-iraf.sh" "${INSTALL_ARGS[@]}" || {
  rc=$?; echo "运行时/Provider 安装失败（rc=${rc}）" >&2; exit "${rc}"; }

if [ "${DRY}" = "1" ]; then
  echo "== 演练模式：跳过控制面（--dry-run 不写系统目录）"
  exit 0
fi

echo "== 第 2 段：控制面（Provider Supervisor）"
# token：没有就本地生成；但要提示必须与板卡一致
if [ ! -f "${TOKEN_FILE}" ]; then
  mkdir -p "$(dirname "${TOKEN_FILE}")"
  python3 -c "import secrets;print(secrets.token_urlsafe(32))" > "${TOKEN_FILE}"
  chmod 600 "${TOKEN_FILE}"
  echo "  已生成控制面 token：${TOKEN_FILE}"
  echo "  ⚠ 必须把它同步到板卡：ssh root@<板卡IP> \"cat > /etc/iraf/supervisor.token\" < ${TOKEN_FILE}"
fi

CP_ARGS=(--host-id "${HOST_ID}" --env-root "${HOME}/.config/iraf" --repo "${REPO_ROOT}")
if [ -n "${REGISTRY_DIR}" ]; then
  CP_ARGS+=(--registry-dir "${REGISTRY_DIR}")
elif [ -n "${FROM_BOARD}" ]; then
  CP_ARGS+=(--from-board "${FROM_BOARD}")
else
  # 默认用仓库自带的注册表（config/），离线也能装
  CP_ARGS+=(--registry-dir "${REPO_ROOT}/config")
fi
for a in "${ALLOW[@]:-}"; do [ -n "$a" ] && CP_ARGS+=(--allow "$a"); done

bash "${SCRIPT_DIR}/../install_control_plane.sh" "${CP_ARGS[@]}" || {
  rc=$?; echo "控制面安装失败（rc=${rc}）" >&2; exit 7; }

echo "== 完成：运行时 + Provider + 控制面均已安装"
echo "   下一步（板卡侧）：providers/*.yaml 加 hosts 条目 → registry_check --seal → 热加载验证"
exit 0