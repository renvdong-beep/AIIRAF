#!/usr/bin/env bash
# 板级安装入口（目标端执行；x86 开发端可用 --dry-run 在沙箱内做全流程演练）
#
# 流水线（设计 §5/§6，步骤 08）：
#   预检（bundle 校验和 → 解包与契约 → 声明 → 板卡门禁 → 宿主事实/磁盘/RAM/依赖/systemd）
#   → 版本化安装目录 → 离线安装 SDK wheel → 目标端二进制 wheelhouse → 写 env
#   → 符号链接切换 → 后置 profile-check → 后置 verify.sh → 注册并启动 systemd 单元
#
# 用法：
#   bash install.sh --help
#   bash install.sh --bundle build/sdk/iraf-board-e300-0.2.0-aarch64.tar.gz --dry-run --root <沙箱目录>
#   bash install.sh --bundle <tar.gz> --allow-unverified      # 目标端真实安装
#
# 退出码（设计 §5 统一约定）：
#   0 成功 / 演练完成（演练 ≠ 已安装）
#   1 参数错误
#   2 预检失败（声明未实测、架构/Python/glibc/磁盘/RAM/依赖/systemd 不符、bundle 缺校验和）
#   3 安装失败（IO/权限/依赖安装失败）：已回滚
#   4 校验失败（SHA-256 不符、bundle 成员被篡改、已装目录与 bundle 不一致）
#   5 后置校验失败（profile-check / verify.sh / 服务启动失败）：不激活新版本，回滚激活链接
#
# 声明来源：bundle 内 config/sdk/package_matrix.yaml 的 delivery.install（安装布局）。
# 实现层：deploy/sdk/lib_board_bundle.py（随 bundle 交付到 <version>/scripts/）。
# 解释器：默认取 PATH 中的 python3；可用 --python 指定目标端解释器。
#
# 硬边界（不得违反）：
#   * 演练（--dry-run）必须显式给 --root 沙箱目录；退出码 0 只表示演练流程完成，
#     报告里 evidence_scope=rehearsal_only、real_install_allowed=false（存在 blockers），
#     **禁止**把演练结果表述为"已安装"或"板卡可用"。
#   * 演练不执行 systemctl、不安装目标端二进制 wheel（aarch64 wheel 不能在 x86_64 上安装），
#     这些阶段在报告里标记 DEFERRED，目标端执行同一命令。
#   * profile-check 失败即不启动服务；后置校验脚本缺失即安装失败并回滚，禁止静默跳过。
set -euo pipefail

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"

PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_board_bundle.py"
BUNDLE=""
ROOT=""
ALLOW_UNVERIFIED="0"
DRY_RUN="0"
EXTRA=()

usage() {
  cat <<'USAGE'
板级安装入口（目标端；IRAF iraf-24h 步骤 08）

用法：bash install.sh --bundle <板级 bundle tar.gz> [选项]

选项：
  -h, --help             显示本帮助并退出（退出码 0）
  -n, --dry-run          沙箱演练：必须同时给 --root；不写真实系统目录、不启动服务
  --bundle <tar.gz>      板级 bundle（必填），需同目录存在 <bundle>.sha256
  --root <dir>           文件系统根：演练必填（沙箱目录）；真实安装默认 /（即真实系统路径）
  --allow-unverified     真实安装时显式放行"板卡声明未实测"（不放行契约层失败与事实不符）
  --python <exe>         目标端解释器（默认 python3）
  --json-out <path>      额外把安装报告写一份到该路径（默认只写 <root>/<log_dir>/）
  --repo-root <dir>      仅调试用：覆盖实现层默认仓库根

环境变量：PYTHON

退出码：0 成功/演练完成 / 1 参数错误 / 2 预检失败 / 3 安装失败（已回滚）
        4 校验失败 / 5 后置校验失败（不激活，已回滚）

说明：本机是 x86_64 开发端、目标板卡不在场（x86-first 战役）。真实安装在开发端会被
      预检明确拒绝（架构/Python 标签/声明未实测），这是 fail-closed 的预期行为；
      目标端安装与 /health 验收记为 DEFERRED，不得用演练结果替代。
USAGE
}

log() { printf '[install] %s\n' "$*" >&2; }
err() { printf '[install][错误] %s\n' "$*" >&2; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -n|--dry-run)
      DRY_RUN="1"
      ;;
    --allow-unverified)
      ALLOW_UNVERIFIED="1"
      ;;
    --bundle)
      [ "$#" -ge 2 ] || { err "--bundle 缺少取值"; exit 1; }
      BUNDLE="$2"; shift
      ;;
    --root)
      [ "$#" -ge 2 ] || { err "--root 缺少取值"; exit 1; }
      ROOT="$2"; shift
      ;;
    *)
      case "$1" in
        --python|--json-out|--repo-root)
          [ "$#" -ge 2 ] || { err "$1 缺少取值"; exit 1; }
          EXTRA+=("$1" "$2"); shift
          ;;
        *)
          err "无法识别的参数：$1（用 --help 查看用法）"
          exit 1
          ;;
      esac
      ;;
  esac
  shift
done

if [ -z "${BUNDLE}" ]; then
  err "缺少 --bundle（板级 bundle tar.gz）"
  exit 1
fi
if [ "${DRY_RUN}" = "1" ] && [ -z "${ROOT}" ]; then
  err "演练模式必须显式给出 --root（沙箱目录），不得落到真实系统路径"
  exit 1
fi
if [ "${DRY_RUN}" != "1" ] && [ -z "${ROOT}" ]; then
  ROOT="/"
fi
[ -f "${LIB}" ] || { err "缺少实现层 ${LIB}（本脚本必须与 lib_board_bundle.py 同目录）"; exit 3; }
command -v "${PYTHON}" >/dev/null 2>&1 || {
  err "找不到解释器 ${PYTHON}（可用 PYTHON=/usr/bin/python3 指定）"
  exit 2
}

ARGS=(--bundle "${BUNDLE}" --root "${ROOT}" --python "${PYTHON}")
if [ "${ALLOW_UNVERIFIED}" = "1" ]; then ARGS+=(--allow-unverified); fi
if [ "${DRY_RUN}" = "1" ]; then ARGS+=(--dry-run); fi
if [ "${#EXTRA[@]}" -gt 0 ]; then ARGS+=("${EXTRA[@]}"); fi

if [ "${DRY_RUN}" = "1" ]; then
  log "模式：dry-run 沙箱演练（root=${ROOT}；不写真实系统目录、不启动服务、不打假证据）"
else
  log "模式：真实安装（root=${ROOT}）"
fi

# `if ! cmd; then rc=$?` 取反后 $? 恒为 0，会吞掉退出码：这里显式保存。
set +e
OUT="$("${PYTHON}" "${LIB}" install "${ARGS[@]}")"
rc=$?
set -e
printf '%s\n' "${OUT}"
if [ "${rc}" -ne 0 ]; then
  err "安装未通过（退出码 ${rc}）：详见上面的 JSON 报告（stages / blockers / failure）"
  exit "${rc}"
fi
if [ "${DRY_RUN}" = "1" ]; then
  log "演练完成（退出码 0）：这不代表已安装，也不构成目标端证据"
else
  log "安装完成（退出码 0）"
fi
exit 0
