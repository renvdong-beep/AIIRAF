#!/usr/bin/env bash
# 安装记录漂移检查（只读）：回答"记录里的 env/凭据是否还和落盘一致"。
#
# 为什么要有它（2026-11-06 实测）：运维手工改过 /etc/iraf/*.env 后，受控卸载会被
# "配置文件哈希不符"拒绝；如果等到卸载时才发现，现场已无法判断该保哪份配置。
# 本入口在**升级前/排障时**先给结论，不删任何文件。
#
# 用法：
#   bash deploy/sdk/verify_record.sh --profile edge-intel
#   bash deploy/sdk/verify_record.sh --profile sim-x86 --json
#
# 退出码：0 一致 / 1 参数错误 / 2 声明失败 / 4 漂移（受控卸载会被拒绝）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"
LIB="${SCRIPT_DIR}/lib_install.py"

# 目标端：安装树自带 pyyaml/jsonschema 与 scripts/install_profile_check.py；
# 仓库开发态没有 <repo>/lib，这两条 PYTHONPATH 只是无害的追加。
export PYTHONPATH="${REPO_ROOT}/lib:${REPO_ROOT}/src:${PYTHONPATH:-}"

[ -f "${LIB}" ] || { echo "[verify-record][错误] 缺少实现层 ${LIB}" >&2; exit 3; }

export IRAF_REPO_ROOT="${REPO_ROOT}"
set +e
"${PYTHON}" "${LIB}" record-check "$@"
rc=$?
set -e
if [ "${rc}" -eq 0 ]; then
  echo "[verify-record] 记录与落盘配置一致" >&2
else
  echo "[verify-record][错误] 记录漂移（退出码 ${rc}）：见上面的逐项清单" >&2
fi
exit "${rc}"
