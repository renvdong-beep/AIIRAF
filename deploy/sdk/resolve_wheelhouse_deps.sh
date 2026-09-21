#!/usr/bin/env bash
# wheelhouse 传递依赖闭包解析入口（x86 开发端；只读，产出候选清单）
#
# 职责：读声明矩阵 → 解析现有 wheelhouse 内每个 wheel 的 dist-info/METADATA
#       → 对目标环境（python_tag/platform_tag 声明）求值 marker → 递归解析缺失依赖
#       → 在索引上选出候选 wheel（只下载到候选暂存目录用于读 METADATA）
#       → 产出 candidates.json + summary.md（中文）。
#
# **本脚本不改 config/sdk/package_matrix.yaml，也不写正式 wheelhouse。**
# 回填声明必须由人工确认候选清单后执行（决策：候选清单交人工确认，不私自扩清单）。
#
# 用法：
#   bash deploy/sdk/resolve_wheelhouse_deps.sh --help
#   bash deploy/sdk/resolve_wheelhouse_deps.sh --target aarch64-manylinux_2_28-cp310
#   bash deploy/sdk/resolve_wheelhouse_deps.sh --target aarch64-manylinux_2_28-cp310 --offline
#
# 退出码（与 deploy/sdk 其它脚本同一约定）：
#   0 成功；1 参数/用法错误；2 预检失败（依赖/声明/目录缺失）；
#   3 解析失败（wheel 无 METADATA、Requires-Dist 无法解析、marker 变量未知、索引失败）；
#   4 校验失败（保留给 --verify，当前无校验模式）
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
matrix_default="config/sdk/package_matrix.yaml"
target=""
offline=0
output="build/iraf-24h/dep-closure"
python_bin="/usr/bin/python3"
extra_args=()

usage() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'TXT'

选项：
  --target <id>        矩阵中声明的构建目标 id（必填）
  --matrix <path>      矩阵路径（默认 config/sdk/package_matrix.yaml）
  --output <dir>       报告输出目录（默认 build/iraf-24h/dep-closure）
  --work-dir <dir>     候选 wheel 暂存目录（默认 <output>/candidate-wheels）
  --offline            不探测索引，只报告一级缺失
  --include-extra <n>  纳入指定 extra 的依赖（默认不纳入；可重复）
  --python <path>      解释器（默认 /usr/bin/python3）
  -h|--help            显示本帮助
TXT
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target) target="${2:-}"; shift 2 ;;
    --matrix) matrix_default="${2:-}"; shift 2 ;;
    --output) output="${2:-}"; shift 2 ;;
    --work-dir) extra_args+=(--work-dir "${2:-}"); shift 2 ;;
    --offline) offline=1; shift ;;
    --include-extra) extra_args+=(--include-extra "${2:-}"); shift 2 ;;
    --python) python_bin="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1（用 --help 查看用法）" >&2; exit 1 ;;
  esac
done

if [[ -z "$target" ]]; then
  echo "缺少 --target <id>：必须显式指定矩阵中声明的构建目标（不猜默认值）" >&2
  exit 1
fi

cd "$repo_root"

# 预检：解释器、packaging、PyYAML、矩阵、wheelhouse 目录
if [[ ! -x "$python_bin" ]]; then
  echo "解释器不可执行：$python_bin（用 --python 指定）" >&2
  exit 2
fi
if ! "$python_bin" -c "import packaging, yaml" >/dev/null 2>&1; then
  echo "缺少依赖：$python_bin 需要 packaging 与 PyYAML（本机：/usr/bin/python3 已具备）" >&2
  exit 2
fi
if [[ ! -f "$matrix_default" ]]; then
  echo "矩阵不存在：$matrix_default" >&2
  exit 2
fi
wheelhouse="build/wheelhouse/${target}"
if [[ ! -d "$wheelhouse" ]]; then
  echo "wheelhouse 目录不存在：$wheelhouse（先跑 fetch_wheelhouse.sh）" >&2
  exit 2
fi

echo "[resolve-wheelhouse-deps] 仓库根：$repo_root"
echo "[resolve-wheelhouse-deps] 目标：$target　矩阵：$matrix_default"
echo "[resolve-wheelhouse-deps] wheelhouse：$wheelhouse"
echo "[resolve-wheelhouse-deps] 模式：$([[ $offline -eq 1 ]] && echo offline || echo 'online（只读索引 + 候选下载到暂存目录）')"

cmd=("$python_bin" deploy/sdk/lib_wheelhouse_deps.py
     --matrix "$matrix_default" --target "$target" --output "$output")
[[ $offline -eq 1 ]] && cmd+=(--offline)
if [[ ${#extra_args[@]} -gt 0 ]]; then cmd+=("${extra_args[@]}"); fi

set +e
"${cmd[@]}"
rc=$?
set -e
exit "$rc"
