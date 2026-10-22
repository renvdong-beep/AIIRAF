#!/usr/bin/env bash
# 板卡侧诊断采集（IRAF 双域部署 · 调试入口）—— **只读**，把"这台板卡上 IRAF 现在什么状态"一次收齐。
#
# 为什么要有它：真机排障最贵的是"来回问一句、等一次登录"。本脚本一次拿走判读所需的全部事实，
# 并把"失败点"直接标出来（哪些期望不成立），而不是让人去翻日志猜。
#
# 采什么（全部只读；凭据不落盘、输出脱敏）：
#   ① 宿主事实：os-release / iaros-version / uname / python / glibc / systemd / 磁盘 / 负载
#   ② 已装 IRAF：版本目录、current 链接、安装记录摘要、env 文件（**脱敏**）、单元文件与状态
#   ③ 运行时：单元是否 active、journal 最近若干行、监听端口、相关进程
#   ④ 可执行性自检：用 env 文件跑 `iraf-agentos --list-skills`（能列出技能 = 运行期环境装配成功）
#   ⑤ 冲突面：AgentOS 自己的目录/端口/服务（我们不可碰的清单）
#   ⑥ 判读结论：逐条给出"期望 / 实测 / 是否成立"
#
# 用法（在**开发机**上执行，参数从环境变量来；密码不落盘）：
#   export IRAF_BOARD_HOST=10.203.247.74 IRAF_BOARD_USER=root IRAF_BOARD_NAME=e300-lab
#   read -rs -p '板卡密码: ' SSHPASS; export SSHPASS
#   bash deploy/sdk/board_diag.sh --profile edge-intel
#   # 证据：build/board/<name>/diag/<UTC 时间戳>/{summary.json, raw/*.txt}
#
# 退出码：0 全部读通（不代表 IRAF 已就绪）/ 1 参数或依赖错误 / 2 不可达 / 3 采集部分失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON="${PYTHON:-python3}"

HOST="${IRAF_BOARD_HOST:-}"
BOARD_USER="${IRAF_BOARD_USER:-root}"
SSH_PORT="${IRAF_BOARD_PORT:-22}"
BOARD_NAME="${IRAF_BOARD_NAME:-board}"
PROFILE="${IRAF_BOARD_PROFILE:-edge-intel}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

log() { printf '[board-diag] %s\n' "$*" >&2; }
err() { printf '[board-diag][错误] %s\n' "$*" >&2; }

usage() {
  cat <<'USAGE'
板卡侧诊断采集（只读）

用法：bash board_diag.sh [--profile <角色>] [--name <板卡名>]

环境变量：IRAF_BOARD_HOST（必填）、IRAF_BOARD_USER、IRAF_BOARD_PORT、SSHPASS（必填）、
          IRAF_BOARD_NAME、IRAF_BOARD_PROFILE

采到的证据：build/board/<name>/diag/<UTC 时间戳>/{summary.json, raw/*.txt}
退出码：0 全部读通 / 1 参数或依赖错误 / 2 不可达 / 3 采集部分失败
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --profile) PROFILE="$2"; shift ;;
    --name) BOARD_NAME="$2"; shift ;;
    *) err "无法识别的参数：$1（用 --help 查看用法）"; exit 1 ;;
  esac
  shift
done

[ -n "${HOST}" ] || { err "缺少 IRAF_BOARD_HOST"; exit 1; }
[ -n "${SSHPASS:-}" ] || { err "缺少 SSHPASS（板卡密码；只从环境变量传，勿写进命令历史）"; exit 1; }
command -v sshpass >/dev/null 2>&1 || { err "找不到 sshpass（apt install sshpass）"; exit 1; }

# 角色的生效布局（路径/端口/单元名全部来自声明，脚本内不写死）
LAYOUT="$("${PYTHON}" "${SCRIPT_DIR}/lib_install.py" layout --profile "${PROFILE}")"
read_layout() { "${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])[sys.argv[2]])" "${LAYOUT}" "$1"; }
PREFIX="$(read_layout prefix)"
CONFIG_DIR="$(read_layout config_dir)"
LOG_DIR="$(read_layout log_dir)"
DATA_DIR="$(read_layout data_dir)"
CURRENT="$(read_layout current_link)"
VERSION="$(read_layout version)"
UNIT="$("${PYTHON}" -c "import json,sys;print(json.loads(sys.argv[1])['residue_units'][0])" "${LAYOUT}")"
PYTHON_BIN="$(read_layout python_bin)"

OUT_DIR="${REPO_ROOT}/build/board/${BOARD_NAME}/diag/${STAMP}"
RAW_DIR="${OUT_DIR}/raw"
SSH_DIR="${REPO_ROOT}/build/board/${BOARD_NAME}/ssh"
mkdir -p "${RAW_DIR}" "${SSH_DIR}"

SSH_OPTS=(-o "StrictHostKeyChecking=accept-new" -o "UserKnownHostsFile=${SSH_DIR}/known_hosts"
          -o "ConnectTimeout=10" -o "BatchMode=no" -p "${SSH_PORT}")
TARGET="${BOARD_USER}@${HOST}"
PROBE_TIMEOUT="${IRAF_BOARD_PROBE_TIMEOUT:-40}"
remote() { SSHPASS="${SSHPASS}" timeout "${PROBE_TIMEOUT}" sshpass -e ssh "${SSH_OPTS[@]}" "${TARGET}" "$1"; }

log "目标 ${TARGET}｜角色 ${PROFILE}｜证据 ${OUT_DIR}"

# 可达性（带重试：实测板卡高负载时 ssh 认证会被饿死，属瞬时拥塞）
ok=0
for _ in 1 2 3; do
  if remote 'echo iraf-board-diag-ok' >/dev/null 2>&1; then ok=1; break; fi
  log "认证未成功，10 秒后重试（板卡负载高时常见）"; sleep 10
done
[ "${ok}" = "1" ] || { err "无法 ssh 到 ${TARGET}"; exit 2; }

FAILED=0
probe() {
  local id="$1" cmd="$2" rc=0
  if ! remote "${cmd}" > "${RAW_DIR}/${id}.txt" 2>&1; then rc=$?; FAILED=1; log "探针 ${id} 返回非零（rc=${rc}）"; fi
  sed -i -E 's/(password|passwd|token|secret|api[_-]?key)([=": ]+)[^"[:space:]]+/\1\2<redacted>/Ig' "${RAW_DIR}/${id}.txt"
  printf '%s\t%s\n' "${id}" "${rc}" >> "${OUT_DIR}/probe-status.tsv"
}
: > "${OUT_DIR}/probe-status.tsv"

# ① 宿主事实
probe host "cat /etc/os-release; echo ---; cat /etc/iaros-version 2>/dev/null || echo unverified; echo ---; uname -a; echo ---; uptime"
probe runtime "python3 -V; echo ---; ldd --version | head -1; echo ---; systemctl --version | head -1; echo ---; df -h / /opt 2>/dev/null | head -4; echo ---; free -m | head -2"

# ② 已装 IRAF（存在性 + 结构）
probe iraf_tree "ls -la '${PREFIX}' 2>/dev/null || echo 'NO-PREFIX'; echo ---;
  ls -la '${PREFIX}/${CURRENT}' 2>/dev/null || echo 'NO-CURRENT'; echo ---;
  ls '${PREFIX}/${VERSION}' 2>/dev/null | head -20 || true; echo ---;
  if [ -f '${PREFIX}/${VERSION}/iraf-install-files.json' ]; then head -40 '${PREFIX}/${VERSION}/iraf-install-files.json'; else echo 'NO-RECORD'; fi"
probe iraf_env "ls -la '${CONFIG_DIR}' 2>/dev/null || echo 'NO-CONFIG-DIR'; echo ---;
  sed -E 's/(TOKEN|SECRET|PASSWORD)=.*/\1=<redacted>/I' '${CONFIG_DIR}/${PROFILE}.env' 2>/dev/null || echo 'NO-ENV-FILE'"

# ③ 运行时
probe units "systemctl status '${UNIT}' --no-pager 2>&1 | head -12; echo ---;
  systemctl is-enabled '${UNIT}' 2>&1; echo ---;
  ls -la /etc/systemd/system/'${UNIT}' 2>&1"
probe journal "journalctl -u '${UNIT}' --no-pager -n 30 2>&1 | tail -30 || echo 'NO-JOURNAL'"
probe ports "ss -lntp 2>/dev/null | head -20"
probe processes "OUT=\$(ps -eo pid,etime,cmd 2>/dev/null | grep -iE 'iraf' | grep -v grep); if [ -n \"\$OUT\" ]; then echo \"\$OUT\"; else echo 'NO-IRAF-PROCESS'; fi"
probe logs "ls -la '${LOG_DIR}' 2>/dev/null || echo 'NO-LOG-DIR'; echo ---;
  tail -n 20 '${LOG_DIR}/control-provider-ready.log' 2>/dev/null || true"

# ④ 可执行性自检：用渲染出的 env 跑 CLI，能列出技能就说明运行期环境装配成功
# 可执行性自检：交付单元的 ExecStart 走的是**模块形式**（`python -m iraf_adapters.…`，因为
# PYTHONPATH=src:lib 的源码布局），所以这里也优先用模块形式；若存在 console script 则优先用它。
probe cli_list_skills "if [ ! -f '${CONFIG_DIR}/${PROFILE}.env' ]; then echo 'CLI-NOT-RUNNABLE: env 文件不存在';
  elif [ -x '${PREFIX}/${CURRENT}/lib/bin/iraf-agentos' ]; then set -a; . '${CONFIG_DIR}/${PROFILE}.env'; set +a;
    if [ -f '${CONFIG_DIR}/runtime.secrets' ]; then set -a; . '${CONFIG_DIR}/runtime.secrets'; set +a; fi;
    PYTHONPATH='${PREFIX}/${CURRENT}/src:${PREFIX}/${CURRENT}/lib' \
    '${PREFIX}/${CURRENT}/lib/bin/iraf-agentos' --list-skills 2>&1 | head -25;
  else set -a; . '${CONFIG_DIR}/${PROFILE}.env'; set +a;
    if [ -f '${CONFIG_DIR}/runtime.secrets' ]; then set -a; . '${CONFIG_DIR}/runtime.secrets'; set +a; fi;
    PYTHONPATH='${PREFIX}/${CURRENT}/src:${PREFIX}/${CURRENT}/lib' \
    '${PYTHON_BIN}' -m iraf_adapters.agentos.cli --list-skills 2>&1 | head -25; fi"

# ⑤ 冲突面：AgentOS 自己的东西（我们不可碰）
probe agentos "ls -d /etc/agentos /opt/IntewellAgent 2>/dev/null; echo ---;
  ss -lntp 2>/dev/null | grep -E ':(8083|9119|5432|8888)' | head -6 || echo 'NO-AGENTOS-PORT'"

# ⑥ 判读结论
"${PYTHON}" - "${OUT_DIR}" "${PROFILE}" "${PREFIX}" "${CURRENT}" "${VERSION}" "${CONFIG_DIR}" "${UNIT}" <<'PY'
import json, sys
from pathlib import Path

out, profile, prefix, current, version, config_dir, unit = sys.argv[1:8]
raw = Path(out) / "raw"


def text(name):
    path = raw / ("%s.txt" % name)
    return path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""


facts = {
    "host": text("host").splitlines()[:6],
    "executables": [
        {"name": "已装前缀（%s）" % prefix, "ok": "NO-PREFIX" not in text("iraf_tree")},
        {"name": "current 链接", "ok": "NO-CURRENT" not in text("iraf_tree")},
        # 正向证据：记录里必须真的出现安装记录的 schema 串（"没看到 NO-RECORD"不算过）
        {"name": "安装记录", "ok": '"schema_version": "iraf.install-record' in text("iraf_tree")},
        {"name": "env 文件（%s/%s.env）" % (config_dir, profile), "ok": "NO-ENV-FILE" not in text("iraf_env")},
        # 把"部署期还没填的必填项"与"装坏了"分开判：留空的 IRAF_PROFILE / IRAF_BACKEND_CONFIG
        # 是**待部署填写**（不是安装缺陷），但运行期一定起不来 ⇒ 单列一项，避免误判。
        {"name": "env 必填项已填（IRAF_PROFILE / IRAF_BACKEND_CONFIG）",
         "ok": any(line.startswith("IRAF_PROFILE=") and len(line) > len("IRAF_PROFILE=")
                   for line in text("iraf_env").splitlines())
               and any(line.startswith("IRAF_BACKEND_CONFIG=") and len(line) > len("IRAF_BACKEND_CONFIG=")
                       for line in text("iraf_env").splitlines())},
        {"name": "单元文件（%s）" % unit, "ok": "No such file" not in text("units")},
        {"name": "单元 active", "ok": "active (running)" in text("units") or "active (exited)" in text("units")},
        # 正向证据：有输出、且不含任何失败标记（否则"报错信息非空"会被误判成可运行 —— 实测踩到过）
        {"name": "CLI 可运行（--list-skills）",
         "ok": bool(text("cli_list_skills").strip())
               and not any(marker in text("cli_list_skills") for marker in
                           ("CLI-NOT-RUNNABLE", "No such file", "Traceback", "ModuleNotFoundError",
                            "command not found", "用法错误"))},
        {"name": "IRAF 进程存在", "ok": "NO-IRAF-PROCESS" not in text("processes")
                                          and "iraf" in text("processes").lower()},
    ],
    "agentos_untouched": {"agentos_port": "NO-AGENTOS-PORT" not in text("agentos")},
    "note": "本文件只做**判读**：ok=true 表示该期望成立，不表示 IRAF 能力已验收（真机验收另有证据）",
}
ready = all(item["ok"] for item in facts["executables"])
summary = {
    "schema_version": "iraf.board-diag/v1",
    "profile": profile,
    "target": {"host": None, "user": None},   # 地址由采集脚本掌握，不写进证据
    "collected_at_utc": Path(out).name,
    "iraf_ready": ready,
    "facts": facts,
    "raw_dir": "raw/",
}
Path(out, "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
print("[board-diag] IRAF 就绪=%s（逐项见 summary.json）" % ready)
for item in facts["executables"]:
    print("  %s %s" % ("✓" if item["ok"] else "✗", item["name"]))
PY

log "采集完成：${OUT_DIR}/summary.json（原始：${RAW_DIR}/）"
[ "${FAILED}" = "0" ] || { err "有探针返回非零（已留证）"; exit 3; }
exit 0
