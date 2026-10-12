#!/usr/bin/env bash
# 板卡只读采集（IRAF 交付 P1）—— 为回填 BoardProfile/HyperProfile 提供实测事实。
#
# 用途：在开发机上对**边缘板卡**执行一组**只读**命令，把原始输出与索引落盘，供
#       scripts/board_collect_summary.py 生成 profiles/boards/<board>.yaml 的回填补丁。
#
# 硬边界（不得违反）：
#   * 本脚本**只读**：不安装、不写盘、不改服务、不改配置。唯一的外部副作用是往本地
#     build/board/<name>/ 落证据（build/ 已被 .gitignore 忽略）。
#   * 凭据只从环境变量来：SSHPASS（密码）与 IRAF_BOARD_HOST/USER/PORT。**不得**写进仓库、
#     不得出现在命令行参数里（用 sshpass -e 从环境读，避免密码进入 ps/历史）。
#   * 输出的 env/config 内容会做**脱敏**（password/token/secret 值替换为 <redacted>）。
#   * 采集不到的事实一律写 unverified，禁止猜测（AGENTS 3.2）。
#
# 用法：
#   export IRAF_BOARD_HOST=10.203.247.74 IRAF_BOARD_USER=root IRAF_BOARD_NAME=e300-lab
#   read -rs -p '板卡密码: ' SSHPASS; export SSHPASS
#   bash scripts/board_collect.sh
#   # 证据：build/board/<name>/{collect.json, raw/*.txt, ssh/known_hosts}
#
# 退出码：0 成功 / 1 参数或依赖错误 / 2 采集预检失败（不可达、认证失败）/ 3 采集过程失败
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

HOST="${IRAF_BOARD_HOST:-}"
BOARD_USER="${IRAF_BOARD_USER:-root}"
SSH_PORT="${IRAF_BOARD_PORT:-22}"
BOARD_NAME="${IRAF_BOARD_NAME:-board}"
CONNECT_TIMEOUT="${IRAF_BOARD_CONNECT_TIMEOUT:-8}"

log() { printf '[board-collect] %s\n' "$*" >&2; }
err() { printf '[board-collect][错误] %s\n' "$*" >&2; }

[ -n "${HOST}" ] || { err "缺少 IRAF_BOARD_HOST（板卡 IP/主机名）"; exit 1; }
[ -n "${SSHPASS:-}" ] || { err "缺少 SSHPASS（板卡密码，从环境变量传入，勿写进命令历史）"; exit 1; }
command -v ssh >/dev/null 2>&1 || { err "找不到 ssh"; exit 1; }
command -v sshpass >/dev/null 2>&1 || { err "找不到 sshpass（apt install sshpass）"; exit 1; }

OUT_DIR="${REPO_ROOT}/build/board/${BOARD_NAME}"
RAW_DIR="${OUT_DIR}/raw"
SSH_DIR="${OUT_DIR}/ssh"
mkdir -p "${RAW_DIR}" "${SSH_DIR}"

SSH_OPTS=(-o "StrictHostKeyChecking=accept-new" -o "UserKnownHostsFile=${SSH_DIR}/known_hosts"
          -o "ConnectTimeout=${CONNECT_TIMEOUT}" -o "BatchMode=no" -p "${SSH_PORT}")
SSH_TARGET="${BOARD_USER}@${HOST}"

# 单次远端命令执行（只读）。stdout 原样返回，失败即报错。
# 每条命令**本地加超时**（远端 `docker ps` 之类可能挂住 —— 实测踩过一次，整轮采集被 300s 兜底杀掉）：
# 超时按 rc=124 留证，不中断其余探针。
PROBE_TIMEOUT="${IRAF_BOARD_PROBE_TIMEOUT:-30}"
remote() { SSHPASS="${SSHPASS}" timeout "${PROBE_TIMEOUT}" sshpass -e ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "$1"; }

# 采集一个探针：id → 远端命令；输出落 raw/<id>.txt，同时做脱敏。
# 单个探针失败**不中断**整体采集（远端 grep 无命中就会返回 1，那是"事实为空"而不是"采集坏了"）；
# 但状态必须逐项留证：raw/<id>.txt 的字节数 + 退出码进 collect.json。
STATUS_FILE="${OUT_DIR}/probe-status.tsv"
: > "${STATUS_FILE}"
probe() {
  local id="$1"; shift
  local cmd="$1"
  local target="${RAW_DIR}/${id}.txt"
  local rc=0
  if ! remote "${cmd}" > "${target}" 2>&1; then
    rc=$?
    log "探针 ${id} 返回非零（rc=${rc}）——已留证，不中断采集"
  fi
  # 脱敏：键值型凭据一律替换（保留键名，便于判断"声明在哪"）
  sed -i -E 's/(password|passwd|token|secret|api[_-]?key)([=": ]+)[^"[:space:]]+/\1\2<redacted>/Ig' "${target}"
  printf '%s\t%s\t%s\n' "${id}" "${rc}" "$(wc -c < "${target}")" >> "${STATUS_FILE}"
  log "已采集 ${id}（rc=${rc}，$(wc -c < "${target}") 字节）"
}

log "目标：${SSH_TARGET}:${SSH_PORT}；证据目录：${OUT_DIR}"

# --- 0) 可达性与认证（fail-closed：这里失败不继续采集；带重试，因为实测板卡负载 6.2/4 核时
#        ssh 认证会被饿死、偶发超时，重试即可恢复 —— 重试不算"绕过判据"，只处理瞬时拥塞） ---
auth_ok=0
for attempt in 1 2 3; do
  if remote 'echo iraf-board-collect-ok' > /dev/null 2>&1; then
    auth_ok=1
    break
  fi
  log "第 ${attempt} 次认证未成功（可能是负载导致的瞬时超时），10 秒后重试"
  sleep 10
done
if [ "${auth_ok}" != "1" ]; then
  err "无法通过 ssh 认证/连接 ${SSH_TARGET}（检查 IP、密码、ssh 服务，或板卡负载）"
  exit 2
fi

# --- A) 身份与版本 ---
probe load 'uptime; echo ---; top -bn1 2>/dev/null | head -15'
probe identity 'id; echo ---; uname -a; echo ---; systemd-detect-virt; echo ---; cat /proc/cmdline'
probe os_release 'cat /etc/os-release; echo ---; cat /etc/back.os-release 2>/dev/null || true'
probe iaros_version 'cat /etc/iaros-version 2>/dev/null || echo unverified'

# --- B) 运行时（决定 python_tag / platform_tag / glibc_min） ---
probe runtime 'python3 -V; echo ---; command -v python3 pip3 python3.10 python3.11 || true; echo ---;
  pip3 -V 2>/dev/null || true; echo ---; ldd --version | head -1; echo ---; systemctl --version | head -1;
  echo ---; command -v node npm docker containerd || true; echo ---; node -v 2>/dev/null || true'

# --- C) 资源（安装预检要用：RAM/磁盘/CPU） ---
probe resources 'nproc; echo ---; free -m; echo ---; df -h / /opt /var /tmp 2>/dev/null; echo ---;
  lsblk -o NAME,SIZE,TYPE,MOUNTPOINT 2>/dev/null | head -20'

# --- D) AgentOS 现状（它先占的目录/端口/服务，是我们"不可碰"的清单） ---
probe agentos_files 'ls -la /etc/agentos 2>/dev/null; echo ---; ls -la /opt/agentos 2>/dev/null | head;
  echo ---; ls /opt/IntewellAgent 2>/dev/null; echo ---; ls /opt/IntewellAgent/agentbase/bin 2>/dev/null | grep -i intewell'
probe agentos_config 'cat /etc/agentos/host-worker.env 2>/dev/null; echo ---; cat /etc/agentos/host-bindings.json 2>/dev/null'
probe agentos_services 'systemctl list-units --type=service --state=running --no-pager 2>/dev/null | head -40;
  echo ---; systemctl list-unit-files --no-pager 2>/dev/null | grep -iE "agentos|intewell|postgres|docker" | head -20'

# --- E) IRAF 是否已存在（卸载/重装时要知道"有没有装过"） ---
probe iraf_presence 'ls -la /opt/iraf-sdk /etc/iraf /var/lib/iraf /var/log/iraf 2>/dev/null || true;
  echo ---; command -v iraf-agentos iraf-runtime-http iraf-runtime-grpc || echo "no-iraf-console-scripts";
  echo ---; systemctl list-unit-files --no-pager 2>/dev/null | grep -i iraf || echo "no-iraf-units"'

# --- F) 监听端口（端口冲突判据；也是"哪些服务真的在跑"的证据） ---
probe listen_ports 'ss -lntp 2>/dev/null | head -30 || netstat -lntp 2>/dev/null | head -30'

# --- G) 域/虚拟机线索（智能域 vs 运控域：Hyper 的角色映射只能从这里读，禁止猜） ---
probe domains 'ls /dev 2>/dev/null | grep -iE "rtos|hyper|vm|musa|shm" | head -20; echo ---;
  ls /opt/musa 2>/dev/null | head -20; echo ---; dmesg 2>/dev/null | grep -iE "hyper|intewell|aros|vm[0-9]" | head -20'

# --- H) 签名 HyperProfile / Release manifest 搜索（铁律 6.10 的唯一合法来源） ---
probe manifests 'find /etc /opt /usr/share /var/lib -maxdepth 4 \( -iname "*packaging_config*" -o -iname "*hyperprofile*" -o -iname "*hyper*profile*" -o -iname "*release*manifest*" -o -iname "*ariadne*" -o -iname "*iaros*" \) -not -path "*/proc/*" 2>/dev/null | head -40'

# --- J) AgentOS 落地形态（我们要绑定的对象；也是"不可碰"清单） ---
probe agentos_cli 'ls /opt/IntewellAgent/agentbase/optional-mcps 2>/dev/null | head -20; echo ---;
  /opt/IntewellAgent/agentbase/bin/intewell-agent-cli --help 2>&1 | head -40'
probe agentos_runtime 'ps -eo pid,ppid,etime,cmd 2>/dev/null | grep -iE "intewell|agentos" | grep -v grep | head -20; echo ---;
  PID=$(ss -lntp 2>/dev/null | grep -F ":8083" | grep -oE "pid=[0-9]+" | head -1 | cut -d= -f2);
  echo "agentos_health_pid=${PID:-unknown}";
  if [ -n "${PID:-}" ] && [ -r "/proc/${PID}/cmdline" ]; then
    echo "--- cmdline:"; tr "\0" " " < "/proc/${PID}/cmdline"; echo;
    echo "--- env(过滤):"; tr "\0" "\n" < "/proc/${PID}/environ" | grep -iE "^(LD_LIBRARY_PATH|PYTHONPATH|PYTHONHOME|PATH|AGENTOS_|INTEWELL_)" | head -20;
  fi;
  echo ---; ls -d /opt/agentos /var/lib/agentos-host-worker 2>/dev/null || echo "no /opt/agentos";
  echo ---; ls /opt/IntewellAgent/agentbase/bin/python* 2>/dev/null || echo "no-agentbase-python";
  echo ---; ls -d /opt/IntewellAgent/agentbase/lib/python3.10 2>/dev/null || true;
  ls /opt/IntewellAgent/agentbase/lib 2>/dev/null | grep -i "libpython" | head -5 || echo "no-libpython-in-agentbase"'

# docker 单独一条并**短超时**：实测 docker ps 可能挂住（整轮采集被兜底杀掉一次）
probe docker_state 'timeout 8 docker ps --format "{{.Names}}|{{.Image}}|{{.Status}}|{{.Ports}}" 2>&1 | head -20;
  echo "docker_ps_rc=$?"'

# --- L) Hyper 宿主侧产物（/download 是 dmesg 里 vdisk 的路径；VM 角色映射的证据源） ---
probe download_tree 'ls -la /download 2>/dev/null | head -20; echo ---; ls /download/image 2>/dev/null | head -20;
  echo ---; find /download -maxdepth 3 -iname "*.json" -o -maxdepth 3 -iname "*.yaml" 2>/dev/null | head -20'

# --- M) 系统健康（实测踩到：load 6.3 但 CPU 97.6% 空闲 + 30 僵尸 ⇒ 必须留证，
#        否则后面"安装失败/超时"会被误判成我们的脚本问题） ---
probe health 'echo "僵尸数: $(ps -eo stat= | grep -c Z)"; echo ---;
  ps -eo pid,ppid,stat,etime,comm 2>/dev/null | awk "\$3 ~ /^Z/" | head -20; echo ---;
  echo "D 状态（不可中断 I/O）:"; ps -eo pid,ppid,stat,etime,comm 2>/dev/null | awk "\$3 ~ /^D/" | head -20'

# --- N) 域间网络（要装到运控域就必须知道它怎么寻址；只能实测，禁止猜 IP） ---
probe net_topology 'ip -br a 2>/dev/null | head -20; echo ---; ip -br l 2>/dev/null | grep -iE "virtio|tap|veth|bridge|br[0-9]" | head -20;
  echo ---; ls /sys/class/net 2>/dev/null | tr "\n" " "; echo'

# --- O) ROS 2 / 运控域能力证据（BoardProfile.runtime.ros2 的唯一来源） ---
probe ros2 'ls -d /opt/ros/* 2>/dev/null || echo "no-/opt/ros"; echo ---;
  command -v ros2 rviz2 colcon 2>/dev/null || echo "no-ros2-on-path"; echo ---;
  ls /etc/profile.d 2>/dev/null | grep -i ros || true'

# --- K) NPU / 媒体能力（BoardProfile.adapters.npu 的证据来源） ---
probe npu_media 'ls /opt/musa 2>/dev/null | head -12; echo ---; ls /dev 2>/dev/null | grep -iE "xh2|houmo|npu|m1000|dri|video" | head -12 || echo no-npu-dev'

# --- I) 离线安装能力（形态 B 走 wheelhouse：先确认能不能在线抓、或必须离线） ---
probe network 'command -v curl wget python3 || true; echo ---;
  timeout 6 curl -sS -o /dev/null -w "aliyun_http=%{http_code}\n" https://mirrors.aliyun.com/pypi/simple/ 2>&1;
  echo "curl_rc=$?"; echo ---;
  timeout 6 curl -sS -o /dev/null -w "pypi_http=%{http_code}\n" https://pypi.org/simple/ 2>&1;
  echo "curl_rc=$?"; echo ---;
  timeout 6 curl -sS -o /dev/null -w "newapi_http=%{http_code}\n" http://192.168.100.89:3000/v1/models 2>&1;
  echo "curl_rc=$?"'

# --- 汇总 collect.json（索引 + 顶层事实；解析交给 board_collect_summary.py） ---
python3 - "${OUT_DIR}" "${HOST}" "${BOARD_USER}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

out = Path(sys.argv[1])
host, board_user = sys.argv[2], sys.argv[3]
raw = out / "raw"

probes = []
for line in (out / "probe-status.tsv").read_text(encoding="utf-8").splitlines():
    if not line.strip():
        continue
    pid, rc, size = line.split("\t")
    probes.append({"id": pid, "returncode": int(rc), "bytes": int(size),
                   "raw": str((raw / ("%s.txt" % pid)).relative_to(out))})


def lines_of(name):
    path = raw / ("%s.txt" % name)
    if not path.is_file():
        return []
    return [line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.strip()]


identity = lines_of("identity")
report = {
    "schema_version": "iraf.board-collect/v1",
    "collected_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    "simulation": False,
    "target": {"host": host, "user": board_user, "transport": "ssh"},
    "method": "只读采集：不安装、不写盘、不改服务；凭据来自环境变量，输出已脱敏",
    "probes": probes,
    "headline": {
        "identity": identity[:4],
        "iaros_version": lines_of("iaros_version")[:6],
        "os_pretty_name": [line for line in lines_of("os_release") if line.startswith("PRETTY_NAME")],
        "python": lines_of("runtime")[:1],
        "glibc": [line for line in lines_of("runtime") if line.startswith("ldd")],
        "systemd": [line for line in lines_of("runtime") if line.startswith("systemd")],
        "cpu_cores": lines_of("resources")[:1],
    },
    "notes": [
        "本文件只是索引与顶层事实；写入 profiles/boards/*.yaml 前必须逐项复核 raw/ 原文（AGENTS 3.2：未知写 unverified，禁止猜测）",
        "probes[].returncode 非 0 表示该条命令返回非零（常见：远端 grep 无命中），其输出仍已留证",
    ],
}
(out / "collect.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(report["headline"], ensure_ascii=False, indent=2))
PY

log "采集完成：${OUT_DIR}/collect.json（原始证据：${RAW_DIR}/）"
log "下一步：python3 scripts/board_collect_summary.py --board-dir ${OUT_DIR}（生成 BoardProfile 回填补丁）"
