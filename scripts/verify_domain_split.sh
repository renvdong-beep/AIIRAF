#!/usr/bin/env bash
# 双域容器化验证：控制域（Capability Provider 替身）↔ 智能域（TaskFlow + SkillRuntime + Policy）
#
# 本脚本是 docs/deploy-topology-and-domain-split.md §6 里"待实现"的那个入口。
# 它把整条链路做成一条可复跑命令：起容器 → 同步代码 → 装依赖 → 生成 proto stub → 起服务端
# → 智能域发起四例 → 收证据。**不改 IRAF 框架的公共契约**（跨域只用 api/proto 的 gRPC 接口）。
#
# 四例（判据写在智能域客户端里，本脚本只负责编排与收证）：
#   ① normal      stand 全链成功（证据满足 skill 输出 schema）
#   ② denied      Profile 摘要不匹配 ⇒ IRAF-POLICY-DENIED
#   ③ cancel      执行中取消 ⇒ CANCELLED（服务端 GetExecution 复核，并证控制域执行了安全停机）
#   ④ unreachable 控制域不可达 ⇒ IRAF-EXECUTION-FAILED
#
# 用法（仓库根）：
#   IRAF_GRPC_TOKEN=<部署下发的凭据> bash scripts/verify_domain_split.sh [--no-sync] [--out <dir>]
#
# 退出码：0 四例全通过 / 1 有不符合项 / 2 前置或环境错误 / 3 编排失败
#
# ⚠ 必须声明的局限（写进报告，不得含糊；见该文档 §6）
#   · 容器共享同一内核与本机时钟 ⇒ **不能**证明实时性（周期抖动/看门狗/确定性），那只属于 HIL / 真机
#   · 容器不是板卡 ⇒ 不得作为 e300 / firefly_rk3588 从 unverified → verified 的证据
#   · 在 x86 上跑 aarch64 rootfs（qemu-user-static）⇒ 只证"装得上、连得通"，不得当性能证据
#   · 智能域容器**共享控制域的网络命名空间**：因为开发 gRPC 适配器（server.py）按安全设计
#     **只允许绑定回环**，跨容器回环要通只能共享 netns。这是实验室捷径，
#     **不构成网络隔离证据**；生产用 mTLS + 真实接口 + 独立 netns。禁止为图方便去放开那条绑定限制。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMG="${IRAF_DOMAIN_IMAGE:-rootfs_openeuler_24.03-lts-sp1_aarch64:latest}"
CTL="${IRAF_CONTROL_CONTAINER:-iraf-control-domain}"
INTEL="${IRAF_INTEL_CONTAINER:-iraf-intel-domain}"
RUNTIME_DIR="/opt/iraf"
OUT_DIR="${REPO_ROOT}/build/acceptance/domain-split"
TOKEN="${IRAF_GRPC_TOKEN:-}"
DO_SYNC=1
# 每次运行的独立日志/账本目录：审计账是**追加**写的，若复用同一路径，上一轮的行会留在里面，
# 审计时无法区分"这轮的结果"与"上轮的历史"（本轮就踩过：修好延迟前的两行残留在账里，
# execution_id 为空且相隔 4 s，看上去像新结果）。用运行目录而不是删旧文件：可复现且无副作用。
RUN_ID="${IRAF_RUN_ID:-$(date +%Y%m%d-%H%M%S)}"
RUN_LOGS="${RUNTIME_DIR}/logs/run-${RUN_ID}"

PROFILE_REL="profiles/unitree_go2_mujoco.yaml"
SAFETY_REL="profiles/safety/quadruped_lab.yaml"
SKILL_ROOT_REL="skills"

log() { printf '[双域验证] %s\n' "$*"; }
err() { printf '[双域验证][错误] %s\n' "$*" >&2; }
die() { err "$*"; exit "${2:-3}"; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '1,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
    --no-sync) DO_SYNC=0 ;;
    --out) [ "$#" -ge 2 ] || die "--out 缺少取值" 2; OUT_DIR="$2"; shift ;;
    --token) [ "$#" -ge 2 ] || die "--token 缺少取值" 2; TOKEN="$2"; shift ;;
    *) die "未知参数：$1（--help 查看用法）" 2 ;;
  esac
  shift
done

# ---------- 步骤 0：前置检查（显式失败，不做半成品） ----------
command -v docker >/dev/null 2>&1 || die "找不到 docker" 2
docker image inspect "${IMG}" >/dev/null 2>&1 || die "镜像不存在：${IMG}（先准备 openEuler 24.03 rootfs 镜像）" 2
if [ -z "${TOKEN}" ]; then
  die "缺少部署凭据：请用 IRAF_GRPC_TOKEN=<token> 或 --token <token> 提供（服务端拒绝无凭据启动，本脚本不设默认值）" 2
fi
# 跨架构：x86 宿主跑 aarch64 rootfs 需要 binfmt。已注册则跳过（一次即可）。
if [ "$(uname -m)" != "aarch64" ]; then
  if ! ls /proc/sys/fs/binfmt_misc/qemu-aarch64 >/dev/null 2>&1; then
    log "注册 qemu binfmt（x86 宿主跑 aarch64 必需，只需一次）"
    docker run --privileged --rm multiarch/qemu-user-static --reset -p yes >/dev/null 2>&1 \
      || die "binfmt 注册失败：请确认可访问 multiarch/qemu-user-static 镜像（或宿主机已装 qemu-user-static）" 2
  fi
fi

mkdir -p "${OUT_DIR}"
EVID_LOG="${OUT_DIR}/run.log"
: > "${EVID_LOG}"
exec > >(tee -a "${EVID_LOG}") 2>&1
log "证据目录：${OUT_DIR}"

# ---------- 步骤 1：容器（幂等） ----------
ensure_container() {
  local name="$1" netmode="$2"
  local current
  current="$(docker inspect "${name}" --format '{{.State.Status}}' 2>/dev/null || echo missing)"
  if [ "${current}" != "missing" ]; then
    if [ "${current}" != "running" ]; then
      docker start "${name}" >/dev/null
    fi
    local mode
    mode="$(docker inspect "${name}" --format '{{.HostConfig.NetworkMode}}')"
    if [ "${mode}" != "${netmode}" ]; then
      log "容器 ${name} 网络模式为 ${mode}，与要求 ${netmode} 不符 ⇒ 按本脚本的编排重建（容器由本脚本持有）"
      docker rm -f "${name}" >/dev/null
      current="missing"
    fi
  fi
  if [ "${current}" = "missing" ]; then
    if [ "${netmode}" = "bridge" ]; then
      docker run -d --name "${name}" "${IMG}" sleep infinity >/dev/null
    else
      docker run -d --name "${name}" --network "${netmode}" "${IMG}" sleep infinity >/dev/null
    fi
    log "已创建容器 ${name}（网络：${netmode}）"
  else
    log "复用容器 ${name}（网络：${netmode}）"
  fi
}

log "== 步骤 1 起/复用两个域容器"
ensure_container "${CTL}" "bridge"
ensure_container "${INTEL}" "container:${CTL}"

# ---------- 步骤 2：同步代码 ----------
if [ "${DO_SYNC}" = "1" ]; then
  log "== 步骤 2 同步代码（src/skills/profiles/config/api/scripts）"
  SYNC_TGZ="$(mktemp -t iraf-domain-sync-XXXXXX.tgz)"
  tar -czf "${SYNC_TGZ}" -C "${REPO_ROOT}" \
    --exclude='__pycache__' --exclude='*.pyc' \
    src skills profiles config api scripts
  for c in "${CTL}" "${INTEL}"; do
    docker exec "${c}" sh -c "mkdir -p ${RUNTIME_DIR}"
    docker cp "${SYNC_TGZ}" "${c}:${RUNTIME_DIR}/sync.tgz" >/dev/null
    docker exec "${c}" sh -c "cd ${RUNTIME_DIR} && tar -xzf ${RUNTIME_DIR}/sync.tgz"
  done
fi

# ---------- 步骤 3：装依赖（幂等） ----------
log "== 步骤 3 容器内安装依赖（aarch64 轮子；控制域不装仿真依赖）"
for c in "${CTL}" "${INTEL}"; do
  docker exec "${c}" sh -c 'mkdir -p /opt/iraf/logs; python3 -m pip install --quiet pyyaml jsonschema grpcio protobuf grpcio-tools' \
    || die "容器 ${c} 安装依赖失败（检查网络/镜像源）" 3
  docker exec "${c}" python3 -c "import yaml, jsonschema, grpc, grpc_tools, google.protobuf; print('  依赖 OK：%s' % '${c}')" \
    || die "容器 ${c} 依赖自检失败" 3
done

# ---------- 步骤 4：生成 proto stub（补上 build_sdk.sh 声明为"不可复现"的 grpc 层） ----------
log "== 步骤 4 生成 iraf.v1 proto stub（消息层 + grpc 层）"
docker exec "${CTL}" sh -c "cd ${RUNTIME_DIR} && bash scripts/gen_proto_stubs.sh ${RUNTIME_DIR}/proto" \
  || die "proto stub 生成失败（控制域）" 3
docker exec "${CTL}" sh -c "tar -czf /tmp/iraf-proto.tgz -C ${RUNTIME_DIR} proto"
docker cp "${CTL}:/tmp/iraf-proto.tgz" "${OUT_DIR}/proto-stubs.tgz" >/dev/null
docker cp "${OUT_DIR}/proto-stubs.tgz" "${INTEL}:${RUNTIME_DIR}/proto.tgz" >/dev/null
docker exec "${INTEL}" sh -c "cd ${RUNTIME_DIR} && tar -xzf proto.tgz && PYTHONPATH=${RUNTIME_DIR}/src:${RUNTIME_DIR}/proto python3 -c 'import iraf.v1.runtime_pb2_grpc'" \
  || die "智能域加载 proto stub 失败" 3
log "  两端 stub 就绪"

# ---------- 步骤 5：起控制域服务端（每个用例一个实例，避免互相污染） ----------
# 进程清理只按 **PID 文件**：不用 pgrep/pkill 匹配命令行（那种做法会连带命中包装层，
# 本仓库已因此误杀过一次批量任务）。每次起服务前先读 pid 文件按 PID 停，再起新的。
stop_server() {  # stop_server <日志名>
  # ⚠ PID 文件的路径**必须稳定**（不随 RUN_ID 变）：它是进程生命周期句柄，
  # 否则下一轮找不到上一轮的 PID ⇒ 旧服务端不退 ⇒ 端口占用、脚本第二次就跑不起来。
  # 证据（日志/审计账）才是每轮独立的东西，见 RUN_LOGS。
  local tag="$1"
  local pidfile="${RUNTIME_DIR}/logs/server-${tag}.pid"
  local pid
  pid="$(docker exec "${CTL}" sh -c "cat ${pidfile} 2>/dev/null || true" | tr -d '[:space:]')"
  if [ -n "${pid}" ]; then
    docker exec "${CTL}" sh -c "kill ${pid} 2>/dev/null || true"
    log "  已按 PID ${pid} 停止旧服务端（${tag}）"
  fi
}

start_server() {  # start_server <端口> <日志名> <backend-config-json>
  local port="$1" tag="$2" config="$3"
  stop_server "${tag}"
  docker exec -d \
    -e "PYTHONPATH=${RUNTIME_DIR}/src:${RUNTIME_DIR}/proto" \
    -e "IRAF_PROFILE=${RUNTIME_DIR}/${PROFILE_REL}" \
    -e "IRAF_SAFETY_POLICY=${RUNTIME_DIR}/${SAFETY_REL}" \
    -e "IRAF_SKILL_ROOT=${RUNTIME_DIR}/${SKILL_ROOT_REL}" \
    -e "IRAF_BACKEND_ENTRYPOINT=iraf_adapters.domain_stub:ControlDomainStubBackend" \
    -e "IRAF_BACKEND_CONFIG=${config}" \
    -e "IRAF_EVENT_STORE=${RUN_LOGS}/events-${tag}.sqlite" \
    -e "IRAF_GRPC_TOKEN=${TOKEN}" \
    -e "IRAF_GRPC_DEVELOPMENT=true" \
    -e "IRAF_GRPC_HOST=127.0.0.1" \
    -e "IRAF_GRPC_PORT=${port}" \
    "${CTL}" sh -c "cd ${RUNTIME_DIR} && echo \$\$ > ${RUNTIME_DIR}/logs/server-${tag}.pid && exec python3 -m iraf_adapters.grpc.server > ${RUN_LOGS}/server-${tag}.log 2>&1"
  local waited=0
  while [ "${waited}" -lt 60 ]; do
    if docker exec "${CTL}" sh -c "grep -q 'listening on' ${RUN_LOGS}/server-${tag}.log" 2>/dev/null; then
      log "  控制域服务端就绪：127.0.0.1:${port}（${tag}）"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  docker exec "${CTL}" sh -c "cat ${RUN_LOGS}/server-${tag}.log" || true
  die "控制域服务端 ${tag} 未在 60 s 内就绪" 3
}

log "== 步骤 5 起控制域服务端（本轮账本目录：${RUN_LOGS}）"
docker exec "${CTL}" sh -c "mkdir -p ${RUN_LOGS}"
start_server 50051 basic "{\"fault\":\"none\",\"trace_file\":\"${RUN_LOGS}/stub-trace-basic.jsonl\"}"
start_server 50052 cancel "{\"fault\":\"none\",\"latency_ms\":4000,\"trace_file\":\"${RUN_LOGS}/stub-trace-cancel.jsonl\"}"
start_server 50053 unreachable "{\"fault\":\"unreachable\",\"trace_file\":\"${RUN_LOGS}/stub-trace-unreachable.jsonl\"}"

# ---------- 步骤 6：智能域发起四例 ----------
run_case() {  # run_case <用例名> <端口>
  local case_name="$1" port="$2"
  log "== 步骤 6 用例 ${case_name}（智能域 → 控制域 127.0.0.1:${port}）"
  local rc=0
  docker exec \
    -e "PYTHONPATH=${RUNTIME_DIR}/src:${RUNTIME_DIR}/proto" \
    -e "IRAF_CASE=${case_name}" \
    -e "IRAF_SDK_TARGET=127.0.0.1:${port}" \
    -e "IRAF_SDK_TOKEN=${TOKEN}" \
    -e "IRAF_PROFILE_PATH=${RUNTIME_DIR}/${PROFILE_REL}" \
    -e "IRAF_SAFETY_PATH=${RUNTIME_DIR}/${SAFETY_REL}" \
    "${INTEL}" sh -c "cd ${RUNTIME_DIR} && python3 scripts/verify_domain_split_client.py" \
    > "${OUT_DIR}/case-${case_name}.log" 2>&1 || rc=$?
  cat "${OUT_DIR}/case-${case_name}.log"
  if [ "${rc}" -ne 0 ]; then
    CASE_FAILED="${CASE_FAILED} ${case_name}(rc=${rc})"
  fi
  return 0
}

CASE_FAILED=""
run_case normal 50051
run_case denied 50051
run_case cancel 50052
run_case unreachable 50053

# ---------- 步骤 7：收控制域侧审计账（证"安全停机/故障注入真的发生在控制域"） ----------
log "== 步骤 7 收集控制域审计账与日志"
echo "run_id=${RUN_ID}（本轮账本目录 ${RUN_LOGS}）" > "${OUT_DIR}/run-id.txt"
for tag in basic cancel unreachable; do
  docker exec "${CTL}" sh -c "cat ${RUN_LOGS}/stub-trace-${tag}.jsonl" > "${OUT_DIR}/control-trace-${tag}.jsonl" 2>/dev/null || true
  docker exec "${CTL}" sh -c "cat ${RUN_LOGS}/server-${tag}.log" > "${OUT_DIR}/control-server-${tag}.log" 2>/dev/null || true
done
docker exec "${CTL}" sh -c "ls -l ${RUN_LOGS}" > "${OUT_DIR}/control-logs.txt" 2>/dev/null || true

log "== 汇总"
if [ -z "${CASE_FAILED}" ]; then
  log "四例全部符合预期（证据：${OUT_DIR}）"
  exit 0
else
  err "存在不符合项：${CASE_FAILED}"
  exit 1
fi
