#!/usr/bin/env bash
# 把一台新的 x86 主机接入 IRAF 控制面（阶段 E3）
#
# 在哪跑：**新的 x86 主机**上（coretek 用户，具备 sudo）
# 前提：① 该机已能跑 MuJoCo provider（python3 + mujoco + 与板卡同版本 SDK 布局）
#       ② 板卡可达（用于取注册表镜像，默认 10.203.247.147）
#       ③ 板卡侧 token（/etc/iraf/supervisor.token）能拷过来
#
# 用法：
#   bash install_second_host.sh --host-id x86-2 --board 10.203.247.147 \
#        --env-root ~/.config/iraf --repo ~/AIIRAF
#
# 幂等：重复执行只会覆盖脚本/单元/manifest，不删数据。
set -eu

HOST_ID="x86-2"
BOARD="10.203.247.147"
ENV_ROOT="${HOME}/.config/iraf"
REPO="${HOME}/AIIRAF"
SUP_DIR="${HOME}/iraf-instance/scripts"
TOKEN_FILE="${HOME}/.config/iraf/supervisor.token"
PORT=8770

while [ $# -gt 0 ]; do
  case "$1" in
    --host-id) HOST_ID="$2"; shift 2 ;;
    --board)   BOARD="$2"; shift 2 ;;
    --env-root) ENV_ROOT="$2"; shift 2 ;;
    --repo)    REPO="$2"; shift 2 ;;
    *) echo "未知参数：$1"; exit 2 ;;
  esac
done

say() { echo "[$HOST_ID] $*"; }

say "1/7 前置检查"
command -v python3 >/dev/null || { echo "缺 python3"; exit 3; }
python3 -c "import yaml, numpy" 2>/dev/null || { echo "缺 PyYAML/numpy"; exit 3; }
python3 -c "import mujoco" 2>/dev/null || { echo "缺 mujoco（该机需能跑 provider）"; exit 3; }
[ -d "$REPO" ] || { echo "仓库不存在：$REPO（先 clone AIIRAF）"; exit 3; }
echo "  ok: python3/yaml/numpy/mujoco/repo"

say "2/7 取板卡注册表镜像"
TMP=$(mktemp -d)
ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new "root@${BOARD}" \
    "tar czf - -C /etc/iraf scenes providers" > "${TMP}/reg.tgz"
tar xzf "${TMP}/reg.tgz" -C "${TMP}"
[ -d "${TMP}/scenes" ] || { echo "镜像不完整"; exit 4; }
echo "  ok: $(ls "${TMP}/providers" | wc -l) providers / $(ls "${TMP}/scenes" | wc -l) scenes"

say "3/7 生成本机 manifest（会按 --host-id 过滤 hosts）"
python3 "${REPO}/scripts/emit_host_manifest.py" \
    --src "${TMP}" --out "${TMP}/iraf-agent" --host-id "${HOST_ID}" --env-root "${ENV_ROOT}" \
    || { echo "manifest 生成失败（多半是本机没有对应 provider env：$ENV_ROOT）"; exit 5; }
sudo rm -rf /etc/iraf-agent
sudo cp -a "${TMP}/iraf-agent" /etc/iraf-agent
sudo chown -R root:root /etc/iraf-agent && sudo chmod -R a+rX /etc/iraf-agent

say "4/7 安装 supervisor"
mkdir -p "${SUP_DIR}"
cp "${REPO}/scripts/iraf_provider_supervisor.py" "${SUP_DIR}/"
chmod +x "${SUP_DIR}/iraf_provider_supervisor.py"
if [ ! -f "${TOKEN_FILE}" ]; then
  say "缺少控制面 token：${TOKEN_FILE}"
  say "  从板卡拷贝：ssh root@${BOARD} 'cat /etc/iraf/supervisor.token' > ${TOKEN_FILE} && chmod 600 ${TOKEN_FILE}"
  exit 6
fi
chmod 600 "${TOKEN_FILE}"
mkdir -p "${HOME}/.config/systemd/user"
cat > "${HOME}/.config/systemd/user/iraf-provider-supervisor.service" <<UNIT
[Unit]
Description=IRAF Provider Supervisor (${HOST_ID})
After=network-online.target

[Service]
Type=simple
WorkingDirectory=${HOME}/iraf-instance
ExecStart=/usr/bin/python3 -u ${SUP_DIR}/iraf_provider_supervisor.py --manifest /etc/iraf-agent/manifest.yaml --bind 0.0.0.0 --port ${PORT} --token-file ${TOKEN_FILE} --allow-hosts 127.0.0.1,${BOARD}
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
UNIT

say "5/7 允许 linger 并启动"
sudo loginctl enable-linger "$(id -un)" >/dev/null 2>&1 || say "  warn: enable-linger 失败（重启后可能不自启）"
systemctl --user daemon-reload
systemctl --user enable --now iraf-provider-supervisor
sleep 3
systemctl --user is-active iraf-provider-supervisor

say "6/7 本机自检"
TOK=$(cat "${TOKEN_FILE}")
curl -s -m 5 -H "X-Iraf-Token: ${TOK}" "http://127.0.0.1:${PORT}/status" \
  | python3 -c "import json,sys;d=json.load(sys.stdin);print('  active=%s providers=%d host=%s'%(d.get('active_scene'),len(d.get('providers') or []),(d.get('host') or {}).get('cores')))"

say "7/7 下一步（在板卡上执行，需人审）"
cat <<'NEXT'
  a) 在板卡 providers/*.yaml 的 hosts[] 增加本机条目（含 control_url），示例：
       - id: x86-2
         endpoint: <本机IP>:50054
         control_url: http://<本机IP>:8770
         capacity: {cores: 16, gpu: none}
         priority: 20
     并把 <本机IP> 加入 allowed_hosts
  b) 重新封签并校验：
       python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --seal
       python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --profiles-root /opt/iraf-sdk/0.4.1/profiles
     期望：OK scenes=4 providers=4
  c) 不重启 MCP（热加载）；用 iraf_scene_status(probe=true) 确认两台 host 都可见
  d) 故障转移验证：把 host1 的 __world__ 租约占住 → 提交应改路由到 host2（审计 rerouted_to）
  e) 回滚：删掉 hosts[] 里的新条目 → reseal → 验证 OK
NEXT
say "完成（manifest 与 supervisor 已就绪；注册表条目需在板卡侧改）"
