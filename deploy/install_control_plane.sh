#!/usr/bin/env bash
# IRAF 控制面（Provider Supervisor）安装脚本 —— 不依赖板卡 SSH 也能装。
#
# 与 install_second_host.sh 的区别：
#   · 注册表来源可选 `--from-board <ip>`（ssh 拉）或 `--registry-dir <本地目录>`（离线拷贝）；
#   · token 来源可选文件或标准输入，缺了会明确告诉你从哪拷；
#   · 不假设任何板卡 IP：`--allow` 显式给出允许调用本控制面的板卡地址（可重复）。
#
# 用法：
#   bash deploy/install_control_plane.sh --host-id x86-1 --allow 10.203.247.147 --allow 10.203.247.112
#   bash deploy/install_control_plane.sh --host-id x86-2 --from-board 10.203.247.147
#   bash deploy/install_control_plane.sh --host-id x86-3 --registry-dir /tmp/iraf-reg
#
# 退出码：0 成功 / 1 参数 / 2 前置 / 3 注册表 / 4 manifest / 5 token / 6 启动
set -uo pipefail

HOST_ID="x86-1"
REGISTRY_DIR=""
FROM_BOARD=""
ENV_ROOT="${HOME}/.config/iraf"
REPO="${HOME}/AIIRAF"
SUP_DIR="${HOME}/iraf-instance/scripts"
TOKEN_FILE="${HOME}/.config/iraf/supervisor.token"
PORT=8770
ALLOW=("127.0.0.1")

while [ $# -gt 0 ]; do
  case "$1" in
    --host-id) HOST_ID="$2"; shift 2 ;;
    --registry-dir) REGISTRY_DIR="$2"; shift 2 ;;
    --from-board) FROM_BOARD="$2"; shift 2 ;;
    --env-root) ENV_ROOT="$2"; shift 2 ;;
    --repo) REPO="$2"; shift 2 ;;
    --token-file) TOKEN_FILE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --allow) ALLOW+=("$2"); shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 1 ;;
  esac
done

say() { echo "[control-plane] $*"; }

say "1/6 前置检查"
command -v python3 >/dev/null || { echo "  缺 python3"; exit 2; }
python3 -c "import yaml" 2>/dev/null || { echo "  缺 PyYAML"; exit 2; }
[ -f "$REPO/scripts/iraf_provider_supervisor.py" ] || { echo "  仓库缺 supervisor 脚本：$REPO"; exit 2; }
python3 -c "import mujoco" 2>/dev/null && say "  mujoco OK（本机可跑 provider）" || say "  warn: 无 mujoco（只做控制面无妨，跑 provider 才需要）"

say "2/6 准备注册表源（scenes/ + providers/）"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
if [ -n "$REGISTRY_DIR" ]; then
  cp -a "$REGISTRY_DIR"/. "$TMP"/ || { echo "  拷贝失败：$REGISTRY_DIR"; exit 3; }
elif [ -n "$FROM_BOARD" ]; then
  ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new "root@${FROM_BOARD}" \
      "tar czf - -C /etc/iraf scenes providers" > "$TMP/reg.tgz" || { echo "  拉取失败（需要 root@${FROM_BOARD} 公钥）"; exit 3; }
  tar xzf "$TMP/reg.tgz" -C "$TMP" || exit 3
else
  echo "  必须给 --from-board <ip> 或 --registry-dir <目录>" >&2; exit 3
fi
[ -d "$TMP/scenes" ] && [ -d "$TMP/providers" ] || { echo "  注册表不完整（缺 scenes/ 或 providers/）"; exit 3; }
say "  scenes=$(ls "$TMP/scenes" | wc -l) providers=$(ls "$TMP/providers" | wc -l)"

say "3/6 生成 host manifest（按 $HOST_ID 过滤 hosts）"
python3 "$REPO/scripts/emit_host_manifest.py" --src "$TMP" --out "$TMP/iraf-agent" \
  --host-id "$HOST_ID" --env-root "$ENV_ROOT" >/dev/null || {
  echo "  生成失败：多半是本机缺少对应 provider 装配单（$ENV_ROOT/iraf-provider@*.env）"; exit 4; }
sudo -n rm -rf /etc/iraf-agent && sudo -n cp -a "$TMP/iraf-agent" /etc/iraf-agent && \
  sudo -n chown -R root:root /etc/iraf-agent && sudo -n chmod -R a+rX /etc/iraf-agent || {
  echo "  安装 manifest 失败（需要 sudo）"; exit 4; }
say "  /etc/iraf-agent/manifest.yaml 就绪"

say "4/6 安装 supervisor 与 user 单元"
mkdir -p "$SUP_DIR" "${HOME}/.config/systemd/user"
cp "$REPO/scripts/iraf_provider_supervisor.py" "$SUP_DIR/"
chmod +x "$SUP_DIR/iraf_provider_supervisor.py"
if [ ! -f "$TOKEN_FILE" ]; then
  echo "  缺控制面 token：$TOKEN_FILE" >&2
  echo "  从板卡拷：ssh root@<板卡IP> 'cat /etc/iraf/supervisor.token' > $TOKEN_FILE && chmod 600 $TOKEN_FILE" >&2
  exit 5
fi
chmod 600 "$TOKEN_FILE"
ALLOW_CSV="$(IFS=,; echo "${ALLOW[*]}")"
cat > "${HOME}/.config/systemd/user/iraf-provider-supervisor.service" <<UNIT
[Unit]
Description=IRAF Provider Supervisor (${HOST_ID})
After=network-online.target

[Service]
Type=simple
WorkingDirectory=${HOME}/iraf-instance
ExecStart=/usr/bin/python3 -u ${SUP_DIR}/iraf_provider_supervisor.py --manifest /etc/iraf-agent/manifest.yaml --bind 0.0.0.0 --port ${PORT} --token-file ${TOKEN_FILE} --allow-hosts ${ALLOW_CSV}
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
UNIT
say "  单元已写：ExecStart ... --allow-hosts ${ALLOW_CSV}"

say "5/6 启用 linger 并启动"
sudo -n loginctl enable-linger "$(id -un)" >/dev/null 2>&1 || say "  warn: enable-linger 失败（重启后可能不自启）"
systemctl --user daemon-reload
systemctl --user enable --now iraf-provider-supervisor
sleep 3
systemctl --user is-active iraf-provider-supervisor || { echo "  启动失败：journalctl --user -u iraf-provider-supervisor"; exit 6; }

say "6/6 自检"
TOK="$(cat "$TOKEN_FILE")"
curl -s -m 5 -H "X-Iraf-Token: $TOK" "http://127.0.0.1:${PORT}/status" \
  | python3 -c "import json,sys;d=json.load(sys.stdin);print('  active=%s providers=%d host=%s'%(d.get('active_scene'),len(d.get('providers') or []),d.get('host_id')))" \
  || { echo "  /status 自检失败"; exit 6; }

cat <<NEXT

下一步（板卡侧，需人审）：
  1) 在 /etc/iraf/providers/*.yaml 的 hosts[] 增加本机条目（endpoint 与 control_url 指向 <本机IP>:${PORT}）
     并把 <本机IP> 加入 allowed_hosts
  2) python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --seal
     python3 /root/iraf-instance/scripts/registry_check.py --root /etc/iraf --profiles-root /opt/iraf-sdk/current/profiles
  3) 不重启 MCP（热加载），iraf_scene_status(probe=true) 确认本机 host 可见
NEXT
exit 0