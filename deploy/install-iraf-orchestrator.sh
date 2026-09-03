#!/usr/bin/env bash
set -euo pipefail

repo_dir=\${1:-/home/coretek/AIIRAF}
config_src=\${repo_dir}/orchestrator/config.example.json
service_src=\${repo_dir}/deploy/iraf-orchestrator.service
config_dir=/etc/iraf
env_file=\${config_dir}/orchestrator.env
config_dst=\${config_dir}/orchestrator.json

if [[ $(id -u) -ne 0 ]]; then
  echo "必须以 root 执行安装脚本" >&2
  exit 1
fi
if [[ ! -f \${config_src} || ! -f \${service_src} ]]; then
  echo "仓库路径无效，缺少调度配置或 service 模板: \${repo_dir}" >&2
  exit 1
fi
if [[ ! -f \${env_file} ]]; then
  echo "缺少 \${env_file}；请创建受控凭据文件，不要把 Token 写入仓库" >&2
  exit 1
fi

install -d -m 0750 -o root -g coretek "\${config_dir}"
install -m 0640 -o root -g coretek "\${config_src}" "\${config_dst}"
install -m 0644 "\${service_src}" /etc/systemd/system/iraf-orchestrator.service
systemctl daemon-reload
systemctl enable iraf-orchestrator.service
echo "已安装并设置开机启动；服务保持 stopped，请确认后执行 systemctl start iraf-orchestrator"
