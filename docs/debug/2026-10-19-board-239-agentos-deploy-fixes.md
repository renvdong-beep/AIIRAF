# 板卡 10.203.247.239（RK3588 / Ubuntu 20.04）AgentOS 部署修复记录

日期：2026-10-19
机器：10.203.247.239，用户 topeet/topeet（sudo 需密码），root shell 由交互式 `sudo -i` 获得
部署根：`/home/topeet/AgentOS/IntewellAgent/artifacts/arm64/agentos-bundle-713e5187-arm64/deploy`
镜像标签：`713e5187-arm64`

本记录只覆盖**部署缺陷**，不含任何真机能力结论；所有服务均为板上本机服务，与 IRAF 实时闭环无关。

## 1. relay 崩溃循环（Errno 98）—— 已修复

现象：`agentos-relay` 无限重启，日志 `OSError: [Errno 98] Address already in use`。
根因：relay 使用 `network_mode: host`，`PORT: ${AGENTOS_RELAY_PORT:-8080}`，而板上 8080 被板厂
`crtmpserver`（pid 1638，systemd enabled，非本项目）占用。

处理：`.env` 中 `AGENTOS_RELAY_PORT=8084`（已由上一轮改好），再重建容器：

```bash
cd /home/topeet/AgentOS/IntewellAgent/artifacts/arm64/agentos-bundle-713e5187-arm64/deploy
docker compose -f docker-compose.production.arm64.yml --env-file .env \
  up -d --no-deps --force-recreate agentos-relay
```

验收（实测）：`restarts=0`、`Up (healthy)`、`ss` 显示 `0.0.0.0:8084`、日志
`AgentOS relay listening on 0.0.0.0:8084`。

坑位：
- compose 文件顶层有 `name: agentos-production`，所以**不需要** `-p`；之前"创建 agentos-db 撞名"
  不是项目名问题，而是不带服务名的 `up -d` 会尝试创建 `container_name: agentos-db`，而它已由
  非 compose 方式（native）存在。**任何重建都必须带服务名 + `--no-deps`**。
- 上一轮记录的"残留半删除容器 `e5244d355d8e_agentos-relay`"实测不存在，`docker ps -a` 里只有
  正常命名的容器，直接重建即可。

## 2. mem-manager 崩溃循环（ModuleNotFoundError: agentos）—— 厂商镜像缺陷，本地兜底

根因（镜像内自检）：`agentos-mem-manager:713e5187-arm64` 里 `WorkingDir=/app` 是空目录，
site-packages 只有 pip/setuptools/wheel/packaging，没有 `agentos` 包；而 compose 里
`command: ["python", "-m", "agentos.mem_manager"]` ⇒ 必然 `ModuleNotFoundError`。
**厂商镜像必须重出**（55,730,688 字节那份）。

本地兜底：同标签的 `agentos-worker:713e5187-arm64` 内含完整 agentos 包，实测
`python -c "import agentos.mem_manager"` 成功
（`/usr/local/lib/python3.10/site-packages/agentos/mem_manager.py`）。故只替换 image，
其余 command/env/volumes/depends_on 全部沿用基础 compose：

`deploy/docker-compose.memfix.override.yml`（本地副本 `scripts/239-memfix-override.yml`）

```yaml
services:
  agentos-mem-manager:
    image: agentos-worker:${AGENTOS_IMAGE_TAG:?set AGENTOS_IMAGE_TAG}
```

```bash
docker compose -f docker-compose.production.arm64.yml \
  -f docker-compose.memfix.override.yml --env-file .env \
  up -d --no-deps --force-recreate agentos-mem-manager
```

验收（实测）：`restarts=0`、`Up`、`ss` 显示 `127.0.0.1:8091` 在听、容器日志无异常。

**边界（不得夸大）**：只证明"服务进程存活 + 端口在听"。`/` 与 `/health` 均返回 404
（这两条不是它的健康路由），**尚未验证记忆读写 API 的真实功能**。

坑位：
- 这个 override 是本地改动，**换镜像后 net 磁盘成本约 +1.4 GB**（vfs 驱动下每个容器是一份
  完整镜像拷贝：可用空间 11 G → 9.6 G）。任何人不带 `-f override` 直接跑部署脚本，
  mem-manager 会静默退回崩溃循环且脚本退出码仍是 0。厂商新镜像到位后应删除该文件。

## 3. host-worker 未运行 —— 已启动

根因：按设计按需启动，且 `/etc/agentos/host-worker.env` 权限为 `-rw------- root root`，
**以 topeet 用户查状态会 PermissionError，必须以 root 运行**。

```bash
intewell-agent-host-worker start        # 仅 root 可用
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8083/readyz   # → 200
```

健康路由是 `/readyz`（不是 `/healthz`）。日志：`/root/.intewell/logs/agentos-host-worker.log`。

## 4. host CLI 无 provider —— 已按本机 Hermes 同源配置打通

所有 `intewell-agent*` 子命令在参数解析前就初始化 agent，未配 provider 时直接退出：
`❌ Failed to initialize agent: No LLM provider configured`（连 `config --help` 都会失败，
所以只能用配置文件，不能用 `config set` 引导）。

配置位置与字段来源（读代码确认，不靠猜）：
- 路径：`~/.intewell/config.yaml`（`intewell_cli/config.py` 文件头注释第 5 行；root 即 `/root/.intewell/config.yaml`）
- 字段：`agent/agent_init.py:1038` 同时接受 `key_env` 与 `api_key_env`
- key 来源：`~/.intewell/.env` 的 `NEWAPI_KEY`

写入内容（与本机 `~/.hermes/config.yaml` 同源同值，仅把模型收敛到 flash）：

```yaml
model:
  default: deepseek-flash
  provider: custom:company
  base_url: http://192.168.100.89:3000/v1
custom_providers:
  - name: company
    base_url: http://192.168.100.89:3000/v1
    key_env: NEWAPI_KEY
    api_mode: chat_completions
    default_model: deepseek-flash
    context_length: 1000000
```

key 不落明文命令历史：从 `deploy/.env` 的 `CUSTOM_API_KEY` 直接重定向到
`/root/.intewell/.env`，只回显长度（实测 `key_len=51`），两文件均 `chmod 600`。

验收（两次真实推理，不是模板回话）：
- `intewell-agent-cli -z "只回答四个字：链路已通"` → `链路已通`，EXIT=0
- `intewell-agent-cli -z "把 37 和 59 相加，只输出阿拉伯数字结果"` → `96`

坑位：
- CLI 以 root 运行时读 `/root/.intewell/`；以 topeet 运行会读 `/home/topeet/.intewell/`，
  那里没有配置 ⇒ 会重现"No LLM provider configured"。用哪个用户，就在哪个家目录放配置。
- 每次调用都会打 4 条 `plugin.yaml` 解析警告（`0x0005` 控制字符：nemo_relay、spotify、xai、photon），
  不影响功能，属包内容缺陷。

## 5. 环境中不可修的部分（留档，不要重复尝试）

- docker 存储驱动是 `vfs`：内核 5.10.198 未编 overlayfs
  （`modprobe: FATAL: Module overlay not found`、`/proc/filesystems` 无 overlay），
  `fuse-overlayfs` 未装 ⇒ overlay2 不可用，`/var/lib/docker` 15 G 且每容器一份完整拷贝。
  这是内核限制，不是配置错误。
- 板厂 `crtmpserver` 占用 8080，不属本项目，未改动。

## 6. 本轮末次实测快照（2026-10-19 01:1x）

```
agentos-db / task-api / worker-1 / worker-2 / searxng / searxng-network-monitor  Up 2 days
agentos-relay        Up (healthy)   restarts=0  0.0.0.0:8084
agentos-mem-manager  Up             restarts=0  127.0.0.1:8091（worker 镜像兜底）
host-worker(非容器)  readyz=200     pid supervisor 916435
worker-1/2 /healthz=200
df / 58G 46G 9.6G 83%
```

## 7. 未完成事项

1. relay 只验证了"进程健康 + 端口在听"，**未做一次真实事件投递**（写入口应为 POST + `AGENTOS_RELAY_TOKEN`）。
2. mem-manager 只验证存活，**未验证记忆读写**；且需要厂商重出镜像以撤掉 override。
3. host-worker 由 supervisor 拉起，**没有 systemd 单元 ⇒ 重启板卡后不会自启**。
4. 磁盘：第一档 root 部分（journal 3.44 GiB + sunlogin 47.1 MiB）未执行，
   脚本 `/home/topeet/cleanup_scripts/clean_board_239_root.sh`；第二档 2.32 GiB 未动。

## 8. dashboard web 端（9119）—— 已启动并验证

9119 是 `intewell-agent-cli dashboard`（`intewell_cli/main.py:2800` 默认
`AGENTOS_DASHBOARD_URL=http://127.0.0.1:9119`），**默认只绑 loopback**。

启动（必须 root，配置在 `/root/.intewell/`）：

```bash
cd /root && nohup intewell-agent-cli dashboard > /tmp/239-dashboard.log 2>&1 &
```

验收（实测）：
- `ss -ltn` → `LISTEN 127.0.0.1:9119`
- 日志 → `INTEWELL_DASHBOARD_READY port=9119`
- `GET /` → 200，`<title>Intewell Agent - Dashboard</title>`，SPA 资源
  `/assets/index-VRId_EjD.js`
- 页面内嵌 `window.__INTEWELL_SESSION_TOKEN__="…"`（43 字符）、
  `__INTEWELL_AUTH_REQUIRED__=false` ⇒ 浏览器加载即完成鉴权，无需手填 token
- 带 `X-Intewell-Session-Token` 请求 `GET /api/config` → 200，
  返回 `{"model":"deepseek-flash","toolsets":["intewell-cli"],…}`（读到的正是第 4 节写的配置）
- 不带 token → 401 `{"error":"dashboard_session_token_invalid"}`
- 关掉启动它的 shell 后复测 `GET /` 仍 200，进程存活（pid 919605）

注：以上四条是 **loopback 阶段**的实测。改为 `0.0.0.0` + basic 鉴权后，页面不再内嵌
`__INTEWELL_SESSION_TOKEN__`（`__INTEWELL_AUTH_REQUIRED__` 变 true），改为 cookie 会话，见下。

远程访问：已改为直连（按需求），并配了 basic 登录

初版是 loopback 绑定 + ssh 隧道。因为需要直接从工位访问 `http://10.203.247.239:9119/`，
改为绑 `0.0.0.0`。**前提是必须先配鉴权**：本版 `--insecure` 已是 NO-OP
（`main.py:12031` 注释与 `dashboard --help` 均明示，2026-06 起非 loopback 绑定强制需要
auth provider，`start_server` 无 provider 时 fail-closed）。

配置（`/root/.intewell/config.yaml`）：

```yaml
dashboard:
  basic_auth:
    username: admin
    password: Iraf-239-Dash
    secret: <64 hex>     # 会话签名密钥，缺失则重启后会话失效
```

启动：

```bash
cd /root && nohup intewell-agent-cli dashboard --host 0.0.0.0 --port 9119 --no-open \
  > /tmp/239-dashboard.log 2>&1 &
```

验收（实测，均从工位机器直接打 IP，不经隧道）：
- `ss -ltn` → `LISTEN 0.0.0.0:9119`（此前是 `127.0.0.1:9119`）
- `GET http://10.203.247.239:9119/` → 302 → `/login?next=%2F` → 200 登录页
- `POST /auth/password-login`，body 需含 `provider: basic`
  → `{"ok":true,"next":"/"}` + 200 + Set-Cookie
- 带该 cookie `GET /` → 200，`<title>Intewell Agent - Dashboard</title>`
- 错密码 → 401 `{"detail":"Invalid credentials"}`
- 防火墙：`iptables -S INPUT` 首条 `-P INPUT ACCEPT`，ufw 未启用 ⇒ 9119 无过滤

保留 tunnel 方式作备选：`ssh -N -L 9119:127.0.0.1:9119 topeet@10.203.247.239`
（仅在改回 `--host 127.0.0.1` 时才有意义）。

安全口径（必须写进交付说明，不许淡化）：
- basic 登录的意义是"不要裸奔"，不是"企业级鉴权"：明文口令存在 `/root/.intewell/config.yaml`
  （600，root only），provider 用 stdlib scrypt 存哈希、会话为 HMAC 签名 token；
- 端口暴露范围 = 该 LAN 内任何能连 10.203.247.239 的主机。**要更强控制就退回
  loopback + 隧道，或用 self_hosted / nous provider 走 OAuth**（`plugins/dashboard_auth/` 下有
  basic / self_hosted / nous / drain 四个 provider 可选）。

坑位：
- `intewell-agent-cli dashboard --status` 报 `No intewell dashboard processes running.`
  但进程确实在跑（pid 919605，ss 有 9119）。⇒ `--stop` 对它无效，停止只能
  `pkill -f "intewell-agent-cli dashboard"`，并自行确认 9119 已释放。
- headless 板上它会尝试拉起 chromium，日志刷 `gpu_init.cc: Passthrough is not supported, GL is egl`、
  `Failed to connect to the bus`、`Failed to query video capabilities` —— 只是噪音，不影响 web 服务。
- 无 systemd 单元 ⇒ **重启板卡后 9119 不会自启**，需重跑上面的启动命令。
