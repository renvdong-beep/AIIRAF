# AgentOS MCP 绑定运维手册（IRAF 北向）

状态：0.4.1（2026-11-06）
适用：智控域 `.147` / 其他 aarch64 Hyper 边缘板。  
口径：`simulation=true`；本文只讲 IRAF 任务级 MCP 如何绑进 AgentOS。

## 0. 机制总览（先看这张图）

```text
AgentOS 宿主 chat / worker / 上位机 harness
        │  streamable-http + Authorization: Bearer <token>
        ▼
边缘 IRAF  MCP server（一个进程 = 一个场景端点）
        │  iraf_submit_task / iraf_get_task / iraf_robot_mode
        ▼
IRAF TaskFlow → Skill Runtime → Policy Gateway
        │  provider.proto（Health / Invoke / Cancel，跨机 gRPC）
        ▼
x86 Provider（MuJoCo；simulation=true）
```

四条边界：

1. **只暴露任务级入口**：不是把每个机器人 Skill 注册成 MCP 工具。工具只有 5 个
   （`iraf_enter_robot_mode`、`iraf_exit_robot_mode`、`iraf_robot_mode`、
   `iraf_submit_task`、`iraf_get_task`），技能选择与参数校验在 IRAF 内部完成。
2. **身份来自 transport**：HTTP 形态必须是 Bearer token；注册表只存 SHA-256，缺表
   fail-closed（退出码 2）。工具参数里没有 caller/role/tenant，写了也不认。
3. **一个端点绑定一个场景**：`IRAF_PROFILE` + `IRAF_BACKEND_CONFIG.endpoint` 在进程启动时
   固定，运行中不改（AGENTS 2.6）。ur5=8765、piper=8766、joint 预留 8767。
4. **AgentOS 不拥有 x86 权限**：Provider 的启停归 x86 本地 supervisor；边缘只调
   Health/Invoke/Cancel，Provider 不可用时返回 `IRAF-TRANSPORT-FAILED` /
   `IRAF-PROVIDER-UNAVAILABLE`，不自动拉起、不伪造成功。

两套传输：

| 形态 | 谁拉起 | 身份来源 | 适用 |
|---|---|---|---|
| stdio | 本机宿主作为子进程 | `IRAF_RUNTIME_SUBJECT`（进程边界即信任边界） | AgentOS 与本机 runtime 同机 |
| streamable-http | systemd 常驻 | Bearer token（`/etc/iraf/mcp-tokens.json`） | 跨机 harness / AgentOS / 语音前端 |

## 1. 两套 home，别混

| 调用路径 | 配置位置 | 说明 |
|---|---|---|
| dashboard `/chat`（`tui_gateway`，宿主进程） | `/root/.intewell/config.yaml` | chat 实际走这里；宿主配置被 AgentOS 重写会丢 mcp 段 |
| worker / task-api（容器） | `/var/lib/agentos-shared/.intewell/config.yaml` | 容器 home 是宿主目录的 bind mount，配置持久；但镜像重建会丢 `mcp` Python 包 |

两套都需要 `mcp_servers.iraf-task`，且都要带 `Authorization: Bearer <token>`。
只配 worker 时，chat 仍会报“没有 mcp”。

## 2. 一键脚本

板上入口：

```bash
B=/opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh

bash $B --status
# ur5 端点（沿用现有 server 名 iraf-task）
bash $B --apply --verify --server-name iraf-task --url http://<板卡IP>:8765/mcp
# piper 端点（独立 server 名）
bash $B --apply --verify --server-name iraf-piper --url http://<板卡IP>:8766/mcp
bash $B --verify --url http://<板卡IP>:8765/mcp
bash $B --revert --server-name iraf-piper
```

行为：

- `--status`：服务、host config、worker 容器、token、mcp 包状态（只读）。
- `--apply`：
  - 宿主：优先 dashboard API `POST /api/mcp/servers`（需要 `/etc/iraf/agentos-admin`，
    `username=...` + `password=...`，600）；API 会触发热加载。
    没有 admin 文件时降级为直接合并 `/root/.intewell/config.yaml`，并提示需要重启
    `tui_gateway` 才生效。
  - worker：合并 `/var/lib/agentos-shared/.intewell/config.yaml`，重启 worker。
  - token：宿主 `agentos-host`；worker `agentos-worker`；文件 600，只存明文一次。
- `--verify`：检查
  1. 对应 `iraf-mcp-*.service` active、`/healthz` 200、无 token 401；
  2. 宿主 config 里有目标 server（有 admin 文件时改为 API test）；
  3. 每个 worker `import mcp.client.streamable_http`；
  4. 每个 worker `intewell-agent-cli mcp test <server>` 能发现 5 个工具。
  结果写 `/etc/iraf/agentos-mcp-binding-<server>.json`（600）。
- `--revert`：只删指定 server 段，保留其它 server。

`board_upgrade.sh` 在 edge-intel 安装完两个 MCP 单元后，自动对
`iraf-task:8765` 与 `iraf-piper:8766` 各执行一次 `--apply --verify`。

## 3. dashboard admin 凭据（host API 必需）

```bash
# 板上 root；值不要进 git / 聊天记录
install -m 600 /dev/null /etc/iraf/agentos-admin
cat > /etc/iraf/agentos-admin <<'EOF'
username=admin
password=<dashboard 管理员口令>
EOF
chmod 600 /etc/iraf/agentos-admin
```

没有这份文件时脚本只做 YAML 合并，并明确告警“需重启 dashboard”；这是降级路径，
不能声称 chat 已接入。

## 4. 常见故障

### 4.1 chat 说没有 mcp 工具

先按顺序查：

1. `systemctl is-active iraf-mcp-http`、`ss -lntp | grep 8765`
2. `GET /healthz` = 200，`POST /mcp` 无 token = 401
3. 宿主 `/root/.intewell/config.yaml` 是否有 `mcp_servers.iraf-task`
   （AgentOS 重写 config 会清掉；用 `--apply --verify` 补）
4. worker：`docker exec -e HOME=/home/agentos agentos-worker-1 \
   intewell-agent-cli mcp test iraf-task`
5. worker 日志：`grep 'registered 9 tool' /home/agentos/.intewell/logs/agent.log`
6. 新会话：旧会话的工具快照不会自动补 MCP

### 4.2 worker 日志报 streamable_http is not available

```
MCP server 'iraf-task' requires HTTP transport but
mcp.client.streamable_http is not available.
```

原因：worker 镜像没有 `mcp>=1.26,<2`，或容器重建后装包丢失。
处置：重装 `mcp==1.26.0` 到 worker，重启 worker；长期必须由厂商打进镜像。
`--verify` 会把缺包标成 FAIL。

### 4.3 systemd 服务启动后立即退出（code 0）

装配单里 `IRAF_MCP_TRANSPORT=stdio` 会覆盖 unit 的 `Environment=`（systemd
EnvironmentFile 优先级更高）。0.4.1 的 unit 已改为在 ExecStart 的 shell 前缀里强制
`IRAF_MCP_TRANSPORT=http`；手工单元不要只写 `Environment=`。

### 4.4 token 401

- 注册表 `/etc/iraf/mcp-tokens.json` 必须 600，只存 SHA-256；
- token 过期/被 revoke；`iraf-mcp-token list` 查看；
- 客户端配置必须带 `Authorization: Bearer <token>`，不是只写 URL；
- host/worker/upper 三套 token 不要混用。

## 5. 验收判据

只有以下全过，才算“chat 接入完成”：

- `--verify` 全部 OK；
- 新 chat 会话的工具列表出现 `mcp__iraf_task__iraf_enter_robot_mode`；
- 输入“进入机器人开发模式”返回 `mode=robot_dev`、`skills=15`、`simulation=true`；
- 事件库有对应 execution（如提交任务）。

## 6. 证据路径

- 绑定验证：`/etc/iraf/agentos-mcp-binding.json`
- MCP 审计：`/var/log/iraf/mcp-audit.jsonl`
- AgentOS 日志：`/root/.intewell/logs/agent.log`、
  worker 内 `/home/agentos/.intewell/logs/agent.log`
- 服务单元：`/etc/systemd/system/iraf-mcp-http.service`

## 7. 当前实机状态（2026-11-05，10.203.247.147）

### 7.1 版本与服务

```
/opt/iraf-sdk/current -> /opt/iraf-sdk/0.4.1
iraf-mcp-http.service  active / enabled
0.0.0.0:8765 监听；GET /healthz 200；POST /mcp 无 token 401
```

TLS 未启用（HTTP），只在可信内网使用。

### 7.2 三套 token（均为 600，明文只在各自文件）

| 用途 | 文件 | subject | token id | 到期 |
|---|---|---|---|---|
| 上位机 harness | `/etc/iraf/mcp-client.token` | upper-sdk | tok-e81492d3 | 2027-02-02 |
| AgentOS worker/task-api | `/etc/iraf/mcp-agentos.token` | agentos-worker | tok-446c233e | 2027-02-02 |
| AgentOS 宿主 chat | `/etc/iraf/mcp-host.token` | agentos-host | tok-4a089e63 | 2027-02-03 |

注册表：`/etc/iraf/mcp-tokens.json`（600，只存 SHA-256）。
Dashboard 自动化凭据：`/etc/iraf/agentos-admin`（600；只记录路径，值不写入文档）。

### 7.3 两套 AgentOS 配置

```
宿主 chat：
  /root/.intewell/config.yaml
  mcp_servers.iraf-task = http://10.203.247.147:8765/mcp
  Authorization: Bearer <agentos-host token>（字面值，600 文件权限）
  enabled: true

worker：
  /var/lib/agentos-shared/.intewell/config.yaml（10001:10001，600）
  mcp_servers.iraf-task = 同上
  Authorization: Bearer <agentos-worker token>
  enabled: true
```

worker 容器内 `/home/agentos/.intewell` 是共享目录的 bind mount。
worker 镜像 `agentos-worker:c29de20f-arm64` **不带 mcp 包**；当前两个 worker 已手工安装
`mcp==1.26.0`，容器重建后会丢，需要重装或由厂商写入镜像。

### 7.4 verify 实测（板上）

```
$ bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh \
    --verify --url http://10.203.247.147:8765/mcp

service                  OK healthz=True no_token=401
host_api                 OK tools=5
agentos-worker-1.mcp     OK
agentos-worker-1.tools   OK（5 个工具）
agentos-worker-2.mcp     OK
agentos-worker-2.tools   OK（5 个工具）
```

验证记录：`/etc/iraf/agentos-mcp-binding-iraf-task.json`（600）。

### 7.5 多场景端点（0.5.0 已上板，2026-11-06）

0.4.1 只监听 8765（UR5）。0.5.0 安装后 `.147` 实际状态：

```
/opt/iraf-sdk/current -> 0.5.0
iraf-mcp-http.service   active/enabled  0.0.0.0:8765  profile=profiles/ur5_mujoco.yaml
iraf-mcp-piper.service  active/enabled  0.0.0.0:8766  profile=profiles/piper_mujoco.yaml
```

实例级装配单（由安装器渲染，不是手工改）：

```
/etc/iraf/edge-intel.env          ← 基础装配单（板侧脚本读回端点）
/etc/iraf/iraf-mcp-http.env       ← ur5：PROFILE=profiles/ur5_mujoco.yaml
/etc/iraf/iraf-mcp-piper.env      ← piper：PROFILE=profiles/piper_mujoco.yaml
```

实测（2026-11-06）：

```
8765 healthz=200  POST 无 token=401
8766 healthz=200  POST 无 token=401
8765 → MODE robot_dev profile=profiles/ur5_mujoco.yaml skills=15 simulation=True
8766 → MODE robot_dev profile=profiles/piper_mujoco.yaml skills=15 simulation=True

ur5  : execution_id=480ecb67-a982-5ff2-8476-2b79139de0aa status=SUCCEEDED
       lift_delta_m=0.07633 center_distance_m=0.00011855145570256166
piper: execution_id=4b1ed285-a655-5114-9fd0-4d07853c61cd status=SUCCEEDED
       lift_delta_m=0.083308 center_distance_m=0.0000514227433
```

⚠ AgentOS 侧绑定尚未通过：宿主 dashboard 凭据 401、worker 镜像缺 `mcp` 包。
边缘 MCP 本身正常（上面的工具发现与抓取都是有效 token 直连证据），但 chat 仍看不到工具。

宿主日志（2026-11-05 03:42:33）：

```
MCP server 'iraf-task' (HTTP): registered 9 tool(s):
  mcp__iraf_task__iraf_enter_robot_mode
  mcp__iraf_task__iraf_exit_robot_mode
  mcp__iraf_task__iraf_robot_mode
  mcp__iraf_task__iraf_submit_task
  mcp__iraf_task__iraf_get_task
  + list_resources / read_resource / list_prompts / get_prompt
AgentOS capability assembly: 9 MCP tool(s) registered
```

### 7.5 使用与复验

- chat 必须**新开会话**才会采纳 MCP 工具；旧会话快照不会自动补。
- 新会话输入“进入机器人开发模式”，预期调用
  `mcp__iraf_task__iraf_enter_robot_mode`，返回 `mode=robot_dev`、`skills=15`、
  `simulation=true`。
- 日常复验：

```bash
bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh --status
bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh \
  --verify --url http://10.203.247.147:8765/mcp
```

- 若宿主配置被 AgentOS 重写、或 worker 容器重建：

```bash
bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh \
  --apply --verify --url http://10.203.247.147:8765/mcp
```

  并在 worker 缺 mcp 包时补 `mcp==1.26.0`（长期应由镜像提供）。
