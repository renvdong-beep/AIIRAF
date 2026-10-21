# 语音语义 → 机器人任务：道路与测试说明（2026-10-21）

本文回答一个问题：**麦（或麦拿到的语义）之后，怎么把任务送到 IRAF，并怎么验证这条路是通的**。
结论先说：IRAF 的北向只有两个任务级工具（`iraf_submit_task` / `iraf_get_task`），
和"谁在说话"解耦；宿主可以是板上 AgentOS（stdio），也可以是**任何**跨机 MCP 宿主（HTTP）。
本轮已经把跨机 HTTP 传输做进框架并在真板上测通。

## 1. 三种接入形态（按现场条件选）

| 形态 | 传输 | 宿主在哪 | 现状 |
|---|---|---|---|
| A 本机宿主 | stdio | 板上（AgentOS CLI / dashboard / ACP 拉起子进程） | 已通（接口级实测 SUCCEEDED） |
| B 跨机宿主 | streamable-http | 任意有麦的机器（x86 桌面、手机、网关） | **本轮新增并实测通** |
| C 平台内语音 | 由 AgentOS 自己的 voice 会话发起 | 有 `/dev/snd` 的机器 | 代码路径在（`intewell-agent-cli voice`），板上无麦未测 |

板卡实测事实（智控域 .147）：
```
intewell-agent-cli voice status
  microphones : 0
  device note : container without forwarded audio (no /dev/snd)
  voiceprints : 0 (required=yes, threshold=0.75)   ← 唤醒词会被拒，先 enroll 或关掉校验
```
⇒ 语音前端**不能**跑在这个域里，必须形态 B 或 C。

## 2. 形态 B：跨机 MCP over HTTP（本轮打通）

### 2.1 板上起服务

`IRAF_MCP_*` 端点声明在装配单 `/etc/iraf/edge-intel.env`（默认 `stdio` + `127.0.0.1:8765`）。
跨机时用启动脚本覆盖 host 与允许的 Host 头：

```bash
# 板上 root
IRAF_PROVIDER_TOKEN=<provider token> \
IRAF_MCP_ALLOWED_HOSTS=127.0.0.1:8765,localhost:8765,10.203.247.147:8765 \
bash start_mcp_http.sh          # 日志 /var/log/iraf/mcp-http.log；--stop 停止
```

两个必须知道的行为（都是实测踩出来的）：
- **必须给 `IRAF_MCP_ALLOWED_HOSTS` 加板卡自己的 `host:port`**：MCP SDK 默认开 DNS
  rebinding 防护，只认 loopback 的 Host 头，否则跨机请求直接
  `421 Misdirected Request`。
- server 需要 `IRAF_PROVIDER_TOKEN`（跨机 Provider 鉴权）；缺了会 fail-closed。
  模型网关 key 从 `/etc/iraf/agentos.key` 读，不需要在命令行给。

### 2.2 跨机验证（现在就能跑，不需要麦）

x86 开发机（有 `mcp` 包的解释器，本仓库用 `build/mcp-venv/bin/python`）：

```bash
cd /home/coretek/AIIRAF
build/mcp-venv/bin/python scripts/probe_mcp_http.py \
  --url http://10.203.247.147:8765/mcp --text "抓取 box_01"
```

判绿标准（本轮实测）：
```
TOOLS ['iraf_submit_task', 'iraf_get_task']
status= SUCCEEDED | error_code= | skill= pick_object
execution_id= 1f2e486c-96d1-552b-afec-ac6d7e3f2332
查回 status= SUCCEEDED | execution_id= 1f2e486c-… | 同一条= True
```

失败判读（都是实测过的真现象）：
- `末端未到达目标抓取位姿: distance≈0.374m tolerance=0.005000m` ⇒ MuJoCo 场景**有状态**，
  上一次成功抓取把目标带走了。复跑前重启 x86 Provider（换干净场景）。
- `MuJoCo 抓取当前只接受 world 坐标系` ⇒ 意图解析这一次给出的抓取位姿不在 world 系
  （实测 2 次里出现 1 次，属模型输出不稳定；**这是正确 fail-closed，不是集成错**）。
  重跑一次即可；要根治得在意图层约束 frame 字段。

### 2.3 把语音宿主接上来（有麦之后）

有麦机器上装 AgentOS（或任何 MCP 宿主），登记远程 server —— HTTP 形态**不需要**
`PYTHONPATH`、不需要在板上有 Python 环境：

```bash
intewell-agent-cli mcp add iraf-remote --url http://10.203.247.147:8765/mcp
printf 'y\n' | intewell-agent-cli mcp add iraf-remote --url http://10.203.247.147:8765/mcp
intewell-agent-cli mcp test iraf-remote      # 期望：Connected, 2 tools discovered
```

然后语音：
```bash
intewell-agent-cli voice enroll --name <你的名字>     # 录 6 秒声纹
intewell-agent-cli voice selftest                   # 录 3 秒 + 转写 + TTS 回放
intewell-agent-cli voice listen                     # 唤醒词「intewell / 因特威尔」后说话
#   说：「抓取 box_01」→ agent 调 iraf_submit_task → 回来的是 status + execution_id
```
注意 `voice status` 里 `required=yes` 时**必须先 enroll 声纹**，否则唤醒词会被拒；
不想校验就把 `chat_cfg.json` 的 `require_speaker_verification` 设为 false。

如果宿主是 Hermes 这类通用 agent，同样只是"加一个 HTTP MCP server"，配置指向
`http://10.203.247.147:8765/mcp` 即可，无需改 IRAF 任何代码。

## 3. 形态 C：板上 AgentOS 自己的语音会话（未测）

`intewell-agent-cli voice` 走的是 AgentOS 的 `chat_cfg.json` 会话；它不在
`_command_has_dedicated_mcp_startup` 的例外清单里，也不是 `{None, chat, rl}` 的后台清单，
因此启动时会走**同步** `discover_mcp_tools()` ⇒ 工具在第一次 agent turn 之前就注册好，
理论上第一轮就能调任务级工具（比 dashboard 要等第二轮更干净）。
**未实测**：板上没有麦克风（`microphones: 0`），需换带音频的机器或厂商开放 `/dev/snd`。

## 4. 未闭合的安全项（必须说清楚）

- 当前 HTTP 形态**没有鉴权**：任何能连到 `:8765` 的机器都能提交任务（IRAF 内部仍有
  PolicyGateway 与 SafetyPolicy 把关，但"谁能发任务"没有被限制）。
- 上线前至少要做：只绑内网/专用网段、加 mTLS 或 bearer token（MCP HTTP 支持 header
  鉴权，宿主侧 `--auth header`）、限流与来源白名单。
- 在完成之前，**不要**把 `IRAF_MCP_HOST` 改成 `0.0.0.0` 后长期常驻；本轮只是现场验证。

## 5. ACP 通道的实测限制（不是我们的绑定问题）

`intewell-agent-cli acp` 多轮会话里，`/tools` 能列出
`mcp__iraf_task__iraf_submit_task`（20 个工具），但真实 turn 的模型说"工具集里没有 iraf_*"。
已在板上试过两个补丁（`acp_adapter/server.py` 的 prompt 开头刷新、`acp_adapter/session.py`
的 `_make_agent` 返回前刷新），**都未生效** —— 说明过滤点不在会话构造，而在 turn 的
请求组装。补丁脚本与还原方式：`patch_acp_session.py`（`--revert` 还原，原文件已备份为
`session.py.iraf-orig`）。这条作为厂商侧缺陷记录，**不作为平台级证据**，也不影响
形态 A（stdio，已实测通）与形态 B（HTTP，已实测通）。

## 6. 口径

本轮一切结论 **simulation=true**：执行端是 x86 MuJoCo，板上只跑 TaskFlow/Skill/Policy
与 MCP 入口；不构成真机能力证据（AGENTS §0.1 / 1.7）。
