# E300 双域：语义 → Skill → 跨机执行 首次打通（2026-10-19/20）

机器：智控域 10.203.247.147（Intewell R1.2.1 / aarch64 / Python 3.11.6 / glibc 2.38）
      运控域 10.203.247.149（同 OS，独立 VM，vm-1-net0 桥接到 br0）
      x86 仿真 Capability Provider：10.203.247.121:50052（headless，offscreen_frames）

## 1. 链路结论（有实测输出）

```
板上语义（LLM 意图解析）
  → IRAF TaskFlow / Skill Runtime / PolicyGateway（都在板上）
  → gRPC 跨机调用 x86 Capability Provider（Bearer token 鉴权）
  → MuJoCo 真跑（piper_pick 场景）
  → fail-closed 返回精确误差
```

最后一次实测（板上 CLI，`--text "抓取 box_01"`）：

```
resolved_skill: pick_object
adapter: agentos-intent       intent_provider: openai-compatible-intent 1.2.0 (deepseek-flash)
status: FAILED  error_code: IRAF-PROVIDER-ERROR
reason: 末端未到达目标抓取位姿: distance=0.014450m tolerance=0.005000m
        delta=[0.004229494155511354, 5.415238105131032e-06, 0.013817039392045947]
        qpos={'joint1': 3.784e-05, 'joint2': 1.1861, 'joint3': -0.12330,
              'joint4': 1.525e-06, 'joint5': -0.12034, 'joint6': -3.308e-08}
simulation: true
```

判读：**语义解析、策略判定、跨机鉴权、能力执行都已成立**（臂真的动了，qpos 有值）；
剩下的失败是"末端到不了抓取位姿"——误差 14.45 mm，其中 Z 向贡献 13.82 mm，容差 5 mm。
这是模型/标定层的残差问题，不是集成层问题。它与既有记录
`docs/debug/2026-09-24-joint-model-dog-arm.md §11.67`（residual_tolerance_m 已判死、
下一步判别实验是 align_max_attempts 3→6）是同一族现象。

## 2. 为打通链路修掉的四个真缺陷（均已入仓）

1. **网关不支持 `response_format=json_schema`**（400 `This response_format type is unavailable now`）
   ⇒ `src/iraf_adapters/model/openai_intent.py` 做降级链 `json_schema → json_object → none`，
   **降级不放宽校验**（仍用同一 JSON Schema）。逐字段探针：`scripts/probe_intent_gateway.py`。
2. **我把 `_validate_profile_semantics` 挤成嵌套函数**（插入新方法时结构错位），板上报
   `AttributeError: 'OpenAICompatibleIntentProvider' object has no attribute ...`
   ⇒ 修正类结构；补 `tests/unit/test_intent_provider.py`（8 例，含直接调 `parse()` 的用例）。
3. **`max_tokens` 写死 256 被推理型模型吃光**：`finish_reason=length`、`completion_tokens=256`、
   `content_len=0` ⇒ "模型未返回严格 JSON 意图"。改为可配（默认 1024，
   `IRAF_AGENTOS_MAX_TOKENS` 覆盖），并让报错带 `finish_reason/completion_tokens/content_len`。
4. **探针脚本把两个秘密混用**：`IRAF_PROVIDER_TOKEN`（跨机 Provider）与 `IRAF_AGENTOS_TOKEN`
   （模型网关）本是两回事，混用导致 provider 侧 `UNAUTHENTICATED`。探针已拆开。

部署侧另修两条（见 `deploy/sdk/board_deploy.sh`）：`config/install/` 含目录时必须 `scp -r`；
卸载路径不再强制 `--wheelhouse`。

## 3. 装机状态（两域分机，各自独立）

```
智控域 .147  edge-intel   /opt/iraf-sdk/0.2.0        /etc/iraf/edge-intel.env
                          iraf-runtime-grpc.service  CPUQuota=150% MemoryMax=1G  disabled/inactive
                          env: PROFILE=profiles/piper_mujoco.yaml
                               SAFETY_POLICY=profiles/safety/simulation_lab.yaml
                               RUNTIME_SUBJECT=board-e300-intel
                               endpoint=10.203.247.121:50052
                          --list-skills → 13 个技能
运控域 .149  edge-motion  /opt/iraf-sdk-motion/0.2.0 /etc/iraf-motion/edge-motion.env
                          iraf-control-provider.service CPUQuota=100% MemoryMax=512M disabled/inactive
                          （真机控制入口未定义 ⇒ 仍是 readiness 桩，铁律 6.8）
```

## 4. 环境事实（排查用）

- 智控域**原本没有默认路由**（只有 10.203.247.0/24 直连），到不了模型网关 192.168.100.89
  ⇒ 实测报 `Network is unreachable`。本次临时补：
  `ip addr del 10.203.247.147/24 dev br0 && ip addr add 10.203.247.147/23 dev br0 &&
   ip route add default via 10.203.246.1 dev br0`
  **注意这是临时命令，重启即失** ⇒ 要持久化必须写进该域的网络配置。
- br0 的端口是 eth2 + vm-0-net0 + vm-1-net0（`/sys/class/net/br0/brif/` 可查；
  `bridge link show` 在这台机器上输出为空，别据此误判"没挂进桥"——我误判过一次）。
- 板卡→x86 Provider：`tcp_50052=OPEN`，connect≈0.88 ms。
- 两个域都跑 Intewell R1.2.1、Python 3.11.6、glibc 2.38，与 `profiles/boards/e300.yaml`
  声明的 cp311/manylinux_2_28_aarch64 一致。

## 5. 未闭合

### 5.1 已拿到 SUCCEEDED 的闭环证据（2026-10-20 复测）

板上 CLI，`--text "停止机器人"`：
```
execution_id: 838347b7-dbf1-5606-9be6-096a5c2a2e64
resolved_skill: stop      status: SUCCEEDED      error_code: ""      退出码 0
```
更早一次 `--text "把 joint1 移动到 0.1 弧度"` 也拿到过 `status: SUCCEEDED` / `resolved_skill: move_joint` / 退出码 0。
⇒ 「自然语言 → Skill → 板上策略 → 跨机 Provider 执行」在这条链上是通的，且终态如实。

同一轮修掉的第四个缺陷：`stop` 被执行层拒绝
`IRAF-CAPABILITY-UNKNOWN 能力 'stop' 未登记参数形状` —— 根因是
`provider_service._POSITIONAL` 只登记了 `emergency_stop`，没有技能名 `stop`。
已补 `"stop": ()`，并加 `tests/unit/test_capability_registry.py` 做"技能目录 ↔ 能力表"静态对账
（AST 解析，不需要 grpc）。**注意：provider 是 x86 上的常驻进程，改完必须重启它才生效**——
本轮就因此多失败了一次（板上装了新代码但旧 Provider 仍在跑）。

### 5.2 已闭合：pick_object 的残差根因是**缺省时长**，不是标定

板上 `抓取 box_01` 之前报 `末端未到达目标抓取位姿 distance=0.014450m tolerance=0.005000m`。
打开 `IRAF_DEBUG_PICK=1` 抓到相位账本：

```
PICK_INPUTS {"duration_ms": 1000, "phase_ms_each": 200, ...}
PICK_TRACE  HOME_HOLD  center_distance_m=0.163883  (起点)
PICK_TRACE  APPROACH   center_distance_m=0.071105
PICK_TRACE  DESCEND    center_distance_m=0.014450   ← 门禁读数，仍 > 0.005
```

根因：`src/iraf_skills/common/manipulation.py` 里 `duration_ms` 缺省 **1000 ms**，
而 backend 按 5 段切分（`phase_ms = duration_ms // 5`）⇒ 每段只有 **200 ms**；
而 `mujoco_backend` 相位注释里的标定值是**每段 1600 ms（总 8000 ms）**。
即"代码默认值比标定值短 8 倍"，臂根本没有收敛时间。**不是标定残差，也不是容差问题。**

修法（commit 8854a1a）：新增 `PICK_NOMINAL_DURATION_MS = 8000`、`PLACE_NOMINAL_DURATION_MS = 6400`
（分别对应 5 段 / 4 段 × 1600 ms），把缺省值改为标定值；**判据 tolerance 未动**。

修后实测（关闭 debug，干净复跑，板上 `--text "抓取 box_01"`）：

```
status: SUCCEEDED      resolved_skill: pick_object      simulation: true
execution_id: 1e8171e9-160e-5c00-a307-e7767bb2b2e1
policy: 1.0.0  decision: 8abded60-f884-468e-b2b6-76befad0cb8f
accepted: true         confirmation: "constraint"       target_id: box_01
pose_source: live_target_body
grasp_alignment.center_distance_m = 9.387527253937005e-06   ← 9.39 µm（容差 5 mm）
bilateral_contact = true    force_ok = true
lift_delta_m = 0.092124     lift_peak_delta_m = 0.080084     lift_attitude_max_deg = 21.470857
regrasp.applied = true      regrasp.force_imbalance_ratio = 1.0155
```

即残差从 **14.45 mm → 9.39 µm**，`grasped/lifted` 均为 true，终态 SUCCEEDED。

补充：此前还出现过一次 `distance=0.163883m` 且 `qpos` 全 0 的异常，是在同一"相位过短"条件下
发生的首调异常；改缺省时长后 1 次 debug 复跑 + 1 次干净复跑**都 SUCCEEDED**，未再复现
（样本仅 2 次，若再出现需单独追）。

### 5.3 其余未闭合

1. ~~pick_object 残差~~ —— **已闭合**，见 5.2（根因是缺省时长，不是标定容差）。
2. ~~智控域默认路由是临时命令~~ —— **已持久化**（见 §6）；运控域仍无默认网关（按需再加，见 §6.2）。
3. 运控域 edge-motion 仍是桩；真机入口未定义，不能声称运控能力可用。
4. 语义层目前在**板上 IRAF CLI 内部**用 LLM 解析（`IRAF_AGENTOS_*`），板上 AgentOS 仍未安装；
   与"AgentOS 发语义"的最终形态还差一层绑定（MCP/子进程，见技能阶段 4）。
5. `visual_pick` 也会被模型选中（"抓取 box_01" 曾被解析成 visual_pick）——
   意图歧义属模型行为，若要固定技能需在提示词/契约层收敛，勿用放宽校验解决。

## 6. 最终状态（网络 + 双域，2026-10-20 收尾）

### 6.1 地址与持久化

```
智控域  10.203.247.147   edge-intel     br0: 10.203.247.147/23  gw 10.203.246.1
运控域  10.203.247.158   edge-motion    eth0: 10.203.247.158/24  （无默认网关）
x86    10.203.247.121   Capability Provider :50052
```

- **运控域换过地址**：原先 10.203.247.149 在办公网上**被另一台设备占用**
  （我这侧 ARP 看到 6a:cc:09:c2:e0:ea，而桥内真实运控域是 fe:fe:00:01:00:00）⇒
  外部访问被抢答，表现为"宿主/串口侧通、办公网侧不通、且时通时不通"。改到 .158（实测空闲）后正常。
- **智控域网络已持久化**（原来是手敲的临时 `ip addr/route`）：
  `nmcli con mod br0 connection.autoconnect yes`
  实读：`autoconnect=yes / ipv4.method=manual / addresses=10.203.247.147/23 / gateway=10.203.246.1`，
  `ip route` 显示 `default via 10.203.246.1 dev br0 proto static metric 425`。
- **运控域**：`nmcli con mod eth0 connection.autoconnect yes`（原来 no，重启不会自己起）。
  **仍无默认网关**（/24 掩码；运控域是被调用方，不需要出网段）。
  将来若它要主动访问 x86 或模型网关，需先改 /23 再加网关：

```
nmcli con mod eth0 ipv4.addresses 10.203.247.158/23 ipv4.gateway 10.203.246.1
nmcli con up eth0
ip route; ping -c2 10.203.246.1; ping -c2 10.203.247.121
```

  ⚠ 在 /24 掩码下执行 `ip route add default via 10.203.246.1` 必然失败
  （`Nexthop has invalid gateway`）——网关必须与接口处于同一子网。

### 6.2 两域最终装机状态

```
智控域 .147  edge-intel   /opt/iraf-sdk/0.2.0          unit CPUQuota=150% MemoryMax=1G  disabled
                          --list-skills → 13 技能
                          语义实跑 SUCCEEDED ×3：stop / move_joint / pick_object
                          pick: center_distance=9.39e-06 m, lifted=true, lift_delta=0.092124 m
运控域 .158  edge-motion  /opt/iraf-sdk-motion/0.2.0   unit CPUQuota=100% MemoryMax=512M disabled
                          仍是 readiness 桩（真机控制入口未定义，铁律 6.8）
```

复装流程都是"受控卸载 → 重装"：卸载逐条打印删除项并保留 `/var/lib|log/iraf*`，
卸载后六路残留核验 = 0；安装 exit 0，自检 `iraf_sdk-ok+stubs`。

### 6.3 演示入口（手动启动 / 脚本）

| 用途 | 入口 | 说明 |
|---|---|---|
| 板上语义（无显示） | `python3 -m iraf_adapters.agentos.cli --text "抓取 box_01"` | 需 `cd /opt/iraf-sdk/current`（相对路径 profile）+ env 文件 + 两个 token |
| x86 跨机执行端 | `bash /home/coretek/scripts/x86_provider_50052.sh [--headless]` | 听 0.0.0.0:50052；`--self-check` 只验装配 |
| 本机纯显示 | `python3 scripts/view_mujoco.py` | 开 MuJoCo 窗口跑一次抓取（无 LLM） |
| 本机语义 + 显示 | `python3 scripts/agentos_view.py --text "抓取 box_01"` | LLM 解析 → display_pick → 开窗边抓边看 |
| 录屏 | `bash scripts/record_demo.sh [输出] [秒数] [--whole-screen]` | 抓 MuJoCo 窗口或整屏 |

### 6.4 MCP 的定位（回答"IRAF 需要 MCP 吗"）

**不需要**。当前两条通道已覆盖：
- 语义 → 任务：`iraf_adapters.agentos.cli`（板上 CLI，内部调 LLM 做受约束意图解析）
- 能力 → 执行：`CapabilityProviderService`（gRPC，跨机到 x86 仿真 Provider）

MCP 只在**"板上有 AgentOS，要让它的 LLM 把 IRAF 当工具调"**时才需要：用
`intewell-agent-cli mcp add <name> --url|--command …` 把 IRAF 暴露成 MCP 工具
（内部只调 IRAF 公开入口，不得旁路 Policy）。当前板上没装 AgentOS ⇒ MCP 无对象。

## 7. 四个常见问题的定点回答（含历史沿革）

1. **运控域无默认网关怎么办**：见 §6.1 的命令块。要点：网关必须与接口同子网，
   所以先 `ipv4.addresses=10.203.247.158/23` 再 `ipv4.gateway=10.203.246.1`；
   /24 下直接加会 `Nexthop has invalid gateway`。当前不需要它出网段（它是被调用方）。
2. **演示怎么手动启动**：见 §6.3 的入口表。四类入口分别是
   板上语义（`iraf_adapters.agentos.cli`）、x86 执行端（`scripts/iraf_provider_serve.py`）、
   本机纯显示（`scripts/view_mujoco.py`）、本机语义+显示（`scripts/agentos_view.py`）、
   录屏（`scripts/record_demo.sh`）。
3. **之前版本的演示场景也是 view_mujoco.py 吗**：是。沿革（`git log --follow`）：
   - 2026-09-11 `a62e6c9 feat: add Ubuntu MuJoCo viewer launcher` ⇒ `scripts/view_mujoco.py` 成为
     MuJoCo 演示的显示入口（此前没有 viewer 入口）；
   - `scripts/view_piper_mujoco.py` 只是**兼容转发**入口（文件头已标 deprecated，内部 import
     `view_mujoco` 并转发参数）；
   - 2026-10-16 新增 `scripts/agentos_view.py` 作为"语义 → 显示"的薄桥，其显示通道**仍然是**
     `scripts/view_mujoco.py`，没有另起一套。
4. **IRAF 需要 MCP 吗**：不需要，见 §6.4。MCP 只在"板上有 AgentOS、要把 IRAF 当工具调"时才引入；
   当前板上无 AgentOS，语义在 IRAF CLI 内部完成，能力走 gRPC `CapabilityProviderService`。

## 8. AgentOS 绑定：任务级 MCP（2026-10-20 晚，按使用者纠正重做）

**纠正**：AgentOS 的 skills 与 IRAF Skill Manager 管的机器人 Skill **不是一个概念**，
不得互相注册。第一版 MCP server 暴露了 `iraf_list_skills` / `iraf_execute_skill`
（等于允许 AgentOS 绕过 Skill Manager 直连 skill.execute），已废弃并删除注册。

现在（正确层次）`scripts/iraf_mcp_server.py` 只暴露两个**任务级**工具：

| 工具 | 作用 |
|---|---|
| `iraf_submit_task(text, deadline_seconds=120)` | 提交一句任务语义 → IRAF TaskFlow；**技能选择/参数校验/策略判定都由 IRAF Skill Manager 做** |
| `iraf_get_task(execution_id)` | 读 `IRAF_EVENT_STORE`（`SqliteExecutionStore.get`）返回权威执行事实 |

回归门禁：`tests/unit/test_mcp_server_layer.py` 用 AST 断言"只允许任务级工具名"，
一旦有人加回技能级工具名即失败。本地 2 例通过。

板上登记（智控域 .147）：
```
intewell-agent-cli mcp remove iraf
intewell-agent-cli mcp add iraf-task --command /usr/bin/python3 \
  --env IRAF_PROVIDER_TOKEN=… IRAF_ENV_FILE=/etc/iraf/edge-intel.env \
        PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
        IRAF_AGENTOS_MODEL_URL=http://192.168.100.89:3000/v1 IRAF_AGENTOS_MODEL=deepseek-flash \
  --args /opt/iraf-extras/iraf_mcp_server.py
```
模型网关 key 放 `/etc/iraf/agentos.key`（600），server 自己读，不写进 config.yaml。

**卡点（AgentOS 侧，不是我们接口的问题）**：`-z` 一次性会话里看不到 MCP 工具。
证据链：
- `intewell-agent-cli mcp test iraf-task` → ✓ Connected (893ms)、2 tools discovered；
- `mcp list` → iraf-task `✓ enabled`；`~/.intewell/logs/mcp-stderr.log` 里 server 被启动多次；
- `-z "...列出含 iraf 的工具..." -t all --yolo` → 模型回"没有"；
- 代码：`agentos/capabilities.py::assemble_capabilities(background_mcp=True)` 把 MCP 发现放在**后台线程**，
  工具靠 `agent/turn_context.py` 在**下一轮**刷新才被采纳；`-z` 只有一轮 ⇒ 永远看不到；
- 把 `mcp_discovery_timeout` 从 1.5 调到 20 也不生效（该参数用于 tui_gateway 的等待，不是 one-shot）。
- 交互式 REPL 另有一个坑：登录 shell 注入了 `INTEWELL_AGENT_HOME`（指向另一套安装）⇒
  CLI 报 "It looks like Intewell isn't configured yet"；`env -u` 也未能绕过，需单独定位。

**可用的验证路径**：dashboard（`http://10.203.247.147:9119/sessions`，admin/123）的会话是多轮的，
`tui_gateway/ws.py` 明确调用 `start_background_mcp_discovery`，因此**第一轮之后**工具会被采纳：
先发一句闲聊，再让它"调用 iraf-task 的 iraf_submit_task 提交：抓取 box_01"。

### 8.1 任务级 MCP 接口的直连验证（不经过 AgentOS，2026-10-20）

用 MCP 客户端直连我们的 server（`scripts/probe_mcp_client.py`，板上副本 /opt/iraf-extras/）：

```
TOOLS ['iraf_submit_task', 'iraf_get_task']
status= SUCCEEDED | error_code= | skill= pick_object |
execution_id= 550ff284-e817-5527-8dca-02ae446b1623
查回 status= SUCCEEDED | execution_id= 550ff284-e817-5527-8dca-02ae446b1623
```

⇒ 「任务级工具 → IRAF TaskFlow → Skill Manager 选技能 → Policy → 跨机 Provider」这条链在
**MCP 这层**是成立的；`iraf_get_task` 读的是事件库（`SqliteExecutionStore.get`），不是缓存。

**可复跑的前提（重要）**：MuJoCo 场景是**有状态**的。上一次成功抓取会把 box_01 从原位带走，
同一 Provider 实例再抓必然 `末端未到达目标抓取位姿 distance≈0.374m`（实测数字）。复跑演示前要
**重启 Provider 换干净场景**：
```
P=$(pgrep -f "[i]raf_provider_serve" | head -1); kill "$P"
bash /home/coretek/scripts/x86_provider_50052.sh --headless
```

### 8.2 仍未打通：AgentOS 会话装载 MCP 工具

见 §8 的"卡点"。结论：接口层已验证，卡点在 AgentOS 的
`background_mcp=True`（后台发现 + 下一轮刷新），`-z` 单轮会话结构上看不到工具；
dashboard 会话是多轮，等待使用者在界面复验。
