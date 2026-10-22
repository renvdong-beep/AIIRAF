# IRAF 的北向绑定：任务级 MCP（2026-10-20 定稿）

本文只讲**IRAF 对外怎么被平台调用**。路径/端口/单元/组件清单的唯一事实来源仍是
`config/install/install_profiles.yaml`；构建目标来自 `config/sdk/package_matrix.yaml`。

## 1. 两个概念不许混（这是本文件存在的理由）

| | 归谁管 | 是什么 |
|---|---|---|
| **机器人 Skill** | IRAF `Skill Manager`（`skills/<name>/skill.yaml`） | 契约（input/output schema）、preconditions、timeout/recovery、safetyClass、providers（entrypoint/selector）。执行链固定：TaskFlow → Skill Runtime → Policy → Capability Provider |
| **AgentOS 的 skills / agent 能力** | AgentOS 平台 | 平台自己的工具与工作流编排，**与机器人 Skill 不是一回事** |

因此：**不得把 IRAF 的机器人 Skill 注册成 AgentOS 的 skill/MCP 工具**。那等于让平台绕过
Skill Manager 直接 `skill.execute`，把契约校验、策略判定和证据链都跳过。

## 2. 正确形状：只暴露任务级入口

`src/iraf_adapters/mcp/server.py`（stdio，`python3 -m iraf_adapters.mcp.server`；也支持
`IRAF_MCP_TRANSPORT=http` 的 streamable-http）提供三个**任务级**工具：

| 工具 | 入参 | 行为 |
|---|---|---|
| `iraf_robot_mode` | `action=enter|status|exit`、`ttl_seconds=1800` | 机器人开发模式：只读装配事实（技能清单/profile/策略/端点/simulation）+ 置一个会话级模式标记。**不选技能、不执行动作** |
| `iraf_submit_task` | `text`（自然语言任务语义），`deadline_seconds=120` | 交给 IRAF TaskFlow；**技能选择、参数校验、策略判定全在 IRAF 内部**；返回里带 `robot_mode` |
| `iraf_get_task` | `execution_id` | 读 `IRAF_EVENT_STORE`（`SqliteExecutionStore.get`）返回权威执行事实 |

#### 为什么加 `iraf_robot_mode`（语义触发，2026-10-21 使用者口径）

平台自己的 skill 与 IRAF 的机器人 Skill 是两套概念；一句话有歧义（"抓取/放置"既可能是
平台动作也可能是机器人任务）时，平台模型容易走偏。解决办法**不改别人的容器或框架**，
只在 IRAF 自己的北向上加一个显式模式：

```
用户："开始机器人开发模式"
   → 平台调用 iraf_robot_mode(action="enter")
   → 返回 mode=robot_dev + 13 个机器人 Skill 清单 + profile/策略/端点/simulation
   → 之后同一会话的 iraf_submit_task 回传 robot_mode=robot_dev
用户："结束机器人开发模式" → iraf_robot_mode(action="exit") → mode=idle
```

实测（板上 0.3.5，HTTP 跨机）：`TOOLS ['iraf_robot_mode','iraf_submit_task','iraf_get_task']`；
`status` → `mode=idle | skills=13 | simulation=True`；`enter` → `mode=robot_dev, remaining_s=599`；
`submit` → `SUCCEEDED | skill=pick_object | robot_mode=robot_dev |
execution_id=c73aadf7-fb53-5e35-8afa-a9d529d623a4`；`exit` → `mode=idle`。

边界：模式是**语义收口**，不是安全边界——安全上限仍只来自签名的 RobotProfile 与
SafetyPolicy，`iraf_robot_mode` 既不放大权限也不跳过 PolicyGateway。

数据流：

```
AgentOS（语义层，自己的 agent/skills）
   │  iraf_submit_task(text)         ← 只给任务，不指定 skill
   ▼
IRAF TaskFlow → Skill Runtime → Skill Manager 选技能 → Policy Gateway → Capability Provider
   ▲
   └── iraf_get_task(execution_id) 读事件库，回答"到底发生了什么"
```

## 3. 注册方式（平台侧）

```bash
# 板上（root）。key 放 600 文件，不要写进 config.yaml
intewell-agent-cli mcp add iraf-task --command /usr/bin/python3 \
  --env IRAF_PROVIDER_TOKEN=<跨机Provider token> \
        IRAF_ENV_FILE=/etc/iraf/edge-intel.env \
        PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
        IRAF_AGENTOS_MODEL_URL=<模型网关>/v1 \
        IRAF_AGENTOS_MODEL=<模型名> \
  --args -m iraf_adapters.mcp.server
```

环境变量含义：

| 变量 | 必需 | 说明 |
|---|---|---|
| `IRAF_ENV_FILE` | 是 | 装配单（profile / 安全策略 / skill root / 跨机 endpoint / event store） |
| `IRAF_PROVIDER_TOKEN` | 是 | 跨机 Capability Provider 的鉴权；缺则 fail-closed |
| `IRAF_AGENTOS_MODEL_URL` / `IRAF_AGENTOS_MODEL` | 是 | IRAF 内部**受约束**意图解析（把任务语义映射到契约内技能） |
| `IRAF_AGENTOS_TOKEN` 或 `IRAF_AGENTOS_KEY_FILE` | 是 | 模型网关 key；默认从 `/etc/iraf/agentos.key`（600）读 |
| `IRAF_VERSION_DIR` | 否 | 默认 `/opt/iraf-sdk/current` |

## 4. 验证（三级，逐级加证据）

0. **跨机（HTTP）**：`IRAF_MCP_TRANSPORT=http` + `IRAF_MCP_ALLOWED_HOSTS` 起服务，
   开发机用 `scripts/probe_mcp_http.py --url http://<板>:8765/mcp` 直连。
   实测 2026-10-21：`SUCCEEDED` / `1f2e486c-96d1-552b-afec-ac6d7e3f2332`（读回同一条）。
   道路与语音接入方式见 `docs/voice-to-task-path.md`。

1. **接口级（不经过平台）**：`scripts/probe_mcp_client.py` 直连 server，断言
   `TOOLS == ['iraf_submit_task','iraf_get_task']`、`status=SUCCEEDED`、`iraf_get_task` 读回同一
   `execution_id`。板上实测 2026-10-20：`550ff284-e817-5527-8dca-02ae446b1623` / SUCCEEDED。
2. **平台级**：平台会话里让模型调用这两个工具（见 §5 的已知限制）。
3. **可复跑前提**：MuJoCo 场景**有状态** —— 成功抓取会把目标带走，同一 Provider 实例再抓必然
   `末端未到达目标抓取位姿 distance≈0.374m`（实测）。演示前重启 Provider 换干净场景。

## 5. 已知限制（2026-10-20 实测；2026-10-21 在 AgentOS 1.0.0 上复核，见
`docs/debug/2026-10-21-agentos-1.0.0-mcp-binding.md`）

- AgentOS **CLI `-z`（单轮）看不到 MCP 工具**：`agentos/capabilities.py` 用
  `assemble_capabilities(background_mcp=True)` 在后台线程发现 MCP，工具要靠
  `agent/turn_context.py` 在**下一轮**才被采纳；单轮结构上不可能可见。
  调大 `mcp_discovery_timeout`（1.5s → 20s）无效（该参数属于 tui_gateway 的等待）。
  **1.0.0 仍是这个行为**（实测 `-z` 答「无」）。
- **`intewell-agent-cli acp` 天生没有 MCP 工具**（1.0.0 新增实测）：`main.py` 把 ACP 归入
  “自己后面会做”的入口，但 `acp_adapter/` 并未调用 `start_background_mcp_discovery`，
  所以多轮也调不出 `iraf_submit_task`（模型原话：工具集里没有这个工具）。
- 多轮会话（dashboard / `tui_gateway`）会调 `start_background_mcp_discovery`，**第一轮之后**可用。
  ⚠ 它是**进程启动时**发现一次：`mcp add` 之后若 dashboard 未重启，进程里仍是旧配置。
- 交互式 REPL / `--cli` 在登录 shell 下会撞到 `INTEWELL_AGENT_HOME`（指向另一套安装），
  CLI 报 "It looks like Intewell isn't configured yet"；**1.0.0 重装后另有原因**：
  `/root/.intewell/.env` 不存在（无 provider）⇒ 交互通道直接进 setup 引导。
  补 `OPENAI_BASE_URL` + `OPENAI_API_KEY`（复用 `/etc/iraf/agentos.key`）后恢复。
- dashboard 只认表单登录（`/auth/password-login` + cookie）；`/api/auth/ws-ticket` 需要登录态，
  Basic 认证 401 ⇒ 程序化复验需要可登录的 admin 口令（或人工在页面上发两轮）。

## 6. 回归门禁

`tests/unit/test_mcp_server_layer.py`（AST 静态检查，不需要安装 `mcp`）断言：

- 只允许 `iraf_submit_task` / `iraf_get_task` / `iraf_robot_mode` 三个**任务级**工具名；
- 禁止出现 `iraf_execute_skill` / `iraf_list_skills` / `iraf_run_skill` 等技能级工具；
- `scripts/iraf_mcp_server.py`（兼容转发入口）不得再定义工具，避免两套实现。

## 7. 对接原则：只从 IRAF 侧对接（2026-10-21 使用者口径）

> 「我们不应该修改其他人的容器或者框架，我们只能从 iraf 的本身去对接。」

因此：

- **不改** AgentOS 的容器镜像、worker 配置、包目录或源码补丁。曾试过的若干改动
  （worker 容器里 pip 装 `mcp`、写 `/home/agentos/.intewell/config.yaml`、
  补 `tui_dist/package.json`、给 ACP 打补丁）**已全部回滚**，只作为"厂商需补齐"的证据保留。
- **只加** IRAF 自己的东西：`iraf_robot_mode`（语义收口）、HTTP 传输（跨机可达）、
  契约收窄（`frame_id const world`）。这些都不依赖平台内部实现。
- 平台侧需要满足的最小条件（属厂商/集成方职责，不由我们代改）：
  1. 宿主或 worker 镜像里要有 MCP 客户端（`mcp>=1.26,<2`）；实测 worker 镜像缺这个包；
  2. 会话进程要能读到 `mcp_servers` 配置（worker 的 home 与宿主是两套）；
  3. 多轮会话必须有 MCP 发现（1.0.0 的 dashboard 有、ACP 没有）。

任一条不满足时的兜底（不碰对方）：`iraf_robot_mode` + `iraf_submit_task` 仍可通过
IRAF 自带 CLI（`deploy/sdk/board_text_task.sh`）或 HTTP MCP（`:8765`）直接使用。

## 8. 机器人开发模式的落地规格（A + C，2026-10-21 定）

**分工**：判断权在平台（项目/代码/文件/构建/对话这些事务仍由 AgentOS 处理），
判定权在 IRAF（这句话算不算机器人任务、该用哪个技能）。要的是 **robot-first 优先**，
不是把每句话都硬塞给 IRAF。

### 8.1 IRAF 侧提供什么（已实现，0.3.8）

| 入口 | 返回 | 用途 |
|---|---|---|
| `iraf_enter_robot_mode()` | `mode=robot_dev`、`skills`（13 个）、`intents`（每技能意图示例）、`routing_policy`（分派守则）、`profile`、`safety_policy`、`provider_endpoint`、`simulation` | 进入模式并一次性拿到**路由表**，平台据此判断"该不该交给 IRAF" |
| `iraf_submit_task(text)` | `status / resolved_skill / execution_id / robot_mode` | 交给 TaskFlow → Skill Manager → Policy → Provider |
| 无关请求 | `error_code=IRAF-NOT-ROBOT-TASK`（reason 里列可用技能） | **过滤信号**：平台据此回退到自己的处理 |
| `iraf_get_task(id)` | 事件库里的权威执行事实 | 核对结果，不采信模型自述 |

`routing_policy` 是**建议性文本**，不是开关；真正过滤发生在 IRAF 意图层（判不出来就
`IRAF-NOT-ROBOT-TASK`），所以它不会放宽任何安全边界。

### 8.2 平台侧要加的那一条规则（可直接粘贴进会话配置 / skill 文本）

> 当用户说「开始机器人开发模式」或「进入机器人开发模式」时，调用
> `mcp__iraf_task__iraf_enter_robot_mode`。
>
> 该模式生效期间：
> 1. 凡**可能涉及机器人本体或物理动作**的请求（移动、抓取、放置、标定、视觉、停止/急停），
>    **必须先调用** `mcp__iraf_task__iraf_submit_task(text=<用户原话>)`，
>    由 IRAF 的 Skill Manager 选技能并执行；不要先在平台侧自行解释或执行。
> 2. 若返回 `error_code=IRAF-NOT-ROBOT-TASK`，说明该请求与机器人无关，
>    按你自己的工具与流程处理（**不要**把它当成机器人失败，也不要包装成成功）。
> 3. 平台自身事务（项目、代码、文件、构建、对话）由你判断处理，**不要**交给 IRAF。
> 4. 需要核对执行事实时调用 `mcp__iraf_task__iraf_get_task(execution_id)`，不要采信模型自述。
> 5. 用户说「结束机器人开发模式」时调用 `mcp__iraf_task__iraf_exit_robot_mode`。

宿主若支持 MCP prompt，可直接取 `robot_routing_policy` 的内容注入，等价于上面的守则。

### 8.3 三条验收用例（缺一不可）

| 输入 | 期望 |
|---|---|
| `抓取 box_01` | IRAF `SUCCEEDED` + `execution_id`；`iraf_get_task` 回查同一 id；`robot_mode=robot_dev` |
| `今天天气怎么样` | 先交给 IRAF → `IRAF-NOT-ROBOT-TASK`（reason 列技能）→ 平台回退自己回答；**不产生机器人动作** |
| `看下当前目录有哪些文件` | 平台自己处理，**不调用** IRAF（不得被误吞） |

判绿要同时给两样证据：事件库里那条 execution 的 `status`，以及平台侧"没有产生机器人动作"的记录。
口径仍 `simulation=true`，不构成真机能力证据。

### 8.4 传输与常驻

- 同机宿主（AgentOS 在本机拉起）：用 **stdio**，不需要端口。
- `IRAF_MCP_TRANSPORT=http`（`:8765`）只用于**跨机宿主**（例如语音前端在有麦的机器上）
  或从开发机做跨机验收；当前**无鉴权**，按需拉起、用完即停，上线前必须加 token/mTLS。
- 平台侧前置条件仍见 §7（MCP 客户端、能读 `mcp_servers`、多轮会话有 MCP 发现）。

## 9. 让平台"主动用我们的"，而不是自己重新实现（2026-10-22）

实机证据（板上会话 `20261022_015831_e9`）：AgentOS 在机器人任务里 **0 次**调用
`mcp__iraf_task__*`，却用 `host_command` 读了我们十几个源文件、自己跑脚本，还就地修改了一个
平台 skill `iraf-mcp-task-submit`（内容是"用 shell 拉起我们的 stdio MCP"，且写着板上已不存在的
`/opt/iraf-extras/*`）。它没有"不会用"，而是**被一条旁路 skill 指挥着绕开主路**。

### 9.1 IRAF 侧抓手（0.3.9 已实现，纯我方）

| 抓手 | 内容 |
|---|---|
| 工具描述硬约束 | `iraf_submit_task` 明写"机器人动作的**唯一受支持入口**"，禁止 `execute_code`/`terminal`/`host_command` 自行驱动，禁止改源码迁就一次失败 |
| 返回里给下一步 | 每次 submit 都带 `next_steps`：成功→怎么核验；`distance≈0.374m`→场景状态、先复位 Provider、别改代码；`IRAF-NOT-ROBOT-TASK`→交给平台 |
| 资源里给案例 | `iraf://examples/pick_box_01`、`iraf://examples/filtered_request`、`iraf://skills/catalog`（技能+意图示例），模型可引用既有案例而不用重写 |
| 模式返回里给示范 | `iraf_enter_robot_mode` 返回 `skills` / `intents` / `routing_policy` / `examples`（成功+被过滤各一例） |

实测（0.3.9，跨机）：`TOOLS` 5 个、`RESOURCES` 3 个；`mode=robot_dev | skills=13 |
intents=13 | examples=['robot_task','non_robot']`；过滤用例返回
`IRAF-NOT-ROBOT-TASK` 且 `next_steps` 明确"由平台自己处理、不要重试"。

### 9.2 平台内容层（属其配置/内容，可回滚，不是改框架）

- **停用旁路 skill** `~/.intewell/skills/iraf-mcp-task-submit`（整目录移到备份区）。
- **注入路由守则**到 `~/.intewell/SOUL.md`（每会话生效），内容与 §8.2 一致。
- 两件事都有脚本与备份：`apply_agentos_routing.py`（`--revert` 可原样还原）。
- 可选（更硬的约束）：机器人开发模式下收紧平台工具集，禁用 `execute_code`/`terminal`/`host_command`。

### 9.3 运行保障

演示/验证前必须复位仿真场景（重启 x86 Capability Provider），否则必现
`distance≈0.374m tolerance=0.005000m`；模型会把这种环境状态误判成实现 bug，
进而自己"修"代码。同一时间只跑一路任务（并发搬运会让目标匀速位移）。
