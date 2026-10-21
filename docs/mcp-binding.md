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
