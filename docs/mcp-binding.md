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

`src/iraf_adapters/mcp/server.py`（stdio，`python3 -m iraf_adapters.mcp.server`）只提供两个工具：

| 工具 | 入参 | 行为 |
|---|---|---|
| `iraf_submit_task` | `text`（自然语言任务语义），`deadline_seconds=120` | 交给 IRAF TaskFlow；**技能选择、参数校验、策略判定全在 IRAF 内部** |
| `iraf_get_task` | `execution_id` | 读 `IRAF_EVENT_STORE`（`SqliteExecutionStore.get`）返回权威执行事实 |

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

1. **接口级（不经过平台）**：`scripts/probe_mcp_client.py` 直连 server，断言
   `TOOLS == ['iraf_submit_task','iraf_get_task']`、`status=SUCCEEDED`、`iraf_get_task` 读回同一
   `execution_id`。板上实测 2026-10-20：`550ff284-e817-5527-8dca-02ae446b1623` / SUCCEEDED。
2. **平台级**：平台会话里让模型调用这两个工具（见 §5 的已知限制）。
3. **可复跑前提**：MuJoCo 场景**有状态** —— 成功抓取会把目标带走，同一 Provider 实例再抓必然
   `末端未到达目标抓取位姿 distance≈0.374m`（实测）。演示前重启 Provider 换干净场景。

## 5. 已知限制（2026-10-20 实测，未闭合）

- AgentOS **CLI `-z`（单轮）看不到 MCP 工具**：`agentos/capabilities.py` 用
  `assemble_capabilities(background_mcp=True)` 在后台线程发现 MCP，工具要靠
  `agent/turn_context.py` 在**下一轮**才被采纳；单轮结构上不可能可见。
  调大 `mcp_discovery_timeout`（1.5s → 20s）无效（该参数属于 tui_gateway 的等待）。
- 多轮会话（dashboard / `tui_gateway`）会调 `start_background_mcp_discovery`，**第一轮之后**可用。
- 交互式 REPL / `--cli` 在登录 shell 下会撞到 `INTEWELL_AGENT_HOME`（指向另一套安装），
  CLI 报 "It looks like Intewell isn't configured yet"；需单独定位。

## 6. 回归门禁

`tests/unit/test_mcp_server_layer.py`（AST 静态检查，不需要安装 `mcp`）断言：

- 只允许 `iraf_submit_task` / `iraf_get_task` 两个工具名；
- 禁止出现 `iraf_execute_skill` / `iraf_list_skills` / `iraf_run_skill` 等技能级工具；
- `scripts/iraf_mcp_server.py`（兼容转发入口）不得再定义工具，避免两套实现。
