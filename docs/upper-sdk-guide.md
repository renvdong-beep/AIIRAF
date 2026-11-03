# IRAF 上位机 SDK 交付与使用指南

版本：0.4.0（2026-11-03）  
适用对象：在**自己的 x86 Ubuntu** 上开发上位机 harness / LLM agent，通过任务级接口调用**边缘侧 IRAF** 的团队。  
口径：一切执行结论 `simulation=true`，不构成真机/实时能力证据。

> 南向 Provider SDK（对方 MuJoCo 被边缘调用）见 `docs/provider-sdk-guide.md`。

---

## 0. 交付物与状态（先看这一节，避免误用）

### 0.1 本指南覆盖、且已经可用

| 能力 | 状态 | 证据 |
|---|---|---|
| 任务级 MCP（5 个工具：enter/exit/robot_mode/submit/get） | 可用 | HTTP/stdio 实测，5 tools |
| MCP HTTP 鉴权（Bearer token、scope、资源绑定、审计、热撤销） | 可用 | 401/403/热撤销/跨调用方隔离实测 |
| 上位机 Python 客户端 `iraf_sdk.mcp.McpToolClient` | 可用 | 实测 list_tools / enter / submit / get |
| 南向 Provider SDK `iraf_sdk.provider.ProviderKit` | 可用（核心） | gRPC 冒烟：Health/Invoke/幂等/鉴权/缺 deadline 拒绝 |
| 既有北向客户端 `iraf_sdk.HttpSkillClient` / `GrpcSkillClient` | 可用（既有） | 原 SDK 能力，未改动 |
| 旧 AgentOS stdio 绑定 | **保持兼容** | 5 个工具名不变、stdio 不要求 token、env 文件身份 |

### 0.2 尚未交付（不要对外宣称）

- conformance kit（`iraf-provider-selftest` / `iraf-conformance` / `iraf-doctor`）——未实现。
- Profile/SafetyPolicy 一键校验 CLI 的对外封装——仓库有 `scripts/profile_check.py`，但未做成 SDK 命令。
- mTLS 的 CN→subject 绑定（当前由反向代理终止 mTLS 或只用 token+TLS）。
- Provider 帧回传 RPC（`provider.proto` 只有 Health/Invoke/Cancel）。
- OAuth 2.1、限流。

### 0.3 产物

```text
build/sdk/iraf_sdk-0.4.0-py3-none-any.whl
  sha256 = 651df3ce9c6d4ddb30f5620cc283532aeb2c7577e8582f29d2caa219d6ec535b
  成员：iraf_sdk/{__init__,client,errors,mcp,provider}.py + dist-info

build/sdk/iraf-runtime-0.4.0-aarch64.tar.gz
  sha256 = 6e63bad33fee18f47f2ecd0700d86bcd340ae513b43f481996ebafcc0835eb12
```

重新构建（改了 `src/iraf_sdk/**` 后必须重建，否则交付的是旧代码）：

```bash
bash deploy/sdk/build_sdk.sh --target aarch64-manylinux_2_28-cp311
```

> 正式对外发版前应升 `pyproject.toml#project.version` 并重跑 `deploy/sdk/build_sdk.sh`，
> 让 wheel、manifest、sha256 一致（版本单一来源，禁止手改产物版本）。

---

## 1. 拓扑与端口

```text
上位机 x86 Ubuntu                         边缘侧（智控域 / 运控域）
┌─────────────────────────┐              ┌──────────────────────────────┐
│ 你们的 harness / agent   │              │ 智控域：IRAF TaskFlow/Skill/  │
│                         │  北向 MCP    │        Policy + MCP server   │
│  iraf_sdk.mcp           │ ───────────> │                              │
│  (Bearer token + TLS)   │  :8765/mcp   │  南向 gRPC → Capability      │
└─────────────────────────┘              │  Provider（运控域/仿真）      │
                                         └──────────────────────────────┘
```

- 默认端口（以装配单为准，代码不硬编码）：MCP `8765`、Runtime gRPC `50051`、
  Capability Provider `50052`（piper）/`50053`（ur5）。
- 身份只来自 transport：HTTP 是 `Authorization: Bearer <token>`；stdio 是部署环境
  `IRAF_RUNTIME_SUBJECT`。工具参数不接受 caller/role/tenant。

---

## 2. 边缘侧：安装、配置、签发 token、启动

### 2.1 安装（若由你们自己部署边缘 IRAF）

```bash
# 构建（开发端）
bash deploy/sdk/build_sdk.sh

# 安装到边缘角色（按声明；示例为智控域）
bash deploy/sdk/install-iraf.sh --profile edge-intel \
  --sdk-wheel build/sdk/iraf_sdk-0.3.22-py3-none-any.whl \
  --runtime-bundle build/sdk/iraf-runtime-0.3.22-aarch64.tar.gz \
  --secrets-file /path/to/runtime.secrets
```

装配单模板：`deploy/sdk/env/edge-intel.env.tmpl`。关键项：

```bash
IRAF_PROFILE=profiles/piper_mujoco.yaml
IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml
IRAF_SKILL_ROOT=/opt/iraf-sdk/current/skills
IRAF_BACKEND_ENTRYPOINT=iraf_adapters.remote.provider_backend:RemoteProviderBackend
IRAF_BACKEND_CONFIG='{"endpoint":"<provider>:50052","token_env":"IRAF_PROVIDER_TOKEN","timeout_s":180,"subject":"board-runtime"}'
IRAF_EVENT_STORE=/var/lib/iraf/iraf-runtime.db
IRAF_RUNTIME_SUBJECT=edge-runtime

# MCP 传输
IRAF_MCP_TRANSPORT=http
IRAF_MCP_HOST=0.0.0.0
IRAF_MCP_PORT=8765
IRAF_MCP_PATH=/mcp
IRAF_MCP_ALLOWED_HOSTS=<边缘IP>:8765,127.0.0.1:8765

# 鉴权（HTTP 必需）
IRAF_MCP_TOKEN_FILE=/etc/iraf/mcp-tokens.json
IRAF_MCP_AUDIT_LOG=/var/log/iraf/mcp-audit.jsonl

# TLS（对外必需；也可由反向代理终止）
IRAF_MCP_TLS_CERT=/etc/iraf/tls/server.crt
IRAF_MCP_TLS_KEY=/etc/iraf/tls/server.key
```

意图解析还需要（submit 用）：

```bash
IRAF_AGENTOS_MODEL_URL=https://<模型网关>/v1
IRAF_AGENTOS_MODEL=<模型名>
IRAF_AGENTOS_TOKEN=<key>        # 或 IRAF_AGENTOS_KEY_FILE=/etc/iraf/agentos.key（600）
```

> 端口、路径、场景、Provider 端点都来自声明/装配单。不要把 IP、token 写进仓库或脚本。

### 2.2 签发 token（明文只出现一次）

```bash
# 首次：创建注册表并签发
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json issue \
  --subject harness-a --tenant cust-a \
  --scopes task.submit,task.read \
  --resources '*' \
  --expires-days 90

# 查看 / 撤销（撤销按文件签名热加载，无需重启 MCP）
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json list
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json revoke --id tok-xxxxxxxx
```

约束：

- 注册表必须 `600`；权限宽了服务端拒绝加载。
- 只存 SHA-256；明文只输出一次。
- `--resources` 填允许的 `resource_id` 列表；`*` 表示该部署不做资源隔离。
- scope 只允许 `task.submit`、`task.read`；未知 scope 加载期即失败。

### 2.3 启动 MCP（HTTP）

```bash
# 前台调试（生产用 systemd/容器，不要让会话进程抢端口）
IRAF_MCP_TRANSPORT=http \
IRAF_MCP_HOST=0.0.0.0 \
IRAF_MCP_PORT=8765 \
IRAF_MCP_ALLOWED_HOSTS=<边缘IP>:8765 \
IRAF_MCP_TOKEN_FILE=/etc/iraf/mcp-tokens.json \
IRAF_MCP_TLS_CERT=/etc/iraf/tls/server.crt \
IRAF_MCP_TLS_KEY=/etc/iraf/tls/server.key \
python3 -m iraf_adapters.mcp.server
```

自检：

```bash
curl -s -o /dev/null -w 'healthz=%{http_code}\n' http://<边缘IP>:8765/healthz       # 200
curl -s -o /dev/null -w 'no_token=%{http_code}\n' -X POST http://<边缘IP>:8765/mcp  # 401
```

失败关闭规则：

- 没有 token 注册表且未显式 `IRAF_MCP_INSECURE=1` ⇒ 进程退出码 2。
- `IRAF_MCP_INSECURE=1` 只允许绑 `127.0.0.1/localhost/::1`；对公网/内网地址直接拒绝。
- 对外部署必须 TLS；`IRAF_MCP_TLS_REQUIRE_CLIENT=1` 可要求客户端证书，但
  当前 uvicorn 不向 ASGI 暴露 peer cert，**CN→subject 绑定未实现**，建议由网关终止 mTLS。

---

## 3. 上位机：安装与最小可运行示例

### 3.1 安装

```bash
python3 -m pip install build/sdk/iraf_sdk-0.3.22-py3-none-any.whl
python3 -m pip install 'mcp>=1.26,<2'      # MCP 客户端可选依赖，wheel 未声明 extras
```

要求：Python >= 3.9；MCP 客户端固定 1.x（2.x 移除了 AgentOS 依赖的 API）。

### 3.2 最小示例（streamable-http）

```python
import asyncio
from iraf_sdk.mcp import McpToolClient

async def main():
    async with McpToolClient(
        "https://<边缘IP>:8765/mcp",
        token="<签发的 token>",
        timeout_s=120,
    ) as client:
        print(await client.list_tools())          # 5 个任务级工具
        print(await client.enter_robot_mode())    # 只读盘点 skills/profile/simulation
        result = await client.submit_task("抓取 box_01")
        print(result["status"], result.get("execution_id"))
        if result.get("execution_id"):
            print(await client.get_task(result["execution_id"]))
        await client.exit_robot_mode()

asyncio.run(main())
```

> 演示前先确认场景干净。MuJoCo 场景有状态：一次成功抓取会搬走 `box_01`，
> 再抓必报 `distance≈0.374m tolerance=0.005m`；此时复位场景（重启 Provider 或
> `scripts/probe_scene_state.py --reset`），不要改代码。

### 3.3 stdio 形态（同机、无网络）

```python
from iraf_sdk.mcp import StdioMcpToolClient

async with StdioMcpToolClient(
    "/usr/bin/python3", ["-m", "iraf_adapters.mcp.server"],
    timeout_s=120,
    env={**os.environ, "IRAF_ENV_FILE": "/etc/iraf/edge-intel.env"},
) as client:
    ...
```

stdio 没有网络鉴权，身份由 `IRAF_RUNTIME_SUBJECT` 给出（通常在 env 文件里）。

### 3.4 非 MCP 的既有客户端

如果 harness 不需要 LLM/agent 语义，直接用任务级 gRPC/HTTP 更稳：

```python
from iraf_sdk import GrpcSkillClient, raise_for_status

client = GrpcSkillClient("<边缘IP>:50051", token, timeout_s=120)
result = client.submit(
    skill="move_joint",
    profile={"name": "piper", "version": "1.0.0", "digest": "<sha256>"},
    safety_policy={"name": "simulation_lab", "version": "1.0.0", "digest": "<sha256>"},
    parameters={"positions": {"joint1": 0.1}, "duration_ms": 500},
)
raise_for_status(result)
```

---

## 4. MCP 工具参考（任务级，5 个）

| 工具 | 参数 | 返回要点 |
|---|---|---|
| `iraf_enter_robot_mode` | 无 | `mode=robot_dev`、`skills`、`intents`、`skill_triggers`、`profile`、`safety_policy`、`provider_endpoint`、`simulation`、`routing_policy`、`examples`、`next_steps` |
| `iraf_exit_robot_mode` | 无 | `mode=idle`、`was_active` |
| `iraf_robot_mode` | `action=enter|exit|status`、`ttl_seconds=1800` | 同上；`status` 只读 |
| `iraf_submit_task` | `text`（自然语言）、`deadline_seconds=120` | `status`、`execution_id`、`error_code`、`reason`、`resolved_skill`、`robot_mode`、`next_steps` |
| `iraf_get_task` | `execution_id` | 事件库权威事实；**只返回本调用方自己的执行** |

资源：`iraf://examples/pick_box_01`、`iraf://examples/filtered_request`、`iraf://skills/catalog`。  
Prompt：`robot_routing_policy`、`robot_dev_mode`。

语义约定：

- 技能选择、参数校验、策略判定都在 IRAF 内部；调用方只给任务语义，不指定 skill。
- 非机器人请求返回 `IRAF-NOT-ROBOT-TASK`，平台应回退到自己的处理，不要重试。
- 进入模式后先只读盘点，不要"冒烟提交"（会弄脏仿真场景）。
- 同一资源同一时刻只跑一条任务；并发会让目标位移/租约冲突。
- 失败不要包装成成功；用 `iraf_get_task` 核对事实。

---

## 5. 鉴权与错误码

| HTTP / 返回 | 含义 | 处置 |
|---|---|---|
| 401 `IRAF-UNAUTHENTICATED` | 缺/错/过期/已撤销 token | 重新签发；不要重试同一 token |
| 403 `IRAF-POLICY-DENIED` | token 缺 scope 或资源不匹配 | 换有权限的 token；不要放宽参数 |
| `IRAF-AUTH-NOT-CONFIGURED` | 服务端缺 token 注册表 | 部署问题，联系运维；不要开 insecure |
| `IRAF-NOT-ROBOT-TASK` | 语义与机器人无关 | 平台自己处理，不重试 |
| `IRAF-INPUT-INVALID` | 参数/文本不合法 | 按 reason 修正 |
| `distance=... tolerance=...` | 场景状态（目标已被搬走） | 复位场景后再试一次 |

审计：`IRAF_MCP_AUDIT_LOG` 每行 JSON，含 `auth_ok/auth_failed/tool_call`、
`subject/tenant/token_id/tool/execution_id/status`；不含 token 明文或哈希。

---

## 6. 与旧 AgentOS 的兼容（未删除任何旧路径）

保留不变：

- 5 个任务级工具名与行为不变；`scripts/iraf_mcp_server.py` 兼容转发入口保留。
- stdio 形态仍是 AgentOS 的默认接入方式；旧的 `mcp add` 登记**不需要改**。
- `iraf_robot_mode` / enter / exit / resources / prompts 全部保留。
- 旧文档 `docs/mcp-binding.md`、探针脚本、事件库格式未删除。

唯一新增要求：

- stdio 进程必须有 `IRAF_RUNTIME_SUBJECT`。旧装配单 `deploy/sdk/env/edge-intel.env.tmpl`
  一直有 `IRAF_RUNTIME_SUBJECT=@RUNTIME_SUBJECT@`，所以板上现有登记照常工作。
- 若某台机器的 env 文件缺这一项，在 `mcp add` 里补一行，或写进 env 文件：

```bash
intewell-agent-cli mcp add iraf-task --command /usr/bin/python3 \
  --env IRAF_ENV_FILE=/etc/iraf/edge-intel.env \
        IRAF_RUNTIME_SUBJECT=edge-runtime \
        IRAF_PROVIDER_TOKEN=... \
        PYTHONPATH=/opt/iraf-sdk/current/lib:/opt/iraf-sdk/current/src \
        IRAF_AGENTOS_MODEL_URL=... IRAF_AGENTOS_MODEL=... \
  --args -m iraf_adapters.mcp.server
```

行为差异（安全修正，需知会）：

- `iraf_get_task` 现在按执行归属过滤。AgentOS 与 CLI 若使用**同一个**
  `IRAF_RUNTIME_SUBJECT`（板上默认如此），回查行为不变；若历史执行用了别的 subject，
  会返回 `NOT_FOUND`，属于有意的跨调用方隔离。
- HTTP 形态新增 token 鉴权；stdio 形态不需要 token 注册表。
- `iraf_robot_mode` 状态按 subject 隔离；stdio 单进程单 subject，行为不变。

兼容自检（本机实测可复跑）：

```bash
# stdio：只给 IRAF_RUNTIME_SUBJECT，不配 token 注册表
IRAF_RUNTIME_SUBJECT=edge-runtime PYTHONPATH=src:build/generated/python \
  python3 -m iraf_adapters.mcp.server   # 由 MCP 客户端 list_tools 验证 5 个工具
```

---

## 7. 与南向 Provider 的关系

本 SDK 已包含 `iraf_sdk.provider.ProviderKit`（0.4.0）：第三方可以用它把**自己的 MuJoCo /
控制器**实现成 `CapabilityProviderService`，由边缘 IRAF 调用。详见
`docs/provider-sdk-guide.md`。

边界不变：上位机 harness 仍只走上层任务接口，不直接调 Provider Invoke（禁止旁路）；
Provider 只执行能力，不做策略判定。

---

## 8. 验收清单（交付前逐项打勾）

1. `GET /healthz` → 200；无 token `POST /mcp` → 401。
2. 有效 token 连接后 `list_tools` == 5 个任务级工具。
3. `enter_robot_mode` → `mode=robot_dev`、`skills=15`、`simulation=true`。
4. `submit_task("今天天气怎么样")` → `IRAF-NOT-ROBOT-TASK`，无 execution_id、无动作。
5. `submit_task("抓取 box_01")`（场景干净）→ `SUCCEEDED` + `execution_id`；
   `get_task` 回查同一 id。
6. 第二个 subject 的 token 读第一个的 `execution_id` → `NOT_FOUND`。
7. 只有 `task.read` 的 token 提交 → `IRAF-POLICY-DENIED`，无 execution_id。
8. `revoke` 后旧 token 立即 401（无需重启）。
9. 旧 AgentOS stdio 登记：`list_tools` 仍为 5 个工具，`submit` 正常。
10. 所有结论标注 `simulation=true`，不表述为真机能力。

本机已实测（2026-11-03）：

- HTTP：401 / 200 / 5 tools / robot_dev / 15 skills / 撤销后 401。
- 过滤：`IRAF-NOT-ROBOT-TASK` + `execution_id=287191a0-5cbc-5d69-a2c6-12526215887c`。
- 跨调用方：B 读 A 的 id → `NOT_FOUND`。
- 越权：只有 `task.read` 的 token 提交 → `IRAF-POLICY-DENIED`。
- 单测：57 项通过（auth/store/server layer/sdk adapter 等）。

---

## 9. 故障排查

| 现象 | 原因 | 处置 |
|---|---|---|
| 启动即退出 2，提示缺注册表 | HTTP 形态未配 token | 签发 token 并设 `IRAF_MCP_TOKEN_FILE` |
| 401 一直不过 | token 错/过期/撤销；或注册表权限不是 600 | `list` 检查；重新签发；`chmod 600` |
| 421 Misdirected Request | `IRAF_MCP_ALLOWED_HOSTS` 没含实际 Host | 把 `IP:port` 加进去 |
| 403 `IRAF-POLICY-DENIED` | scope 不足 / 资源不匹配 | 换 token 或补 resources；不要放宽参数 |
| submit 报"意图 Provider 未配置" | 缺 `IRAF_AGENTOS_MODEL_URL/MODEL/TOKEN` | 补齐模型网关配置 |
| `distance≈0.374m` | 场景已被上一次执行搬走 | 复位场景，不要改代码 |
| `ModuleNotFoundError: mcp` | 上位机未装 MCP 客户端 | `pip install 'mcp>=1.26,<2'` |
| 工具集为空 | 宿主单轮/ACP 结构不发现 MCP | 用多轮会话或用 SDK 直接连 |

---

## 10. 版本与兼容

| 项 | 值 |
|---|---|
| SDK wheel | `iraf_sdk-0.3.22-py3-none-any.whl`（py3-none-any） |
| Python | 客户端 >= 3.9；本机 3.10 实测；边缘板 3.11 |
| MCP 客户端 | `mcp>=1.26,<2`（1.26.x 实测） |
| IDL | `iraf.v1` / `idlv1 1.0.0`，只增不改 |
| 边缘 runtime | 0.3.22（与 SDK 同版本交付） |
| 鉴权 | Bearer token（不透明）+ 可选 TLS；mTLS CN 绑定未实现 |

升级规则：SDK 版本、IDL 兼容范围、边缘 runtime 版本三者一起声明；不兼容变更必须
先改 `api/` 并升 IDL 版本，不允许静默改字段语义。
