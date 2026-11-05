# IRAF 使用说明

> 适用版本：v0.4.1。本文覆盖上位机 SDK、任务级 MCP（5 个工具）、AgentOS 接入、AI Studio 接入。
> 事实来源：`docs/upper-sdk-guide.md`、`docs/agentos-mcp-integration.md`、`docs/mcp-binding.md`。
> 全程 `simulation=true`。

---

## 1. 使用方式总览

| 方式 | 入口 | 适用 |
|---|---|---|
| 上位机 Python 客户端 | `iraf_sdk.mcp.McpToolClient` | 自有程序/脚本接入 |
| 任务级 MCP（streamable-http） | `http://<边缘IP>:8765/mcp`（Bearer token） | 上位机 harness、AgentOS、AI Studio |
| stdio MCP | `IRAF_RUNTIME_SUBJECT` + `IRAF_MCP_SCOPES`（进程边界即身份） | 同机宿主 |

统一语义：**调用方只给任务语义，不指定 skill**。技能选择、参数校验、策略判定都在 IRAF 内部。

## 2. 上位机 SDK 安装与最小示例

```bash
# 只装客户端（纯标准库）
pip install iraf-sdk

# 需要 MCP 客户端 / 南向 Provider SDK 时用 extras
pip install "iraf-sdk[mcp,provider]"
```

```python
import asyncio
from iraf_sdk.mcp import McpToolClient

async def main():
    c = McpToolClient("http://<边缘IP>:8765/mcp", token="<token>")
    print(await c.list_tools())          # 5 个任务级工具
    await c.enter_robot_mode()           # 进入机器人模式（先只读盘点）
    r = await c.submit_task("把 box_01 抓起来")   # 自然语言任务
    print(r["execution_id"], r["status"])
    print(await c.get_task(r["execution_id"]))    # 回查权威事实

asyncio.run(main())
```

## 3. 任务级 MCP 工具参考（5 个）

| 工具 | 参数 | 返回要点 |
|---|---|---|
| `iraf_enter_robot_mode` | 无 | `mode=robot_dev`、`skills`、`intents`、`skill_triggers`、`profile`、`safety_policy`、`provider_endpoint`、`simulation`、`routing_policy`、`examples`、`next_steps` |
| `iraf_exit_robot_mode` | 无 | `mode=idle`、`was_active` |
| `iraf_robot_mode` | `action=enter\|exit\|status`、`ttl_seconds=1800` | 同上；`status` 只读 |
| `iraf_submit_task` | `text`（自然语言）、`deadline_seconds=120` | `status`、`execution_id`、`error_code`、`reason`、`resolved_skill`、`robot_mode`、`next_steps` |
| `iraf_get_task` | `execution_id` | 事件库权威事实；**只返回本调用方自己的执行** |

资源：`iraf://examples/pick_box_01`、`iraf://examples/filtered_request`、`iraf://skills/catalog`。
Prompt：`robot_routing_policy`、`robot_dev_mode`。

**语义约定**：
- 非机器人请求返回 `IRAF-NOT-ROBOT-TASK`，平台应回退到自己的处理，不要重试。
- 进入模式后先只读盘点，不要"冒烟提交"（会弄脏仿真场景）。
- 同一资源同一时刻只跑一条任务；并发会让目标位移/租约冲突。
- 失败不要包装成成功；用 `iraf_get_task` 核对事实。

## 4. 在 AgentOS 中使用 IRAF

### 4.1 绑定（edge-intel 安装后自动执行，0.4.1+）

AgentOS 有两套 home：
- 宿主 chat（tui_gateway）：`/root/.intewell/config.yaml`
- worker/task-api：`/var/lib/agentos-shared/.intewell/config.yaml`（容器 bind mount）

```bash
bash /opt/iraf-sdk/current/deploy/sdk/board_agentos_mcp_bind.sh \
  --apply --verify --url http://<板卡IP>:8765/mcp
```

脚本做五件事：① 从 docker inspect 发现 worker 与共享 home；② 签发/复用 `agentos-host` 与 `agentos-worker` 两个独立 token；③ 宿主优先走 dashboard API `POST /api/mcp/servers`（需 `/etc/iraf/agentos-admin` 600，无凭据则降级 YAML 合并并告警）；④ worker 合并共享 `config.yaml`（备份 + 原子写 + 幂等，保留其它 server）；⑤ `--verify` 检查服务/宿主/worker/工具发现，结果写 `/etc/iraf/agentos-mcp-binding.json`。

运维：`--status` / `--verify` / `--revert`。注意 worker 镜像必须自带 `mcp>=1.26,<2`（厂商镜像职责，安装器只检查告警，不私装）。

**首次配置（host 侧 dashboard 凭据）**：宿主走 dashboard API 需要先建凭据文件，否则脚本降级为 YAML 合并并告警「需重启 dashboard」（此时不能声称 chat 已接入）：

```bash
# 板上 root；值不要进 git / 聊天记录
install -m 600 /dev/null /etc/iraf/agentos-admin
cat > /etc/iraf/agentos-admin <<'EOF'
username=admin
password=<dashboard 管理员口令>
EOF
chmod 600 /etc/iraf/agentos-admin
```

首次完整流程：① 建 `/etc/iraf/agentos-admin`；② `board_agentos_mcp_bind.sh --apply --verify --url http://<板卡IP>:8765/mcp`；③ 在 AgentOS chat / worker 里验证能发现 `iraf-task` 的 5 个工具（`intewell-agent-cli mcp test iraf-task`）。

### 4.2 AgentOS 侧使用

绑定后，AgentOS chat / worker 即可在会话中调用 `iraf_enter_robot_mode` / `iraf_submit_task` / `iraf_get_task` 等工具，用自然语言下发任务（如"把 box_01 抓起来"），IRAF 内部完成技能选择 + 策略判定 + 仿真执行，`iraf_get_task` 回查执行事实。

## 5. 在 AI Studio 中使用 IRAF

AI Studio（团队自研 AI 工具平台）接入 IRAF 与上位机/AgentOS 同一套 MCP 通道：

1. 在 AI Studio 的「模型/工具」配置里，把 IRAF 任务级 MCP 注册为一个 MCP server：
   - endpoint：`http://<边缘IP>:8765/mcp`
   - `Authorization: Bearer <AI Studio 专用 token>`（与 AgentOS/上位机 token 分开签发，四套凭据互不复用）

   AI Studio 专用 token 需先在边缘侧签发（明文只出现一次，注册表只存 SHA-256）：

   ```bash
   python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json issue \
     --subject ai-studio --scope task.submit,task.read
   ```

2. 在 Prompt / 工作流里调用 `iraf_enter_robot_mode` → `iraf_submit_task` → `iraf_get_task`；
3. 用于"任务级机器人能力"编排：AI Studio 负责 LLM/数据集/评测/工作流，IRAF 负责机器人技能的安全执行，边界清晰。

## 6. 鉴权与错误码

| HTTP / 返回 | 含义 | 处置 |
|---|---|---|
| 401 `IRAF-UNAUTHENTICATED` | 缺/错/过期/已撤销 token | 重新签发；不要重试同一 token |
| 403 `IRAF-POLICY-DENIED` | token 缺 scope 或资源不匹配 | 换有权限的 token；不要放宽参数 |
| `IRAF-AUTH-NOT-CONFIGURED` | 服务端缺 token 注册表 | 部署问题；不要开 insecure |
| `IRAF-NOT-ROBOT-TASK` | 语义与机器人无关 | 平台自己处理，不重试 |
| `IRAF-INPUT-INVALID` | 参数/文本不合法 | 按 reason 修正 |
| `distance=... tolerance=...` | 场景状态（目标已被搬走） | 复位场景后再试一次 |

## 7. token 管理（`iraf_tools.mcp_tokens`）

```bash
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json issue --subject upper-sdk --scope task.submit,task.read
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json list
python3 -m iraf_tools.mcp_tokens --file /etc/iraf/mcp-tokens.json revoke --id tok-xxxxxxxx
```

token 只存 SHA-256、文件 600、按文件签名热加载（撤销无需重启）；明文只在签发时出现一次。

## 8. 审计

`IRAF_MCP_AUDIT_LOG` 每行 JSON：`auth_ok/auth_failed/tool_call`、`subject/tenant/token_id/tool/execution_id/status`；不含 token 明文或哈希。
