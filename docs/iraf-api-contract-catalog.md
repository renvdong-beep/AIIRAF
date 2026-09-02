# IRAF 公共 API 契约目录

**状态**：M1 编码输入  
**事实来源**：后续以 `api/proto/iraf/v1/` 中通过 `buf build` 的文件为准

## 通用规则

- 每个写请求包含 `request_id`、`idempotency_key`、`correlation_id` 和 deadline；execution/task ID 由服务端生成。
- 身份、租户和角色只来自 transport `AuthenticatedContext`，不出现在可由业务客户端自行填写的授权字段中。
- TaskEngine 调用 SkillRuntime 时携带短期签名 TaskDelegation，固定原调用者、task_id、允许 Skill/Profile、资源范围和 expiry，系统身份不得扩大权限。
- 所有流式事件包含对象 ID、严格递增 sequence、occurred_at 和 resume token；客户端断线重连时从已确认 token 继续。
- 错误统一包含稳定 code、中文诊断、retryable 和脱敏 details；gRPC status 表达传输/服务级结果，IrafError 表达领域结果。
- 所有 VersionRef 包含 name、SemVer 和内容 digest；运行开始后固定解析结果。

## TaskService

| RPC | 请求关键字段 | 响应/流 | 幂等与权限 |
|---|---|---|---|
| `Submit` | TaskFlow VersionRef、inputs、deadline、Profile refs | task_id、initial state | `task.submit`，按 idempotency key |
| `Cancel` | task_id、reason | accepted、state | `task.cancel`，重复取消幂等 |
| `Resume` | task_id、reason、expected_sequence | accepted、new sequence | `task.resume`；SAFE_HOLD/HANDOVER_REQUIRED 专用 |
| `Get` | task_id | TaskSnapshot | `task.read` |
| `Watch` | task_id、after_sequence/resume_token | TaskEvent stream | `task.read`，支持续传与心跳 |

## SkillRuntimeService

| RPC | 请求关键字段 | 响应/流 | 权限 |
|---|---|---|---|
| `Execute` | SkillGoal、idempotency key | SkillFeedback stream | TaskEngine 内部；直接调用需 `skill.execute.direct` |
| `Cancel` | execution_id、reason | accepted、state | execution owner 或 `skill.cancel.any` |
| `GetExecution` | execution_id | ExecutionSnapshot | `skill.read` |

## WorldModelService

| RPC | 请求关键字段 | 响应/流 | 约束 |
|---|---|---|---|
| `GetSnapshot` | entity selector、at_sequence | typed entities/relations、snapshot sequence | 默认只返回未过 TTL 数据 |
| `Query` | typed filter、limit、page token | entities/relations、next token | 不接受任意 SQL/脚本表达式 |
| `WatchChanges` | selector、after_sequence/resume token | StateChanged stream | 慢消费者断开并返回最后有效 token |

WorldModel 写入不是公开业务 API，只允许签名 adapter、Runtime 和状态融合组件使用受限内部 `IngestObservation`，并记录 source、sequence、frame、timestamp、confidence、TTL 与 schema version。

## EventService

| RPC | 请求关键字段 | 响应 | 约束 |
|---|---|---|---|
| `ListEvents` | correlation/object selector、time range、page token | 脱敏事件页 | 权限和租户过滤先于查询 |
| `GetReplayManifest` | task_id 或 correlation_id | digest、版本、事件/对象 URI | 不内联原始图像/密钥 |

## AgentOSBridge 管理契约

该契约属于 `adapters/agentos/` 的北向兼容面，不计入四个机器人业务服务，也不引入 AgentOS 私有业务对象。

| RPC | 请求关键字段 | 响应 | 约束 |
|---|---|---|---|
| `GetCompatibility` | AgentOS protocol/IDL/build 摘要、feature bits | IRAF 支持范围、协商 feature、限制和 digest | major 不兼容时 fail closed；结果写审计事件 |
| `Health` / `Ready` | 无或调用方摘要 | 依赖健康、接收任务状态、当前模式 | 不泄露凭据和设备细节 |
| `Drain` | deadline、reason、expected generation | accepted、运行/待处理数量 | 拒绝新任务；不绕过 TaskFlow/资源/安全状态机 |
| `Shutdown` | drain token、reason | accepted、终态摘要 | 只在 drain 完成或授权安全停止后执行 |

兼容测试至少覆盖当前和上一 AgentOS 主版本。`GetCompatibility` 只协商管理与公共服务 feature；不得协商关闭认证、Policy Gateway、SafetyEvent 或终态不可复活规则。

## 认证与流控

入口使用 mTLS 工作负载身份或经验证 JWT；AgentOS 委托必须包含 audience、tenant、subject、scope、expiry 和唯一 credential ID。服务端 interceptor 验证后构造只读 AuthenticatedContext。Watch 流必须设置最大缓冲、心跳和空闲超时；超过缓冲的消费者收到 `IRAF-STREAM-LAGGED` 与可恢复 token，不能拖垮 Runtime。
