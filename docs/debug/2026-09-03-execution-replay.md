# Execution Replay Manifest 实现与验收记录

**日期**：2026-09-03  
**边界**：development simulation 的审计与最小回放索引，不包含传感器大对象、控制帧或真机证据

## 实现

- `api/proto/iraf/v1/events.proto` 新增 `ReplayManifest` 与 `EventService.GetReplayManifest`。
- `SqliteExecutionStore.get_replay_manifest()` 从持久化终态和顺序事件构建确定性 manifest。
- Runtime 为成功与失败执行统一冻结 requested Skill、RobotProfile、SafetyPolicy、资源、控制器和 `simulation` 边界。
- manifest 只包含原始请求、结果、subject 和 idempotency key 的 SHA-256，不返回这些原值。
- `scripts/export_replay_manifest.py` 可从开发 Runtime 的 EventStore 导出 JSON 和 SHA-256 校验文件。
- 一键仿真验收会分别导出本次 direct task 和在线 AgentOS intent task 的 replay manifest，并将两者纳入总证据包门禁。

### AgentOS intent 失败回放

- AgentOSBridge 在解析前生成 intent 请求摘要，不保存自然语言原文。
- Provider 失联或结构化输出失败时，由 SkillRuntime 记录 `PENDING -> VALIDATING -> FAILED`，不会进入 Skill Provider 或 Backend。
- 成功解析后，intent provider 名称、版本、模型、请求摘要和 resolved Skill 以只追加方式写回同一个 execution；不允许覆盖终态、事件或安全边界字段。
- `scripts/verify_intent_failure_replay.py` 使用显式不可用 Provider 验证无动作失败路径，并输出 `iraf.intent-failure-replay/v1` 报告。
- 在线成功门禁调用部署配置中的 Qwen Provider，要求回放包含 `agentos-intent`、模型身份、请求摘要、resolved Skill 及完整四态事件链。

## 验收场景

1. 成功执行包含 `PENDING -> VALIDATING -> RUNNING -> SUCCEEDED` 单调事件序列及 Skill/Provider/Profile/Policy 身份。
2. 参数拒绝执行仍保留请求 Skill、仿真边界、Profile/Policy 和失败终态。
3. manifest 不出现原始 subject、idempotency key 或 Skill 输出。
4. 不存在的 execution 返回 `NOT_FOUND`，未认证 gRPC 请求返回 `UNAUTHENTICATED`。
5. replay JSON 和证据包 manifest 均可用各自 `.sha256` 文件从仓库根目录校验。
6. intent Provider 失联形成可回放失败，且 manifest 中不出现自然语言原文。
7. 在线 Qwen intent 成功执行与回放属于同一个 execution，模型、Skill 和事件链身份完整。

## 未覆盖

- 当前不保存原始输入与输出，只保存摘要；确定性重执行需要后续受访问控制的对象存储。
- 当前 EventStore 是单机 SQLite，生产 EventService 的 mTLS、tenant 隔离、retention 和远端续传尚未完成。
- 仿真 replay 不能替代 ROS 2/Linux-RT、RTOS 或 HIL 的 telemetry 与安全证据。
