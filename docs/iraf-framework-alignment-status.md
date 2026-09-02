# IRAF 框架对齐与实施状态

**日期**：2026-09-02  
**状态**：开发仿真基线，不代表真机或生产 AgentOS 放行

## 已按铁律收敛

- canonical IDL 位于 `api/proto/iraf/v1/`，AgentOS 私有映射位于 `adapters/agentos/proto/v1/`。
- `iraf_core.runtime.SkillRuntime` 是执行唯一入口，负责 Policy、Registry、租约、幂等和事件记录。
- Skill 从 `skill.yaml`、JSON Schema 和 Provider entrypoint 加载，新增 Skill 不修改 Runtime。
- AgentOS Dispatcher 仅做协议适配，HTTP 和 gRPC 都是开发仿真 transport，不承载领域逻辑。
- Model Provider 只能输出 manifest 声明的 Skill，输出在 Provider 与 Runtime 中分别校验。
- 未验证的 RobotProfile、SafetyPolicy 和 Skill manifest 仅允许 `simulation=true`。
- 旧 `run_move_joint.py` 直控入口已停用；通用 CLI 只提交 canonical Skill 请求。
- 执行结果和状态事件写入 SQLite，重复幂等键不会重复下发动作。
- gRPC 只注册 Execute/GetExecution；Cancel 未实现，因此不会出现在服务端方法表中。

## 当前可验证能力

- Piper MuJoCo `move_joint` 与 `stop`。
- E300 OpenAI-compatible 模型提供文字意图解析。
- canonical Protobuf JSON 与 canonical gRPC Execute/GetExecution 到 SkillRuntime 的任务执行。
- manifest/schema、策略拒绝、资源冲突、幂等冲突、未验证真机拒绝、gRPC 鉴权和 Cancel 未注册测试。
- gRPC 开发单元仅允许回环地址、仿真 Profile 和显式 `IRAF_GRPC_DEVELOPMENT=true`。

## 尚未完成

- AgentOS 正式 TaskFlow、Skill Query、Tool/Skill Manager 私有接口契约测试。
- gRPC 生产 server、mTLS、取消语义和多阶段反馈流。
- 签名 Profile/Policy/manifest 的正式信任链。
- SafetyEvent、QUARANTINED、控制器 safe-state 确认和授权复位。
- HIL/真机证据；当前所有动作结论仅限仿真。

## 目录边界

```text
api/proto/             canonical Protobuf
src/iraf_core/         平台无关 Runtime/Policy/Registry/Store
skills/*/skill.yaml    声明式 Skill 与 schema
adapters/agentos/      AgentOS 映射
adapters/model/        模型 Provider
adapters/mujoco/       仿真 Capability Backend
adapters/http/         开发仿真 HTTP transport
adapters/grpc/         开发仿真 canonical gRPC transport
profiles/              Robot/Safety 配置
tests/                 契约与拒绝路径验证
```
