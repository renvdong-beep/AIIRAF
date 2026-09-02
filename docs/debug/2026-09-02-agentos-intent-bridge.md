# AgentOS 文字意图桥接验收记录

日期：2026-09-02

## 实现

- 公共 IDL 新增 `IntentRequest` 与 `InterpretIntent`。
- 新增可配置的 OpenAI-compatible Intent Provider，端点、模型、令牌和 TLS 策略只来自部署环境。
- Qwen3 推理显式关闭 thinking，输出必须是严格 JSON，且 Skill 必须来自 Registry。
- AgentOSBridge 将意图转换为标准 TaskRequest，随后仍经过 Policy Gateway、RobotProfile、ControlAuthority 和 Capability Backend。
- Runtime 新增 `POST /v1/intents`，支持 Bearer 身份、截止时间、幂等键和执行审计。

## 验证

- 单元测试 9 项通过，包括 Provider 失败不调用 Backend。
- Protobuf 描述符生成成功，Python 模块编译成功。
- E300 模型服务实际将“停止机器人”解析为 `stop`，MuJoCo Backend 执行成功。
- 返回记录包含 execution、correlation、Skill、Profile、Policy、模型、资源和控制器版本信息。

## 限制

当前验证的是 E300 模型服务与 Ubuntu IRAF Runtime 的桥接，尚不是 AgentOS 私有任务服务的正式集成；生产远程接口仍需 mTLS 和持久化任务存储。