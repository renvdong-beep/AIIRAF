# 最小链路验证记录

**日期**：2026-09-02  
**环境**：Ubuntu 10.203.247.145，Piper MuJoCo 仿真，E300 AgentOS Qwen3-0.6B

## 场景

提交中文意图：`将 joint1 移动到 0.2`

链路：

```text
AgentOS/E300 模型
  -> HTTP Intent Adapter (/v1/intents, loopback)
  -> OpenAICompatibleIntentProvider
  -> AgentOSBridge
  -> SkillRuntime
  -> PolicyGateway + RobotProfile/SafetyPolicy 校验
  -> ControlAuthorityManager 租约
  -> SkillRegistry: move_joint@1.0.0
  -> common_motion_sim Provider
  -> MuJoCo Backend
```

## 结果

- `status=SUCCEEDED`
- `skill=move_joint@1.0.0`
- `provider=common_motion_sim/python_adapter`
- 返回 `{"skill":"move_joint","accepted":true}`
- 结果包含 execution_id、Policy decision、Profile/SafetyPolicy 摘要和事件序列。
- 未使用独立 Piper 业务脚本；动作由 manifest 声明和通用 Provider 入口完成。

## 边界

本记录只证明开发仿真链路，不证明真机、HIL、mTLS、取消、故障隔离或生产 AgentOS 私有协议已完成。
