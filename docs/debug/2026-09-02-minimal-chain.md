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

## 可重复验收

```bash
./scripts/verify_minimal_chain.py --mode direct
./scripts/verify_minimal_chain.py --mode both
```

`--mode direct` 验证公共 `ExecuteSkillRequest` 下游链路；`--mode both` 额外调用 AgentOS 意图 Provider，并将模型不可达单独标为意图阶段失败。每次运行把脱敏结果写入 `build/acceptance/minimal-chain.json`。

## 当前运行结果

2026-09-02 远端 `10.203.247.145` 上，direct 和 AgentOS intent 模式均返回 `SUCCEEDED`，Provider 为 `common_motion_sim/python_adapter`，意图 Provider 为 `Qwen3-0.6B`。此前 Qwen3 endpoint `10.203.247.86:9119` 因 vLLM/MCCL watchdog 崩溃暂时不可达，已在边缘板卡重启 `qwen-vllm.service` 后恢复；恢复后的链路已完成真实中文意图验收。

## 边界

本记录只证明开发仿真链路，不证明真机、HIL、mTLS、取消、故障隔离或生产 AgentOS 私有协议已完成。
