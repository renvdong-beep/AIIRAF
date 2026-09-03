# Ubuntu-only IRAF 编码任务计划

本计划不依赖 E300、AgentOS、在线模型或真机，仅在 Ubuntu 仿真环境执行。

## 任务顺序

1. TaskFlow 状态迁移、取消、超时和幂等回归测试。
2. ControlAuthorityManager 租约 TTL、fencing token 和冲突测试。
3. RobotProfile 与 SafetyPolicy 能力边界测试。
4. ProviderRouter 统一输入输出和拒绝路径测试。
5. 回放 manifest、脱敏和故障证据测试。
6. MuJoCo 本地稳定性与故障注入验收。

每项任务必须通过 Coding Worker 的白名单验证，独立分支提交并保留证据；未通过的任务不得释放后续依赖任务。
