# MuJoCo 仿真基线验收记录

**日期**：2026-09-03  
**边界**：仅限 `simulation=true` 的 development simulation，不代表真机、实时控制或 HIL 通过

## 目标

将已有的连续 MuJoCo 物理循环补齐为可观测、可注入故障、失败后安全收敛且可重复验收的仿真基线。

## 实现

- `MujocoBackend.simulation_status()` 提供目标/有效步频、步进次数、耗时、超限、失败、最近步进时间和最近错误。
- HTTP `/health` 携带仿真指标；步进线程失败时返回 `503` 和 `degraded`，不再继续报告健康。
- 认证后的 `GET /v1/simulation/metrics` 返回完整指标快照。
- 步进异常后立即将 MuJoCo 控制量归零并锁存错误；未显式停止和清除故障前拒绝重启。
- 故障注入默认关闭。验收脚本只在独立本地仿真实例中临时启用 `step_delay` 和 `step_failure`，不通过 HTTP/gRPC 暴露故障控制入口。

## 可重复验收

```bash
python -m unittest discover -s tests/unit -v
./scripts/verify_simulation_baseline.py
./scripts/verify_minimal_chain.py --mode direct
```

仿真基线报告写入 `build/acceptance/simulation-baseline.json`，必须同时满足：

1. 连续步进启动且有效步频达到配置门槛。
2. 延迟注入被记录为步进超限，线程保持健康。
3. 步进失败后线程停止、控制量归零、健康状态降级。
4. 未清除故障时重启被拒绝，显式清除后可恢复。

## 2026-09-03 运行结果

- 单元测试共 44 项，全部通过。
- 真实 Piper MuJoCo 验收通过：目标步频 500 Hz，稳态采样有效步频约 464 Hz，频率比约 0.929。
- 注入 20 ms 步进延迟后 `step_overrun_count` 从 0 增至 1，连续线程保持健康。
- 注入步进失败后 `step_failure_count=1`、`running=false`、控制量归零；未清除时重启被拒绝，清除后恢复健康。
- `iraf-runtime.service` 重启加载新实现后为 `active`、`NRestarts=0`，`/health` 与认证指标接口均返回健康状态。
- 重启后的最小 direct task 返回 `SUCCEEDED`，Skill 为 `move_joint@1.0.0`，Provider 为 `common_motion_sim/python_adapter`。

## 尚未覆盖

- 指标当前是进程内快照，尚未接入 Prometheus 或统一 Observability 服务。
- 本验收不证明 ROS 2/Linux-RT deadline、RTOS motion permit、现场总线看门狗或物理急停。
- 真机和 HIL 必须使用独立验收记录，不得复用本报告作为硬件证据。

## 一键证据包

`scripts/verify_development_simulation.py` 将单元测试、仿真故障验收、Runtime direct task 和在线 Qwen intent task 汇总到 `build/acceptance/development-simulation/`。`manifest.json` 记录：

- 每条验收命令的退出码、耗时、日志路径和日志 SHA-256。
- Git commit、分支、dirty 状态和变更路径数量。
- Python、MuJoCo、Protobuf、PyYAML、JSON Schema 和 gRPC 版本。
- RobotProfile、SafetyPolicy 和 Skill contract 文件摘要。
- Runtime 健康快照、单项报告路径、大小与摘要。

`manifest.sha256` 用于校验 manifest 本身。证据包明确标记 `simulation_only=true`；工作区为 dirty 时只允许作为开发阶段证据。

2026-09-03 已实际运行一键入口，环境、单元测试、仿真基线、意图 Provider 故障回放、direct task、在线 Qwen intent task、两条成功执行回放和 Runtime 健康九项门禁全部通过。意图故障门禁验证模型身份和 `PENDING -> VALIDATING -> FAILED` 事件链，同时确认未触发动作后端、原始自然语言未进入回放证据；在线成功门禁验证 Qwen3-0.6B、resolved Skill 和四态事件链进入同一 replay manifest。
