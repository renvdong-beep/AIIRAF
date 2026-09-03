# IRAF 文档导航

IRAF（Intewell Robot Application Framework）将 AgentOS 任务请求转换为可审计、可恢复且受安全策略约束的机器人 Skill。它不替代实时控制器、现场总线主站、BSP、驱动或 Hypervisor。

## 代码分层

```text
api/proto/             canonical Protobuf 与版本化公共契约
src/iraf_core/         TaskFlow、SkillRuntime、Policy、Registry、Lease、Store
src/iraf_adapters/     AgentOS、HTTP/gRPC、模型、MuJoCo、ROS 2 适配器
src/iraf_skills/       可复用 Skill Provider
src/iraf_tools/        可安装 CLI 入口
skills/*/              Skill manifest 与 JSON Schema
profiles/              RobotProfile 与 SafetyPolicy
deploy/                环境模板和 systemd 服务模板
scripts/                可重复的 setup、health 和最小链路验收脚本
tests/                 单元、契约、集成和后续 HIL/fault 验证
```

旧的 `adapters/`、`skills/common/` 和 `tools/` 仅作为迁移期兼容入口，新的代码不得继续写入这些目录。

## 当前状态

当前代码为 development simulation 基线。HTTP/gRPC Runtime 启动时由 `MujocoSimulationSupervisor` 持有唯一连续物理步进循环，`move_joint` 只提交目标并由同一 Backend 执行，服务停止时统一收敛。后端提供目标/有效步频、步进耗时、超限、失败和最近错误指标；步进异常后控制量归零并锁存故障，必须显式清除后才能重启。真实 ROS 2、Hyper IPC、RTOS/现场总线、签名 Profile、mTLS、SafetyEvent 的 RTOS 回读和 HIL 证据尚未完成；开发版 EventService.ListEvents 已可查询本地审计事件。

一键开发仿真验收：`scripts/verify_development_simulation.py`，它顺序执行完整单元测试、真实 MuJoCo 故障验收、Runtime direct task 和执行回放导出，并把命令日志、依赖版本、源码状态、契约摘要及报告摘要写入 `build/acceptance/development-simulation/manifest.json`。源码工作区为 dirty 时，该结果只能作为开发证据，不能作为冻结发布制品。

单项验收入口：最小链路使用 `scripts/verify_minimal_chain.py --mode direct`；AgentOS 意图使用 `--mode intent` 或 `--mode both`；仿真基线使用 `scripts/verify_simulation_baseline.py`；执行回放使用 `scripts/export_replay_manifest.py --execution-id <id>`；MuJoCo 画面渲染使用 `scripts/render_piper_mujoco.py`。Runtime 服务可长期运行 MuJoCo，健康接口返回 `continuous_simulation: true` 和 `simulation` 指标；认证后的 `GET /v1/simulation/metrics` 返回同一指标快照。需要持续观察画面时使用 `scripts/render_piper_mujoco.py --forever`，该入口同样通过 Runtime、SkillRuntime、Backend 和 Supervisor，不维护第二套动作控制逻辑。tty 使用 EGL 生成 PNG/GIF，图形会话才使用交互 viewer。详细设计、实施计划、安全论证和验证记录见同目录下的 `iraf-*.md` 文档。
