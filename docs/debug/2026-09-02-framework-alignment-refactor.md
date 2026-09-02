# IRAF 铁律对齐重构验收记录

日期：2026-09-02

- 将执行逻辑从 AgentOS Dispatcher 移入 IRAF SkillRuntime。
- 将硬编码 Skill 注册改为 manifest、JSON Schema 和 Provider entrypoint。
- 拆分 canonical Protobuf 与 AgentOS Bridge Protobuf，并用锁定的 grpcio-tools 生成代码。
- 增加 SafetyPolicy、仿真信任边界、SQLite 幂等快照与状态事件。
- HTTP 标记为 development-simulation，并由 systemd 最小权限运行。
- E300 模型结构化输出由 manifests 动态生成，未通过 schema 时禁止调用 Backend。
- 单元测试、canonical HTTP、MuJoCo 和文字 stop 场景通过。

限制：尚未接入 AgentOS 正式 TaskFlow/Skill Manager 服务；不得把本次结果表述为生产 AgentOS 或真机能力。