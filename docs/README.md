# IRAF 文档导航（v1.8）

## 产品与架构

- [v1.8 总体架构图](../iraf-refined-architecture.png)：AgentOS、Skill Query、Tool/Skill Manager、TaskFlow、Provider 与仿真/真机后端。
- [v1.8 详细接口/安全架构图](../iraf-detailed-interface-security.png)：公共接口、Manifest/Registry、执行契约、状态、幂等、资源隔离与 SafetyEvent。

- [总体架构图](../agentos机器人应用框架-更新版.svg)：AgentOS、IRAF、多模式 Provider 与执行底座。
- [IRAF 详细技术架构图](../IRAF详细技术架构图.svg)：公共 API、核心模块、Provider、MuJoCo/真机后端与安全纵切面。
- [原始架构设计](agentos-robot-application-framework-design.md)：IRAF 的定位、三域边界与总体原则。
- [工程化设计](iraf-engineering-design.md)：模块、跨平台、质量与易用性基线。
- [易用性与泛化实施方案](iraf-developer-experience.md)：用户旅程、脚手架与体验验收。
- [Piper 与机器狗协同仿真](iraf-piper-quadruped-simulation.md)：首期端到端场景。
- [Piper 多模式控制](iraf-piper-multimode-control.md)：传统控制、VLA、学习策略与 MuJoCo 后端。
- [竞品与开源能力借鉴](iraf-competitive-landscape.md)：整体平台与局部开源能力的复用、自研和持续跟踪边界。
- [AgentOS / Hyper 构型兼容设计](iraf-agentos-hyper-compatibility.md)：北向契约、三项发布构型、跨域通道与验收矩阵。
- [初期安全论证](iraf-safety-case.md)：安全状态、危险、防护与复位协议。

## 可编码详细设计

- [IDL、Registry 与 Runtime](iraf-idl-runtime-detailed-design.md)：Protobuf、manifest、Provider 接口、状态、租约和错误码。
- [公共 API 契约目录](iraf-api-contract-catalog.md)：四类服务、消息字段、认证、幂等和流式续传。
- [TaskFlow 与 Policy Gateway](iraf-taskflow-policy-detailed-design.md)：DSL、持久化、恢复、策略规则和互锁。
- [边缘适配、数据闭环与 AgentOS 接入](iraf-edge-data-detailed-design.md)：ROS 2、Piper/机器狗、世界模型、回放、部署。

## 实施与治理

- [实施计划](iraf-implementation-plan.md)：三阶段工程实施路线与风险。
- [当前框架对齐状态](iraf-framework-alignment-status.md)：已实现核心、适配器边界、验证证据与未完成项。
- [开发与验证流程](iraf-development-workflow.md)：CI、运行模式与发布。
- [24 小时连续开发审查](iraf-24h-readiness-review.md)：产品/项目 Go/No-Go 与依赖。
- [项目治理、RAID 与评审记录](iraf-governance-review.md)：RACI、风险依赖和正式签署。
- [ADR 目录](adr/)：已经接受或待验证的架构决策。
- [ADR-0003：Humble/MuJoCo 基线与 Ubuntu 24.04 兼容](adr/0003-humble-mujoco-platform.md)
- [ADR-0004：首期四足平台选择宇树 Go2 EDU](adr/0004-unitree-go2-edu.md)
- [ADR-0005：AgentOS 北向契约与三 Hyper 构型适配](adr/0005-agentos-hyper-compatibility.md)
- [工程实施铁律](../AGENTS.md)：所有编码和交付必须遵守的规则。
