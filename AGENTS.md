# IRAF 工程实施铁律

## 0. 项目定位

IRAF（Intewell Robot Application Framework）将 AgentOS 的任务请求转化为安全、可观测、可恢复的机器人 Skill。IRAF 是应用框架，不替代实时控制器、现场总线主站、BSP、驱动或 Hypervisor。

## 1. 不可违反的安全铁律

1. LLM、VLM、VLA、远程服务和业务应用不得直接发送电机、关节、力矩、现场总线或驱动命令。
2. 所有外部请求必须经过 `TaskFlow -> Skill Runtime -> Policy Gateway -> Capability Provider`，禁止任何旁路。
3. 安全上限只来自已签名的 `RobotProfile` 与 `SafetyPolicy`。调用参数只能收紧，不能放宽安全边界。
4. 实时控制路径必须本机、确定、有界、带时间戳和看门狗；网络推理、HTTP、gRPC、AgentOS 均不得进入 L0/L1 实时闭环。
5. 模型、网络、传感器、Provider 或设备不可用时，只能显式失败、安全停机、恢复或人工接管；禁止返回伪造成功。
6. 急停、设备失联和安全事件优先级高于任务成功、重试和模型指令。终态不得被回写为成功。
7. 仿真环境必须声明 `simulation=true`；不得把仿真或 Mock 结果表述为真机/实时能力。
8. 身份与角色只能来自受信 transport 的 `AuthenticatedContext`；不得信任请求体中的 caller、role 或 tenant。
9. SafetyEvent 后相关资源必须进入 `QUARANTINED`，控制器确认 safe state 且授权复位前不得释放或重新调度。

## 2. 编程铁律

1. 先写契约与验收场景，再写实现。任何公开消息、状态、错误码或 Skill 参数变化先修改 `api/` 的 IDL。
2. Protobuf 是管理与 SDK 的唯一事实来源；ROS 2 topic/action 仅是边缘适配层，不能成为公共 API 的唯一来源。
3. 不允许吞掉错误、空 catch、默认成功、无限重试、无 deadline 的 RPC、无超时的等待或无取消句柄的任务。
4. 所有状态变迁必须幂等、带单调序号、可追溯；所有异步操作须有 `correlation_id`、截止时间和取消路径。
5. TaskFlow、Skill Runtime、Policy 等核心逻辑不得依赖某块板、某个 ROS topic、GPU/NPU SDK 或发行版私有库。硬件差异只能放在 `adapters/` 与 `profiles/boards/`。
6. 不可变配置优先。不得在运行中隐式修改 Profile、Safety Policy、模型、控制器或 Skill 版本。
7. 不提交密钥、内部地址、模型权重、传感器原始数据、构建产物或日志。远程接口使用 mTLS、最小权限、超时、限流和健康检查。
8. 每一个功能必须测试成功路径和拒绝/失败路径；安全、状态机、契约和 profile 变更必须增加回归测试。
9. 不允许只靠 Mock 成功完成验收。仿真要发布真实状态与故障；硬件功能须有 HIL 或真机证据。
10. 合并前必须通过格式化、静态检查、单元测试、IDL 兼容检查和相关仿真测试。板级/实时接口变更必须附验收记录。
11. 第三方或跨发行版 Provider 默认独立进程，通过版本化 gRPC/ROS 2 Action 通信；禁止把 C++ `.so` 当作通用跨平台插件 ABI。
12. 所有物理动作必须由服务端生成 execution_id，并使用幂等键、资源租约和 fencing token；旧 token 或终态执行发出的控制请求必须被拒绝。
13. Piper 的传统控制、VLA、学习策略、遥操作和回放必须经过 ControlAuthorityManager；同一执行器任一时刻只允许一个控制源。

## 3. 分层与目录铁律

目标结构见 `docs/iraf-engineering-design.md`：`api/` 放公共 IDL，`core/` 放平台无关逻辑，`adapters/` 放 ROS 2/仿真/主站/模型集成，`profiles/` 放机器人、板卡和安全配置，`tests/` 放分层验证。

- 每个 Skill 必须声明输入/输出 schema、能力依赖、前置与成功条件、错误码、超时、取消、恢复、安全等级和版本。
- 每个 `BoardProfile` 必须声明 OS、架构、能力、适配器、镜像摘要和测试证据；未知值必须写 `unverified`，不能猜测。
- 每次执行记录任务、Skill、Provider、模型、控制器、Profile、Policy 版本，策略决定、状态迁移和 trace ID，并脱敏。

## 4. 交付铁律

工作项开始前须明确：目标环境、验收场景、Skill/Profile/IDL 影响、风险等级和失败/接管路径。完成的定义是：代码与文档一致，契约兼容性已验证，成功/失败路径都有证据，且没有突破本文件的安全边界。

## 5. 工程习惯与自动化铁律

1. 项目内 Markdown、操作说明、调试记录、注释和面向使用者的报错统一使用中文；命令、代码、变量、文件名和协议名可保留原文。引用英文资料时补充中文结论和适用范围。
2. 发现重复、易错或需要人工介入的操作时，先把它设计成可复跑的脚本、CLI 子命令或 CI 任务，再写操作手册。人工步骤仅可作为硬件安全确认，且必须记录责任人和证据。
3. 所有端点、目录、镜像、模型、超时、板卡能力和版本配置必须集中在版本化 Profile/配置文件中；禁止在业务代码、脚本和文档中分散硬编码。
4. 排查或修改前，先阅读 `docs/debug/`、对应模块的 README、已知问题和历史验收记录；修复后必须新增或更新调试记录、回归测试与自动化检查，避免重复踩坑。
5. 构建、仿真、打包和部署均须从仓库根目录的受控入口执行，输出写入 `build/`、`artifacts/` 或 CI 工作目录，禁止污染系统目录、源码目录或未声明的临时位置。
6. 脚本必须 `set -euo pipefail`（适用时），显式检查依赖、输入、架构和返回值；失败要给出可操作的中文原因，禁止继续执行并制造半成品。
7. 变更必须小而可审查。提交采用 Conventional Commits；架构、IDL、安全、实时和板级变更需额外架构审查，并附设计/验证证据。

## 6. 易用性与泛化铁律

1. 目标用户是机器人应用开发者、算法工程师、集成商和运维人员。默认路径必须使其仅提供 TaskFlow、Skill 参数与 Robot Profile，而不要求了解 ROS topic、现场总线或板卡 SDK。
2. 每项公共能力必须提供可发现的 schema、中文说明、最小可运行示例、明确错误码和诊断建议；禁止依赖隐式环境变量、隐藏默认值或口头知识。
3. 新机器人和新 Skill 必须通过脚手架、声明式 Profile、标准 Provider 接口和契约测试接入。若接入需要复制/修改 Runtime 核心代码，先重构接口而不是继续堆特例。
4. 对常见场景提供“金路径”：`init` 创建工程，`dev up` 启动仿真，`skill scaffold` 创建 Skill，`profile check` 检查能力，`test scenario` 运行验收，`package/deploy` 生成可追溯产物。命令实现前不得在文档中假称已可用。
5. 任何可选加速器、模型、仿真器和机器人本体必须可替换、可降级、可声明；应用只能依赖能力，不得依赖具体厂商、模型名、topic 或设备路径。
6. 架构模块、边界、数据流、Provider 模式、平台或安全路径发生变化时，必须在同一变更中更新总体架构图、IRAF 详细技术架构图和相关 ADR；图文不一致不得合并。
7. Ubuntu 22.04/Humble 是认证基线；Ubuntu 24.04 只能以 Jammy/Humble 容器兼容或独立 Jazzy 实验档进入。禁止在 Noble 主机混装 Humble 并宣称受支持。
8. 首期四足本体固定为 Go2 EDU；厂家 SDK/MJCF 示例只证明对应低层能力，导航、停靠、传感器和交接必须分别验收，禁止用 mock 结果替代真能力证据。
9. AgentOS 只能经 `AgentOSBridge` 和版本化公共服务进入 IRAF；AgentOS 私有类型、身份声明、ROS topic 或设备凭据不得穿透公共边界。
10. 必须兼容 `MQ50-E300-3VM-LPR`、`MQ50-E300-3VM-LPP`、`FIREFLY-RK3588-2VM-LR` 三个正式 Hyper 构型。VM 数量、编号、IP、OS 和角色映射只能来自签名 HyperProfile/Release manifest，禁止硬编码。
11. 2VM 构型合并部署角色时不得合并安全边界；Motion 与 Master 仍须独立进程、权限、资源限额和设备所有权。S600 未有正式发布构型前禁止标记 S2。
