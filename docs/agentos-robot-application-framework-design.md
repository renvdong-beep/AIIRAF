# AgentOS 机器人应用框架设计说明

**版本**：v0.1  
**日期**：2026-08-27  
**配套架构图**：[总体架构图](../agentos机器人应用框架-更新版.svg) · [IRAF 详细技术架构图](../IRAF详细技术架构图.svg)  
**兼容设计**：[AgentOS / Hyper 构型兼容设计](iraf-agentos-hyper-compatibility.md)

## 1. 目标与边界

本文定义鸿道机器人面向应用开发的统一框架，暂称 **IRAF（Intewell Robot Application Framework）**。框架基于 AgentOS 运行，但不重复实现 AgentOS 的通用能力。

目标是让巡检、导览、配送、迎宾、工业操作、家庭服务、安防和园区运营等应用，通过统一的任务、技能和能力接口开发，并能在 Ubuntu 仿真环境和边缘真机之间迁移。

### 1.1 分层边界

| 层级 | 责任 | 不负责 |
|---|---|---|
| AgentOS | Agent 调度、记忆、存储、工具、权限、治理、模型调用编排 | 机器人具体运动控制和设备协议 |
| IRAF | 任务编排、技能运行、能力抽象、本体配置、安全策略、数据闭环 | Hypervisor、底层驱动和实时主站实现 |
| 执行底座 | 智控、运控、通信主站、设备协议、实时保障 | 业务任务语义 |

## 2. 技术判断

人形及通用机器人不会由单一 VLA 模型完全驱动。可工程化的系统是分层混合架构：传统控制保障实时性与安全；学习策略提升技能适应性；VLM/VLA/LLM 负责理解任务、选择技能与失败重规划。

```text
任务理解 / 世界模型 / VLA
        ↓
TaskFlow 任务编排
        ↓
Skill Runtime 技能执行
        ↓
导航、操作、WBC、MPC、状态估计
        ↓
设备通信、执行器与安全保护
```

结论：VLA 是可插拔的任务智能与策略来源，不能直接输出电机指令，也不能替代实时控制和安全保护。

## 3. 总体架构

配套 SVG 架构图保持原有的四层结构，并补充以下关键抽象：

1. **TaskFlow**：任务图、行为树、状态机、重试、恢复和人工接管。
2. **World Model**：持续维护机器人、环境、任务和预测状态。
3. **Policy Gateway**：统一执行输入校验、权限、约束和安全策略。
4. **Robot Profile**：将不同机器人本体的能力和边界配置化。
5. **Skill Lifecycle**：技能从注册、仿真、训练到部署、运行、采集和迭代的闭环。

IB-Robot 的可借鉴方向应限定为应用框架能力：任务/流程编排、设备与能力封装、技能路由、状态反馈、运行记录和数据闭环。具体模块是否复用，应在获得其源码、许可和接口说明后评审，不应先假定其实现可直接集成。

## 4. 三域职责

### 4.1 智控域

负责非硬实时的认知和决策：

- AI Agent、LLM、VLM、VLA；
- 世界模型、长期记忆、任务记忆；
- 自然语言理解、视觉语义理解；
- TaskFlow 生成或调整；
- 技能选择、失败重规划；
- 遥操作数据、训练与评测数据管理。

智控域输出受约束的任务或 Skill Goal，例如“执行 `pick_object`，目标为 `box_01`”，不输出原始关节、电机、力矩或现场总线报文。

### 4.2 运控域

负责把任务目标变成连续、可验证的机器人运动：

- ROS 2、AGIROS 与机器人能力适配；
- Skill Runtime、Capability API、Robot Profile；
- 导航、定位、避障、抓取、装配、视觉伺服；
- 逆运动学/动力学、WBC、MPC、轨迹跟踪；
- 状态估计、执行反馈、局部恢复。

### 4.3 通信主站域

负责确定性设备通信、聚合与诊断：

- EtherCAT、AUTBUS 等主站；
- 设备发现、设备状态、通信诊断；
- 周期控制帧下发和状态帧采集；
- 通信超时、看门狗、受控停机；
- 与驱动器、传感器、执行器的协议适配。

### 4.4 域间通道

| 通道 | 方向 | 内容 | 原则 |
|---|---|---|---|
| AI 任务通道 | 智控域 ↔ 运控域 | TaskFlow、Skill Goal、世界状态、技能反馈 | 异步、可重试、语义化 |
| 实时控制通道 | 运控域 ↔ 通信主站域 | 受约束目标、设备状态、故障事件 | 固定格式、带时间戳、超时保护 |
| 管理通道 | AgentOS ↔ 各域 | 生命周期、配置、权限、日志、升级、健康检查 | 受治理、可观测 |

AgentOS 通过独立 `AgentOSBridge` 使用四个 IRAF 公共服务，并通过管理契约完成版本协商、健康、drain 和 shutdown。南向部署由签名 `HyperProfile` 映射当前三个正式发布构型：E300 3VM-LPR、E300 3VM-LPP、FIREFLY-RK3588 2VM-LR。IRAF 不硬编码 VM 编号/IP；2VM-LR 中运控与主站角色同 VM 但保持进程、权限、资源和设备隔离。详细约束以兼容设计和 ADR-0005 为准。

## 5. 世界模型

世界模型不是单纯地图，也不是一次性视觉识别结果。它是对当前状态及动作后果的持续性内部表示。

```text
环境：物体、空间、障碍物、人员、可通行区域
机器人：位姿、关节、传感器、电量、健康状态
任务：当前步骤、已完成动作、失败原因、优先级
关系：物体位置、归属、可抓取区域、人机距离
预测：采取某个动作后的可达性、碰撞风险和任务结果
```

世界模型主要驻留在智控域，但必须持续接收运控域提供的可信机器人状态、执行结果与异常事件。接口可定义为：

```text
world.get_state()
world.update(observation)
world.query(entity, relation)
world.predict(action)
```

## 6. TaskFlow 任务编排

TaskFlow 是业务应用到 Skill 的唯一主要入口。它可以采用行为树、状态机或任务图实现，并应包含前置条件、成功条件、超时、重试、恢复和人工接管。

配送任务示例：

```yaml
task: delivery
steps:
  - skill: navigate
    destination: pickup_station
  - skill: pick_object
    object_id: package_01
  - skill: navigate
    destination: delivery_station
  - skill: place_object
    target: delivery_station
  - on_failure: recover_or_handover
```

VLA 可以协助生成或调整任务图，但 Skill Runtime 必须负责执行检查，不能让模型绕过框架直接控制设备。

## 7. Skill 设计

### 7.1 定义

Skill 是可注册、可发现、可执行、可取消、可恢复、可评测的机器人能力单元。例如：`navigate`、`detect_object`、`pick_object`、`place_object`、`inspect`、`return_home`。

Skill 不是单个模型，也不是单个 ROS 节点；它是对 ROS 2 节点、行为树、传统算法、学习策略或 VLA 策略等实现的统一封装。

```text
应用 / Agent → TaskFlow → Skill Runtime → Skill Provider → 运控与通信主站
```

### 7.2 Skill 合约

每个 Skill 必须声明：

- 输入、输出及 schema；
- 依赖的机器人能力；
- 前置条件、成功条件与失败码；
- 超时、可取消性、重试和恢复策略；
- 安全等级和权限要求；
- Skill、模型、控制器和本体版本。

示例：

```yaml
apiVersion: iraf.intewell.io/v1
kind: Skill
metadata:
  name: pick_object
  version: 1.0.0
spec:
  inputs:
    object_id: {type: string, required: true}
    target_pose: {type: pose, required: false}
  requires: [arm, gripper, rgbd_camera]
  preconditions: [robot.mode == ready, emergency_stop == false]
  provider:
    type: ros2_action
    endpoint: /skills/pick_object
  policy:
    timeout_seconds: 30
    retry_limit: 2
    fallback: [re_perceive, adjust_grasp, request_handover]
  outputs: [grasp_pose, object_pose, execution_trace]
```

### 7.3 参数分级

Skill 参数应参数化，但不能全部由外部任意修改。

| 参数级别 | 示例 | 修改者 |
|---|---|---|
| 任务输入 | 目标物体、目的地、数量、优先级 | Agent、应用、用户 |
| 执行偏好 | 速度等级、超时、路线偏好 | Agent、应用，必须在范围内 |
| 技能配置 | 抓取策略、模型版本、相机选择 | 授权开发者或运维 |
| 安全边界 | 限速、限扭、碰撞阈值、禁区 | 安全策略与受控运维 |

Skill Runtime 的参数路径必须是：

```text
外部参数 → schema 校验 → 范围校验 → 权限校验
→ 与 Robot Profile / Safety Policy 合并 → Skill Provider 执行
```

### 7.4 Skill 状态机

```text
单次执行：PENDING → VALIDATING → RUNNING → SUCCEEDED
异常终态：FAILED / ABORTED / CANCELLED / SAFETY_STOP
任务编排：失败 → 新建恢复执行 / HANDOVER_REQUIRED / 任务终止
```

单次 Skill execution 到达终态后不可复活。重试和恢复由 TaskFlow 创建新的 execution，并通过因果 ID 关联原失败执行，避免设备动作被隐式重复。

### 7.5 多实现选择

同一 Skill 可按本体、环境和资源选择不同 Provider：

```text
pick_object.sim：MuJoCo 仿真实现
pick_object.classical：视觉 + MoveIt + 传统抓取规划
pick_object.policy：ACT / Diffusion / 强化学习策略
pick_object.vla：VLA 引导的操作策略
```

Skill Runtime 依据 Robot Profile、运行模式、模型可用性和安全策略进行路由。上层应用只调用 `pick_object`，不绑定某一模型或具体 ROS topic。

## 8. L0 实时能力与 AI 的关系

大模型/VLA 不参与实时闭环。L0 实时能力使用确定性控制算法与传感器闭环实现。

| L0 能力 | 主要方式 | 执行位置 |
|---|---|---|
| 急停 | 急停状态机、驱动失能、安全继电器 | 通信主站域，必要时硬件 |
| 限速 | 速度/加速度/力矩饱和、禁区规则 | 运控域 + 通信主站域 |
| 避障 | 传感器融合、局部代价地图、局部规划器 | 运控域 |
| 轨迹跟踪 | PID、LQR、MPC、WBC、纯跟踪 | 运控域 |
| 力控 | 阻抗/导纳/力矩控制、接触检测 | 运控域，底层设备闭环在通信主站域 |
| 失联保护 | 看门狗、控制帧超时、缓停/失能 | 通信主站域 |

延迟目标应按层级划分：

| 层级 | 典型目标 |
|---|---|
| 大模型任务理解与重规划 | 0.5 秒至数秒 |
| 感知、技能策略、局部决策 | 30 至 200 ms，动态视觉伺服可更低 |
| 运控闭环 | 10 至 100 Hz，具体取决于本体 |
| 主站与设备周期 | 通常 1 至 4 ms，按设备协议和硬件能力确定 |

AI 只负责“做什么、选哪个 Skill、给出什么目标”；Skill 启动后，由运控域和通信主站域持续闭环。因此 AI 推理耗时不会直接阻塞机器人行走、避障、抓取和安全停机。

## 9. Ubuntu 开发与仿真运行时

Ubuntu 不应使用只返回固定结果的 Mock AgentOS 来替代系统验证。应正式提供 **AgentOS Dev Runtime**：运行真实的 AgentOS 核心逻辑，仅替换边缘特有的 Hyper 分域、实时主站和真机驱动。

| 能力 | Ubuntu Dev Runtime | 边缘生产运行时 |
|---|---|---|
| AgentOS 核心 | 同一份真实代码 | 同一份真实代码 |
| SDK、IDL、TaskFlow、Skill、Robot Profile | 同一契约和版本 | 同一契约和版本 |
| 模型调用 | 云端、局域网推理服务、本地 GPU/CPU、回放 | 本地摩尔线程推理服务或云端 |
| 机器人执行 | MuJoCo、HIL 适配器 | 真实 ROS 2、运控、设备主站 |
| 实时与安全 | 软件约束、故障注入 | 实时调度、设备看门狗、急停和硬件限位 |

推荐三种启动 Profile：

```text
dev-sim：Ubuntu + AgentOS Dev Runtime + 仿真机器人
dev-hil：Ubuntu + AgentOS Dev Runtime + 真实或半真实通信/控制设备
edge-prod：边缘 AgentOS + 三域 + 真实机器人
```

### 9.1 可信度保障

1. 用容器交付固定版本的 `agentos-dev-runtime`，避免开发者手工拼装环境。
2. Dev Runtime 与生产版从同一 API 契约、同一核心代码和同一版本策略构建。
3. 对 SDK、TaskFlow、Skill 状态迁移、错误码和日志格式运行双端契约测试。
4. CI 建立 Ubuntu 与边缘的兼容性矩阵，至少覆盖当前版和上一版 SDK。
5. 运行时显式暴露 `simulation`、`realtime`、`hardware` 等能力标识，不伪造真机能力。

## 10. 局域网调用边缘 AI 芯片

Ubuntu 开发机可通过局域网调用边缘端摩尔线程推理服务：

```text
Ubuntu：AgentOS Dev Runtime + ROS 2 仿真 + 应用开发
        ↓ Model Provider API（gRPC / HTTP）
边缘端：摩尔线程推理服务 + 模型与权重
```

AgentOS 应定义统一模型接口，例如：

```text
ModelProvider.generate(...)
ModelProvider.embed(...)
PolicyProvider.infer(...)
```

可提供不同 Provider：

- `LocalMusaProvider`：边缘端本地调用摩尔线程服务；
- `RemoteMusaProvider`：Ubuntu 经局域网调用边缘推理服务；
- `CloudProvider`：调用云端模型；
- `CudaProvider`：Ubuntu 的 NVIDIA GPU 推理；
- `CpuProvider`：小模型和功能验证；
- `ReplayProvider`：固定结果回放，用于可重复测试。

局域网推理只适用于任务理解、场景语义、目标识别、VLA 技能选择和失败重规划。实时闭环不能依赖网络。必须具备鉴权、TLS、超时、限流、健康检查、模型版本校验和断网降级策略。

## 11. 首期实现范围

### 11.1 必须优先完成

1. IRAF SDK 与统一 IDL：`Observation`、`RobotState`、`SkillGoal`、`SkillFeedback`、`SafetyEvent`、`TaskStatus`。
2. TaskFlow 最小运行时：状态机或行为树、超时、取消、重试、恢复与接管。
3. Skill Registry、Skill Runtime、Policy Gateway 与 Robot Profile。
4. 三个基础 Skill：`navigate`、`pick_object`、`place_object`，以及 `stop` 和 `recover`。
5. AgentOS Dev Runtime 容器与 MuJoCo 仿真适配器。
6. 实时控制和设备主站的受控目标接口及安全事件上报。
7. 运行记录、回放和最小契约测试。

### 11.2 后续接入

- VLA、世界模型、扩散策略、强化学习策略；
- 遥操作与示范数据采集；
- 多机器人协同；
- 云边模型调度和模型版本治理；
- 更丰富的工业、服务与人形机器人 Skill 包。

## 12. 验收指标

首期不以“模型多大”作为核心验收，而以可迁移、可恢复、可观测和安全为标准：

1. 同一 TaskFlow 能在 Ubuntu 仿真和边缘真机加载并执行。
2. 同一 Skill 在不同 Provider 下保持统一输入、输出、状态与错误码。
3. AI 服务不可用时，已启动的 L0/L1 运动闭环继续受控，任务可安全降级、暂停或接管。
4. 所有 Skill 调用可追溯到任务、版本、输入、执行轨迹、模型版本和结果。
5. 外部参数无法突破 Robot Profile 和 Safety Policy 的安全边界。
6. 断网、模型超时、设备失联、传感器异常等故障可通过故障注入和回放验证。

## 13. 结论

鸿道机器人应用框架的核心价值不是再造一个机器人底层系统，也不是绑定单一 VLA，而是在 AgentOS 之上提供稳定的机器人领域抽象：

```text
Agent 应用 → TaskFlow → Skill → Capability → 智控/运控/通信主站 → 设备
```

AgentOS 提供通用 Agent 运行与治理；IRAF 将其转化为可执行、可验证、可恢复的机器人应用；智控域、运控域与通信主站域分别保证任务智能、运动执行和确定性设备通信。该设计既能采用当前成熟的 ROS 2、MPC、WBC 与设备主站能力，也能在不破坏安全边界的前提下接入 VLA、世界模型和未来学习策略。
