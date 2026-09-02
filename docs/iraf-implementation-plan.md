# IRAF v1.8 工程实施计划

**目标**：在 Ubuntu 仿真端和 E300 AgentOS 边缘端形成同一套 TaskFlow、Skill、RobotProfile 语义可回放、可验证、可迁移的最小垂直闭环。

**实施原则**：先交付可运行的任务闭环，再扩展 Provider；模型只参与语义规划或策略选择，任何模型、AgentOS 或网络服务不得直接写入关节、力矩、现场总线或驱动控制通道。

## 1. 环境与现状基线

### 1.1 目标环境

| 环境 | 地址/架构 | 责任 | 运行边界 |
|---|---|---|---|
| Ubuntu 开发/仿真机 | `10.203.247.145`，以 x86_64 为当前基线 | IRAF 核心、ROS 2、MuJoCo、测试与回放 | 不承载 E300 专用运行时；可使用 Jammy/Humble 容器隔离依赖 |
| E300 边缘板卡 | `10.203.247.72`，aarch64 | AgentOS、边缘模型和后续真机适配 | 通过容器运行 aarch64 Intewell-Agent；不把板卡私有类型泄漏到 IRAF 公共 API |

### 1.2 已有可复用工程

| 能力 | 现有位置 | 使用方式 |
|---|---|---|
| Piper ROS 2 工作空间 | `/home/coretek/piper_ros` | 作为 ROS 2 上游工程，只通过 Adapter 接入，不复制到 IRAF 核心 |
| Piper MuJoCo 控制包 | `/home/coretek/piper_ros/src/piper_sim/piper_mujoco` | 当前关节控制基线；订阅 `/joint_states`，驱动 MuJoCo 模型 |
| Piper Gazebo/ROS 2 控制 | `/home/coretek/piper_ros/src/piper_sim/piper_gazebo` | 用于 ROS 2 控制和后续真机接口对照 |
| Piper MuJoCo 模型 | `/home/coretek/piper_ros/src/piper_description/mujoco_model` | 由 `MuJoCoAdapter` 加载；模型路径进入 `RobotProfile` |
| MuJoCo 3.x 与学习环境 | `/home/coretek/MuJoCoBin` | 用于独立仿真、数据采集和学习 Provider，不作为公共控制 API |
| Python 虚拟环境 | `/home/coretek/miniconda3/envs/mujoco_graspnet` | Python 3.9.21、MuJoCo 3.3.1；由仿真启动脚本显式选择 |

现有 Piper 控制脚本使用 `mujoco_py`，而独立 MuJoCo 环境使用 MuJoCo 3.x API。两套运行时必须在 Adapter/启动脚本中隔离，不能在同一 Python 进程隐式混用。

### 1.3 当前已具备

- 可复用 MuJoCo 仿真和 Piper ROS 2 关节控制代码。
- Piper 仿真抓取场景基础闭环。
- RobotProfile 统一规约基础。
- E300 AgentOS 容器部署和 Qwen3-0.6B OpenAI 兼容 API：`https://10.203.247.72:9119/v1`。

当前 `AIIRAF` 目录仍以架构和工程设计文档为主，公共服务与执行内核按本计划实现。

## 2. 目标模块与协调关系

```text
AgentOS / 应用请求
        |
        v
AgentOSBridge（认证上下文、版本兼容、超时、取消）
        |
        v
TaskFlow（任务状态机、依赖、重试、恢复、人工接管）
        |
        v
SkillRuntimeService -> PolicyGateway -> ProviderRouter
        |                         |
        |                         +-> Classical Provider（ROS 2 / MoveIt / 本地规划）
        |                         +-> VLA Provider（ACT、Diffusion 等策略）
        |                         +-> RL Provider（学习策略）
        |                         +-> Model Provider（Qwen/远程模型/回放）
        v
ControlAuthorityManager（单写者、租约、fencing token）
        |
        v
ROS2 Adapter -> Joint/Trajectory 控制接口 -> MuJoCoAdapter 或真机后端
```

### 2.1 不可跨越的边界

1. `TaskFlow`、`SkillRuntimeService` 和 `PolicyGateway` 不依赖 ROS topic、板卡 SDK、GPU/NPU 或具体模型。
2. ROS 2 是当前关节控制实现和边缘适配层，不是 IRAF 公共契约；公共消息使用版本化 Protobuf/JSON Schema。
3. Classical、VLA、RL 和遥操作 Provider 均输出标准化 `ControlIntent` 或 `TrajectoryGoal`，必须经过 `ControlAuthorityManager` 和策略校验。
4. Provider 不得直接访问电机、现场总线或驱动；`ROS2Adapter`/真机后端负责最后一跳，并提供本地超时、限幅、看门狗和安全停止。
5. VLA/RL/远程模型不进入实时闭环。模型失联只影响尚未开始的规划或策略选择；已经开始的运动由本地控制器完成停止、暂停或恢复。
6. 同一机器人同一时刻只允许一个控制源持有有效租约；旧租约、终态执行和过期 fencing token 必须拒绝。

### 2.2 目录目标

```text
AIIRAF/
├── api/proto/                 # Task、Skill、Observation、RobotState、Event、错误码
├── core/taskflow/             # 状态机、取消、重试、恢复、幂等
├── core/skill-runtime/        # SkillRuntimeService：生命周期、路由、执行上下文
├── core/policy/               # 参数范围、能力、碰撞、安全策略
├── core/registry/             # Skill Query、Tool/Skill Manager 的事实源
├── adapters/agentos/          # AgentOSBridge
├── adapters/ros2/             # ROS 2 JointState/Trajectory/Action 适配
├── adapters/mujoco/           # Piper MuJoCo 仿真适配
├── adapters/real/             # 真机后端，后续接入
├── providers/classical/       # Classical CapabilityProvider：ROS 2/MoveIt/传统算法
├── providers/vla/             # ACT、Diffusion 等策略 Provider
├── providers/rl/              # RL CapabilityProvider：强化学习策略
├── profiles/robots/           # Piper RobotProfile
├── profiles/boards/           # Ubuntu/E300 BoardProfile
├── skills/piper/              # stop、move_joint、pick、place
├── tests/{unit,contract,simulation,hil}/
└── tools/                     # profile-check、scenario、release-evidence
```

## 3. 分阶段交付

### 一阶段：仿真最小垂直闭环

**目标**：同一套 Skill 合约在 Ubuntu MuJoCo 仿真中运行，并能由 E300 AgentOS 发起任务。

**实现顺序**：

1. 建立仓库骨架、Ubuntu 22.04/ROS 2 Humble 开发容器、Python 仿真环境声明和受控入口脚本。
2. 定义并生成公共 IDL：`TaskRequest`、`SkillGoal`、`SkillFeedback`、`RobotState`、`Observation`、`SafetyEvent`、`TaskStatus`。
3. 实现 `RobotProfile`、`BoardProfile`、`SafetyPolicy` 的 YAML schema、版本和静态检查；模型路径、关节名、控制频率、限位和仿真标识不得硬编码在业务代码中。
4. 实现 `SkillRegistry`、Skill Query、Tool/Skill Manager 的 manifest 查询和能力匹配；未注册 Skill、能力缺失或版本不兼容必须显式拒绝。
5. 实现 `TaskFlow` 和 `SkillRuntimeService` 状态机：`PENDING -> VALIDATING -> RUNNING -> SUCCEEDED`，异常终态包含 `FAILED/ABORTED/CANCELLED/SAFETY_STOP`；所有请求带 correlation ID、deadline、幂等键和取消路径。
6. 实现 `PolicyGateway` 和 `ControlAuthorityManager`，先支持 `stop`、`move_joint`、`pick_object` 的仿真安全边界。
7. 基于现有 `/home/coretek/piper_ros` 实现 `ROS2Adapter`：订阅/发布标准化状态，调用现有 ROS 2 关节控制；不得让 TaskFlow 直接依赖 `/joint_states`。
8. 实现 `MuJoCoAdapter`：加载 Piper 模型，发布真实仿真状态和故障事件；明确 `simulation=true`，不得将仿真结果标记为真机能力。
9. 实现 `AgentOSBridge`：接收 E300 AgentOS Task，转换为 IRAF `TaskRequest`，返回状态/反馈；使用版本化接口、超时、取消和健康检查。
10. 增加回放 manifest、结构化日志和最小 WorldModel snapshot；成功、拒绝、超时、取消、模型断连、仿真故障均需可回放。

**一阶段验收**：

- Ubuntu 上可启动 Piper MuJoCo 场景，完成 `move_joint -> pick_object -> place_object -> stop` 仿真流程。
- E300 AgentOS 可提交同一 TaskFlow，Ubuntu 返回单调序号的 SkillFeedback 和最终状态。
- ROS 2 控制代码可复用，TaskFlow/Skill 合约不包含 ROS topic 或 Python 虚拟环境细节。
- Provider 未注册、参数超限、租约冲突、取消、模型不可用和仿真故障均进入明确错误/安全终态。

### 二阶段：Provider 扩展与仿真/真机统一

**目标**：在不改变 Skill 公共契约的前提下，引入传统算法、VLA、RL 和真机后端。

1. 将传统 ROS 2/MoveIt/轨迹规划实现为 Classical Provider；保留本地控制器作为默认和安全回退。
2. 定义 VLA Provider 输入输出契约，支持 ACT、Diffusion 等策略；策略只输出受约束的目标/轨迹，不拥有控制通道。
3. 定义 RL Provider 生命周期、观测版本、动作空间和离线回放；策略异常必须回退 Classical Provider 或安全停止。
4. 扩展 `ControlAuthorityManager`：Provider 互斥、租约续期、优先级、抢占、fencing token 和人工接管。
5. 将 `pick/place/accept_payload/delivery_handoff` 组织为 TaskFlow；分别验证 TF、物体掉落、设备失联和双控制源冲突。
6. 增加真机后端接口；真机适配只能新增 `adapters/real`、BoardProfile 和设备证据，不修改核心 TaskFlow。
7. 在 E300 上通过 AgentOSBridge 调用边缘模型；模型服务使用 deadline、限流、熔断和断网回退，不能进入 ROS 2 实时控制周期。

**二阶段验收**：同一 Skill 在 MuJoCo 和真机后端使用同一输入输出 schema；切换 Classical/VLA/RL Provider 不改变 Skill 合约；模型断连、Provider 崩溃和控制权抢占均有可验证的安全结果。

### 三阶段：规模化能力与发布

1. 完善 WorldModel、传感器融合、遥操作、数据采集、回放和策略评估闭环。
2. 建立 E300 LPR/LPP、FIREFLY 等 HyperProfile 和 release manifest；VM、OS、架构、设备和镜像摘要由发布元数据提供，不在代码硬编码。
3. 发布多架构 OCI、SBOM、签名摘要、IDL 兼容报告和回滚包。
4. 完成跨平台契约测试、HIL、故障注入、急停恢复和 48 小时稳定性验证；未完成证据的 BoardProfile 保持 `unverified`。

## 4. 模块协调与接口规则

| 变更 | 必须同步评审 | 验收证据 |
|---|---|---|
| Skill 输入/输出、状态或错误码 | `api/proto`、Skill、Runtime、SDK、文档 | current/previous 兼容测试 |
| ROS 2 控制或 MuJoCo 模型 | ROS2Adapter、MuJoCoAdapter、RobotProfile | 仿真回放、故障注入、状态频率 |
| 新增 Classical/VLA/RL Provider | Provider manifest、ProviderRouter、ControlAuthorityManager | Provider 切换、租约冲突、超时回退 |
| AgentOS 接入 | AgentOSBridge、认证上下文、部署 Profile | E300 提交任务、取消、断连恢复 |
| 真机或板卡能力 | BoardProfile、SafetyPolicy、部署包 | HIL/真机记录、设备断连、急停恢复 |

每次执行必须记录 task、skill、provider、model、controller、RobotProfile、SafetyPolicy 版本/摘要、策略决定、状态迁移、时间戳、故障和 trace ID；不得记录密钥、原始敏感传感器数据或模型权重。

## 5. Definition of Ready / Done

**Ready**：明确目标环境、用户行为、Skill/Profile/IDL 影响、风险等级、成功与失败场景、取消/恢复/接管路径；硬件工作项必须确认 BSP、设备权限和接口所有权。

**Done**：代码、IDL、Profile、文档一致；成功和拒绝/失败路径都有测试；仿真或 HIL 证据已归档；可回放；没有通过 Mock、参数覆盖或旁路绕过安全边界。

## 6. 产品与项目门禁

### 产品门禁

- 用户只需提交 TaskFlow、Skill 参数和 RobotProfile，不需要了解 ROS topic、板卡 SDK 或模型部署细节。
- 第一阶段必须有一条可演示金路径：自然语言任务由 E300 AgentOS 发起，Ubuntu Piper MuJoCo 完成抓取并返回状态。
- 产品能力按“已验证能力”发布；仿真、真机、模型和 VLA 能力分别标识，不能以演示成功替代能力证据。
- 任何模型替换不得改变 Skill 合约和安全上限；模型不可用时必须给出可理解的失败原因和恢复建议。

### 项目门禁

- 当前最大风险不是算法，而是公共 IDL、ROS 2 控制边界和 AgentOSBridge 尚未实现；一阶段不得先扩展 VLA/RL Provider。
- Ubuntu 与 E300 的接口、端口、认证、证书、超时和版本必须集中在 BoardProfile/部署配置中，禁止散落在脚本和文档。
- 每个阶段只在上一阶段验收证据完整后开始；未完成的板卡、真机或 Provider 标记 `unverified`，不得进入发布矩阵。
- 每周至少保留一次成功流程和一次失败/安全流程的回放记录，作为项目状态而非口头汇报依据。

## 7. 当前下一步

编码从一阶段第一个垂直切片开始，顺序固定为：

1. 创建 `api/`、`core/`、`adapters/`、`profiles/`、`skills/`、`tests/` 骨架。
2. 先实现 `RobotProfile`、`SkillGoal/Feedback`、状态机和 `stop/move_joint`。
3. 接入现有 Piper ROS 2/MuJoCo 控制，完成 Ubuntu 本地回放。
4. 实现 AgentOSBridge，打通 E300 Task 提交、状态反馈和取消。
5. 通过一阶段验收后，再实现 `pick/place` 和 Classical/VLA/RL Provider。
