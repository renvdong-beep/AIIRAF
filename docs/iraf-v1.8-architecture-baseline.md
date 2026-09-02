# IRAF v1.8 架构基线

## 图纸

- `iraf-refined-architecture.png`：总体架构图，说明 AgentOS、IRAF 控制面、Provider 与仿真/真机后端的模块边界。
- `iraf-detailed-interface-security.png`：详细接口/安全架构图，说明 Skill Query、Tool/Skill Manager、公共契约、执行状态、幂等、租约和 SafetyEvent。

## 工程边界

IRAF 是 AgentOS 到机器人 Skill 的应用框架，不替代实时控制器、BSP、驱动、现场总线主站或 Hypervisor。AgentOS 只能通过 AgentOSBridge 和版本化公共服务接入；业务请求按 `TaskFlow -> SkillRuntime -> Policy Gateway -> Provider` 的契约边界处理。Provider 分为传统算法控制、VLA 和强化学习等类型，ROS 2/MoveIt2/ros2_control 属于传统算法控制 Provider；MuJoCo 和真机属于后端适配环境。

## 当前基础与实施路线

当前已具备可复用 MuJoCo 仿真代码、ROS 控制代码、Piper 抓取场景闭环和机器人 Profile 统一规约基础。

- 一阶段：在 Ubuntu 22.04 + ROS 2 Humble + MuJoCo 上完成公共契约、Registry、Skill Query、Tool/Skill Manager、TaskFlow、Runtime、Policy 和单体仿真；板卡侧 AgentOS 通过容器运行 aarch64 的 Intewell-Agent RPM。
- 二阶段：完成 classical/VLA/RL 多模式 Provider、交接闭环、HIL 和边缘适配。
- 三阶段：完成 World Model、模型 Provider、遥操作和数据闭环。

所有 Skill 必须声明 schema、能力依赖、前置/成功条件、错误码、超时、取消、恢复、安全等级和版本。仿真必须声明 `simulation=true`，不得把仿真结果表述为真机实时能力。
