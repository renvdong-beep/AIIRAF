# AgentOS 机器人应用框架对话纪要

**日期**：2026-08-27  
**主题**：面向智控域、运控域与通信主站域的机器人应用框架设计

## 讨论结论

1. 当前机器人系统采用分层混合架构：传统控制解决实时、安全和物理可执行性；学习策略增强技能适应性；VLM/VLA/LLM 用于任务理解、技能选择和失败重规划。
2. AgentOS 是通用 Agent 运行底座，负责调度、记忆、工具、存储、权限、治理和模型调用编排；机器人应用框架在其上提供 TaskFlow、Skill、Capability、Robot Profile 和执行闭环。
3. 执行底座按三个域分工：
   - 智控域：AI Agent、VLA、世界模型、任务规划；
   - 运控域：ROS 2、AGIROS、导航、操作、WBC/MPC、Skill Runtime；
   - 通信主站域：EtherCAT、AUTBUS、设备聚合、周期通信和主站诊断。
4. 世界模型不仅是环境地图，还维护机器人状态、环境对象与关系、任务进度和动作结果预测。
5. 应用由 TaskFlow 编排 Skill。Skill 是可注册、可发现、可执行、可取消、可恢复、可评测的能力单元，不等同于单个模型或 ROS 节点。
6. Skill 参数分为任务输入、执行偏好、技能配置和安全边界。外部应用只能在 schema、权限和安全策略约束下传递可覆盖参数。
7. L0 实时能力不经过大模型推理，采用状态估计、局部规划、PID/LQR/MPC/WBC、阻抗/导纳控制、看门狗和硬件急停等确定性控制机制。
8. Ubuntu 应提供真实核心逻辑的 AgentOS Dev Runtime，而不是只返回固定结果的 Mock；仅替换 Hyper 分域、实时主站和真机驱动为仿真/HIL 适配器。
9. Ubuntu 可通过局域网调用边缘端摩尔线程推理服务。远程推理用于任务理解、语义感知和重规划，不进入避障、轨迹跟踪、力控及设备周期等实时闭环。
10. 可借鉴 IB-Robot 的任务编排、设备能力封装、技能路由、状态反馈和数据闭环；具体代码复用需在取得其源码、许可和接口说明后单独评审。

## 已交付资料

- `agentos机器人应用框架-更新版.svg`：更新后的总体架构图。
- `agentos-robot-application-framework-design.md`：完整设计说明。
- `agentos-robot-application-framework-design.docx`：完整设计说明的 Word 版本。

## 后续建议

1. 固定 IRAF 的 SDK、IDL 与 Skill 合约。
2. 先完成 `navigate`、`pick_object`、`place_object`、`stop`、`recover` 的最小 Skill 集。
3. 交付容器化 AgentOS Dev Runtime 和 MuJoCo 仿真适配器（当前工程决策已统一为 MuJoCo）。
4. 建立 Ubuntu 与边缘端的 SDK/TaskFlow/Skill 契约测试和故障注入测试。
5. 在 IB-Robot 资料明确后形成模块映射与复用边界清单。
