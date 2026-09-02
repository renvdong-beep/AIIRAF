# ADR-0003：Ubuntu 22.04、ROS 2 Humble 与 MuJoCo 基线

**状态**：已接受（待 M0 spike 验证）  
**日期**：2026-08-28  
**取代**：ADR-0001  
**决策人**：产品负责人、架构负责人、运控负责人（待签名）

## 决策

原生开发与仿真认证基线采用 Ubuntu 22.04 LTS、ROS 2 Humble 和 MuJoCo，覆盖 amd64/arm64。Ubuntu 24.04、CentOS Stream 9 与 openEuler 24.03 LTS 可作为宿主运行同一锁定的 Ubuntu 22.04/Humble OCI 开发环境，执行核心构建、契约测试和无头 MuJoCo 仿真；GUI、GPU、DDS multicast 和设备透传须按宿主单独验证。

Ubuntu 24.04 的原生 ROS 2 路径采用 Jazzy，而不是在 Noble 上拼装 Humble。该路径首期定义为实验兼容档：公共 IDL、TaskFlow、Profile 和非 ROS 核心必须编译测试；`Ros2Adapter`、Piper、Go2 和 MuJoCo 桥必须通过完整契约/场景测试后，才能升级为正式支持。Humble 与 Jazzy 分属独立 workspace/镜像，不允许在一个进程或运行环境中混装二进制依赖。

Piper 采用 `agilexrobotics/agx_arm_ros` ROS 2 分支并锁定 commit。MuJoCo 与 ROS 2 的桥接边界固定为 `SimulationAdapter`，M0 spike 比较并锁定：

1. `ros-controls/mujoco_ros2_control` 在 Humble 上的源码构建与 ros2_control 兼容性；
2. 若不满足，使用 MuJoCo C API 实现最小 `iraf_mujoco_bridge`，只提供 joint state、actuator command、clock、camera/lidar 和场景控制接口。

不得让 TaskFlow、Skill 或业务应用依赖具体桥的 topic、插件或内部类型。

## 理由与后果

ROS 2 Humble 对 Ubuntu 22.04 amd64/arm64 提供 Tier 1 支持，Piper ROS 2 驱动提供 Humble 依赖路径。MuJoCo适合接触动力学、操作策略和 VLA/模仿学习数据闭环，但其 ROS 2 桥不是 MuJoCo 官方稳定 API，因此桥接组件必须锁定来源、commit、许可证和契约测试。

Humble 官方支持期到 2027 年 5 月。项目必须在 2026 年 12 月前完成 24.04/Jazzy 兼容 spike 和迁移 ADR；公共 IRAF IDL 与 Provider 契约不得绑定 Humble，以降低迁移成本。

## M0 验证

1. Ubuntu 22.04 amd64/arm64 安装 ROS 2 Humble、MuJoCo、Piper 模型与 MoveIt2，锁定全部 commit/digest。
2. MuJoCo 无头运行 Piper、眼在手相机、机器狗、相机和雷达，仿真时钟可重复。
3. 传统 ros2_control/MoveIt2 与 VLA/策略 Provider 均通过同一 SkillFeedback/ResourceLease 接口执行。
4. Ubuntu 24.04、CentOS/openEuler 宿主启动相同 Jammy/Humble OCI digest，完成无头场景和 ROS 2 action smoke。
5. Ubuntu 24.04/Jazzy 独立 CI lane 编译公共核心和 Ros2Adapter，输出接口差异与阻塞项，不将其误标为 Humble 原生支持。
6. 输出桥接选型报告、许可证清单、性能基线和 Humble 迁移风险。
