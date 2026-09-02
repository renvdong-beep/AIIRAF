# ADR-0001：开发与仿真平台基线

**状态**：已废弃，由 ADR-0003 取代  
**日期**：2026-08-28  
**决策人**：架构负责人、运控负责人（待签名）

## 决策

原生参考环境采用 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic，覆盖 amd64/arm64。CentOS Stream 9 与 openEuler 24.03 LTS 使用同一锁定 OCI 开发环境执行核心构建、契约测试和无头仿真；不承诺 ROS 2/Gazebo 原生包或 GUI/GPU 行为与 Ubuntu 等价。

Piper 采用 `agilexrobotics/agx_arm_ros` ROS 2 分支，M3 前锁定 commit、依赖与许可证。机器狗型号在完成 URDF、ROS 2 controller、传感器、负载和许可证评估后由新 ADR 决定。

> 2026-08-28：项目决定采用 Ubuntu 22.04、ROS 2 Humble 和 MuJoCo，本 ADR 不再作为实施依据。

## 理由与后果

ROS 2 Jazzy 对 Ubuntu 24.04 amd64/arm64 提供 Tier 1 支持，Gazebo Harmonic 是其推荐组合；Piper 当前 ROS 2 驱动包含 Jazzy 安装路径。容器边界使 CentOS/openEuler 能复用同一工具链，但宿主内核、GPU、DDS multicast 和设备映射仍须独立测试。

## M0 验证

1. 三类宿主启动同 digest Dev Container，执行 `buf build`、C++/Python smoke test。
2. Ubuntu 原生和三个宿主容器运行无头 Gazebo world 与 ROS 2 pub/sub/action smoke。
3. amd64/arm64 构建通过；不具备 arm64 runner 时交叉构建只标记 build-only。
4. Piper URDF、MoveIt2 demo、夹爪和眼在手相机模型验证后，才能把该 commit 标记为 supported。
