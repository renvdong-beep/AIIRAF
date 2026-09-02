# IRAF 竞品与开源能力借鉴分析

**评审日期**：2026-08-28  
**目的**：明确可复用能力、IRAF 自研边界和持续跟踪对象，避免重复建设或被单一厂商技术栈锁定。

## 1. 结论

市场上存在接近 IRAF 的平台，也存在覆盖单项能力的成熟开源项目，但没有一个参考项目同时覆盖本项目要求的 AgentOS 接入、传统控制/VLA 多模式仲裁、Piper 与四足协同、MuJoCo 仿真、异构 Linux/边缘板部署以及可审计安全执行。

IRAF 应采用“集成成熟内核，补齐统一控制面”的策略：ROS 2、MoveIt2、ros2_control、Nav2 和 MuJoCo 作为执行与仿真基础；LeRobot 作为数据集/策略接口的重要参考；Viam 的资源模型、模块注册和设备运维体验值得借鉴；OpenRAL 的快慢策略协同和安全运行时值得跟踪；Isaac ROS、Open-RMF、Genie Sim 作为可选 Provider 或专项能力，而不是核心依赖。

## 2. 对标矩阵

| 项目 | 已有优势 | IRAF 应借鉴 | 不直接照搬的原因 |
|---|---|---|---|
| [Viam](https://docs.viam.com/what-is-viam/) | 声明式硬件资源、统一 API、模块注册、边缘到云的数据和版本化部署 | `RobotProfile`、资源 API、模块生命周期、快速诊断和分批发布体验 | 云控制面和自有运行时不是本项目的实时/安全边界；ROS 2 与国产边缘板仍需 IRAF 适配 |
| [OpenRAL](https://github.com/OpenRAL/openral) | 快策略、慢推理、感知、奖励与传统控制的类型化编排 | Provider 类型、可追踪策略链、VLA 与经典控制协作方式 | 项目较新，成熟度、长期兼容和硬件覆盖需持续验证，暂不作为基线依赖 |
| [MoveIt 2 / MTC](https://github.com/moveit/moveit_task_constructor) | 机械臂规划、碰撞检查、分阶段抓放任务 | Piper 传统控制主路径、确定性执行器、抓放 stage 设计 | 解决操作规划，不负责跨机器人任务治理、模型路由和边缘发布 |
| [LeRobot](https://github.com/huggingface/lerobot) | 统一数据集、机器人/策略接口、ACT/Diffusion/VLA 训练评测和回放 | 数据集映射、策略 Provider SPI、遥操作采集、训练-部署-纠错闭环 | Python/学习策略栈不是安全控制器，也不负责 ROS 2 资源仲裁和多本体事务 |
| [NVIDIA Isaac ROS](https://developer.nvidia.com/isaac/ros) | Jetson/DGX 上的感知流水线和 NITROS 加速 | 作为 NVIDIA 板型的感知/零拷贝 Provider | 不适用于所有 E300/S600/FIREFLY 组合，不能进入公共 IDL 或核心调度语义 |
| [Open-RMF](https://www.open-rmf.org/) | 多机器人任务分派、车队适配和设施协同 | 后续多机器狗任务竞价、车队 Adapter 和场地资源建模 | 首期是单 Piper + 单 Go2 的物品交接，直接引入会扩大系统复杂度 |
| [AgiBot Genie Sim](https://github.com/AgibotTech/genie_sim) | 场景生成、合成数据、自动评测、ROS 2/MoveIt/ros2_control 集成 | 场景资产清单、批量扰动、排行榜式回归指标和 agent-ready 操作流程 | 当前主要面向智元人形和 Isaac/Newton 后端，与 MuJoCo/Go2 基线不同 |
| [Unitree Go2 开源栈](https://github.com/unitreerobotics/unitree_mujoco) | 官方 Go2 MJCF、SDK2/DDS、ROS 2 和 MuJoCo sim-to-real 示例 | 四足模型、消息、仿真/真机一致接口和低层验证 | 官方 MuJoCo 当前主要覆盖低层控制；Nav2、停靠、托盘与任务互锁仍由 IRAF 集成 |

## 3. Build / Reuse / Integrate 决策

| 能力 | 决策 | 首期实现 |
|---|---|---|
| ROS 通信、TF、动作接口 | Reuse | ROS 2 Humble；公共 IRAF IDL 不暴露发行版私有类型 |
| Piper 规划与传统控制 | Reuse + Integrate | MoveIt2、ros2_control、AgileX ROS 2 驱动 |
| Go2 低层仿真与真机通信 | Reuse + Integrate | `unitree_mujoco`、`unitree_ros2`、SDK2，全部锁定 commit |
| 四足导航、停靠和托盘交接 | Integrate | Nav2/定位算法 + `UnitreeGo2Adapter` + IRAF Skill 合约 |
| 学习策略与数据格式 | Integrate | LeRobot 兼容导入/导出，IRAF `ActionProposal` 保持独立 |
| TaskFlow、Provider 路由、控制权仲裁 | Build | IRAF 核心差异化能力 |
| Policy Gateway、安全投影、审计回放 | Build | 本地确定性门禁，不委托给 VLA 或云端服务 |
| 多平台构建与板级发布 | Build + Integrate | OCI、多架构 manifest、BoardProfile、HIL 证据包 |
| GPU 感知加速 | Optional Integrate | NVIDIA 目标可用 Isaac ROS，其他板保持等价 Provider 合约 |

## 4. 产品差异化目标

IRAF 的首个可验证卖点不是“又一个 ROS 封装”，而是让目标用户用同一 TaskFlow 在 MuJoCo、HIL 和真机上切换，并能在传统控制、VLA、学习策略、遥操作和回放之间受控切换。应用开发者只看到 `pick_object`、`dock_for_handoff`、`place_object` 等稳定能力；集成商通过 Profile/Adapter 接入本体；安全负责人可以回放每次策略选择、资源租约、控制权切换和停止确认。

## 5. 持续对标流程

每季度由产品、架构和运控共同更新一次矩阵，记录上游版本、许可证、活跃度、接口变化、已验证能力和替换成本。任何引入都必须先通过许可证审查、固定 commit 的最小 spike、契约测试和退出方案；竞品演示或 README 声明不能直接作为 IRAF 支持证据。
