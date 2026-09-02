# ADR-0004：首期四足平台选择宇树 Go2 EDU

**状态**：已接受（待 M0/M3 spike 验证）  
**日期**：2026-08-28  
**决策人**：产品负责人、架构负责人、运控负责人、硬件负责人（待签名）

## 决策

Piper 物品交接场景的首期四足平台选择 **宇树 Go2 EDU**。仿真采用官方 `unitreerobotics/unitree_mujoco` 的 Go2 MJCF 与 SDK2/DDS 接口，ROS 2 集成采用官方 `unitreerobotics/unitree_ros2`，真机能力只通过 `UnitreeGo2Adapter` 暴露为 IRAF 的标准 `QuadrupedAdapter` 合约。

不选择 AIR/PRO 作为开发基线；采购 SKU 必须书面确认 SDK2/ROS 2 二次开发权限、网络接口和所需传感器访问。智元公开生态继续作为仿真、数据和具身模型参考，但不作为首期四足本体。

## 理由

1. `unitree_ros2` 官方列出 Go2，并推荐 Ubuntu 22.04 + ROS 2 Humble，与项目基线一致。
2. `unitree_mujoco` 提供 Go2 MJCF、C++/Python 仿真和 SDK2/ROS 2 的 sim-to-real 示例，与 MuJoCo 基线直接对齐。
3. 官方开放 SDK2、ROS 2、雷达 SDK 和 L1 雷达 SLAM 资料，能够覆盖本项目的底盘、相机/雷达接入和仿真验证入口。
4. 智元当前公开项目的优势集中在人形、场景生成和具身评测，未形成与 Go2 官方 ROS 2 + MuJoCo 四足链路等价的证据。

## 限制与风险

- 官方 `unitree_mujoco` 当前主要用于低层控制和 sim-to-real 验证，不等于已经提供可验收的 Nav2、停靠、托盘交接或完整传感器仿真。
- Go2 自带运动服务与关闭运动服务后的低层状态行为不同，IRAF 不混用高低层控制权；切换必须经过 `ControlAuthorityManager`、状态确认和安全停机。
- 托盘、载荷、重心、碰撞包络和安装结构属于二次机械设计，真机试验前必须依据厂家限制和实测结果更新 Robot/Safety Profile。
- `unitree_ros2` 与 `unitree_mujoco` 当前为 BSD-3-Clause，但 SDK、固件、模型资产和附加算法仍需逐项形成许可证 BOM，不能由仓库主许可证替代审查。

## 验证门禁

1. 锁定 Go2 EDU SKU、固件、`unitree_sdk2`、`unitree_ros2`、`unitree_mujoco` commit 和许可证清单。
2. 在 Ubuntu 22.04/Humble 上完成 loopback DDS、站立/停止、状态读取、时钟与进程重启 smoke。
3. 在组合 `handoff_lab.xml` 中加入相机、雷达、托盘和碰撞模型，证明不修改 IRAF Skill 合约即可切换仿真/真机 Adapter。
4. 完成 `stand`、`stop`、`navigate`、`dock_for_handoff`、`accept_payload` 的正反向契约测试；未验证的厂家能力不得用 mock 标记通过。
5. HIL 和真机逐项验证急停、通信失联、传感器失效、姿态不稳、载荷偏移和控制模式切换。

## 参考

- [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)
- [Unitree 开源清单](https://www.unitree.com/cn/mobile/opensource/)
- [Unitree 开发者文档](https://support.unitree.com/home/en/developer/)
