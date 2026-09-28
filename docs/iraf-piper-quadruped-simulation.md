# Piper 机械臂与宇树 Go2 EDU 协同仿真设计

**目标场景**：Piper 固定工位使用眼在手相机识别台面物品，通过传统控制或受约束 VLA/学习策略完成抓取并放入机器狗载具；带相机与雷达的机器狗确认接收、避障导航至投递点。IRAF 负责任务编排、Provider/模式仲裁、安全和可观测性；各机器人确定性控制器负责本体闭环。

## 1. 首期建模选择

Piper 采用锁定的 AgileX ROS 2 模型、夹爪、ros2_control 与 MoveIt2 配置，并派生经校核的 MJCF；机器狗确定采用 **宇树 Go2 EDU**。Go2 仿真从官方 `unitree_mujoco` 的 MJCF、SDK2/DDS 消息和 sim-to-real 示例开始，真机通过官方 `unitree_ros2` 接入。IRAF 仍保留通用 `QuadrupedAdapter` 合约，由 `UnitreeGo2Adapter` 屏蔽厂家 topic、DDS domain、固件和控制模式。

选型依据见 [ADR-0004](adr/0004-unitree-go2-edu.md)。Go2 官方 MuJoCo 当前主要支持低层开发，不能将官方示例等同于 `navigate`、`dock` 或交接能力；这些能力必须由现有运动服务/导航栈与 IRAF Provider 组合并独立验收。四足关节级步态控制首期不重写。

建议仿真后端分两层：

| 层 | 首选 | 作用 |
|---|---|---|
| 功能/集成仿真 | Ubuntu 22.04 + ROS 2 Humble + MuJoCo | Topic/action、接触、传感器、导航、TaskFlow 回归 |
| 策略评测 | 同一 MuJoCo 场景的批量无头运行 | VLA、ACT/Diffusion、传统控制对比与域随机化 |

首期固定 Ubuntu 22.04、ROS 2 Humble、MuJoCo 作为 CI 参考后端；Piper ROS 2 仓库和 MuJoCo/ROS 桥锁定 commit 后接入。详细模式设计见 `iraf-piper-multimode-control.md`。

## 2. 系统与坐标架构

```text
                            AgentOS / IRAF TaskFlow
                                       |
                    delivery_handoff（状态机、恢复与接管）
              +------------------------+------------------------+
              |                                                 |
        Piper Provider Router                          Unitree Go2 Skill Provider
 classical / VLA / learned -> executor            navigate -> dock -> accept -> deliver
              |                                                 |
 eye-in-hand RGB-D / RGB camera                  camera + LiDAR + IMU + locomotion controller
              +------------- ROS 2 / simulation world ----------+
```

统一 TF 树至少包含 `map -> odom -> base_link`（机器狗）、`map -> piper_base_link -> piper_link* -> tool0 -> camera_optical_frame`（机械臂）、`tray_frame`（机器狗接物区）和 `object_<id>`。抓取只接受经标定的 `object_pose`；放置只接受满足 `tray_frame` 可达性、碰撞间隙、机器狗静止和载具空闲条件的目标位姿。

## 3. Capability 与 Skill 合约

| Skill | Provider | 输入 | 成功条件 | 关键失败与恢复 |
|---|---|---|---|---|
| `detect_object` | Piper 感知 | `object_id`、ROI | 置信度/位姿/时间戳合格 | 重拍、重定位、人工确认 |
| `pick_object` | Piper + MoveIt2 | 物体位姿、夹爪策略 | 抓取力/夹爪状态/视觉确认 | 重感知、调整抓取、放弃 |
| `navigate` | 四足 controller + Nav2 | `destination` | 到达位姿且无安全事件 | 重规划、等待、接管 |
| `dock_for_handoff` | 四足 | `dock_pose` | `tray_frame` 误差、稳定站立、速度为零 | 重新对接、停止 |
| `place_object` | Piper + MoveIt2 | `tray_frame` 目标位姿 | 释放、载具视觉/重量确认 | 重新放置、保持、接管 |
| `accept_payload` | 四足 | 期望物品 | 载具状态已锁定/物体确认 | 拒收、重新对接 |
| `stop` / `recover` | 两侧 | 原因、授权 | 安全停机/有限恢复 | 必须审计 |

`delivery_handoff` TaskFlow 的关键状态是：`VERIFY_READY -> DOG_DOCKING -> ARM_PERCEIVE -> ARM_GRASP -> PLACE_IN_TRAY -> DOG_CONFIRM -> DOG_DELIVER`。任一步异常进入 `SAFE_HOLD`；只有在物体状态、双机状态和人工授权满足时才可恢复。机器狗移动期间 Piper 不得执行跨越安全工作空间的动作。

## 4. 感知、标定与安全约束

- **眼在手**：机械臂末端相机负责物品检测、6D 位姿估计、抓取前后复核。必须维护 `tool0 -> camera` 外参，采用手眼标定并将误差、标定版本写入 Robot Profile。
- **机器狗相机**：负责载具占用确认、对接标记/托盘状态与投递点语义，不作为急停的唯一依据。
- **机器狗雷达**：负责建图/定位、障碍物与局部避障。相机和雷达故障分别降级；二者均不可用时禁止自主 `navigate`。
- **交接互锁**：机器狗速度为零、姿态稳定、驻停确认、托盘位姿有效、Piper 工作空间无人/无碰撞、急停未触发，才允许 `place_object`。
- **重量/占用确认**：仿真阶段以托盘碰撞和物体附着状态模拟；真机优先增加载荷/重量或双目视觉确认。单一“夹爪已张开”不能判定交接成功。

## 5. 仿真资产与接口清单

```text
adapters/sim/piper/             # URDF/Xacro、ros2_control、MoveIt2、eye-in-hand camera
adapters/sim/mujoco/            # MJCF、SimulationAdapter、ROS 2 bridge
adapters/robots/unitree_go2/    # unitree_ros2/SDK2、导航、停靠、传感器映射
adapters/sim/unitree_go2/       # 官方 Go2 MJCF 派生场景、camera/lidar/tray
worlds/handoff_lab/             # handoff_lab.xml、托盘、物体、障碍、投递点
profiles/robots/piper_lab.yaml
profiles/robots/unitree_go2_edu.yaml
profiles/safety/handoff_lab.yaml
skills/{detect_object,pick_object,place_object,navigate,dock_for_handoff,accept_payload}/
tests/simulation/handoff/
```

`UnitreeGo2Adapter` 实现通用 `QuadrupedAdapter`，只暴露能力和标准反馈，屏蔽 Unitree topic、DDS domain、步态控制频率、SDK 和固件差异。更换四足本体时，只替换模型、传感器插件、运动 Provider 与 profile，不改 TaskFlow 或 Piper Skill 合约。

## 6. 分阶段验证

1. **机械臂单体**：加载 Piper、眼在手相机与物体，完成相机标定、检测、规划、抓取和放回；注入遮挡、无物体、逆解失败。
2. **Go2 单体**：先用官方 MuJoCo loopback 完成站立、停止、状态读取和 sim-to-real 接口 smoke，再加载相机、雷达、IMU、托盘与已选导航/运动服务，完成定位、避障、停靠、急停和传感器失效降级。
3. **交接集成**：机器狗停靠，Piper 抓取并放入托盘，机器狗确认并离开；验证时间戳、坐标变换、互锁和回放。
4. **故障注入**：网络/模型断开、相机遮挡、雷达失效、物体滑落、对接误差、机器狗失稳、Piper 规划超时、急停。
5. **迁移准备**：每个通过场景保留场景版本、模型摘要、Profile、参数、日志和视频；真机只在 HIL 通过后启用同一 Skill。

## 7. 首期验收

验收先锁定场景包，避免通过改变物体或参数“优化”结果：三种盒状物，质量分别为 0.05、0.15、0.30 kg，最大外形不超过 100 x 80 x 60 mm；五个初始位姿、三条导航路线和固定随机种子。该范围仅是仿真基准，真机范围不得超过 Piper、夹爪和机器狗载荷的厂家/实测限制。

| 指标 | MVP 门槛 |
|---|---|
| 标称交接 | 预热 10 次后执行 50 次，成功率 >= 90%，伪成功为 0 |
| 物体位姿 | 抓取前平移误差 <= 10 mm、旋转误差 <= 3 deg |
| 停靠误差 | `tray_frame` 平移误差 <= 30 mm、偏航 <= 2 deg，且速度为 0 |
| 状态互锁 | 未满足托盘/稳定/安全条件时，`place_object` 拒绝率 100% |
| 取消与安全停止 | Runtime 200 ms 内发出停止请求；仿真控制器 500 ms 内确认，资源保持隔离至 safe state |
| 故障注入 | 每类至少 10 次，预期终态正确率 100%，非预期运动为 0 |
| 可追溯 | 100% 调用可关联 Task/Skill/Provider/Profile/Policy、事件和 replay manifest |

故障类别包括遮挡、误对接、物体缺失/滑落、导航障碍、模型不可用、相机/雷达失联、TF 过期、重复请求、Provider 崩溃和急停。同一 `delivery_handoff` TaskFlow 不依赖特定机器狗 topic；替换合规 `QuadrupedAdapter` 后必须复跑同一契约和场景测试。
