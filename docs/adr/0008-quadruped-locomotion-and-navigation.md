# ADR-0008：四足移动（行走/转弯）与到点导航的能力边界

**状态**：已接受（2026-09-21 定稿；步骤 01 见 `plans/iraf-24h-2/01-*.md`）
定稿只表示**边界与口径已确定**，不表示能力已交付：`locomote` / `navigate_to` 仍须按步骤 03/04
逐项验收通过后才回填 `profiles/unitree_go2_mujoco.yaml`（能力只在验收通过后声明）。
**日期**：2026-09-21（草案 cbd4c02 → 定稿）
**适用**：宇树 Go2 EDU（首期唯一四足本体，铁律 6.8）；仿真 `simulation: true`，真机需另行 HIL 验收
**关联**：ADR-0004（Go2 EDU 选型）、ADR-0007（场景交互范围）、
`profiles/safety/quadruped_lab.yaml`（移动边界与停止语义的唯一事实来源）、
`config/go2_loopback.yaml`（机型声明的 `locomotion` 段：控制实现参数与来源路径）、
`src/iraf_skills/quadruped.py`（门禁实现）、`AGENTS.md` 1.13 / 2.12 / 6.8

## 1. 决策

1. **步态控制源**：自研**参数化对角小跑（trot）控制器**，作为独立 Provider（`iraf_skills.quadruped`），
   参数（步频、步高、占空比、相位、支撑/摆动相、机身高度、PD 增益、力矩上限来源）**全部来自声明**；
   **不**引入 `unitree_sdk2` 的高层运动服务，也**不**使用学习策略（首期）。
   理由：仓库内只有厂商模型资产（`vendor/unitree_go2` 锁的是 22 件模型，不含控制器源码）；
   自研参数化 trot 的每一条物理假设都可声明、可复算、可负向测试。
2. **能力拆分**：`locomote`（速度指令 vx/vy/wz + duration_ms）与 `navigate_to`（目标位姿 x/y/yaw + 容差）
   都经 `TaskFlow → SkillRuntime → PolicyGateway → ControlAuthority → Provider → 适配器`；
   `navigate_to` 的**外环**（位姿闭环）与 `locomote` 的**内环**（速度跟踪）共用同一控制源，
   由 `ControlAuthorityManager` 保证单写者，禁止两个 Provider 同时写控制通道（铁律 1.13 / 2.13）。
3. **停止语义矩阵**（必须显式区分，禁止混用）：

   | 语义 | 取值 | 适用 | 后果 | 落地状态（步骤 01） |
   |---|---|---|---|---|
   | 受控停止 | `damped_hold`（减速到零并保持站立） | 行走/转弯中的 `stop`、看门狗超时、到达终点 | 保持可控、可继续 | 已登记在安全策略 `spec.stop_modes`，门禁 `resolve_stop_mode` 已交付（步骤 02 起被控制器调用） |
   | 失能停机 | `torque_zero_release`（松力） | 急停、安全事件、显式"失能"请求 | 力矩型执行器松力后躺倒（已在报告中显式记录） | 已登记；**急停/安全事件是唯一合法入口** |

   - 移动中的 `stop` **一律** `damped_hold`；`torque_zero_release` 只在急停/安全事件路径使用。
   - **调用方不得自行选择停机语义**：`resolve_stop_mode(是否移动, 急停标志, 机型声明的站立语义)`
     是唯一解析入口；未知模式、未登记模式、以及"非急停路径请求失能停机"一律显式失败
     （`IRAF-QUADRUPED-COMMAND-REJECTED`）。
   - **站立态**停机语义仍是机型声明 `config/go2_loopback.yaml` 的 `stop.mode: torque_zero_release`
     （步骤 15/17 已验收：松力后出现静止段，末速 0.003964950131491022 m/s）。本步**不改变**站立语义，
     以免既有实测数字变化（逐位一致门禁）；如要一并改为 `damped_hold`，须另立步骤并重跑 loopback 验收。
4. **不做**：ROS 2/Nav2 导航栈、SLAM、地形越障、动态避障、人形运动。首期只做**平面直线/原地转弯/到点**。

## 2. 必须声明的安全边界（缺声明即失败，不允许默认值）—— 步骤 01 已全部落地

| 项 | 取值（唯一来源：`profiles/safety/quadruped_lab.yaml` 的 `spec.quadruped_limits`） | 数值口径 / 取法 |
|---|---|---|
| `max_speed_mps` | 0.5 | 与宇树官方 SDK 示例"慢速"档同量级；使用者确认的硬约束 |
| `max_yaw_rate_rad_s` | 1.0（≈57°/s） | 既有声明，保留 |
| `max_accel_mps2` | 0.5 | 推导：0.5 m/s ÷ 机型声明 `locomotion.ramp_s`=1.0 s；两者互锁（自洽门禁 `ramp_s × max_accel ≥ max_speed`） |
| `allow_in_place_turn` | `true`（布尔） | 使用者确认"允许原地转弯"；声明为 false 时纯偏航指令必须被拒（纯偏航 = 线速度为零且有偏航速度） |
| `max_base_translation_m` / `max_tilt_deg` | 0.05 / 10° | **站立语义**，保留（"站着有没有被带走"） |
| `workspace_m`（矩形，绝对坐标） | x ∈ [−0.5, 0.5]、y ∈ [−0.5, 0.5] | 实测（`build/iraf-24h-2/01/probe-workspace.txt`）：台面半边长 0.8 m、Go2 躯干碰撞盒半长 0.1881 m ⇒ 边缘余量 0.30 m > 0.1881 m，另留 0.11 m 给步态摆动与读数延迟；0.5 为保守取整下界 |
| `max_tilt_moving_deg` | 15.0 | **移动语义**（与站立的 10° 分开）；取自本 ADR 草案建议上限；须由步骤 02/03 实测复核——超限时修控制器，**不放宽阈值** |
| `watchdog_timeout_ms` | 100.0 | 推导：10 个控制周期（控制频率 100 Hz 声明于机型声明 `control.frequency_hz`）；与 `locomotion.heartbeat_period_ms`=10 ms 自洽（心跳周期必须 ≤ 超时） |
| 看门狗动作 | `damped_hold`（受控停止） | 声明为失能停机即被拒；状态过期（年龄 > 超时）⇒ **拒绝下发新指令**，不用过期状态继续控制 |
| 资源互锁 | 移动中禁止切换控制源 | 由 `ControlAuthorityManager` 单写者 + 租约 TTL ≥ 机型声明动作时长（`profile_check --quadruped` 门禁 8）保证；机械臂作业期间要求狗 `speed == 0`（二期两臂一狗交接预留） |

**控制实现参数**（不写阈值）：`config/go2_loopback.yaml` 的 `locomotion` 段只声明
`ramp_s` / `control_frequency_source`（点号路径，避免同一事实两处）/ `heartbeat_period_ms` /
`watchdog_action` / `damped_hold.{deceleration_source, hold_pose_source}`；
每个"来源"都必须能在本声明内解析，解析不到即显式失败（`load_locomotion_declaration`）。

## 3. 验收门禁（数值口径，全部来自声明；未达标即 FAILED，不放宽）

| 阶段 | 判据 | 阈值与来源 |
|---|---|---|
| S1 原地踏步 | 四腿接触序列正确、机身高度波动、持续时长无跌倒 | 时长与高度波动按步骤 02 声明；倾角 ≤ `max_tilt_moving_deg`（15°，安全策略），位置 ∈ `workspace_m` |
| S2 定速直行 | 平均速度误差、横向漂移、航向偏差 | 速度误差 ≤ 10%（对比声明 `max_speed_mps` 的目标值）；漂移 ≤ 0.10 m / **1.0 m 行程**；航向 ≤ 5° |
| S3 定速转弯 | 角速度误差、转弯半径/原地转弯一致性 | 角速度误差 ≤ 10%（对比声明 `max_yaw_rate_rad_s`）；原地转弯（`allow_in_place_turn=true`）允许，线速度须保持 ≈ 0 |
| S4 到点 `navigate_to` | 位置误差、航向误差、末速、期间安全事件 | ≤ 0.10 m；≤ 5°；末速 ≤ 0.05 m/s（`stop.speed_tolerance_mps`）；安全事件 = 0 |
| 负向（每条都要有证据） | 超速指令、超加速度指令、越界目标点（落在 `workspace_m` 外）、原地转弯被禁时的纯偏航指令、看门狗超时/状态过期、常规 stop 请求 `torque_zero_release`、无租约调用、终态执行复用 fencing token | 全部被拒并给出错误码 |

**口径修正记录**：草案的 S2 写"漂移 ≤ 0.10 m / **2 m 行程**"，但声明的 `workspace_m` 只有 1.0 m 见方
（由 0.8 m 半边长台面推导），2 m 直行在本工作空间内不可达。**行程按声明工作空间取 1.0 m**，
漂移阈值（0.10 m）保持不变——这是把不可达的判据改成可达的判据，不是放宽精度要求。

## 4. 后果与风险

- 步态控制是本项目迄今最难的一段：接触序列、摩擦与力矩饱和对参数敏感；预期迭代多，
  **不得**用"看起来在动"当验收，必须用上表数值。
- 自研 trot ≠ 厂商运动服务：不许宣称"导航/避障可用"，只承诺"平面定速移动 + 到点"。
- `max_tilt_moving_deg=15°` 与 `max_accel_mps2=0.5` 尚未被真机/实测复核（来源分别是本 ADR 建议值与
  控制预算推导），步骤 02/03 必须给出实测数值；若实测超限，**修控制器**。
- 全部结论仅适用 `simulation: true`；真机需 `unitree_sdk2` + HIL 验收（另立 ADR/证据）。
  目标端/板卡验收 DEFERRED（板卡不在场），不得用 dry-run/mock 冒充。
