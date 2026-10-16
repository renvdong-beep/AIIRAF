# 机器狗行走/转弯/到点 + 两臂一狗接力的分步计划（需求 2 深水区）

- 制定时间：2026-09-21
- 使用者需求原话：机器狗要能控制行走和转弯，并能运动到指定位置；后续用**两个机械臂 + 一个机器狗**轮番抓取与运输方块
- 前置结论：`docs/adr/0008-quadruped-locomotion-and-navigation.md`（草案，先定安全边界再实现）
- 现有基础（已验收）：
  - `stand` / `stop` 技能（build/acceptance/go2-skills），`stop` 语义目前是 `torque_zero_release`（松力躺倒）
  - 步态控制器**不存在**：`profiles/unitree_go2_mujoco.yaml` 的 capabilities 只有 [stand, stop]，
    `locomote` 目录在但被明确拒绝（实现层也拒绝）
  - 安全策略已预留上限：max_speed_mps 0.5、max_yaw_rate_rad_s 1.0、max_base_translation_m 0.05、max_tilt_deg 10.0
  - S1 交互入口与实时观看已就绪（`scenario.py interact --display interactive_viewer`，桌面可见）
  - 机械臂已接入同一交互入口（config/machines/piper.yaml，A1）

## 阶段与步骤（每步：声明先改 → 实现 → 正/负路径验收 → 中文调试记录 → 单独提交）

### 阶段 1：能走（原地踏步 → 定速直行 → 定速转弯）

| 步 | 内容 | 关键产物 | 验收判据（数值来自声明） |
|---|---|---|---|
| L1 | ADR-0008 定稿 + 停止语义矩阵（`damped_hold` / `torque_zero_release`） | ADR、`profiles/safety/quadruped_lab.yaml` 扩展、`config/go2_loopback.yaml` 的 `locomotion` 段 | 声明自洽门禁：缺任一上限即失败；`stop` 在移动中取 `damped_hold`（负向：混用即失败） |
| L2 | 参数化 trot 控制器（原地踏步）：四腿相位/步高/机身高度/PD 全声明 | `src/iraf_adapters/unitree/gait.py`（或 `iraf_skills/quadruped.py` 内的 Provider）、`config` 新增 `gait` 段 | 10 s 无跌倒；高度波动 ≤ 声明；接触序列正确（按传感器/接触力取证） |
| L3 | `locomote` 技能（速度指令 + 速度斜坡 + 看门狗） | `skills/locomote/skill.yaml`（回填能力）、Provider、`scripts/verify_go2_locomote.py` | vx=0.2 m/s：平均速度误差 ≤ 10%、横向漂移 ≤ 0.10 m/2 m、航向偏差 ≤ 声明；超速/越界被拒 |
| L4 | 到点 `navigate_to`（位姿外环 → 速度内环，单写者） | `skills/navigate_to/skill.yaml`、Provider（或 S2 场景步骤）、`scripts/verify_go2_goto.py` | 位置误差 ≤ 0.10 m、航向误差 ≤ 5°、末速 ≤ 0.05 m/s、安全事件 = 0；负向：不可达/越界目标被拒 |

### 阶段 2：与机械臂交接（两臂一狗接力的最小闭环）

| 步 | 内容 | 验收判据 |
|---|---|---|
| L5 | 场景包扩展：第二台机械臂 + 托盘 + 方块 + 狗的落点；声明式（`scene.yaml`/`baseline.yaml`/`scenario.yaml`） | `scene_check` 通过；两臂一狗均在场景里可寻址；厂商资产只读 |
| L6 | 交接互锁：狗 `speed == 0` 且姿态稳定才允许机械臂放件，反之移动中禁止臂作业 | 互锁拒绝率 100%（负向用例逐条有证据） |
| L7 | S2 端到端场景：`pick → place_in_tray → dog_goto → dog_return → pick`（轮番抓取与运输） | 全流程 SUCCEEDED + 每步判据；失败注入（传感器不可用/超时/看门狗）终态正确 |

## 纪律与边界

1. 每一步都必须先改声明（SafetyPolicy / 机型声明 / Skill manifest），再写实现；能力**仅在验收通过后**回填。
2. 不得用"看起来在动"当验收：速度/位置/倾角/末速都要数字，且阈值只来自声明。
3. `navigate_to` 不是导航栈：不承诺避障、SLAM、地形越障；移动中禁止切换控制源（单写者由租约保证）。
4. 仿真结论一律 `simulation=true`；真机需 `unitree_sdk2` + HIL，另立 ADR。
5. 工作区冻结后再跑最终验收（验收脚本已含脏工作区门禁）；提交只 add 本步声明路径。
