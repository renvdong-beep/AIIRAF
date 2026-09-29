# 宇树本体场景交互仿真设计（机器狗 + 机器人）

**状态**：设计草案 v0.1（**未实现**）
**日期**：2026-09-20
**范围**：需求 2 —— 引入宇树（Unitree）机器人与机器狗，以"场景交互"方式做机器人仿真操作与演示
**依据**：`docs/adr/0004-unitree-go2-edu.md`、`docs/iraf-piper-quadruped-simulation.md`、`docs/iraf-engineering-design.md` §4/§8、`AGENTS.md` 6.8（首期四足固定 Go2 EDU）

---

## 1. 实测资产事实（`unitreerobotics/unitree_mujoco`，浅克隆探测）

```text
仓库：https://github.com/unitreerobotics/unitree_mujoco
HEAD：1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d  2026-09-07  "Merge pull request #131 from keeprobot/main"
顶层：unitree_robots/  simulate/  simulate_python/  example/ros2/  terrain_tool/  doc/
```

`unitree_robots/` 各本体资产文件数（实测）：`a2 195`、`h1_2 93`、`g1 72`、`h1 57`、`r1 45`、`b2w 41`、`b2 37`、`h2 34`、`go2w 28`、`go2 22`、`as2 20`。

MJCF 清单（实测节选）：`go2/go2.xml`、`go2/scene.xml`、`go2w/go2w.xml`、`g1/g1_23dof.xml`、`g1/g1_29dof.xml`、`g1/scene_29dof.xml`、`h1/h1.xml`、`h1_2/…`、`h2/…`、`r1/…`、`b2/scene_terrain.xml`、`a2/qrc_map_flat.xml`（二维码地图）、`terrain_tool/scene.xml`。

`go2/go2.xml` 关键内容（实测）：

- 执行器：12 个 `<motor>`，即 `FR/FL/RR/RL` × `hip/thigh/knee`（`ctrlrange` 分别为 `-23.7 23.7` 与 knee `-45.43 45.43`）；
- 关节默认 `damping=0.1 armature=0.01 frictionloss=0.2`，`option cone=elliptic impratio=100`；
- 传感器：`<site name="imu">`、`jointactuatorfrc` 力矩传感器（含噪声）等；
- **无 `<camera>`、无雷达 sensor**（grep 实测 0 命中）。

`simulate_python/`：`unitree_mujoco.py`、`unitree_sdk2py_bridge.py`（MuJoCo ↔ SDK2/DDS 桥）、`config.py`、`test/test_unitree_sdk2.py`；`example/ros2/`：`stand_go2.cpp`、`motor_crc.{h,cpp}`（CRC 校验的电机命令）。

相关仓库分支（`git ls-remote` 实测）：`unitree_ros2`（`master`、`ros2_service_support`）、`unitree_sdk2`（C++，`main` 及多个 G1 手臂分支）、`unitree_sdk2_python`（`master`、`stand_test`）。

**结论（决定架构的 4 条）**：

1. 厂商 MJCF 提供**本体 + 执行器 + IMU/力矩传感器**，**不提供相机、雷达、托盘、地形道具** → 这些必须由 IRAF 世界/场景构建器按声明注入，完全复用 Piper 侧的"场景生成器 + 声明式几何"模式（`scripts/build_robot_pick_scene.py` 的做法），不得把传感器写死在厂商文件里。
2. 存在 MuJoCo ↔ SDK2 桥（`simulate_python/unitree_sdk2py_bridge.py`），因此"同一 Skill 合约在仿真与真机间切换"有官方路径可走。
3. 官方示例是**低层控制**（`stand_go2.cpp` 直发电机 + CRC）——`AGENTS.md` 6.8 明确：导航、停靠、传感器与交接必须分别验收，禁止用示例或 mock 结果替代真能力证据。
4. **铁律 6.8 只锁定 Go2 EDU 作为首期四足本体**；人形（g1/h1/h1_2/h2/r1）资产虽在库内，但不在首期承诺范围，需要单独 ADR 与验收，否则属于"未经声明的能力宣称"。

---

## 2. 分层与边界（新引入的模块与它不该做的事）

```text
Studio / 语音（见伴随设计文档，后期）
        |
AgentOSBridge（既有，版本化公共服务）
        |
TaskFlow -> SkillRuntime -> PolicyGateway -> CapabilityProvider   ← 既有骨架，本次不改这一层语义
        |
QuadrupedAdapter / HumanoidAdapter（新增契约，平台无关）
        |
UnitreeGo2Adapter / UnitreeHumanoidAdapter（新增，屏蔽厂家 topic/DDS domain/固件/控制模式）
        |
场景运行时：MuJoCo world(scene package) | （真机）unitree_ros2 / SDK2
```

边界规则：

1. `QuadrupedAdapter` / `HumanoidAdapter` **只暴露能力与标准反馈**（站立、停止、速度/位姿、状态、急停确认），不暴露 `FR_hip` 这类厂家关节名、DDS domain、CRC 细节、步态频率。四足关节级步态控制首期不重写（`iraf-piper-quadruped-simulation.md` §1）。
2. 高层运动服务与低层电机控制**不得混用**：切换必须经过既有 `ControlAuthorityManager`，同一执行器任一时刻只有一个控制源（`AGENTS.md` 1.13 的同类原则）。
3. 相机/雷达/托盘/道具属于**场景声明**，不属于适配器；真机与仿真的差异通过 profile 与 adapter 切换，Skill 合约不变。
4. 仿真运行必须在证据里带 `simulation=true`（`AGENTS.md` 1.7），不得表述为真机/实时能力。

---

## 3. 场景包（Scene Package）——"场景交互"的载体

统一目录约定（与 `examples/demo3_arm/` 同风格，放在仓库根 `scenes/<scene_id>/`）：

```text
scenes/handoff_lab/
  scene.yaml              # 场景声明（唯一事实来源）
  baseline.yaml           # 参考姿态/初始状态/求解参数/验收阈值（复用 config 基线模式）
  profile_<robot>.yaml    # 该场景使用本体指向 profiles/ 的哪一份（或直接引用）
  scenario.yaml           # 交互与验收脚本（步骤、判据、故障注入）
  README.md               # 中文：怎么跑、看到什么、失败怎么定位
```

`scene.yaml` 声明内容（全部数据，无代码）：本体（`robot: unitree_go2` / `piper` / `unitree_g1`）、地形/工作台、道具（盒体、托盘）、传感器（相机内外参、雷达、IMU）、光照（复用已落地的声明式光源，见 `docs/debug/2026-09-20-scene-lighting-declaration.md`）、随机化种子、`simulation: true`。

**场景交互的三种形态**（按实现成本递增，逐级启用）：

| 形态 | 交互方式 | 依赖 | 用途 | 状态 |
|---|---|---|---|---|
| S1 命令式交互 | CLI 提示符下选择本体/场景/动作，实时 viewer 观察 | 既有 `view_mujoco.py` 管线 + 场景包 | 现场演示、逐动作排查 | 待实现 |
| S2 脚本化场景 | `scenario.yaml` 声明的步骤序列自动执行并出证据 | `SkillRuntime` 直连（走 Policy/Authority） | 演示回放、回归、CI | 待实现 |
| S3 遥操作/控制权切换 | 键盘/手柄输入映射为速度指令，经 `ControlAuthorityManager` 在两个控制源间切换 | 需要新的 `TeleopProvider` 与控制权仲裁 | 人机交互演示、接管路径验证 | 待实现（风险最高，最后做） |

统一入口（与既有金路径同构，命名先定契约、实现后置）：

```text
scripts/scenario.py list                                   # 列出可用场景包与声明能力
scripts/scenario.py run --scene scenes/handoff_lab --scenario nominal [--viewer]
scripts/scenario.py run --scene scenes/handoff_lab --scenario fault_sensor_loss
scripts/scenario.py interact --scene scenes/handoff_lab     # S1 交互模式
```

**禁止**：为宇树单开一条绕过 `TaskFlow -> SkillRuntime -> Policy` 的"演示直连通道"。演示必须走与验收相同的链路，否则演示结果不构成能力证据（`AGENTS.md` 1.2）。

---

## 4. Skill 与能力合约（沿用 `handoff_lab` 已定义语义，逐项验收）

| Skill | Provider | 成功条件（判据，不是"命令已发出"） | 状态 |
|---|---|---|---|
| `stand` / `stop` | 四足本体 Provider | 躯干高度与姿态稳定 ≥ 声明时长；速度为零；`stop` 有确认反馈 | **已实现（MuJoCo 仿真）** |
| `locomote`（速度指令） | LocomotionProvider（高层运动服务） | 目标速度跟踪误差在声明范围内，超时/超速触发安全停止 | **已实现（MuJoCo 仿真）** |
| `navigate` | Nav2 或厂商运动服务 | 到达位姿且无安全事件（相机/雷达故障时禁止自主导航） | 待实现 |
| `dock_for_handoff` | 四足 | `tray_frame` 平移误差 ≤ 30 mm、偏航 ≤ 2 deg、速度为零 | **已实现（MuJoCo 仿真）**：实测 0.025516453 m / 0.143376133° |
| `accept_payload` | 四足 | 托盘占用/载荷确认（不得只凭"夹爪已张开"） | **已实现（MuJoCo 仿真）**：实测落位间隙 +0.000627866 m（**在模型自身的接触 margin 0.001 m 之内** ⇒ 判"落在承载面上"）、整链末速 0.003442571 m/s（判据 ≤0.01）、载荷中心偏移 0.052586728 m（判据 ≤0.06）；托盘现挂在狗背 `tray_frame`（2026-09-29，见 `docs/debug/2026-09-24-joint-model-dog-arm.md` §11.23(47)） |
| `pick_object` / `place_object` | Piper + MoveIt2 | 既有判据（双指接触 + 抬升 + 命中目标） | **已实现（MuJoCo 仿真）** |

先落地"本体单体"能力（`stand`/`stop`/`locomote`/状态读取），再谈交接集成；顺序不可颠倒。

> **2026-09-28 状态更新**：四足侧能力（`stand`/`stop`/`locomote`/`dock_for_handoff`/`accept_payload`）
> 均已在 MuJoCo 仿真链路验收；联合世界（Go2 + Piper 同一份 MJCF）整链 s01–s05 全绿，
> 证据 `build/acceptance/handoff_lab/nominal/report.json`（`passed=True`、`failed_checks=0`），
> 详见 `docs/debug/2026-09-24-joint-model-dog-arm.md` §11.23(41)(42)。`navigate` 与真机/板级证据仍待交付。

---

## 5. 与 Piper 的交接集成

复用 `docs/iraf-piper-quadruped-simulation.md` 已定义的 `delivery_handoff` TaskFlow（`VERIFY_READY -> DOG_DOCKING -> ARM_PERCEIVE -> ARM_GRASP -> PLACE_IN_TRAY -> DOG_CONFIRM -> DOG_DELIVER`，异常进 `SAFE_HOLD`）。本次不重新设计该状态机，只把载体从"文档"变成"可跑的场景包"。

互锁条件（缺一不可，任一不满足即拒绝 `place_object`）：机器狗速度为零、姿态稳定、驻停确认、`tray_frame` 有效、Piper 工作空间无碰撞、急停未触发。

---

## 6. 人形本体的处理（诚实边界）

- 首期**不承诺**人形能力，原因是 `AGENTS.md` 6.8 锁定 Go2 EDU，且人形涉及手臂/全身控制、平衡与安全策略，属新的安全边界。
- 允许的范围：以 `scene.yaml` 引入人形**模型**做静态场景展示（例如作为被操作对象或环境实体），并在证据里标注"仅模型/渲染，不代表人形运动能力"。
- 若要把人形运动纳入范围，必须先出 ADR（本体选型、控制源、安全策略、验收场景），再实现。这与"不得把 mock 当能力"同一条纪律。

---

## 7. 契约与 IDL 影响

| 变更 | 类型 | 兼容性 |
|---|---|---|
| 新增 `QuadrupedAdapter` / `HumanoidAdapter` 接口 | 新 adapter 契约 | 新能力，需在 `profile_check` 中做 declared ⊆ implemented 校验 |
| 新增 `scenes/<id>/scene.yaml`、`scenario.yaml` schema | 新增声明 | 与 `config/*.yaml` 同层，不进公共 IDL |
| 新增 Skill：`stand`/`stop`（四足）/`locomote`/`navigate`/`dock_for_handoff`/`accept_payload` | Skill manifest 新增 | `api/proto/iraf/v1/skill.proto` 若需新增字段，先改 IDL 再实现（`AGENTS.md` 2.1） |
| 既有 `pick_object`/`visual_pick`/`move_joint` | **不变** | 交接场景复用，不改语义 |

---

## 8. 分阶段与验收（每阶段独立提交、独立证据）

| 阶段 | 内容 | 验收证据（真实产物，禁止 mock） | 主要风险 |
|---|---|---|---|
| U1 | 锁定 `unitree_mujoco` / `unitree_sdk2_python` commit + 许可证 BOM，vendor 到 `vendor/unitree_go2/`（只读，带 `source-lock.json`） | `source-lock.json` + 许可证清单 + 模型哈希 | 许可证逐项审查未完成即不得分发资产 |
| U2 | Go2 loopback：MuJoCo 步进 + `stand`/`stop`/状态读取 smoke（先不接 IRAF） | 躯干高度/姿态/速度曲线与稳定时长数字；无 GUI 也可复现 | 官方示例是低层控制，切勿直接宣称 `navigate` 可用 |
| U3 | 场景构建器注入 camera/lidar/托盘/地形（声明式，厂商文件只读） | 生成 MJCF + 传感器实测图像/点云统计（如雷达点数、相机内参与实际 fovy 误差） | 传感器噪声与仿真真实性差异，需在证据里如实标注 |
| U4 | Adapter + Skill：`stand`/`stop`/`locomote` + 状态反馈，走 `TaskFlow -> SkillRuntime -> Policy` | 成功路径 + 拒绝路径（越界速度、过期状态、无权限）各一份执行证据 | 控制权与安全停机路径 |
| U5 | `navigate` / `dock_for_handoff` / `accept_payload` + 互锁负向测试 | 停靠误差 ≤ 30 mm / ≤ 2 deg、速度为零；互锁拒绝率 100% | 导航栈集成复杂度最高 |
| U6 | `delivery_handoff` 端到端（Piper + Go2）+ S1/S2 交互演示 | 标称交接成功率、故障注入预期终态正确率 100%、非预期运动为 0 | 跨本体时序与坐标一致 |

---

## 9. 决策记录（2026-09-20 已确认）

> 原选项文本保留在下方以便回溯。已确认 7 项决策中与本需求相关的是 4 / 5 / 6；其余子问题标注「未确认」，实施前必须复核，不得当作已确认。

1. **首期本体范围** —— **已确认：B Go2 EDU + 人形仅静态模型/场景实体（2026-09-20）**。
   - 原选项：(A) 只 Go2 EDU（符合铁律 6.8）；(B) Go2 EDU + 人形仅作静态模型/场景实体；(C) Go2 EDU + 人形纳入运动范围（需先出 ADR）。
   - 硬约束：人形（G1/H1/H1-2/H2/R1）在本战役中**只能是静态模型 / 场景实体**：不得声明任何运动、导航或力矩能力，不得接入 `locomote`/`navigate` 类 Skill，任何涉及人形的产物与证据必须显式标注「仅模型」（见 §10）。
2. **四足控制来源** —— **未确认**（不在本轮 7 项决策内）：本战役按建议 A 实施（官方 `simulate_python` 桥 + SDK2 高层运动服务，与 ADR-0004 一致）；若实施中发现与 ADR-0004 冲突，**以 ADR 为准并先补 ADR**，不得默默改向低层电机控制。
   - 原选项：(A) 先用官方 `simulate_python` 桥 + SDK2 高层运动服务；(B) 直接低层电机控制；(C) 只用位移/速度学模型（简化）。
3. **场景包落盘位置** —— **已确认：A 新增 `scenes/<id>/`（2026-09-20）**。
   - 原选项：`scenes/<id>/`（新目录，本体无关）vs 复用 `examples/<robot>_<scene>/`。
   - 落点：与 `examples/`（单机型干跑示例）语义分开；`examples/demo3_arm/` 属另一窗口，**本战役不触碰**。
4. **交互形态优先级** —— **已确认：A 先 S2 脚本化，再 S1 命令式（2026-09-20）**。
   - 原选项：先 S2 脚本化场景（可自动出证据）vs 先 S1 命令式交互（演示直观）。
   - 落点：演示必须复用可回归的路径；S3 遥操作本轮不做。

---

## 10. 本战役硬约束：人形「仅模型」

决策 4.B 在下游按以下方式落地，任何违反即本战役验收失败：

| 允许（本战役） | 禁止（需另行 ADR） |
|---|---|
| 人形 MJCF 作为**静态场景实体**加载（作为障碍物/背景/被观察对象） | 人形任何 `stand`/`locomote`/`navigate` 能力声明或 Skill 注册 |
| 声明中的 `capabilities` 标注为 `static_model_only` | 把「仅模型」产物表述为已具备运动能力（`AGENTS.md` 1.7 / 6.8） |
| 场景包 `scene.yaml` 里登记人形资产（哈希 + 许可证） | 用人形资产冒充 Go2 的能力证据 |

证据标注规则：涉及人形的场景/验收产物必须在报告字段中带 `evidence_level: "仅模型"`（英文键 + 中文取值），并在 `simulation: true` 之外单独标注；缺少该标注的产物不得作为人形相关能力的证据。
