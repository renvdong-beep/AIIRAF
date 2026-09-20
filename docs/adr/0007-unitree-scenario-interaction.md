# ADR-0007：宇树场景交互首期范围——Go2 EDU 运动能力，人形仅静态模型

**状态**：已接受（U1~U6 分级验收，**未实现**；命令与场景入口在实现前只是产品契约）
**日期**：2026-09-20
**决策人**：产品负责人、架构负责人、运控负责人、场景/仿真负责人（待签名）

## 决策

宇树（Unitree）本体以「场景交互」方式纳入 IRAF 仿真与演示，首期范围锁定如下：

1. **本体范围**：运动能力只对 **宇树 Go2 EDU** 声明与验收（延续 ADR-0004）。人形资产（`g1` / `h1` / `h1_2` / `h2` / `r1`）在厂商模型库内虽然齐备，但**只能是静态模型 / 场景实体**（障碍物、背景、被观察对象），`capabilities` 标注 `static_model_only`，证据必须带 `evidence_level: "仅模型"`，并在 `simulation: true` 之外单独标注。
2. **禁止项（越界即本战役验收失败）**：人形的任何 `stand` / `locomote` / `navigate` / 力矩能力声明或 Skill 注册；把人形资产用作 Go2 能力的证据；把「仅模型」产物表述为已具备运动能力。
3. **场景载体**：新增 `scenes/<scene_id>/`（`scene.yaml` / `baseline.yaml` / `profile_<robot>.yaml` / `scenario.yaml` / `README.md`），与 `examples/`（单机型干跑示例）语义分开；`scene.yaml` 是场景的唯一事实来源（本体、地形与工作台、道具、传感器、光照、随机化种子、`simulation: true`），**全部是数据，无代码**。
4. **交互形态优先级**：先 **S2 脚本化场景**（`scenario.yaml` 声明的步骤序列自动执行并出证据，可回归、可进 CI），再 **S1 命令式交互**（CLI 提示符 + 实时 viewer）；**S3 遥操作 / 控制权切换本轮不做**。
5. **一律走公共链路**：演示与交互必须复用 `TaskFlow -> SkillRuntime -> PolicyGateway -> CapabilityProvider`，**禁止**为宇树单开一条绕过策略与运行时的「演示直连通道」（`AGENTS.md` 1.2）。
6. **传感器与道具由 IRAF 注入**：厂商 MJCF 保持**只读**，相机 / 雷达 / 托盘 / 地形由场景构建器按声明注入；`QuadrupedAdapter` / `HumanoidAdapter` 只暴露能力与标准反馈，不暴露厂家关节名、DDS domain、CRC 细节与步态频率。

## 理由

1. **`AGENTS.md` 6.8**：首期四足本体固定 Go2 EDU；厂家 SDK/MJCF 示例只证明对应低层能力，导航、停靠、传感器与交接必须分别验收，禁止用 mock 结果替代真能力证据。
2. **安全性**：人形涉及手臂/全身控制、平衡与安全策略，属**新的安全边界**，且会引入新的本体选型、控制源与急停路径，必须另行 ADR 后再实现（`AGENTS.md` 0 节 / 1.13 控制权单一来源原则）。
3. **实测资产事实**（`unitreerobotics/unitree_mujoco`，HEAD `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`，2026-09-07）：
   - `go2/go2.xml` 为 12 个 `<motor>`（力矩型，非位置型），含 `<site name="imu">` 与 `jointactuatorfrc` 力矩传感器；
   - **无 `<camera>`、无雷达 sensor**（grep 实测 0 命中）⇒ 传感器必须由场景构建器注入，厂商文件不能改；
   - 官方 `simulate_python/unitree_sdk2py_bridge.py` 提供 MuJoCo ↔ SDK2/DDS 桥，是「同一 Skill 合约在仿真与真机间切换」的官方路径；
   - 官方 `example/ros2/stand_go2.cpp` 是**低层控制**（直发电机 + CRC）⇒ 不得据此宣称 `navigate` 或高层运动能力可用。
4. **可取证性**：先用可自动出证据的 S2，避免演示路径与验收路径分叉（`AGENTS.md` 2.8、5.2）。

## 限制与风险

| 风险 | 失败表现 | 处理 |
|---|---|---|
| 厂商模型无相机/雷达 | 场景无视觉/点云，误以为「传感器已仿真」 | 由场景构建器按 `scene.yaml` 注入，并发布**实测**图像/点云统计；摄像头内外参与实际 fovy 误差需给数字 |
| 高低层控制混用 | 步态与电机命令互相打断 | 切换必须经既有 `ControlAuthorityManager`；同一执行器任一时刻只有一个控制源 |
| 官方示例被当作能力证据 | 用 `stand_go2.cpp` 宣称 `navigate` 可用 | U5/U6 前不声明 `navigate`/`dock_for_handoff`；示例只证明低层链路 |
| 许可证未审查即分发资产 | 资产分发合规风险 | U1 先出 `source-lock.json` + 许可证 BOM；未完成不得把模型资产入库分发 |
| 人形能力被隐式宣称 | 场景里加载人形即被读成「会走」 | `static_model_only` + `evidence_level: "仅模型"`，缺标注的产物不构成能力证据 |
| 场景包与 `examples/` 混用 | 单机型干跑与多本体场景互相污染 | 目录语义分离（决策 5.A）；`examples/demo3_arm/` 属另一窗口，不得触碰 |
| 仿真被表述为真机能力 | 能力宣称越级 | 所有仿真证据带 `simulation: true`；真机/HIL 证据单独登记（`AGENTS.md` 1.7） |

## 验证门禁

按 U1~U6 分阶段验收，每阶段独立提交、独立证据，**不得越级**：

1. **U1**：锁定 `unitree_mujoco` / `unitree_sdk2_python` commit + 许可证 BOM，只读 vendor 到 `vendor/unitree_go2/`，产出 `source-lock.json` + 许可证清单 + 模型文件哈希。
2. **U2（Go2 loopback）**：MuJoCo 步进 + `stand`/`stop`/状态读取 smoke（先不接 IRAF），给出躯干高度、姿态、速度曲线与稳定时长的**数字**，无 GUI 也可复现。
3. **U3（场景生成）**：场景构建器注入 camera/lidar/托盘/地形，生成 MJCF；给出传感器实测统计（雷达点数、相机内参与实际 fovy 误差）。
4. **U4（Adapter + Skill）**：`stand`/`stop`/`locomote` + 状态反馈走 `TaskFlow -> SkillRuntime -> Policy`，**成功路径与拒绝路径（越界速度、过期状态、无权限）各一份执行证据**。
5. **U5（导航与停靠）**：`navigate` / `dock_for_handoff`：停靠误差 ≤ 30 mm、偏航 ≤ 2 deg、速度为零；`accept_payload` 互锁负向测试**拒绝率 100%**（机器狗速度为零、姿态稳定、驻停确认、`tray_frame` 有效、Piper 工作空间无碰撞、急停未触发，缺一即拒绝）。
6. **U6**：`delivery_handoff` 端到端（Piper + Go2）+ S1/S2 交互演示：标称交接成功率、故障注入预期终态正确率 100%、非预期运动为 0。
7. **人形边界检查**：任何含人形的产物必须带 `evidence_level: "仅模型"`；缺失即该产物不得作为证据，并视为越界。

## 参考

- `docs/adr/0004-unitree-go2-edu.md`（Go2 EDU 选型与风险）
- `docs/iraf-unitree-scenario-interaction-design.md`（§1 实测资产、§3 场景包、§4 Skill 合约、§8 分阶段、§10 人形仅模型硬约束）
- `docs/iraf-piper-quadruped-simulation.md`（`delivery_handoff` 状态机与互锁条件）
- `docs/iraf-engineering-design.md` §4 / §8
- `AGENTS.md` 1.2（禁旁路）、1.7（仿真声明）、1.13（控制权单一来源）、6.8（首期四足固定 Go2 EDU）
