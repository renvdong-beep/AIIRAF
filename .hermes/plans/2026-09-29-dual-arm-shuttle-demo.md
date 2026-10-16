# 计划：Piper + UR5e 双臂轮转 + Go2 穿梭运输（演示级）

状态：已侦察，待执行。起点 HEAD `b8eb22e`（main = piper_go2 = origin 同一提交）。

## 0. 目标（使用者原话）

"Piper + UR5e 机械臂进行交替抓取放置和卸载，Go2 轮流走动到 2 个机械臂下，进行方块运输的演示"。

⇒ 一条**两站流水线**：A 臂把方块装进狗背托盘 → 狗走到 B 站位 → B 臂把方块从托盘取下（卸载）
→ （可选）狗走回 A 站，循环。

## 1. 侦察结论（已实测，决定可行性）

| 项 | 结论 |
|---|---|
| 联合构建器是否支持多臂 | **支持**：`_attach_robots(..., attached_ids)` 收列表、`--attach` 可重复（`action="append"`） |
| UR5e 底层资产 | **齐全**：`build/models/ur5e_2f85/ur5e_2f85.xml`（21 KB，含 source-lock）+ `scripts/assemble_ur5e_2f85.py` |
| UR5e 基线声明 | **形状与 Piper 同构**：`model.{source,arm_joints,finger_geoms,bodies}`、`grasp.{solver,approach_direction,pregrasp_*,tip_clearance_m}`、`target.{id,half_size_m}`、`acceptance.*` 全在；**缺** `place_*`/`regrasp`/`carry_constraint` |
| UR5e Profile | 有 `spec.gripper.{open,closed,pad_offset_m,left/right_index}`、`spec.manipulation.targets`、`capabilities: [move_joint, pick_object, visual_pick, stop]`；**缺** `spec.model.*`（联合构建所需：`position_gain` / `self_collision_excludes` / `finger_friction`） |
| 通用求解器 | **已存在**：`scripts/build_robot_baseline.py` 的 `build_reference_poses / derive_grasp_for_target / raised_home_pose`（机型无关，`build_ur5_baseline.py` 只是转发）；**缺口**：其契约是 `(root, baseline, target_id=None)`，而场景 `reference_solver` 要求 `entry(root, baseline, target_xy_override_m, target_z_override_m)` ⇒ 需要**契约适配**（把 Piper 侧已有的两个 override 参数补到通用入口，属复用而非移植） |
| 停靠站位 | 现只有 `handoff_station_frame`（world-fixed site，`dock_for_handoff` 按帧名停靠）⇒ 加第二个帧即可，机制不变 |
| 场景包 | `scenes/` 下只有 `handoff_lab`；`reference_solver` 是按场景声明的 |

## 2. 增量（每步独立可验收，先"看得见"再严格）

**I1｜UR5e 单臂进联合世界（看得见）** —— 状态：**构建已通（2026-09-29）**，运行期未验收
- 已落地：
  · 契约适配（通用 `build_reference_poses` 支持 `target_xy_override_m` / `target_z_override_m` + 声明化前馈）；
  · UR5e Profile 补 `spec.model.*`（`position_gain` / `finger_friction` / `derived_from_report` / `model.home`
    从装配模型 keyframe 导出）；基线补 `gravity_feedforward`；
  · 场景追加 `robots[ur5e]`（placement `[0.45, 0.45, 0.0]`、偏航 270°、`manipulation_report`、`reference_solver`）；
  · `config/machines/ur5e.yaml` + `ur5e_joint.yaml`；`baseline.robots/robots_joint/initial_state` 三处映射。
- **验收实测**：`--attach piper --attach ur5e` **EXIT=0**（nq=48 / nv=46 / nu=27）；
  `scene_check --require-model` **EXIT=0**；piper-only 产物与 HEAD 版构建器**逐字节一致**
  （sha256 `e04e708c…`，diff 0 行）⇒ 构建器改动对单臂惰性。
- **未完**：见 I1b。
- 口径更正：I1 原写"Piper 行为逐位不变"——实测**运行数字不可能逐位不变**（同产物连跑 s01 末速
  `1.4887881241839877e-05` vs `1.4887880813600727e-05`）。无损只能对**产物**断言（已照此办）。

**I1b｜联合报告按本体给 manipulation 段（I2/I3 的前置）** —— 未开工
- 现状（已显式登记，不是静默）：顶层 `manipulation/gripper/targets/name_map` 只描述主臂
  （`scene.model.joint_manipulator: piper`）；ur5e 在 `manipulation.attached_manipulators[]` 里
  `resolved=false` / `name_map_included=false`。
- 要做的：把 `_joint_manipulation` 抽成**按本体**调用（declared_names + prefix + 参考姿态求解器 +
  place_targets 都按本体给），报告给出 `manipulation.per_robot.<id>`，`name_map` 按本体分节；
  ur5e 的参考姿态求解器已在声明里（通用入口 + `config/ur5_simulation_baseline.yaml`）。
- 验收：`ur5e` 的 `resolved=true`；两条臂的 `name_map` 各自能解析到联合模型里的对象；piper 侧不变。

**I1c｜联合世界稳定性量化（另案，本轮只交付工具）** —— 进行中
- 新见失败模式（**比已知的"末速超限"更严重**）：s04 报"Backend 未确认载荷已放下"，
  s05 报载荷最低点 `-0.000215511 m`、承载面 `0.343222481 m`、接触 geom `[]` ⇒ **方块掉到台面**。
  该次为 piper-only 且模型与 HEAD 逐字节一致 ⇒ 非构建器引入。
- 工具：`scripts/joint_stability_probe.py --runs N`（按**进程退出码**判定，归档失败模式与数值范围）。
- **关键观察（2026-09-29）**：同一份双臂产物 5 连跑进行中 —— 前 3 轮 run1 通过 / run2 通过 /
  **run3 失败**，而 run3 的失败终态与上面那次 piper-only 失败的终态**逐位相同**
  （载荷最低点 `-0.000215511 m`、同一句 `Backend 未确认载荷已放下`）
  ⇒ 失败是**确定性终态**（掉在台面的同一处），但**触发是时序相关**的：
  realtime 演示模式下 guest 每步推进的植物步数在 210~660 之间波动（§11.23(43) 已实测）。
- 下一步诊断（便宜且能定性）：同一产物在 **realtime=false** 下的通过率对照
  （需要一条"批量/CI 用"的机型声明；注意 `robots_joint` 的键集合必须与 `scene.robots[].id` 完全一致，
  所以不能简单地往 `robots_joint` 里再加一条 —— 得用别的方式给出 override）。
- **根因已找到并修掉（2026-09-29，§11.25）**：不是时序，是**附加本体执行器的关键帧 ctrl = 0**
  ⇒ 位置执行器把臂驱到零位、一上电大幅摆动 ⇒ 扰动共享植物。Piper 零位在臂上方所以一直没暴露。
  修法：按子模型 actuator 声明给 ctrl（`_declared_actuator_ctrl`）+ 长度自检 + 逐执行器留痕。
- **复测结果（修复后 3 连跑）**：2/3 = 66.7%，失败模式仍是
  `s04_place_in_tray::require_release`（方块掉到台面），失败终态与修复前**逐位相同**
  （载荷最低点 `-0.000215511 m`）⇒ **ctrl 修复消掉的是"整株失稳"这一类，没消掉"掉件"这一类**。
  合并样本 6/8 = 75%。
- **下一步（必须先补证据）**：失败轮的 s04/s05 `evidence` 为空 ⇒ 先在失败路径带出
  载荷落点 xy / 释放瞬间托盘位姿与相对偏移 / 落稳间隙，再谈机制与修法
  （样本已存 build/diagnostics/report-postfix-run2-fail.json）。
- 这一步**挡住 I3/I4**：卸载要在托盘里抓活体，而"货物在运输/放置后未必还在托盘"这点必须先解决。

**I1c-2｜掉件机制判死（进行中，2026-09-29）**
- 现状：合并样本 6/8 = 75%，失败全部是 `s04_place_in_tray::require_release`，失败终态逐位相同
  （载荷最低点 `-0.000215511 m` = 落在台面上）⇒ **终态确定、触发不确定**。
- 取证手段：`IRAF_DEBUG_PLACE=1` 的 `PLACE_TRACE` 在**执行过程中**打印（不看最终成败）
  ⇒ 6 连跑采集（`build/diagnostics/place-trace-6runs.json` + 逐轮日志）。
- 要抽取的量（每轮都要，通过轮作对照）：
  ① 释放那一刻的**载荷中心 xy 与最低点 z**（相对托盘中心与承载面）；
  ② 载荷相对**夹口（指腹中点）**的横向/竖向偏移；
  ③ `payload_contacts`（载荷与谁接触：指腹/托盘/台面）在各段的演变；
  ④ 释放后的**落稳间隙**与末速；
  ⑤ 载荷最终落点 xy（判"是掉在托盘外侧的哪一边"）。
- 假设清单（必须能被上面的量区分，测完即判死或否掉）：
  H1 释放瞬间方块**蹭到托盘壁**（托盘壁高 vs 载荷半高不匹配）⇒ 载荷被推向一侧翻出；
  H2 释放瞬间载荷带**残余速度**（settle 窗口不够）⇒ 滑过托盘边缘；
  H3 载荷在**搬运段就已经偏出**夹口中心（offset 0.038~0.054 的变化量）⇒ 放到托盘时半边悬空；
  H4 托盘（挂狗背）在放置时的**实测位姿**与名义位姿有偏差（狗漂移/tray 安装面）⇒ 名义落点落在边缘。
- 判据（用于判死）：能给出"哪一段、哪一刻、载荷相对托盘的位置与接触"的实测序列，
  并指出 H1–H4 中**唯一**能被数据支持的那一条；不能给就是"仍未判死"，不许写结论。
- 修法方向（等判死后再定，先记录候选）：提高托盘壁/加导角、加大放置间隙与落稳窗、
  在放置前加一次"载荷居中"纠偏、或收紧托盘位姿的容差门禁。

**I2｜第二个停靠站位 + 狗穿梭** —— **已交付并验收（2026-09-29）**
- 机制：站位注册表（声明 `dock_for_handoff.stations` + `default_station`）+ 步骤按**站名**选择
  （技能输入只多一个 `station` 键；解析在纯函数 `dock.resolve_dock_station`，9 项单测）。
- 几何（全部实测）：B 站 (0.45, 0.45, 0.0)；UR5e 基座 (0.45, 0.45)→(0.45, 0.90)、yaw 270°→90°；
  UR5e 在托盘高度可达环带 r ∈ [0.023982, 1.051524] m、方位 36/36；B 站距 Piper 基座 0.90 m > 0.594284。
- 验收（`--world joint` 7/7 全绿）：s02b 平移 0.022742246608613046 / 偏航 0.664181922862879；
  s05b 到 B 站后载荷仍在托盘 1.0 / 偏移 0.032778818；s01–s05 数字见 §11.25。

**I2 设计依据（留档，已按此落地）**
- **B 站位置**：`handoff_station_frame_b` = **(0.45, 0.45, 0.0)**，期望偏航与 A 站一致（`[1,0,0,0]`）
  ⇒ 托盘/方块在 B 站的朝向与 A 站相同（卸载抓取姿态与 A 站可比，避免多一个 90° 变量）。
  依据：
    ① 在四足安全策略的 `quadruped_limits.workspace_m`（x,y ∈ [−0.5, 0.5]）**之内**（这是 A 站从
       0.55 改到 0.45 的那条硬约束）；
    ② 距第二台臂基座 0.45 m（与 A 站距 Piper 基座 0.45 m 同值）；
    ③ 第二台臂在托盘高度的可达环带 **r ∈ [0.023982, 1.051524] m、方位覆盖 36/36**
       （`scripts/probe_ur5_reach_band.py`，20000 次 FK、只按位置不约束姿态 —— 与 Piper 侧
       r ∈ [0.002028, 0.594284] 同一口径）⇒ 0.45 m 有大量余量；
    ④ 距 **Piper 基座 0.90 m > 0.594284 m** ⇒ A 臂够不到 B 站（角色分离：装=A、卸=B）；
    ⑤ 躯干半长 0.1881 m + 基座半宽 0.09 m < 0.45 m ⇒ 停靠时不与第二台臂基座干涉。
- **第二台臂基座**：由 (0.45, 0.45) 移到 **(0.45, 0.90)**，偏航由 270° 改为 **90°**。
  两条理由：
    · 站位移到 B 后，臂必须仍在"自己站位"的 0.45 m 处 ⇒ 基座沿 +y 让出走廊（狗走 x=0.45, y∈[0,0.45]）；
    · **修正一处我上轮写错的声明**：厂商模型在 `shoulder_pan=0` 时前伸方向是 **−x**
      （`scripts/probe_ur5_workspace.py` 实测 pinch=[−0.492,−0.134,0.488]、"指向与 −z 的 cos"=1.0000），
      要让它正面朝 −y 的 B 站 ⇒ yaw = **90°**（我上轮写的 270° 是背对站位）。
      注：UR5e 的 `shoulder_pan` 覆盖全周（36/36 方位），所以 yaw 只影响"名义正面"与关节取值分布，
      不影响可达性；但声明与意图必须一致。
- 验收：新增步骤"从 A 走到 B 并停靠"，判据复用 A 站口径（平移 ≤0.03 m、偏航 ≤2°、
  `max_speed_m_s` ≤0.05），且 A 站既有验收数字不回归。

**I3｜UR5e 抓取（先自抓自放，再谈交接）** —— **进行中（2026-09-30）**
- 先在 B 站旁的表位让 UR5e 抓一个方块（与 Piper 同链路：TaskFlow→Skill→Policy→Provider→Adapter）
- 验收：UR5e `pick_object` 的 `lift_delta_m ≥ 0.02`、双侧接触、无伪造成功

**I3 进度（已完成的部分）**
- `per_robot[ur5e]` 按本体解参考姿态（`resolved=True`、`name_map` 47 条）✓
- 联合报告的"本场景目标" `joint_scene_target_m/_quat_wxyz` + 步骤来源 `grasp_pose_from: joint_scene_target` ✓
- `s06_unload_at_b` 已进 `scenario.yaml`（ur5e / pick_object / 判据 `pose_tolerance_m 0.005`、
  `min_lift_delta_m 0.02`、双侧接触）✓
- **前馈通道对账**（§11.26）：装配期被 `gravity_feedforward` 键口径挡死 ⇒ 构建期按
  `actuator_trnid` 统一到位置指令通道名，`BUILD_EXIT=0`、8 项单测绿 ⇒ s06 **真的执行了** ✓

**I3 当前阻塞（§11.27/§11.28，已量化，待定案）**
- `distance=0.011073 m > 0.005`；`delta_m=[-0.0034982, -0.003944493, +0.009737022]`
  ⇒ 载荷被搬动后**运行期位姿 ≠ 构建期标称位姿**（托盘实际承载面比标称高 ~9.74 mm，横向 ~5.3 mm，
  且逐轮不同：前一轮 0.009506 m）。
- 已定案并落地 **A**：`grasp_pose.pose_source: live_target_body`（契约先行）+ 后端按目标体执行时刻位姿解析。
- 已挖出并修掉四处静默失败（§11.28）：`name_map` 跨本体混用、夹爪几何不走 `name_map`、
  **home 参考姿态压在托盘载荷里 31.6 mm**（`home_rise_m` 0.30 在本场景不可达，残差 4.233e-01 m）、
  构建期只查 grasp 相位且载荷摆在道具初始位 ⇒ 新增**逐相位残差门禁 + 逐相位侵入检查**
  + 场景侧 `baseline_overrides`（声明 `home_rise_m: 0.10`，实测边界 0.10~0.15 之间）。
- **剩余一条（下一步）**：关节解仍是**构建期**按标称位姿解出的，载荷被搬动 ~16 mm ⇒ 运行期
  `末端未到达目标抓取位姿 distance=0.015812m` ⇒ 需**运行期按真值重解参考关节解**
  （复用 `iraf_core.kinematics.solve_position_ik` + 已声明的 `reference_solver` 契约；
  新增声明式开关 + 证据字段，契约先行）。

**I3 → I4 之间新增（2026-09-30 定案）**
- 卸载落点 = **A（B 站旁台面）**：UR5e 把方块从托盘取下后就近放到台面，先证明"能取下来"。

**I4｜完整轮转（装 → 运 → 卸）**
- Piper：抓取 → 放进狗背托盘（已有链路 ✓，s03/s04 升级为两站点之一）
- 狗：从 A 站走到 B 站（托盘在位）
- UR5e：**从托盘取下方块**（卸载）——**这是最难的一块**：现在 `pick_object` 的抓取位姿是**构建期**按方块声明位姿解的，而托盘里的方块是**运动载体上的活体** ⇒ 需要"名义停靠位姿 + 运行期纠偏"的**反向**用法（与放置段同机制，但用于抓取；`place_pose_correction` 已有三条 fail-closed 自证可复用）
- 验收：四段判据（装/运/卸/复核）全绿；连跑 `stability_probe.py 8` 通过率 8/8

## 3. 已知风险与纪律

- **能力口径**：UR5e 侧一旦启用焊接约束（`require_friction_lift: false`），必须在证据里保留"纯摩擦下的实测拖拽量"，不得宣称夹爪独立完成抬升（与 Piper 侧同纪律）。
- **不做跨机型特例**：UR5e 的一切差异只能进 `profiles/`、`config/`、`scenes/*/scene.yaml`；核心逻辑不得出现机型分支（若需要，先重构接口）。
- **每个增量都要**：先改契约/声明 → 再改实现 → 跑判据 → 数字进文档（§11.23 续号）+ 单独提交。
- **HIL 与真机**：本计划全部结论为 `simulation=true` 仿真证据。

## I4（下一阶段）：卸载落点 + 轮转编排 —— 声明缺口清单（2026-09-30 拟定）

**当前状态（实测）**：`nominal --world joint` 已能整链 8/8（含 s06 卸载抓取）。
s06 的确认实参（`PICK_RESULT`）：`lift_delta=0.078034 m`（判据 ≥0.02）、`center_distance=0.002875 m`
（判据 0.005）、`bilateral=True`、`force_ok=True`。**卸载后"放到 B 站旁台面"尚未接上**（你定的 A 方案）。

**I4-a 卸载落点（A：B 站旁台面）**
1. **声明缺口**（先补齐，均为声明层，不动代码分支）：
   - `scenes/handoff_lab/scene.yaml`：为 ur5e 增加**可放置目标**（台面），几何/位姿由声明给出；
     选点约束（与既有实测同口径）：① 落在 ur5e（基座 (0.45,0.90)、yaw 90°）的**可达**域内；
     ② 不侵入狗的走廊（狗走 x=0.45、y∈[0,0.45]）；③ 台面高度与 B 站托盘高度差不要太大（减少下降行程）。
   - `config/ur5_simulation_baseline.yaml`：补 `grasp.place_entry`（`build_place_reference_poses`）、
     `grasp.carry_gripper`、`grasp.place_pose_correction`（与 piper 同结构的声明；
     数值按 ur5e 自己的实测分布定，**不照抄 piper**）、`acceptance` 相关阈值；
   - `scripts/build_robot_baseline.py`：若缺 `build_place_reference_poses` 入口则新增（共享实现优先，
     避免第二份口径 —— 与 §11.31 的前馈入口同一做法：转发而非复制）。
2. **步骤与判据**（沿用既有判据词表，不新造口径）：
   - `s07_place_at_b_table`：`place_object`，robot=ur5e，判据 `require_release=true` +
     落点判据（台面中心偏移上限按**台面尺寸 − 载荷半宽**推出，与 s04 的 0.06 同法）+ `max_speed_m_s`；
   - 失败必须显式（不静默照放）——既有 `touchdown`/横向闭环与三条 fail-closed 自证直接复用。
3. **验收**：连跑 ≥6 轮，s07 通过率与 s04 同级（当前 s04 横向纠偏量 8 轮分布极差仅 0.87 mm）。

**I4-b 轮转编排（装 → 运 → 卸 → 回）**
- 在 `scenario.yaml` 追加：`s08_return`（狗从 B 回 A，判据复用 s02b 口径）与**第二轮**装运卸（验证可重复）；
- 轮转的**控制权**必须逐段显式交接（§1.13）：Piper 抓取时 UR5e 不动作、UR5e 卸载时 Piper 不动作
  （现状已由 `capability`/策略与执行器作用域保证，需在场景注释里写明并加一条拒绝路径测试）。

**风险与纪律（沿用）**
- 任何新声明先改 `api/` 或 schema，再改实现（契约先行）；判据与验收口径**不得**因为"跑不过"而放宽；
- 内部停止条件（闭环容差/守卫阈值）必须由**实测分布或验收域**反推（本会话 §11.44~§11.46 的做法）；
- 每步完成后：数字进 `docs/debug/2026-09-24-joint-model-dog-arm.md`，并按需同步架构图/ADR（§6.6）。

### I4-a 候选落点的实测依据（2026-09-30 补）

- 可达带（`probe_ur5_reach_band.py --model build/models/ur5-pick-scene.xml`，与 Piper 侧同口径：工具点=双侧 pad box 中点、只按位置筛、关节范围取自模型自身 `jnt_range`）：
  - z=0.025 m（载荷置于台面时的中心高）：**r ∈ [0.042868, 1.027868] m**，方位 36/36
  - z=0.050 m：r ∈ [0.018021, 1.048068] m，方位 36/36；z=0.075 m：r ∈ [0.002676, 1.027228] m，方位 36/36
- 候选落点（载荷中心）**世界 (0.70, 0.60, 0.025)**：距 UR5e 基座 (0.45,0.90) **r = 0.390512 m** ⇒ 落在上述可达带内；
  在台面内（|x|,|y| ≤ 0.8）；距狗走廊（x=0.45、y∈[0,0.45]）x 向 0.250 m、y 向 0.150 m；距狗身中心（半长 ~0.19）净距 ≈ 0.102 m
- 声明化时该点写进 `scene.yaml`（**不写进代码**），并在报告里留证 `pose_source`

- **候选落点已证明可解**（2026-09-30）：(0.70,0.60,0.025) ⇒ 基座系 (−0.3,−0.25,0.025)、r=0.390512484；四相位残差 6.5e-08~9.8e-08；姿态偏差 0.007306°/0.006877°（限 3°）⇒ pass=True。证据 build/diagnostics/ur5e-place-point-feasibility.json；探针 scripts/probe_ur5e_place_point_feasibility.py

### I4-a 声明缺口的**键级清单**（2026-09-30 只读分析；campaign 期间不动产物）

**ur5e 侧缺失（对照 piper 侧已声明项）**

| 位置 | piper 侧（已声明） | ur5e 侧 | 处置 |
|---|---|---|---|
| `scenes/handoff_lab/scene.yaml: robots[piper].reference_solver.place_entry` | `build_place_reference_poses` | **缺失** | 新增；实现走**共享转发**（同 §11.31 前馈入口做法，避免第二份口径） |
| `scripts/build_robot_baseline.py` | — | **无 place 入口函数** | 补入口（先尝试转发 piper 的 `build_place_reference_poses`；若其含 piper 专属几何则抽到 core） |
| `config/ur5_simulation_baseline.yaml: grasp.carry_gripper` | `hold` | 缺失 | 先镜像 `hold`（2F-85 腱驱动同理：搬运段按实测 ctrl 保持，避免持续合拢挤出载荷），再按实测验证 |
| `…: grasp.lift_gripper` | `closed` | 缺失 | 先镜像 |
| `…: grasp.place_settle_ms` | `3000` | 缺失 | 先镜像，再按 ur5e 的落稳实测重定 |
| `…: grasp.place_pose_correction`（mode/tolerances/step/iters） | `touchdown` + lateral 0.035 / step 0.3 / iters 10 / residual 0.001 | 缺失 | **镜像结构**，数值先取 piper 的，落地后按 ur5e 自己的实测分布重定（§11.44 的做法） |
| `…: grasp.carry_constraint`（weld 名/锚点/节拍上限） | `box_01_lift_constraint` / `grasp_anchor` | 缺失（**lift 侧的 weld 已启用**：`require_friction_lift: false` ⇒ `lift_constraint=box_01_lift_constraint`、`lift_anchor_body=grasp_anchor`） | 搬运段是否需要该约束按实测（若掉件再加） |
| `scenes/.../scene.yaml` 的可放置目标 | piper 用托盘（`mount` 挂在狗背上） | **ur5e 无目标** | 声明**世界固定**接收体（构建器已支持：`_place_targets` 不再要求 `mount`）⇒ 台面点 **(0.70, 0.60, 0.025)**（§11.48 已证明可解 + 裕度研究） |

**顺序（重建一次到位，避免反复）**：② scene 声明放置目标 + schema → ③ ur5e 基线补上表各项 →
④ 构建器补 place 入口（转发）→ ⑤ 新增 `s07_place_at_b_table`（判据沿用：require_release +
落点偏移上限（按台面尺寸 − 载荷半宽推出）+ max_speed）→ ⑥ 重建三份产物 + scene_check → 连跑 ≥6 轮。

**注意**：`scenes/handoff_lab/scene.yaml` 第 123-124 行的既有注释已写明"没有 place_entry（该基线未声明
放置段）⇒ 联合报告的 place_targets 不由它提供"——本清单就是把这句话补上。
