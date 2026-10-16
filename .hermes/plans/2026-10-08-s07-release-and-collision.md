# 计划：s07 释放被写回（已修）+ UR5e 腕部撞狗（待修）+ 停靠偏航缺口（待修）

日期：2026-10-08
HEAD（进入本轮）：`dc95b9a`
状态：s07 一行级修法**已落**（未提交），30 轮通过率批次**进行中**

## 0. 本轮已完成的定位（数值证据）

### 0.1 s07 根因：抬离段把"刚放下的载荷"重新夹住
```
链路（三层都对齐了才成立）：
  构建期：src/iraf_adapters/unitree/scene_builder.py:1979-1983
          key = "place_%s_positions" % phase
          merged = {k: v for k, v in out_gripper["lift_positions"].items() if k in _declared_gripper_keys(...)}
          merged.update(positions)          # positions = 该相位的臂关节解
          ⇒ 夹爪通道取自 **lift_positions = 抓取抬升（闭爪）** 的值
          ⇒ 实测：UR5e 四段都是 rq2f85_fingers_actuator=163.0（闭）
                   Piper 四段都是 joint7=0.023 / joint8=-0.023（也是闭，open 是 ±0.035）
                   （探针 scripts/probe_place_waypoint_gripper.py，三层产物并排）
运行期：src/iraf_adapters/mujoco/mujoco_backend.py:2628
          self._set_gripper_controls(dict(gripper["open_positions"]))   # ③ 释放，**无条件**张开
      原 2632-2636：
          if carry_release_gripper:            # ← 条件挂错：把"搬运期是否张爪"当成"释放后是否保护"
              for name in gripper["open_positions"]: retreat_goal[name] = open 值
      ⇒ UR5e 声明 carry_constraint.release_gripper: false
              （config/ur5_simulation_baseline.yaml:171，语义＝**搬运期间**指腹合着当锚点参照）
        ⇒ 该保护被跳过 ⇒ 撤退段回放把 163 写回 ⇒ 指腹合拢夹住载荷
⇒ 判据 released = 指腹与载荷不再接触 = False ⇒ s07「Backend 未确认载荷已放下」
   实测相位梯（§11.91）：after_touchdown_6 pad_span 0.091302 / finger{False,False}
                        after_retreat      pad_span 0.039260 / left:True
```
**修法（1 行条件删除 + 中文留档）**：抬离段**无条件**把声明的张开值写进 `retreat_goal`。
- 对 Piper（release_gripper=true）：旧代码已经走这段 ⇒ `retreat_goal` 逐位不变 ⇒ **零影响**（预期）
- 对 UR5e：修复

### 0.2 单轮结果（`build/diagnostics/s07fix1.log`，clean，无调试开关）
```
passed_true = 8/9（修前 7/9）    exit=5（仅 s02b）
s07  evidence: place_released=1.0  place_payload_in_tray=1.0
               place_offset_from_tray_center_m=0.03452368（验收 0.065）
               place_settled_gap_m=-0.000512995  place_settled_on_target=1.0
停靠数值与基线**逐位相同**（说明本改动未扰动停靠物理）：
  A 站 0.02856927179873537 / -0.4246947295906039
  B 站 0.028036 / 2.449925°（FAILED：pos 过 0.030、yaw 超 2.000）
s06 evidence: grasp_center_distance_m=0.0020460720192433134
```

## 1. 进行中：30 轮通过率批次
`bash scripts/campaign_pass_rate.sh 30 > build/diagnostics/campaign-fixB-30.log`
- 批跑的验收口径：整链 passed / 逐步骤状态 / s06 残差 min~max / s07 落点 min~max
- **批跑期间冻结源码**（批次逐轮重读 `scripts/scenario.py`）
- 判读：s07 必须 30/30；且 s06 残差 min/max 与停靠数值应与基线一致（说明修法只动 s07 那一段）

## 2. 待办 A：`place_settled_speed_mps` 出现 66~495 m/s（证据可信性缺口）
```
实测分布：
  s04（Piper，臂自身场景）：4.0405e-05  ← 合理
  s07（UR5e，联合世界）：66.427628625（acct-round1/3/4/5/6、align-round10~12 **逐位相同**）
                        0.000210779（acct-round2）
                        494.799241551（本轮 s07fix1）
  历史：above-campaign-round2=0.072073101、round3=0.000340886/7.5231e-05
口径：iraf_adapters/mujoco/payload_facts.py:117-123
      所有 **自由关节**（联合模型里只有两个：狗躯干 + box_01_free）线速度上界
⇒ 15 kg 躯干不可能 66 m/s、载荷躺在托盘里（gap -0.0005、in_tray=1.0）也不可能 ⇒ **不是物理解**
⇒ 首要假设：某处**直接写 qpos 造成"瞬移"**，随后 qvel 由差分得到（0.13 m / dt=0.002 ≈ 66 m/s；
   1.0 m / 0.002 ≈ 495 m/s —— 与两个观测量级吻合）；次假设：读 qvel 与 owner 推步竞争（撕裂读）
   （但 66.427628625 逐位相同 ⇒ 撕裂读可能性低，瞬移假设优先）
⇒ 必须做的**只读**探针（批跑结束后跑）：
   在 s07 settle 那一刻，逐自由关节打印 ‖v‖ 与 qpos/qvel 原值，并记录最近一次
   直接 qpos 写的来源；判"是哪一具本体、由谁瞬移"
   ⚠ 判据不能只看 `place_settled_on_target`（它不含速度）⇒ 若真是瞬移，必须把它写成**证据缺口**
      并在文档里明确"这一项当前不可信"，不得当作"已落稳"的证明

## 3. 待办 B：UR5e 腕部本体撞狗（用户原始需求）
```
运行期实测接触（build/diagnostics/collide1.log）：
  ur5e_wrist_1_link ↔ tray_01   最深 -0.000807 m，n=136，首见 after_above
  ur5e_wrist_1_link ↔ FL_hip    最深 -0.000067 m，n=133，首见 place_descend@step300
  ur5e_wrist_2_link ↔ base_link 最深 -0.000746 m，首见 place_above@step640
构建期自检（联合报告 place_reference）：
  waypoint_contacts = []                      ← 四个航点端点**无**载体接触
  path_contact_count = 192，但取样的 20 条全是 arm ↔ world / place_pad_b，**carrier: False**
⇒ **缺口找到了**：构建期把载体摆在**站位帧的标称位姿**
   （src/iraf_adapters/unitree/scene_builder.py:2067-2091：data.qpos[:]=0 后
     直接把 site_pos/site_quat 写进载体 free joint）
   而运行期狗的实测停靠在 B 站偏离标称 **2.8036 cm / 2.4499°**
   ⇒ 标称位姿下"腕部离托盘还有几毫米"，实测位姿下就变成**亚毫米穿透**
   ⇒ 三类碰撞（tray / FL_hip / base_link）在构建期**结构上看不见**
⇒ 下一步（**先隔离验证再动构建器**，用户纪律）：
   探针 scripts/probe_place_carrier_dock_offset.py：
     取联合模型 → 把载体按**实测停靠位姿**（含平移+偏航）摆放 → 对四个航点 + 段间插值做 FK
     → 报 arm↔carrier 接触对与最小净空；与 collide1.log 的运行期接触对**对账**
   若对账成立 ⇒ 修法＝让构建期自检支持"载体按**声明的最坏停靠偏差**摆放"
     （声明键先落 IDL/schema：如 reference_solver.place_carrier_dock_offset_m/_yaw_deg，
       取值由 `dock_for_handoff.stations.<站>.approach/hold 容差`推出，不写死数字）
   ⇒ 之后才改放置航点**姿态**（现有杠杆：`grasp.place_approach_direction`，
       当前 UR5e **未声明** ⇒ 缺省竖直 [0,0,1]；Piper 显式声明 [0,0,1]）
   已否定的三轮"平移落点垫"实验结果保留在 §11.92（不是充分手段）

## 4. 待办 C：B 站偏航缺口 0.450°
`-2.449924664481998°` 对判据 2.0°（pos 0.028036 已过 0.030）。
确定性（30/30 逐位一致）⇒ 可攻。

**已读出的机理（来自现成探针 `scripts/probe_dock_hold_yaw.py` 的口径说明 + 本会话在
`build/diagnostics/demo2.log` 的 `DOCK_TRACE` 上直接读到的数字）**
```
DOCK_TRACE samples = [elapsed, dx, dy, yaw_err, cmd_vx, cmd_wz, body_yaw, sim_time]
  （列定义见 src/iraf_adapters/unitree/unitree_go2.py:1962-1969）
  发零触发点（首条 cmd_vx==0）：偏航误差 ≈ **-1.92°**（贴着**控制**容差 approach_yaw_tolerance_rad=0.034 rad=1.948°）
  保持窗末态（demo2，带调试开关）：yaw_err = **-0.041768 rad = -2.393°**、平移 0.026114801676794204 m
  ⇒ **归零后漂移 ≈ 0.45~0.50°**（偏航）与 **≈ 0.011 m**（平移）；验收 2.0°/0.030 ⇒ 余量被漂移吃光
```
**候选（按代价从低到高，且**先声明后实现**）**
1. **（声明级，最便宜）按站收紧控制容差**：`config/go2_joint.yaml` 的
   `dock_for_handoff.stations.handoff_b.approach_yaw_tolerance_rad`（现 0.034）与
   `approach_position_tolerance_m`（现 0.020）—— 它们是**控制**容差（控制器瞄多准），**不是验收判据**。
   目标：让发零发生在 ≤1.4°（漂移 0.5° 后仍 < 2.0°）。**必须验证**接近段仍在 `timeout_s` 内完成
   （B 站 reached_s 9.4~10.4 s，接近超时 12 s ⇒ 余量只有 1.6~2.6 s，收紧可能撞超时 ⇒ 必须实测）。
   验收判据 2.0° / 0.030 **一个字不动**。
2. 减小归零后漂移：`halt` 的冻结延迟（实测 0.49 s，`freeze_delay_s`）—— 属控制/停驻机制，改动面更大。
3. 偏航回路阻尼 `wz = gain*err - kd*实测偏航角速度`（改控制律，需先声明增益）。
4. B 站布局（级联 s06 托盘交接）—— 最后才动。

**需要的仪器（批跑结束后 1 次即可）**：`bash scripts/probe_dock_hold_trace.sh`（`IRAF_DEBUG_DOCK=1`）
→ `python3 scripts/probe_dock_hold_yaw.py build/diagnostics/dockhold-<标签>-round*.log`
（注意：带调试开关的运行与 clean 运行**数值不同**，判读机理可以，取"基线数值"不行。）

## 5. 提交计划（Conventional Commits，中文）
1. `fix(mujoco): 抬离段无条件保持声明张开值，修 s07 载荷被判未放下` （+ 本文档 §11.93 + 探针）
2. （30 轮绿后）文档：§11.93 定案与批次数字
3. 待办 A/B/C 各自独立提交

**实际已完成提交（本轮）**
```
bdc09e0 fix(mujoco): 抬离段无条件保持声明张开值，修 s07 载荷被判未放下
2e23190 test(scripts): 放置航点夹爪通道三层并排探针 + 载体停靠偏差净空探针
2a82066 docs(debug): §11.93 s07 真根因与修法 + 30 轮批次证据 + 两个待办
f8fc3f7 test(scripts): 载体停靠/高度偏差净空探针 + 运行期臂关节 FK 对账探针
a21ff67 docs(debug): §11.93附 量化构建期载体高度口径错 0.2766m 与腕部亚毫米擦碰
```

## 8. S2 实测记录（2026-10-08，进行中）
```
[实验 1] 改落点垫无效（已有结论，本轮复核）：腕部**跟着落点走**，外移落点垫不增加与狗的净空。
[实验 2] 把 B 站站位 −y 0.12 m（scene.yaml: handoff_station_frame_b 0.45→0.33），受控重建后实测：
   tray_01 清了：wrist_1↔tray_01 −0.027656 → **+0.043947**
   base_link 清了：wrist_2↔base_link −0.056840 → 未进 margin 0.05 的名单（> 50 mm）
   **但前左髋/大腿冒出来**（横向挪动时"托盘居中、髋偏一侧"响应相反）：
     wrist_1↔FL_hip −0.014323 ｜ wrist_1↔FL_thigh −0.015130 ｜ forearm↔FL_hip −0.010839
   ⇒ 单方向平移会"按下葫芦浮起瓢"，必须用**方向×幅度二维扫描**（探针 --scan，8 方向 × 3 幅度）
      以"最坏载体净空"单值为判据选点（新增打印：`【最坏载体净空】`）。
[口径] 探针的载体位姿必须含**运行期实测躯干高度 0.276627 m**；用构建期现状（z=0）会得到
   "完全没有碰撞"的假清白（这正是 §11.63 之后一直没发现撞腕的原因）。
[待验证] 站位改变后：s02b 停靠偏航（此前实验显示站位改变会动 yaw：−2.393→−2.644/−2.648）、
   s06（从托盘取件，托盘位姿随之改变）、s07 与整链。⇒ 选点后必须跑 clean 单轮 + 连跑。
```

## 7. 待办 B 的执行顺序（2026-10-08 已定）
```
已确证（三条互相独立）：
  ① 构建期载体**高度口径**错 **0.276627 m**（构建期躯干 z=0；运行期实测 0.276627 =
     PLACE_TRACE.tray_top 0.343627 − XML tray 偏移 0.067；并获 stand_height_m 0.27
     与 height_mean_m 0.279953602548388 两处佐证）
  ② 运行期腕部/前臂与狗是**亚毫米擦碰**（wrist_1↔tray_01 −0.000453、wrist_1↔FL_hip +0.000892）
  ③ 标称位姿下探针给的是**负几十毫米**净空（wrist_1↔tray_01 −0.036537 @z=0.276627）
⇒ 步骤（**先仪器、再几何**）：
  S1（仪器，只读探针已就绪）：确立"标称净空"作为**构建期判据**的口径 ——
     载体位姿 = 站位帧 xy/yaw + **声明机身高度**（stand_height_m / 实测）+ 声明的最坏停靠偏差。
     先落 IDL/schema（键名候选 `reference_solver.place_carrier_stance_height_m`、
     `place_carrier_dock_offset_m/_yaw_deg`），取值**由容差推出，不写死**。
  S2（几何候选，逐个试、每个都量净空与 B 站 yaw）：
     a) 声明 `grasp.place_approach_direction`（UR5e **当前未声明** ⇒ 缺省竖直 [0,0,1]）
        加"向基座一侧"的倾角 —— 构建期注释里已记录这是把腕部拉回可达球的杠杆；
     b) 抬高 descand 终点的**保持高度**（`grasp.home_rise_m` / 落点垫位置微调）；
     c) 改 B 站落点垫/站位的相对布局（级联 s06，最后才动）。
     判据：**标称净空 ≥ 停靠容差价（0.030 m）+ 余量**，且 s06/s07 与 B 站 yaw 不退化。
  S3：几何修好后把"放置路径载体接触"从**只告警**升级为**声明驱动的硬门禁**（§11.63 已写明该计划），
     使"构建期绿但运行期撞翻"不再可能。
  ⚠ 每次改声明都要**受控重建**（`build_ur5_baseline.py` → `build_scene.py --attach …` → `scene_check`），
     并用 `scripts/probe_place_carrier_dock_offset.py` 量净空（静态 FK，秒级）。
```

## 9. 待办 C（停靠归零后漂移）——已排除的杠杆与当前实验（2026-10-08）
```
实测结构（config/go2_joint.yaml 自身留档 + §11.86/§11.93附）：
  到位残差 ≈ **控制容差本身**（`approach_command` 一进容差立刻返回精确零）
  归零后额外漂移 ≈ **0.011 m / 0.45~0.50°** = 步态在容差附近的速度地板（0.03955 m/s @0.15 档）× 冻结延迟 0.49 s
  ⇒ 偏航：1.948°(容差) + 0.45°(漂移) = 2.40° > 验收 2.0°；平移：0.0280 + 0.011 也贴 0.030
已排除（都是**实测扫描**，不是推断）：
  · `braking_lead_s`（提前发零）：0.5/0.8 ⇒ 末态 1.415/1.549 m 且 DOCK_HOLD_FAILED（偏航 −14.25°/+31.88°）；
    1.0/1.2/1.5 ⇒ 0.196/0.218/0.285 m（越早发零漂得越远）⇒ 保留 0.0
  · 降低接近速度：0.05 档的速度地板 **0.08178 m/s** 反而比 0.15 档（0.03955）更差
  · `settle_mode`：`frozen_hold` 已是较好档（`damped_hold` 的 stop 路径把机身再推 +15.7 mm）
⇒ 剩下两条结构性路线：
  (i) 让"发零"发生在更靠近目标处 ⇒ **收紧控制容差**（approach_yaw_tolerance_rad 0.034 → 0.026 = 1.49°）
      前提：狗能收敛到 <1.5°（**本轮正在实测**，`build/diagnostics/yawtol1.log`）
  (ii) **冻结期锁偏航**：现在 `hold_pose_world` 只传 (x, y)（`unitree_go2.py` dock 段）⇒ 保持窗内偏航
      不受控，漂移全额落到末态。把偏航并入保持目标是结构性修法（代码）
判据 acceptance（0.03 m / 2.0° / 0.05 m·s⁻¹）**一个字不动**；`approach_*` 是控制容差、可调。
```

## 10. ①（抓取侧重解）实施侦察结果（2026-10-08，只读定位）
```
现象链（实测）：
  运行期 pick 段调用 payload_facts.resolve_grasp_pose_correction(declaration, delta_m)
    （src/iraf_adapters/mujoco/payload_facts.py:371-417，调用点 mujoco_backend.py:1062）
    delta_m = |实测目标 live − 名义解| ⇒ 超 max_correction_m 即 refused=true、调用方显式失败
  站位改动后：live=[0.425852,0.178727,0.371967] vs 名义解=[0.45,0.45,0.375264] ⇒ 0.272365942 m > 0.07
产物证据（build/scenes/handoff_lab/handoff_lab_joint.json）：
  manipulation.per_robot.ur5e.reference_pose_resolution 的子键里**只有 place_reference**、
  **没有 grasp_reference**；place_reference.source = "resolved_for_joint_model"（放置侧按联合模型重解）
  ⇒ **抓取侧没有"按联合模型/实测托盘位姿重解"这一步**（名义解仍是臂自己基线的老站位解）
实施点（建议，先落 schema/契约再改）：
  · scene_builder 里放置侧重解走的是声明的 `reference_solver.<...>place_entry` + 由
    `place_carrier_frame` 推出 `target_local_m`/`transit_local_m`（本会话已读到的同一段代码）；
  · 抓取侧应对称处理：由**实测托盘位姿**（构建期=标称停靠姿态，运行期=live_fk）推出臂基座系下的
    grasp 目标，调用 pick 侧声明的求解器入口重解 home/approach/grasp/lift，并把结果写进
    `manipulation.per_robot.<arm>.home/approach/grasp/lift`（report 里注明 source=resolved_for_joint_model）；
  · 契约侧：`reference_solver` 需要一个**与 place_entry 并列的抓取入口声明**（若已存在则复用，
    先查 `config/scene.schema.json` 的 reference_solver 段再决定是否新增键）。
验收：站位改动后 s06 不再因 max_correction_m 被拒（即名义解随托盘走），且**不改判据、不放宽上限**。
```

## 6. 约束复述（不得违反）
- 判据一个都不动（0.030 / 2.0° / 0.06 / 0.065 / 0.005 全同）
- 探测本身不得改变被观测对象（批跑期间不跑重探针、不改源码）
- 否定结论必须留档；未验证的物理/控制改动一律回滚
- 清理只按 PID；批次完成判据＝末轮日志停更 + 无残留进程
