# 计划：`dock_for_handoff`（狗走到交接停靠位）—— 联合场景的第一块

状态：**计划（未实现）**。前置：`locomote` 已在三层验收通过并声明（2026-09-23）。

## 0. 契约已经在场景里（无需重新发明）

`scenes/handoff_lab/scenario.yaml` 的 `nominal.s02_dock`：

    action: dock_for_handoff    robot: unitree_go2    params: {}      pending_closed_by: 步骤 18
    criteria: { translation_error_max_m: 0.03, yaw_error_max_deg: 2.0, max_speed_m_s: 0.05 }

目标帧来自 `scenes/handoff_lab/scene.yaml`：`props: tray_01`（`kind: tray`，`frame: tray_frame`），
注释写明「停靠判据（平移 ≤30mm、偏航 ≤2deg）针对 tray_frame，见 baseline.yaml 的 acceptance 段」。
⇒ **判据与目标帧都是既有声明**，实现层不得写第二份数字。

## 1. 要新增的声明（缺一即显式失败，全部集中在版本化配置）

| 键（拟） | 含义 | 放哪 | 依据 |
|---|---|---|---|
| `dock_for_handoff.<profile>.target_frame` | 停靠目标帧名（本场景 `tray_frame`） | 场景/机器人配置 | 场景已有 `tray_frame`；名字不硬编码进代码 |
| `dock_for_handoff.standoff_m` | 目标帧的**前向偏置**（停在托盘前方多远） | 同上 | 需实测：由托盘几何与臂展决定 |
| `dock_for_handoff.approach_speed_mps` | 接近速度（受安全策略上限约束，只能收紧） | 同上 | 取 min(本键, 安全策略 max_speed_mps) |
| `dock_for_handoff.period_s` / `timeout_s` | 控制周期与总超时（超时即显式失败） | 同上 | 与 `control.frequency_hz` 自洽 |
| `dock_for_handoff.settle_s` | 到位后保持时长（用于判据 `max_speed_m_s`） | 同上 | 判据要"末速 ≤0.05"，需一个保持窗 |
| `dock_for_handoff.rel_tolerance` | 何时停走：pose 误差 < rel × 判据阈值 即进入保持 | 同上 | 避免"越调越抖"；停下才谈末速 |

## 2. 分步实现（每步一个提交，都能单独验证）

1. **适配器能力 `dock_for_handoff`**（`iraf_adapters/unitree/`）：
   读目标帧位姿（`mj_name2id` + `xpos`/`xmat`）→ 用 `locomote` 按声明速度接近 →
   误差进入 `rel_tolerance` 后 `stop` 并保持 `settle_s` → 报告 `{final_translation_error_m,
   final_yaw_error_deg, final_speed_mps, control_cycles, timeout, target_frame}`。
   失败路径：目标帧不存在、超时、误差不收敛（显式失败，不返回"近似成功"）。
2. **Skill 三件套** `skills/dock_for_handoff/{skill.yaml,input.json,output.json}`：
   输入 = 可选 `target_frame` 覆盖（默认取声明）；输出 evidence 含上面四量与 `simulation: true`。
3. **Provider**（`iraf_skills/quadruped.py`）：与 `StandProvider` 同构（`_evidence` 校验必需键）。
4. **适配器 `IMPLEMENTED_CAPABILITIES` + Profile + 场景清单**三处同步加 `dock_for_handoff`
   （**清单第三处**：`scene.yaml` 的 `robots[].capabilities` —— 已在本轮因 locomote 踩过一次）。
5. **场景层**：删掉 `s02_dock` 的 `pending_closed_by`，跑 `nominal`（预期 s01/s02/s03 过、
   s04/s05 仍 pending 且被显式跳过）。

## 3. 三层验收判据（缺一层都不算完成）

| 层 | 入口 | 判据 |
|---|---|---|
| 适配器 | 新脚本 `scripts/verify_go2_dock.py` | 平移 ≤0.03 m、偏航 ≤2.0°、末速 ≤0.05 m/s；超时/不收敛必须显式失败（负向用例各一条） |
| 技能层 | `scenario.py interact` 的 `dock_for_handoff` 命令 | 输出满足 `dock_for_handoff.output.json`；未声明能力仍被拒 |
| 场景层 | `scenario.py run --scenario nominal` | `s02_dock` 出现在**已执行步骤**里（不在 `pending_steps`），报告 `passed` 与基线一致 |
| 回归 | loopback 三基准 + 四工况 + stand_stop/fault 场景 | 逐位不变 / 全过（证明没污染既有路径） |

## 5. 核实结果（2026-09-23 落计划时逐条查过，避免下一步重新摸索）

| 依赖 | 核实 | 结果 |
|---|---|---|
| 目标帧在场景声明里 | `scene.yaml` `props: tray_01`（box 0.12×0.08×0.01、0.35 kg、friction "1.4 0.03 0.001"），`frame: tray_frame` | ✅ |
| 目标帧在**已构建模型**里 | `build/scenes/handoff_lab/handoff_lab.xml` 命中 `tray_frame` ×1、`tray_01` ×4 | ✅（实现可直接 `mj_name2id`） |
| 场景级判据 | `baseline.yaml` acceptance：`dock_translation_error_max_m: 0.03`、`dock_yaw_error_max_deg: 2.0`、`min_stable_hold_s: 3.0`、`max_speed_m_s: 0.0`、`timeout_s: 180.0` | ✅ 存在 |
| 步骤级判据 | `scenario.yaml` `s02_dock.criteria`：平移 ≤0.03、偏航 ≤2.0、**末速 ≤0.05** | ✅ 存在 |
| 现有 dock 实现 | `src/iraf_skills/`、`src/iraf_adapters/` 里 grep `dock` **零命中** | ✅ 从零做（计划有效） |

⚠ **发现一处判据双源，必须先对齐（否则会出现"两处不一致时听谁的"）**：
场景级 `baseline.yaml` 写 `max_speed_m_s: 0.0`，而步骤级 `scenario.yaml s02_dock.criteria` 写
`max_speed_m_s: 0.05`。二者对"停稳"的口径不同。处置（实现前必须定，且只留一处为准）：
建议以**步骤级**为准（它是判据的最近消费者），并把 `baseline.yaml` 的值改注释指向步骤级，
或在 `scene.yaml`/`baseline.yaml` 里显式写明"场景级 = 默认值、步骤级可覆盖"的覆盖关系；
不允许实现层自行二选一。

## 7. 第一轮实现后的实测与两处待改（2026-09-23 晚）

已实现：适配器 `dock_for_handoff`（声明段 `config/go2_loopback.yaml` 的 `dock_for_handoff`；
判据由调用方传入；`command_provider` 逐拍按实测位姿误差生成指令；失败判定在 provider 内转成
`MpcUnavailableError(DECISION_DAMPED_HOLD, …)` ⇒ 由 `locomote` 记账终态 —— **本方法不回写终态**，
实测在外层再 `finish` 一次会被框架的 `TerminalExecutionError` 拦下）。

实测（`build/iraf-a6a13/dock_measure.py`，场景模型 handoff_lab.xml，判据 0.03 m / 2.0° / 0.05 m/s）：

    settled_at_s = **0.0**（托盘就在出生点旁 ⇒ 一开始就在容差内，接近过程被"跳过"）
    末态 平移误差 **0.001148 m** ✅（判据 0.03）｜偏航误差 **0.0°** ✅（判据 2.0）
    **failure = damped_hold**：「保持窗 1.020 s 内末速 **0.063591 m/s** > 判据 0.050000 m/s（未停稳）」

⇒ 两处必须改（都属设计层，不是调参）：

1. **到位后要交给静态保持**：`max_speed_m_s: 0.05` 按**瞬时速度**量，而"零速指令的 trot"仍让机身
   在原地以 5~20 cm/s 抖动（原地踏步的固有振荡 ⇒ 该判据在 trot 下不可达）。正解：到达后切到
   **stand 控制器**保持 `settle_s`（其实测 |v| = 4.5e-4 m/s，远小于 0.05）—— 这样"停稳"才是事实。
2. **判据要真的被"考"到**：托盘与出生点几乎同位 ⇒ 接近过程没发生。需要 `standoff_m`：
   停靠点取"目标帧 + 前向偏置"，并选一个**需要走路**的初始距离（否则验收测的是"站着不动"）。
   该项在计划第 1 节已列为待标定声明键。

⇒ 在两处改完并跑出"走一段 → 停稳 → 报末态"的证据之前，**`dock_for_handoff` 不得进 capability 清单**。

## 8. 保持原语：三种都试过、都失败（2026-09-23 晚，同一 `dock_measure.py`）

到位后的"停稳"我按顺序试了三种，全部有数字、全部显式失败（**没有**把它们当成功）：

| 候选 | 保持窗内速度 | 结论 |
|---|---|---|
| ① 交给**中步态位形**（把交接瞬间的实测关节角当保持目标） | 最大 **0.3117 m/s** | 那是在"保持一个迈步中的位形"（可能一条腿在空中），不是站稳 |
| ② 直接交给 `stand`（声明站立位形，四足落地） | 含过渡 0.3585、**稳态段 0.5030 m/s** | 交接时是迈步态 ⇒ 站立控制在"抢救"而不是站稳 |
| ③ 项目已声明的 `damped_hold`（受控停止：减速到零并保持站立） | 末速 **0.9716 m/s**，`static_entered=false` | 报「未在声明预算内进入静止段」⇒ FAILED（正确失败） |

**结构性结论**：缺口不在"停靠算法"，而在**项目还没有「trot → 站定」这条声明化的步态退出/站点保持原语**。
`locomote` 的指令归零后，`gait_in_place` 仍在原地踏步 ⇒ 狗狗**停在迈步中途**（某条腿在空中），
任何后续停止原语都从坏状态起步（三种候选的失败数字正是这个坏状态的三种表现）。
loopback 的 `stop` 之所以"过"，是因为它走的是 `torque_zero_release`（松力 ⇒ 声明语义就是塌陷）——
那是"停机"不是"站定"，不能拿来当停靠的保持。

⇒ 下一轮第一件事（按证据）：**设计并声明化"步态退出/站点保持"**（候选：相位感知的停步——等到
四足落地的相位窗口再交接；或按声明在停止前把落足点收到中立位再切静态保持），判据：
① 从 trot 过渡到静止段的 |v| ≤ 声明阈值；② 过渡期间不摔倒（倾角 ≤ 声明）；③ 过渡后位姿不漂出容差。
原语就绪后，停靠的保持段改用它；那之后 `dock_for_handoff` 才谈三层验收与能力声明。

## 9. 「步态退出/站点保持」的前提已验证（2026-09-23 晚，用已有逐拍数据，未跑新仿真）

设计候选（§8 的下一步）是**相位感知停步**：等四足都真正落地的那一刻再交接。这个前提先被量了：

    理论窗口（声明推算：duty=0.6、相位偏移 FR/RL=0.0、FL/RR=0.5）
      offset 0 的腿支撑相 u∈[0,0.6)、offset 0.5 的腿支撑相 (u+0.5) mod 1 ∈[0,0.6)
      ⇒ 四足同时落地 = u ∈ [0.5,0.6) ∪ [0,0.1) ⇒ 每周期两段、各 10% = 33.3 ms（周期 333.3 ms）
    实测（build/iraf-a6a9/hold-dense2.json 的 leg_force_n 逐拍序列，零指令 hold）
      四足同时落地的起点出现在 u = 0.050 / 0.560 / 0.070 / 0.090 / 0.570 —— **全部落在理论窗口内**
      逐拍占比 6/85 = 7.1%（低于理论 20%：落地/离地瞬间的力仍在阈值附近）
      ⇒ 有效窗口约 2~3 拍/段（100 Hz）、每周期两段 ⇒ 等待期望 ≈ 0.15 s ⇒ **可行**

⇒ 实现约束（下一轮照此做，不许临时发明数字）：
  1. 触发判据用**已声明的** `gait.verification.contact_force_threshold_n`（与 B1 支撑集同一判别量），
     不得在实现层写阈值（本节的 5.0 N 只是分析时的粗筛，不进代码）；
  2. 交接动作 = 冻结步态相位（把目标锁在"四足落地"的位形）并交给静态保持，而不是让腿从空中砸下来；
  3. 判据：过渡段到静止段 |v| ≤ 声明阈值、过渡期间倾角 ≤ 声明、过渡后位姿不漂出容差。

## 6. 诚实边界

- 全部是 MuJoCo 仿真；**上板（A7）证据另计**。
- 停靠只保证"走到声明位姿并停住"，**不保证**与臂的物理交接（载荷确认是 `accept_payload`，
  属下一步）；也不保证动态避障（本场景无障碍物）。
- `standoff_m` 与 `approach_speed_mps` 若要标定，必须在文档里写明是**扫出来的标定值**，
  并给出扫描范围与判据（照 `command_bias_mps` 的先例）。

## 10. 停靠证据作废 + 门禁落地（2026-09-24，进行中）

**重大更正**：`build/iraf-a6a13/dock-measure.json` 的「36 µm / 偏航 0.0°」**不是停靠精度**。
`scene.yaml` 把 `tray_01` 托盘**挂在四足背上**（`pose.mount.entity: unitree_go2`），
而 `dock_for_handoff.target_frame: tray_frame` 就在躯干上 ⇒ 目标帧与机身**同一刚体**：

    偏航误差恒 0（同一 xmat，逐位相同）
    平移误差 = |R·(0,0,0.057)|_xy（帧的竖直偏置在机身倾斜下的投影）—— 探针复算 3.634612751e-05
    settled_at_s = 0.0 ⇒ 接近回路从未执行；61.5 s 全部零指令

⇒ 上一轮「适配器层达标」的表述作废；`s02_dock` **继续待交付**（它本来就该待交付）。
详见 `docs/debug/2026-09-24-dock-target-self-frame.md`（含可复跑探针与全套数字）。

**本轮已落地**（不需授权，属适配器层缺陷修复）：
  1. 门禁 `dock.assert_target_is_world_fixed`：目标帧所属 body ∈ 机器人子树 ⇒ **执行前**显式失败
     （实测：真实场景模型上 0.001 s 失败、零物理步进）；报告增 `target_frame_body` /
     `target_frame_world_fixed`；
  2. 单测 6 项；删掉恒 `None` 且无消费者的 `settle_completed_at_s`。

**下一步需要授权（跨界：场景契约 + 构建器 + 臂可达范围）**：
  新增**世界固定**的交接站位帧（例 `handoff_station` / `handoff_station_frame`，位置由
  Piper 可达范围定），`dock_for_handoff.target_frame` 指向它，并要求：
  ① `settled_at_s > 0`；② 起点到站位有可测位移（`standoff_m` 真正生效）；
  ③ 平移/偏航/末速按步骤级 criteria；④ 全程 `simulation=true`。

**同一轮新增待办（D2，已量化、未修）**：`halt_at_stance` 在 `command_provider` 路径下
没有冻结相位（61.5 s 零指令后仍是 trot 力型、偏航 0.0007°→4.9650°）；且 `halted_at_s`
写进 `state_holder` 却**没进报告** ⇒ 先补可见性再修语义（冻结需同时作用于 QP 接触表）。
（**2026-09-24 收尾已修并提交 `db1287b`**：真机制是"冻结只作用于形状目标、QP 相位未冻结"，
抽出 `phase_elapsed` 让两者共用；末拍偏航 0.6917°→0.0449°、|v| 4.166e-02→2.1846e-04 m/s。
另：`static_entered` 不在 `damped_hold` 报告里却被两处当判据 ⇒ 假失败，统一到 `stop_verdict`。）

## 11. 更上游的阻塞：联合仿真根本没有"狗 + 臂"这一个模型（2026-09-24 只读核查）

站位帧（§10 的下一步）要满足"臂够得着狗背上的托盘"，于是先核查"臂在哪"。核查结果：

```
build/scenes/handoff_lab/handoff_lab.xml   首行 <mujoco model="go2">、body 数 19
                                           （= Go2 躯干子树 18 + tray_01；**没有任何 piper 体**）
scripts/build_scene.py                     接口是 --robot <场景内某一个 id> ⇒ 每次只装配一个本体
build/scenes/handoff_lab/handoff_lab.json  manipulation_absent_reason =
    "本机机型为四足（kind=quadruped），无夹爪与抓取技能：target_id/gripper/vision 为 null，
     manipulation 段不参与运行期装配"
臂自己的产物流                              build/models/piper-pick-scene.xml（另一份模型，2026-09-20）
```

⇒ 三件事同时成立，缺一不可地解释了当前进度：
1. **"臂把方块放进狗背上的托盘"在当前产物里结构上不可能发生** —— 不是缺 Skill，是缺**模型装配**
   （两个本体各自一个 MJCF，且 `robots` 里的 `piper` 未进入四足那份模型）；
2. 因此 §10 的**站位帧坐标现在定不下来**：它取决于"臂在联合模型里的位置/朝向/相对台面关系"，
   而这些事实目前**没有声明**；凭空写一组坐标就是造数字（违反铁律 5.3）；
3. `nominal` 的 `s03_pick` 报"piper 属另一编辑窗口、未声明 robot.backend 绑定"，
   与第 1 条是同一个根因的两个表象。

**决策点（需授权，三选一，我推荐 A）**：

* **A（推荐，物理可交接）**：联合场景扩展为**一个 MJCF 双本体模型** —— 场景声明里新增"本体间
  相对位姿"（臂基座相对台面/狗站位的声明字段），构建器支持多本体装配并断言两本体的位姿与
  台面几何自洽。代价：场景契约 + 构建器 + schema 变更；收益：交接（放块入托盘）能在**同一个
  求解器**里发生，真接触、真摩擦，不需要任何"假装"。
* **B（两个模型/两个进程 + 声明的相对变换）**：交接用声明的接口完成（臂把方块放到世界位姿 ⇒
  声明式"贴附"到托盘）。代价小；但"物理交接"是被**简化**的，必须在文档与判据里显式声明
  这一简化，且不得表述为真实交接能力。
* **C（先各自验收、几何耦合）**：狗停靠到声明站位（世界固定帧）、臂把方块放到该站位的托盘
  投影范围内，耦合只按**几何**判（托盘footprint vs 方块），不模拟接触。最省事，但"轮番抓运"
  的物理闭环仍未成立。

另：这份核查也解释了为什么 `s02_dock` 的世界固定站位帧**必须和 A/B/C 一起定** ——
站位要么是"臂够得着的位置"（依赖臂的摆放，属 A/B），要么是"几何投影范围内的位置"（属 C）。

### 11.1 站位/摆位的量化约束（只读 FK 采样，`build/iraf-a6a14/reach_probe.py` 可复跑）

用厂商 MJCF 直接采样关节限位内的位形（20000 次 FK，工具点取 `link6`），得到臂在**各高度的
可达水平环带**（相对臂基座；臂基座若在台面上则 z 即离台面高度）：

```
关节限位（模型事实）: j1 [−2.618, +2.168]  j2 [0, 3.14]  j3 [−2.967, 0]
                      j4 [−1.745, +1.745]  j5 [−1.22, +1.22]  j6 [−2.0944, +2.0944]
全局: r_max = 0.627429 m；工具点 z 范围 [−0.245397, +0.750233] m
分层（|z−目标| ≤ 0.02）：
  z=0.05  r ∈ [0.006832, 0.623693]      z=0.30  r ∈ [0.011554, 0.605776]
  z=0.20  r ∈ [0.021056, 0.623445]      z=0.34  r ∈ [0.002028, 0.594284]
  z=0.40  r ∈ [0.001307, 0.568411]      z=0.50  r ∈ [0.005713, 0.512210]
```

狗站立时的托盘（按场景模型事实换算，不猜）：

```
托盘底面 z = 0.27995360254838803（loopback 实测 base_link 高度）+ 0.057（tray_01 局部 z 偏置）
           = 0.336954 m；顶面 z = 0.346954 m（台面顶面 z = 0.0）
⇒ 需覆盖的高度层 ≈ 0.347 m ⇒ 该层可达环带 r ∈ [0.002028, 0.594284] m
⇒ 约束：托盘**中心**（或托盘外缘任一点）到臂基座的水平距离必须 < 0.594 m；
        托盘 0.24×0.16 m 的外缘也在环带内的可用余量 = 0.472 m
对照（既有抓取目标）: box_01 顶面 z=0.050、r=0.190 ⇒ 落在该层 [0.006832, 0.623693] 内 ✅
```

**诚实边界**：上述只按**位置**可达筛选，**未约束工具姿态**（"把方块放进托盘"要求工具朝下、
且方块要能落进托盘内壁）。因此 r_min 偏乐观（臂可以折回到基座附近但姿态不可用）；
工程上应再按姿态可达性筛一次，并把结论写成"可用环带"而不是"可达环带"。
这一步与 §11 的 A/B/C 决策绑定（姿态约束取决于臂怎么摆、托盘朝哪）。

### 11.2 「两个臂」这一侧的上游缺口（2026-09-24 只读核查）

终局是"两个臂 + 一个狗轮番抓运"，但当前：

```
场景声明：scenes/handoff_lab/scene.yaml 只有 1 个 arm（piper, role: manipulator，第 28~29 行）
可用臂资产：vendor/ 下只有 agilex_piper 与 unitree_go2（**没有第二个臂的厂商资产**）
             build/models/ 只有 piper 系列产物（piper-pick / piper-multi / piper-random）
构建器：`robots` 被提及 10 次（读取与校验列表），但 `--robot <单个 id>` ⇒ 无逐本体装配循环
```

⇒ 结论与代价（无论 A/B/C 哪个方案都要面对）：
1. **第二个臂需要自己的完整交付链**：厂商资产锁定（vendor U1）→ Profile → 机型基线 →
   能力验收 → 技能层；这不是"再声明一行"的事；
2. A（单模型）必须把构建器从"一次一个本体"扩到 **N 个本体 + 本体间相对位姿的声明与自洽断言**，
   否则连"两个臂"都装不进同一个模型；B/C 只需声明 N 组相对变换/几何耦合，但第二个臂的资产链一样要补；
3. 因此"轮番抓运"的**最近一步**仍是**一个臂 + 一个狗**的端到端闭环（s02_dock → s03_pick → s04 →
   s05），第二个臂作为后续独立工作项（含资产与验收），不要与本次 A/B/C 决策混在一起压工期。

### 11.3 停靠机制的可判实验（2026-09-24，夹具在 build/，**未改生产声明**）

自指帧被门禁挡下后，"接近过程"从未真正跑过；本实验把目标帧临时指向**世界物** `box_01`
（夹具 `build/iraf-a6a14/robot-station-probe.yaml`，只改 `target_frame` 与 `timeout_s: 20.0`；
生产声明仍指向 `tray_frame`），从而第一次让闭环接近真的执行。判据取自 s02_dock 的 criteria
（0.03 m / 2.0° / 0.05 m/s，由调用方传入）。探针 `build/iraf-a6a14/station_probe.py`，
报告 `station-probe.json`，两次运行逐位可复现（0.077448772 两次相同）。

```
目标帧 box_01（kind=body、body=box_01、world_fixed=true）      ← 门禁通过、不再是自指量
settled_at_s = 2.6599999999999286        （自指帧时恒为 0.0 ⇒ 对照：接近从未执行）
行程：first 0.000384 / max 0.161309 / last 0.148779 m          （自指帧时恒 0）
到位时刻 t=2.66 s：狗 xy=(0.160066,−0.000265)、相对目标**声明位姿**误差 = 0.029935 m（判据 0.03）
末拍 t=21.50 s：误差 0.077448772 m ⇒ failure = DOCK_DRIFTED（显式失败，不是假成功）
偏航 −0.302659018°（判据 2.0°）✓、末速 2.18552552e-04 m/s（判据 0.05）✓
```

**误差预算逐项对账**（这是本实验最有价值的部分）：

```
到位时误差      0.029935 m   = 目标声明位姿 vs 狗到位位姿
+ 狗保持期自漂  0.012673 m   （18.4 s 内的狗侧位移；yaw +0.6211°）
+ 目标侧被推    ≈0.034841 m  （自由体 box_01 被推走 ~35 mm）
= 末态误差      0.077449 m   （与报告值一致）
⇒ 狗只在容差内待了 38/2150 拍（t=2.66~3.09 s）
```

三条可用于设计的结论：
1. **接近机制成立**：世界物作为目标时闭环收敛并进入容差（0.029935 m < 0.03）—— 这是狗侧第一次
   *真实*的停靠证据（此前"36 µm"是自指的）。但它**不构成停靠验收**（目标本身不是合法站位帧）。
2. **控制容差必须严于验收判据**：`approach_command` 在容差边界即返回零 ⇒ 到位误差天然贴着判据
   （本例余量仅 65 µm），任何保持期漂移都会把验收打红。目标站位落地时，控制容差（声明值）应
   显著小于步骤级 criteria（例：控制 0.01 m / 判据 0.03 m），否则"通过"靠运气。
3. **自由体不能当停靠靶**：box_01 被推走 ~35 mm（占末态误差的 45%）。⇒ 站位帧必须是**焊死的静态
   物体**；而构建器当前给每个 world prop 都加自由关节、且不支持世界固定 site（`mount_frames` 只能
   挂在躯干上、跨本体挂载会显式失败）⇒ 站位帧需要**构建器 + schema 的小改动**（原以为"一行+一个
   prop"，实测不是）。这项与 A/B/C 一起决策。

### 11.4 站位帧落地后的停靠实测：机制成立，停止精度被「速度地板 + 冻结延迟」限制（2026-09-24）

落地内容：`scene.yaml` 声明 `frames: [handoff_station_frame]`（worldbody site，实测
`site_bodyid = 0` ⇒ 世界固定、不在躯干子树内）；`dock_for_handoff.target_frame` 指向它；
控制容差 0.015 m / 1.0°（声明）；timeout 12 s；验收判据仍取 s02_dock 的 0.03 m / 2.0° / 0.05 m/s。
探针 `build/iraf-a6a14/station_acceptance.py`（判据从场景步骤读，不另定数字）。

三次实测（同探针、同夹具）：

```
① 控制容差 0.005 m ⇒ 跑满 20 s 超时；**超时那刻残差 0.006330 m / 0.329184°** —— 已在验收判据内
   却始终进不了控制容差 ⇒ 实测**行走控制器在 0.15 m/s 接近档的定位地板 ≈ 6.3 mm**
   （指令已小到 ~5 mm/s 量级，残差随步态周期抖动）
② 控制容差 0.015 m / timeout 12 s ⇒ 到位 t=6.560 s（误差 0.014856 m）、行程 0.535 m ✓、
   偏航通过 ✓、末速通过 ✓，但**末态平移 0.060969 m** ⇒ DOCK_DRIFTED
③ 逐拍归因：**到位后 1.01 s 内位移 40.6 mm**（到位瞬间 |v| = 0.039549 m/s），随后 4 s 只再走 ~6 mm
   ⇒ 不是慢速爬行，而是"带 ~40 mm/s 惯性进容差 + 站定冻结需 ≥1 个步态周期"造成的**滑行超调**
④ 假设否证：`gait.walk.command_bias_mps` 在零指令时置零 ⇒ **更差**（63.5 → 87.8 mm、
   偏航 −0.44° → −3.02°）⇒ bias 在零指令下也在**部分抵消**步态整流，改动已回退（行为保持）
```

候选修法（未实施，按代价排序；全部需要**新的实测**，不得拍数）：
(a) **制动提前量**：到位判据改为 `误差 ≤ braking_lead_s × |v_body|`（实测 latency ≈ 1 s），
    用惯性把残差带到 ~0 —— 引入一个**标定过的**时间常数（须扫描 + 留证据）；
(b) **接近速度下调**：0.15 → 0.05 m/s，滑行量按比例缩小（未实测；受"速度地板"约束，过低会走不动）；
(c) **重定判据**：0.03 m 对"以 0.15 m/s 接近的步态"偏紧（滑行 40 mm）——若改 0.06 m 则本次即过，
    但那是**改判据**而非改控制器，必须人工决策（铁律 3：不得为让测试通过而放宽门禁）。

状态（诚实）：`dock_for_handoff` **仍未达成验收、仍未在 Profile 声明能力**；
世界固定帧机制本身已验收（6 项单测 + 构建产物实测）。

## 13. A 方案（单模型双本体）进度与剩余阻塞（2026-09-24）

已落地（提交 c04da74 + ff9056e）：
  · 声明：`robots[].placement`（相对位姿）、`robots[].profile_name`（场景 id ↔ Profile 身份显式映射）、
    `model.joint_output`；piper 的 Profile 补 `spec.model{vendor,file,trunk_body}`；
  · 构建器：`_attach_robots()` 用 **MuJoCo 原生 `MjSpec.attach(child, prefix, frame)`** 合成；
    关键帧 qpos 变长按**声明**补齐（`spec.home` ∪ `spec.gripper.open_positions`，缺键即显式失败）；
    CLI `--attach <本体>`（可重复）；`injections.attached_robots` 进报告。

逐门通过顺序（每一步都是"显式失败 → 修 → 再过"）：
  ① 身份检查：场景 id `piper` ≠ Profile 名 `piper_mujoco` ⇒ 加显式 `profile_name` 映射（仍硬校验）
  ② `spec.model` 缺失 ⇒ 给臂的 Profile 补构建期声明
  ③ 厂商锁 schema 不匹配（`iraf.piper-model-source-lock/v1` vs U1）⇒ **显式双约定支持** +
     在报告里标注该锁是 **host-specific**（见下）
  ④ 抓手关节无初值 ⇒ 取 `spec.gripper.open_positions`（不猜）
  ⑤ **当前阻塞**：`spec.to_xml()` 写产物时资产路径解析失败 ——
     `vendor/agilex_piper/piper_description/mujoco_model/../../../vendor/unitree_go2/.../base_link.STL`
     不存在：两个本体的 meshdir 不同，合成后相对路径互指。

下一步（修 ⑤，二选一，我建议 A1）：
  · A1（自包含，推荐）：把臂引用的 mesh/texture **拷进 `build/scenes/handoff_lab/assets/`**，
    在合成 spec 里把 `spec.meshes[].file` / 纹理路径改写到该目录，再 `to_xml()` ⇒ 产物自包含、
    不写死本机路径；
  · A2（省事但产物绑机器）：把资产路径写成绝对路径并**在报告里标注 host-specific**。

新债 S4（本轮回溯发现，登记进队列）：`vendor/agilex_piper/piper_description` 是**符号链接**
（→ `~/piper_ros/src/piper_description`），即**臂的厂商资产并未入库**，其锁里写的是本机绝对路径
⇒ ① 换一台机器/新克隆无法复现臂侧仿真；② 联合产物的可移植性受此限制。迁移到 U1 约定（实体入库 +
重写锁 + 许可证 BOM）是一个独立工作项。

## 14. 联合模型之后：让臂侧装配指向联合产物（精确待办，2026-09-24 实测合同差异）

现状：
  · `build/scenes/handoff_lab/handoff_lab_joint.xml` 已产出（nbody=28/nq=34/nu=20），
    同模型 FK 实测"臂够得到托盘"（最小距离 0.038048 m）；
  · 联合报告 `handoff_lab_joint.json` 的 `output` **已指向**联合产物 ✓ —— 但臂侧的
    `robot.backend_config.mode: scene_report` 还缺两项（实测 `scripts/scenario.py:build_backend_config`
    的必需键）：
      `gripper`：现在是 `None`；臂自己报告里的形状是
        `{wrist_body, left_finger_body, right_finger_body, left_finger_geom, right_finger_geom,
          open_positions{joint7/8}, closed_positions{joint7/8}, lift_positions{joint1..8}, min_lift_delta_m}`
      `targets[]`：现在是 `[]`；形状是 `{id, body, geom, material, rgba, position_m, quaternion_wxyz, euler_deg}`
  · `robot.backend_config.report` 现指 `build/models/piper-pick-scene.json`（臂自己的场景）⇒ 要么新建
    一份指向联合报告的机型绑定（推荐，不动既有臂侧验收），要么改既有绑定（会影响臂侧既有验收）。

实现要点（下次直接照做）：
  1. 联合报告增 `manipulation` 段：从**声明**生成（臂 Profile 的 `spec.gripper` + 场景的 `props`），
     但名字必须换成**联合模型里的实际名字**：`piper_link6/link7/link8`、指爪 geom 名要从
     合成模型的 `model_facts`（几何名表）读，**不要**照抄臂自己报告里的 `piper_left_finger`
     （那是臂自己场景的命名，联合模型里不存在 ⇒ 会装配失败或指错）。
  2. 新增机型绑定 `config/machines/piper_joint.yaml`（`backend_config.report` 指向联合报告；
     `target_tolerance_m` 仍显式声明），从而**不改** `config/machines/piper.yaml` 与臂侧既有验收。
  3. 验收：`load_backend(mujoco_arm, <joint report>, …)` 通过 + 在联合模型上跑一次 `move_joint`
     （到声明 home）与 `stop`；随后才是 `s03_pick` 的三项判据口径。
  4. 诚实边界：联合产物的资产是绝对路径（host-specific）、臂资产是符号链接（债 S4）⇒ 上述验收
     只在**本机**可复现，迁移需先做 S4。

**§14.1 实测进展与当前精确卡点（2026-09-24）**

已实现：
  · 联合报告 `manipulation` 段：继承**臂侧场景报告**的 `gripper`/`targets`，名字按可判定规则改写
    （`<prefix>name` 存在且 `name` 不存在 ⇒ 改写；用**编译后的模型**校验，不看文本）；
    实测 `wrist_body=piper_link6`、`left/right_finger_body=piper_link7/8`、
    `open_positions={piper_joint7:0.035, piper_joint8:-0.035}`、`targets=[(box_01, box_01_geom)]` ✓
  · `robots[].manipulation_report` 声明（缺声明即失败，不猜）+ schema
  · 改名规则 **fail-closed**：名字在联合模型里既无原名也无前缀名 ⇒ **退出码 3** 显式失败
    （避免装配指错对象）

**当前卡点（退出码 3，原因已定位到一行）**：
  厂商 Piper MJCF 的**指爪 geom 是无名的**（`<geom type="mesh" mesh="link7"/>`，实测
  `piper_link7 -> [None]`）；臂自己的场景构建器给它们起了 `piper_left_finger`/`piper_right_finger`
  —— 这个命名**只存在于臂自己的产物里**，联合模型里没有 ⇒ fail-closed 拒发报告（这是**正确行为**）。

两种修法（下次二选一，建议 A）：
  · **A（对齐命名，推荐）**：联合构建时给附加本体的指爪 geom **命名**（用臂侧报告里声明的那个名字，
    已在声明里：`gripper.left_finger_geom=piper_left_finger`）⇒ 联合模型与臂侧约定一致，
    grasp 判据（按 geom 命中双边接触）能直接用；
  · B（只改报告）：联合报告不继承 geom 名，而是**从模型里解析**"指爪 body 上的碰撞 geom"并写进报告
    （需记录解析规则与候选，避免"取第一个 geom"这种隐式选择）。

**§14.2 A 方案已落地（2026-09-24，使用者点选 A）**

选 A（声明式名字映射）而不是"选择性前缀"的理由：A 立刻可用、改动面在解析层（可审计），
而选择性前缀要自己实现 MjSpec 的改名 + 追引用（joint↔actuator、mesh/material↔geom 漏一处会**静默**坏掉）。

已完成（提交见 git 历史；证据 `build/iraf-a6a14/arm_on_joint_model.json`）：
  · 联合报告新增 `manipulation.name_map`（16 条 = 8 关节 + 8 连杆）+ `name_map_facts`
    （mapped/identical/conflicts/missing 四桶；实测 conflicts=0、missing=0）；
  · `scripts/scenario.py:build_backend_config` 透传 name_map；联合报告缺 name_map ⇒ 拒绝装配；
  · `mujoco_backend._parse_name_map` 四条装配期校验；解析点覆盖 `_direct_actuator_channel`、
    `_joint_name_of`（回写仍为声明名）、`_body_id`、`_joint_qpos`；
  · `scripts/emit_backend_config.py` 一并搬 name_map；
  · 证据：负向（去 name_map）复现 `找不到关节或执行器: joint1`；正向 `move_joint`/`stop`
    经 SkillRuntime 跑通，joint3/5/7 末态误差 0.0031607172863115096 / 0.016467153997658635 /
    3.857912913088346e-08 rad（判据 ≤0.05），狗的 12 个执行器逐位未被触碰；
  · 回归：`tests/unit/test_joint_name_map.py`（7 项）。

**§14.3 仍待做（下一步排序）**

1. 两个本体的**执行绑定都指向联合 MJCF**：`baseline.yaml` 里 piper 指 `piper_joint.yaml`、
   go2 需要新建 `config/machines/go2_joint.yaml`（`backend_config.report` → 联合报告）；
   随之要回答"两个后端实例 = 两份 MjData、谁步进"这一问题（**未解决，且是联合场景的前提**）。
2. `s03_pick` 三项判据口径（`pose_tolerance_m` / `min_lift_delta_m` / `require_bilateral_contact`）。
3. `MujocoBackend.stop()` 的**作用域**（现在归零整条 ctrl 数组 ⇒ 会碰主本体执行器）——
   安全语义问题，待裁定。
4. 联合产物资产相对化（消 host-specific）；债 S4（臂厂商资产入库 + 重写锁 + 许可证 BOM）。

## 15. 共享植物（一份 MjData + 多控制器）—— 设计（2026-09-24 实测依据）

### 15.1 为什么必须做（实测，不是推断）

1. `build/iraf-a6a14/joint-model-two-backends-probe.json`：把两个本体的机型声明**都指向同一份
   联合 MJCF**，得到的仍是**两个独立世界**（`same_mjdata_object=False`；一侧步进 200 步，
   另一侧 base qpos 逐位未变）⇒ 狗把方块放到托盘上这件事，臂的世界里**看不见**。
2. `MujocoBackend.stop()` 归零**整条** `ctrl` 数组 ⇒ 联合模型里臂的 stop 会碰主本体执行器
   （实测 `FR_thigh` 0.9 → 0.0）⇒ 作用域必须按执行器划分。
3. 关键帧 `ctrl` 对臂侧 8 个执行器是零填充（`joint-model-keyframe-ab.json` 的
   `keyframe_ctrl_rest_nonzero=False`）⇒ 臂的开合初值不来自声明。

### 15.2 设计（4 步，每步独立可验收；S1/S2 不改变单本体行为）

**S1 `MujocoPlant`（新文件 `src/iraf_adapters/mujoco/plant.py`）**
  · 持有 `model` / `data` / `step_index` / `owner`（**唯一时间推进者**的标识）；
  · `advance(n)` 仅 owner 可调，非 owner 调用**显式失败**（禁止两个步进者把时间推两遍）；
  · `wait_until(index, timeout)` 供 guest 等时间（等价于以前 `move_joint` 自己步进）；
  · 复用现有 `RLock` 语义（viewer 的 `sync()` 必须与步进互斥）。

**S2 后端支持注入植物（`MujocoBackend` / 四足后端各加 `plant=None` 参数）**
  · `plant=None` ⇒ 自带 model/data、自带步进 ⇒ **单本体路径逐位不变**；
  · 注入时用 `plant.model` / `plant.data`，并声明本后端的**执行器集合**（由 `name_map` 的
    反向表 + actuator 前缀判定），写 `ctrl` 前校验"这个执行器属于我"，越界即显式失败
    （对应铁律 1.13：同一执行器任一时刻只允许一个控制源）；
  · 四足后端仍是 owner（它已有 100 Hz 的 MPC 消费循环），臂后端是 guest。

**S3 `stop()` 作用域**
  · `_safe_stop_controls()` 只归零**本后端声明的执行器**；单本体路径下该集合 = 全部执行器
    ⇒ 逐位不变（用 loopback 三基准回归证明）。这是**安全语义**变更，落地时要在文档里写明
    "急停的作用域 = 本控制器的执行器，全植物急停由场景层单独提供"。

**S4 场景装配**
  · 新增 `config/machines/go2_joint.yaml`（`backend_config.report` → 联合报告）；`baseline.yaml`
    的 piper / unitree_go2 都指向联合产物；
  · `scenario.assemble` 在联合模式下：先建 `MujocoPlant`，再建两个视图（顺序固定，避免"谁先步进"的竞态）。

### 15.3 验收（每条必须给数字，不接受"命令被接受"）

1. 两个视图共享同一对象：`arm.plant.data is dog.plant.data` = True；
2. 臂 `move_joint` 期间狗的姿态**逐位不变**，且狗 `locomote` 期间臂的关节角**逐位不变**；
3. 臂 `stop` 后狗的 12 个执行器 ctrl 逐位不变（S3 的判据）；
4. 单本体回归：loopback 三基准 0.279953602548388 / 3.302184116303541e-05 /
   0.0032068060511408506 逐位不变，10/10 checks；
5. 端到端：狗在联合模型里走到站位 → 臂抓取托盘上的方块 → 放回 → 载荷确认
   （对应 `s02_dock` / `s03_pick` / `s04` / `s05`）。

## 12. 「世界固定帧」的最小改动设计（待授权后照此落地，本轮只写设计）

目的：让停靠有一个**合法**目标（世界固定、不与机器人刚性相连），从而把 s02_dock 从"待交付"推进到
可执行。改动面刻意压到最小，且**不引入任何机型数字**（坐标仍来自声明）。

### 12.1 声明形状（`scenes/<id>/scene.yaml` 新增一段，与 `props`/`sensors` 平级）

```yaml
frames:                                  # 世界固定参考系（导航/停靠/交接的目标帧）
  - id: handoff_station_frame             # 帧名 = 构建产物里的 site 名（唯一）
    kind: site
    pose:
      pos_m: [0.55, 0.0, 0.0]             # 世界系；站位坐标
      quat_wxyz: [1.0, 0.0, 0.0, 0.0]     # 期望机身朝向（yaw 由此给出）
    note: >-
      站位坐标的依据：机械臂在托盘高度（z≈0.347 m）的可达环带 r ∈ [0.002028, 0.594284] m
      （build/iraf-a6a14/reach_probe.py），取 r ≈ 0.55 m 保守内侧；**待臂的摆放确定后复核**。
      控制容差必须严于步骤判据（见 §11.3 结论 2），否则保持期漂移会吃掉验收余量。
```

### 12.2 契约与实现（各一处，均无隐式默认值）

| 位置 | 改动 | 失败路径（显式） |
| --- | --- | --- |
| `config/scene.schema.json` | 新增 `frames`：`id`（必填、唯一）、`kind: {enum: [site]}`、`pose.{pos_m(3), quat_wxyz(4)}` 必填 | 缺 `pose`/非有限 ⇒ 退出码 2 |
| `src/iraf_adapters/unitree/scene_builder.py` | 新增 `_inject_world_frames(xml_root, frames)`：在 `<worldbody>` 下直接 `ET.SubElement(worldbody, "site", ...)`；名字与既有 site/body 重名即失败；把注入结果写进报告（`frames[]`，含 `injected_as: world_site`） | 重名 / 非法四元数 / `kind` 非 site ⇒ 退出码 2 或 5（与既有注入同表） |
| 报告字段 | `model_facts.frames = [{id, kind, injected_as, pose_source: "declaration"}]` | 缺 ⇒ 验收脚本判失败 |

为什么用 `<site>` 而不是 prop：site 无质量、无关节，天然世界固定且不参与碰撞（不会像 prop 那样被推
或在台面上占位）；停靠只读它的位姿。**不需要**给 prop 加 `static` 开关，改动面因此更小。

### 12.3 验收判据（四层，全部可复跑）

1. **构建层**：`scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2` 退出码 0；
   报告里出现该 frame 且 `injected_as == "world_site"`；重名/缺 pose 的负向夹具 ⇒ 退出码 2。
2. **门禁层**：`scripts/verify_dock_target_frame.py` 用 `target_frame: handoff_station_frame` 跑一遍，
   必须**不再**触发自指门禁（该 site 的所属 body 是 worldbody，不在躯干子树内）；
   负向：把同一帧改挂到躯干（夹具）⇒ 门禁显式失败（本提交 `ba22c9e` 已具备）。
3. **停靠层**（适配器）：`settled_at_s > 0`、行程 ≥ 声明站距的 90%、平移误差 ≤ **控制容差**、
   偏航 ≤ 控制容差、保持窗末速 ≤ `stop.speed_tolerance_mps`；`failure is None`。
4. **场景层**：`s02_dock` 去掉 `pending_closed_by` 后，`scenario.py run --scenario nominal` 的预检
   不再因该步失败（判据可评测性已于提交 `224f925` 补齐）。

### 12.4 与 A/B/C 的关系

本设计**不依赖** A/B/C（站位是世界固定帧，臂在不在同一个模型里都能用）；但站位坐标的**复核**依赖
"臂的摆放"（A/B 会给出；C 只给几何）。⇒ 建议顺序：**先落这个最小改动**让 s02_dock 闭环（狗侧），
再按 A/B/C 决定臂侧，最后才谈"放块入托盘"的物理交接。
