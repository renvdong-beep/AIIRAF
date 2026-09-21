# 四足行走专项设计包（战役 iraf-24h-2 步骤 02b 收口产物）

- 生成时间：2026-09-21
- 状态：**未完成能力**（`locomote` / `navigate_to` 未回填 Profile；本包不是"已能行走"的证明）
- 依据：ADR-0008（决策 1/1b–1h）、`docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md`（§1–§26）
- 用途：把 02b 的判决与机机理固化，供下一阶段（专项）从"已量清的位置"继续，而不是重跑一遍

## 1. 结论一句话

在**现架构**（位置级步态轨迹 + 执行器级力矩平衡器 + "实测接触/声明相位"支撑集判定）下，
本战役未能让 Go2 在 MuJoCo 里稳定原地踏步；三条路线均有实测判决，失败机理已定位到
**分配器秩限制**与**步态动力学发散**两条硬约束。下一阶段应按"迈步式 crawl（落足点规划）"
这一**架构变更**立专项，而不是继续在同一架构内调参。

## 2. 已判决的三条路线（每条都有数字，不再重复试）

| 路线 | 实测判决 | 关键数字 |
|---|---|---|
| 动态 trot（duty 0.5，位置级，无平衡器） | 19→20 项判据中 **11 条失败** | height_mean_m −38.66691503360235、height_std_m 50.77553945155333、max_tilt_deg 149.00568880619002、max_displacement_m 3.5763946259283177、ctrl_saturated_samples 7306、max_tracking_error_rad 0.3101749935977546 |
| 位置级重心转移（wave + `gait.sway`） | 机制**被隔离实验证伪**（步高 0.001 m 下幅度 5 mm 即翻；幅度 0 稳定） | 幅度 0：z_mean 0.279955、倾角 0.091°、漂移 0.00649 m；幅度 0.005 m：z_mean −12.184303、倾角 171.175°、漂移 4.78069 m |
| 力矩级平衡器（B1：足端力分配 + 姿态 PD） | 机制落地且参与（参与率 9.3%→26.5%），姿态/高度稳定但**四足不离地**；翻转支撑集口径后仍翻倒 | trot+B1：height_mean 0.289225708631836、倾角 0.27631651775922383°、漂移 0.01567937199659495 m、饱和 0，但四腿支撑相 1.0、clear_swing_cycles 0（自锁）；wave+B1：exit=5 / failed 11 |

## 3. 已定位的硬约束（下一阶段必须正面处理）

1. **分配器秩限制**：`balance.allocate_foot_forces` 的力旋量映射在**两腿支撑**时秩 5<6（trot 复评 `exit=2`，未生成报告）
   ⇒ 2 腿支撑与现分配器**不相容**；≥3 腿支撑才是其可行域。这解释了为什么"trot + 平衡器"必然走不通。
2. **自锁（已定位并已修机制）**：支撑集若**只按实测接触**判定，"该抬还没抬"的腿被计入支撑并分摊 ~mg/4 ⇒ 被压住；
   支撑腿位置权重为 0 ⇒ 摆动轨迹抬不动它。修法（决策 1f/1g）＝**声明相位 ∧ 实测接触**，且**按控制模式**区分
   （仅步态时钟激活时启用；静态保持回 `contact_only`）。**该机制已落地并带 fail-closed 兜底**（见 §4）。
3. **静态可稳需要逐相位迈步**：四相位可行域交集 **0 点**（对角互斥 FL+RR=0、FR+RL=0）；重心移到支撑三角形
   形心需平移 **0.079~0.081 m**（此时最小法向力 47.325754 N = 30.9317 % mg）⇒ 静态可稳的 wave 本质是
   **完整 crawl**（逐相位改落点），不是"常量重心微调"。
4. **D3（占空比/步频）见底**：`duty` 与步频两个声明层变量已试完，继续调＝空转（第 21/22 轮实测结论）。
5. **看门狗预算与 latch**：静态保持路径在 dc 口径下支撑集余量为 0，单腿卸载连续 14~15 周期 > 预算 10 ⇒ latch 触发；
   预算 10→1000 后：恢复窗口倾角 **116.0440876799953 → 7.883272174700346**（判据 ≤15）、最低高度
   **−0.08798844894154116 → 0.2673849212842271**、翻倒 370 → None。⇒ 预算是真实约束，需按"退化为 3 腿支撑"的
   最大允许窗口重新定档（声明层）。

## 4. 本战役实际交付的基础设施（可复用，别重写）

- `config/go2_loopback.yaml`：`gait`（19 项判据全声明）、`locomotion`、`balance`（含 `stance_classification`
  按**控制模式**区分：`gait_clock_active: declared_and_contact` / `gait_clock_inactive: contact_only`）、
  `render`、`stand/stop` 语义（`damped_hold` / `torque_zero_release` 并列）
- `src/iraf_adapters/unitree/gait.py`：参数化步态生成器（trot/wave、相位、摆动轨迹、腿部 IK/FK、`assess_trot`）
- `src/iraf_adapters/unitree/balance.py`：期望机身力旋量 → 足端力分配（最小范数 + 逐腿截断 + 残差）→ 关节力矩；
  `declared_and_contact` / `contact_only` 两口径与一致性门禁
- `src/iraf_adapters/unitree/unitree_go2.py`：`_run_control` 支持位置/力矩两通路（未启用时与既有路径**逐位一致**）、
  `gait_in_place`、`trot_in_place`（薄包装）、显示面（`display_lock`/`render_frames`）
- 场景生成：`scene_builder.initial_alignment`（按实测把最低几何抬到支撑面，消除 18.372 mm 穿透）
- 验收入口：`scripts/verify_go2_gait_in_place.py`（20 项判据）、`scripts/verify_go2_balance.py`、
  `scripts/view_go2_gait.py`（显示，非验收）；契约用例 ~50（gait/balance/限位/TTL）
- 门禁与纪律：技能租约 TTL ≥ 声明时长、力矩上限只读模型 ctrlrange、能力仅在验收通过后回填、
  声明与门禁同文、逐位一致回归、fail-closed 兜底

## 5. 建议的下一阶段（专项，按推荐顺序）

1. **C：迈步式 crawl（落足点规划）** —— 唯一能同时满足"静态可稳"与"分配器秩可行"的路线。
   落地前置（第 22 轮已量清）：`gait` 段需新增**声明段**（步幅/落点/摆动相落点策略，现 11 个顶层键中
   `stride|foothold|step_length` 命中 0）；实现侧改目标生成（现每腿水平目标 = `neutral_x_m + dx + damping − sway`，
   `foot_offset` 只产生抬脚 z）；摆动相必须**落到计划落点**而不是固定中立位；门禁需按新语义同文更新。
   **这是架构变更**，需扩大文件边界并单独立步骤/专项。
2. **看门狗预算按"退化为 3 腿支撑"重定档**（声明层，成本低，是 C 的前置之一）。
3. 之后再谈速度与跟踪指标（`max_tracking_error_rad` 等量级问题），最后才是回填 `locomote` 与 `navigate_to`。

## 6. 诚实边界

- 全部结论 `simulation=true`；真机与目标端 **DEFERRED**（板卡不在场，需 `unitree_sdk2` + HIL）。
- `locomote` / `navigate_to` **未声明**（铁律 2：验收不通过不得声明）；`gait_prod` 验收长期 `exit=5`。
- 本包不含"该路线必可行"的证明：C 的落地前置是**只读代码/声明量测**，不是可行性证明。
- 战役 1 的 stand/stop 旧数字（height_mean_m 0.2801007638069918 等）测于 18 mm 穿透前提，已被 A′ 后的重测取代。
