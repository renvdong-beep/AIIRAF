# 四足行走专项计划：迈步式 crawl（落足点规划）

- 建立时间：2026-09-21
- 依据：`docs/progress/2026-09-21-quadruped-walk-special-design.md`（§3 硬约束 / §5 建议）、
  `docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md`（§26.3 / §27.5 / §27.6）
- 人工决策：**C —— 改判落足点规划（迈步式 crawl）**（2026-09-21，主窗口）
- 状态：**计划（未执行）**。本文件不构成"该路线可行"的证明；`locomote` / `navigate_to` 仍未声明。
- 锚点：全部 `file:line` 于 HEAD `6b0d0fc` 复核（gait.py:942 / balance.py:660,374 / unitree_go2.py:737 /
  config/go2_loopback.yaml:102,145,334,440,457 / profiles/safety/quadruped_lab.yaml:24,48,54）

## 0. 为什么是 C，以及 C 的物理内容

已判决的三条路线都在"**平移机身**"这条轴上：位置级 sway（被隔离实验证伪，5 mm 即翻）、
力矩级平衡器（静态可稳但四足不离地）、trot（11/19 判据失败且两腿支撑与分配器秩 5<6 不相容）。

C 换轴：**不改机身目标，改足端落点**。静态可稳的算术结论是
「四相位可行域交集 = 0 点；重心移到支撑三角形形心需平移 **0.079~0.081 m**，此时最小法向力
47.325754 N = 30.9317 % mg」⇒ 支撑三角形必须被**逐相位重新构造**（把某条腿挪到新落点），
而不是靠机身常量平移。这同时满足静态可稳与「≥3 腿支撑（分配器可行域）」。

## 1. 目标环境与验收场景

- 环境：宿主 x86_64 / `/usr/bin/python3` 3.10.12 / mujoco 3.3.3 / `simulation=true`（**非真机**）
- 场景：`scenes/handoff_lab` + `unitree_go2`（沿用战役 2 的场景与 `initial_alignment` 对齐前提）
- 验收分级（ADR-0008 §3 口径）：
  - **L1 静态可稳（本专项第一步，必须先过）**：四足逐相位离地（每腿 `clear_swing_cycles` 达标）、
    稳态倾角 ≤ `max_tilt_moving_deg`（15°）、机身高度在声明容差内、**看门狗零触发**、
    安全事件 0、`locomote` 仍未声明
  - **L2 定速直行 / 定速转弯（`locomote` 回填的前置）**：速度误差、航向误差、原地转弯
  - **L3 到点（`navigate_to`）**：位置误差 ≤ 0.10 m、航向 ≤ 5°、**末速 ≤ `stop.speed_tolerance_mps` 0.05 m/s**
- 金路径（本专项）：
  `PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_crawl.py --config config/go2_loopback.yaml`
- 观感（不是验收证据）：`DISPLAY=:0 XAUTHORITY=<桌面授权> MUJOCO_GL=glfw PYTHONPATH=src /usr/bin/python3 scripts/view_go2_gait.py --config config/go2_loopback.yaml --seconds 45`

## 2. 影响面（先写清，避免开工后发现越界）

| 面 | 影响 | 说明 |
|---|---|---|
| 公共 IDL（`api/`） | **无** | 落足点是适配器层声明，不进公共契约；`locomote`/`navigate_to` 的能力声明仍未回填 |
| RobotProfile / SafetyPolicy | **无阈值变更** | 看门狗预算重定档单列为 Step 6（声明层，需单独授权口径） |
| 架构图 | **无新模块** | 只改适配器内部目标生成语义 |
| ADR | **需追加** | ADR-0008 决策 **1j**：首期步态由 wave → 迈步式 crawl，并登记"原地"口径的新语义 |
| 能力声明 | **不回填** | 铁律 2：验收未全绿不得回填 |

## 3. 文件边界（路径级，供并行窗口避让）

允许新建 / 修改（本专项独占）：

- `.hermes/plans/2026-09-21_quadruped-crawl-foothold.md`（本文件，本地草稿区）
- `config/go2_loopback.yaml` —— **只新增** `gait.foothold` 段与注释；既有键**一字不改**
- `src/iraf_adapters/unitree/gait.py` —— 只做三件：① 新增 `foothold` 声明解析与校验函数；
  ② 改 `gait_joint_targets`（`:913-961`）的水平项目标生成（引入相位相关落点插值，**保留既有分支**）；
  ③ 新增专项判据函数（新函数，**不改** `assess_gait`）
- `scripts/verify_go2_crawl.py`（新验收入口）
- `tests/unit/test_foothold_contract.py`（新契约用例）
- `docs/adr/0008-quadruped-locomotion-and-navigation.md`（追加决策 1j，1–5 节不动）
- `docs/debug/2026-09-21-quadruped-crawl-foothold.md`（新调试记录）
- `build/iraf-24h-3/**`（证据区，gitignored）

**禁止触碰**（并行窗口 / 门禁红线）：

- `src/iraf_core/**`、`examples/demo3_arm/**`（另一窗口）
- `assess_gait` 的 20 项判据、`gait.verification.max_displacement_m: 0.05`（`:440`）、
  `gait.duty_factor: 0.75`（`:334`）、`gait.verification.min_swing_fraction: 0.25`（`:457`）、
  `balance.allocation.min_stance_legs: 3`（`:145`）、`balance.enabled`（仍 `false`）、
  `profiles/safety/quadruped_lab.yaml` 既有阈值、`scripts/verify_go2_gait_in_place.py`
- 提交纪律：只 `git add` 本步声明的路径；禁 `git add -A`；禁 heredoc 提交

## 4. 步骤（每步产出可复跑证据；数字用同一套抽取原语）

### Step 1 声明段 + 契约门禁（声明先于实现）
- 新增 `gait.foothold`：`mode ∈ {static, per_phase}`（缺键 ⇒ 退出码 2，不允许默认值）、
  `stride_m`（每相位水平位移，**实测所需量级 0.079~0.081 m**）、`phase_direction_map`（**必须环形顺序**，
  否则方向向量 x 分量每窗口翻转 —— §9 已踩过）、`ramp_s`、`smoothing_s`
- 工作空间**不新增重复事实**：继续读 `profiles/safety/quadruped_lab.yaml` 的 `workspace_m`（`:54`）
- 门禁：`scripts/profile_check.py --quadruped` 增「`foothold` 段合法性 + `mode=static` 与 `per_phase` 的组合约束」
- 证据：`profile_check --quadruped` exit=0 + **负向用例**（删键 ⇒ 退 2）
- 判据：新增门禁必须带负向对照（缺键/越界/方向表不闭合三种）

### Step 2 相位相关落点插值（**逐位一致是硬门禁**）
- 改 `gait_joint_targets` 水平项：现为 `neutral_x_m + dx + damping[0] − sway[0]`（`:942-944`），
  改为「按相位插值到计划落点」；`foot_offset`（`:618`）仍只负责抬脚 z
- **不变量**：`foothold.mode = static` 时，目标角与现路径**逐位一致**（用逐位对照脚本证明，
  与战役 2 的做法相同：先快照证据再比对，不得"看起来一样"）
- 证据：`tests/unit/test_foothold_contract.py` 全绿 + 逐位对照 `compare-static-parity.txt`

### Step 3 摆动相落到计划落点
- 摆动相轨迹从"只抬 z、原地落回"改为"抬 z + 落到计划 x/y"；支撑集口径继续按
  `gait_clock_active: declared_and_contact`（`config:102-104`）与 `min_stance_legs: 3` 耦合
- 摆动相期间支撑腿数**声明为 3**（不得再降）⇒ 与分配器秩可行域自洽
- 证据：新判据函数输出的摆动落点误差（声明容差内）+ 支撑腿数游程门禁

### Step 4 专项验收入口与新判据
- `scripts/verify_go2_crawl.py`：复用 `assess_gait` 的**度量抽取**，但判据集合与阈值**独立声明**
  （新键 `gait.foothold.verification.*`），避免污染既有 20 项
- 必须带**负向对照**：喂坏样本（人为把倾角/净漂移改坏）必须非 0 退出，证明判据不是恒真
- 证据：`build/iraf-24h-3/step04/…`（正例退 0 / 负例退非 0 全表）

### Step 5 第一项实跑（**最关键验收，早跑早暴露**）
- 只跑 L1 静态可稳一项，拿真实数字：`max_tilt_deg`、`net_drift_per_cycle_m`、`peak_body_excursion_m`、
  四腿 `clear_swing_cycles`、看门狗触发周期
- **决定性风险在这里暴露**：黏滑（设计包 P4：失稳驱动可能是位置级目标本身）是否随落点变化缓解，
  **未实测**。若 L1 仍翻倒且归因到黏滑/动力学，则 C 也判否 ⇒ 回人工，**不得改判据**
- 证据：`build/iraf-24h-3/step05/l1-run*.txt`（原始 JSON + 判据表）

### Step 6 看门狗预算按"退化为 3 腿支撑"重定档（声明层，独立授权口径）
- 依据实测（预算 10→1000 使恢复窗口倾角 116.0440876799953 → 7.883272174700346）反算声明值，
  **不直接抄 1000**；给出换算过程与适用范围
- 证据：单键对照实验（改前/改后逐键差异数 = 1）

### Step 7 收尾：ADR 决策 1j + 调试记录 + 汇总
- ADR-0008 追加 1j（首期步态 = 迈步式 crawl、"原地"新语义、验收入口）；写专项调试记录与证据索引
- 能力仍**不回填**，除非 L1+L2 全绿

## 5. 已代定的两个决策（含可测分支条件，可被人工一票否决）

### D-C1 验收入口与"原地"口径
- 决定：新增 `scripts/verify_go2_crawl.py`，口径 = **周期净漂移 + 峰位移**（新声明键）；
  既有原地入口 `verify_go2_gait_in_place.py` 的 20 项判据与 `max_displacement_m: 0.05` **一字不动**，
  继续作为回归（证明 crawl 没让既有路径退化）
- 理由：`max_displacement_m` 量的是**全程峰值**位移（`gait.py:1062-1071`），而迈步式 crawl 的物理本质
  就是周期内真实平移（实测重心需移 0.079~0.081 m）⇒ 留在 0.05 峰位移口径下 C 永远不可能通过。
  那不是"严"，是把判据套在错误的量上；既有判据不动、新口径是新事实，不构成放宽
- **分支条件（可测）**：
  - 若 Step 5 实测「周期净漂移 ≤ 声明 ∧ 峰位移 > 0.05」⇒ 按本决定执行（C 用新入口验收，旧入口作回归）
  - 若实测「周期净漂移 > 声明」⇒ **C 不通过**（净漂移不得放宽），回人工
  - 否决方式：人工可要求「原地 = 全程 ≤ 0.05 m」，此时 C 在该口径下**必然不可实现**，应按 A 收口

### D-C2 看门狗预算定档
- 决定：Step 6 用实测反算（"退化为 3 腿支撑"的最大允许窗口），不直接采用 1000
- 分支条件：重定档后必须逐位复跑静态保持路径（`verify_go2_balance.py`），倾角与最低高度不得恶化

## 6. 风险与失败/接管路径

| 风险 | 等级 | 判据/接管 |
|---|---|---|
| 黏滑使落点变化也无效（最大未知） | **高** | Step 5 早暴露；失败 ⇒ C 判否回人工，不改判据 |
| 摆动相支撑腿数掉到 2 ⇒ 分配器秩 5<6 | 中 | 声明支撑腿数 = 3 并设游程门禁；掉到 2 即判失败 |
| 相位方向映射顺序错（§9 踩过：x 分量翻转） | 中 | Step 1 门禁要求方向表**环形闭合**，带负向用例 |
| 逐位一致性被破坏（既有 wave/trot 退化） | 中 | Step 2 的 static 逐位对照 + 全量单测失败数不得变化 |
| 工期 | 低 | 预计 4~6 tick；到期末通过按 A 收口 |

## 7. DoD（完成定义）

- 代码与文档一致（声明↔门禁↔判据↔调试记录同文）；契约兼容性已验证；成功与失败路径都有证据；
- 未突破 AGENTS.md 安全边界；判据与阈值**从未放宽**；`locomote`/`navigate_to` 仅在有验收证据后才回填；
- 全部结论标注 `simulation=true`；真机/目标端仍 `DEFERRED`。
