# A6a-3b：自有 QP 构造器（移植清单与逐位校验点）

- 日期：2026-09-23
- 前置：`docs/debug/2026-09-23-locomote-provider-integration-spec.md`（A6a 契约）
- 为什么需要：`provider_runtime.step(request)` 只做"发送/收解/门禁"；**从机器人状态构造 QP** 的那一层
  目前只在研究侧（上游 `ComTraj` + `CentroidalMPC`）。生产路径不得依赖 `build/research/`（gitignored 隔离区），
  故必须在 `adapters/unitree/mpc/` 内落一份**自有实现**，并用已有的 `traj_parity` 门禁对研究侧**逐位**校验。
- 状态：**清单已定，实现待做**。

## 1. 要移植的部件（研究侧位置 → 目标模块）

| 部件 | 研究侧位置（行号为实测） | 目标 | 备注 |
|---|---|---|---|
| 参考轨迹生成 | `com_trajectory.py:27 generate_traj`（含接触表、Raibert 落足点、摆动腿五次多项式、`pos/vel/rpy/omega_traj_world`、`r_*_foot_world`） | `trajectory.py` | **保值摊销**：不计算无读取方的 `Ac/Bc/gc`（§15；13 数组逐位一致、P50 4.7464→2.9661 ms） |
| 离散动力学 | `com_trajectory.py:272 _discreteDynamics`（`Ad`/`Bd`/`gd`） | `trajectory.py` | 只此一份；`_continuousDynamics` 的产物**无读取方 ⇒ 不移植** |
| 状态参考向量 | `com_trajectory.compute_x_ref_vec()` | `trajectory.py` | 参与逐位校验 |
| QP 组装/更新 | `centroidal_mpc.py:178 _build_sparse_matrix`、`:235 _update_sparse_matrix`、`_assemble_A_matrix`、`:122 _compute_bounds`、`:324 _precompute_friction_matrix` | `qp_builder.py` | H 为纯对角（nnz=384）；A 为 `[I;A]` 形式由 `osqp_native.stack_identity_rows` 组装 |
| 常量 | `centroidal_mpc.py:12-17`（`Q`/`R`/`MU`/`NX`/`NU`）、`:20-36 OPTS` | 见 §3 | **必须落声明**，不得散在代码里（铁律 5.3） |

**不移植**：`mujoco_model.py`（仿真侧，生产用我们的主站/真机适配器）、`plot_helper.py`、
`_continuousDynamics`（死代码）、任何 CasADi 依赖（生产求解走 `osqp_native`）。

## 1.1 不得造第二份"接触表"事实来源（本轮澄清；违反即触铁律）

上游 `gait.py` 的相位/接触判定是 `phases = mod(PHASE_OFFSET + t/period, 1)`、`contact = phases < duty`；
**我们仓库已有同构实现**：`src/iraf_adapters/unitree/gait.py`（`is_stance(phase) = phase < duty`，
相位偏移来自声明 `gait.legs.*.phase_offset`；ADR-0009 §8.1 已实测"同构"）。
⇒ `trajectory.py` **必须复用** `iraf_adapters.unitree.gait` 的相位/接触表逻辑，
**不得**在 `mpc/` 里再写一份（同一事实只能有一处来源）。移植时只需校验：
用我们的接触表算出的 `contact_table` 与研究侧用它的接触表算出的结果**逐位一致**（同相位参数下）。

**已落地**：`src/iraf_adapters/unitree/mpc/contact.py`（`contact_table` / `current_mask`，行序 `LEG_ORDER`）。
移植中发现三处必须显式化的差异（不写清就会静默错位）：

1. **腿序**：上游 `PHASE_OFFSET = [0.5, 0.0, 0.0, 0.5]` 的索引顺序是 `[FL, FR, RL, RR]`
   —— **RL 在 RR 之前**，与我们既有 wave 声明的书写顺序不同 ⇒ `LEG_ORDER` 显式写死，
   不靠字典迭代顺序（单测覆盖插入顺序无关）。
2. **采样时刻**：上游 `compute_contact_table` 取 `t = t0 + arange(N)·dt + dt/2`，而
   `compute_current_mask` 取 `t = t0`（`dt=0, N=1` ⇒ 半拍项为 0，两者自洽）⇒ 半拍项由
   `half_step` 参数显式化，`current_mask` 是它的单点特例。
3. **相位结构（实测，与直觉不同）**：`duty = 0.6 > 0.5` + 两组偏移相差 0.5 ⇒ 两组支撑窗口
   在相位环上相交**两段**、总长 `2·(duty − 0.5) = 0.2` 周期 ⇒ 存在**四足同时支撑**窗口、
   **无腾空相**。「trot 恒两条腿支撑」只在 `duty ≤ 0.5` 成立。单测 64 采样实测重叠
   `12/64 = 0.1875`，与解析式一致。

**⚠ 阻塞项（本轮新发现，尚未解决）**：`config/` 与 `profiles/` 里**唯一的 `phase_offset` 声明是
wave 的环形顺序**（`config/go2_loopback.yaml:368-371`：FL 0.0 / FR 0.25 / RR 0.5 / RL 0.75），
**没有 trot 的对角相位声明**（上游 trot = FL 0.5 / FR 0.0 / RL 0.0 / RR 0.5）。
⇒ 按铁律 5.3「常量集中在版本化声明、代码不得写数字」与 §1.1「同事实一处来源」，
`contact_table` 的入参必须来自一份**合法的 trot 步态声明**（经 `gait.load_gait_declaration`
校验，含 `kind: trot`、`duty_factor: 0.6`、两组对角 `phase_offset`、`verification` 等段）。
本轮**未**伪造该声明（伪造即制造第二份事实来源）；下一步先落这份声明并让它过既有校验门禁，
再把 `contact_table` 接到它上面。

### 2.0 接触表逐位校验（已完成，2026-09-23）

脚本 `build/research/mpc-repo/verify_contact_parity.py`｜报告 `build/research/mpc-repo/contact_parity.json`
（`all_ok: true`，退出码 0）。基准 = 上游 `convex_mpc.gait.Gait`（不是我们重写的式子）。

| 对照项 | 规模 | 结果 |
|---|---|---|
| A 声明侧：上游 `PHASE_OFFSET` vs 我们按 `LEG_ORDER=[FL,FR,RL,RR]` 排开的声明偏移 | 4 项 | 一致：两侧均 `[0.5, 0.0, 0.0, 0.5]` |
| B `compute_contact_table`（含内部 `+dt/2`）vs `contact_table(half_step=True)` | 400 组 t0 × N=16 | **0 处不一致**（dtype 均为 int32） |
| C `compute_current_mask` vs `current_mask` | 2 周期 × 480 点 | **0 处不一致** |
| D 半拍反证：`ours(half_step=False, t0)` vs `theirs(t0 − dt/2)` | 400 组 | 399 组一致；余 1 组为 wrap 边界 1 ulp（见下） |

D 的 1 处差异（**不是**抹平，是分类）：`t0 = 0.12499999999999999`、行 `[FL, RL]`、index `[0, 2]`、
theirs `0` / ours `1`、`min_phase_distance_to_boundary = 0.0`。成因：两侧在该点分别取到
`phase == 1.0` 与 `phase == 0.9999999999999999`，`mod 1.0` 后落到 `0.0`（支撑）与 `0.999…`（摆动）
⇒ 真值同点、1 ulp 取舍。**生产路径是 `half_step=True`（B 项），0 处不一致。**

经验（两次对照式写错，均为我自己的口径错误，非被测代码错误，记下防重复）：
1. 上游 `compute_contact_table` **内部自己**加 `dt/2` ⇒ 「无半拍」侧必须传 `t0 − dt/2`
   （首版写成 `+dt/2` ⇒ 400/400 假不一致）；
2. 判定边界有**两处**：`phase == duty` 与 `phase == 0/1`（wrap）。只查前者会把 wrap 边界
   的 1 ulp 翻转误判成真差异（首版即如此）。

### 2.1 足端参考轨迹移植（已落实现，待逐位校验）

`src/iraf_adapters/unitree/mpc/reference.py::foot_reference_trajectory`（上游 `com_trajectory.py:113-207`
的逐拍状态机语义移植）。三处**必须原样复刻**的上游事实（顺手"修正"就会与它的解不一致）：

1. **参考足端是"机身相对"量**：上游变量名带 `_world`，但每拍都减去了 `p_base_traj_world =
   current_config.base_pos`（= 轨迹第 i 列的机身位置）⇒ 我们对外的量是 `td − base_pos`。
2. **落足点里的 z 是常量 `0.02 m`**，不是机身高度；`T = swing + 0.5·stance`、`pred_time = T/2`。
3. **上游混用坐标系**：`drift` 直接用 `dq[0:3]`（它在 `com_trajectory` 里传的是 `R_world_to_body @ v_world`
   ⇒ **体坐标系**速度）当世界系位移项；`yaw_rate_des_world` 实际赋的也是**体坐标系**角速度。
   偏航非零时这与"世界系"语义不自洽，但它是上游既有事实 ⇒ 逐位复刻并如实登记，不改写。

逐拍状态机（每条腿独立，`mask_previous` 初值 **2**）：跳变到 0（离地）⇒ 记录落足点、当拍参考置零；
跳变到 1（触地）⇒ 取被记录的落足点；掩码未变 ⇒ 递推上一拍值（初值 2 保证第 0 拍不取 `[-1]`）。
⚠ 落足点在**离地拍**的状态下算出并保存 ⇒ 触地拍即使机身位置已变，参考值不随之改变（单测覆盖）。

腿序来源 = `contact.LEG_ORDER`（与接触表行序同一事实，不靠字典迭代顺序）。

**常量已落声明**：`mpc_model.touchdown`（`nominal_z_m` 0.02 / `swing_factor` 1.0 /
`stance_half_factor` 0.5 / `lookahead_factor` 0.5），由 `reference.touchdown_parameters` 派生
`t_swing = (1−duty)·period`、`t_stance = duty·period`、`pred_time = lookahead_factor·T`；
`foot_reference_trajectory` 只收 `nominal_z_m` 与 `pred_time_s`（前瞻时间**只算一处**）。

**逐位校验（已完成，2026-09-23）**：脚本 `build/research/mpc-repo/verify_foot_reference_parity.py`｜
报告 `build/research/mpc-repo/foot_reference_parity.json`（`all_ok: true`，退出码 0）。
基准 = 上游 `convex_mpc.gait.compute_touchdown_world_for_traj_purpose_only`（用它的 `PinGo2Model()`
+ `update_model_simplified(q, dq)` 设状态，hip 偏移取它的 URDF 值）：

| 项 | 规模 | 结果 |
|---|---|---|
| 落足点公式（nominal + drift + rot − base_pos） | 200 随机状态 × 4 腿 = 800 次 | **语义不一致 0 处**；800 处被分类为**往返 ulp**（我们的输出是机身相对量 ⇒ 比较式 `rel + pos` 在浮点上 `(a−b)+b ≠ a`），最大 \|Δ\| = **2.776e-17** ≤ 判据 `4·eps·尺度 = 8.88e-16`，其中 **1479/2400 个分量逐位相等** |

覆盖的状态域：`pos` x/y ∈ ±1.0 m、z ∈ [0.25, 0.29]、`rpy` roll/pitch ∈ ±0.1 rad、yaw ∈ ±π、
体速度 x/y ∈ ±1.0 m/s、`w_body` z ∈ ±1.0 rad/s（含偏航非零 ⇒ 同时校验 `R_z @ hip_offset` 与 `rot` 项）。
沿用的纪律：只打印计数会把"我的对照式写错"伪装成"被测代码错" ⇒ 本脚本打印首个样本的
两侧数值 / 分量级差异 / ulp 判据，并按"语义 vs ulp"两栏分别计数（本轮首版即因对照式差 1 ulp 报 800/800）。

### 2.2 状态相关参考轨迹逐位校验（已完成，2026-09-23）

脚本 `build/research/mpc-repo/verify_state_reference_parity.py`｜报告 `build/research/mpc-repo/state_reference_parity.json`
（`all_ok: true`，退出码 0）。基准 = 上游 `convex_mpc.com_trajectory.ComTraj.generate_traj` 的状态段
（用它的 `PinGo2Model()` + `update_model_simplified(q, dq)` 设状态，用它的 gait 与 `time_step`）。

| 项 | 规模 | 结果 |
|---|---|---|
| `pos_traj_world` / `vel_traj_world` / `rpy_traj_world` / `omega_traj_world` | 200 随机状态 × 4 数组 = 800 次 `tobytes()` 比较 | **全部逐位一致**，ulp 分类 0 处，最大 \|Δ\| = 0.000e+00 |
| 视界长度 | 上游 `N = int(period/time_step)` | **16**，与 `mpc_model.horizon` 一致；`time_step = 0.020833333333` |

状态域：`pos` x/y ∈ ±1.0 m、z ∈ [0.25, 0.29]、roll/pitch ∈ ±0.05、yaw ∈ ±π、体速度 x/y ∈ ±1.0 m/s、
`w_body` z ∈ ±1.0 rad/s；入参 `vx/vy ∈ ±1.0`、`z_des ∈ [0.25, 0.29]`、`yaw_rate ∈ ±1.0`、
`p_des ∈ ±1.5`（含被 ±0.1 m 钳位与未被钳位两种情形）。

结论：`reference.py` 的**两个函数（状态轨迹、足端参考）均通过逐位校验**，可接 `provider_runtime`。
⚠ 三处上游口径在此再次确认（移植时未做任何"顺手修正"）：`pos_des_world` 是**就地钳位并跨调用保留**的状态量；
参考相位里 `z` 由 `z_pos_des_body` 直接覆盖；`vel/omega` 视界内常量。

#### 2.3 第③步 `qp_builder.py` 的校验判据更正（2026-09-23，重要）

计划里原先写的「按 `r2_*` 探针**已存 JSON** 对照 `h_diag/g/a` 三元组与边界」**不成立**：
`ls build/research/mpc-repo/*.json` 后 `grep -l h_diag *.json` **为空** —— 研究侧存下的是
`eval_trot_23-*.json`（闭环时序）与 `r2_*.json`（**只含计时/迭代数/目标值**），
**从未存过 QP 入参**。⇒ 第③步的逐位校验必须先**新写一个用代理截获上游 QP 入参的探针**
（做法可照 `verify_osqp_native.py`：包住上游 `solve_QP` 截获 `h/g/a/lba/uba/lbx/ubx`），
不能指望"已经有证据"。

**已落地的部分（本地校验，未对上游）**：`src/iraf_adapters/unitree/mpc/qp_builder.py::cost_diagonal`
—— `H = 2·tile([q_diag, r_diag], horizon)`，实测 **尺寸 384、nnz 384**（纯对角）、对角跨度
`max/min = 100.0 / 2.0e-5 = 5.0e6`（病态来源，与 §1 的 `r_diag = 1e-5` 一致）；11 项单测
（含 7 项非法声明门禁）。**尚未**与上游 QP 入参比对 ⇒ 该模块**不得**接进 `provider_runtime`。

## 2. 逐位校验点（用已提交的 `traj_parity` 门禁）

对**同一批状态/时间序列**（研究侧已存的 200 样本）比较下列数组的 `tobytes()`：
`Ad`、`Bd`、`gd`、`contact_table`、`pos_traj_world`、`vel_traj_world`、`rpy_traj_world`、
`omega_traj_world`、`r_fl/fr/rl/rr_foot_world`、`compute_x_ref_vec()`、`initial_x_vec`
（**13 项**；`initial_x_vec` 补进门禁后为 14 —— 见调试记录 §28 的口径更正）。

QP 层校验：同一状态/时间下，自有 `qp_builder` 产出的 `h_diag / g / a_rows / a_cols / a_vals /
lbx / ubx / lba / uba` 与研究侧截获值逐位一致（用 `r2_*` 系列探针已存的 JSON 作对照）。

## 3. 常量落声明（铁律 5.3；不得在代码里写数字）

新增到 `config/go2_locomote.yaml` 的 `mpc_model` 段（**待落**，本清单先定义字段）：
`schema_version` 之外新增：`q_diag`（12 个状态代价）、`r_diag`（12 个输入代价，含 `1e-5` 这个
导致病态、进而必须做等价缩放的量级）、`mu`（摩擦系数，**必须与足端 geom 摩擦配对**）、
`horizon`（`GAIT_T/dt = 16`）、`gait_hz`（3.0）、`duty`（0.6）、`swing_height_m`、
`stand_height_m`（0.27）。缺任一键 ⇒ 显式失败。

## 4. 验收判据（A6a-3b）

1. §2 的数组在 200 样本上**逐位一致**（`traj_parity` 门禁，失败即列出首个不一致索引）；
2. 自有 `qp_builder` 产出的 QP 经 `osqp_native` 求解：**`solve` P50 ≤ 4.000 ms**、状态全 `ok`
   （与第 2 块同口径；研究侧同批为 1.9972 ms）；
3. 常量全部来自声明（用 §3 的缺键负向用例验证：删任一键 ⇒ 显式失败）；
4. 既有回归不动：`verify_go2_loopback` 三基准逐位不变。

## 5. 规模与顺序（诚实标注）

预计 `trajectory.py` ~200 行 + `qp_builder.py` ~150 行 + 单测 ~200 行。顺序：
① 常量落声明（1 提交）→ ② `trajectory.py` + 逐位校验（1~2 提交）→ ③ `qp_builder.py` + QP 层逐位校验
（1~2 提交）→ ④ 接 `provider_runtime`（A6a-4）。
