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

**尚缺（下一步）**：`nominal_z_m`（0.02）与 `pred_time` 的两个系数目前由调用方传入，
须落进声明（`mpc_model` 的 touchdown 组）后才能接 `provider_runtime`；本模块自身的逐位校验
（对上游客体在同一批状态上比较四足 `r_*_foot_world`）也尚未做。

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
