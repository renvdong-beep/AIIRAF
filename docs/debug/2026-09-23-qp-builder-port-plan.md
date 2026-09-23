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
