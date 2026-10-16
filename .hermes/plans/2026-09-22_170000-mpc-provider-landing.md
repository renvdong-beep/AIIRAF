# 计划：MPC Provider 落地（adapters/ 独立进程）—— 四块 + 接入要求

- 建立：2026-09-22（MPC 专项决策侧窗口）
- 依据：`docs/adr/0009-mpc-locomotion-provider-evaluation.md` §6 决策 5/6 + §6.5；
  `docs/debug/2026-09-22-mpc-qp-latency.md` §11~§25（R2 全过程实测）
- 边界：**新增** `src/iraf_adapters/unitree/mpc/`（不与现存文件重叠）；测试放 `tests/unit/test_unitree_mpc_qp_scaling.py`
  （已建）与后续 `tests/unit/test_unitree_mpc_*.py`；只 add 自己负责的路径；禁 `git stash/checkout/add -A`。

## 已完成

| 块 | 内容 | 证据/判据 | 状态 |
|---|---|---|---|
| 1 | `qp_scaling`：等价变量缩放 `z = D⁻¹w`，`D = diag(√H_ii)`（纯 numpy，无求解器依赖） | 单测 7 项全过（往返/目标值/行约束/盒约束/对偶/失败路径）；实测收益 iter 340→60、solve 9.8146→1.4307 ms（§13） | **DONE**（提交 `0dc8ff5`） |

单测建成即抓出两处同源数学错误（变量前向应乘 `D`、盒约束对偶回代应乘 `D`），已修正并把 KKT 推导写进 docstring。

## 待做（按序）

### 2. `osqp_native`（原生调用，绕开 CasADi 包装）
- 动机（§11.2/§16）：CasADi 包装与原生比值 **1.04**（同批 QP、纯 solve 口径），但**缩放后**走 CasADi
  仍 4.1580 ms vs 原生 1.4307 ms ⇒ 包装费 ≈2.7 ms/拍成为主项。
- 接口草案：
  - `build_problem(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba)` → 缩放后的 `P, q, A, l, u`
    （`A_full = [I; A]`，行序与 OSQP 约定一致；±inf 原样传递）
  - `solve(problem, prev=None, settings=None)` → `(x, y, status, iter, wall_ms)`；`prev` 提供热启动
  - `update_in_place(solver, q, l, u, Ax)`：稀疏模式不变时走 `update()`（避免每次 `setup()` 的 ≈1.5 ms）
- **依赖策略（硬性）**：`osqp`/`scipy` 只能**延迟导入**（函数体内），缺失时抛
  `RuntimeError("本模块需要 osqp>=1.0（独立 Provider 环境）；当前环境缺失 ⇒ 显式失败")`，
  **不得**让仓库主环境的 pytest 因缺依赖而失败（缺声明即显式失败，不静默降级）。
- 可测部分（无需 osqp）：问题组装（`[I; A]` 行序、l/u 拼接、±inf 保持）、设置映射、
  状态字符串映射与失败分类、热启动向量长度校验 ⇒ 先写这些单测。
- 收尾判据：对 §11.2 截获的同一批真实 QP 重放，`solve` P50 ≤ **4.000 ms** 且状态全 solved。

### 3. `traj_amortize`（参考轨迹保值摊销）
- 做法（§15，已证）：**删除无读取方的 `Ac/Bc/gc`**（全仓库 grep 无读取方），保留全部被消费数组。
- 判据：被消费的 14 个数组 × 200 样本 `tobytes()` **逐位一致**；`generate_traj` P50 由 4.7170 → ≤ **3.0 ms**。

### 4. `provider`（独立进程外壳）
- 硬性（ADR §4.3/§6.5）：**解新鲜度门禁**（解龄 > 2 个 MPC 周期 = 41.667 ms ⇒ `damped_hold`）；
  QP 失败/超时 ⇒ 显式失败路径（移动中 `damped_hold`、急停 `torque_zero_release`），
  **禁止**"继续跑旧解"（本专项的探针犯过这个错，见 §20/§21）；
  版本化接口，跨进程只传声明过的消息（不把 `casadi.DM` 等私有类型穿透边界）。
- 失败路径必须有负向用例（注入 QP 失败/超时/状态过期，断言终态正确且不伪造成功）。

## 接入要求（§25 硬性，已写进包 docstring）
被控对象**不得**从"足端穿透支撑面 18.372 mm"的初始条件起跑，必须先走
`initial_alignment`（`lift_lowest_geom_to_support_plane`，实测抬升 0.018372500 m）。
理由：该初始条件使仿真处于浮点舍入级敏感（5 m 的 x 位移差即改变舍入序列 ⇒ 1e-5 分叉 ⇒ 1.5 s 跌倒）。

## 阻塞/待裁决（不属于本计划，但影响能否收口）
1. **② 的判据**：口径（窗口/均值还是 P95/容差）+ 落 `config/`（推荐）还是 `profiles/` +
   是否把"移动中 QP 越界"记进判据。实测 ② 误差均值 +6.95%~+7.94%、P95 0.1022~0.1036 m/s。
2. **目标板 aarch64 复测**：① 的最终证据（本机只判"可继续"）；板卡 10.203.247.72 / .86 当前不通。
3. **交付形态**：MPC 进仓库后，`capabilities` 回填必须**在 ②③ 通过且板上复测后**才可声明（铁律 6.8）。
