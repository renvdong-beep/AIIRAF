"""四足 MPC Provider（独立进程）—— 平台相关适配层。

为什么在这里：铁律 4「重计算不得进实时闭环」、铁律 3「框架核心不得依赖某块板/某个求解器」
⇒ MPC 与 CasADi/OSQP/Pinocchio 只能存在于 `adapters/`，且默认为**独立进程 Provider**，
通过版本化接口与 `TaskFlow → SkillRuntime → PolicyGateway` 通信；`src/iraf_core/` 不引入任何求解器依赖。

本包的落地顺序（第一块已完成）：
  1. `qp_scaling`：**等价变量缩放**（解不变，只改数值形态）—— R2 量化收益：OSQP 迭代 P50
     340 → 60、solve P50 9.8146 → 1.4307 ms（见 `docs/debug/2026-09-22-mpc-qp-latency.md` §13）；
  2. `osqp_native`：原生 OSQP 调用（避开 CasADi 包装，实测再省 ≈2.7 ms/拍，§16）；
  3. `traj_amortize`：参考轨迹**保值摊销**（删除无读取方的 `Ac/Bc/gc`，逐位一致，§15）；
  4. `provider`：独立进程 Provider 外壳（状态新鲜度门禁、QP 失败/超时显式失败路径）。

接入要求（来自 §25，硬性）：被控对象**不得**从「足端穿透支撑面」的初始条件起跑；
必须先走框架的 `initial_alignment`（`lift_lowest_geom_to_support_plane`，实测抬升 0.018372500 m）。
"""
