# A6a-④（本轮）：把 MPC 解接到执行器级钩子 —— 增量清单

- 日期：2026-09-23
- 依据契约：`docs/debug/2026-09-23-locomote-provider-integration-spec.md` §1/§2/§4/§6
  + `docs/debug/2026-09-23-qp-builder-port-plan.md` §1.2（接入契约，带 file:line）
- 已提交（上一轮）：`0a308bf` `mpc/torque_provider.py`（纯映射层：足端力→关节力矩 + B1 载荷）

## 拆成 4 个可独立验收的增量（本轮做 ①，之后不暂停继续 ②）

| # | 事项 | 判据 | 交付物 |
|---|---|---|---|
| ① | **binding 层** `mpc/torque_hook.py`：`ProviderRuntime` 的裁定 → `_run_control` 的 `torque_provider(cycle_index, info)` 口子（B1 载荷）；无可用解 ⇒ 抛 `MpcUnavailableError`（携带 decision/reason/diagnostics），**绝不复用旧解** | 单测：正常拍载荷逐位手算对上；四类负向（QP `max_iter` / 子进程超时 / 解龄过期 / 急停）各自的 decision 与"不沿用旧解"；`position_weight` 门禁 | `src/iraf_adapters/unitree/mpc/torque_hook.py` + 单测 |
| ② | **状态→QP 请求** `plan_fn`（`dynamics.discrete_dynamics` + `reference` + `qp_builder.assemble_qp` → `protocol.build_request`）+ `locomote()` 去掉显式拒绝，走正式路径（`_run_control` 一行不改） | 端到端：`locomote` 正路径推进运动（末速/倾角在声明内）；负路径终态 `STOPPED`/`SAFETY_STOP` | `unitree_go2.py` 的 `locomote()` + 适配器内的 plan/jacobian 注入 |
| ③ | 契约 §4 逐行闭环（终态映射：`damped_hold`⇒`STOPPED`/`FAILED`、急停⇒`SAFETY_STOP`）+ 4 条负向用例的**真子进程**版本 | `scripts/verify_mpc_locomote_failure_paths.py` ⇒ 退出码 0 + 报告落 `build/acceptance/` | 脚本 + 报告 |
| ④ | 回归：站立态 `stop` 与 loopback 三基准逐位不变 | 三基准 `0.279953602548388 / 3.302184116303541e-05 / 0.0038248382123762478` 逐位相等 | 既有 `scripts/verify_go2_loopback.py` |

## 本轮（①）的两处口径决定（写下来，避免下轮重读）

1. **`will_update(emergency=…)` 只读化**：`ProviderRuntime` 内部 `do_update = (tick % tpu == 0)`；
   为让"只在更新拍才构造 QP 请求"（否则 100 Hz 白造 384 变量 QP）成为**同一份事实**，
   在该类上加只读方法 `will_update(emergency=False)`（不改状态），并在装配层用它。
   `do_update` 改为 `… and not emergency`：**急停拍不该再花一次子进程预算**（铁律 6.6 优先级）。
   钩子侧再做一次自洽校验：`diagnostics["source"] == "subprocess"` 必须与"本拍是否构造了计划"一致，
   否则显式失败（防止 tick 计数器被别的调用方推进导致错拍）。
2. **解向量布局只引用 `qp_builder` 的常量**：`z = [每拍状态 12]×N ‖ [每拍输入 12]×N`，
   力段起点 `N·STATE_DIM`，段内按 `LEG_ORDER` 排腿（与 `assemble_qp` 列序同一事实）。
   本层不重写 12/4/3 这些数字。

## 已知不承诺项（本轮）
- 本层**不**决定终态（`STOPPED`/`FAILED`/`SAFETY_STOP` 由 ② 的调用方映射），也不碰控制器；
- 本层**不**判断支撑集口径（接触表来自 `contact.py` 对 trot 声明的消费，本层只消费本拍那一列）。
