# `locomote` Provider 接入契约（A6a）—— 独立进程 MPC + 门禁

- 日期：2026-09-23
- 依据：ADR-0009 §4.3/§4.4/§6.5、铁律 3/4/6.6/12、`config/go2_locomote.yaml`（② 判据）、
  `docs/debug/2026-09-22-damped-hold-spec.md`（停止语义）
- 状态：**契约先行**（实现待做）。本文件只定接口、边界与判据，不含实现。

## 1. 形态（写死）

- MPC 求解在**独立子进程**里（`python -m iraf_adapters.unitree.mpc.worker`），父侧用
  `MpcProcessClient`（已实现、已测：超时/崩溃/矛盾响应 ⇒ `damped_hold` 且无解）。
- 父侧只传**普通 JSON 类型**（`protocol.py` 保证：数字列表/字符串/布尔/None）；`casadi.DM`、
  scipy 稀疏对象、numpy 数组**不得穿透边界**。
- 框架核心（`src/iraf_core/`）**不引入** `osqp`/`scipy`/`pinocchio`/`casadi` 任何依赖。

## 2. 一次控制拍的数据流（每 2 拍更新一次 MPC，见 §3）

```
控制拍(100 Hz)
 ├─ 读本机状态（MuJoCo/真机）→ 组请求（h_diag/g/a 三元组/lba,uba/lbx,ubx）
 ├─ client.call(请求, timeout_ms=…)          # 独立进程，父侧不阻塞超过 timeout
 ├─ protocol.validate_response(...)          # 版本/类型/契约（失败却带解 ⇒ 拒收）
 ├─ freshness.decide(age_ms, status_class, solve_ms, emergency)
 │    ├─ "ok" ⇒ 用解（z）喂腿控
 │    ├─ "damped_hold" ⇒ 走已实现的 stop(mode="damped_hold")（§4）
 │    └─ "torque_zero_release" ⇒ 立即松力（急停/安全事件）
 └─ 记录：execution_id / 解龄 / 状态分类 / 迭代数 / 求解耗时 / 是否越界
```

- **解龄** `age_ms = now − 最近一次成功求解时刻`；阈值 `STALE_LIMIT_MS = 2 × 20.0 = 40.0 ms`
  （`freshness.py` 的单一事实来源）。
- **禁止**在失败/过期时沿用上一拍的解（本专项的探针犯过此错，见调试记录 §20/§21）。

## 3. 时序（与声明一致）

| 项 | 值 | 来源 |
|---|---|---|
| 关节控制频率 | 100 Hz | `profiles/unitree_go2_mujoco.yaml` 的 `control_frequency_hz` |
| MPC 更新 | 50 Hz（每 2 拍） | ADR-0009 §6.5（预算 20.0 ms） |
| MPC 内部 dt | `GAIT_T/16 = 0.020833 s` | 不改（整除关系，见 §6.5） |
| 参考相位 | 按**实测时间**推进 | 同上（否则步频漂 4.17%） |
| 默认调用超时 | 200.0 ms（配置项，不写死在业务码） | 待落 `config/`（与 §2 的记录项同处） |

## 4. 失败路径（每一条都要有负向用例）

| 触发 | 动作 | 终态 |
|---|---|---|
| QP 状态分类非可用 / 子进程崩溃 / 超时 / 响应非法 | 走 `stop(mode="damped_hold")` | 移动中停住 ⇒ `STOPPED`；停止未达标 ⇒ `FAILED` |
| 解龄 > 40.0 ms（含"未求解"） | 同上 | 同上 |
| 响应非法（`decision=ok` 却带不可用解：长度/数值非法） | 同上 | 同上 |
| **本拍计划不可用**（`plan_fn` 抛错或缺键 ⇒ 构造不出 QP，2026-09-23 由 `torque_hook` 的实施补齐） | 同上 | 同上 |
| 急停 / 安全事件 | `torque_zero_release`，作废当前解（`ProviderCore.invalidate`） | `SAFETY_STOP` |
| 停止未完成时其它控制源申请接管 | 按租约/fencing token **拒绝** | — |

失败**原因**必须可追溯（否则报告里只剩 `freshness` 的判词，看不出是超时/崩溃/响应非法中的哪一类）：
`ProviderRuntime` 把 `client_error`（客户端判定）与 `response_reason`（子进程原因）一并放进
`diagnostics`，`MpcUnavailableError.reason` 带上两者（`src/iraf_adapters/unitree/mpc/torque_hook.py`）。

## 5. 接入要求（硬性）

- **初始条件**：不得从"足端穿透支撑面 18.372 mm"起跑，必须走 `initial_alignment`
  （否则仿真处于浮点舍入级敏感，调试记录 §25）。
- **能力回填**：`profiles/unitree_go2_mujoco.yaml` 的 `capabilities` 追加 `locomote`
  **必须**满足：① 本机 ②③ 判定通过（② 硬判据已落 `config/go2_locomote.yaml`）；
  ② **aarch64 目标板复测通过**（铁律 6.8；本机结论只判"可继续"）。
- **安全边界**：`max_speed_mps 0.5` / `max_yaw_rate_rad_s 1.0` / `max_tilt_moving_deg 15.0`
  只来自安全策略；调用参数只能收紧。

## 6. A6a 的验收判据（实现完成后逐条取证）

1. 独立进程可启动、协议校验生效（含"失败却带解"被拒收的正/负用例）；
2. 正常拍：解被消费并推进运动（末速/倾角在声明内）；
3. 注入 QP 失败/超时/解过期 ⇒ 走 `damped_hold`，**且不出现"继续跑旧解"**（负向用例）；
4. 注入急停 ⇒ `torque_zero_release` + 解作废 + 终态 `SAFETY_STOP`；
5. ② 硬判据（直行/倒退误差均值 ≤ 10%）在本机复算通过；
6. 回归：既有 `stop`（站立态松力）与 loopback 三条基准**逐位不变**。

## 7. 已知限制（诚实标注，写进报告）

- 转弯/侧行的跟踪效率（70.0% / 120.4% 超调）属**传力效率上限**（成因=FR/RL 对角对，见 §30/§31）⇒
  A6a 不承诺其达标，只在报告里按 `config/go2_locomote.yaml` 的"仅记录"项输出。
- 移动中的 `damped_hold` 仿真级证据（spec N1 的真实场景版）**依赖本项的 locomotion 通路**，
  故它在本项完成后才能补齐（原 A1b2，已登记为本项下游）。
