# 计划：MPC 移动 Provider 专项 —— 到期口径 ① 收口 + ②③ 测量

- 建立时间：2026-09-22（本计划由 MPC 专项决策侧窗口建立）
- 依据：`docs/adr/0009-mpc-locomotion-provider-evaluation.md` §6 决策 5 / §6.5（口径已写死）、
  `docs/debug/2026-09-22-mpc-qp-latency.md`（① 的实测与源码依据）
- 状态：① 的判据三条中**只有 P99 成立**，条件 (2)(3) 待补测；②③ 未开始

## 0. 一句话目标

把 ①（QP 时延）判到"通过/不通过"，再按写死的协议测 ②（trot 直行速度跟踪）③（倾角上界）；
三项任一不达标即**收口**（ADR-0009 §6.3），不得在本专项内反复调参。

## 1. 已完成（本轮）

| 项 | 结果 | 证据 |
|---|---|---|
| 依赖隔离环境与自测脚本 | 已建（仓库外、gitignored） | `build/research/mpc-env/`、`build/research/mpc-repo/qp_timing.py` |
| ① 的预算口径判定 | 定死 **20.0 ms（50 Hz 解耦）**：由它实现反推 `int(200 // 48.0) = 4` ⇒ 5 ms × 4 = 20.0 ms | ADR-0009 §6 决策 5 + §6.5 |
| 判据三条 | P99 ≤ 15.0 ms ｜ 越界次数（> 20.0 ms）= 0 ｜ 稳态 max ≤ 20.0 ms（≥2000 次、预热剔除） | 同上 |

## 2. 步骤 1：① 的两项必补测量 —— **已完成（2026-09-22）**

结论：① 按判据**不通过**（wall_step P50 16.899 / P99 18.911 / max 63.811 ms，越界 9 次）；
原 200 次样本 max 19.965 ms 确认为**冷启动**；漏项 `generate_traj` 每拍 4.730 ms。
使用者已授权 **R2**（一次限定实现优化），到期条件与两条分支见
`docs/debug/2026-09-22-mpc-qp-latency.md` §10~§11。

已完成：A 预热分离 + B 计时器包边界（`qp_timing_probe.py`）、GC 归因（`qp_timing_gc_probe.py`，
GC 假设被否）、求解器配置扫描（`r2_solver_sweep.py`）、原生 OSQP 重放（`r2_native_osqp_replay.py`，
**瓶颈 = OSQP 收敛：热启动后 325 次迭代；CasADi/原生 = 1.04 倍**）。

**下一步（R2 剩余）**：① 换求解器（`qpoases` / `proxqp` / `qrqp`，用已截获的同一批 QP 离线对比）；
② 语义等价的预条件/变量缩放；③ 参考轨迹摊销。到期条件：step 口径 P99 ≤ 15.0 ms 且越界 = 0 且 max ≤ 20.0 ms。

改动点（`build/research/mpc-repo/qp_timing.py`）：

- **A. 预热分离**：先跑 `WARMUP = 20` 次不计时（CasADi 函数编译、OSQP 工作区分配、对偶预热
  `lam_a_prev`），再跑 `N = 2000` 次计时；输出预热窗口与稳态窗口**分开**的 P50/P95/P99/max，
  并 dump 稳态里最慢 10 个样本的 `(index, wall_ms, solve_ms)`。
  判读：max 落在预热窗口 ⇒ 条件 (3) 成立，且实现里预热必须放进 init 并写进启动序列；
  max 落在稳态样本 ⇒ ① **不通过**（收口）。
- **B. 计时器包边界**：把 `start` 移到 `traj.generate_traj(...)` **之前**，计时窗口 =
  `generate_traj + solve_QP`（真实每拍计算量），同时保留"只包 `solve_QP`"的对照列。
  参考量（由现有样本反推的被漏项）：mean 差 `11.893 − 10.078 = 1.815 ms`、
  P99 差 `14.102 − 12.267 = 1.835 ms`。
- 顺带（可选，不改判据）：每个样本记录墙钟与是否越界，输出 `overruns_20ms` 计数。

命令（本会话未获授权执行，须由使用者决定）：

```
cd /home/coretek/AIIRAF/build/research/mpc-repo && \
PYTHONPATH=/home/coretek/AIIRAF/build/research/mpc-env:/home/coretek/AIIRAF/build/research/mpc-env/cmeel.prefix/lib/python3.10/site-packages:/home/coretek/AIIRAF/build/research/mpc-repo/src \
/usr/bin/python3 qp_timing.py
```

（静态已核实：`mpc-env/cmeel.prefix/lib/python3.10/site-packages/pinocchio/` 存在、
`mpc-env/cmeel.pth` = `import cmeel_pth` ⇒ 第二个 PYTHONPATH 子路径必需。）

产出：`docs/debug/2026-09-22-mpc-qp-latency.md` 追加"补测 A/B 结果"段（原始数字逐字保留）；
ADR-0009 §6.5 的"现状"一行据结果改为 通过 / 不通过。

## 3. 步骤 2：②③ 的测量协议（已定死，见 ADR-0009 §6.5）

- **摩擦**：两侧同步、**同摩擦配对**判达标 —— (我们 0.4 / 它 0.4)、(我们 0.8 / 它 0.8)；
  交叉点 (我们 0.4 / 它 0.8) 只作灵敏度记录，不作达标依据。
  取值来源：它 `src/convex_mpc/centroidal_mpc.py:15` `MU = 0.8`（README L152 要求 MuJoCo 侧同步）；
  我们 `vendor/unitree_go2/unitree_robots/go2/go2.xml:8`（地板）/`:33`（足端）= 0.4，
  编译后 `build/scenes/handoff_lab/handoff_lab.xml:7-8`/`:31` = 0.4。
  ⇒ 摩擦是**声明参数**，从配置读出、写进报告；禁在脚本里改常量。
- **② 直行速度跟踪**：验收指令速度 ≤ `max_speed_mps = 0.5`（它 ex02 L46 的 0.8 m/s
  只能记为"它的参考工况"）；判据需落新声明段（稳态窗口长度、误差口径、容差数值），
  落地前不得先跑出"好看的数字"再反填阈值。
- **③ 倾角上界**：`max_tilt_moving_deg = 15.0`（`profiles/safety/quadruped_lab.yaml`）。
- 必须同时记录：QP 越界次数、看门狗触发次数、安全事件数（ADR-0009 §4.5），并带**负向对照**
  （QP 失败 / 超时 ⇒ `damped_hold`；急停 ⇒ `torque_zero_release`）。

## 4. 步骤 3：判据落地（新声明段）—— 需要先定归属

- 内容：②③ 的窗口/口径/容差 + ① 的预算与越界计数 + 看门狗/安全事件计数。
- 待定（使用者决策）：声明段落在 `config/`（验收场景声明，随场景走）还是
  `profiles/`（本体/安全侧，随机型走）。倾向 **`config/`**：它们是"本次验收的判据"，
  不是机型固有属性；但如果 ②③ 的口径会随机型变，则应进 `profiles/`。
- 同时产出可复跑判定脚本（`scripts/`，铁律 5.2：易错判定不得靠人工）：
  输入 = 原始样本 + 声明段，输出 = `build/acceptance/` 下的 report.json（含判据逐条 pass/fail）。
- ⚠ 文件边界：本步触碰 `config/` / `scripts/` / `profiles/`，与框架侧窗口可能重叠 ——
  **动手前先划路径**（只 add 自己负责的路径；禁 `git stash/checkout/add -A`）。

## 5. 收口条件（写死，避免长循环）

- ① 条件 (2)(3) 任一不成立 ⇒ ① 不通过 ⇒ 专项收口（记录 max 与样本 index）。
- ②③ 任一不达标 ⇒ 收口，不得在本专项内调参重试。
- 任何一步都不得放宽 `profiles/safety/quadruped_lab.yaml` 的边界。

## 6. 交付 / 协作边界

- `build/research/**` 是使用者的隔离研究环境（gitignored）：本侧窗口不写入、不自动执行其中脚本；
  要复现先报命令由他决定。
- 本侧窗口负责：ADR/计划的文字、口径判定、判据设计；文档写完后报绝对路径。
- 提交：文档侧每次只 add 自己写的路径（本轮已提交 `3085151`：
  ADR-0009 决策 5 + §6.5、`docs/debug/2026-09-22-mpc-qp-latency.md`）。
