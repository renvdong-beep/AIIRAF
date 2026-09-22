# ADR-0009（草案 / PROPOSED）：引入模型式 MPC 作为移动 Provider 的评估与边界

- 状态：**草案（PROPOSED）**，未决。需要人工决策（见 §6）。
- 日期：2026-09-21
- 相关：ADR-0008（四足移动与到点边界，决策 1「首期步态 = 准静态 wave/crawl」）、
  `docs/progress/2026-09-21-go2-locomotion-references.md`（调研笔记）、
  `docs/debug/2026-09-21-quadruped-crawl-foothold.md`（专项 C 的 13 步实测记录）

## 1. 背景

专项 C（落足点规划 / 迈步式 crawl）已按到期口径收口为 `DEFERRED`：实测把障碍定位到**落点计划**层
（RR 相位 100% 帧的支撑余量为负、机身被拖 23.69 mm、过约束），且**6 条解释已被排除**
（trot 动力学、关节空间重心平移、占位方向、触地速度/过渡提前、力矩能力、斜坡）。
继续在"准静态 crawl"这条路线上推进需要求解"机身位移 + 落点"的协同规划，属架构级工作。

外部参考（调研笔记 §1）：`elijah-waichong-chan/go2-convex-mpc`（★129，**MIT**，Python，MuJoCo，Go2）
实现了 **MIT Cheetah 3 的凸 MPC**（接触力型 MPC，CasADi + OSQP），其控制栈**本身就同时解算
"质心（机身）运动 + 足端接触力"**，落足点用 **Raibert 式规划**、摆动腿用**最小 jerk 五次多项式**。

## 2. 该方案与我们阻塞点的关系（为什么值得评估）

- 我们的阻塞是"**机身与足端不同步**"（落点先行、机身被拖）。MPC 的机制正是
  **用同一个 QP 同时决定机身运动与接触力** ⇒ 协同不是补丁，而是它的结构本身。
- 它走的是**动态**路线（trot、duty 0.6、实测 3.0 Hz），不依赖静态支撑余量
  ⇒ 绕开"四相位余量必须全为正"这条我们已量出做不到的约束。
- 作者给出的**实时性证据**（README）：平均模型更新 ≈1.03 ms、QP 求解 ≈1.67 ms、
  MPC 周期 ≈2.70 ms，落在 48 Hz（20.8 ms）预算内；并有"1000 Hz → 200 Hz 无性能退化"的更新记录。
  ⇒ 与"需 GPU"的 RL 路线不同，它是**纯 CPU** 路线。

## 3. 与现行决策的冲突（必须先说清）

- **推翻 ADR-0008 决策 1**：首期步态由「准静态 wave/crawl」改为「动态 trot + MPC」。
- **判据不适用**：专项 C 建的 13 项 crawl 判据（落点误差 / 接触达标率 / 周期净漂移 / 峰位移）
  是按静态 crawl 语义写的；动态 trot 路线需要**另一套判据**（见 §4.5）。
- **既有 20 项原地判据**（`gait.verification`）仍保留强制（它们度量的是"没有偷偷走出去/不塌陷"），
  但 trot 的 duty 0.5 与 wave 的 duty 0.75 不同 ⇒ 需要新的声明段而不是改旧阈值。

## 4. 引入的落地约束（逐条满足，缺一不得上线）

1. **依赖可达性（已核实 ✓）**：`casadi 3.8.1`、`osqp`、`pinocchio` 在 aliyun 镜像上
   **x86_64 与 `cp310 + manylinux2014_aarch64` 均有 wheel**（实测下载成功，casadi wheel 54.2 MB）；
   本机当前 `mujoco 3.3.3 / numpy 2.2.6 / scipy 1.15.2` 已有，三个新依赖均缺。
   ⇒ 可进离线 wheelhouse 与 aarch64 交付物，但**体积与冷启动开销必须实测**并在 BoardProfile 登记。
2. **模型等价性（未做，必做）**：该仓库自带 `models/MJCF/go2/go2.xml` 与 URDF；
   我们在用的是**厂商 MJCF 编译进场景**（`build/scenes/handoff_lab/`）。
   必须逐项对照：整机质量、关节限位、执行器 `ctrlrange`、足端接触几何半径、摩擦系数、
   关键帧；**任何差异**都要写明（含对结论的影响），不得默认等价
   （§8 的教训：我们连 hip/thigh 的 `ctrlrange` 都还没解析全）。
3. **安全包线不可放宽**：MPC 的输出（关节力矩/位置目标）必须经
   `TaskFlow → SkillRuntime → PolicyGateway`；速度/倾角/工作空间上限取自
   `profiles/safety/quadruped_lab.yaml`（`max_speed_mps 0.5`、`max_tilt_moving_deg 15.0`、
   `workspace_m`、`watchdog_timeout_ms 100.0`）；**QP 无解、超时或状态过期必须有显式失败路径**
   （移动中 `damped_hold`、急停 `torque_zero_release`），不得降级为"继续跑旧解"。
4. **Provider 形态**：MPC 与 CasADi/Pinocchio 只能存在于 `adapters/`（独立进程优先），
   **不得**把求解器依赖带进 `src/iraf_core/`；机型差异只进 `profiles/`+`config/`。
5. **判据（需新声明段）**：速度跟踪误差、倾角上界、接触力上界（防砸地）、
   **QP 求解时延 P99 与超时次数**、看门狗触发次数 = 0、安全事件 = 0、
   以及"停止语义"的正/负路径；全部落进声明并带负向对照。
6. **不得引用他人性能数字**：README 的 0.8/0.4 m/s、4.0 rad/s、2.70 ms 是**作者环境**下的数字；
   在本项目内必须自测（本机 CPU + 我们的场景 + 我们的模型），测不出就说测不出。

## 5. 风险清单

| 风险 | 说明 |
|---|---|
| 依赖重量 | CasADi(54 MB wheel) + Pinocchio + OSQP 进目标板，磁盘/内存/冷启动都要量；Pinocchio 的 aarch64 wheel 实际可用性要在板上复验（本次只在镜像侧核实了可下载） |
| 模型差异 | 若他们的 MJCF 与我们厂商模型存在质量/限位/摩擦差异，性能与稳定性结论不可直接迁移 |
| 实时性 | 2.70 ms 是作者机器；我们的场景（含台面/其它物体）与 CPU 不同，需自测 P99 与是否能在 200 Hz 环路内稳定运行 |
| 路线反复 | 从准静态改动态 = 推翻决策 1；若 MPC 路线也失败，需要有明确的收口条件，避免又一次长循环 |
| 源码获取 | 本机 `github.com` 的 git 协议被卡（3 次 clone 失败：TLS 断流/443 超时），`raw.githubusercontent.com` 不可达；只能走 `api.github.com` contents API（未认证限流 60/小时）逐文件取 ⇒ 取回完整源码需要时间或换镜像 |

## 6. 待决项（人工决策，代理不选路）

1. 是否接受**首期由准静态改动态 trot + MPC**（即修订 ADR-0008 决策 1）？
2. 若接受：专项 C 是否就此收口（`DEFERRED`），并**新开 MPC 专项战役**（含依赖入 wheelhouse、
   模型等价性对照、自测实时性与判据落地）？建议的**到期口径**：以"本机自测 QP P99 时延 +
   trot 直行速度跟踪误差 + 倾角上界"三项为第一阶段验收，任何一项不达标即收口。
3. 若不接受：维持准静态路线，则下一步仍是 §13 结论所指的"机身位移 + 落点协同规划"
   （需新的架构级设计，不在专项 C 内继续试参数）。

## 8. 已核实的实现事实（2026-09-21 补充：源码已取回并读过）

取源路线（本机实测）：`github.com` 的 git 协议与镜像、`raw.githubusercontent`、`codeload`、`ghproxy.net` 的
tar.gz **全部不可用**；`api.github.com` 可达但未认证限流 60/小时且仓库 155 MB（jsDelivr 清单被 50 MB 限制拒绝）；
**可用路线 = `cdn.jsdelivr.net` 单文件取**（实测 HTTP 200、0.66 s）。已取回并读过：

| 文件 | 大小 | 已读到的事实 |
|---|---|---|
| `src/convex_mpc/gait.py` | 6.4 KB | `HEIGHT_SWING = 0.1`（摆动顶点 0.1 m）；`Gait(frequency_hz, duty)`：`period=1/f`、`stance_time=duty·period`；`compute_contact_table`：`phases = mod(PHASE_OFFSET + t/period, 1)`、**`contact = phases < duty`** —— 与我们的 `is_stance(phase) = phase < duty` **同构**；`compute_swing_traj_and_touchdown`：**Raibert 式落足点**（`hip_pos_world = body_pos + R_z @ hip_offset` + 速度项），`make_swing_trajectory(p0, pf, t_swing, h_sw)` |
| `src/convex_mpc/centroidal_mpc.py` | 13.0 KB | `COST_MATRIX_Q = diag([1,1,50,10,20,1,2,2,1,1,1,1])`、`COST_MATRIX_R = diag([1e-5]*12)`（12 输入 = 4 足 × 3 维接触力）；求解器 **CasADi `ca.conic('S','osqp',...)`** + **对偶预热启动**（`lam_a_prev`）；`_precompute_friction_matrix`（摩擦锥不等式，静态）；`_build/_update_sparse_matrix`（提速关键）；`_compute_bounds` 用**接触表**（`contact = phases < duty`）决定哪些腿出力；自带 `solve_time` 统计打印 |
| `src/convex_mpc/go2_robot_data.py` | 13.7 KB | **`pinocchio.robot_wrapper.RobotWrapper.BuildFromURDF`**，模型来自 `models/URDF/go2_description/urdf/go2_description.urdf`（**不是 MJCF**）；足端 frame 名 `FL_foot_joint` / `FR_foot_joint` / `RL_foot_joint` / `RR_foot_joint`；`get_hip_offset(leg)`、`compute_com_x_vec()`、`update_model(q, dq)` |
| `examples/ex00_demo.py` … `ex04_*.py` | ~10.5 KB × 5 | 五个自包含例程（原地 trot / 直行 / 侧行 / 转向 + 综合 demo），各含完整控制回路与绘图 |
| `models/MJCF/go2/go2.xml` / `scene.xml` | 14.9 / 1.5 KB | 它自带的 Go2 MJCF 与场景（**可用于与厂商 MJCF 的对照**，尚未开始） |
| `pyproject.toml` / `environment.yml` | 364 / 446 B | 包在 **`src/`** 布局；项目名 `convex-mpc-unitree-go2` v1.0.0；**`requires-python >=3.10,<3.11`**；conda-forge 依赖：`python=3.10`、**`numpy<2`**、**`pinocchio`**、`casadi`、`scipy`、`matplotlib>=3.7,<3.9`、`eigenpy`/`hpp-fcl`/`boost-cpp`/`assimp`/`urdfdom`/`eigen`/`cmake`/`ninja` 等；pip 侧 **`mujoco==3.1.6`** |

### 8.1 由此得到的三条硬结论

1. **必须独立环境/独立进程**：它要求 **`numpy<2`**（我们是 `numpy 2.2.6`）与 **`mujoco==3.1.6`**（我们是 `3.3.3`）
   ⇒ 直接装进我们现有环境会**改坏**现有仿真栈。这与我们的铁律（跨发行版/第三方 Provider **默认独立进程**）
   正好一致 —— 把 MPC 作为独立 Provider 进程、独立 venv，依赖冲突被隔离，也不会污染 `src/iraf_core/`。
2. **建模路线不同，必须做对照**：它用 **URDF + Pinocchio**，我们用**厂商 MJCF**（编译进场景）。
   对照项至少：整机质量与惯量、关节限位、足端 frame 定义（`*_foot_joint`）、接触几何、摩擦系数。
   在对照完成前，**不得**把它的性能数字或稳定性结论迁移过来。
3. **相位约定与我们同构（好消息）**：它的接触判定 `phases < duty` 与我们的 `is_stance` 完全同形，
   `PHASE_OFFSET` 与我们的 `gait.legs.*.phase_offset` 是同一族表达 ⇒ 我们的声明模型可以直接承载
   它的步态参数（频率、duty、相位偏移），不需要为它造第二套相位语义。

### 8.2 新增的落地风险（由 8.1 得出）

| 风险 | 说明 |
|---|---|
| 环境隔离成本 | 需要独立 venv（conda-forge 依赖链很长：pinocchio/eigenpy/hpp-fcl/boost/assimp/urdfdom…）；aarch64 目标板上 **conda-forge 依赖是否可得未核实**（本轮只核实了 pypi 侧 `pinocchio`/`casadi`/`osqp` 的 aarch64 wheel 可下载） |
| 版本分叉 | 我们的 `mujoco 3.3.3` 与它的 `3.1.6`、我们的 `numpy 2.x` 与它的 `numpy<2` 长期并存 ⇒ 两套仿真栈要各自锁定版本并在 BoardProfile/发布清单里登记 |
| 对照工作量 | MJCF ↔ URDF 的模型对照是**前置必做项**，不是可选优化 |

## 7. 诚实边界

- 本 ADR 为**草案**：所有外部工程的能力描述来自其 README 与源码结构阅读，
  **未在本项目内运行**过 `go2-convex-mpc` 的代码；性能数字均标注为"作者环境"。
- 依赖可达性只在**镜像侧**核实（能下载到 wheel），**未**在 x86_64 本机与 aarch64 目标板实际安装
  并跑通导入。
- 完整源码尚未取回（网络受限），模型等价性对照因此**尚未开始**。
