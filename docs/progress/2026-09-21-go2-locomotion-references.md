# Go2 / 四足行走可借鉴的开源工程（调研，2026-09-21）

- 目的：为专项 C（落足点规划）的**阻塞点**（"落点计划本身静态不可稳"，实测 RR 相位 100% 帧支撑余量为负，
  见 `docs/debug/2026-09-21-quadruped-crawl-foothold.md` §13）找可借鉴的成熟做法。
- 约束：纯 CPU + MuJoCo 3.3.3、确定性控制器、禁厂商 SDK 与机型特例进核心（只能进 `adapters/`+`profiles/`）、
  声明驱动、可复跑判据。
- 网络实测：`github.com` / `api.github.com` **可达**；`raw.githubusercontent.com` **不可达**（取文件要走 contents API）。

## 1. 最贴合我们约束的两个（先看这两个）

| 工程 | 数据 | 为什么 |
|---|---|---|
| `khaledgabr77/unitree_go2_ros2` | ★173，C++，未标 License，2026-09-15 | Go2 的 **CHAMP 确定性步态控制器**完整集成（ROS 2 Jazzy）。**已 clone 成功**（41M）到 `build/research/go2-locomotion/unitree_go2_ros2/`；其 Go2 步态声明可直接与我们对照：`swing_height 0.04`、`stance_depth 0.01`、`stance_duration 0.25`、`nominal_height 0.225`、`max_linear_velocity_x 0.3 / y 0.25 / z 0.5`、`com_x_translation 0.0`、`odom_scaler 0.9` |
| `elijah-waichong-chan/go2-convex-mpc` | ★129，Python，**MIT**，2026-04-17 | "凸 MPC 在 MuJoCo 里控制 Go2"——**唯一同时满足** MuJoCo + Go2 + 确定性模型式控制 + Python + 宽松许可的完整控制器。（本机两次 clone 均因网络中断失败，需要走 API 取源码或重试） |

## 2. CHAMP 框架（确定性静态步态的成熟参考）

CHAMP 是四足"非 RL"控制的事实标准，核心（gait / body / step 控制器）**不在主仓库**，在：

| 仓库 | 数据 | 内容 |
|---|---|---|
| `chvmp/libchamp` | ★36，C++ 头文件库 | `gait/`、`body_controller/`、`step_controller/`、`kinematics/`、`quadruped_base/`（**核心**，已取回 `build/research/go2-locomotion/champ-core/`） |
| `chvmp/pychamp` | ★13，**Python** | 同结构 Python 版：`champ/gait.py`、`champ/body_controller.py`、`champ/kinematics.py`（**与我们同语言，最易借鉴**） |
| `chvmp/champ` | ★2317，BSD-3 | ROS 节点封装（不含核心步态算法） |
| `chvmp/robots` | ★277 | 各机型 CHAMP 配置集合（含多种四足） |
| `chvmp/champ_setup_assistant` | ★93 | 交互式生成机型配置（含 `gait_config.h` 模板） |

**已读到的两个关键机制**（`pychamp__champ__gait.py`、`libchamp__body_controller__body_controller.h`）：

1. **相位延迟步态日程**：`stride_period = stance_duration + swing_duration`；每腿有自己的时钟
   `leg_clock = elapsed − phase_delay[leg]·stride_period`，再由 `leg_clock` 落在 `(0, stance)` 还是
   `(−swing, 0)` 决定支撑/摆动相。trot 的 `phase_delay = [0.0, 0.5, 0.5, 0.0]`（对角同相），
   另有 `phase_delay_zero = [0.5, 0.0, 0.0, 0.5]` 用于**站立↔行走的过渡**。
   ⇒ 与我们 `gait.legs.*.phase_offset` + `duty_factor` 是同一族表达（我们已具备），
   但**过渡态是显式建模**的（我们目前靠 `ramp_s` 渐变，没人管过渡期的相位重排）。
2. **机身位姿命令 = 把四条腿目标统一反向平移**：`body_controller::poseCommand` 对每条腿算
   `req_translation = −req_pose.position.{x,y,z}`（并限幅 `max_translation_z = −zero_stance.Z()*0.65`），
   即"要机身往哪走，就让所有足端目标往反方向移"。
   ⇒ **这正是我们的 `sway`**；差别是 CHAMP 把它作为**主命令**（机身轨迹先行、足端跟随），
   而我们目前是"落点先行、机身被拖"（实测 0.4 s 被拖 23.69 mm、RR 相位余量为负）。

## 3. 经典理论与判据（我们已在量同一个量）

- **McGhee & Frank 1968**《On the stability properties of quadruped creeping gaits》定义了爬行步态的
  **纵向稳定余量（longitudinal stability margin）**；**Song & Waldron** 给出了支撑多边形稳定余量的通用定义。
  ⇒ 我们 §13 实测的"重心投影到三腿支撑三角形的有符号余量"**就是**这个量；标准解法是
  **固定抬腿顺序 + 机身纵向前后位移**（而不是让四条腿各自为政）—— 与 §10.3/§13 的实测结论完全一致。
- 可查的入门资料：Google Scholar 上的 McGhee & Frank 1968 原文；任何四足/六足教材的"爬行步态"章节。

## 4. RL 路线（能走通，但与我们的架构冲突）

| 工程 | 数据 | 备注 |
|---|---|---|
| `unitreerobotics/unitree_rl_gym` | ★3562，BSD-3 | 官方 RL 训练环境；**Isaac Gym ⇒ 需 GPU** |
| `unitreerobotics/unitree_mujoco` | ★1201，BSD-3 | **官方 MuJoCo 仿真**（模型/scene/SDK 桥接，纯 CPU 可跑）——可用来交叉验证我们的模型与场景保真度 |
| `google-deepmind/mujoco_playground` | ★2224，Apache-2 | MJX 加速的 RL 环境库（含四足 locomotion） |
| `google-deepmind/mujoco_menagerie` | ★4094 | 含 `unitree_go2` 高质量模型（XML/执行器建模可与厂商 MJCF 对照） |
| `Glowing-Torch/Deploy-an-RL-policy-on-the-Unitree-Go2` | ★39，MIT | ROS 2 + **MuJoCo 验证**后切真机（`is_simulation` 参数） |
| `alexeiplatzer/unitree-go2-mjx-rl` | ★21 | Go2 的 MJX/RL 实验 |
| `darshmenon/quadruped-robotics-stack` | ★22 | 一个仓库里同时有 **RL + CHAMP + Quad-SDK NMPC** 三套 Go2 控制器（做对比很方便） |
| `despargy/maestro_mujoco` | ★18，MIT | Go1/Go2 在 MuJoCo 中**处理易滑地面**（对应我们的黏滑问题） |
| `shaoxiang/awesome-unitree-robots` | ★55 | Unitree 开源项目索引（继续挖的入口） |

**与我们的冲突（必须走 ADR 才能引入）**：RL 策略是**不透明权重**（无法声明其行为边界）、
训练需 GPU、且"策略输出 = 力矩/位置目标"要作为 Provider 接入就必须声明版本/哈希/失败路径。
⇒ 若要考虑 RL，应按 ADR 立"学习型 Provider"边界（含失败/降级路径），不要直接塞进确定性控制路径。

## 5. 建议的借鉴顺序（按我们的约束与阻塞点）

1. **先借 `pychamp` 的两个机制**（同语言、可直接读懂）：相位延迟日程（含**站立↔行走过渡态**）
   + "机身位姿命令 = 足端目标统一反向平移"。把我们的 crawl 从"落点先行、机身被拖"
   改成"**机身轨迹先行、落点跟随**"，设计目标直接用 §13 的支撑余量（要求四个相位都 > 0）。
2. **再对照 `go2-convex-mpc`**（MIT/MuJoCo/Go2）：看它的**落足点与机身轨迹如何由 MPC 同时给出**；
   若它的实现能在纯 CPU 下跑，可作为"同一条验收下"的参照实现。
3. **用官方 `unitree_mujoco` + `mujoco_menagerie` 的 Go2 模型交叉验证**我们的模型/执行器保真度
   （我们曾在 §8 只测到 calf 的 `ctrlrange`，hip/thigh 的 class 继承没解析全）。
4. **RL 仅作为备选路线**（需要 ADR 决策），优先级低于 1~3。

## 6. 诚实边界

- 本调研只做了**可达性验证 + 结构阅读 + 参数摘录**，**未**运行任何外部工程的代码，
  因此"他们的做法可行"是**文献/代码层面的判断**，不是我们在本项目内的实测结论。
- 星星数/许可证/最近提交时间来自 GitHub API（2026-09-21 查询），可能随后变化；
  `chvmp/champ` 与 `elijah-waichong-chan/go2-convex-mpc` 的完整源码**未**取回（前者核心不在主仓库、
  后者传输中断），只取回了 `libchamp` / `pychamp` 的关键文件与 Go2+CHAMP 集成仓库。
- 取回的文件都在 gitignored 的 `build/research/` 下；**未**把任何外部代码拷进 `src/`。
