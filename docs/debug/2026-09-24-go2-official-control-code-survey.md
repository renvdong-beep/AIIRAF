# 宇树官方 Go2 控制代码调研（2026-09-24，只读，用于定位"停靠精度"的上游参照）

## 0. 为什么查

停靠实测（docs/debug/2026-09-24-dock-target-self-frame.md §6.3/§6.4）暴露：本机自研 trot 在容差附近
的**速度地板 ≈ 40~80 mm/s**，滑行超调把停靠末态卡在 0.063 m，达不到 0.03 m 判据；且"制动提前量""降
接近速度"两条调参路线都被实测否掉。⇒ 需要看官方是怎么做的，判断这是"调参问题"还是"控制器问题"。

## 1. 官方公开仓库（`unitreerobotics`，2026-09-24 经 api.github.com 列举）

| 仓库 | 语言 | 与本次问题相关的部分 |
| --- | --- | --- |
| `unitree_sdk2` / `unitree_sdk2_python` | C++ / Python | **机载控制接口**：`SportClient`（速度级命令） |
| `unitree_mujoco` | C++ | 官方 MuJoCo 仿真 + `sdk2` 桥（本仓库 vendor 的就是它的模型资产） |
| `unitree_rl_gym` | Python | **官方学习步态的训练配置**（部署到真机的策略来源） |
| `unitree_rl_mjlab` / `unitree_rl_lab` / `unitree_sim_isaaclab` | C++ / Python | 新一代 RL 与仿真栈 |
| `unitree_guide` | C++ | **开源经典四足控制器**（FSM + 平衡 + trot，Gazebo），配套《四足机器人控制算法》 |
| `unitree_legged_sdk` / `unitree_ros2` / `unitree_ros` | C++ | 旧 SDK / ROS 2 / ROS 1 接口层 |

## 2. 关键事实（逐条摘自官方文件，均给出路径）

### 2.1 应用层只能给**速度**，精度是机载控制器的职责

`unitree_sdk2_python/unitree_sdk2py/go2/sport/sport_client.py` 暴露的是运动命令与状态查询，
与本问题相关的是：`Move(vx, vy, vyaw)`（连续速度）、`StopMove()`、`BalanceStand()`、`StandUp()`、`Sit()`。
⇒ 官方接口**没有**"把机身开到某点位姿"的位置级命令；真机上的停靠精度由机载步态控制器（部署的
RL 策略）决定。**这一条与 IRAF 的分层完全一致**：`locomote` 是速度级能力，精度归控制器。

### 2.2 官方学习步态的 PD 刚度远低于本仓库自研 trot

`unitree_rl_gym/legged_gym/envs/go2/go2_config.py`：

```
control_type = 'P'
stiffness = {'joint': 20.}      # N·m/rad
damping   = {'joint': 0.5}      # N·m·s/rad
action_scale = 0.25
decimation   = 4
base_height_target = 0.25  m    # 训练用 URDF 模型的初始 z = 0.42（与 MJCF 不同源）
```

对照本仓库 `config/go2_loopback.yaml`：`control.kp_nm_per_rad = 150`、`kd = 4`，再乘
`locomote.leg_position_gain_scale = 0.3` ⇒ 实际 kp = 45、kd = 1.2，即**比官方硬 2.25 倍（kp）/ 2.4 倍（kd）**，
相对官方裸值（20/0.5）则是 7.5 倍 / 9 倍。

### 2.3 官方**开源控制器**是经典 trot + 平衡，官方自己承认它只是入门级

`unitree_guide/README.md`：状态机 `Passive → FixedStand → Trotting`，键盘给平移/转向速度，
"provides a basic quadruped robot controller for beginners. To achieve better performance, additional
fine tuning of parameters or **more advanced methods (such as MPC etc.)** might be required."
⇒ 它与我自研的路径同类（关节空间 trot + 平衡），**不提供**低速精密定位能力；注意它的 FSM 里
`FixedStand` 正是"步态退出到静态站立"的原语——与本仓库 `locomote.halt_at_stance` 同一角色。

## 3. 对本次问题的判断（据此改判）

1. **不是调参问题**：官方在应用层只给速度，精度由控制器决定；而官方开源的两条可用路径里，
   `unitree_guide` 是入门级 trot（与我的同级），`unitree_rl_gym` 是学习步态（另一套技术路线）。
2. **值得试的一条官方依据改动**：PD 刚度按官方档（kp 20 / kd 0.5）对齐 —— 本仓库的位置环
   比官方硬 2~9 倍，"过硬的位置环 ⇒ 足端无法小步蠕动"是可测假设（见 §4 的 A/B）。
3. **不应做的事**：把 0.03 m 判据放宽（授权人已明确不动）；或宣称"官方 SDK 支持位姿停靠"
   （官方没有该接口）。

## 4. A/B：位置环刚度向官方档对齐（`locomote.leg_position_gain_scale`）

被测档：0.3（现产，kp=45）、0.20（kp=30）、0.133（kp=20 = **官方裸值**）、0.08（kp=12）。
同一停靠夹具与判据（s02_dock 的 0.03 m / 2.0° / 0.05 m/s）。实测
（`build/iraf-a6a14/gain_alignment_ab.py` → `gain-alignment-ab.json`）：

```
scale 0.300（kp=45.0 kd=1.20，现产）⇒ 到位 t=6.56 s、到位速度 0.03955 m/s
                                      ｜末态平移 0.063459 m｜偏航 −0.4391°｜DOCK_DRIFTED
scale 0.200（kp=30.0 kd=0.80）      ⇒ **未到位**（无 settled_at）｜0.138839 m｜偏航 +4.0472°｜damped_hold
scale 0.133（kp=20.0 kd=0.53，官方档）⇒ **未到位**｜0.190995 m｜偏航 +6.5390°｜damped_hold
scale 0.080（kp=12.0 kd=0.32）      ⇒ **未到位**｜0.179509 m｜偏航 +3.1731°｜damped_hold
```

⇒ **假设否证**：位置环越软越差，官方档（20/0.5）在**本架构**下连"进容差"都做不到（偏航漂到 3~7°、
末态 0.14~0.19 m）。原因不是"官方参数错"，而是**控制架构不同**：
`unitree_rl_gym` 的 20/0.5 是配**学习策略**的（策略以 50 Hz 输出位置目标、`action_scale=0.25` rad，
刚度小是让策略"有柔度可学"）；本仓库是**脚本化步态轨迹跟踪**，同样的软增益只会让腿撑不住轨迹。
⇒ 结论：官方的 PD 数值**不可直接移植**（架构绑定），能移植的是**接口分层**（速度级能力 + 精度归控制器）。

### 4.1 参数空间已穷尽（三条调参假设全部被实测否掉）

```
① braking_lead_s（制动提前量）：0.0 最优，其余档 0.20~1.55 m 且会失稳 → references 见 §6.4
② approach_speed（0.15 → 0.05 m/s）：更差（0.05 档 lead0 时 1.999 m）
③ leg_position_gain_scale（向官方刚度对齐）：越软越差，官方档直接到不了位
```

⇒ 停靠精度是**控制器/步态**问题，不是参数问题。剩余可行路线（实现级）：
(a) **停靠专用低速步态**：更小步长 + 更低的速度地板（当前地板 ≈ 40~80 mm/s 是超调的直接来源）；
(b) **落足点位置反馈**：把机身 XY 误差喂给落足点规划（现在是固定步幅的退让驱动），
    用小步"走拢"而不是靠速度指令压到零；
(c) 判据重定（授权人已明确**不做**）。
