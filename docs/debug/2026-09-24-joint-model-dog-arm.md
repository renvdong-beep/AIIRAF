# 联合模型（狗 + 臂）首次构建成功与同模型可达实测（2026-09-24）

## 0. 一句话

在**同一个 MJCF** 里装下 Go2 与本仓库的 Piper 臂：`build/scenes/handoff_lab/handoff_lab_joint.xml`
（`nbody=28`、`nq=34`、`nu=20`），并**在同一模型里实测**：臂能够到狗背上的托盘
（FK 采样 20000 次，工具点 `piper_link6` 到托盘顶面 `(0.450, 0.000, 0.346954)` 的**最小距离 = 0.038048 m**）。

这条结论取代了此前 `.hermes/plans/2026-09-23-dock-for-handoff.md` §11.1 的**假设**
（"臂基座在台面原点附近、只按位置可达筛选"）—— 现在是同一模型里的真实位姿 FK。

## 1. 产物与实测（可复跑）

```
构建：PYTHONPATH=src python3 scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2 --attach piper
      ⇒ 退出码 0，写 model.joint_output = build/scenes/handoff_lab/handoff_lab_joint.xml
探针：PYTHONPATH=src MUJOCO_GL=glfw python3 build/iraf-a6a14/joint_reach_probe.py
      ⇒ build/iraf-a6a14/joint-reach-probe.json（4/4 判据通过）

nbody=28（狗 20 + 臂 8）｜nq=34（狗 26 + 臂 8）｜nu=20（狗 12 + 臂 8）
臂 body 全部按前缀 `piper_` 命名 ⇒ 无名字冲突
臂基座世界位置 = (0.45, −0.45, 0.123)（`piper_link1` 自身 z 偏置 0.123）
     与声明 `robots.piper.placement.pos_m = (0.45, −0.45, 0)` 的**平面误差 = 0.0 m**
托盘顶面（狗站立姿态、托盘在站位）目标 = (0.450, 0.000, 0.346954)
FK 采样最小距离 = **0.038048 m ≤ 0.05** ⇒ 够得到
```

## 2. 实现要点（`src/iraf_adapters/unitree/scene_builder.py` 的 `_attach_robots`）

用 **MuJoCo 原生 `MjSpec.attach(child, prefix, frame)`** 合成，而不是手写名字前缀重写
（后者极易漏改 mesh/material/actuator 的 joint 引用）。逐门实测记录（每步"显式失败 → 修 → 再过"）：

| 门 | 现象 | 处理 |
| --- | --- | --- |
| 身份 | 场景 id `piper` ≠ Profile 名 `piper_mujoco` | 新增 `robots[].profile_name` **显式映射**，值仍必须等于 Profile 的 `metadata.name` |
| 构建期声明 | 臂的 Profile 无 `spec.model` | 补 `{vendor, file, trunk_body}`（臂自己的产物仍走其基线） |
| 厂商锁 | 臂侧是 `iraf.piper-model-source-lock/v1`（`files[].path` 为**本机绝对路径**） | 校验器**显式支持两套 schema**，映射规则=「模型目录名 + `relative_path`」，并在报告里标 `host_specific: true` |
| 关节初值 | 抓手 `joint7/8` 不在 `spec.home` | 取 `spec.gripper.open_positions`（声明来源；缺键即显式失败，不猜） |
| 资产路径 | `to_xml()` 报 `.../piper_description/mujoco_model/../../../vendor/unitree_go2/.../base_link.STL` 不存在 | 把附加本体的 mesh/texture 拷到产物目录 `assets/<本体 id>/`；`MjsCompiler` **无 `meshdir` 可写属性** ⇒ 暂用绝对路径并登记债务 |
| 关键帧 | `keyframe 0: invalid qpos size, expected length 34` | 按「主模型关键帧 + 各附加本体的**声明**初值」**覆写**（`attach` 可能已自行扩展过 ⇒ 不能追加）；**且收集顺序必须在覆写之前**（我第一版在循环内覆写、用还没收集完的列表 ⇒ 写回 26 而非 34） |

## 3. 诚实边界（未解决项，已登记）

1. **联合产物的资产是绝对路径** ⇒ 产物 **host-specific**：`MjsSpec.compiler` 没有 `meshdir` 可写属性，
   两个本体的资产无法用单一相对 meshdir 表达。报告里 `injections.attached_robots[].assets_path_style
   = "absolute_host_path"` 如实标注。
2. **债 S4（新）**：`vendor/agilex_piper/piper_description` 是指向 `~/piper_ros/src/piper_description`
   的**符号链接** ⇒ 臂的厂商资产**并未入库**、其锁写死本机路径 ⇒ 换机/新克隆无法复现臂侧仿真。
   迁移到厂商 U1 约定（资产实体入库 + 重写锁 + 许可证 BOM）是独立工作项。
3. 本产物的**动力学**未做任何验收：只证明"装进了同一模型、位姿与声明一致、几何上够得到"。
   交接（把方块放进托盘）的真实接触/摩擦、以及 `s03_pick` 的三项判据口径，都还没做。

## 4. A 方案落地：联合模型里**按标准链路驱动机械臂**（2026-09-24 续）

### 4.1 问题（实测报错，不是推测）

合并模型里附加本体的对象一律带前缀（`piper_joint1`、`piper_link6`），而 Profile / 技能参数的
口径是**声明名**（`joint1`）⇒ `backend.move_joint({"joint1": …})` 直接

```
ValueError: 找不到关节或执行器: joint1        （src/iraf_adapters/mujoco/mujoco_backend.py）
```

这是"**Profile 名 == 模型名**"这条隐含前提第一次被跨本体场景打破（历史上 Piper 两者恰好同名，
UR5e 只是关节名/执行器名不同，都在单本体内）。

### 4.2 修法（A 方案：声明式名字映射）

| 层 | 改动 | 位置 |
| --- | --- | --- |
| 构建器 | attach **之前**采集附加本体自身模型的对象名；套用既有的"仅当加前缀才存在"规则，**机械生成** `name_map`，并按 mapped / identical / conflicts / missing **四桶留痕** | `scene_builder._attach_robots`、`_joint_manipulation` |
| 报告 | `manipulation.name_map`（声明名 → 模型名）+ `manipulation.name_map_facts` | `build/scenes/handoff_lab/handoff_lab_joint.json` |
| 运行期装配 | `scene_report` 模式把 `name_map` 透传进后端配置；**联合报告却没有 name_map ⇒ 拒绝装配**（fail-closed） | `scripts/scenario.py:build_backend_config` |
| 后端 | 装配期校验四条（同值 / 声明名已存在 / 指向不存在 / 多对一），解析点覆盖 `_direct_actuator_channel`、`_joint_name_of`（**回写仍是声明名**）、`_body_id`、`_joint_qpos` | `mujoco_backend._parse_name_map` 等 |
| 部署产物 | `emit_backend_config.py` 一并搬 `name_map` | `scripts/emit_backend_config.py` |

`conflicts`（主本体与附加本体**同名**，映射有歧义）**不进表**、只留痕：本次联合模型实测
`conflicts = 0`、`missing = 0`，`name_map` 共 16 条 = 8 关节（`joint1..8 → piper_joint1..8`）
+ 8 连杆（`link1..8 → piper_link1..8`）。厂商 MJCF 的 geom 是**无名**的（实测 `piper_link6` 下
两个 geom 名均为空串）⇒ geom 桶为空是事实，不是漏采。

### 4.3 证据（`build/iraf-a6a14/arm_on_joint_model.py`，7/7 判据）

链路与 `interact` / `run` 完全相同：`scenario.assemble`（declaration = `config/machines/piper_joint.yaml`）
→ `SkillRuntime.execute`（TaskFlow → Policy → Provider → 后端）。

```
负向对照（去掉 name_map，等价改动前输入）：FAILED / IRAF-EXECUTION-FAILED / "找不到关节或执行器: joint1"
装配期门禁 4/4 全部拒绝（同值映射 / 声明名已存在 / 指向不存在的模型名 / 多对一）
move_joint（声明名口径 joint3/joint5/joint7，2000 ms）SUCCEEDED
  joint3 目标 −0.4 → 实测 −0.3968392827136885（误差 0.0031607172863115096）
  joint5 目标 +0.5 → 实测  0.5164671539976586（误差 0.016467153997658635）
  joint7 目标 +0.02 → 实测 0.02000003857912913（误差 3.857912913088346e-08）
  （判据：末态误差 ≤ 0.05 rad；"位移过半"是错判据 —— 关键帧把臂初始化在声明的 home
    joint3=−1.15、joint5=0.35，起始非零，我已因此误判过一次）
命令落点：ctrl[piper_joint3]=−0.4、ctrl[piper_joint5]=+0.5、ctrl[piper_joint7]=+0.02（其余 0）
状态回写键：last_positions 仍为声明名（joint1..joint8）
主本体隔离：狗的 12 个执行器 ctrl 与指令前**逐位相同**
```

### 4.4 发现（记录，不在本变更里改）

`MujocoBackend.stop()` → `_safe_stop_controls()` 写的是 `self.data.ctrl[:] = 0.0` —— **整条 ctrl 数组**，
联合模型里臂的 stop 会把主本体（狗）的 12 个执行器一并归零（实测 `FR_thigh` 0.9 → 0.0）。
单本体路径无差别（模型里只有本体的执行器 ⇒ 现有回归逐位不变），但"紧急停止的作用域"属**安全语义**，
按铁律 1.6 应单独裁定（收敛到本后端自己的执行器 = 更弱的安全动作），故只登记。

### 4.5 未做（下一步）

1. `handoff_lab` 的执行绑定仍是两个**单本体**模型（`baseline.yaml`: piper → `config/machines/piper.yaml`、
   go2 → `config/go2_loopback.yaml`）⇒ 要在联合模型上跑场景，需要一份指向联合报告的四足绑定
   （`config/machines/go2_joint.yaml` 之类）并把两个本体**指向同一个 MJCF**；同时要回答
   "两个后端实例 = 两份 MjData"这一问题（谁步进、如何共享状态）。
2. `s03_pick` 三项判据口径（`pose_tolerance_m` / `min_lift_delta_m` / `require_bilateral_contact`）。
3. 联合产物的资产相对化（消 host-specific）；债 S4（臂资产入库）。

## 5. 联合场景的两个前提问题：实测判死（2026-09-24 续）

### 5.1 "两个本体都指向同一份联合 MJCF"**不**等于同一个世界

探针 `build/iraf-a6a14/joint_model_two_backends_probe.py` → `joint-model-two-backends-probe.json`：

```
同一份 XML 两次 from_xml_path：same_mjmodel_object=False、same_mjdata_object=False、
                              same_qpos_buffer=False（nq=34/nu=20 两边一致）
实验 A：peer（未带 name_map 的第二实例）步进 200 步 ⇒ 狗的 base 从 z=0.288372 掉到 0.0771723378766752；
        而**从未步进过的**臂实例里 base 仍是 [0, 0, 0.288372, 1, 0, 0, 0]（逐位未变）
实验 B：臂实例 move_joint + 200 步（joint5 → 0.39921470672285164）⇒ peer 的 base 前后**逐位相同**
⇒ two_independent_worlds = True
```

对比事实：四足后端同样自持模型与数据（`src/iraf_adapters/unitree/unitree_go2.py:305`、`:315`）⇒
真实装配里每个本体一个后端实例 = 每个本体一份世界。

**教训（我自己的判据错误）**：第一版探针拿"已经跑过 `move_joint` 的臂实例"去比 peer 的初值，
量到 0.288372 → 0.077172 就得出"两侧互相泄漏"的相反结论 —— 那是**臂自己那份世界里的狗在倒塌**。
判据必须是"未步进过的新实例 vs 步进后的对侧"，即**一次只动一侧**。

### 5.2 关键帧 ctrl 不是控制器：狗在**两个模型里塌得一样**（与本变更无关）

探针 `build/iraf-a6a14/joint_model_keyframe_ab.py` → `joint-model-keyframe-ab.json`
（同关键帧初值、不下任何新指令、只步进；timestep 0.002 s）：

```
A 四足单本体 handoff_lab.xml   （nu=12，key.ctrl 长度 12）
  base_z：0 → 0.2883725 ｜ 50 → 0.23317931719286822 ｜ 100 → 0.08431690724624108
          200 → 0.0771723436516014 ｜ 500 → 0.07724787916154259
B 联合 handoff_lab_joint.xml   （nu=20，key.ctrl 长度 20，后 8 位全零 ⇒ 臂侧是零填充）
  base_z：0 → 0.288372   ｜ 50 → 0.23317881719286826 ｜ 100 → 0.08431662103679326
          200 → 0.0771723378766752 ｜ 500 → 0.07724848525427036
⇒ joint_assembly_is_cause = False
```

结论两条：
1. 狗在关键帧 ctrl 下 0.1 s 内就坐到地面上（0.288 → 0.077）——**两个模型完全一致**，
   与联合装配无关；狗的站立/行走必须由控制器（`stand` / `locomote` 的 MPC 路径）驱动。
   这也解释了为什么直接步进联合模型时"狗不见了"：它是塌下去的，不是渲染问题。
2. 联合模型的关键帧 `ctrl` 对臂侧 8 个执行器是**零填充**（不是声明的 `open_positions ±0.035`）
   ⇒ 臂侧 ctrl 初值不是声明值；关键帧只保证 **qpos** 初值来自声明。若将来需要"臂初始保持张开"，
   要把 `ctrl` 也按声明补齐（属共享植物装配的工作项）。

## 6. 共享植物落地：场景层 + 技能层（2026-09-24 续）

实现（`robot.plant` 声明 + `scenario.assemble` 的 plant_registry）：

| 层 | 改动 |
| --- | --- |
| 声明 | `config/go2_joint.yaml`（四足，`model.file` → 联合产物、`robot.plant.role: owner`）；`config/machines/piper_joint.yaml`（臂，`robot.plant: {role: guest, owner: unitree_go2, guest_timeout_factor: 30.0}`） |
| 装配 | `scenario.assemble` 读 `robot.plant`：owner 装配后把植物登记进 `plant_registry`；guest 从登记处取同一株注入（`plant` + `plant_guest_timeout_factor` 进后端配置）。缺 registry / guest 先于 owner / 系数非正数 ⇒ **显式失败** |
| 后端 | `load_backend(..., **kwargs)` 透传；四足后端 `from_config(..., plant=None)`；两者步进都走 `plant.step_once(self)` |

证据 `build/iraf-a6a14/shared-plant-skills.json`（6/6，**走 `assemble` + `SkillRuntime.execute`**，
与 `interact`/`run` 同链路）：

```
负向：guest 先装配 ⇒ 「装配顺序必须 owner 先于 guest」；声明了 plant 但无 registry ⇒ 「必须传入 plant_registry」
正向：registry = {unitree_go2}，臂拿到的是同一株植物（same_data=True）
狗 stand（技能层）SUCCEEDED：base_z 0.2760863419784013，plant 推进 4000 步，8 个周期
臂 move_joint（技能层，声明名 joint3/joint5/joint7）SUCCEEDED，墙钟 0.4698 s：
  末态误差 0.0031713854410383435 / 0.01808743373350441 / 3.9077193394326803e-08 rad
臂 stop（技能层）后：改动过的狗执行器下标集合 = []（狗 ctrl 仍是 stand 的真实力矩 −5.357969215043955 … 6.67198225602607 N·m），臂 8 个执行器归零
```

### 6.1 我的三个判据错误（同一类，必须记住）

1. 拿"已步进过的实例"当对照 ⇒ 把对侧自己的演化当成泄漏（§5.1）。
2. 用"位移过半"判到位 ⇒ 起始位形非零（关键帧 home joint3=−1.15）时把已到位的关节判成没动。
3. 在 owner 仍在跑时比较"指令前后 ctrl 快照" ⇒ owner 每拍都在写自己的 ctrl，判据天然不成立。
   正确做法：**先冻住 owner 再量**，并记录"被改动的下标集合 ⊆ 本后端拥有的执行器"。

共同点：**判据必须只让一个变量动**（一次只动一侧 / 只比末态 / 先停扰动源）。

### 6.2 一个隐蔽的变量遮蔽缺陷（自己引入，已修）

`scenario.assemble` 里我第一版用局部变量 `registry` 存植物登记处，覆盖了上面的**技能注册表** ⇒
`SkillRuntime` 拿到 dict，所有技能解析失败（`'dict' object has no attribute 'resolve'`），
而两处 try/except 一度把失败吞成"状态未返回"。已改名 `plant_registry`，并加回归测试
`tests/unit/test_joint_name_map.py::AssemblePlantWiringTests`（断言 `runtime.registry` 仍是
`SkillRegistry` 且 `resolve("move_joint")` 非空）。

## 7. 可见演示：狗与臂同框、同一个世界（2026-09-24 续）

入口 `build/iraf-a6a14/shared_plant_demo.py`（必须显式 `--display interactive_viewer`；`auto` 不开窗）：

```
DISPLAY=:0 XAUTHORITY=<...> MUJOCO_GL=glfw PYTHONPATH=src \
  python3 build/iraf-a6a14/shared_plant_demo.py --seconds 12 --phase both
⇒ build/iraf-a6a14/shared-plant-demo.json（4/4 判据通过）
```

| 阶段 | 现象（实测数字） |
| --- | --- |
| 窗口 1：狗 `stand` × 臂 `move_joint` **同时** | 窗口开（543 帧）；狗 SUCCEEDED，base_z **0.2793580236507228**，植物推进 **15000** 步；臂 SUCCEEDED，末态误差 **0.0031713854268711206 / 0.01808743374791255 / 3.9094728232491605e-08** rad |
| 窗口 2：狗 `locomote` 前进 0.2 m/s × 臂保持 | 窗口开（604 帧）；狗 SUCCEEDED，base 从原点走到 **(0.46809567761708076, −0.0008316464979186758, 0.2877945528530491)**；臂 joint5 仍是 **0.518087433753746**（= 它被指令的目标位形）⇒ 同一个世界里狗走、臂保持 |

### 7.1 第三次踩到同一个时序坑（必须记住）

guest 的请求**必须等 owner 已经开始推进**才能发：第一版让臂线程与窗口同时起跑 ⇒ 臂的等待立刻
超时（`PlantWaitTimeout`）；第二版把延时设成 2.5 s，却又**超过了狗的 stand 墙钟**（8000 ms 仿真
只需 ~0.7 s 墙钟）⇒ owner 已停推，臂请求被技能层判 FAILED（且我第一版没记 `error_code`/`reason`，
只能看到 status=FAILED）。正确做法（已落地）：**owner 的仿真时长要远长于 guest 的墙钟耗时**
（stand 30000 ms 仿真 ≈ 2.5 s 墙钟，覆盖臂 200 步），guest 在 1.2 s 后插入 ⇒ 两者真正重叠。

## 8. 机械臂抓取到位误差 12.113 mm 的机制判死（2026-09-24 续）

现象（`build/acceptance/handoff_lab/nominal/report.json` 的 s03_pick，exit 5）：
`IRAF-EXECUTION-FAILED 末端未到达目标抓取位姿: distance=0.012113m tolerance=0.005000m
delta=[0.0022387082780914447, 5.394919764631356e-06, 0.011904138538288277]`

探针 `build/iraf-a6a14/grasp_error_probe.py` → `grasp-error-probe.json`（判死两个岔路）：

| duration_ms | max|q − q_des|（rad） | 备注 |
| --- | --- | --- |
| 1600 | **0.016796437522027752** | 还没稳定 |
| 4000 | **0.005316921001245409** | |
| 8000 | **0.005314844537182922** | 与 4000 几乎相同 ⇒ **平台期** |

- `hypothesis_A_not_converged = true`（关节误差 5.3e-3 rad 远大于 1e-3）、
  `hypothesis_B_declaration_vs_model = false`（不是“声明几何 vs 模型几何”不符）。
- 关键旁证：报告的 `gravity_feedforward_phases = []`（**一个相位都没有**），而
  `MujocoBackend._pick_ctrl_offsets(phase)` 只从 `gripper.gravity_feedforward` 取前馈
  ⇒ 抓取三段（home/approach/grasp）**全是零前馈** ⇒ 纯 PD 稳态下垂（τ_g/kp）。
- 时间无关性把“收敛慢/稳定窗不够”排除掉：加长 2 倍稳定时间只把误差从
  0.005316921001245409 改到 0.005314844537182922（差 2.08e-6）。

⇒ 结论：**12.1 mm 的 z 向到位误差 = 缺重力前馈造成的稳态下垂**（关节级 5.3e-3 rad，
经串联臂放大到末端 ~12 mm），不是判据问题、不是几何声明问题、不是时序问题。

修法（下一项，按声明生成的路线）：给 Piper 的抓取相位生成 `gripper.gravity_feedforward`
—— 与 UR5e 同一套配方（`scripts/build_ur5_baseline._gravity_hold_ctrl`：按该位形的
`qfrc_bias` 与执行器 `gainprm[0]` 算 τ_g/gain），写进臂侧场景报告，由基线构建器生成，
**不在实现层写默认值**。验收：同一探针的关节误差降到 ≤1e-3 rad 量级、s03_pick 的
`distance` ≤ 0.005 m。

## 9. 重力前馈已实施（2026-09-24）：关节级下垂消除，门禁残差另有独立原因

已落地（三处接线全部照 §9 原文实施）：
- `config/piper_simulation_baseline.yaml` 新增 `gravity_feedforward: {hold_ms: 4000,
  tolerance_rad: 0.001}`（声明量，实现层不写默认值）；
- `scripts/build_piper_baseline.py: build()` 对 home/approach/grasp/lift 四个姿态逐个调用
  `iraf_core.kinematics.gravity_hold_ctrl`（用**注入增益后**的场景模型），写回
  `reference.gravity_feedforward` + 自证证据，并写进 `gripper.gravity_feedforward`
  （键与位置指令一一对应，错位即显式失败）；
- 生成方式按契约：该脚本把场景 JSON 打到 **stdout**（实测 `main()` 只 print，不写文件）⇒
  `python3 scripts/build_piper_baseline.py > build/models/piper-pick-scene.json`。
  报告现含 `reference_poses`（✓）与四相位前馈：grasp 增量
  `{joint2: -0.002137427, joint3: -0.005371549, joint5: -0.001086334, …}`，
  自证方法 `qfrc_bias_plus_static_hold`，静态保持残余最大 **2.9551e-05 rad**（限 0.001）✓

关节级判死指标（同一条探针，改前 → 改后）：

| duration_ms | max\|q − q_des\| 改前（rad） | 改后（rad） | 倍数 |
| --- | --- | --- | --- |
| 1600 | 0.016796437522027752 | 0.018786867883619607 | （未稳定，不比较） |
| 4000 | 0.005316921001245409 | **0.00021921636035648895** | 24× |
| 8000 | 0.005314844537182922 | **2.9338248136322893e-05** | 181× |

⇒ §8 判死的机制（缺前馈的稳态下垂）**已被消除**。

**但 s03_pick 仍未过**（`nominal` 复跑，`build/acceptance/handoff_lab/nominal/report.json`）：
`末端未到达目标抓取位姿: distance=0.014450m tolerance=0.005000m
delta=[0.004229494155511382, 5.415238119581278e-06, 0.013817039392046003]`
—— 关节误差已经只有 1e-4 量级，而残差仍集中在 **z 向 13.817039392046003 mm**（改前 11.904 mm）
⇒ **还有第二个独立原因：门禁的几何口径与模型实际几何不一致**（与下垂无关）。

### 9.2 几何口径**已排除**（2026-09-24 实测），真正剩下的是"参数没送到 provider"

探针 `build/iraf-a6a14/grasp_geometry_probe.py` → `grasp-geometry-probe.json`
（走到 grasp 位姿后**同一帧**量四组量）：

```
目标中心            (0.189999426,  0.0,        0.025)
实际 pad 中点       (0.189999247, -5.574e-06,  0.054828218)
门禁期望点          (0.189999426,  0.0,        0.054835769)   ← 目标中心 + 轴·pad_offset_m(0.029835769)
求解器 finger_center (0.189999426, -0.0,        0.05482814 )
⇒ 门禁残差 = [-1.79e-07, -5.574e-06, -7.551e-06] ⇒ distance = 7.551e-06 m ≪ 0.005 ✓
   轴·offset − (求解器 finger_center − 目标中心) = 7.629e-06 m
```

⇒ **门禁几何口径与模型实际几何是一致的**（残差 7.6 µm，比 5 mm 门禁小三个量级），
`pad_offset_m` 的声明值也没错。§9 里"第二个独立原因 = 几何口径不符"的猜测**被实测否掉**。

那么 `nominal` 里 s03 的 `distance=0.014450m` 是什么？—— **相位时长**：
`PickObjectProvider.execute` 读 `inputs.get("duration_ms", 1000)` ⇒ 每段 `duration_ms // 5`
= 200 ms（未声明时长时的默认值）⇒ 明显不够收敛（同一路径给到 1600 ms/段时残差 7.551e-06 m）。

**但**：在 `scenes/handoff_lab/scenario.yaml` 的 s03_pick 里声明 `duration_ms: 8000` 之后
**复跑结果逐位相同**（`distance=0.014450m`、delta 与 qpos 全同）⇒ 该参数**没有到达 provider**，
即存在于"步骤声明 → 运行期请求"之间的某个环节被丢掉/覆盖。已核对的：
- YAML 解析后 `s03_pick` 的 params = `{"target_id": "box_01", "grasp_pose_from": "report_target",
  "duration_ms": 8000}` ✓（声明本身没问题）；
- 技能输入契约允许 `duration_ms`（1..30000）✓；`_request` 原样透传 `parameters` ✓；
- 但 `_dispatch_step` 到 provider 之间还有 PolicyGateway / 技能调用层 ⇒ **下轮从
  `_dispatch_step` 往下一步步打印实际送进 provider 的 inputs**（这是唯一还没看的环节）。

（纪律提醒：这类"声明了却不起作用"的问题必须当成**链路缺陷**查到具体一行，不能用"改个数字
试试"绕过 —— 否则同一类问题会在别的参数上重演。）

### 9.3 参数链路已逐环核对：**四环全部原样透传**，矛盾点收窄到 `pick_object` 内部

已核（每一环都读了实现，不是推断）：

| 环节 | 位置 | 结论 |
| --- | --- | --- |
| 步骤 → 请求 | `scripts/scenario.py: _dispatch_step` → `_request(..., step["params"], ...)` | 原样放进 `parameters` ✓ |
| 策略准入 | `iraf_core/policy.py: PolicyGateway.validate` | `PolicyDecision(..., parameters)` 原样返回；仅当 `duration > min(manifest.timeout_seconds*1000, safety.max_duration_ms)` 才拒绝（8000 ≤ 30000，不触发）✓ |
| 清单分发 | `iraf_core/registry.py: Manifest.invoke(inputs)` | `validate_inputs` 后原样交给 `provider.execute(inputs, lease)` ✓ |
| Provider | `iraf_skills/common/manipulation.py: PickObjectProvider` | `duration_ms=int(inputs.get("duration_ms", 1000))` → `backend.pick_object(duration_ms=…)` ✓ |

而实测的两个数字互相矛盾（这是当前**未定位**项，不臆测原因）：
- 探针 `grasp_geometry_probe.py` 用**与 pick 相同的三次 `_move_trajectory`（各 1600 ms）+ 同名前馈**
  走到 grasp 位姿 ⇒ 门禁残差 **7.551e-06 m**；
- 走 `nominal` 的 s03_pick（声明 `duration_ms: 8000` ⇒ 每段同样 1600 ms）⇒ 门禁残差
  **0.013817039392046003 m（z 向）**，且两次复跑逐位相同、墙钟 3.697321214945987 → 3.718921724939719 s
  （只差 0.6% ⇒ 时长几乎没起作用）。

⇒ 下一步的**仪器化点**（唯一还没直接观测的地方是 pick 内部拿到的实参）：
1. 给 `MujocoBackend.pick_object` 加一个与既有 `IRAF_DEBUG_PICK` 同风格的观测（打印
   `duration_ms`、三段实际 `duration_ms//5`、`home/approach/grasp_positions` 的键与 `gripper`
   配置来源路径）⇒ 一眼看出 pick 内部与探针的入参是否一致；
2. 同一次观测里打印 `_pick_ctrl_offsets(phase)` 的返回（前馈是否真的被取到 —— 例如键名
   与位置指令不一致时前馈会**静默失效**，而 `build_robot_pick_scene` 的显式校验在 Piper 这条
   路径上**没有**（§9 第 2 点）：这正是最可疑的一环）；
3. 必要时把探针与 pick 的**同一段**代码抽成共享函数，消除"两处实现"本身。

### 9.6 相位级观测：门禁**能过**（同一路径 9.388e-06 m），但报告里的门禁异常来自另一个状态

新增相位级观测 `MujocoBackend.dump_pick_phase(...)`（`IRAF_DEBUG_PICK=1` 开关），pick 与探针
**共用同一实现**。`IRAF_DEBUG_PICK=1` 复跑后逐相位对照：

| 相位 | 探针 distance_m | pick distance_m | pick joint_qpos（joint2/joint3/joint5） |
| --- | --- | --- | --- |
| HOME_HOLD | 0.163883207 | **0.163883207** | 0.0 / −0.0 / −1e-09 |
| APPROACH | 0.039995062 | **0.039995062** | 0.98664089 / −0.085722541 / −0.112597474 |
| DESCEND | 0.000009388 | **0.000009388** | 1.259001453 / −0.123957681 / −0.120452556 |

⇒ 两条路径**逐相位一致**（同一后端配置、同一 positions、同一前馈、同样 1600 ms/段），
并且 **DESCEND 之后门禁残差只有 9.388e-06 m ≪ 0.005** ⇒ 这一段**能过**；日志里也接著出现了
GRIP_OPEN / GRIP_CLOSE / LIFT 相位 ⇒ 这张 run 确实越过了对齐门禁。

**但**同一份报告里 s03_pick 的失败原因仍写着
`末端未到达目标抓取位姿: distance=0.014450m ... qpos={joint2: 1.1861117642286438,
joint3: -0.12329910321790603, joint5: -0.1203431601667951}` —— 其中的 `joint2` 与 DESCEND
dump 的 `1.259001453` **不是同一个状态**，残差也不是 9.388e-06。

⇒ 新矛盾（下一项，两条候选，都要用观测判死，不猜）：
1. **pick 被调用了不止一次**：`PICK_INPUTS` 每个进程只出现 1 行（本次），但需给观测加上
   调用序号/时间戳确认；若跑两次，第二次会从"已夹住/已抬升"的状态重跑三段 ⇒ 残差不同；
2. **DESCEND 与门禁之间状态被改动过**：候选是并发步进（后台线程推进植物）或
   `_log_pick_phase` 的副作用；判别办法：在门禁之前**紧接着**再 dump 一次
   （`PICK_PHASE DESCEND_PRE_GATE`），若与 DESCEND 的 dump 不同 ⇒ 有东西在动；相同 ⇒
   是"另一次调用"（候选 1）。

这一轮顺带还有两个已知小缺陷待修：`_log_pick_phase` 与新的 `PICK_PHASE` 观测**共用前缀**
（`grep PICK_PHASE` 会混进两套格式，我第一版解析脚本就因此崩溃）⇒ 应改成不同前缀；
以及 §9.5 记的清理路径 `LeaseConflict: lease expired`（进程退出码 1）。

### 9.5 实参观测落地：pick 拿到的入参与探针**一致**，矛盾因此收窄到"相位内/相位间的执行"

给 `MujocoBackend.pick_object` 加了与既有 `IRAF_DEBUG_PICK` 同风格的实参观测（环境变量开关，
默认关闭）。`IRAF_DEBUG_PICK=1` 跑 `nominal` 得到：

```
PICK_INPUTS {"target_id": "box_01", "duration_ms": 8000, "phase_ms_each": 1600,
             "positions_keys": {home/approach/grasp/lift 均为 joint1..joint8},
             "feedforward_offsets": {"home": {...joint2: 0.005229714, joint3: -0.009013975...},
                                     "approach": {...}, "grasp": {...joint2: -0.002137427,
                                     joint3: -0.005371549, joint5: -0.001086334...}, "lift": {...}},
             "gripper_fields": [..., "gravity_feedforward", "pad_offset_axis", "pad_offset_m", ...]}
```

⇒ ① `duration_ms=8000`／每段 1600 ms 确实生效（此前"复跑逐位相同"不代表参数没到，
是我把两次不同的对照混在一起比较）；② 四相位前馈**非空且已进 pick**（排除了 §9.3 的
"前馈被静默丢"嫌疑）；③ 位置指令键与门禁口径一致。

**但** pick 的门禁残差仍是 z 向 `0.013817039392046003 m`，而我的探针用**同一后端配置、同一批
positions、同一批前馈、同样 1600 ms/段**得到 `7.551e-06 m` ⇒ 差异只能在"相位内/相位间的执行"
（例如 `_log_pick_phase` 的副作用、`_move_trajectory` 的 settle 与实际步进方式、或 pick 在
相位之间读了一次状态）。**下一轮的仪器化（唯一还没观测的地方）**：在 pick 的**每个相位之后**
打印 `_grasp_alignment_evidence` 的 `center_delta_m` 与 `joint_qpos`（同一开关），并让探针用
**同一个打印**跑一遍 —— 两份逐相位序列一比即可指出是哪一段分叉。

**顺带发现（小缺陷，登记）**：本次 `nominal` 进程在报告写完之后以 `LeaseConflict: lease expired`
（`iraf_core/authority.py:46`，由某条清理路径的 `authority.validate(lease)` 触发）异常退出、
退出码 1 ⇒ 清理路径不应在租约已过期时再校验（应容忍 expired 或先释放再校验）。

### 9.4 A/B 对照组工装 bug（我自己踩的，记进纪律）

`build/iraf-a6a14/pick_vs_probe_ab.py` 想在**同一后端**上对比"探针路径 vs pick 内部路径"，
结果两边都测出 `distance=0.16388329862783327 m`（pad 中点 z=0.210702143，目标 z=0.025），
且 pick 耗时 29.905061601195484 s、末态 `joint1=0.0` ⇒ 机械臂**根本没动**。

原因（不是产品缺陷，是我的工装写错）：脚本的 `_reset()` 做了
`backend.data = mujoco.MjData(backend.model)` —— 而共享植物（`MujocoPlant`）**仍持有旧的
`data`** ⇒ `plant.step_once()` 推的是旧缓冲区，写在新 data 上的 ctrl 永远不会生效。
教训：**换 data 必须同时换 `plant.data`**（或干脆不换，用后端自己的 data 复位 qpos）。

⇒ 该 A/B 结论作废；可信的仍是**不换 data** 的探针：`grasp_geometry_probe.py` 的
门禁残差 **7.551e-06 m**。§9.3 的矛盾（探针 7.6e-06 vs nominal 1.3817e-02）**仍未定位**，
下轮按 §9.3 的仪器化点直接观测 pick 内部实参，不要用"换 data 对照"这种会自伤的工装。

（第 2 点给出一个具体嫌疑：Piper 的 `_pick_ctrl_offsets` 返回的键是**关节名**，而
`_move_trajectory` 用 `_actuator_channel(name)` 解析 —— 若前馈字典里的名字不是该段的
位置指令键，`unknown` 检查会抛错（不会静默）；但若前馈**相位字典为空**则整段无前馈 ⇒
回到 §8 的下垂状态。观测先看这一条。）

下一步的判死顺序（不要一次改多处）：
1. 门禁用的是 `pad_boxes`（若声明）否则左右指腹 geom 的**中点**；先量"模型里实际 pad 中点
   与目标中心的几何关系"（`data.geom_xpos` + `xpos[target]`，逐轴），与门禁期望的
   `目标中心 − pad_offset_m · 接近轴`（当前 0.029835769 沿 +z）对比 ⇒ 看差异是**常数偏置**
   （声明值与模型不符）还是**随位形变化**（口径不同：例如参考求解器的
   `finger_height_correction_m` / `tip_clearance_m` 已经把这部分算进 `grasp_point_m`）；
2. 若为常数偏置 ⇒ 修**声明**（`pad_offset_m`/`pad_offset_axis` 由参考姿态重算，属构建期产物），
   不得改门禁阈值（收紧/放宽安全边界都需单独授权）。
3. 我自己的探针里那个 `distance`（~0.0596 m）是**自建的粗糙几何**，不能当门禁代理
   （它与门禁差 4 倍以上）⇒ 验证只能用真 `pick_object`。这条也写进纪律。

## 9.1 原配方与接线点（追溯用，已实施）

**结论**：Piper 抓取相位的重力前馈**没有生成过**，而仓库里已有现成配方与被验证过的消费路径。

现成配方（`src/iraf_core/kinematics.py:507`）：

```python
gravity_hold_ctrl(model, arm_joints, hold_positions, hold_ms=4000, tolerance_rad=1e-3,
                  use_keyframe=True)
# 返回 ({关节名: ctrl 增量}, 证据字典)；内部：qfrc_bias(qvel=0)/gainprm[0]，并做**静态保持自证**
# （把增量加到目标 ctrl 上跑 hold_ms，要求残余关节误差 ≤ tolerance_rad，否则抛 KinematicsError）
```

消费路径（`scripts/build_robot_pick_scene.py:466-491`，UR5e 在用）：把
`reference["gravity_feedforward"][phase]` 从**关节名**翻译成 **ctrl 通道名**后写进
`gripper["gravity_feedforward"][phase]`，并对"前馈键与位置指令键不一致"**显式失败**（错位 = 静默失效）。
后端取用处是 `MujocoBackend._pick_ctrl_offsets(phase)` → `_move_trajectory(..., ctrl_offsets)`。

**接线点有三处（这是为什么不能只写一行）**：
1. `scripts/build_piper_baseline.py: build()`（`scene = build_scene(...)` 之后）—— 对
   `reference` 的 home/approach/grasp/lift 四个姿态逐个调用 `gravity_hold_ctrl`，写回
   `reference["gravity_feedforward"]` + `reference["gravity_feedforward_evidence"]`；
   ⚠ `home` 是 `{关节名: 值}` 而 approach/grasp/lift 是 `{"joint_positions": {...}}`（两种形状），
   ⚠ 必须用**注入增益后**的模型（`build/models/piper-pick-scene.xml`，`scene.arm_position_kp = 200`）
   作为 `model` 入参，否则增益错、增量就错。
2. 该路径用的是 **Piper 自己的** `build_piper_pick_scene.build_scene`（不是通用
   `build_robot_pick_scene`）⇒ 缺少"关节名 → ctrl 通道名"的翻译与键一致性校验，需按第 2 段的方式补上
   （Piper 上执行器与关节同名，但**不得**依赖这个巧合；校验必须显式）。
3. `config/piper_simulation_baseline.yaml` 需新增两个**声明**量：`hold_ms` 与 `tolerance_rad`
   （实现层不写默认值）。

**勘误（留痕）**：本文件 §8 之前我写"报告里没有参考姿态"依据的是 `reference_pose`/`grasp_pose` 两个
键名 —— 实际键名是 `reference_poses`（`build_piper_baseline.build()` 的 `scene["reference_poses"]
= reference`）。但**当前盘上的** `build/models/piper-pick-scene.json` 里确实没有该键
（实测 `reference_poses 存在: False`、`gripper.gravity_feedforward: None`）⇒ 结论不变，只是
"为什么没有"应当说成"该报告由更早的路径生成/尚未重跑"，而不是"构造器不写"。这也是又一次
"别假设报告键名"的实例。

验收（下轮照做即为闭环）：
1. 重跑 `python3 scripts/build_piper_baseline.py`（默认 `--baseline config/piper_simulation_baseline.yaml`
   `--scene build/models/piper-pick-scene.xml`），报告出现 `gripper.gravity_feedforward` 四个相位；
2. `build/iraf-a6a14/grasp_error_probe.py` ⇒ `max|q − q_des|` 从 0.005314844537182922 rad 降到 ≤1e-3 量级；
3. `scenario.py run --scene scenes/handoff_lab --scenario nominal` ⇒ s03_pick 的 `distance` ≤ 0.005 m、
   三项判据由"无证据判失败"转为有实测值。

## 10. 下一步

1. `s03_pick` 三项判据口径（`pose_tolerance_m` / `min_lift_delta_m` / `require_bilateral_contact`）
   → s04/s05 → `nominal` 全场景 → 两臂轮番运输。
2. 联合产物的资产相对化（消 host-specific）；债 S4（臂厂商资产入库 + 重写锁 + 许可证 BOM）；S3。
3. owner 侧作用域的**声明化**（现在 owner = 模型全部执行器；联合植物里应显式声明"我拥有哪些"）。


