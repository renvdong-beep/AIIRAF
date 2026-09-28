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

## 10. 里程碑：`nominal` 端到端跑绿（含机械臂真抓取）—— 2026-09-24

`build/acceptance/handoff_lab/nominal/report.json`：`passed = true`、`failed_checks = []`、
`exit 0`，counts = {steps: 5, executed: 3, skipped_pending: 2, faults: 0}

| 步骤 | 结果 | 实测 |
| --- | --- | --- |
| s01_verify_ready（四足 stand） | EXECUTED / SUCCEEDED | 推进 7.999999999999341 s、末速 2.6904543928034624e-05 m/s |
| s02_dock（四足停靠交接站位） | EXECUTED / SUCCEEDED | 平移 0.02858758381668675 ≤ 0.03；偏航 0.6011861926754793 ≤ 2.0；末速 0.00026129710678484644 ≤ 0.05 |
| s03_pick（**机械臂真抓取**） | EXECUTED / SUCCEEDED | `pose_tolerance_m` **9.387527248483805e-06** ≤ 0.005；`min_lift_delta_m` **0.088446** ≥ 0.02；`require_bilateral_contact` **1.0**；墙钟 53.69587149005383 s |
| s04/s05 | SKIPPED_PENDING | 能力待交付（`place_object` / `accept_payload`） |

这条链上四个真缺陷（全部本轮或前几轮修掉，都有实测依据）：

1. **缺重力前馈** ⇒ 关节稳态下垂 5.314844537182922e-03 rad ⇒ 末端 z 向 11.904 mm（§8）；
   修：构建期生成四相位 `gripper.gravity_feedforward`（`gravity_hold_ctrl` + 静态保持自证），
   改后关节残差 2.9338248136322893e-05 rad。
2. **`run_scenario` 装配不传 `declaration_document`** ⇒ `scene_report` 模式的机械臂根本装配不了
   （`'str' object has no attribute 'get'`，exit 4）。
3. **`_dispatch_step` 假定后端有 `read_state`** ⇒ 机械臂首次被下发即 `AttributeError`；
   现按"无测量"处理（缺依据的判据判失败，不静默通过）。
4. **租约 TTL 短于墙钟 + 收尾释放路径严格** ⇒ 抓取实际 53.69587149005383 s 而 TTL 30 s，
   收尾 `release` 抛 `LeaseConflict` 把整步变成未捕获异常、**报告写不出来**（磁盘上留着上一轮的
   陈旧报告 —— 我据此把旧失败原因当成新结果，误判过一次）。修：TTL 30 → **120 s**（同一类问题的
   第三例，前两例是 locomote 60 s / dock_for_handoff 120 s）；`SkillRuntime` 收尾改为
   容忍"已过期"并留痕（`cleanup_note`），而"租约被顶掉"仍显式失败。

判据/观测纪律（本轮新增两条）：
- 排查"声明了参数却不起作用"必须看**实参**：`IRAF_DEBUG_PICK=1` 打印 `PICK_INPUTS`（调用序号、
  时间戳、duration_ms、每段时长、四段 positions 键、前馈取值）与 `PICK_TRACE`（逐相位残差与关节角）；
- **磁盘上的报告可能是上一轮的**：进程崩溃时报告不落盘 ⇒ 读报告前先确认本次运行真的写成功了
  （本次的教训：把陈旧报告的失败原因当成新结果）。

## 11. 联合世界迁移（2026-09-24）：`--world joint` 已可用，s02 在联合模型通过

声明与执行器的改动（默认行为不变）：
- `config/scene.schema.json`：`scene_baseline` 新增可选 `robots_joint`（联合世界的机型绑定）；
- `scenes/handoff_lab/baseline.yaml`：新增 `robots_joint: {piper: config/machines/piper_joint.yaml,
  unitree_go2: config/go2_joint.yaml, humanoid_static: 待交付}`（保留原 `robots` ⇒ 单本体验收仍可复现）；
- `scripts/scenario.py`：`run --world {single,joint}`（缺省 single）；`machine_declaration(...,
  world=)` 按世界取绑定、缺声明即显式失败；装配时按**声明角色排序**（owner 先、guest 后）并传入
  `plant_registry`；报告新增 `"world"` 字段留痕。

实测：
- `stand_stop --world joint` ⇒ **exit 0 / passed=true**（stand 末速 2.6904769153005103e-05 m/s；
  stop 末速 0.0026950035834258264），报告 `world: joint` ✓ 两个本体在同一株植物上装配并执行。
- `nominal --world joint` ⇒ exit 5（诚实红）：
  · s01 ✓（2.6904769153005103e-05 m/s）
  · **s02_dock ✓ 在联合模型里通过**：平移 0.028587598959680105 ≤ 0.03、偏航 0.6011883605747808 ≤ 2.0、
    末速 0.0002612973083285042 ≤ 0.05（与单本体基准 0.02858758381668675 同量级 —— 狗是独立刚体，
    加入臂不改变它的动力学，这正是预期）
  · s03_pick 失败：`等待 owner(unitree_go2) 推进到第 10751 步超时（0.060 s，当前 10750 步）`

### 11.1 架构性卡点（本轮判死，下一步的入口）

场景执行器是**逐步串行**的：执行 s03 时狗的 runtime 并不在工作 ⇒ **没有人在推进植物**
（植物只在某个后端 `step` 时才前进）。这不是带宽或超时系数问题（我用 1 步 × timestep 0.002 s ×
系数 30 = 0.060 s 的超时精确对上），而是**"一份植物 + 多控制器"里"时间推进者必须在有人工作时一直活着"**
这条约束没有被满足。

而且四足是**力矩型执行器**（PD 在适配器每拍算，见 `config/go2_loopback.yaml` 的 `torque_limit_source:
model` 与 loopback 报告 `stop.collapsed: true`）⇒ 没有人给它算控制量时它会**塌**（§5.2 已量化：
0.1 s 内 0.288372 → 0.077172）。所以联合世界里"狗站着、臂去抓"这件事，要求狗的控制器在**臂执行期间
持续在线**，否则托盘会随狗塌掉而离开臂的可达范围。

**修法（下一步，按声明而非硬编码）**：
1. 在 owner 的机型声明里增加"植物驻留"声明（例如 `robot.plant.hold: {skill: stand, duration_ms: N,
   reissue: true}`），表示"联合世界执行期间由本本体持续执行该技能维持植物状态"；
2. `run_scenario` 在 world=joint 时按该声明起一个**后台驻留线程**（循环执行该技能；技能本身走
   SkillRuntime，租约 TTL 需覆盖循环周期 —— 已有 TTL 经验见 §10 第 4 点），并在所有步骤结束后
   停止它、把"驻留期间的实测"（如躯干高度序列）写进报告；
3. 判据仍然只来自实测：臂抓取的三项判据 + 驻留期间狗的高度/姿态不越界（阈值来自声明）。

### 11.2 植物驻留已实现（按 §11.1 的修法），但联合世界运行**崩溃**（未定位，诚实登记）

已落地（声明驱动，仅 `world=joint` 生效 ⇒ 单本体默认路径不受影响）：
- `config/go2_joint.yaml`：owner 声明 `robot.plant.hold: {skill: stand, duration_ms: 8000}`；
- `scripts/scenario.py`：新增 `_start_plant_residency` / `_stop_plant_residency` —— 联合世界下按声明
  起后台线程**循环执行 owner 的 hold 技能**（走 SkillRuntime；`deadline_offset_ms=600000`；
  单周期失败即停止驻留并留痕），步骤执行结束后停止，汇总（周期数/失败周期/owner 收尾基座高度）
  写进报告 `plant_residency` 字段；owner 未声明 `hold` 时**拒绝跑联合世界**（不猜）。

实测：`nominal --world joint` ⇒ **exit 139（段错误，core dumped）**，进程在写报告**之前**崩掉。
⚠ 判读陷阱（本战役第三次踩到）：磁盘上的 `build/acceptance/handoff_lab/nominal/report.json`
是**上一轮**（无驻留那轮）的陈旧报告（mtime 17:45:47 vs 运行时刻 17:47:35），里面那句
`等待 owner(unitree_go2) 推进到第 10751 步超时` **不是**本轮驻留版的结果 ⇒ 本轮驻留是否生效
**未观测到**（不能据陈旧报告下结论）。

下一步（先定位崩溃，再谈驻留效果）：
1. 崩溃特征（SIGSEGV）指向**跨线程 MuJoCo 访问**：驻留线程里 owner 的 `stand` 会 `stamp` 步进植物，
   主线程的臂在同一条时间线上写 ctrl 并 `mj_forward` —— 两者虽共用植物锁，但 MuJoCo 的
   C 侧调用在多线程下的边界值得怀疑（也可能是我在某处绕过了锁）。
   定位手段：① 用 `faulthandler.enable()` + `-X faulthandler` 抓崩溃时的 Python 栈；
   ② 若确认是并发访问 ⇒ 改成**单线程时间线**（owner 的驻留与新提案合并：步骤执行时由
   "谁在工作谁推进"统一到一个调度点），而不是两个线程争锁；
2. 定位后重跑 `nominal --world joint`，判据同样是 s03 的三项实测 + `plant_residency.cycles ≥ 1`
   且 `failed_cycles = []` + `owner_final_base_z_m` 在声明范围内。

### 11.3 段错误已修（持锁粒度）+ 驻留生效 + 新的精确卡点：参考姿态是**场景专属**的

**段错误根因与修法**（faulthandler 抓到栈顶）：`quadruped.gravity_bias_torque` 内部会
`mj_forward`（**写**派生量），而四足控制循环 `_run_control` 读 `qpos/qvel/time` 与它时**没持植物锁**
⇒ 与对侧（臂，主线程）在同一份 MjData 上并发 `mj_forward`/`mj_step` ⇒ SIGSEGV。
修法（刻意**按控制周期**持锁，不是整段）：`_run_control` 里三处（读 q/dq 并 `copy()`、读 `time`、
算 `tau_ff`）加 `with self._lock:`；整段持锁会把 guest 的 `wait_until` 饿死（它只在锁外采样步数）。

**修后实测**（`nominal --world joint`，exit 5，报告已落盘；`Fatal Python error` 计数 0）：

```
plant_residency: cycles=16  failed_cycles=[]  wall=26.868515  owner_final_base_z_m=0.2760156601664448
  ⇒ 狗在臂执行期间**全程站着**（0.276 ≈ 站姿高度；对照：无人控制时 0.1 s 内塌到 0.077172）
s01_verify_ready ✓（推进 16.00000000000201 s、末速 1.4887881152579008e-05 m/s）
s02_dock ✓：平移 0.028125012624229208、偏航 0.5801787093348428、末速 0.00024289065500333266
s03_pick ✗ 末端未到达目标抓取位姿: distance=0.369305m tolerance=0.005000m
          delta=[0.2600008873862923, -0.2622663598754907, -0.001477328276297013]
```

**新卡点（精确）**：残差 0.369 m 且方向在 x/y 平面（+0.26, −0.26）⇒ 臂**够不到**联合模型里的目标。
原因不是控制、也不是判据，而是**数据出处**：`piper_joint.yaml` 指向的联合报告把 gripper 的
`home/approach/grasp/lift_positions` 从**臂自己场景**的报告继承了过来（`_joint_manipulation` 的改名规则
只改名字、不改数值），而那组参考姿态是按"方块在 (0.19, 0, 0.025)"求解的；联合模型里方块的世界位置
由 go2 场景的 props 决定 ⇒ 命令位姿落在联合模型里**另一个地方**。`grasp_pose` 本身是对的
（由 `grasp_pose_from: report_target` 从**联合报告**的 targets[] 组装），错的是**命令位姿**。

修法（下一步，构建期、按声明）：
1. 联合构建时对**联合模型**重跑臂侧参考姿态求解器（`build_piper_baseline` 的
   `build_reference_poses` / `solve_position_ik` 一族），把 home/approach/grasp/lift 的关节解与
   `pad_offset_m` 写进联合报告（数值来自联合模型，不再是"继承来的"）；
2. 若短期不便接：至少在 `_joint_manipulation` 里**显式失败**而不是静默继承（
   "继承的参考姿态是场景专属的，联合模型必须重算"）—— 静默继承正是这次 0.369 m 的来源；
3. 验收：s03 的三项实测（`pose_tolerance_m` ≤0.005 等）+ 驻留 `cycles ≥ 1`、`failed_cycles = []`。

### 11.4 0.369 m 的完整解释与修法（几何量清，可达性没问题）

探针 `build/iraf-a6a14/joint_reference_frame_probe.py` → `joint-reference-frame-probe.json`
（把**继承的命令位姿**写进 qpos 后 FK，量指腹中点与目标中心）：

| | 目标中心 (box_01) | 臂基座 | 基座→目标 | 命令位姿下的指腹中点 | 与目标中心差 |
| --- | --- | --- | --- | --- | --- |
| 联合模型 | (0.19, 0.0, 0.025) | (0.45, −0.45, 0.123) | (−0.26, **+0.45**, −0.098) | (0.45, **−0.26**, 0.054828181) | **0.368903942 m** |
| 臂自己场景 | (0.189999426, 0.0, 0.025) | (0.0, 0.0, 0.123) | (+0.19, 0.0, −0.098) | (0.189999234, 0.0, 0.054828181) | 0.029828181 m（= pad_offset） |

- 方块位置在两模型里**几乎相同**（差 5.74e-07 m）；**臂基座差 (0.45, −0.45, 0)** ⇒ 差异只来自臂的 placement。
- 命令位姿把指腹放在"基座 + (0, +0.19)"（即 (0.45, −0.26)）—— 这正是**臂自己基座系**下 (0.19, 0) 的镜像，
  说明那组 `grasp_positions` 是**在臂自己基座系里求解的关节解**，MJCF 的 qpos 是局部量 ⇒ 换个基座位姿
  就直接照搬 ⇒ 指腹落在别处。这不是控制问题、也不是可达性问题：**联合模型里基座→目标 0.5197 m
  ≤ 臂可达 0.594284 m（§11.1 实测）⇒ 够得到**。

**修法（下一步，明确的输入输出）**：把目标位置换算到**臂基座系**后重算参考姿态：
1. 由 `robots[].placement`（`pos_m` + `quat_wxyz`）求出臂基座系到世界系的旋转 R 与平移 t；
2. 局部目标 = Rᵀ·(目标世界位置 − t)（本场景：Rz(−90°)·(−0.26, 0.45, −0.098) = (0.45, 0.26, −0.098)，|xy| = 0.5197 ✓）；
3. 用**局部目标**跑臂侧参考姿态求解器（`scripts/build_piper_baseline.py` 的
   `solve_position_ik` / `build_reference_poses` 一族，当前它只会按 baseline 里的
   `model.source` 自建模型 ⇒ 需要泛化成"接受任意 model + 局部目标 + 指腹几何名"）；
4. 把结果写进联合报告的 `gripper.{home,approach,grasp,lift}_positions` 与 `pad_offset_m`
   （数值来自**联合模型 + 该 placement**，不再是继承来的）；
5. 验收：探针的 `pad_mid_to_target_norm_m` 回到 ~0.029828181（pad_offset 量级），
   再跑 `nominal --world joint` 判 s03 三项实测。

**在重算落地之前**：`_joint_manipulation` 的静默继承必须至少**可见** —— 下一次改动会在联合报告里
加一项 FK 自检（用联合模型 + 继承位姿算指腹中点与目标的差，写进报告字段；差超声明阈值即可见告警，
不静默），这正是本次 0.369 m 之所以能藏到运行期的原因。

### 11.5 参考姿态重解已实现并量化（下一步只需把它写进联合报告）

改动：`scripts/build_piper_baseline.build_reference_poses` 新增两个**可选**覆盖参数（缺省 None ⇒
行为逐位不变）：`target_xy_override_m`（抓取点相对**臂基座**的 xy）与 `target_z_override_m`（抓取点 z）。
探针 `build/iraf-a6a14/joint_reference_resolve_probe.py` → `joint-reference-resolve-probe.json`：

```
绑定位姿 pos = (0.45, -0.45, 0.0)、绕 z 90°；世界目标 = (0.189999426, 0, 0.025)
⇒ 基座系局部目标 = (0.45, 0.260000574, 0.025)，|xy| = 0.519711746 m ≤ 可达 0.594284 m ✓
重解 grasp 解: joint1 0.546062498 / joint2 2.310233536 / joint3 -1.192424598 /
              joint4 0.115442695 / joint5 -0.60488275 / joint6 1.89e-07
```

门禁残差的**三级台阶**（同一口径：pad中点 − 轴·pad_offset − 目标中心）：

| 用哪组解 / 哪个 pad_offset | 残差 |
| --- | --- |
| 继承的位姿 + 继承的 pad_offset（当前联合报告） | **0.367696068 m** |
| **重解**的位姿 + 继承的 pad_offset | **0.018389447 m**（残差几乎全在 z） |
| **重解**的位姿 + **重解**的 pad_offset（`finger_height_correction_m` = **0.01145198**） | **5.681e-06 m** ✓ |

⇒ 修法完整且已量化：**位姿与 pad_offset 必须一起重解**（pad_offset 不是通用常数，它由该场景的
方块尺寸/指尖配平决定：臂自己场景 0.029835769、联合场景 0.01145198）。
也解释了 z 覆盖为何"无效"：`grasp_target[2]` 只是初值，真正决定高度的是 `balance_tip_clearance`
的指尖离台配平（本次 `finger_tip_z_m` = 0.005829938、`tip_clearance_m` = 0.005）。

**下一步（把重解接进构建，按声明而非硬编码机型）**：
1. 在场景/机型声明里显式给出"联合模式参考姿态求解器"（如 `robots[].reference_solver:
   {module: build_piper_baseline, baseline: config/piper_simulation_baseline.yaml}`），构建器**按声明**
   调用它（保持"机型差异只进 profiles/config"的纪律，不在 scene_builder 里写 piper 特例）；
2. 把结果写进联合报告的 `gripper.{home,approach,grasp,lift}_positions`（加前缀改名）与
   `pad_offset_m`，并在报告里留 `reference_pose_source: resolved_for_joint_model` 出处；
3. 验收：`reference_pose_check.distance_m` ≤ 声明容差（预期 ~5.681e-06）、
   再跑 `nominal --world joint` 判 s03 三项实测。


### 11.6 参考姿态重解已接进构建（2026-09-24 续）：0.367696068 m → 5.663e-06 m，卡点转移到**臂动力学口径**

**改动（全部按声明，构建器内不出现机型/脚本名）**

- `scenes/handoff_lab/scene.yaml`：`robots[piper].reference_solver: {module: scripts/build_piper_baseline.py, entry: build_reference_poses, baseline: config/piper_simulation_baseline.yaml}`。
- `config/scene.schema.json`：增 `reference_solver` 定义 + **fail-closed 规则**（声明了 `manipulation_report` 就必须同时声明 `placement` 与 `reference_solver`：静默继承即构建失败）。
- `src/iraf_adapters/unitree/scene_builder.py`：`_load_declared_callable`（按声明动态加载仓内脚本；加载前把脚本目录放进 `sys.path` —— 实测平级导入坑）、`_joint_reference_resolution`（世界目标 → 臂基座系、按声明调用、校验四相位 + `finger_height_correction_m`、摆放含倾斜即显式失败）、`_joint_manipulation` 覆写四段 `*_positions` 与 `pad_offset_m` 并写 `reference_pose_source` / `manipulation.reference_pose_resolution`。
- `scripts/build_piper_baseline.py`：抽出公开入口 `build_reference_feedforward(model, reference, baseline, prefix)`；**臂场景产物逐位不变**（`build/iraf-a6a14/piper-pick-scene-before.*` 对比重建结果，JSON/XML/pose 三份 diff 全空）。
- `tests/unit/test_joint_name_map.py`：新增两条回归 —— 缺 `reference_solver` ⇒ 显式失败；重解值**逐关节**覆写继承值并留痕。

**实测（`build/scenes/handoff_lab/handoff_lab_joint.json`）**

- `manipulation.reference_pose_check.distance_m` = **5.663e-06 m**（容差 0.005，`within_declared_tolerance: true`）；上一版 0.367696068 m。
- `resolved_pad_offset_m` = **0.011452003**（继承 0.029835769）；`local_target_m` = [0.45, 0.26, 0.025]、`local_xy_radius_m` = **0.519711458** ≤ 可达 0.594284 m；approach/grasp/lift 各 6 个关节被替换，home 无差（两侧都是零位）。
- 与探针 5.681e-06 的差（1.8e-08）有明确出处：构建器取**模型里 `box_01` 的 xpos**（0.19, 0, 0.025，与运行期门禁同源），探针取臂侧报告的 `position_m`（0.189999426）。
- `scripts/scene_check.py --scene scenes/handoff_lab` ⇒ exit 0 / `passed: true`。

**`nominal --world joint` 复跑：s03 仍红，但原因换了**

```
s01_verify_ready SUCCEEDED
s02_dock         SUCCEEDED  （0.028125012624229208 m / 0.5801787093348428 deg，与上一轮逐位相同）
s03_pick         FAILED     distance=0.274949m tolerance=0.005000m
                 delta=[0.24114787436338767, 0.09218743459485515, 0.09458519690813935]
                 末态 qpos joint5=-1.0358268081644664 joint6=-0.5718125594197493
                 （命令 joint5=-0.6048826552919585 joint6=1.892254047965281e-07）
```

上一轮 delta 是 [0.26, −0.2623, −0.0015]（纯 xy 错位 = 继承旧解），本轮残差换向 ⇒ **位姿重解确实生效**（构建期 FK 已证 5.663e-06），剩下的是**跟踪/保持**问题。

**机制判死（探针 `build/iraf-a6a14/joint-reference-feedforward-probe.json`）**

联合模型里的臂来自 **Profile 声明的厂商 MJCF**（`--attach` 用 `spec.model.file`），臂自己场景注入的是**声明增益**（`scene.arm_position_kp: 200.0`，逐关节取 `max(kp, damping×1.5)`）：

| 关节 | 联合模型 kp | 臂自己场景 kp | 关节阻尼 |
| --- | --- | --- | --- |
| joint1 | 10000 | 450 | 300 |
| joint2 | 2000 | 200 | 100 |
| joint3 | 500 | 200 | 20 |
| joint4 | 50 | 200 | — |
| joint5 | 20 | 200 | — |
| joint6 | 5 | 200 | — |

⇒ **两个模型里的臂不是同一个动力学系统**。静态保持残余（`gravity_hold_ctrl` 同口径，hold 4000 ms、容差 `tolerance_rad` 0.001）：

| 模型 | home | approach | grasp | lift |
| --- | --- | --- | --- | --- |
| 联合模型 | 1.102e-06 ✓ | 0.029975223 ✗ | 0.036213257 ✗ | 0.206096435 ✗ |
| 臂自己场景 | 2.8763e-05 ✓ | 3.812e-06 ✓ | 3.809e-06 ✓ | 3.816e-06 ✓ |

门禁预测（联合模型 + 重解姿态 + 正确前馈，保持 1600 ms = s03 每段时长）：grasp 命令时 5.681e-06 m ⇒ **7.165929e-03 m**；approach 0.030983737、lift 0.045533780。

#### ⚠ 11.6 更正（同日，我自己踩的坑）：上面那张静态保持表**被污染**，不能据此断言"增益差异是机制"

更正依据（实测，`build/iraf-a6a14/joint-finger-geometry-probe.json`）：
把臂关节设为重解姿态时，**夹爪关节（`piper_joint7/8`）没有被设值** —— 探针与
`build_reference_feedforward` 都只把**臂关节**传给了 `gravity_hold_ctrl` 的 `hold_positions`，
于是 joint7/8 停在模型默认/关键帧状态：

```
指腹张开向量模长：张开时 0.090362481 m ｜ 闭合时 0.020378284 m（且 left↔right 指腹 dist = −0.0）
指腹与方块重叠：box_01_geom ↔ piper_left_finger  −0.033339 m
                box_01_geom ↔ piper_right_finger −0.032634 m
```

⇒ 手指闭合时**指腹本身就卡在 50 mm 方块里**，接触力当然会把臂顶离姿态，
"联合模型保持残余 0.029975223/0.036213257/0.206096435"里有多少来自**增益差异**、
多少来自**手指状态**，**目前分不开**。臂自己场景那张表（3.8e-06 全通过）之所以好看，
是因为它的关键帧里 joint7/8 恰好是**张开**（与 approach/grasp 的指令一致）——
属于**侥幸**，不是设计。

**这一条必须记住**：静态保持/前馈的 `hold_positions` 必须给**该相位的完整位置指令**
（臂关节 + 该相位声明的 joint7/8 开合），否则量的是"手指卡在方块里"的动力学。
`gripper.{approach,grasp,lift}_positions` 里本来就带 joint7/8（approach/grasp 张开 0.035、
lift 闭合 0.023）⇒ 修法是把它们一并传进前馈入口，而不是新造数据。

**另一条独立事实（已实测，与手指无关）**：摆放偏航决定 joint1 是否为 0，从而决定夹爪绕竖直轴
的朝向是否与方块面平行：

| 摆放 yaw | joint1（重解） | 基座系目标 | 指腹张开向量的朝向 |
| --- | --- | --- | --- |
| 90°（原声明） | 0.546062498 rad（31.3°） | [0.45, 0.260000574, 0.025] | 绕竖直轴偏 25.3°⇒斜切方块棱边 |
| 120°（本次改，未提交） | 0.000357393 rad | [0.519711432, 0.000166605, 0.025] | 与方块面平行 |

⇒ 本文件 §11.5 的量化结论（位姿 + pad_offset 必须一起重解：5.663e-06 m）**仍然成立且已提交**
（提交 d7ef6da）；需要重做的是"保持/前馈"这一层的测量与修法，**顺序**：
① 前馈入口按相位传全量位置指令（含 joint7/8）；② 摆放偏航改为 120°（让 joint1≈0）；
③ 重测"联合模型 vs 臂场景"的静态保持残余 —— 只有那时才能判断执行器增益口径是否需要声明化。

**当前工作树状态（未提交，故意）**：`scene_builder` 的按声明增益注入、Profile 的
`spec.model.position_gain`、`scene.yaml` 的 `feedforward_entry` 与 120° 摆放、基线的
`arm_position_kp_damping_ratio` 都在工作树里；其中 `feedforward_entry` 会让联合构建在
静态保持判据处**失败（exit 5）**⇒ 在 ① 修好之前**不得**提交。

**因此本轮故意不把 `feedforward_entry` 加进声明**：它在联合模型上会因静态保持判据失败而让构建直接失败（数值见上表），从而掩盖"位姿重解已生效"这一已验证成果。顺序应当是：先声明化**执行器刚度口径**（联合模型里附加本体的执行器必须与已验收的臂场景同一口径），再把前馈重算接进声明 —— 两者都用现成判据验收：四个参考姿态在联合模型上的静态保持残余 ≤ 声明 `tolerance_rad`。

### 11.7 保持/前馈为何不达标：**姿态把 `piper_link6` 插进方块 14.516 mm**（2026-09-24 判死）

**结论（构建期即可见，无阈值：存在接触对即侵入）**

```
命令姿态 @ qpos0      ： box_01_geom | <未命名#66>(body=piper_link6)  dist = -0.014516 m
命令姿态 @ keyframe 0 ： box_01_geom | <未命名#66>(body=piper_link6)  dist = -0.014516 m
（其余与方块相关接触均为 workbench|box_01_geom = +0.000000，属正常支撑）
```

指腹中点是对的（FK 5.663e-06），但**腕部 link6 已经插在方块里 14.516 mm**；两种初值下逐位相同 ⇒ 纯几何。
接触力把臂顶离姿态：保持 4 s 的臂残差 t=0.2 s **0.037687312 rad** → t=4.0 s **0.039962049 rad**（限 0.001），
指腹随后被推进方块（`piper_left_finger` −0.020710 / `piper_right_finger` −0.028479），
而方块自身只漂移 **0.006188 m**（[0.19, 0, 0.025] → [0.188568, −0.004174, 0.020661]）⇒ **是臂被推**。

**根因层**：位置-only 的参考姿态求解器 + 本场景几何（目标距基座 0.519711458 m，近伸直位形）。
臂自己场景的目标距基座只有 0.19 m（折叠位形），link6 远离方块，所以同一套配平规则下是 3.8e-06。
⇒ 必须在**求解器层**加"姿态不得让臂 link 侵入目标"（可复用 UR5e 侧 `GraspPoseSolver` 的
`pointing_direction` 口径让腕部停在目标上方）。

**本轮否定掉的假设（全部有数字，不要重走）**

| 假设 | 实测 |
| --- | --- |
| 够不到 | 0.519711458 ≤ 可达 0.594284 ✗ |
| 把臂搬近即可 | 基座移到 0.313209195 m 后参考几何正常（FK 6.605e-06、pad_offset 0.026730756），但 **s02_dock 崩**（`Provider output does not match schema: None is not of type 'number'`）⇒ y=−0.30 落在四足走廊里 ✗ |
| 基座浮动 | 两模型基座漂移 **0.000000000 m** ✗ |
| 执行器增益差异 | 注入后联合 XML 实测 kp = 450/200/200/200/200/200（与臂场景同值），残余仍 0.039962033 ✗ |
| 夹爪关节没设值 | 命令姿态下指腹**不**接触；给/不给夹爪两口径的残余同值 ⇒ 不是主因 ✗ |
| 四足塌下来踢了方块 | "四足未受控" 与 "四足按关键帧守住" 两组轨迹**逐位相同**（方块漂移、FL_thigh、接触数、臂残差全部一致）✗ |

**新增的构建期可见性（本轮交付）**：联合报告的 `manipulation.reference_pose_clearance_check`
（判据与臂场景 `validate_grasp_pose` 一致：张开时手指不与任何物体接触；**归属按 `geom_bodyid` → body 前缀判定**）。
为什么必须按 body 判定：联合模型里 `piper_link1..link6` 的 geom **全部无名**（id 60–66），
按"名字带 `piper_` 前缀"过滤会把唯一真正侵入的那个 geom **静默跳过**（我的第一版就是这么漏的）。
回归测试：`tests/unit/test_joint_reference_clearance.py`（无名 geom 侵入必须报出；主本体接触不算侵入；
缺目标时 `skipped`）。**同一个盲区也存在于臂侧** `validate_grasp_pose`（只查指尖/手指，不看 link）—— 登记为债。

#### 11.7 附：抬高抓取点**不是**修法（实测否掉，2026-09-24）

探针 `build/iraf-a6a14/joint_grasp_height_sweep_probe.py` → `joint-grasp-height-sweep-probe.json`
（把目标 z 抬高 0..28 mm，每个取值量 link 侵入 / 指腹高度 / 门禁残差）：

| 抬高 | pad_offset_m | link 侵入 | 指腹 z | 门禁 FK 残差 |
| --- | --- | --- | --- | --- |
| 0 mm | 0.011452003 | −0.014516 | 0.035430 | **5.603e-06** ✓ |
| 4 mm | 0.007140908 | −0.014648 | 0.035122 | 3.993018e-03 ✗ |
| 8 mm | 0.002824746 | −0.014781 | 0.034811 | 7.994479e-03 ✗ |
| 12 mm | 0.000000000 | −0.014294 | 0.035993 | 1.1999379e-02 ✗ |
| 20 mm | 0.000000000 | −0.010411 | 0.043984 | 1.9999265e-02 ✗ |
| 28 mm | 0.000000000 | −0.004522 | 0.051974 | 2.7999150e-02 ✗ |

两个后果一起看就否掉了这条路：
1. **门禁口径不允许**：门禁算的是"指腹中点 − 轴·pad_offset − **方块中心**"，方块中心由物理（台面 + 半边长）决定，
   抬高"抓取点"不能移动方块 ⇒ 抬高多少、残差就是多少（4/8/12 mm 对应 3.99e-03/7.99e-03/1.20e-02）；
   同时 `balance_tip_clearance` 不再需要修正 ⇒ `pad_offset` 掉到 0，口径彻底失衡。
2. **link6 也抬不出去**：即使抬 28 mm，侵入只从 −0.014516 缩到 −0.004522（仍为负）⇒ 侵入不是"高度不够"，
   而是**姿态本身**（腕部俯仰）把 link6 送到方块里。

⇒ 修法只能落在**姿态**上：让夹爪轴受**声明的** `grasp.approach_direction`（基线已声明 `[0, 0, 1]`，
目前只被"预抓取偏移"与"指尖配平方向"消费、**没有**参与 IK 约束）约束 —— 即 UR5e 侧
`GraspPoseSolver(pointing_direction=…)` 已经有的那种朝向约束。判据现成：`reference_pose_clearance_check.overlapping == false`
且四姿态静态保持残余 ≤ `gravity_feedforward.tolerance_rad`。

### 11.8 朝向约束的落点已定位（侦察，2026-09-24）：core 已有位姿型 IK，Piper 缺两处声明

§11.7 的修法是"把夹爪轴约束到声明的接近方向"。侦察结论（**不需要在 Piper 侧另写求解器**）：

| 已有 | 位置 | 口径 |
| --- | --- | --- |
| `iraf_core.kinematics.solve_pose_ik(model, data, target_position, target_rotation, arm_joints, points, flange_site, …)` | core，与机型无关 | 位置误差取 `points` 的世界系算术平均（与 `solve_position_ik` **同一口径**）、姿态误差取**法兰 site** 的旋转向量，拼成 6 维做阻尼最小二乘；残差可接受性由调用方门禁判定 |
| `GraspPoseSolver`（`scripts/build_robot_baseline.py:144`） | 通用编排层 | 只做接线：① 臂关节 id/夹持区 geom/法兰 site；② 由**模型实测的局部轴**推出目标旋转（`v_point_local`/`v_spread_local` → `R = W·Lᵀ`），避免假设"法兰 x 轴 = 开合轴"；③ 求解后按同一口径量中点与两轴 |

Piper 侧缺的只有两处**声明**（缺即显式失败，不给默认值）：

1. **法兰 site**：`solve_pose_ik` 要一个 MuJoCo site 作为工具坐标系，而厂商 Piper MJCF **没有 site**
   （Profile 只声明了 `spec.model.bodies.wrist = link6`，那是 body 不是 site）。UR5e 侧是
   `scripts/assemble_ur5e_2f85.py` 在装配期注入法兰 site + 基线声明 `flange_site` ⇒ Piper 侧照同一口径：
   Profile 声明 `spec.model.flange_site`，由场景/探测构建器在 `link6` 处注入（**不能在已编译模型上补 site**）。
2. **张开轴（开合轴）**：UR5e 基线声明 `grasp.spread_axis: [1,0,0]`，Piper 基线**只声明了**
   `grasp.approach_direction: [0.0, 0.0, 1.0]`（目前只被"预抓取偏移"与"指尖配平方向"消费，**没进 IK**）。
   本场景方块是轴对齐的（`props[box_01].pose.quat_wxyz = [1,0,0,0]`），而臂自己场景实测的张开向量是
   `[0.000729041, 0.09035789, 0.000546168]`（≈ 世界 y，face-on）⇒ 联合路径要的张开轴可由**目标自身声明的
   姿态**机械推出（`targets[].quaternion_wxyz` 的一条轴），而不是在代码里写死方向。

**为什么必须做成"联合路径可选启用"**：同一条 `reference_solver.baseline`
（`config/piper_simulation_baseline.yaml`）也被**臂自己场景**使用，而臂侧已验收的数字（抓取残差 9.39e-06 m、
`pad_offset_m 0.029835769`、`validate_grasp_pose` 全过）都建立在**位置型**解上；把 Piper 求解器整体换成
位姿型会让臂侧产物与验收数字全部改变。故取向：`reference_solver` 声明里给可选 `args`（如
`axis_constraint: {pointing: approach_direction, spread_axis: target_axis_y, orientation_tolerance_rad: …}`），
构建器按声明把它透传给求解器入口；**不声明则逐位不变**（臂侧零影响）。

验收判据（两条同时绿才算修好，都是现成的）：
`manipulation.reference_pose_clearance_check.overlapping == false`（当前 `piper_link6 dist_m -0.014516`）
**且**四姿态静态保持残余 ≤ `gravity_feedforward.tolerance_rad`（0.001，当前 0.039962049）。

## 12. 下一步

0. **（2026-09-24 判死，§11.7）** 求解器层：参考姿态必须**不得让臂 link 侵入目标**（当前 `piper_link6` 与方块重叠 −0.014516 m ⇒ 保持残余 0.039962049 rad）；可复用 UR5e `GraspPoseSolver` 的 `pointing_direction`：把夹爪轴约束到**声明的** `grasp.approach_direction`（§11.7 附：抬高抓取点已被数字否掉 —— 门禁口径不允许，且抬 28 mm 侵入仍为负）。修完再声明 `feedforward_entry` 并判 s03。
0b. **（2026-09-24 新增）** 附加本体的执行器刚度口径声明化（联合模型里的臂必须与臂自己场景同一口径：Profile 声明 → 构建器按声明注入 `kp = max(arm_position_kp, 关节阻尼×ratio)`）；验收判据 = §11.6 的静态保持残余表；通过后再把 `feedforward_entry` 加回 `reference_solver`。
1. `s03_pick` 三项判据口径（`pose_tolerance_m` / `min_lift_delta_m` / `require_bilateral_contact`）
   → s04/s05 → `nominal` 全场景 → 两臂轮番运输。
2. 联合产物的资产相对化（消 host-specific）；债 S4（臂厂商资产入库 + 重写锁 + 许可证 BOM）；S3。
3. owner 侧作用域的**声明化**（现在 owner = 模型全部执行器；联合植物里应显式声明"我拥有哪些"）。


