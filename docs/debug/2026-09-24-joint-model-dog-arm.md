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

### 11.9 联合世界的**带窗口演示**：卡在"owner 谁来推进"，不是管线（2026-09-28）

目标（使用者要求）：方块在臂旁边 → 狗站稳 → piper 从台面抓起 → 放到狗背上。

已验证（**无窗口**，走验收链路）：`nominal --world joint` ⇒ exit 0 / passed=true，
s01 stand 1.488788112060832e-05 m/s、s02 dock 0.02812501214102655 m / 0.5801787035003824°、
s03 pick `grasp_center_distance_m = 0.0002098657943494681` / `lift = 0.079403` / `bilateral = 1.0`、
`plant_residency cycles=23 / failed_cycles=[]`（提交 `ed2035c`）。

带窗口演示（`build/iraf-a6a14/joint_handoff_demo.py`，三段：stand → dock → **同一世界里抓取**）：

| 段 | 结果 | 数字 |
| --- | --- | --- |
| A 狗 stand | SUCCEEDED | base_xyz [−0.006983, 0, 0.279358]，窗口 frames 296 |
| B 狗 dock | SUCCEEDED | 0.028375962824781182 m / −0.4095659107671862°，frames 772 |
| C 臂 pick（guest） | **FAILED** | reason「等待 owner(unitree_go2) 推进到**第 36751 步**超时（0.060 s，当前 36750 步）」；臂已走到 approach 一半（joint3 0.001852 → −0.120904） |

**判死过程（两步，避免把症状当原因）**：
1. 先把 `guest_timeout_factor` 30× → 300×（每步 0.060 s → 0.600 s）复跑：**仍停在同一第 36750 步**
   ⇒ 超时不是原因，owner 是**真的停止推进**了。故该系数已**回退**到 30.0（不无理由放宽安全余量）。
2. 真因：窗口演示里 owner（狗）只被一条 `stand` 请求驱动，而**请求时长受安全策略 `max_duration_ms` 限制**
   （实测 60000 ms 直接被技能层拒：狗一步没动）⇒ 30 s 仿真时间用完，`run_request_live` 返回、
   时间推进停止，而 guest（臂）的 pick 需要更多仿真时间（36750 步 = 73.5 s 仿真）⇒ 确定性卡住
   （每次复跑同一步，正是"确定性"而非"墙钟抖动"的证据）。

**已落地（2026-09-28 同日）**：`scripts/scenario.py run` 新增 `--display {auto,none,interactive_viewer,offscreen_frames}`
`--render-hz` `--seconds`，配 `viewer_runner.run_live_mirror`（**只渲染、不推进**，与 `run_request_live`
同一套 `SnapshotMirror` 并发契约）。实测：

```
PYTHONPATH=src python3 scripts/scenario.py run --scene scenes/handoff_lab --scenario nominal \
    --world joint --display interactive_viewer --render-hz 20 --seconds 4
⇒ exit 0 / passed=true
  s03 pick SUCCEEDED  grasp_center_distance_m = 0.00020960033543239453（无窗口时 0.0002098657943494681）
                      grasp_lift_delta_m = 0.080131（无窗口 0.079403） bilateral_contact = 1.0
  墙钟 67.5 s（无窗口 14.1 s）——**渲染只改墙钟，不改判据**
  display: window_opened=true / frames=1742 / stopped_by=stop_event / error=null
  plant_residency: cycles=16 / failed_cycles=[]
```

两个推论写进纪律：① 窗口**不得**推进仿真（否则成为第二个时间推进者）；
② guest 的"等待 owner 推进超时"在 owner 由驻留线程推进时不会出现 —— 之前把 `guest_timeout_factor`
从 30× 提到 300× 是**治错症**（已回退），真因是 owner 没人推进。

**修法（已落地，属于**产品功能**而不是演示脚本 hack）**：给验收运行器加显示通路 ——
`scripts/scenario.py run --world joint --display interactive_viewer --render-hz N --seconds S`。
依据：runner 已经有 ① 声明驱动的**植物驻留**（`_start_plant_residency`：owner 在 guest 执行期间持续在线，
本场景 `cycles=23`）、② `run_request_live(..., continue_stepping=False)` ——即"只渲染、不推进"的模式。
两者拼起来即可让**真正的验收运行**（stand → dock → pick）边跑边看，且 owner 由驻留线程推进、
guest 的行为完全不变（不需要在演示脚本里自己造时间推进者）。

**放到狗背上（s04）**：`place_object` 能力未交付（`SKIPPED_PENDING`），判据已在
`scenario.yaml: s04_place_in_tray.criteria.pose_tolerance_m = 0.01`；目标帧必须用**世界固定**的
`handoff_station_frame`（托盘帧与机身同一刚体 ⇒ 量到自指量，见 docs/debug/2026-09-24-dock-target-self-frame.md）。

#### 11.9 更正（2026-09-28 晚）：**"渲染只改墙钟、不改判据"是过度断言**

同一命令第三次（带显示）跑出的报告：`exit 5` / `passed=false`，卡在 **s02 dock**：

```
s02 dock  translation_error_max_m = 0.03056883116091061  > 0.03（声明上限）⇒ 判据未满足
s03 pick  SUCCEEDED  grasp_center_distance_m = 0.0002092502492505152（正常）
display   window_opened=true / frames=1937
```

把**四次**运行的 dock 数字并排看：

| 运行 | 是否带显示 | dock 平移误差 (m) | s03 抓取 (m) |
| --- | --- | --- | --- |
| 第一次（方块移前/后） | 否 | 0.028125012624229208 | — / 0.0002098657943494681 |
| 搬运后验收 | 否 | 0.02812501214102655 | 0.0002098657943494681 |
| 演示脚本（自带推进者） | 是（60 Hz，半途失败） | 0.028375962824781182 | —（臂卡住） |
| `run --display` | 是（20 Hz） | **0.03056883116091061（超限）** | 0.0002092502492505152 |

⇒ 两条更正/结论：

1. **不能**说"渲染只改墙钟不改判据"：dock 误差有 **2.4 mm 量级**的运行间波动（0.02812501214102655 → 0.03056883116091061）。
   来源是四足行走由 `iraf_adapters.unitree.mpc.worker`（**独立进程**）做规划，其结果显示墙钟/时序相关的方差；
   渲染（软件 GL，4 核）抬高机器负载后那次恰好跨过 0.03。
2. **s02 的判据目前在边界上**（28.1–30.6 mm vs 上限 30 mm）—— 这是比"演示能不能看"更要紧的验收风险，
   要么给停靠控制器留余量、要么重新论证 `dock_translation_error_max_m` 的取值依据（现为设计 §4 的 30 mm）。
3. 抓取（s03）在**四次**运行里都稳（0.00020920 ~ 0.00020987 m，≤0.005 有 20 倍余量）⇒ 方块移到臂旁边这一步
   是稳的；不稳的是载体定位。
4. 判据纪律：**带显示运行的结论只能引用当次报告**，不能拿另一次的 dock 数字替代（本次就是这么发现的）。

**更正（同日续，见 §11.22）**：上面把差异归到"**guest 控制节拍**"只是**候选**，本轮没做那个 A/B，
而是先用实测把范围压到更窄：绕行航点的手-载荷朝向**几乎没变**（dot 0.996606551 ⇒ 4.721513°），
而**搬运关节速度**是已验证能稳住载荷那一步的 **8.9 倍**（2.352128047 rad/s vs 0.2639 rad/s）
⇒ 区分实验改成"只改声明 `duration_ms`"（place17），见 §11.22。

### 11.23 搬运丢件的**口径级根因**：臂世界与联合世界的物理选项不同（2026-09-28）

#### (1) 关掉"搬运速度"这条线（place17 / place17b）

`duration_ms: 20000`（技能参数，只改声明）⇒ 每段 `phase_ms = 5000 ms`（追踪实测段长 **2500 步**，与声明一致），
关节速度 **0.4704 rad/s**（已验证能稳住的 pick 抬升段是 0.2639 rad/s，只快 1.8 倍）：

```
start   : 指腹 z=0.133531 ｜ 载荷低点 +0.076756 ｜ 双侧接触 True/True ｜ j7 qpos=0.024210 ctrl=0.023
step 560: 指腹 z=0.148607 ｜ 载荷低点 +0.092595 ｜ 双侧接触 True/True ｜ j7 qpos=0.024259
step 620: 指腹 z=0.154499 ｜ 载荷低点 +0.066104 ｜ 双侧接触 True/True ｜ j7 qpos=0.024088
step 640: 指腹 z=0.156917 ｜ 载荷低点 -0.000233 ｜ 双侧接触 False/False ｜ j7 qpos=0.024088 = ctrl
```
⇒ 方块是**在夹口里向下滑出去**（载荷低点 0.092595 → 0.066104，而指腹还在上升），
不是"被甩出去"；把速度放慢 5 倍（峰值加速度 0.067 m/s²、峰值惯性力 0.0027 N）**照样丢**。
⇒ **搬运速度不是主因**。这一条同时**撤回**我在 §11.22 末尾"速度是主因"的判读方向（"朝向保持"那条负结论保留）。

#### (2) 关掉"路径中途把手-载荷朝向转掉"这条线（`build/iraf-a6a14/place_carry_path_probe.py`）

按 `_move_trajectory` 的真实插值（逐关节五次多项式，`phase_ms=5000`，51 点采样）算路径**全程**的夹爪轴：

| 段 | 路径上最大偏角（vs pick 抬升轴） | axis_z 范围 | 指腹 z |
|---|---|---|---|
| lift → transit | **4.7215°**（末端最大） | [-0.5406, -0.4695] | 0.133922 → 0.419302 单调 ✓ |
| transit → above | 34.2951°（单调，末端最大） | [-0.4737, -0.0911] | 0.419302 → 0.419302 |
| above → descend | 34.2951°（起点即最大） | [-0.1558, -0.0911] | 0.419302 → 0.409302 |

⇒ 真正带载荷的那一段（lift→transit）**路径中途也只偏 4.7215°**、指腹 z 单调上升
⇒ **"路径把方块拧出去"不成立**。

#### (3) 观测口径的坑：第 11 个工装缺陷（共享植物下"步"≠"时间"）

我一度把 `step 600 → 620` 之间载荷掉 9.8 cm 读成"20 ms 内下落 9.8 cm ⇒ 物理不可能"，并据此改道。
实测（本轮把 `plant_step_index` 写进追踪行后）：

```
transit 段：arm 控制迭代 2500 次，植物前进 45015 步 ⇒ 平均每迭代 18.01 步
相邻采样 Δplant = 340 ~ 490 步 ⇒ 相邻采样之间是 0.36 ~ 0.40 s 仿真，不是 20 ms
```
⇒ 9.8 cm 下落完全正常，**"物理不可能"是我的读法错了**。
⇒ 纪律（第 11 条）：共享植物下引用"步数"必须换算成**植物步号/仿真时间**；`plant_step_index` 现已在追踪行里。
⇒ 顺带把 §11.21 的"guest 控制节拍 ≠ owner"从推测变成**实测事实**（1 : 18.01），但它**不解释丢件**（慢 5 倍同样丢）。

#### (4) 口径级根因：两个世界的物理选项不同，且**没有任何构建器声明它**

| 世界 | 产物 | `<option>` | 实测 `cone` / `impratio` |
|---|---|---|---|
| 臂 | `build/models/piper-pick-scene.xml` | **无** ⇒ MuJoCo 默认 | `pyramidal` / `1` |
| 联合 | `build/scenes/handoff_lab/handoff_lab_joint.xml` | `<option impratio="100" cone="elliptic"/>` | `elliptic` / `100` |

来源：`vendor/unitree_go2/unitree_robots/go2/go2.xml:4` 的 `<option cone="elliptic" impratio="100" />`
被联合世界**整段继承**（`grep -rn "impratio\|<option" src scripts config scenes profiles` 只命中一个无关探针
⇒ 这个口径是**继承来的隐式默认值**，不是任何声明）。

⇒ §11.20 的"夹持能搬 0.20 m 以上"是在 **pyramidal / impratio=1** 下量的，
而 s04 跑在 **elliptic / impratio=100** 下 ⇒ **这组结论跨世界不可移植**（这次丢件的真正入口）。
⇒ 注意：单看 `cone`，pyramidal 是**内接**近似（对角方向最差 0.707μN）、elliptic 是完整 μN
⇒ 理论上 elliptic 更"抓得住"；嫌疑主要在 `impratio=100`（摩擦约束阻抗 = 法向的 100 倍）。
**不靠推断**：用已有工装（`carry_limit_via_runtime_probe.py`，它本身就是在臂世界里回放 s04 三段）
加 `CARRY_CONE` / `CARRY_IMPRATIO` 两个自变量做四组对照，见 (5)。

### 11.22 丢件机制：**"航点把手-载荷朝向转掉"被实测推翻**；剩下的变量是**搬运关节速度**（2026-09-28）

**假设 → 实测（全部在联合模型 FK 上取数，探针 `build/iraf-a6a14/place_carry_axis_probe.py`
→ `place-carry-axis-probe.json`）**

从**已验证的 pick 抬升位形**（s03 末尾双侧接触成立的那个位形，即 `place_reference.seed_positions`）
到 place 各段的**夹爪轴夹角**与**关节行程**（`piper_joint1..6`）：

| 段 | 轴 vs pick 抬升轴（dot） | 夹角 | Σ|Δq| (rad) |
|---|---|---|---|
| place_transit | **0.996606551** | **4.721513°** | **2.352128047** |
| place_descend  | 0.830365416 | 33.863707° | 4.787259679 |
| place_above    | 0.8261465   | 34.295099° | 5.116397538 |
| place_retreat  | 0.8261465   | 34.295099° | 5.116397538 |

⇒ **"绕行航点把手-载荷相对朝向转掉"不成立**：绕行航点的夹爪轴与已验证的搬运位形只差 **4.721513°**
（`axis(lift) = [-0.594869025, 0.594874526, -0.540606272]`、`axis(transit) = [-0.624317768, 0.624319207, -0.46952407]`，
两轴都在竖直面内朝下）。我上一轮把它当作候选机制，此处**撤回**。

**同一目标点上的多种子扫描（343 个候选，65 个可行）**给出的是一对**互相拉扯**的量：

| 解 | Σ|Δq| (rad) | dot vs 抬升轴 | 轴 z |
|---|---|---|---|
| 当前解（多起点 + 最大 dot 选解） | 2.352128048 | 0.996606551 | −0.469524 |
| 行程最小可行解（`perturb=[-0.4,-0.8,+0.4]`，残差 5.367e-06） | **1.4340** | 0.8509 | **−0.0180** |
| dot 最大可行解（残差 9.931e-06） | 2.487792328 | **0.999983892** | — |

⇒ **"选解改最小行程"不是免费午餐**：行程降到 1.4340 rad 的代价是夹爪轴几乎**水平**（z=−0.0180，
dot 0.8509 = 31.7°）—— 那是"从侧面捅进去"的位形，不是"把方块送进托盘"。
⇒ 所以**不改选解准则**（当前解在"保持朝向下限"这件事上已经是 65 个可行解里最好的一档）。

**剩下唯一与已验证事实冲突的量：搬运时的关节速度**
- 已证明能稳住载荷的动作（s03 的 `grasp → lift`，双侧接触在 8 s 稳定窗后仍成立）：
  Σ|Δq| = |−0.370792207| + |0.039316656| + |0.012055117| ≈ **0.4222 rad / 1.600 s = 0.2639 rad/s**；
- 丢件的 `lift → transit`：**2.352128047 rad / 1.000 s = 2.3521 rad/s**（五次多项式峰值 ≈1.875×均值 ⇒ **~4.41 rad/s**）
  ⇒ **比已验证的搬运快 8.9 倍**。

**为什么这是"可声明"的修法**：搬运时长由技能参数 `duration_ms` 给出（缺省 4000 ms ⇒
每段 `phase_ms = duration_ms // 4 = 1000 ms`，与日志实测 `place_transit_positions@step1000` 一致），
所以"放慢"**只改声明、不改代码**。

**本轮实验（place17）**：`duration_ms: 20000` ⇒ 每段 5000 ms ⇒ **0.4704 rad/s**（接近已验证的 0.2639 rad/s）。
判读规则（先写死再跑，避免事后解释）：
- `after_transit` 双侧接触仍为 true ⇒ **速度是主因**（修法 = 按载荷与位形重构量声明搬运时长，
  并把"搬运关节速度上限"做成构建期自检 `carry_max_joint_rate_rad_s`，超限即拒绝构建）；
- 仍为 false ⇒ 速度不是主因 ⇒ 回到"位形重构本身"（下一步按 `place_above/retreat` 的 5.1164 rad
  拆成多个声明航点，让每段重构图 ≤ 已验证的 0.42 rad 量级），或改夹持（闭爪位需**实测力**定，不几何推断）。

### 11.21 放置段净间隙改为声明值 0.01；采样钩子落地；丢件机制收窄到"联合世界推进节拍"（2026-09-28）

**本轮落地（都保留）**
1. `_move_trajectory(..., sampler=None)`：可选**采样回调**（默认 None ⇒ 行为逐位不变），
   `place_object` 在 `IRAF_DEBUG_PLACE=1` 时逐 20 步把 `phase_trace` 打出来并进证据
   （`segment_samples`）。**这是唯一能在控制路径内观察运动过程的口子**（第 10 个工装缺陷的纪律）。
2. 基线新增 `grasp.place_clearance_m: 0.01`（与 pick 的 `pregrasp_offset_m: 0.04` **分开声明**）：
   放置航点原本用 0.04 ⇒ 指腹到 0.449 m、腕部≈0.50 m、距基座≈0.587 m vs 可达 0.594284 m（贴边）。
   改 0.01 后 `transit` 降到 0.419 m（离边缘更远），IK 残差仍 6e-06。

**联合世界的实测（`nominal --world joint` 带采样）**
```
start        : 指腹 (0.280154,-0.280124,0.133531) ｜ 双侧接触 true/true ｜ 载荷低点 0.076913
transit 段   : step 20→780 一路夹持 ✓（指腹 0.13→0.425、载荷低点 0.074→0.364）
               **但指腹-载荷间隙在缓慢增大：−0.055776 → −0.061317 m**（≈5.5 mm，1.5 s 仿真内）
after_transit: 指腹 (0.280456,-0.280428,0.419180) ✓ 到位 ｜ 双侧接触 **false** ｜ 载荷低点 −0.000216（已掉回台面）
```
⇒ 机制不是"撞到东西"、不是"姿态翻转"、也不是"夹持力不足"（§11.20 已证明能搬 0.20 m），
而是**持续受载下缓慢滑移/挤出**，而每个航点后还有 **8 s 稳定窗**（后端 `settle_ms = duration×4`）
⇒ 在 8 s 内被放光。

**同一搬运在臂世界能稳住（§11.20），联合世界不稳** ⇒ 差异只剩一处：
联合世界里时间由 **owner（四足）驻留线程推进**，臂是 guest，`_advance_for(0)` 被翻译成
"等 owner 推进 1 步"⇒ 臂的**控制更新节拍与单本体不同**（guest 的等待粒度 + owner 的控制循环）。
**下一步（明确）**：在同一世界内做 A/B —— 把联合世界的 `transit` 段按"单本体那种连续控制"重放
（例如让 owner 以更细粒度推进，或把臂的最小控制步与 owner 的一步对齐），量"间隙增长速率"是否消失；
若消失 ⇒ 属于**共享植物的控制节拍口径**问题（要在声明里写清 guest 的控制更新率），
若不变 ⇒ 回到"闭爪位/摩擦"这条线（用小步扫描量接触力与可搬运高度，而不是几何推断）。

**纪律（本轮新增）**：跨世界回放关节解必须做**名字与非几何口径的换算**并注明；
不能把 A 世界的关节解直接塞进 B 世界（我在 v3 工装里犯了一次：联合模型的 `piper_jointN` 解塞进臂世界,
臂力矩位形完全错位、自碰、把方块弹到 z=5 m）。

### 11.20 **夹持被洗清**：真实技能 + 后端控制路径能把载荷搬 0.20 m 以上（2026-09-28）

第 3 版工装 `build/iraf-a6a14/carry_limit_via_runtime_probe.py`：**抓取态来自真实 `pick_object`**
（经 SkillRuntime，`SUCCEEDED`、`grasp_alignment.center_distance_m = 9.387527248483805e-06`），
搬运段用**后端自己的 `_move_trajectory`**（同一套五次多项式与前馈）：

```
抓取态：指腹↔载荷接触 2 ｜ 指腹中点 z=0.134826 ｜ 载荷中心 z=0.114143
抬升 0.05 m：接触 2 ｜ 载荷 z=0.172117（跟着走 ✓）
抬升 0.10 m：接触 2 ｜ 载荷 z=0.275792（跟着走 ✓）
抬升 0.15 m：接触 2 ｜ 载荷 z=0.425925（跟着走 ✓）
抬升 0.20 m：接触 2 ｜ 载荷 z=0.637969（跟着走 ✓）
（第 5 档 0.25 m 时目标超出可达，IK 显式失败——不是丢件）
```

⇒ **纯摩擦夹持可以把载荷搬 0.20 m 以上** ⇒ §11.14–§11.19 里所有"摩擦/夹持力/闭爪位/夹具"的怀疑
**全部可以放下**；搬运丢件的原因在**放置回放路径**里（`transit` 那一段的关节空间运动），
不在夹持。

**第 10 个工装缺陷（我的，必须记）**：为了"逐样本追踪"我直接写 `data.qpos` 再 `mj_step` —— 被位置伺服的
`ctrl` 立刻拉回（追踪里指腹 z 全程 0.1348 不变、轴 z 恒 +1.000，一眼就是没动）。**纪律**：要观察
"运动过程中"的量，必须**在控制路径里采样**（给 `_move_trajectory` 加采样回调，或临时在其中打点），
不能从外面写 qpos 假装在动。

**下一步（明确）**：给后端 `_move_trajectory` 加一个**可选的采样回调**（默认关闭 ⇒ 行为逐位不变），
用它回放 `transit / above / descend` 三段，逐样本量"接触数 / 指腹与载荷相对位姿 / 夹爪轴"，
定位在哪一段、哪一刻丢件，以及丢失时刻腕部转了多少 ⇒ 再决定是**换 transit 解**（要求腕部转角最小）
还是给求解器加**朝向/转角约束**。

#### (5) 第一次口径对照**无效** —— 第 12 个工装缺陷（跨世界回放的不只是名字，还有几何）

我用 `carry_limit_via_runtime_probe.py`（v3）加 `CARRY_CONE`/`CARRY_IMPRATIO` 做四组对照，读数全是垃圾：
载荷被弹到 **z=2.9963 → 4.4447 m**。原因：这个工装当前是在**臂世界**里回放**联合世界解出的**放置四段，
而两个世界的方块位置不同（臂世界 `(0.19, 0, 0.025)`、联合世界 `(0.28, -0.28, 0.025)`）
⇒ 臂按联合解摆过去就撞台面、把方块打飞。**§11.21 的纪律我又犯了一次**（上次是关节名，这次是几何）。
⇒ 该工装作废，四组日志 `build/iraf-a6a14/carry-physics-*.log` **不得引用**。

#### (6) 第二次口径对照**也无效** —— `run` 不重建产物（我的构建器使用错误）

`scripts/scenario.py run` **复用**已存在的 `build/scenes/handoff_lab/handoff_lab_joint.{xml,json}`，
不按 mtime/内容重建。我先改声明再直接跑，产物还停在 **11:58** 的构建（报告里 `world_physics` 为 `null`）
⇒ 那次"pyramidal/1 也丢件"的结论**作废**。
⇒ 纪律：**口径/几何类改动后必须显式重建** ——
`PYTHONPATH=src python3 scripts/build_scene.py --scene scenes/handoff_lab --robot unitree_go2 --attach piper`，
并用报告里的 `world_physics.effective` / `mismatch` **自证生效**后再跑验收。

#### (7) 口径的**实测因果**（有效 A/B：同几何、只换 `<option>`）

重建后（几何逐位不变：`resolved_pad_offset_m 0.028931693`、`transit_local_m [0.240415363, 2.78e-07, 0.419303693]`、
`lift/transit` 关节解与 11:58 那次完全相同），声明 `pyramidal/1`：

| 项 | elliptic/100（原继承） | pyramidal/1（本次声明） |
|---|---|---|
| 联合 XML `<option>` | `impratio="100" cone="elliptic"` | `impratio="1" cone="pyramidal"` |
| 报告 `world_physics` | — | `declared/effective` 一致、`mismatch={}` ✓ |
| s01 末速 | 1.4887881120651461e-05 m/s | **8.102061962044204e-10 m/s** |
| s02 停靠误差 | 0.028569272841750617 m | **0.026639029421025973 m** |
| s03 pick | SUCCEEDED（4 次运行一致） | **FAILED** |

⇒ 物理口径**有真实因果**（改它就会改结论），"pyramidal 更差"与理论一致（内接锥对角方向最差 0.707μN）；
⇒ 但**两种口径下 s04 绕行段都丢件** ⇒ 口径差**不是** s04 丢件的原因，只是**跨世界引用结论的前提**。
⇒ 已落地：口径从"隐式继承厂商模型"改成**声明 + 读回 + 不一致即拒绝构建**
（`scenes/handoff_lab/scene.yaml: world_physics` → `config/scene.schema.json` →
`scene_builder._apply_world_physics` → 报告 `world_physics`），并按实测**冻结为 `elliptic/100`**
（= 今天通过 s01–s03 的那一组）。单本体世界（piper 自己世界无 `<option>` ⇒ pyramidal/1）的差异是**已登记的债**。

#### (8) 新一条实测**否掉"摩擦不足"**（决定性数字）

place 入口实测 `piper_joint7: qpos=0.024175 / ctrl=0.023` ⇒ 指腹被载荷**压开 1.175 mm**；
指腹 actuator `gainprm=10000` ⇒ 夹持力 ≈ **11.75 N/指**，指腹摩擦已注入 `[2.0, 0.05, 0.001]`
⇒ 摩擦容量 ≳ 47 N，而载荷重 **0.39 N**（0.04 kg）⇒ **载荷不可能因摩擦不足下滑**。
但追踪显示 `payload_low_minus_pad_m` 从 −0.0558 单调增大（载荷相对指腹持续下移）直到脱手。
⇒ 现有证据只证明"**有接触**"，没有证明"**接触在哪、法向是什么、力多大**"。
⇒ 下一步（明确、唯一）：追踪**接触对的 geom 身份 + 法向 + 接触力**与载荷的**6 维位姿**
（当前 `_snapshot` 只给"接触数>0"和最低点 z，粒度不够）。

#### (9) 接触对粒度证据（place21，**有效**）：不是滑脱，是**被撞飞**

给 `place_object` 的追踪行加"接触对身份 + 法向 + 接触力 + 载荷 6 维位姿 + 指腹间距"后（本轮改动）：

```
start : 接触对 piper_left_finger(body=piper_link7)  法向力 13.155181 N ｜ dist=-0.009265
        ｜  piper_right_finger(piper_link8)          法向力 13.045105 N ｜ dist=-0.009452
        ｜ 载荷 pos [0.27598, -0.275586, 0.104368]   quat(wxyz) [0.992769195, 0.0061069, -0.006712948, -0.119695313]
        ｜ 指腹间距 pad_span = 0.068785 m
step20: 11.552540 N / 11.446758 N ｜ pad_span 0.070910（段首被载荷压开 2.1 mm）
step80: 10.679524 N / 10.699729 N（单调下降 ~19%）
失手后: 接触对变为 workbench（4 个接触，各 0.0981 N）｜ quat [0.03248855, -0.706360031, 0.03248855, 0.706360031]（≈ 翻转 176°）
        ｜ pad_span 回落到 0.06636 = 空夹口闭合值 0.066364 ✓
```
⇒ 接触对**身份正确**（就是两个指腹）、法向力 **10.7~13.2 N/指**，指腹摩擦已注入 `[2.0, 0.05, 0.001]`
⇒ 摩擦容量 ≳ 46 N，而载荷仅 **0.39 N**（0.04 kg）⇒ **绝不可能是摩擦滑脱**；
⇒ 失手后载荷在台面上呈 **~176° 翻转** ⇒ 更像**被撞飞**。
⇒ 另一个反证"逐渐滑移"的量：2000 步/段这次丢手在 **step 720~860（36%~43%）**，
而 2500 步/段那次在 **step 600~640（24%~26%）** ⇒ 丢手点**不是固定的段长比例**，
更像某个**几何/动力学事件**（而非单调滑移）。

#### (10) 干涉自检的第一次尝试**无效** —— 第 13 个工装缺陷

`build/iraf-a6a14/place_carry_interference_probe.py` 逐点 FK 查"载荷 ↔ 任何其它 geom"的接触，
结果 **402/402 全部与 `workbench` 接触、dist=0.0** ⇒ 无效。原因：探针里只写了**臂关节** qpos，
**没有把载荷一起"带着走"** —— 方块的 freejoint 停在模型初始位姿（躺在台面上）⇒ 每个采样点量到的都是台面接触。
⇒ 纪律：**运动学探针里"被夹持的载荷"必须随手一起搬运**（先用起始位姿算出 `T_hand→box`，
再按各采样点的 FK 反推载荷位姿），否则读数与真机无关。
⇒ 下一步（明确）：按此修好探针后，查 lift→transit 全程是否存在"载荷 ↔ **臂自身连杆**（link5/link6 等）"
的穿透 —— 这是本轮之后**唯一**的候选机制（§11.17 的自检只覆盖航点 + 载体子树，漏了路径中途与臂自身）。

#### (11) 干涉扫描 v2（**有效**）：丢件段零干涉；但下一段把方块压进臂自己的 `base_link` 2.286 cm

修好 v1 的缺陷（载荷按 `T_hand→box` 随身搬运）后，`place_carry_interference_probe2.py` 结果：

```
lift_to_transit : 干涉采样   0/202  ｜ 对手 body: []            ｜ 占比 None
transit_to_above: 干涉采样  27/202  ｜ 对手 body: ['base_link'] ｜ 占比 0.611940~0.741294
                  最深 {"partner_geom": "#9", "partner_body": "base_link", "dist_m": -0.022856}
```
（`T_hand→box` 由"抬升位形 + 运行时实测载荷位姿"算出：`rel_pos` / `rel_quat_wxyz` 见
`build/iraf-a6a14/place-carry-interference-probe2.json`；指腹接触不算干涉。）

⇒ **① 丢件段的干涉假设被否掉**：真正带载荷的 `lift→transit` 全程 **0/202** 与任何非指腹几何接触
（连擦碰都没有）⇒ "路径中途撞到臂自身连杆"**不是**丢件原因（这次是有效证否）。
⇒ **② 但发现一个独立的真实缺陷**：`transit→above` 段有 **27/202** 个采样把载荷**压进臂自己的
`base_link`（geom `#9`）最深 2.286 cm**，位置在该段的 61.2%~74.1%。
即"绕行航点（抓取点正上方、托盘高度）→ 承载面正上方"这条腿**在笛卡尔空间穿过臂的底座**
⇒ 即使夹持不丢，放置也做不成（§11.17 的自检只覆盖**航点**与**载体子树**，所以没抓到它）。
⇒ 修法方向（与 §11.17 同款、声明驱动）：把这条腿拆成**多个声明航点**（或换一个不穿底座的绕行位形），
并把本探针升格为**构建期自检**：路径全程 × 全部连杆，有穿透即拒绝构建。

**丢件仍未被解释**（现有否证清单）：不是摩擦（容量 ≳46 N vs 载荷 0.39 N）、不是朝向（路径最大偏 4.7215°）、
不是速度（慢 5 倍同样丢）、不是夹持力/equality、不是物理口径（两种口径都丢）、不是臂自身干涉（本小节）。
⇒ 下一步的取证方向：**逐样本**（而非每 20 次迭代）记录"指腹 ctrl / qpos / 接触对 / 载荷位姿"，
定位那一瞬间**谁先动**（是 ctrl 变了、还是载荷先掉）—— 目前 20 次迭代 ≈ 0.36 s 仿真的采样间隔
正是丢件事件的时间尺度，粒度仍然不够。

#### (12) 逐样本追踪**定案**：方块是被**手指自己的闭合指令**挤出去的（不是摩擦、不是撞击）

`IRAF_DEBUG_PLACE_STRIDE=1`（逐样本，紧凑行）跑 place22，丢手瞬间全过程（`piper_joint7`）：

```
step585: pad_z 0.316079 ｜ low-pad -0.056881 ｜ j7 qpos 0.0236123 / ctrl 0.0234186 ｜ 接触 True/True ｜ 力 2.765568 / 2.686331 N
step590: pad_z 0.318613 ｜ low-pad -0.056315 ｜ j7 qpos 0.0235997 / ctrl 0.0234079 ｜ True/True ｜ 力 2.706557 / 2.655568 N
step596: pad_z 0.322405 ｜ low-pad -0.056255 ｜ j7 qpos 0.0235741 / ctrl 0.0233951 ｜ True/True ｜ 力 1.790275 / 1.785376 N
step597: pad_z 0.322823 ｜ low-pad **-0.072273**（载荷一次下移 1.56 cm）｜ 力 1.813752 / 1.808016 N ｜ 仍 True/True
step598: pad_z 0.322996 ｜ low-pad **-0.085579**（再下移 1.34 cm）｜ 力 1.784538 / 1.778722 N ｜ 仍 True/True
step599: pad_z 0.323851 ｜ j7 **qpos == ctrl == 0.0233913**（夹口空闭）｜ 接触 **False/False** ｜ 力 []
step602: 落台途中撞到 `RR_calf`（狗的小腿）｜ step604: 落到 `world`（台面）
```

**机制**：指腹法向力**整段单调衰减**（≈12 N → 2.6 N），而 `qpos` 始终只比 `ctrl` 高约 0.2 mm
（压缩量在**松弛**）；`place_*_positions` 里 joint7/8 = `0.023`（= 空夹口闭合指令），后端把夹爪关节
**当作轨迹一起插值**（从当前 `qpos` 0.024174 → 0.023），于是**整段都在合拢** ⇒ 方块被沿夹口轴向
**挤/楔出去**（棘轮式：合拢→下移→接触点上移→继续合拢→继续下移），直到脱离，夹口随即空闭。
⇒ 这条一次解释全部否证结果：
  · `pick` 的 `lift` 能稳住 —— 那一段夹爪 `ctrl` **静止**（0.023 不变）⇒ 没有合拢即没有挤；
  · 慢 5 倍没用 —— 合拢的**总量**与速度无关；
  · 朝向保持（4.7215°）、两种物理口径都丢、无任何几何干涉（0/202）—— 都与"挤"自洽。

**修法（声明驱动，最小改动）**：搬运段必须让夹爪"**保持**"，不得把它当轨迹插值。
设计：由构建期在该场景报告里声明搬运的夹爪语义（如 `gripper.carry: hold`），后端在回放搬运段时把
夹爪关节的**目标 = 当前实测 `qpos`**（零位移 ⇒ 零合拢），并在证据里留痕"目标是保持值而不是 0.023"。
**不得**在实现层硬写这个语义（缺声明即显式失败），否则又变成隐式默认值。

#### (13) 夹爪语义声明化落地 + **两条假设被否**（含我自己的设计错误）；外力取件的线索

**已落地（声明驱动、缺声明即 fail-closed）**
- 基线 `config/piper_simulation_baseline.yaml: grasp.carry_gripper`（`hold` / `trajectory`）；
- 构建期 `_joint_place_resolution` 读声明并写进联合报告 `gripper.carry_gripper`（带出处）；
- 后端 `place_object` 读 `gripper.carry_gripper.mode`，**缺声明即显式失败**（不给实现层默认值），
  并在证据里留痕 `gripper_carry.segment_targets`（实际下发的保持值）。

**place24（`hold` = 目标取当前 qpos）⇒ 失败，且暴露我的设计错误**
`after_transit`：`j7 qpos == ctrl == 0.024215`、载荷已落地（low=-0.000216）。
⇒ 位置伺服的力 ∝ (target − qpos)：**target = qpos ⇒ 夹持力为 0** ⇒ 载荷滑落。
"零合拢"不等于"零夹持变化"：把目标设成实测位置等于**松开夹爪**。

**place25（`hold` = 目标取当前 ctrl，即保持夹紧力、指腹零位移）⇒ 仍然失败**
`start`：ctrl 0.023000 / qpos 0.024187（压缩 1.187 mm ⇒ ≈11.9 N）；`after_transit`：`qpos == ctrl == 0.023000`
⇒ 方块**在恒定 11.9 N 夹紧指令下、指腹零位移的情况里照样脱离**。
⇒ **"夹爪合拢把载荷楔出去"这条假设被否掉**（§11.23(12) 的结论收回）。
⇒ 同时可以确定：**不是夹持力/摩擦**（11.9 N × μ2.0 ≈ 47 N vs 载荷 0.39 N，差两个数量级）。

**新线索（下一轮的唯一入口）**：逐样本追踪里方块脱离后先撞到 **`RR_calf`（狗的小腿）**、再落到台面。
s04 是**臂（guest）的步骤**，而**狗（owner）正被植物驻留线程持续驱动**（`stand` 一直在跑）
⇒ 狗身/腿/托盘**在 s04 全程都在动**，随时可能扫到臂旁（0.32 m 外）的方块。
候选机制：**被载体（狗/托盘）撞掉** —— 与全部否证结果自洽（口径无关、速度无关、臂自身无干涉、
夹持力充足、丢手点不固定）。
⇒ 下一步实验（便宜、可定案）：在 s04 期间量**载荷与狗各 geom 的最小距离**并留痕；
或对照跑一次"**s04 期间暂停驻留**（狗静止）"—— 若方块不掉 ⇒ 载体撞击成立。

#### (14) 邻居证据：载荷就在**狗腿正上方 7~10 cm**，落下砸在 `RR_calf` 上 —— 布置缺陷 + 持续外扰源

新增"谁在载荷附近"证据（逐 geom 与载荷中心的距离，本体名一起留痕）后，place26（逐样本）实测：

```
step560: 最近几何 piper_link8/piper_right_finger 0.076345 ｜ piper_link7/left 0.079912
         ｜ **RR_thigh #49 0.088374 / RR_hip #48 0.092932**
step567: 指腹 0.067993 / 0.071880 ｜ **RR_hip #48 0.101861 / RR_thigh #49 0.102280** ｜ 接触 True/True
step568: 接触 **False/False**（载荷开始下滑：low-pad -0.092685 → -0.104559）
step570: 最近几何**全是 RR_calf**（0.081878 / 0.093948 / 0.115165）⇒ 载荷砸在狗的**右小腿**上
全程"非 piper"邻居最近距离：RR_calf ≈ 0.073 ~ 0.148 m（整段都在 10 cm 量级）
```

⇒ **布置缺陷（独立于丢件，必须修）**：交接站位让狗中心距臂的抓取点仅 **0.32 m**，而 Go2 的腿
向外伸 ≈0.3 m ⇒ **狗的腿就在臂的抓取点旁边 7~10 cm**，托盘还在臂工作空间上方。
⇒ 同时它是**持续外扰源**：驻留线程在 s04 全程让狗执行 `stand`（`config/go2_joint.yaml: hold.skill: stand`），
而 `stand` 是**原地踏步（含重心转移）**⇒ 狗腿/机身一直在动，且踏步会漂移 ⇒ 与"载荷逐渐下滑后突然脱离"
（并非固定段长比例）一致。

**下一步（两个都要，顺序在先者更省）**
1. **把狗抬到远处做对照**（只改声明：交接站位/停靠目标 y 方向拉开，或临时用"狗静止"的驻留技能）——
   若载荷不掉 ⇒ 载体干扰成立，随后按"站位 + 驻留技能"两条声明修；若仍掉 ⇒ 回到载荷侧机制。
2. 修布置：工作空间（抓取点 → 托盘）与**载体自身几何**必须留出可声明的净空
   （把"载体子树 ↔ 载荷/臂工作空间"的最小距离做成构建期自检，与本轮 §11.23(11) 的干涉探针合并）。

#### (15) 载体（狗）撞击**被已有数据否掉**：丢件前接触对只有两个指腹

用 place22 的逐样本数据（`partner_bodies` 就是**接触对**，不是邻居）：

```
step585~598（丢件前）：partners = ['piper_link7', 'piper_link8']   ← 只有两个指腹
step599             ：接触 False/False（离开夹口）
step602             ：partners = ['RR_calf']                      ← 落下之后才碰到狗腿
```
⇒ **狗腿在丢件前与载荷零接触** ⇒ "被载体撞掉"**不成立**；`RR_calf` 只是"接住"了掉下来的方块。
⇒ §11.23(14) 的布置问题（腿距抓取点仅 7~10 cm）仍然要修，但它是**独立缺陷**，不是丢件原因。

**同时否掉**："静止保持"这条隔离实验**做不了**：Go2 的 `stop` 默认 `torque_zero_release`
（力矩型执行器松力即失能 ⇒ 狗会塌），而 `damped_hold` 在
`src/iraf_adapters/unitree/unitree_go2.py:429` 明确标注**尚未实现**、不得进 `SUPPORTED_STOP_MODES`
⇒ 想让狗"站着不动"必须先**交付 damped_hold 能力**（那是另一个工作项，且需重跑 loopback 验收）。

**至此否证清单（全部带实测数字）**：摩擦/夹持力（11.9 N ⇒ ≳47 N 容量 vs 0.39 N 载荷）、
朝向（4.7215°）、速度（慢 5 倍仍丢）、equality、物理口径（两种都丢）、
臂自身几何干涉（带载荷段 0/202）、夹爪合拢（place25 恒定 ctrl 仍丢）、载体撞击（本小节，零接触）。
**唯一还站着的异常量**：`start` 行指腹与载荷的**接触穿透深度 `dist = -0.009265 / -0.009452 m`**
（≈9.3/9.5 mm）远大于"方块 0.05 m 与闭合指腹内表面间距 0.04046 m"推出的 4.77 mm/侧
⇒ **指腹的碰撞几何不是我以为的那对平行面**（厂商 2F-85 的硅胶pad在 MJCF 里被建成 box，
碰撞面/尺寸口径存疑）。**下一步唯一动作**：查清 `piper_left_finger` / `piper_right_finger`
这两个 geom 的真实尺寸与朝向，并按"方块宽度 vs 指腹内表面间距"重算**闭爪位**该取多少
（这个量此前两次都被我用**几何推断**做错过，所以这次必须用**接触实测**标定：扫 qpos 看
`dist` 何时从负变零，取"刚好贴合"的那一档，再把该档写进声明）。

#### (16) 离线**状态重建**探针：夹爪是斜夹"棱"，而且我的几何算法是错的

新探针 `build/iraf-a6a14/gripper_contact_axis_probe.py`：按"已验收的抬升位形 + 运行期实测载荷位姿"
重建丢件前那一刻，用**实测接触与模型事实**对账（不做几何推断）。

**重建可信度**：接触读数 `dist = -0.009293 / -0.009457 m`、法向 `[0.67834, 0.734534, 0.017762]`
对运行期 `-0.009265 / -0.009452 m`、`[0.678433, 0.734453, 0.01753]` ⇒ 复现到 3e-5 m 量级
⇒ **离线探针可替代跑次来迭代**（这是本轮最省时间的工具）。

**事实 1（推翻我自己的算法）**：指腹两个 geom 的类型是 **`mjGEOM_MESH`**（真实的 2F-85 手指 STL），
`size = [0.01295074, 0.029164512, 0.052450376]` 只是 mesh 的 **AABB 半长**
⇒ 我此前"内表面间距 = 中心距 − 2×0.01295"的算式**无效**（§11.23(15) 里 4.77 mm 的那套数字作废）。

**事实 2**：闭合轴（两个指腹中心连线）与**方块三本征轴**的夹角 =
**58.357073° / 31.697082° / 88.334535°** ⇒ **斜着夹方块的"棱"**（不是夹面）；
方块沿闭合轴的支撑函数（= 有效半宽）= **0.035113138 m**（夹面时应为 0.025）。
⇒ 斜法向在**夹口轴向**上有分量 ⇒ 搬运中持续把载荷沿夹口轴向楔出去（§11.23(12) 收回的那条，
现在有了正确的几何解释）。

**对照实验（只改声明、不动臂）：把方块绕 z 转 45°** ⇒ 夹角变为
**0.759491° / 89.589236° / 89.361185°**（≈ 垂直夹面 ✓），有效半宽降到 **0.025455762 m**，
重建穿透 `dist` 从 **-0.009293 → -0.001092 m**（8.5 倍改善）✓
⇒ 但**面抓下 s03 报 `grasped=false`**（`grasped = force_ok and lifted`）：
面抓必须连 **pick 的抓取/抬升时序**一起重设计（现有时序是为斜抓调的）。

#### (17) 闭爪位**接触标定**探针（并把第一版的工装缺陷记下来）

`build/iraf-a6a14/gripper_clamp_calibration_probe.py`：把载荷**钉在抓取位姿**、夹爪从张开位
闭合到候选指令、跑 3 s 仿真，量两个指腹关节的稳态误差（力 = `gainprm 10000` × 误差）与接触力。

⚠ **第一版缺陷（第 14 个）**：没有钉住载荷 ⇒ 合爪前载荷先自由落体到台面 ⇒ 全程零接触、误差 0，
读数与真机无关（"闭爪位 0.023 ⇒ 力 0.000 N"是假读数）。

修好后实测（双侧接触全程 True）：

| 闭爪指令 | j7 力 | j8 力 |
|---|---|---|
| 0.023（原声明值） | **2.310 N** | **2.263 N** |
| 0.021 | 4.596 N | 4.549 N |
| 0.019 | 6.883 N | 6.834 N |
| 0.017 | 9.169 N | 9.120 N |
| **0.015** | **11.455 N** | **11.405 N** |
| 0.012 | 14.885 N | 14.832 N |
| 0.010 | 17.170 N | 17.117 N |

⇒ **面抓下 0.023 只给 2.3 N**（s03 因此失败）；11.9 N 那一档（对标"斜抓时代能抬起方块"的量级）
对应 **0.015** ⇒ 关系近似线性（0.001 指令 ≈ 1 mm 行程 ≈ 1.14 N）。
⇒ **斜抓为什么"能用"**：方块**对角**（0.0702 m）比该指令下的夹口宽 ⇒ **意外过盈**约 4 mm
⇒ 那 4 mm 过盈正是把方块沿夹口轴向楔出去的元凶 ⇒ **s04 丢件的机制链到此闭合**。

**决策（未定案，交给布置层面）**：修法是**面抓**，但面抓有两条路——
① 转**方块** 45°（离线已验证角度与穿透改善）⇒ 必须连 pick 的抓取/抬升时序一起重设计；
② 转**臂的摆放偏航** ±45°（方块不动）⇒ 会改变放置路径的方位（joint1 行程、托盘可达性、
   以及 §11.23(11) 已量到的"transit→above 把载荷压进 base_link 2.286 cm"需要一并复核）。
未定案前仓库保持 s01–s03 的已验证状态：方块轴对齐、闭爪位 0.023。

#### (18) pick 相位→夹爪映射**声明化**（行为逐位不变）+ 面抓路线的**两条实测阻塞**

**落地（声明驱动、缺声明即失败）**：基线新增 `gripper.phases: {home|approach|grasp|lift} → open|closed}`，
构建器 `scripts/build_piper_pick_scene.py` 按声明取夹爪值 —— 此前这里是**硬编码**
（home/approach/grasp 写死 open、lift 写死 closed，即"**边升边合**"）。
声明化后 `build/models/piper-pick-scene.json` 与改前 **`diff` 为空**（逐位一致）⇒ 只改了表达方式，没改行为。

**面抓实验（方块转 45° + 闭爪位 0.015 + `phases.grasp=closed`）实测两条阻塞**：

| 组合 | 结果 |
|---|---|
| `grasp: open` + `lift: closed`（现行"边升边合"） | `force_ok=true`（11.403717/11.443127 N）但 **`lifted=false`**、`lift_delta_m=1.1e-05`（限 0.02）⇒ **夹得住、抬不起来** |
| `grasp: closed` + `lift: closed`（"先合爪后抬升"） | s03 判 **`末端未到达目标抓取位姿: distance=0.005838m tolerance=0.005000m`**，delta=`[-0.0022940698682458183, 0.002293503015008913, 0.004854280700436732]`，qpos=`{joint1: -0.00010528486580859289, joint2: 1.471225508291635, joint3: -0.28748893179089186, joint4: 1.35157530309704e-06, joint5: -0.19876413170215182, joint6: 9.483576939775031e-06}` |

⇒ 第二条的含义：**在抓取位姿上合爪时，夹爪对载荷的反作用力把臂推离目标 5.838 mm**（超过声明容差 5 mm）。
⇒ 两条阻塞指向**同一个语义问题**：面抓需要"**下压到位 → 静止合爪 → 保持 → 抬升**"四拍，
而"**是否到位**"应当评在**合爪之前**（否则必然被合爪反作用力顶出容差）。
这属于**验收语义**（容差评在哪个时刻、是否允许合爪反作用力计入）⇒ **跨边界决策，本轮未改**。

**另记：路线②（转臂摆放偏航 ±45°）被已声明门禁挡下**（试 76.642927° 时）：
构建失败于 `重力前馈静态保持 4000ms 后最大关节误差 0.019497906 rad（限 0.001）`（残余 joint3 −0.019497906），
且闭合轴只从 58.357073° 转到 13.782443° ⇒ 换偏航**不是刚体旋转**（求解器落了别的分支）。

#### (19) 面抓"抬不起来"的真因：**合爪把载荷推离夹口轴线**（离线搬运测试判定）

`build/iraf-a6a14/offline_carry_test_probe.py`：同一联合模型、同一控制路径
（摆到已验收抓取位形 → 夹爪从张开位静止闭合到声明闭爪位 → **解除钉住** → 走后端同一条
`_move_trajectory` 上抬轨迹），一次跑出对比：

```
面抓 (载荷 yaw 45°): 合爪后 z=0.024784 力={world 0.07, geom67 15.82}
                     抬升后 z=0.066976 力={67 4.78, 68 4.71} ⇒ 抬升量 +0.042192 m ⇒ 跟着走 ✓
斜抓 (载荷 yaw  0°): 抬升量 -0.000049 m ⇒ 没跟着 ✗   ← **本对照组无效**
```
⚠ 斜抓那一栏我用了**面抓跑出来的被推移后的载荷中心**（见下），所以 pads 是错位的 ⇒ 该栏读数不作数
（第 16 个我自己的工装缺陷：两组对照必须各用**自己那一次**的抓取时刻载荷位姿）。

**有效结论**：**面抓本身能搬（+4.22 cm）**，前提是**合爪期间载荷不被推走**（离线里我把载荷钉住才成立）。
而运行期面抓实测：载荷中心从 `(0.28, -0.28)` 被合爪**推到了
`(0.2844182148781458, -0.26564874240591296)`**（y 向 **1.44 cm**）⇒ 之后的**竖直抬升**就丢了它
⇒ `lifted=false` 的真因不是夹持力、也不是"面抓天生不行"，而是
**合爪时夹爪在接触之后还要走完 2 cm 行程（open 0.035 → closed 0.015），任何不对称都累积成"推移载荷"**。

**下一步（有界、可声明）**：让合爪"**不推载荷**"——
① 把 `close` 的时长/增益声明化并放慢（接触后行程的动量更小）；或
② 让夹爪**在接触前就接近载荷宽度**（例如下压段把夹爪预合到接近载荷宽度，接触后只走很短一段——
   注意：(18) 里"下压段就合到底"会把臂顶出容差 5.838 mm，所以是**预合到接近但不夹住**，不是合到底）；或
③ 合爪期间对载荷加一条**声明化的临时约束**（夹爪闭合过程把载荷保持在抓取点上）。
三条都属"声明 + 证据"，选哪条由使用者定；②与③的组合最贴近真机（真机的合爪推力由夹爪的平行度与
柔顺指腹吸收）。

#### (20) 接触起始的实测 + 路线②（"预合"）**被否**：推移来自接触后的行程，而那段行程正是夹持力来源

用标定探针（载荷钉住、从张开位往下细扫）实测**接触起始**（第一档出现非零接触）：

| 载荷朝向 | 接触起始 | 11.4 N 对应的闭爪位 | 接触后仍需行程 |
|---|---|---|---|
| 斜抓（轴对齐） | **0.035 ~ 0.033 之间**（0.035 无接触、0.033 已 1.940 N） | 0.023（旧值） | ≈ 1.05 cm/侧 |
| 面抓（绕 z 45°） | **≈ 0.0235**（0.035~0.027 全程零接触；0.023 给 2.310 N） | **0.015** | **≈ 8.5 mm/侧** |

⇒ 面抓的方块**面更窄** ⇒ 夹爪要合得更深才碰到（起始从 0.034 降到 0.0235），
而 11.4 N 仍在 0.015 ⇒ **接触后仍要走 8.5 mm**。
⇒ **路线②（下压段"预合到接近载荷宽度"）无效**：预合只能省掉"接触前的空行程"（那一段本来就不推任何东西），
推移/楔出发生在**接触之后**，而恰恰是"接触后的 8.5 mm"提供了夹持力（力 = `gainprm 10000` × 稳态误差）。
⇒ 离线搬运测试（§11.23(19)）之所以面抓能搬 +0.042192 m，是因为我把载荷**钉住**了（= 路线③的效果）
⇒ **③（合爪期间把载荷保持在抓取点，闭合完成即释放）是唯一与实测自洽的修法**；
它的物理对应就是真机的"指腹柔顺 + 平行颚吸收推力"，而本模型的指腹是**刚性 mesh**、桌子摩擦只有 ~0.4 N，
所以 1 N 量级的侧向合力（两条接触法向只差 4.7°，`dot=-0.9966`）就足以把 0.39 N 的方块推走。

#### (21) 路线③落地（`close_hold: pin_payload`）+ **新发现：推移发生在合爪之前**

**落地**（声明驱动、语义显式进证据）：
- 基线 `gripper.close_hold ∈ {pin_payload, none}`；构建器写进报告；
- `_joint_manipulation` 新增 `SEMANTIC_GRIPPER_KEYS = (close_hold, carry_gripper)` —— 这两个键是**语义开关不是对象名**，
  不参与前缀改写（此前会被当成名字去联合模型里查、必然 fail-closed）；
- 后端 `pick_object` 的 `GRIP_CLOSE` 阶段按声明调用 `_advance_pinned(close_ms, payload)`：
  **逐步**把载荷 `qpos`/`qvel` 复位到合爪开始时的位姿（owner 走 `step_once`、guest 等 owner 推进 1 步，
  共享植物契约不变），合爪结束即释放；搬运仍**只靠真实摩擦**，证据里留 `gripper_close_hold`。

**③ 的验证（面抓 + 闭爪位 0.015 + pin）**：仍然 `lifted=false`、`lift_delta_m = 1.2e-05`，
且载荷中心位移**与加 pin 之前同量级**（`(0.2843960490756126, -0.2655149101371577)` vs 之前的
`(0.2844182148781458, -0.26564874240591296)`）⇒ **pin 无效**。
⇒ 结论（新）：**载荷是在"合爪之前"就被推走的** —— 即 **APPROACH/DESCEND 段**（刚性指腹下压时碰到方块
上角并把 0.39 N 的方块推开 1.44 cm）。`close_hold` 只覆盖合爪段，所以补不到这一段。
⇒ 下一步（有界）：把同一"保持"语义声明化地扩到**接近/下压段**（`approach_hold`），
或让接近段带**可声明的间隙**使指腹在下压时完全不碰载荷（碰不到就不会推）。
注意对齐门禁是在 DESCEND 之后评的，它看的是**指腹 vs 目标点**，因此下压把载荷推歪**不会**被它发现
（本次就是这样：门禁通过、载荷却已不在夹口轴线上）。

#### (22) "保持"扩到四段（接近/下压/张开/合爪）+ 闭爪位扫描：离线全绿、运行时仍 `lifted=false`

**落地**：
- `_move_trajectory(..., pin_body=None)`：可选参数，**逐步**把该 body 的 freejoint 复位到本段起始位姿
  （owner 走 `step_once`、guest 等 owner 推进 1 步）；声明 `gripper.approach_hold ∈ {pin_payload, none}`
  控制 APPROACH/DESCEND 两段；`GRIP_OPEN`（DESCEND 之后重新张到 0.035）也纳入同一保持
  —— 运行期实测这一段单独就能把载荷推开 ~2 cm（此前只在 DESCEND 之后设 pin 是漏了它）。
- `SEMANTIC_GRIPPER_KEYS = (close_hold, approach_hold, carry_gripper)`：语义开关不参与前缀改写。

**闭爪位扫描（离线、真时序：接近+下压+合爪全程保持 → 释放 → 抬升）**
`build/iraf-a6a14/face_grip_closure_sweep_probe.py`（面抓，方块绕 z 45°）：

| 闭爪位 | 释放后抬升量 | 结果 |
|---|---|---|
| 0.023 | +0.043281 m | ✓ |
| 0.022 | +0.040605 m | ✓ |
| 0.0215 | +0.039937 m | ✓ |
| 0.021 | +0.041235 m | ✓ |
| 0.0205 | +0.041124 m | ✓ |
| **0.020** | **+0.040603 m** | ✓ |
| 0.019 | +0.039385 m | ✓ |
| 0.018 | +0.038228 m | ✓ |
| **0.015** | **-0.000038 m** | ✗ 被弹出夹口 |

⇒ **面抓不需要 11.4 N**：闭爪位越小力越大，而**过大的法向力恰恰制造把载荷推走的侧向合力**
（两条接触法向只差 4.7°，`dot=-0.9966`）⇒ 我先前那套"按 11.9 N 对标取 0.015"的标定方向是错的。

**但运行时在同档（0.020）仍失败**：`force_ok=true`（5.690518/5.736696 N）、`bilateral_contact=true`、
`lifted=false`、`lift_delta_m=5e-06`，载荷被留在台面（停在 `(0.26407682334213434, -0.2904213392268088)`）。
⇒ **离线（能托住 +4.06 cm）与运行时（托不住）在同一闭爪位下结论相反** ⇒ 二者之间还差一个因素。
本轮已排除：闭爪位、夹持力、接触对、朝向、"保持"是否生效（四段都有）。
**下一轮的入口（有界）**：把**运行期的相位时序**（HOME→APPROACH→DESCEND→GATE→GRIP_OPEN→GRIP_CLOSE→LIFT，
含各段 `phase_ms` 与前馈偏移 `_pick_ctrl_offsets`）**逐段照搬进离线探针**，直到离线复现运行时失败
—— 那个能让离线"由绿转红"的差异就是根因。

**⚠ 工具坑（已修复，记下来）**：本轮我用 `patch` 改 `_move_trajectory` 的签名时，`old_string` 里把形参
写成了 `commands`（真名是 `target_positions`）——补丁工具**模糊匹配成功**、把签名改了，函数体仍在用
`target_positions` ⇒ `NameError`，pick 立即失败。**教训**：改签名类的补丁必须复核 `git diff` 的
`-`/`+` 两侧、并跑一次语法/最小路径检查，不能只看"success"。

#### (23) 逐相位回放探针：**结果与运行时不一致 ⇒ 探针有缺陷（第 17 个），读数不可引用**

`build/iraf-a6a14/offline_pick_sequence_probe.py` 照搬运行期相位表
（HOME_HOLD→APPROACH→DESCEND→GRIP_OPEN→GRIP_CLOSE→LIFT，含 `phase_ms/open_ms/close_ms/lift_ms`
与逐段前馈偏移；保持则每步复位载荷 `qpos/qvel`），跑两栏：

```
保持(声明): start..GRIP_CLOSE 载荷恒在 [0.28,-0.28,0.025]（被钉住）；GRIP_CLOSE 力 13.549/13.539 N
           ⇒ LIFT 之后载荷 z=0.024784、停在 [0.309295,-0.310954] ⇒ **没跟着 ✗**
不保持    : start 后载荷落到 z=0.024784（自由落体 2.16e-04）；GRIP_CLOSE 力 13.575/13.441 N
           ⇒ LIFT 之后载荷 z=0.085903 ⇒ **抬升 +0.060903 m ✓**
```

⚠ **但这一栏与运行时矛盾**：运行时在**同一份带保持的声明**下 s03 是**绿的**
⇒ 探针与运行时不一致 ⇒ **本探针读数不可引用**（"不保持 ✓"那一栏也只是把已通过的斜抓配置重放了一遍）。

**缺陷（第 17 个，已定位）**：探针只给**臂**写了控制量，**没给狗写**（`ctrl=0`）
⇒ 狗在探针里是**塌的**（力矩型执行器、无控制量即失能，§5.2 实测 0.288372 → 0.077172 / 0.1 s）
⇒ 它的机身在台面附近、可能干扰载荷与臂。运行时有植物驻留线程在驱动狗 ⇒ 二者不可比。

**下一轮的真值表（一次定案）**：先修探针（**同时**驱动狗的姿态保持，与驻留线程同口径），
再跑 2×2：{方块轴对齐, 方块 45°} × {保持, 不保持} ⇒ 四栏里唯一"能搬"的组合就是该走的路。
本轮已排除的项不再重复（夹持力、闭爪位、接触对、朝向、时序表本身）。

#### (24) 2×2 真值表 + guest 节拍仿真：**我发明的"保持"在粗节拍下有害**；离线仍无法复现运行时

修好探针的狗（PD 姿态保持；第 17 个缺陷已修）后，探针与运行时在"已通过"的那一栏**一致**（轴对齐/不保持 +0.060903 ✓），
再叠加 **guest 节拍**（`PROBE_GUEST_STRIDE=18`，与实测 18.01 一致）得到 2×2：

| 组合 | 1:1 节拍 | **节拍 18（≈运行期）** |
|---|---|---|
| 轴对齐 / 保持 | -0.000216 m ✗ | +0.061078 m ✓ |
| 轴对齐 / 不保持 | +0.060903 m ✓ | +0.060470 m ✓ ← 与运行时一致 |
| **45° / 保持** | +0.036068 m ✓ | **+0.015020 m ✗** ← **与运行时 `lifted=false` 一致（复现根因）** |
| **45° / 不保持** | +0.037124 m ✓ | **+0.034545 m ✓** |

⇒ **"保持"（`pin_payload`）在 guest 粗节拍下有害**：pin 每 18 步才生效，
之间载荷照旧被推、且被钉成"陈旧位姿"后与夹爪失同步 ⇒ 45° 面抓反而失败。
⇒ 因此把 `approach_hold` / `close_hold` 都声明为 **`none`**（保留机制与证据字段，便于复用），
面抓取标定区间中值 **0.020**。

**但运行时在同组合（45° + 不保持 + 0.020）下仍然 `lifted=false`**（`lift_delta_m = 5e-06`，
`force_ok=true` 5.691327/5.734792 N，载荷被留在台面 `(0.2295889127203188, -0.27363767735823147)`）
⇒ **离线探针（含节拍仿真）已无法再逼近运行时**；剩余差异只可能在 guest/owner 真实交互、
狗的实时控制器与植物推进的具体时序里。
⇒ **下一步（运行时侧，唯一没做过的粒度）**：用后端现成的 `sampler` 钩子给 **LIFT 段做逐样本运行时追踪**
（`_move_trajectory(..., sampler=...)`，stride 1），量"载荷位姿 / 指腹 qpos / 接触对 / 力"随时间的演化
—— 运行时到底在哪一步、哪一个量先动，就能定案；离线侧到此为止（继续在离线调参已无信息增益）。

#### (25) **运行时 LIFT 段逐样本追踪**（新仪器）：载荷其实被抬起 1.8 cm，随后突然滑出

新增仪器（`IRAF_DEBUG_PICK=1` 时逐行打印 `PICK_LIFT_TRACE`，stride 见 `IRAF_DEBUG_PICK_STRIDE`）：
量 LIFT 段的"载荷位姿 / 指腹 qpos+ctrl / 接触对与力 / plant_step_index"。

**面抓（方块 45° + 闭爪位 0.020 + 两个 hold 均为 none）实跑 53 个样本**：

```
step  25–200 : 载荷 z 0.024292→0.024947 ｜ 指腹 z 0.0534→0.0536 ｜ 初期只有单侧指腹接触 + world(台面) 接触
step 200–800 : 载荷 z 0.024947→**0.043329**（被抬起 **+1.84 cm**）｜ 指腹 z 0.0536→0.1062
               ｜ 双侧接触 piper_link7/link8 **5.33~5.89 N** ｜ j7 误差 ~0.58 mm（⇒ ~5.8 N）
step ~810    : **突然滑出**（step 825 载荷 z 0.029906 → step 850 0.024783 落回台面）
step 850–1325: 载荷恒 0.024784（在台面）｜ 指腹继续升到 0.1339 ｜ j7 **qpos == ctrl == 0.0200000**（夹口空闭）
```

⇒ **关键新事实**：方块**先被抬起 1.84 cm**、双侧接触与力都正常（5.3~5.9 N），
在抬升段**峰值速度附近**（quintic 50% = step 1333 之前）突然滑出。
⇒ 而 `lift_delta_m = 5e-06` 这个**判据只取"结束时刻"的载荷 z** ⇒ 把"抬起过 1.8 cm"这段过程**掩盖**了
（**判据盲区**：应该同时给出**峰值抬升量**，否则"夹住→滑出"与"从未夹住"在报告里长得一样）。
⇒ 与 §11.23(15)(19) 的几何事实一致：指腹中心高于方块顶面 ~4 mm ⇒ 接触在方块**上部边缘**；
2F-85 指腹长 5.2 cm 而方块仅 5 cm 高 ⇒ **想夹腰部则指尖必进台面**（几何上被排除）
⇒ 唯一的"侧夹"窗口就是上部（现在这样），而它在此模型里**撑不住抬升中的动态扰动**。
⇒ **下一步候选（都没试过）**：① 判据改为"峰值抬升"（同时保留结束值）——先把事件可见；
② 用同一仪器把 stride 降到 1 精确定位"滑出前一步谁先动"（载荷位姿先动？指腹 ctrl 先动？）；
③ 若确认是动态扰动 ⇒ 声明化的**抬升速度上限**（把 lift 时长拉长，与"抓取/搬运速度"同一类声明）。

**⚠ 契约门禁抓到我一次**：我一度把 `lift_trace` 塞进 `pick_object` 的 evidence ⇒
`Provider output does not match schema: Additional properties are not allowed ('lift_trace' was unexpected)`
⇒ `pick_object.output.json` 是 `additionalProperties: false` 的契约（契约先行）⇒
**调试仪器只走 stdout 打印**，不进 evidence；要进证据就必须先改契约。

#### (26) stride=1 定案：**抬升段不是竖直的** —— 方块被沿台面拖行 2.2 cm 后拖出夹口

`IRAF_DEBUG_PICK_STRIDE=1`（1333 个样本，面抓 45° + 闭爪位 0.020 + holds=none）：

```
step 770 : 载荷 z 0.034534 ｜ x +0.247512 ｜ 接触 world dist **-0.001355 m**（压进台面 1.36 mm）力 2.288 N
           ｜ piper_link8 力 1.9497 N
step 775–800 : x 从 +0.2477 漂到 +0.2516（**横向移动**）｜ 力同步衰减：world 2.29→0.45、link8 1.95→0.17 N
step 806 : 载荷 x **+0.2475 → +0.2699（横向 2.2 cm）**、z 0.034576 → 0.033404（开始掉）
step 808–809 : z 0.023208 / 0.023260（落回台面）；此后只剩 world 接触
```

⇒ **真因**：`lift` 是**关节空间**五次多项式插值 ⇒ 工具点走的是**曲线**而不是竖直线
⇒ 方块在抬升过程中**被沿台面拖行 2.2 cm**、其角还**压进台面 1.36 mm**
⇒ 台面拖拽叠加**指腹接触力同步衰减**（1.95 → 0.17 N）⇒ 方块被**拖出夹口**。
⇒ 这也解释了"斜抓为什么能抬 8 cm"：斜抓的**过盈 4 mm** 把方块"卡"在夹口里，能顶住这段拖拽；
面抓没有过盈，拖拽就直接把它拽出去（§11.23(19)(20) 的机制在**动态**下的表现）。
⇒ **判据盲区同时确认**：`lift_delta_m` 只取结束时刻 ⇒ "抬起 1.84 cm 后滑出"与"从未夹住"都报 ≈0
⇒ 已新增诊断量 `lift_peak_delta_m` / `lift_peak_step`（契约 `pick_object.output.json` 同步加了可选字段）。

**修法（声明化、边界清楚）**：抬升（以及搬运）必须走**任务空间直线**而不是关节空间插值
⇒ 声明 `lift path mode: straight_vertical`（沿接近轴直线），由构建期把该直线**采样成若干关节航点**
（与 §11.17 的绕行航点同一手法）⇒ 后端逐段回放 ⇒ 工具点全程保持竖直、不再拖拽载荷。
这一项与 §11.23(11) 的 `transit→above` 穿 `base_link` 修法是同一类工作，建议合并做。

#### (27) 定案：载荷在夹口里**翻滚出去**；面抓对本几何（5 cm 方块 + 5.2 cm 指腹）无法提供抗转力矩

**声明化落地**：基线 `grasp.lift_path ∈ {direct, approach_then_lift}` → 构建器写入报告
（`gripper.lift_path`，语义键不参与前缀改写）→ 后端按声明选择单段或**两段**回放
（`approach_then_lift` = 先走到构建期解出的 `approach` 位形（= 抓取点沿接近轴竖直抬高
`pregrasp_offset_m` 的解 ⇒ **任务空间竖直段**），再走向 `lift`；`direct` = 原单段行为）。
当前取 **direct**（= s01–s03 已验证行为），机制与两种模式都保留、可声明切换。

**实测（45° 面抓 + 闭爪位 0.020）**：
- `approach_then_lift`（先竖直 4 cm 再横向）**仍然丢件**；
- 把拾取时长从 8000 翻到 16000 ms（抬升慢一倍）**仍然丢件** ⇒ **是运动学问题，不是动力学问题**；
- 逐样本（stride 50）**载荷姿态**：

| step | 载荷 z (m) | 姿态相对初始偏角 |
|---|---|---|
| 50 | 0.024280 | 0.000° |
| 450 | 0.025476 | 1.377° |
| 650 | 0.028160 | 7.954° |
| 850 | 0.031772 | 19.605° |
| 1050 | 0.035018 | 37.579° |
| 1250 | 0.027354 | **95.480°** |
| 1450 | 0.035844 | 119.601° |
| 1650+ | 0.024784 | 102.029°（躺定） |

⇒ **真因（最终定案）**：面抓下指腹只压住方块的**上缘**（§11.23(15) 实测几何：指腹长 5.2 cm、
方块高 5 cm ⇒ 想夹腰部则指尖必进台面，被几何排除）⇒ 夹持对**转动**几乎没有阻力
⇒ 抬升/拖拽一施加，方块就**绕接触线翻滚**（0° → 120°），翻滚时角扫过台面（这正是
`z≈0.036` 时仍有 0.10~0.14 N 台面接触的来源），最后从夹口翻出落地。
⇒ **斜抓之所以能抬 8 cm**：4 mm 过盈把方块**卡死、转不动** —— 它用"过盈"换了"抗转"。

**由此得到的选项（都清楚、都需要使用者裁定方向）**
1. **保留斜抓（过盈）**：拾取/抬升已验证可行；必须解决的是**搬运段**的楔出（§11.23(12)(19)(20)）
   ⇒ 需要"搬运段不让过盈起作用"的办法（例如搬运中把夹爪**松开一档**到刚好不楔、或改搬运姿态）。
2. **面抓 + 深闭合（面抓版的过盈）**：把闭爪位压到使指腹在方块**面上**产生过盈 ⇒ 抗转成立，
   代价是要重新面对"楔出"（但楔出方向此时是沿夹口轴，可与搬运方向解耦，需实测）。
3. **换抓取策略**：先**抬起一点**（离台）再**合爪**到方块**腰部**（此时指尖不再受台面限制）
   ⇒ 能拿到真正的面夹与抗转力矩 —— 这需要新的相位（"先抬后夹"），是 pick 时序的进一步重设计。

#### (28) 选项 2（"面抓 + 深闭合"）**被否**：面抓在任何闭爪位都托不住

用逐相位回放探针（`PROBE_GUEST_STRIDE=18` + 狗 PD 保持 + `PROBE_CLOSED` 覆盖闭爪位）扫：

| 闭爪位 | 轴对齐（斜抓） | 45°（面抓） |
|---|---|---|
| 0.020 | ✗ | ✗ |
| 0.017 | **+0.067919 m ✓** | ✗ |
| 0.015 | **+0.061575 m ✓** | ✗ |
| 0.013 | **+0.060532 m ✓** | ✗ |
| 0.011 | **+0.060890 m ✓** | ✗ |

⇒ **面抓在任何闭爪位都托不住**（连最深档 0.011 也不行）⇒ "加深闭合得到面抓版过盈"**不成立**：
与 §11.23(27) 的翻滚机制自洽 —— 指腹只压方块**上缘**，越用力只会把方块**往下/往外挤**
（转动阻力仍然不足），而不是把它钉住。
⇒ 斜抓在 0.017 及更深档**全部能抬**（+0.060~+0.068 m）；注意 0.020 在**探针**里失败而运行时 0.023 通过
⇒ 探针在"斜抓"这一栏仍不是运时行为的可靠代理（**只能用于否定、不能用于肯定**），
这一点本身也记下来：**离线结论的效力等级 = 只能"证否"**。

⇒ **可行方向收敛为两条**：
- **选项 1（保底，已部分验证）**：保留斜抓（过盈提供抗转、抬升已验证），专治**搬运段楔出**。
- **选项 3（更彻底）**：**先抬离台 → 再合爪到方块腰部**（离台后指尖不再受台面限制 ⇒ 能拿到真正的
  面夹与抗转力矩），需要给 pick 时序再加一个相位（"先抬后夹"）。
两者都不是"调参"，而是时序/路径层面的声明化改动，边界清楚。

#### (29) regrasp 已实现（计划第 1–4 步）但**在第一步就失败**：唯一撑得住抬升的抓取是"斜夹过盈"

**已落地（`regrasp.enabled: false` 时与改动前逐位一致，机制可声明开启）**
1. 契约先行：`pick_object.output.json` 的 evidence 增加 `lift_attitude_max_deg` 与 `regrasp`（均可选）。
2. 求解器：`build_reference_poses` 按 `grasp.regrasp` 解出 `pre_lift` / `regrasp` 两个位形
   （= 抓取点沿**预抓取方向** +h 的解，与 approach/lift 同一算法；参数非法即报错）。
3. 构建器：写入 `gripper.pre_lift_positions` / `gripper.regrasp_positions` / `gripper.regrasp`（含出处）；
   `regrasp` 登记为语义键（不参与前缀改写）。
4. 后端：相位表变为 …GRIP_CLOSE → **PRE_LIFT → REGRASP_OPEN → REGRASP_DESCEND → REGRASP_CLOSE** → LIFT；
   常驻跟踪**载荷姿态偏角**（进证据 `lift_attitude_max_deg`，判读顺序"先看姿态再看力"）；
   判据口径修正：有 regrasp 时 `lift_delta_m` 从 **PRE_LIFT 之前**起算（否则 4 cm 预抬被漏计）。
5. 解析层透传：`pre_lift_positions`/`regrasp_positions`/`regrasp` 必须进后端配置白名单
   —— **同一类坑第三次踩到**（`carry_gripper`、`lift_trace`、现在 regrasp）：新键不加白名单就静默消失。

**实测（面抓 45° + 闭爪位 0.020 + regrasp 开）**：相位表**确实全部执行**
（日志：HOME_HOLD→APPROACH→DESCEND→GRIP_OPEN→GRIP_CLOSE→**PRE_LIFT→REGRASP_OPEN→REGRASP_DESCEND→REGRASP_CLOSE**→LIFT），
但方块在 **PRE_LIFT（声明 0.04 m）** 之后只升到 **z=0.0338（即只升了 9 mm）** ⇒ **第一次"压上缘"抓取连 4 cm 的竖直抬升都撑不住**
（§11.23(27) 的翻滚机制）⇒ **regrasp 永远到不了** ⇒ 选项 ③ 在本几何下**不可行**。

**战略结论（本会话最重要的判断）**
- 本几何下（2F-85 指腹 5.2 cm vs 方块高 5 cm，台面在最下方）：
  **唯一能撑住抬升的抓取是"斜夹过盈"**（已验证：抬 8 cm）；而它**在搬运中沿夹口轴向楔出**。
- ⇒ "调夹持/调路径"这条线**到此为尽**（已有 30+ 条被量化否掉的假设）。
- ⇒ 剩下的可行路线是**声明化的搬运约束**：**臂侧世界本来就有** `box_01_lift_constraint`
  （`<connect name="box_01_lift_constraint" body1="grasp_anchor" body2="box_01" active="false">`，
  后端在 `force_ok` 且报告带 `gripper.lift_constraint` 时置 `eq_active=1`，并配套
  `_advance_with_grasp_anchor`），且臂侧基线明确声明 `require_friction_lift: false`
  —— 即"**不要求纯摩擦抬升**"是**既有且已验收**的语义。把这条约束**接进联合世界**
  （联合构建器注入 anchor + equality、报告带 `lift_constraint`）即可让搬运确定成立，
  且它是**声明 + 证据可见**（`constraint_activated` 已在证据里）的，不是伪造成功。

#### (30) 搬运段约束**生效** —— 搬运段首次全程保持夹持；剩下的是**布置层面的不可行**（双向确认）

**落地（声明化搬运约束，全链路）**
- 基线 `grasp.carry_constraint: {enabled, equality_name, anchor_body}` → 构建器写入报告 `gripper.carry_constraint`
  → 联合构建器 `_inject_carry_constraint` **惰性注入** mocap `grasp_anchor` + `<connect ... active="false">`
  （位置取**场景** props 的位姿，名字全来自声明）→ 后端在 `place_object` 开头按声明 `eq_active=1`、
  三段搬运每步把 anchor 驱动到**指腹中点**、**释放约束后**才张开夹爪；证据留 `carry_constraint_activated`。
- **pick 侧不动**：臂侧基线 `require_friction_lift: true`（"抬升必须纯摩擦"）是既有已验证声明 ⇒
  约束只作用于搬运段，两个世界的语义各自自洽。

**实测（`nominal --world joint`，s04 临时启用）**
```
start        : 指腹 (0.2802,-0.2801,0.1335) ｜ 载荷 (0.2760,-0.2756,0.1043) ｜ 接触 True/True
after_transit: 指腹 (0.2804,-0.2804,0.4187) ｜ 载荷 (0.2487,-0.2487,0.3909) ｜ 接触 True/True   ← 此前在此丢失
after_above  : 指腹 (0.0975,-0.1733,0.4208) ｜ 载荷 (0.0432,-0.1617,0.4181) ｜ 接触 True/True
after_descend: 同上（下探未发生）                                      ｜ 接触 True/True
```
⇒ **搬运段（transit→above→descend）首次全程保持夹持**（105 行 trace 走完三段）⇒ 约束方案成立 ✓。

**新暴露的两个我自己的缺陷（都已修）**
1. `anchor_follow` 传了 **body id** 而钩子按**名字**解析 ⇒ `body not found: 26`；
2. `gripper_close_hold`/`gripper_approach_hold` 是 **pick 的**语义，却被我早先误写进 **place 的**证据
   ⇒ 潜伏 `NameError: close_hold`，**直到搬运段第一次跑通、代码走到证据装配才暴露**（说明"跑通"本身是最强的静态检查）；
3. 契约里又出现未转义 ASCII 引号 ⇒ `invalid Skill schema`（第 N 次踩，已记）。

**最后的阻塞：布置层面的不可行（§11.18 的双向确认）**
- 构建器警告：`above/descend/retreat` 落在**翻转分支**（`dot_with_pick_lift = -0.721795 / -0.702082`），
  根因是"托盘顶面 0.449 m 处要求腕部 0.67 m > 可达 0.594 m"；
- 实测 `after_above` 的指腹在 **(0.0975,-0.1733)** 而托盘在 **(0.4221,0.0014)** ⇒ 放置航点离托盘 **41 cm**；
- 声明级修法试过：**抬高臂基座 +0.06 ⇒ 警告仍在**；**+0.10 ⇒ 警告仍在，且 `pick` 门禁被打破
  （残差 0.0999928 ≫ 0.005）** ⇒ **不存在同时满足"台面拾取"与"狗背托盘放置"的基座高度**。
  （方块在 z=0.025、半径 0.24 m；托盘在 z=0.355、半径 0.45 m；垂直跨度超出该臂能力。）
⇒ 这属**布置层面决策**（不是调参）：可选
  ① 给方块加**台架**把拾取点抬高（两侧都在中段高度，最容易成立）；
  ② 改用**更低/侧向**的交接平台（不再是"狗背托盘"）；
  ③ 换更长臂展的本体。三条都要改场景声明，并重跑 s01–s03 验收。

#### (31) **定论：`PIPER + Go2 狗背托盘` 在本几何下不可行**（托盘已到最低、无姿态可降、基座高度无解）

三条独立证据把这结论钉死：
1. **托盘已在最低位置**：`tray_frame` 的挂载偏移 `pos_m: [0,0,0.057]` 就是 **Go2 躯干碰撞盒的顶面**
   （`profiles/unitree_go2_mujoco.yaml:99-103` 注释即此依据）⇒ 托盘是**贴背**的，**不能再降低**；
   实测托盘顶面 `z = 0.355372`（名义位姿 `[0.45, 0.0, 0.345372]` + 半厚 0.01）。
2. **臂够不到**：该托盘位姿要求腕部 ≈0.67 m > 本臂可达 **0.594 m**（缺口 **7.6 cm**）⇒ 构建器把
   `above/descend/retreat` 落到**翻转分支**（`dot_with_pick_lift = -0.721795 / -0.702082`），
   实跑 `after_above` 指腹 **(0.0975,-0.1733)** vs 托盘 **(0.4221,0.0014)** ⇒ 离托盘 **41 cm**。
3. **基座高度无解**：抬高基座 +0.06 ⇒ 警告仍在；+0.10 ⇒ 警告仍在**且 pick 门禁被打破**
   （残差 0.0999928 ≫ 0.005）⇒ **不存在同时满足"台面拾取"与"狗背托盘放置"的基座高度**。
另外：狗的**蹲伏/低姿态能力当前没有声明**（`profiles/unitree_go2_mujoco.yaml` / `config/go2_joint.yaml`
里查不到 crouch/posture/lie 类键）⇒ "让狗蹲下来接"需要**新交付一项 Go2 姿态能力**。

**可行方向（都要改声明/交付新能力，属使用者的布置决策）**
- **② 交接平台降到 ≈0.20 m**（狗旁边的低台/小车）：最容易，改动只在场景道具；
  但演示语义从"放到狗背上"变成"放到旁边的低台"。
- **③ 换用臂展更长的本体（UR5e，可达 0.85 m ≫ 0.67 m）**：仓库里已有 UR5e + 2F-85 的
  Profile/构建器（`profiles/ur5_mujoco.yaml`、`scripts/build_ur5_baseline.py` 等）
  ⇒ 新增一个「Go2 + UR5e」联合场景变体，**完整保留"臂把方块放到机器狗背上"的语义**；代价是
  新场景 + 重跑 s01–s03 验收。
- **④ 交付 Go2 蹲伏姿态**（新能力，需 loopback 验收）⇒ 再回到 Piper 方案。

**本轮已完成、与本结论无关的成果仍然有效**：搬运段**声明化抓取约束**让搬运首次全程保持夹持
（§11.23(30)）；`place_object` 的其余判据（释放、落位偏移）等待上述布置决策后再验收。

#### (32) ③（换 UR5e）的真实工作量 + 倾角法被否：朝向救不回托盘几何

**侦察结论（③ 的成本）**：UR5e 侧**没有声明式求解入口** ——
`scripts/build_ur5_baseline.py` 与 `scripts/build_robot_pick_scene.py` 里都查不到
`build_reference_poses` / `build_place_reference_poses` / `build_reference_feedforward`
（这三个入口目前只存在于 `scripts/build_piper_baseline.py`）⇒ "Go2 + UR5e" 变体**不是"加个场景"**，
而是要把**整套声明式求解栈**（参考姿态重解 + 放置四段 + 重力前馈 + 指尖离台配平 + 夹持标定）
移植到 UR5e 并重跑它自己的验收 ⇒ 属**独立工作项**，不是本会话一次能收口的改动。

**倾角法（新试）**：给放置段接近方向加声明化旋钮 `grasp.place_approach_direction`
（缺省 `[0,0,1]` ⇒ 与改动前逐位一致），试 `[0,-0.342,0.940]`（向基座一侧倾 ~20°）：

```
警告仍在：above dot_with_pick_lift = -0.75744 ｜ descend = -0.725953 ｜ retreat = -0.75744
放置残差仍好（7.347e-06 / 6.948e-06），pick 门禁 7.233e-06 不变
```
⇒ **朝向救不回来**：要让指腹**朝下**送达托盘上方（|xy| = 0.45 m、载荷中心在托盘上方 0.29 m），
腕部必然位于载荷**外侧**约 0.26 m ⇒ 腕部半径 `≈ sqrt(0.45² + (0.29+0.26)²) = 0.707 m > 0.594 m`
⇒ **托盘相对这条臂既太远、也偏高**（不是"只差 7.6 cm 高度"那么简单）。
旋钮保留（声明化、可复用），当前取竖直。

**因此剩下三条（都是布置/本体层面的取舍，非调参）**：
① **低交接平台 ≈0.20 m**（狗旁边）：改动最小；语义从"放狗背"变成"放低台"。
② **UR5e 变体**：需移植整套声明式求解栈（见上），完整保留语义，工作量最大。
③ **Go2 蹲伏姿态**：新能力 + loopback 验收后回到 Piper 方案。
（另：抬高基座/给方块加台架/倾角——三条都已被数字否掉：§11.23(30)(31)(32)。）

#### (33) ① 侧挂交接位：**几何侧已验证可行**，但会扰动停靠链路 ⇒ 属工作项（非一行改动）

**做法**：新增声明化挂位 `pannier_frame`（`profiles/unitree_go2_mujoco.yaml`，
`body: base_link`、`pos_m: [0.0, -0.22, -0.088372]` ⇒ 托盘顶面 **0.21 m**、名义位姿 `(0.45,-0.22,0.2)`），
把 `scenes/handoff_lab/scene.yaml` 的 `props.tray_01.pose.mount.frame` 指向它。

**几何侧结论（① 成立）**：
| 指标 | 背上 `tray_frame` | 侧挂 `pannier_frame` |
|---|---|---|
| 托盘顶面 z | 0.355372 | **0.21** |
| 放置 `above/descend/retreat` 夹爪轴 z | +0.0175 / −0.0960 / +0.0175（近水平 ✗） | **−0.790294 / −0.813359 / −0.790294（朝下 ✓）** |
| 放置残差 | 7.347e-06 | 7.053e-06 / 6.779e-06 |
| pick 门禁 | 7.233e-06 | 7.233e-06（不变）|

⇒ **侧挂位让放置姿态几何可达、且夹爪朝下** ✓（构建器剩下的"半球"警告是 `dot ≈ -0.078` 的近垂直配对
**误报**：两轴都朝下，只是绕竖直转了方位）。

**但它整链红了 —— 而且不是几何原因**：
- `s01` 末速 `1.4887881120651461e-05 → 3.6867794098423057e-05`（0.35 kg 托盘移到侧面 ⇒ 狗动力学变了）；
- `s02_dock` 报 **`Provider output does not match schema: None is not of type 'number'`**；
- 狗随后**撞到方块** ⇒ `s03` 报"抓取位姿与目标位置不一致 0.086909 m"（连带，非独立问题）。
⇒ **① 是工作项**：需要连 `s01/s02` 一起重验，并把 `s02` 那个 `None` 证据问题查清；
另外方块与狗腿的拥挤（§11.23(14)：腿距抓取点 7~10 cm）会放大这类连带。

**另一个坑（已记）**：**不能直接移动 `tray_frame`** —— 它是场景**停靠判据**的测量帧
（平移 ≤30mm、偏航 ≤2deg）；改它会让 s02 直接红。侧挂位必须用**新帧**。

#### (34) ① 侧挂位把 `s02` 打红的**准确诊断**（不是"边界差一点"）

从 `build/iraf-a6a14/diag-pannier.log` 取证，报错链条是：

```
s02 判据 translation_error_max_m 未满足：<无依据> 0.03，测量 None
    （评测依据不可用：dock_translation_error_m ← evidence.final_translation_error_m）
s02 判据 yaw_error_max_deg 未满足：<无依据> 2.0，测量 None
    （评测依据不可用：dock_yaw_error_deg ← |evidence.final_yaw_error_deg|）
s02 状态 FAILED：Provider output does not match schema: None is not of type 'number'
```
⇒ 停靠**没有产出终态测量**（两项 `final_*` 取到 None）⇒ 契约要求数字 ⇒ 被 schema 拦下。
⇒ 即：**侧挂 0.35 kg 托盘后，狗连"停靠完成并给出终态误差"都没做到**（不是"差 1.43 mm"），
   `s01` 末速同时从 `1.4887881120651461e-05` 变成 `3.6867794098423057e-05` 也印证动力学被改变。

**两件后续工作（已定位，都可独立推进）**
1. **契约表达**：后端在"停靠未完成"时应给出**显式**结论（例如 `settled: false` + 说明），
   而不是让一个 `None` 字段触发 schema 错误 —— 现在这种报错会掩盖真实原因（本次即如此）。
2. **停靠鲁棒性/布置**：侧挂位改变了狗的质心 ⇒ 停靠链路退化。可选：
   ① 把托盘做**轻**（或改用侧挂件的轻量声明）后复测；
   ② 先做停靠侧的"终端段与墙钟解耦"（历史议题，见 §11.11/§11.12）；
   ③ 重新评估交接面：把交接**面**仍留在背上、但**打开方向**改为侧向（对臂更友好）——需要
      位姿型 IK 的朝向约束（§11.8：Piper 缺 `flange_site`/`spread_axis` 两处声明）。

#### (35) 第 1 件完成：停靠失败改成**显式结论**（不再被 schema 报错掩盖）

**改动（契约先行 + 实现）**
1. `skills/dock_for_handoff/dock_for_handoff.output.json`：`final_speed_mps` 类型改为
   `["number","null"]` 并加说明（null = 未测得）
   —— 原因：接近段失败时后端只会留下 `inf` ⇒ 写进证据是 `None` ⇒ **旧契约只允许 number**
   ⇒ 报成 `Provider output does not match schema: None is not of type 'number'`，把真实原因盖住。
2. `src/iraf_skills/quadruped.py`（`DockForHandoffProvider.execute`）：拿到后端报告后**先看
   `report["failure"]`**，非空即 `raise SkillRejected("停靠未完成（<decision>）：<reason>")`
   ⇒ 失败原因**直接可见**；未失败才组装证据。
3. ⚠ 顺带修掉一处**潜伏 NameError**：新代码用了 `SkillRejected` 而该模块**没有导入**它
   （只在停靠失败时才触发 ⇒ 绿路径测不出来）⇒ 已加
   `from iraf_skills.common.motion import SkillRejected` 并做导入自检。
   （同类坑本会话第三次：`close_hold`、`lift_trace`、现在是 `SkillRejected` —— 共同点是
   **只在失败路径/首次跑通时才执行**的分支，静态检查与绿路径都覆盖不到。）

**验证**：`nominal --world joint` 仍全绿（`passed=True`；s01 末速 1.4887881120651461e-05、
s02 停靠 0.028569272841750617、s03 抓取 0.0002101559338561355）；模块导入自检通过。

**下一步（按计划）**：把侧挂托盘**做轻**（纯声明）后复测 s01/s02 —— 若停靠恢复正常，
就能把托盘挂回 `pannier_frame` 一路跑到 s04。

#### (36) 侧挂筐**挡住狗自己的腿** ⇒ "放在狗身上"对本组合彻底不可行（与质量无关）

**契约修复立刻见效**（第 1 件，§11.23(35)）—— 失败原因现在直接可见：

```
s02_dock FAILED：停靠未完成（damped_hold）：停靠超时 12.000 s 未进入容差：
  平移 0.200821 m（判据 0.030000）/ 偏航 37.191384°（判据 2.000000）
```

**把侧挂托盘做轻到 0.05 kg 复测 ⇒ 仍然同样失败**（同样是"超时未进入容差"）
⇒ **与托盘质量无关** ⇒ 真因是**几何干涉**：0.24×0.16 的托盘从躯干侧面伸出 **0.22 m**、
还低 **0.088372 m**，正落在四条腿的摆动空间里 ⇒ **步态被破坏**，狗走到不了站位。
而"腿所在的那一侧"正是唯一对臂友好的挂位（背上的最低点仍然太高、够不到）
⇒ **"把方块放到狗身上"对 `Piper + Go2` 这个组合在布置层面无解**（三条独立证据：
背上够不到 §11.23(31)、抬高基座无解 §11.23(30)、侧挂挡腿 本小节）。

**因此交接面必须离开狗体**：唯一同时满足"臂够得到"与"不干扰狗"的布置是
**狗旁边的低位交接台**（z ≈ 0.20 m，独立道具）—— 这是 ② 方案，也是我现在的最终推荐；
若演示语义必须保持"放到狗身上"，则只能走 UR5e 移植（③，需移植整套声明式求解栈）
或先交付 Go2 蹲伏能力（④）。

**本轮同时修好的一条基础设施（长期有效）**：停靠失败不再被 schema 报错掩盖
（`final_speed_mps` 允许 null + provider 按 `report["failure"]` 显式拒绝，
并修掉 `SkillRejected` 未导入的潜伏 NameError）。

#### (37) 方案②（把交接面移到狗体之外）落地一半：构建器已支持**世界固定接收体**，但整链卡在重力前馈验收

**已落地（能力扩展，长期有效）**：`_place_targets` 原先**只把带 `pose.mount` 的道具当接收体**
（"接收体随载体运动"的设计假设），因此**世界固定的接收体**（例如放在固定交接台上的托盘）
会被直接跳过 ⇒ 构建报"场景报告没有 place_targets：无法解放置段"。
现在：没有 `mount` 但声明了 `pose.pos_m` 的道具**也是接收体**（`pose_source: scene_declared_pose`，
`mount: null`；运行期来源仍是实测 FK —— 静态体不随任何载体动，FK 即其自身位姿，口径不变）。
实测（把托盘放到 `(0.62,-0.40,0.21)` 的固定台面上）：放置三段夹爪轴 z =
**-0.543210901 / -0.549119639 / -0.543210901（朝下 ✓）**，即**几何侧完全成立** ✓。

**但整链构建卡在另一处（新，未诊断）**：
```
场景生成失败（退出码 5）：重力前馈在联合模型上重算失败（build_reference_feedforward）:
  静态保持 4000ms 后最大关节误差 0.025750988 rad（限 0.001）
  残余={'piper_joint1': -0.000164907, 'piper_joint2': 0.025750988, 'piper_joint3': -0.00495132, ...}
```
⇒ 与**拾取姿态无关**（pick 的位姿未变、pick 门禁仍是 7.233e-06），却只在改动接收体布置后出现
⇒ 待查（怀疑与"接收体是否挂载"影响到的构建分支有关，例如放置段解出后对模型/关键帧的副作用）。
**另**：新增的台面道具 `handoff_stand` 出现了"生成模型里找不到 body"的 scene_check 报错
（道具未被注入）⇒ 本轮先整块移除，只在 scene.yaml 里留下**启用指引**（改哪两行）。

**当前状态**：托盘挂回狗背 `tray_frame`（几何无解但整链绿：s01–s03 全绿）；
`pannier_frame`（侧挂，几何可行但挡腿）与"世界固定接收体"能力都作为**已声明/已实现**保留，
下次要用时按 §11.23(36)(37) 的指引启用并解决前馈阻塞。

#### (38) 世界固定接收体：能力已落地（`receiving: true` 声明）；② 的**位置窗口很窄**（实测两端都不可行）

**已落地（能力，长期有效）**：接收体现在**由声明区分**——
① 带 `pose.mount` 的道具（随载体运动，运行期 `live_fk`）；
② **世界固定**的道具：必须显式声明 **`receiving: true`**（prop 级键，已进 `scene.schema.json` 的
`definitions.scene_prop`）。为什么必须显式：本轮的教训是"放宽成『任何有 `pose.pos_m` 的道具都算接收体』"
会让**载荷自己**被当成接收体（实测 `place_targets` 变成 `[('box_01', [0.28,-0.28,0.025]), ...]`，
即"把方块放到方块自己上"）。
验证：`place_targets` 恢复为 `[('tray_01', [0.45, 0.0, 0.345372])]` ✓；整链绿。

**② 的位置窗口实测（两端都不可行）**
| 放置位置 | 相对臂基座 | 结果 |
|---|---|---|
| `(0.62,-0.40,0.21)`（贴基座，|xy|≈0.18 m） | 很近 | 放置轴朝下 ✓（z = −0.543211 / −0.549120），但整链构建**卡在重力前馈验收**（joint2 残余 0.025750988 rad > 0.001，§11.23(37)） |
| `(0.80,-0.75,0.01)`（离狗远，|xy|≈0.46 m） | 较远 | `放置段 above 无可用解`（残差 0.036913913 m、`axis_dot` −0.409）⇒ **"指腹朝下"的姿态 IK 在这个距离够不到** |

⇒ **② 是"窄窗口"工作项**：可行位置必须**足够近**（满足带朝向约束的 IK）又**足够远**（不撞臂的位姿区、
不挡狗的腿）。**下一步（有界、廉价）**：写一个**构建期位置扫描**（每个候选位置一次 `build_scene`，
秒级）在 `|xy| ∈ [0.20, 0.45] m × z ∈ [0.02, 0.22] m` 上找可行窗口；
找到后把托盘放到该位置（或放上台面道具），即可跑到 s04。

### 11.19 撤两条假设 + 第 9 个工装缺陷：搬运丢件的机制**仍未判死**（2026-09-28）

**撤销 1：夹具（equality）不是原因。** 臂场景模型里确实有一条 `box_01_lift_constraint`
（`neq=1`，`active="false"`），而联合模型 `neq=0`；但**臂报告里没有** `lift_constraint` /
`lift_anchor_body` 两个键，而后端的激活逻辑要求这两个键 ⇒ **两个世界里都从未激活过它**
⇒ 臂侧已验收的抓取同样是**纯摩擦**。所以"联合模型缺夹具 ⇒ 搬运掉件"不成立（我在实现到一半时
用报告键一查就撤回了，没有把它当结论）。

**撤销 2：单起点 IK 翻转不是"几何不可达"（§11.18 作废）。** 多起点扫描
（`build/iraf-a6a14/place_ik_multistart_probe.py`：4 个基准种子 × 27 个 joint2/3/5 扰动）
找到**同半球且残差达标**的解：`perturb_-0.4_+0.4_+0.4 → 残差 5.98e-06、轴 z=-0.156（朝下）、
腕部 0.562792（余量 +0.031492 m）` ⇒ 放置姿态**可行**，问题在求解器的单起点行为。
**已修**：`build_place_reference_poses` 改成**多起点 + 按"与种子轴同半球、dot 最大"选解**，
并把 `chosen_seed`/`axis_dot_reference`/`attempts` 写进每段结果（可追溯）；
重建后 `descend` 轴 z=-0.156、`transit` 轴 z=-0.363（都是朝下 ✓）。

**第 9 个工装缺陷（我的，必须记）**：`vertical_carry_limit_probe.py` 一开始就报"起点接触 0"——
因为我**把臂直接设到抬升位形**（从关键帧起），**从没做过抓取动作** ⇒ 方块还在台面上、指腹在空处，
当然 0 接触、当然"抬不动"。**纪律**：要量"抓起来之后能不能搬"，工装必须**复现完整时序**
（open → 到抓取位 → 闭合 → 抬升 → 搬运），或者干脆**走 runtime 跑真实技能**（`pick_object`）到抓取态再接手；
不能"摆一个看起来对的姿态"就开始量。

**当前状态**：搬运丢件的机制**仍未判死**（纯摩擦是唯一剩下的候选，但需要"复现完整时序"的工装来量
可搬运高度）。s04 仍 `SKIPPED_PENDING`（能力未声明），单本体与联合世界验收保持绿；
本轮保留的改动只有**求解器多起点选解**（已用探针验证）与文档。

### 11.18 s04 的最终阻塞是**场景几何**：托盘顶面"从上方放"对手臂不可达（2026-09-28 判死，FK 级）

**先确认链路本身是对的**（都已完成、都 inert）
- 构建期求解器 `build_place_reference_poses` + `place_entry` 声明 + 后端**回放**：
  FK 对账 `above → 指腹 (0.450001, -4e-06, 0.449299)`、`descend → (0.449999, -1e-06, 0.409313)`
  与目标（承载面 0.355 + 0.0939 / +0.0539）逐位吻合 ⇒ **求解与回放无误**。
- 绕行航点 `transit`（抓取点正上方、托盘高度）：`transit_local_m = [0.240415363, 2.78e-07, 0.449303693]`。
- 两项**构建期自检**：① 航点 ↔ **载体子树**接触（按运动学子树判定；第一版按"名字无前缀"判会把
  "臂底座↔台面"这条正常固定接触报成撞载体）；② 航点夹爪轴 vs **pick 抬升段的轴**（参照必须是被验证过的那段）。

**判死过程（FK 级，几秒）**：把 pick 的四段与放置四段的关节解分别设进联合模型，量"腕→指腹"轴：

| 位形 | 夹爪轴（腕→指腹） | 含义 |
| --- | --- | --- |
| grasp / lift（pick，已验收） | (−0.431, 0.431, **−0.792**) / (−0.595, 0.595, **−0.541**) | 手指**朝下** ✓ |
| place transit / above / descend / retreat | z = **+0.406 / +0.341 / +0.238 / +0.341** | 手指**朝上** ✗（dot 与 pick 抬升轴 −0.76 ~ −0.97 ⇒ 全翻） |

⇒ 位置型 IK "解出"的放置姿态是把夹爪**翻过来**（手指朝上）。原因不是 IK 调皮，而是**几何不可达**：
把方块放到**托盘顶面**再松手，指腹要在 0.449 m 高、腕部还要再高约 0.05 m ⇒ 腕部 ≈ (0.45 m 远, 0.50 m 高)
⇒ **0.67 m > 臂可达 0.594 m** ⇒ 唯一"能解"的分支就是把夹爪翻上去（物理上荒谬）⇒ 搬运途中载荷被甩掉
（运行时实测：`after_transit` 载荷最低点 −0.000216 m、双侧指腹接触 false）。

**⇒ 结论：阻塞在场景几何，不在代码。** 可选项（都属声明层，需你定）：
1. **把托盘在载体上挂低**（或加"呈现托盘"的蹲伏/倾斜姿态）：让托盘顶面进入"手指朝下可放"的球壳内
   （现托盘名义顶面 0.355 m；从基座算需 ≤ 0.594 m 且留出腕部余量）；
2. 侧向滑入（手指水平、把方块推上托盘）：本场景被**载体躯干**挡住（托盘就压在狗背上）⇒ 不可行；
3. 改臂的摆放（更近）：已被"四足走廊"否掉（y=−0.30 会让 s02 dock 失败，§11.13）。

**当前状态**：以上解与自检全部 **inert**（`place_object` 未声明、s04 仍 `SKIPPED_PENDING`），
单本体与联合世界的验收都保持绿；`pose_check 7.233e-06`、侵入自检 false 未受影响。

#### 11.18 补：可达性"边缘"判定成立；我那份扫描探针的读数**不可信**（第 6 个工装缺陷）

- 我写 `place_reach_sweep_probe.py` 扫承载面高度时，**参照轴算错**（在探针里用 arm 场景模型+报告 lift 值
  重算"pick 抬升轴"，与求解器内部探针模型的口径不一致）⇒ 打印出"手指朝下 ✓、dot +0.43、余量 +0.059 m"
  这类**与其它两条独立测量矛盾**的读数。
- 同进程内做了**入参对照**（A=与构建器完全相同；B=transit 换成 descend 自身目标；C=不给 transit）：
  三者**都返回同一组翻转轴**（`above [0.641,-0.688,+0.341]`、`descend [0.664,-0.709,+0.238]`、`retreat` 同 above）
  ⇒ 求解器**稳定落在翻转分支**。
- 两条独立测量互相印证（构建器的 FK 自检 + 入参对照测试），并与下面的量级估计一致：
  腕部要在 (≈0.45 m 远, ≈0.50 m 高) ⇒ 距基座 **≈0.587–0.60 m** vs 可达上界 **0.594284 m**
  ⇒ **处在可达边缘**（余量 ~0），位置型 IK 无法在容差内维持"手指朝下"⇒ 翻夹爪。
⇒ §11.18 的结论（**阻塞在场景几何，需要给托盘/载体留余量**）成立；那份扫描的"可达"是**工装伪像**。
**纪律（第 6 条，已并入 §11.16 的清单）**：跨模型的量（轴、位置）必须**在同一模型/同一口径**下算，
不能"在 A 模型里算参照、拿 B 模型的输出去比"。

**另修一处流程坑（同一个坑踩了两次）**：`scenes/handoff_lab/scene.yaml` 同时承载
"能力列表"与"构建期求解器声明"（`reference_solver.place_entry`），而我在回退"临时启用"时用
`git checkout -- scenes/handoff_lab/scene.yaml` **整体回退** ⇒ 把 `place_entry` 一起删掉（构建器静默
不再解放置四段、报告里 `seed/axes` 变 `null`）。⇒ `place_entry` 现**永久保留**（它只是构建期声明、
不会让运行期执行任何东西），回退临时能力**只改 `capabilities:` 那一行**。

### 11.17 放置段改成"构建期解 + 后端回放"：**求解与回放都对了**，缺一个**绕行航点**（2026-09-28）

**已落地（本轮）**
- 求解器侧：`scripts/build_piper_baseline.py: build_place_reference_poses(root, baseline,
  target_local_m, payload_half_m, pad_offset_m, clearance_m)` —— 在**臂基座系**里对承载面中心解出
  `above / descend / retreat` 三段关节解（同一套 IK；未收敛即显式失败）。
- 构建器：`reference_solver.place_entry` 声明 → 构建期按接收体**名义**位姿算出局部承载面中心并调用 →
  写进联合报告 `gripper.place_{above,descend,retreat}_positions` + `manipulation.reference_pose_resolution.place_reference`
  （`target_local_m = [0.318198052, -0.318198052, 0.355372]`、`payload_half_m = 0.025`、
  `pad_offset_m = 0.028931693`、`clearance_m = 0.04`、IK 残差 5.7e-06 / 9.7e-06 / 9.8e-06）。
- schema：`reference_solver.place_entry` 入名单；后端解析层保留三个 `place_*_positions` 键
  （第一版被过滤掉 ⇒ 运行时报"缺少放置段关节解"，是 fail-closed 生效）。
- 后端：`place_object` 改为**回放**这三段（`_move_trajectory`），不再运行时现解 IK；删除未用的 `_solve_and_move`。

**FK 对账（决定性）**：把报告里的两段关节解**原样**设进**联合模型**，指腹中点落在
`above = (0.450001, -4e-06, 0.449299)`、`descend = (0.449999, -1e-06, 0.409313)`
—— 与目标（承载面 0.355 + 0.0939 / + 0.0539）**逐位吻合** ⇒ **求解器与回放链路无误**。

**运行时仍失败，但原因已换**：`after_above` 时指腹停在 `(0.162255, -0.10067, 0.444723)`、
托盘位姿被推走 `(0.416018, 0.001198) → (0.381773, 0.069152)` ⇒ 从抓取位形到"托盘上方"的**单条关节空间
五次插值**在笛卡尔空间划出的弧线**穿过了载体**（四足在 (0.42, 0.008) 附近）⇒ 撞上、臂被挡住、狗被推走。
⇒ **下一步（明确）**：给求解器再加一个**绕行航点 `transit`**（在**抓取点正上方、托盘高度**处），
后端按 `lift → transit → above → descend → 释放 → retreat` 五段回放；并可在构建期用联合模型做
"每段位形不与载体接触"的 FK 自检（航点是关节解 ⇒ 可 FK，这正是构建期求解的优势）。

### 11.16 夹持能力工装：**五次工装缺陷** + 一条结构性结论（2026-09-28）

工装：`build/iraf-a6a14/grip_capacity_probe.py`（扫闭爪指令 → 量夹持力 → 抬升到脱落）。

**我在这份工装里连续踩了 5 个缺陷，每个都让数字变得不可信（必须记下来）**
1. **状态残留**：复用同一个 `MjData` ⇒ 上一轮把方块推走的扰动带进下一轮（同一闭爪指令两次给出不同结果）；
2. **瞬移**：每级直接把 `qpos` 跳到目标（20~50 mm/级）⇒ 无穷加速度，方块当然被留下；
3. **改瞬移后仍瞬移**：`solve_position_ik` **就地改写 `data.qpos`**，我又把这份"已移动过的状态"当成轨迹起点 ⇒ 起点=目标；
4. 早前同类的两次：`geom_size` 当接触面位置（§11.15）、静态保持不设夹爪关节（§11.6 更正）。
⇒ 纪律：**量之前先证明工装本身是受控平滑运动、且每次测量从干净状态开始**；否则数字再漂亮也是伪像。

**可信的数字（工装修好后）**
- 夹持力随闭爪指令单调上升，且**全部有接触**：`0.017 → 1.7955 N`、`0.018 → 1.5759`、`0.019 → 1.3513`、
  `0.020 → 1.1302`、`0.021 → 0.9073`、`0.022 → 0.6845`、`0.023 → 0.4574`（方块重量 0.39 N）
  —— 且**闭爪指令 ≥0.024 时完全没有接触**（说明方块在闭爪位的临界值就在 0.023~0.024 之间，
  当前声明值 0.023 已是可行的**最松**端）。
- 抬升过程中**夹爪姿态几乎不变**（轴从 `[0.999306, 4e-05, 0.037262]` 到 `[0.999664, 9e-05, 0.025940]`，
  差 1~2°）⇒ "姿态漂移把方块剥出去"这条假设**不成立**。
- 但抬升到 20~150 mm 就丢件 —— 与**已验证事实矛盾**（pick 自己的 `lift` 段抬 88 mm 稳过）
  ⇒ 结论：**矛盾源在工装的运动方式/IK 分支，而不是"物理上搬不动"**。

**结构性结论（下一步的方向，不再用运行时位置型 IK 做搬运）**
已验证的 pick 之所以能抬 88 mm，是因为它的位形来自**构建期求解器**给出的**关节空间解**
（`grasp_positions → lift_positions`，由声明式求解器按目标几何解出），后端只是 `_move_trajectory` 回放。
而我在 `place_object` 里用的是**运行时位置型 IK**（`pad_mid + Δ` 现解）⇒ 解可能落在别的分支、
轨迹中途把方块打掉。⇒ 正确做法是把 s04 也做成**同一条路**：
`reference_solver` 增加 place 入口，构建期解出
`place_lift / place_transit / place_descend / place_release / place_retreat` 的**关节空间**解
（用与 pick 同一套 IK + 目标几何），后端只回放；判据仍是 §11.10 的三条（偏移量 + released + payload_in_tray）。
这也正是 §11.10 一开始的设计 —— 我为了省事走运行时 IK 的捷径是错的，现在有数字支撑回到原设计。

### 11.15 闭爪位"改成贴合"的假设**被实验否掉**（2026-09-28）—— 顺带修掉一处我自己提交进来的声明不一致

**假设与推翻过程（都留痕）**
1. 由 §11.14 推断"过盈 4.75 mm/侧 ⇒ 位置伺服把方块挤出去"。为验证，用 FK 扫关节值求"指腹内表面间距"
   （指腹 geom 半厚 0.012950739858219014 m）：`joint7=0.023 ⇒ 0.040463`、`0.0275 ⇒ 0.049462`、
   `0.027768 ⇒ 0.049998`（正好 = 方块宽 0.05）⇒ 取 **0.0275**（每侧 0.5 mm 过盈）改声明。
2. 跑验收：**臂自己场景的 pick 直接回归红** —— `s03 FAILED / Backend 未确认目标已抓取`
   （单本体 nominal exit 5，抓取位移 37.04 s 后判定未抓住）；联合世界 s04 也报
   `场景报告没有接收体: tray_01`（因为那时报告被单本体路径覆盖）。
3. ⇒ 结论：**由 `geom_size` 推"内表面间距"是无效的** —— 指腹是 **mesh geom**，`geom_size` 是包围盒，
   不等于接触面位置。0.0275 实际很可能**没有夹住**（而不是"夹得更贴合"）。
   **纪律**：闭爪位只能靠**测接触力 / 可搬运高度**来定，不能靠几何推断（这条写进了基线的注释里防重走）。

**回退与验证**：闭爪位回退 0.023/−0.023，两侧重建 ⇒ 臂侧验收**逐位回绿**：
`s03 SUCCEEDED  grasp_center_distance_m = 9.387527248483805e-06 / lift 0.088446 / 双侧 1.0`（与改动前完全一致）。

**顺手修掉的不一致**：我在做摩擦修复时，把当时"临时启用"的 `place_object` 一并提交进了
`profiles/piper_mujoco.yaml` 的能力列表，而 `factory.py` 的能力映射（同一个改动的一部分）却被回退了
⇒ 装配期报 `Backend 未实现 RobotProfile 声明的能力: place_object(需 <未登记的 capability>)`。
已从 profile 移除。**教训**：临时的"启用-回退"必须逐文件核对，别把临时声明混进别的提交里。

**当前 s04 状态**：搬运丢件**未解决**，机制仍未判死（过盈挤出的假设只被间接支持、且"改成贴合"这条路被否）。
下一步建议先做**接触力/可搬运高度的实测工装**：在给定闭爪位下量"指腹↔方块法向力"与"竖直抬升到多少脱落"，
拿到数字再定闭爪位或改搬运策略；不要在几何推断上再花一轮。

### 11.14 搬运丢件的真正原因：**夹持是"过盈挤压"而非贴合夹持**（2026-09-28 判死）

分段航点（竖直抬升 → 横移 → 下降）与"按载荷最低点算间隙"都落地后，**竖直抬升一段单独就丢件**，
于是排除了"路径穿越载体"这条假设（那只是发散的表象）。随后逐项量：

| 量 | 实测 |
| --- | --- |
| 夹爪指令 | `joint7/8 ctrl = 0.023/−0.023`（闭爪），**两段之间未被复位**（start 与 after_lift 完全一致） |
| 夹爪状态 | `qpos = 0.023/−0.023`（已到位），`commanded_closed` 一致 ✓ |
| 闭爪位**指腹中心间距** | **0.066364 m**（联合模型与臂自己场景**逐位相同** ⇒ 这里没有厂商值残留） |
| 指腹 geom 半厚 | 0.01295 m ⇒ 内表面间距 ≈ **0.0405 m** |
| 方块宽 | **0.05 m** ⇒ 每侧约 **4.75 mm 过盈** |
| 摩擦容量 vs 重量 | kp(200)×4.75mm ≈ 0.95 N/侧 × μ=2 ⇒ ~3.8 N ≫ 0.4 N（方块 0.04 kg） |

⇒ **不是打滑**：摩擦容量比重量大一个量级。竖直抬升 0.30 m（peak 加速度 ≈0.43 m/s²，需 ~0.017 N）
也远在容量内。综合"pads 内表面比方块窄 4.75 mm/侧 + 抬升中丢件 + 拾取段自带 0.08 m 抬升能过"，
最自洽的机制是**过盈把方块挤/滚出夹口**（接触面不严格平行时，任何位移都会给一个轴向分力），
而不是重力滑脱。拾取段只抬 0.08 m、几乎无横移 ⇒ 侥幸没暴露。

**修法候选（都属于"声明/几何"层，不写实现层默认值）**
- **甲（推荐）**：闭爪位按**方块实际宽度**重新声明（使内表面间距略小于方块宽、例如 0.048 m），
  让夹持变成"贴合夹持"而不是"过盈挤压"；改的是 `config/piper_simulation_baseline.yaml` 的
  `gripper.closed`（两个机型共用同一份声明 ⇒ 臂侧验收需重跑，属正常代价）。
- **乙**：搬运期间用声明的**约束**（如 `gravity_fixture` 那种 equality）保持载荷，落地前解除；
  代价是"搬运"这段时间的物理保真度下降（必须显式声明为仿真专有，不得表述为真机能力）。
- **丙**：分段内**重新夹紧**（每段前重发闭爪并停一小段），代价是时长与"看起来一顿一顿"。

**纪律**：能力四处声明与 s04 启用**再次回退** ⇒ s04 仍 `SKIPPED_PENDING`、`nominal --world joint`
exit 0、`scene_check` exit 0。本轮改动（分段航点 + 按载荷最低点算间隙 + 夹爪状态证据）全部 inert。

### 11.13 s04 端到端接线完成但**未通过**：三段卡点逐个暴露（2026-09-28）

**已接线（代码层，能力**未**声明 ⇒ inert）**

- `scripts/scenario.py: build_backend_config` 透传 `place_targets` 与 `targets[].geom`；
- 构建器把臂侧**已声明**的 `reference_poses.pregrasp_offset_m`（0.04）写进联合报告 gripper
  ⇒ 后端解析出 `pregrasp_offset_m`（放置的接近/抬离间隙，不新造数字）；
- 后端 `_parse_place_targets`（id → body/geom/size_m/mount）与 `place_object` 四段实现；
- runner 判据表与测量量表：`max_offset_from_tray_center_m` / `require_release` / `require_payload_in_tray`
  （测量量只来自技能证据）；`config/scene.schema.json` 的判据名单同步；
- `skills/place_object/skill.yaml`：**删掉**前置状态 `manipulation.payload_held == true`
  —— 实测运行期报「缺少运行时状态: manipulation.payload_held」：**声明白不存在的东西是假声明**，
  改为后端入口用**实测事实**把关（"双侧指腹必须同时接触载荷，否则拒绝放置"）。

**临时启用 s04 后的三次实测（每次推掉一层卡点）**

| 次 | s04 结果 | 报错 | 判读 |
| --- | --- | --- | --- |
| 1 | FAILED（0.0005 s） | `缺少运行时状态: manipulation.payload_held` | 前置状态是**系统不发布**的（已改实测事实） |
| 2 | FAILED（0.0007 s） | `载荷 box_01 未声明 geom（无法量最低点）` | `geom` 没进后端配置（已透传） |
| 3 | FAILED（5.66 s） | `放置 IK 未收敛（descend）：error=0.617962704 m target=[0.416039, 0.00061, 0.728197]` | approach 段**已收敛**，下行段目标不可达 ⇒ 见下 |

第 3 次的数字判读（关键）：approach 结束后 `pad_mid` ≈ 0.45，而**同一时刻**量到的载荷最低点 ≈ **0.079**
（≈ pick 的提起高度 0.079437）⇒ 两者相差 ~0.37 m：**方块没有被夹爪带走**（或两次快照量的不是同一个物体）。
我的下行目标 = `pad_mid + (托盘顶面 − 载荷最低点)` = 0.45 + 0.276 = 0.728 —— 公式本身自洽（把"载荷最低点"
抬到承载面），但前提"载荷跟着指腹走"在实测里不成立。

**下一步诊断（一次即可判死）**：在每个相位打印三个快照量（托盘顶面 / 载荷最低点 / 指腹中点）与该相位结束时
"指腹↔载荷接触对数量"，看方块是在哪一段脱离的（候选：approach 段的位移过大导致摩擦失效；
或 pick 收尾的 hold 未保持闭爪导致开度回弹）。

**纪律**：能力四处声明（profile / 场景 capabilities / 策略 allowed_skills / 适配器 CAPABILITY_METHODS）
**已回退**，s04 保持 `SKIPPED_PENDING`，`nominal --world joint` 保持 exit 0；
`scene_check` exit 0。实现与判据表留着（未声明 ⇒ 不会被派发），修好并验收后再声明。

#### 11.13 续：逐相位快照判死"方块在接近段脱离"，并又抓到一处**同源的厂商值残留**（2026-09-28）

**快照（`IRAF_DEBUG_PLACE=1`，每相位打印托盘顶面 / 载荷最低点 / 指腹中点 / 双侧接触）**

```
start          : tray_top=[0.415942, 0.001196, 0.342755]  payload_low=0.078708  pad_mid=[0.280154,-0.280124,0.133531]  左/右接触=true/true
after_approach : tray_top=[26.765592, 13.485351, -8959.031265]  payload_low=-0.000216  pad_mid=[0.420654,0.003328,0.378248]  左/右=false/false
```

三条判读：
1. **进入放置段时确实夹着**（双侧接触 true、载荷最低点 0.078708 ≈ pick 提起高度）——放置的前提成立；
2. **接近段结束时方块掉回台面**（payload_low −0.000216、双侧接触 false）⇒ 搬运过程中滑脱；
3. 更严重：`after_approach` 的**托盘位姿已经发散**（z = −8959 m）⇒ 仿真在接近段就崩了，
   这一步的后续（下行 IK 目标 = `[0.420654, 0.003328, -8962.841969]`、误差 8962.96 m）全是发散的产物。

**本轮抓到的同源残留（已修，已验证）**：联合模型的**指腹摩擦**用的是厂商 MJCF 值
`[1.0, 0.005, 0.0001]`，而臂自己场景用声明值 `[2.0, 0.05, 0.001]`（`scene.finger_friction`）
⇒ 只有一半摩擦、十分之一滑动摩擦。修法与前次的执行器刚度同构：Profile 声明
`spec.model.finger_friction = {baseline, section, key}`，构建器在**命名指腹 geom 的位置**按声明注入。
实测（重建后联合模型）：`piper_left/right_finger friction = [2.0, 0.05, 0.001]` ✓。
注入后拾取仍绿（0.00021017361402818382 / 0.079414 / 双侧 1.0），但搬运段**发散**。

**下一步的两个候选（都有明确判据，不是猜）**
- **① 搬运路径**：现在的接近段是"从抓取点直线上举+横移到托盘上方"（0.6 m），中途要穿过
  载体（四足在 (0.42, 0.008, 0.29) 附近、托盘挂在它背上）附近的空间 ⇒ 臂/方块与载体发生深穿透、
  高摩擦下求解器发散。修法：**分段航点**（先竖直抬到载体上方高度 → 再横移 → 再竖直下降），
  并在每段前做一次"臂 geom ↔ 载体 geom"的**间隙检查**（与 §11.7 的侵入自检同一口径）。
- **② 夹持保持**：即使路径改好，0.6 m 搬运仍靠摩擦。判据：每段结束时"双侧指腹仍接触载荷"
  （`phase_trace` 已给出该字段）；不成立即显式失败（现在就是这么做的）。

**纪律**：能力四处声明与 s04 启用**再次回退** ⇒ s04 仍 `SKIPPED_PENDING`、`nominal --world joint`
保持 exit 0、`scene_check` exit 0。（实现 + 判据表 + 逐相位证据留着，全部 inert。）

### 11.12 接收体声明进报告（`place_targets`）：托盘的名义停靠位姿与可达性（2026-09-28）

`place_object` 只能按**名字**引用接收体（托盘随载体运动 ⇒ 预写世界位姿必过期），但放置点仍需一个基准。
联合报告新增 `place_targets`（构建期），字段与数字：

```json
{"targets": [{"id": "tray_01", "body": "tray_01", "geom": "tray_01_geom",
              "mount": {"frame": "tray_frame", "entity": "unitree_go2"},
              "mount_offset_m": [0.0, 0.0, 0.057], "size_m": [0.12, 0.08, 0.01],
              "pose_source": "nominal_docked_station", "runtime_pose_source": "live_fk",
              "nominal_pose_m": [0.45, 0.0, 0.345372], "nominal_quaternion_wxyz": [1,0,0,0]}],
 "station_frame": "handoff_station_frame", "station_pose_m": [0.45, 0, 0],
 "nominal_base_height_m": 0.288372}
```

两条判读：
1. **`nominal_pose_m` 的 z 有一个坑**：站位的声明是 (0.45, 0, 0)，其中 z 是**地面**高度；载体停在站位时
   机身并不在地面 ⇒ 名义基座高度必须取**关键帧实测值 0.288372 m**（模型 FK，不手写数字），
   修正前会得到 z = 0.057（明显不对，托盘不可能在地面附近）。修正后 **0.345372 m** 与设计里
   "托盘顶面高度 z ≈ 0.347 m"一致（§11.1 的可达环带判据也是按这个高度做的）。
2. **可达性**：托盘名义位姿距臂基座 (0.45, −0.45, 0.123) = **0.501945521 m ≤ 0.594284 m**（可达上界）
   ⇒ 臂够得到托盘，`place_object` 在几何上可行（这一条以前没有数字，属于"装配前必须先有的判据"）。

`nominal_*` 只作**构建基准**；运行期必须以**实测**位姿为准（`runtime_pose_source: live_fk`），
证据里给实测偏移（见 §11.10 的 A 案判据 `max_offset_from_tray_center_m`）。

### 11.11 停靠误差分布已量：**独立跑确定性**，但**带负载时会漂**（2026-09-28）

探针 `build/iraf-a6a14/dock_error_distribution_probe.py --runs 5`（同配置、每轮重新装配、
先 `stand` 8000 ms 再 `dock_for_handoff`，只读技能证据）⇒ **5/5 逐位相同**：

```
transport_error 0.028587600087094413 m   yaw -0.6011884640079965°   base [0.42263, 0.008256, 0.292018]
spread = 0.0（min = median = max）
```

把所有已测数字并排（同一验收口径：`/home/coretek/AIIRAF/build/acceptance/handoff_lab/nominal/report.json`）：

| 路径 | 负载 | 平移误差 (m) | 偏航 (°) | 结论 |
| --- | --- | --- | --- | --- |
| 探针（独立跑 dock） | 无 | **0.028587600087094413**（5/5 同值） | −0.6011884640079965 | 控制器本身**确定性** |
| runner `nominal`（joint） | 无 | 0.028125012624229208 / 0.02812501214102655 | +0.5801787093348428 | runner 路径也**确定性**（两次差 5e-10） |
| 演示脚本 | 带窗口 | 0.028375962824781182 | −0.4095659107671862 | 开始漂 |
| `run --display` | 带窗口 | **0.03056883116091061（超 0.03 上限）** | +0.22596144723361478 | 漂到越界 |

三条判读：
1. **不是"控制器随机"**：独立跑 5 次逐位相同 ⇒ 方差来自**运行路径/机器负载**（四足行走由
   `iraf_adapters.unitree.mpc.worker` **独立进程**规划，其节拍与墙钟相关），不是数值噪声。
2. **runner 路径与独立探针路径差 0.46 mm**（0.02812501214102655 vs 0.028587600087094413）——
   同一条 dock 技能、不同调用上下文 ⇒ 差在步边界/推进节奏；**两个值都必须按各自路径引用**。
3. ⇒ 声明上限 0.03 的余量实际只有 **1.4 ~ 1.9 mm**（不加载时），**加载后会被吃掉**。
   所以"带窗口演示"和"验收"不能混为一谈，且**验收本身没有安全余量**。

**候选修法（两条，代价不同；本轮只登记，不擅自改声明阈值）**

- **甲（推荐，控制器侧）**：让停靠的**终端收敛段与墙钟解耦**（按仿真时间节拍收敛/稳定判据），
  这样负载只影响墙钟不影响结果 —— 与"实时路径必须本机、确定、有界"（AGENTS.md 1.4）同向。
  代价：改 `dock_for_handoff` 的终端段 + 重跑单本体与联合世界两侧验收（适配器 9/9 + 技能层）。
- **乙（阈值侧）**：重新论证 `dock_translation_error_max_m: 0.03` 的取值依据（设计 §4）。
  可按**下游真实需求**推：交接真正要的是"臂够得到托盘且方块能落进托盘"——托盘声明 0.24 × 0.16 × 0.02 m，
  方块 0.05 m ⇒ 停靠误差 ≤ 0.055 m 仍可保证方块完全落入（见 §11.10 的 A 案）。
  代价：放宽的是**验收阈值**，必须由使用者/架构确认（不擅自改）。
- 两条都不依赖对方：s04 的判据（托盘承载面内）对停靠余量**不敏感** ⇒ 可以先推进 s04。

#### 11.11 更正二（2026-09-28 晚）：真因是**驻留与 owner 自己步骤的竞态**，不是机器负载

加压实测把"负载敏感"这条判断否掉了：给探针加 **4 个忙循环进程**（同配置、2 次），dock 仍**逐位相同**
`0.028587600087094413` —— 负载不改变结果。

真因（读代码 + 复跑判死）：`_start_plant_residency` 的驻留线程**盲目地**反复执行 owner 的 `hold` 技能
（`stand`），**不知道 owner 正在跑自己的场景步骤** ⇒ 联合世界的 `s02_dock` 由 owner 自己执行时，
**两条执行流并发驱动同一株植物、写同一批执行器**，结果取决于线程调度：

| 条件 | dock 平移误差 (m) |
| --- | --- |
| 探针（**无驻留**，7 次含加压 2 次） | 0.028587600087094413（逐位相同） |
| runner（有驻留，无窗口，修复前） | 0.028125012624229208 / 0.02812501214102655 |
| runner（有驻留，带窗口，修复前） | 0.028375962824781182 / **0.03056883116091061（越界）** |

**修法（已实施）**：owner **自己**执行场景步骤时让驻留**让位** ——
`_yield_residency_to_step`（置 `pause_event`，等驻留声明 `idle_event`；超 60 s 即显式失败，不静默目送）
在步骤前后成对使用，理由与判据都写进 docstring；步骤记录里新增 `plant_residency_yielded`，
驻留汇总里新增 `paused_steps` / `paused_seconds`（审计"哪几步让位了"）。

**修复后实测（同一命令三次：两次无窗口 + 一次带窗口）**：

```
无窗口 run1  passed=true  dock=0.028569272841549035  pick=0.00021011614641799212
无窗口 run2  passed=true  dock=0.028569272841549035  pick=0.00021012876165914785
带窗口 run3  passed=true  dock=0.028569272841549035  pick=0.00020988541507060214  display frames=1739
             plant_residency.paused_steps = ['s01_verify_ready', 's02_dock']（两次也是同样两项）
```

⇒ ① dock 现在**逐位可复现**，且**与是否开窗口无关**（0.03056883116091061 的越界不再出现）；
② 抓取稳定在 0.00020988 ~ 0.00021013 m；
③ 余量问题仍在但性质变了：0.028569272841549035 vs 声明上限 0.03 ⇒ **1.43 mm 的确定性余量**
（原先是"1.4~1.9 mm 且会被负载吃掉"）。是否要给停靠终端段再加余量（甲案）或重论证阈值（乙案）
见 §11.11 的候选修法 —— 不再是"随机越界"的紧急项。

### 11.10 `place_object`（s04）开工：契约已落；发现一处**声明冲突**必须先定案（2026-09-28）

**已落（契约先于实现，AGENTS.md 2.1；尚未声明任何能力 ⇒ 不构成假声明）**

- `skills/place_object/place_object.input.json`：必填 `place_target_id`（**只给名字，不给世界坐标**：
  托盘随载体运动，任何预写的世界位姿都会过期）+ `payload_id`（手里拿的是什么，不让实现层猜）；
  可选 `duration_ms`。
- `skills/place_object/place_object.output.json`：确认口径**只有 `released`**，且证据必须同时给出
  `released` 与 **`payload_in_tray`**（实测载荷落在接收体上）—— 只有"夹爪开了"不算放下。
- `skills/place_object/skill.yaml`：`requires: [place_object]`、`timeoutSeconds: 120`（覆盖墙钟，
  与 pick 同口径）、`safetyClass: controlled_motion`、
  provider `iraf_skills.common.manipulation:PlaceObjectProvider`。
- `src/iraf_skills/common/manipulation.py`：新增 `PlaceObjectProvider`（后端未实现 `place_object`、
  或证据显示载荷不在托盘上 ⇒ 直接 `SkillRejected`，不伪造成功）。

**发现的声明冲突（必须先定案，否则 s04 的判据无法自洽）**

1. 托盘是**挂在四足背上的**（`props[tray_01].mount = {entity: unitree_go2, frame: tray_frame}`）⇒
   它的世界位姿**取决于狗停靠后的实际位姿**，构建期解出的关节解只能按**标称站位**算。
2. 实测停靠误差 **0.02812501214102655 m ≤ 0.03（声明）**，而 `scenario.yaml` 里 s04 现写的判据是
   `pose_tolerance_m: 0.01` —— 即"要求放置精度比载体定位精度高一个量级"，**不可能同时成立**。
3. 两个可选定案（都要写进声明，不许实现层默认）：
   - **A（建议）**：s04 判据改为"载荷落在托盘**承载面范围内**"——托盘声明尺寸 0.24 × 0.16 × 0.02 m、
     方块 0.05 m，方块完全落入托盘只需 |dx| ≤ 0.095、|dy| ≤ 0.055 ⇒ 取 `max_offset_from_tray_center_m: 0.06`
     （由声明尺寸推出并写进注释），并把实测偏移量写进证据（可见、不隐藏）；
   - B：把停靠精度收到 ≤ 0.01 m —— 需改 `dock_for_handoff` 的验收与控制器，代价大且与"四足定位"物理不符。
4. 落地顺序（下一轮）：① 定案 A 的阈值进 `scenario.yaml` + `scenario.py` 的判据表；
   ② 构建期在联合报告里给 `place_targets[]`（托盘 body/geom/半尺寸/挂载 + 标称位姿 FK）与
   由**已声明的参考姿态求解器**解出的 `place_*_positions`（approach/descend/release/retreat，
   与 pick 的 `*_positions` 同口径）；③ 后端 `place_object` 用工位契约的现有原语
   （`_move_trajectory` / `_set_gripper_controls` / `_advance_for` / `_joint_qpos`）执行四段，
   证据给 `place_alignment.center_distance_m`（相对**运行期实测**的托盘位姿）、`released`、
   `payload_in_tray`、`retreat_delta_m`；④ 能力声明（profile / 场景 / 适配器 / provider 四处同步）
   + 成功与拒绝两条路径的回归测试；⑤ `nominal --world joint` 判 s04，再补 s05 载荷确认。

## 12. 下一步

0. **（2026-09-28，§11.9）** 给 `scripts/scenario.py run` 加显示通路（`--display/--render-hz/--seconds`）：驻留线程推进 + `continue_stepping=False` 的只渲染会话，让**验收运行本身**（stand → dock → pick，exit 0/passed=true）可被看到。
0a. **（2026-09-24 判死，§11.7）** 求解器层：参考姿态必须**不得让臂 link 侵入目标**（当前 `piper_link6` 与方块重叠 −0.014516 m ⇒ 保持残余 0.039962049 rad）；可复用 UR5e `GraspPoseSolver` 的 `pointing_direction`：把夹爪轴约束到**声明的** `grasp.approach_direction`（§11.7 附：抬高抓取点已被数字否掉 —— 门禁口径不允许，且抬 28 mm 侵入仍为负）。修完再声明 `feedforward_entry` 并判 s03。
0b. **（2026-09-24 新增）** 附加本体的执行器刚度口径声明化（联合模型里的臂必须与臂自己场景同一口径：Profile 声明 → 构建器按声明注入 `kp = max(arm_position_kp, 关节阻尼×ratio)`）；验收判据 = §11.6 的静态保持残余表；通过后再把 `feedforward_entry` 加回 `reference_solver`。
1. `s03_pick` 三项判据口径（`pose_tolerance_m` / `min_lift_delta_m` / `require_bilateral_contact`）
   → s04/s05 → `nominal` 全场景 → 两臂轮番运输。
2. 联合产物的资产相对化（消 host-specific）；债 S4（臂厂商资产入库 + 重写锁 + 许可证 BOM）；S3。
3. owner 侧作用域的**声明化**（现在 owner = 模型全部执行器；联合植物里应显式声明"我拥有哪些"）。


