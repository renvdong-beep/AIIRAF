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

