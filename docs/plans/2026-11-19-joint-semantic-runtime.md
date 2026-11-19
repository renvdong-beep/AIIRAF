# 联合世界语义编排 + 无板卡运行（2026-11-19）

- 状态：已批准，进入实施
- 目标节点：x86 `.145`（无板卡也可运行）
- 关联：`docs/plans/2026-11-18-joint-scene-composer.md`（P1–P4 已完成）

## 1. 目标

1. **无板卡本地运行**：不依赖 `.147` 的 MCP/AgentOS，也能语义编排 → 构建 → MuJoCo 运行；
2. **常用物品库**：方块/圆柱/杯子/球/托盘等声明式 prop，可被场景组合选用；
3. **G1 人形接入**：Profile + 场景 + 适配器（至少静态/站立），可加入联合世界；
4. **Franka + 2F-85**：把 UR5 的 Robotiq 2F-85 装配到 Franka；
5. **语义组合 + MuJoCo 运行**：自然语言 → 机器人/物品集合 → 联合场景 → 构建 → 运行 → 验收。

## 2. 架构（不新增框架层）

| 复用 | 新增/扩展 |
|---|---|
| `joint_compose.py`（Profile 并集/场景声明） | **catalog 模式**：跨场景收集机器人条目，按 id 组合 |
| `scene_builder`（MJCF 合成/注入） | props 库读取与注入 |
| `scenario.py run`（进程内 MuJoCo 执行） | 语义入口生成最小 scenario |
| `profiles/*`、`config/scenes|providers` | G1/Franka 新声明 |
| `scripts/assemble_ur5e_2f85.py`（装配参考） | Franka+2F-85 装配脚本 |
| MCP `iraf_compose_joint_scene` | 增加 `props` / `run` 参数（薄封装） |

新增文件：

```
config/props/library.yaml                  # 常用物品模板
src/iraf_adapters/mujoco/joint_catalog.py  # 跨场景机器人目录 + prop 组合
scripts/joint_semantic.py                  # 语义入口（NL/参数 → 组合 → 构建 → 运行）
scripts/assemble_franka_2f85.py            # Franka + 2F-85 装配
profiles/g1_mujoco.yaml                    # G1（阶段 3）
scenes/g1_lab/                             # G1 单机场景（阶段 3）
tests/unit/test_joint_catalog.py           # catalog/props 契约
```

## 3. 阶段

### 阶段 1：无板卡本地运行（0.5 天）
- `scripts/joint_semantic.py`：
  - `--robots piper,go2 --props box:0.28,-0.28,0.025` 或 `--text "piper 和狗，桌上一个方块"`；
  - 生成 `scenes/<id>/{scene,baseline}.yaml` + `profiles/<id>.yaml`；
  - 调用 `build_scene.py` 生成 MJCF；
  - `--run stand`：用 `scenario.py run`（进程内，无需 provider/板卡）做冒烟；
- 不依赖 `.147`；`.145` 本地即可完成。

**验收 S1**：`.145` 上一条命令生成并运行一个新组合（piper+go2+box），无需板卡。

### 阶段 2：常用物品库（1 天）
- `config/props/library.yaml`：`box / cylinder / cup / ball / tray / button` 模板；
- `joint_catalog.py`：`resolve_props(text/params)` → 生成 `scene.props[]`；
- composer/CLI/MCP 接受 `props`；
- builder 已支持 `props` 注入，不修改物理实现。

**验收 S2**：同一场景用不同 prop（方块/杯子）生成并构建通过；报告里 prop 存在。

### 阶段 3：G1 接入（1.5 天）
- 从 `vendor/unitree_g1/g1_widefoot.xml` 提取 joints/limits/actuators；
- `profiles/g1_mujoco.yaml`：先声明 `static_model_only`（场景实体），再按适配器补 `stand/stop`；
- `scenes/g1_lab/` + `config/machines/g1.yaml`；
- composer catalog 可选中 g1。

**验收 S3**：G1 单机场景构建；联合场景中可作为实体/（若实现）站立。

### 阶段 4：Franka + 2F-85（2 天）
- `assemble_franka_2f85.py`：Franka 法兰 + 2F-85 模型、执行器/关节映射、质量/碰撞；
- 新 Profile（Franka 臂 + 2F-85 8 关节/驱动关节）、manipulation 事实（gripper/targets/参考解）；
- 单机 pick 报告；再进联合世界。

**验收 S4**：Franka+2F-85 单机 pick 通过；联合场景可装配。

### 阶段 5：语义组合 + MuJoCo 运行（1.5 天）
- `joint_semantic.py --text "用 piper 和狗，桌上放一个杯子"`：
  - 语义解析（机器人别名/物品别名/数量/位置）；
  - catalog 组合 → 构建 → 生成最小 scenario → `scenario.py run` 冒烟；
- MCP 工具 `iraf_compose_joint_scene` 增加 `text`/`props`/`run`；
- 回归 handoff 9 步与单机 provider。

**验收 S5**：一句话生成并运行新组合；日志含 MuJoCo 步进/动作证据；失败显式。

## 4. 验收矩阵

| 编号 | 判据 |
|---|---|
| S1 | 无板卡 `.145` 一条命令完成组合+构建+运行 |
| S2 | prop 库 ≥5 类，构建报告含对应 body |
| S3 | G1 单机构建通过（静态/站立按能力） |
| S4 | Franka+2F-85 单机 pick 通过 |
| S5 | 自然语言组合 → 构建 → MuJoCo 运行成功 |
| R1 | handoff nominal 9/9 不退化 |
| R2 | 单机 piper/ur5 不退化 |
| R3 | declared ⊆ implemented 装配期通过 |

## 5. 风险

| 风险 | 缓解 |
|---|---|
| 语义解析过度自由 | 只支持受控词表（机器人/物品别名），未知显式失败 |
| 跨场景机器人条目不一致 | catalog 以 `(scene, id)` 唯一，重名显式失败 |
| G1/Franka 事实缺失 | 分阶段：先静态/构建，再运动/抓取 |
| 2F-85 装配动力学不稳定 | 复用 ur5e 装配的 armature/接触参数；构建期自检 |
| 无板卡时 AgentOS 不可用 | CLI 先行；MCP 本地实例可选 |

## 6. 版本

- 阶段 1–2 → 0.4.9
- 阶段 3–5 → 0.5.0
- 每阶段独立提交 + 验收记录；失败可回滚到当前 `04169d5`。

## 执行记录（2026-11-19，阶段 1/2/5 起步）

- 无板卡：`.145` 上直接 `joint_semantic.py --robots piper,unitree_go2 --props "box@..." --build --run`；
- 生成场景 + 构建 + `scenario.py run stand_stop` 全链路通过：

```text
build/scenes/demo_lab/demo_lab_joint.xml  nq=34/nu=20，box_01 在模型里
build/acceptance/demo_lab/stand_stop/report.json
  passed=true
  ss01_stand SUCCEEDED
  ss02_stop  SUCCEEDED
```

- 修复：baseline.initial_state 需按所选机器人过滤；scene 包需 README.md（scene_check 契约）；
- 语义组合默认移除放置段（无接收体时），保留 pick/stand 冒烟。

## 执行记录（2026-11-19，阶段 1/2/5 起步）

- 无板卡：`.145` 上直接 `joint_semantic.py --robots piper,unitree_go2 --props "box@..." --build --run`；
- 生成场景 + 构建 + `scenario.py run stand_stop` 全链路通过：

```text
build/scenes/demo_lab/demo_lab_joint.xml  nq=34/nu=20，box_01 在模型里
build/acceptance/demo_lab/stand_stop/report.json
  passed=true
  ss01_stand SUCCEEDED
  ss02_stop  SUCCEEDED
```

- 修复：baseline.initial_state 需按所选机器人过滤；scene 包需 README.md（scene_check 契约）；
- 语义组合默认移除放置段（无接收体时），保留 pick/stand 冒烟。

## 阶段 3 记录：G1 单机静态接入（2026-11-19）

- `profiles/g1_mujoco.yaml`：29 个铰接关节/限位（`g1_widefoot.xml` 实测）+ `model.home`；
- `vendor/unitree_g1/source-lock.json`：99 个资产（vendor-source-lock/v1）；
- `scenes/g1_lab/`：humanoid + `static_model_only` + `evidence_level: 仅模型`；
- 构建实测：

```text
build_scene --scene scenes/g1_lab --robot g1
→ build/scenes/g1_lab/g1_lab.xml
   nq=36 / nv=35 / nu=29 / ncam=1 / nsite=4
```

- G1+go2 组合：`build_scene --attach g1` 被构建器的
  `使用 --attach 时必须在本体声明 manipulation_report` 拦下（builder 对所有附加本体要求
  manipulation 事实，暂不支持 `model_only` 静态附加）⇒ 这是下一步要解开的构建器限制，
  不是 G1 模型/Profile 问题。

### G1 联合附加的构建器限制（2026-11-19 复核）

- 给 G1 加 `manipulation_report` + dummy `reference_solver` 后，schema 通过；
- 构建仍要求报告里必须有 `gripper` 段（`_joint_manipulation`），而 G1 是 humanoid/静态实体，
  没有夹爪事实 ⇒ fail-closed；
- 正确修法：让 builder 对 `model_only: true` 的附加本体**跳过 manipulation 继承**
  （只附加几何/关节/执行器），而不是给静态实体伪造 gripper；
- 该项已登记为下一步，不在 G1 模型/Profile 侧硬凑。

### G1 进入联合世界（静态实体）已通过

- builder：当报告声明 `manipulation_absent_reason` 且无 gripper 时，`_joint_manipulation`
  **返回 absent**（静态实体只附加几何/关节/执行器，不伪造抓取事实）；
- 新增 `config/machines/g1.yaml`（guest，owner=unitree_go2）；
- `scenes/g1_go2_lab`：G1 + Go2 联合场景；
- 实测：

```text
build rc=0（g1_go2_lab_joint.xml）
scenario run stand_stop：
  build/acceptance/g1_go2_lab/stand_stop/report.json
  passed=true
  ss01_stand SUCCEEDED
  ss02_stop  SUCCEEDED
```

G1 当前是**静态实体**（`static_model_only`，无运动能力）；站立/行走仍需 G1 适配器与运动事实。
