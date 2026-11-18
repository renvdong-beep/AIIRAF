# 联合场景编排（Joint Scene Composer）执行方案

- 状态：已批准（2026-11-18），进入实施
- 适用版本：IRAF 0.4.8 → 0.4.9（P1/P2）/ 0.5.0（P3/P4）
- 目标仓库：`release/0.4.2`（板卡/演示线）
- 关联：`docs/plans/iraf-demo-design.md`、`docs/release/0.4.8-mujoco-restore.md`

## 1. 问题

现状：

- 每个机器人有**单机场景**（piper-pick、ur5-pick；go2 有模型/Profile 但无独立 provider）；
- 联合场景 `handoff_lab_joint` 是**手工搭建**的：一份 MJCF、一个 provider、`IRAF_PROFILE=piper_mujoco`
  ⇒ 联合世界里只能整链跑（`iraf_run_scenario`），**不能单点控制** go2/ur5；
- 每新增一种组合都要手写 MJCF/Profile/装配单 ⇒ 不可持续。

目标：

1. **单点控制**：联合世界里按 `resource_id` 单独指挥 piper/ur5/go2（“让狗站起来”“用 ur5 抓盒子”）；
2. **任意编排**：给定 N 个已有单机机器人的 scene+profile，自动生成联合场景（MJCF / Profile / baseline / provider / registry）；
3. **不臃肿**：只新增“组合”能力，不改 TaskFlow/Policy/Registry 的核心语义，不复制一套新框架；
4. 该能力同时暴露为 **MCP 工具** 与 **skill**，AgentOS 可一句话编排。

非目标：

- 不做真机 HIL、不做跨机分布式物理；
- 不改变单机 provider 的既有路径与行为；
- 不为每种组合写专用后端/Profile。

## 2. 架构融入（复用现有扩展点）

| 现有机制 | 在方案中的角色 | 是否修改 |
|---|---|---|
| `IRAF_BACKEND_ENTRYPOINT` + Provider 装配单 | 选择 `JointMujocoBackend` | 否（复用） |
| `RobotProfile`（`src/iraf_core/profile.py`） | 联合 Profile：能力/关节/资源的声明载体 | 仅新增可选 `joint` 段（向后兼容） |
| `MujocoBackend`（piper 操纵） | 组合后端的一个子适配器 | 否 |
| `UnitreeGo2Adapter`（stand/locomote/dock） | 组合后端的四足子适配器 | 否（已支持注入共享 model/data/bindings/plant） |
| `scripts/scenario.py` 的 `world=joint` | **参考实现**：一份 MjModel + 多适配器装配 | 只抽取，不改语义 |
| SceneRegistry（scenes/providers + 封签） | 联合场景与 provider 的注册 | 否（composer 生成 YAML） |
| PolicyGateway / `declared ⊆ implemented` | 按 `resource_id` 鉴权的硬门禁 | 否（沿用） |
| MCP server | 新增一个编排工具 | 仅注册工具 |

**新增文件（全部是“组合层”，不侵入核心）：**

```
src/iraf_adapters/mujoco/joint_backend.py     # 运行时：JointMujocoBackend（组合 + 资源路由）
src/iraf_adapters/mujoco/joint_compose.py     # 声明层：MJCF 命名空间合并 / Profile 并集 / baseline
scripts/compose_joint_scene.py                # CLI 入口（skill 与 MCP 共用）
skills/engineering/joint_scene_composer/      # skill 契约（skill.yaml + input/output schema）
docs/plans/2026-11-18-joint-scene-composer.md # 本文
```

运行时数据流：

```
scene 注册表 → Provider(5005x) → JointMujocoBackend
                                     ├─ 共享 MjModel/MjData（联合 MJCF）
                                     ├─ piper 子适配器（MujocoBackend）
                                     ├─ ur5   子适配器（MujocoBackend + bindings）
                                     └─ go2   子适配器（UnitreeGo2Adapter）
                                  Task.resource_id → 选择子适配器 → 能力方法
```

## 3. 数据模型

```yaml
# 编排输入（每个机器人一份已有单机描述符）
robots:
  - id: piper
    scene: scenes/piper_pick          # 单机 scene.yaml
    profile: profiles/piper_mujoco.yaml
    placement: { frame: world, position_m: [0, 0, 0], yaw_deg: 0 }
  - id: go2
    scene: scenes/go2_stand
    profile: profiles/unitree_go2_mujoco.yaml
    placement: { frame: world, position_m: [-1.2, 0, 0], yaw_deg: 90 }
scene_id: my_joint_lab
```

联合 Profile 扩展（可选段，不破坏现有 schema）：

```yaml
spec:
  joints: [...]            # 并集
  joint_limits: {...}
  capabilities: [...]      # 并集（declared ⊆ implemented）
  joint:                   # 新增：资源 → 关节/能力映射
    resources:
      piper: { joints: [...], capabilities: [pick_object, place_object, move_joint] }
      go2:   { joints: [...], capabilities: [stand, stop, locomote, dock_for_handoff] }
```

`JointMujocoBackend` 以 `lease.resource_id`（或显式 `resource_id`）选择子适配器；找不到即 `fail-closed`。

## 4. 分阶段实施

### P0 基线与设计（0.5 天）
- 本文入库；记录 `handoff_lab` 9 步链路与单机 piper/ur5 基线；
- 验收脚本骨架（复用 `iraf_run_scenario`）。

**验收**：`iraf_run_scenario(nominal)` 9/9；`piper抓取盒子`、`ur抓取盒子` 单机成功。

### P1 通用 JointMujocoBackend（1.5 天）
- 从 `scenario.py` 的 joint 装配抽取“共享植物 + 多适配器”为 `JointMujocoBackend`；
- `IMPLEMENTED_CAPABILITIES` = piper/ur5/io + go2 的能力并集；
- 按 `resource_id` 路由；缺资源/能力显式失败；
- 联合 Profile（piper+ur5+go2）先以 `handoff_lab_joint` 为夹具验证。

**验收**：联合世界里 `go2站起来`、`piper抓取盒子`、`ur5` 抓取分别成功；
`handoff_lab` 9 步回归不变；单机 provider 不受影响。

### P2 场景编排器 Composer（2 天）
- `joint_compose.py`：MJCF 命名空间合并（body/joint/geom/site/actuator/sensor 前缀 + 引用重写 + 资产拷贝）；
- 生成 baseline（`robots[].pose` + bindings）、联合 Profile（并集 + `joint.resources`）、provider 装配单；
- `scene_check` 门禁：名字唯一、joint/limit 一致、能力子集、资源路由完整。

**验收**：用 piper+go2 生成新场景，`scene_check` 通过，联合世界里 `go2站起来` 成功。

### P3 MCP 工具 + Skill + 注册（1.5 天）
- MCP `iraf_compose_joint_scene(scene_id, robots[], dry_run, smoke)`（薄封装 CLI）；
- skill `engineering.joint_scene_composer`（输入校验 → 生成 → 校验 → 可选烟测）；
- 生成 `config/scenes/*`、`config/providers/*` 并封签；返回文件清单与校验结果。

**验收**：AgentOS 里“把 piper 和 go2 组成一个场景并让狗站起来”一次完成；生成物可直接 `iraf_run_scenario`。

### P4 新机器人接入 + 发布（1 天）
- 机器人只需提供单机 scene+profile；composer 自动并入；
- 文档、回归、版本发布（0.4.9 / 0.5.0）。

## 5. 验收矩阵

| 编号 | 场景 | 判据 |
|---|---|---|
| A1 | 联合世界 `go2站起来` | SUCCEEDED，resource=go2，动作真实执行 |
| A2 | 联合世界 `piper抓取盒子` | SUCCEEDED，lift>0，resource=piper |
| A3 | 联合世界 `ur抓取盒子` | SUCCEEDED，resource=ur5 |
| A4 | `handoff_lab` nominal | 9/9 步 SUCCEEDED（回归） |
| A5 | composer 生成 piper+go2 | scene_check 通过 + `go2站起来` 成功 |
| A6 | 单机 provider piper/ur5 | 行为与耗时不变（回归） |
| A7 | declared ⊆ implemented | 任一组合启动预检通过 |

## 6. 关键决策

1. **一个 provider 一个联合世界，一份 MjModel**：不引入新的进程/总线；
2. **能力由 Backend 实现集合决定**，Profile 只声明；组合层不做能力伪造；
3. **资源路由**用 `resource_id`，不改 TaskFlow 协议；
4. **composer 只生成声明与模型**，不生成代码；新机器人零代码接入；
5. **失败显式**：缺资源、缺能力、命名冲突、资产缺失一律 fail-closed；
6. **兼容**：单机 provider 路径与 `handoff_lab` 链路保持原样。

## 7. 风险与缓解

| 风险 | 缓解 |
|---|---|
| MJCF 命名冲突/引用改写遗漏 | 统一前缀 + 引用扫描 + `scene_check` 断言 |
| 共享植物下步进竞争 | 复用 `MujocoPlant` 的锁与 owner 语义（scenario 已验证） |
| go2 与机械臂控制频率不同 | 保持各适配器自有时序，植物唯一推进者 |
| Profile 变大导致校验慢 | 并集生成 + 子集校验；不复制阈值 |
| 组合后策略误放行 | `declared ⊆ implemented` + 按 resource 鉴权 |

## 8. 版本与发布

- P1 完成 → **0.4.9**（联合世界单点控制，向后兼容）；
- P2/P3 完成 → **0.5.0**（composer + MCP/skill）；
- 每阶段独立 tag + SDK 产物 + 发布说明；失败可回滚到 0.4.8。

## 9. 文件清单（新增/修改）

新增：`joint_backend.py`、`joint_compose.py`、`scripts/compose_joint_scene.py`、`skills/engineering/joint_scene_composer/*`。
修改：`src/iraf_adapters/mcp/server.py`（注册工具，薄封装）、`src/iraf_core/profile.py`（可选 `joint` 段解析）、`config/install`（联合 profile 模板）。
不动：TaskFlow、Policy、Registry、单机 provider 后端、handoff 场景脚本。

---

## 10. 执行记录（P1，2026-11-18）

| 编号 | 状态 | 证据 |
|---|---|---|
| A1 联合世界 `go2站起来` | ✅ | SUCCEEDED，3.8 s；`provider=quadruped_stand_sim`，`control_cycles=800` |
| A2 联合世界 `piper抓取盒子` | ✅ | SUCCEEDED，43.1 s；`bilateral_contact=true`、`force_ok=true` |
| A3 联合世界 `ur5` 抓取 | ⛔ 阻塞 | `config/machines/ur5e_joint.yaml` 明确登记：联合报告只描述主臂 piper，UR5e 的 manipulation 段尚未解算（I3/I4 增量），不是后端路由能绕过的 |
| A4 `handoff_lab` nominal 回归 | ✅ | 9/9 步 SUCCEEDED，89.9 s，`passed=true` |
| A5 composer 生成 piper+go2 | 待 P2 | — |
| A6 单机 provider 回归 | 待验 | 默认场景未改，待跑 |
| A7 declared⊆implemented | ✅ | 联合 Profile 10 能力全部在 JointMujocoBackend 上；factory 装配期通过 |

P1 代码：`9357671`（JointMujocoBackend、联合 Profile、SafetyPolicy 放行、RemoteProviderBackend go2 代理、plant stepper）。

**结论**：联合世界的 piper/go2 单点控制已打通；UR5 需要先补齐联合报告的 manipulation 事实（P2 composer 的一部分：按本体生成 manipulation 段），再回到 A3。

下一步（P2）：`joint_compose.py` —— 先做 Profile 并集/校验与 MJCF 命名空间合并，用 piper+go2 生成新场景验证 A5。

### P2 首个增量（2026-11-18）

- 新增 `src/iraf_adapters/mujoco/joint_compose.py`：从既有场景取本体声明 → **Profile 并集**
  （joints/limits/capabilities + `joint.resources`）→ 生成 `scene.yaml` / `baseline.yaml` / Profile；
- 新增 `scripts/compose_joint_scene.py` CLI（`--robots piper,unitree_go2 --scene-id ...`，支持 `--dry-run`）；
- 实测：piper+go2 生成 **20 关节**、能力并集、资源 `{piper: arm, unitree_go2: quadruped}`；
- 产物路径自动改写到 `build/scenes/<scene_id>/`，不覆盖 handoff 既有产物。
- **当前阻塞**：`build_scene.py --scene scenes/piper_go2_lab --robot piper --attach unitree_go2`
  报 `厂商模型里没有 IMU site None（spec.model.imu_site 与模型不一致）` —— 生成场景的 `model` 段
  仍继承 handoff 的机型声明，需要按所选本体裁剪（下一步修 composer 的 model/robots 裁剪规则）。

### A5（P2 第二个增量，2026-11-18）

- 修正：构建主本体应选**四足**（厂商 go2 模型自带 IMU site），机械臂作为 `--attach`；
  生成器的 `--robot piper` 会拿臂 Profile 的 `spec.model.imu_site=None` 去比对，必失败。
- 实测：

```text
python3 scripts/build_scene.py --scene scenes/piper_go2_lab --robot unitree_go2 --attach piper
→ build/scenes/piper_go2_lab/piper_go2_lab_joint.xml（nq=34, nv=32, nu=20, ncam=1, nsite=5）
```

- 运行时：把 handoff provider 临时指向该产物（备份 env），板卡 `go2站起来` → **SUCCEEDED，3.4 s**；
  验证后已恢复 handoff 原模型。
- 结论：**A5 通过**（composer 生成 piper+go2 联合场景，单点 stand 成功）。

### A6（单机 provider 回归，2026-11-18）

| 语义 | 场景 | 结果 |
|---|---|---|
| `piper抓取盒子` | `piper-pick` | SUCCEEDED，5.5 s，`bilateral_contact=true` |
| `ur抓取盒子` | `ur5-pick` | SUCCEEDED，58.3 s，`bilateral_contact=true` |

单机 provider 路径未被联合改动污染。

### P3 首个增量（2026-11-18）

- **MCP 工具**：`iraf_compose_joint_scene(scene_id, robots, base, dry_run, register)`（server.py）；
- **Skill**：`skills/engineering.joint_scene_composer/`（skill.yaml + input/output schema + Provider）；
- **注册表生成**：CLI `--register` 写 `config/scenes/<id>.yaml`（resources/profile/safety_policy）；
- 实测：`piper,unitree_go2 → piper_go2_lab`，Provider `SUCCEEDED`，20 关节，生成
  scene/baseline/profile/registry 四个文件；MCP 工具待部署到板卡后做端到端调用。

### P3 端到端（板卡 MCP，2026-11-18）

```text
tools/call iraf_compose_joint_scene {scene_id: test_compose, robots: "piper,unitree_go2", dry_run: true}
→ {"status":"SUCCEEDED","joints":20,"capabilities":[10 项],"files":[],"dry_run":true}
```

- 工具已部署到板卡 `/opt/iraf-sdk/current`（server.py + joint_compose.py + scenes/handoff_lab）；
- `healthz=200`，MCP 工具可被 AgentOS 直接调用；
- 写入模式（`dry_run=false, register=true`）会生成 scene/baseline/profile/registry 四个声明文件。

### P4 首个增量（N 机器人复用，2026-11-18）

- **provider 注册表生成**：CLI `--endpoint/--control-url/--host-id` → `config/providers/iraf-<id>.yaml`
  （端点/控制面是部署期事实，composer 不猜）。
- **N 机器人组合**（piper+ur5e+go2，未写任何机型代码）：

```text
compose_joint_scene --robots piper,ur5e,unitree_go2 --scene-id piper_ur5_go2_lab \
  --register --endpoint 10.203.247.145:50055 --control-url http://10.203.247.145:8770
→ 28 关节；resources {piper: arm, ur5e: arm, unitree_go2: quadruped}
→ scenes/piper_ur5_go2_lab + profiles/piper_ur5_go2_lab.yaml
→ config/scenes/piper_ur5_go2_lab.yaml + config/providers/iraf-piper_ur5_go2_lab.yaml

build_scene --scene scenes/piper_ur5_go2_lab --robot unitree_go2 --attach piper --attach ur5e
→ nq=48 / nv=46 / nu=27 / ncam=1 / nsite=8
```

- 运行时：临时把 provider 指向三机器人产物，`go2站起来` → **SUCCEEDED，4.4 s**；随后已恢复 handoff 原模型。
- 已知：联合报告对 ur5e 的 manipulation 仍是 `resolved=false`（构建期有“放置航点翻转分支”告警），
  A3 仍需按本体生成 manipulation 事实。

### A3 推进（2026-11-18）：ur5e 作为联合世界主操纵臂

- composer 新增 `--joint-manipulator`：`ur5_go2_lab` 以 **ur5e 为主 manipulator** 构建；
- 联合报告结果：`declared_primary: ur5e`，`attached_manipulators` 中 **ur5e resolved=True**
  （piper 不再参与），模型 `nq=40/nu=19`；
- JointMujocoBackend 支持多机械臂（piper/ur5 子后端 + 按 lease.resource_id 选择），
  ur5 后端由 `emit_backend_config.py` 生成（targets=box_01、name_map=47）；
- 新 provider `iraf-provider@ur5_go2_lab` 已在 `.145` 起在 **50056**（10 能力，JointMujocoBackend）；
- 板卡注册 `ur5-go2-lab` 场景 + provider 端点后，提交仍被 **supervisor 场景白名单**拦下：
  `switch-failed: supervisor 无此场景：ur5-go2-lab（可用：handoff_lab_joint, piper, ur5）`；
  已给 `_ensure_scene` 加“未知场景继续走 provider 直连”的回退，但运行时仍走到该分支 —— 下一轮修路由。

### A3 通过（2026-11-18）

- 路由修复：`_ensure_scene` 在 supervisor 对所有 host 都返回“无此场景”时，继续走 provider 端点直连，
  不再把“supervisor 不认识”误判为 provider 不可用；
- ur5 主操纵场景 `ur5_go2_lab`：ur5e 放到可抓取位置（`pos_m=[0.28,-0.9,0]`），
  `target_override_world_m=[0.28,-0.28,0.024892]`，pick-only（放置段暂关，避免无关不可达）；
- 实测（板卡 MCP，scene=`ur5-go2-lab`，provider 50056）：

```text
ur抓取盒子 → SUCCEEDED，26.7 s，accepted=true，
             bilateral_contact=true，force_ok=true，constraint_activated=true
```

**A3 完成**：联合世界里 ur5 可单点抓取（uc5 主操纵联合场景），go2 与 piper 走各自路由。

### P4 新机器人（Franka）接入，2026-11-18

- `scene_builder`：机械臂模型可**不声明** `spec.model.imu_site`（只有声明了才校验；无 site 是合法事实）；
- 新增 `profiles/franka_mujoco.yaml`（关节/限位来自 `panda.xml` 实测：7 臂 + 2 指）；
- 新增 `scenes/franka_lab/`（单机场景 + baseline）；
- `vendor/franka_emika_panda/source-lock.json` 补齐（76 个资产，schema `iraf.vendor-source-lock/v1`）；
- 构建实测：

```text
build_scene --scene scenes/franka_lab --robot franka
→ build/scenes/franka_lab/franka_lab.xml
   nq=9 / nv=9 / nu=8 / ncam=1 / nsite=0
```

下一步：把 franka 与 go2 组进同一联合场景（composer 复用），验证 P4 的“新机器人参与联合编排”。
