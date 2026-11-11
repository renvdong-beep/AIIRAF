# 端侧 kinematic 计划后端 · 实施计划（0.4.5 备份 → 0.4.6 交付）

> 目标：**没有 x86、没有 provider 进程、没有 MuJoCo 物理/渲染**时，端侧也能对机械臂动作做
> **真实的运动学计划计算**并反馈结果（不是伪造成功）。
>
> 版本策略：**0.4.5 = 实施前冻结备份**；**0.4.6 = 含 kinematic 后端**。

---

## 0. 背景与边界（先对齐事实）

| 事实 | 依据 |
|---|---|
| 当前仿真里规划 + 计算全在 **x86** 的 MuJoCo backend | `mujoco_backend._move_trajectory`、`iraf_core/kinematics.py`（IK 用 `mj_jacBody` 迭代） |
| 运控域（`edge-motion`）只有声明与桩，**未参与规划** | `install_profiles.yaml`：`verification: unverified`、`backend_entrypoint: unverified`、`iraf-control-provider` 为就绪占位 |
| 端侧（板卡，4 核 + 本地 9B）跑不动 MuJoCo **物理** | 实测 x86 4 核 + A4000 的 ur5 pick 也要 ~57s；物理步进是瓶颈 |
| 端侧 **只跑运动学** 是可行的 | MuJoCo aarch64 wheel 已在离线 wheelhouse（`mujoco-3.14.0`）；只加载模型 + `mj_forward`/IK，不 `mj_step`、不渲染 |

**能算 / 不能算（本计划的硬边界）**

| 计划计算能给 | 计划计算不能给 |
|---|---|
| FK/IK 收敛与关节角 | 接触是否发生 |
| 关节轨迹、各段时长 | 是否滑移/掉落 |
| 限位余量、可达性 | 沉降/漂移/振动 |
| 夹爪开口 vs 物体尺寸 | 抓取成功率 |

因此所有输出必须标 **`planned=true, physics=false`**，判据只说"计划可行"，不得表述为"抓取成功"。

---

## 1. 版本与回滚

| 版本 | 内容 | 做法 |
|---|---|---|
| **0.4.5（备份）** | 当前 HEAD（含紧凑输出、rt_latency skill 等），**不含 kinematic** | bump `pyproject.toml` → 构建 wheel+bundle → `git tag v0.4.5` → 产物取回 `dist/0.4.5/` |
| **0.4.6（交付）** | "+kinematic 后端/场景/路由/报告" | 实施完成后 bump 0.4.6 → 构建 → `git tag v0.4.6` → `dist/0.4.6/` |

回滚方式：`git checkout v0.4.5` 重装 0.4.5 产物，或只删 kinematic 场景声明（MCP 回到现有行为）。

---

## 2. 设计

### 2.1 新后端：`kinematic`

```
src/iraf_adapters/kinematic/
├── __init__.py
└── backend.py        # KinematicBackend
```

实现方式：

1. 用 `mujoco.MjModel.from_xml_path(<scene>.xml)` 加载**同一份**场景（bundle 的 `prebuilt/scenes/`）；
2. `mj_forward` + `iraf_core.kinematics.solve_*_ik` 求解目标位姿；
3. 关节轨迹 = 从当前 `qpos` 到 IK 解的线性/三次插值（步数按 profile 声明频率），**只算数**，不 `mj_step`；
4. 每步检查 `jnt_range` 限位、速度/加速度上限（来自 profile 声明）；
5. 输出计划证据：`planned=true, physics=false, backend=kinematic`、各段时长、终点误差、限位余量、夹爪开口。

实现的能力（与现有 skill 对齐）：

| 能力 | 计划计算内容 |
|---|---|
| `move_joint` | 目标角 → 轨迹步数/时长/限位余量 |
| `pick_object` | IK 到抓取位姿、闭合轨迹、抬升高度（声明值）、夹爪开口 vs 物体尺寸 |
| `place_object` | IK 到放置位姿、释放高度、终止误差 |
| `reset_scene` / `stop` | 立即成功（无物理） |

**可复用**：`iraf_core/kinematics.py`（IK 契约）、`config/ur5_simulation_baseline.yaml` / `config/piper_*`（速度/时长声明）、`deploy/sdk/prebuilt/scenes/*`（模型）。

### 2.2 场景与 Provider 声明

新增场景（**与真实仿真并存，不覆盖**）：

```yaml
# config/scenes/ur5-pick-planned.yaml
scene_id: ur5-pick-planned
switch_name: ur5-pick-planned
provider_id: iraf/ur5-pick-planned
simulation: true
backend: kinematic          # 新字段
provider:
  hosts: [{id: local, endpoint: local, control_url: ""}]
```

`piper-pick-planned` 同构。`backend: kinematic` 缺省不存在 ⇒ 旧行为零影响。

### 2.3 MCP 本地路由（不起 provider、不连 x86）

`iraf_adapters/mcp/server.py`：

- 解析场景后若 `backend == "kinematic"`：
  - **不**创建 world 租约到远端（或创建本地租约但不连 host）；
  - 用 `SkillRuntime`（现有链路）加载 `profiles/<robot>_mujoco.yaml` + **本地 `KinematicBackend`**；
  - 仍走 `TaskFlow → Skill → Policy → Provider`（铁律 2），只是 Provider 是本地进程内后端；
- 返回结果与现有 `iraf_submit_task` 同构，附加 `planned=true, physics=false, backend=kinematic`。

### 2.4 报告与口径

- 报告目录：`build/acceptance-planned/<scene>/<scenario>/report.json|md`（**与物理验收目录分离**）；
- 报告头强制包含：`planned=true`、`physics=false`、`backend=kinematic`、`model=<scene xml>`、`model_sha256`；
- chat 摘要前缀：`⚠️ 计划计算（无物理仿真）`；
- `iraf_get_task` 的 compact 返回同样带 `planned/physics` 标记。

---

## 3. 任务清单（按顺序执行）

- [ ] **T0 计划 MD 入库**（本文件同时放到仓库 `docs/plans/`）
- [ ] **T1 冻结 0.4.5**：bump → 构建 wheel+bundle（universal）→ `git tag v0.4.5` → `dist/0.4.5/` + sha256 记录
- [ ] **T2 kinematic 引擎**：`KinematicBackend`（IK/FK、轨迹、限位、时序）+ 单测
- [ ] **T3 场景/Provider 声明**：`ur5-pick-planned`、`piper-pick-planned`；schema 增加 `backend` 字段
- [ ] **T4 MCP 本地路由**：`backend: kinematic` 走进程内 Runtime/Provider；不连 supervisor
- [ ] **T5 端侧验证（板卡）**：装 mujoco aarch64（离线 wheelhouse）→ 跑 `ur5-pick-planned`/`piper-pick-planned`
      - 记录：IK 迭代次数/耗时、轨迹时长、限位余量、夹爪开口、报告路径
      - 与 x86 MuJoCo **规划输入**对照（同一 IK 契约，终点应一致）
- [ ] **T6 口径检查**：报告/返回值/chat 文案三处都有 `planned/physics` 标记
- [ ] **T7 发布 0.4.6**：bump → 构建 → `git tag v0.4.6` → `dist/0.4.6/` + 文档更新

---

## 4. 验收标准

1. **端侧可跑**：板卡上（无 x86、无 provider 进程）`iraf_submit_task(text="抓取 box_01", scene_id="ur5-pick-planned")`
   返回 `status=SUCCEEDED, planned=true, physics=false`，单次耗时 **< 2s**（目标 <500ms）；
2. **数字合理**：IK 位置误差 < 1e-3 m；关节角在 `jnt_range` 内；轨迹时长与声明速度一致（误差 <5%）；
3. **可对照**：同一目标位姿在 x86 MuJoCo 的规划输入与 kinematic 解的关节角差 < 1e-6（同一 IK 契约）；
4. **口径正确**：报告、MCP 返回、chat 摘要均含 `planned=true, physics=false`，且**没有**"9/9 判据通过"这类物理结论；
5. **不回归**：原 `ur5-pick`/`piper-pick`/`joint-handoff` 行为不变（0.4.5 场景仍可用）。

---

## 5. 风险与对策

| 风险 | 对策 |
|---|---|
| 板卡装 mujoco 失败/体积大 | 离线 wheelhouse 已有 aarch64 cp311；只装运行时（不装 viewer）；实测 import 耗时再评估 |
| 模型加载慢（meshes 多） | 只加载所需场景；预加载缓存；首帧耗时记入报告 |
| IK 在板卡上不收敛 | 复用 x86 已验证的初值与阻尼参数；失败显式返回 `IK-FAILED`，不伪造 |
| 与物理结论混淆 | 三处强制标注 + 目录分离 + 文档红线 |
| 场景 schema 兼容 | `backend` 字段可选，缺省 `mujoco`；`registry_check` 校验新增枚举 |

---

## 6. 进度记录（实施中持续更新）

| 时间 | 版本 | 事件 | 产物/证据 |
|---|---|---|---|
| 2026-11-11 17:35 | — | 计划创建，goal 启动 | 本文件 |
| | | T1 0.4.5 备份 | （待填 sha256） |
| | | T5 端侧验证 | （待填报告路径） |
| | | T7 0.4.6 发布 | （待填 sha256） |
