# IRAF 场景 / Provider 架构设计（2026-11-06）

状态：设计基线（v0.1，待 ADR-0012 评审）  
范围：piper 抓取、ur5 抓取、联合世界（handoff_lab）三个仿真场景；AgentOS 在智控域，
x86 仿真主机由对方/我方独立运维。  
口径：`simulation=true`，不构成真机/实时能力证据。

## 1. 设计原则

1. **AgentOS 不拥有 x86 权限**，也不应该拥有。它只提交任务、选择场景/资源。
2. **Provider 生命周期归 x86 本地**（systemd / supervisor）。边缘 IRAF 只调用
   `provider.proto` 的 Health / Invoke / Cancel，不启动、不重启进程。
3. **一个 MuJoCo 世界一个 Provider 进程**；一个 Provider 可以承载多个机器人
   （联合世界），但只有一个 ControlAuthority。
4. **场景是一等公民**：`scene_id` 决定模型、机器人清单、Profile、SafetyPolicy、
   能力、端点、显示方式；不能在运行中隐式换 Profile。
5. **部署期声明、运行期不可变**：Profile / SafetyPolicy / Provider manifest 签名后
   不热改；切换场景 = 受控部署动作。
6. **不可用显式失败**：Provider 未启动或未 ready 时返回
   `IRAF-PROVIDER-UNAVAILABLE` / `IRAF-TRANSPORT-FAILED`，不伪造成功、不无限重试。

## 2. 三层职责

```text
┌────────────────────────────────────────────────────────────┐
│ AgentOS / 上位机 harness（语义层）                          │
│  · 只提交任务、选择 scene_id / resource                     │
│  · 不直连 Provider、不执行 x86 命令、不管 MuJoCo 生命周期    │
└───────────────────────────┬────────────────────────────────┘
                            │ MCP / gRPC（任务级）
┌───────────────────────────▼────────────────────────────────┐
│ 边缘 IRAF（智控域）                                         │
│  · TaskFlow / Skill Manager / Policy / ControlAuthority     │
│  · SceneRegistry：scene_id → provider manifest              │
│  · 只路由能力调用；Provider 不可用显式失败                  │
└───────────────────────────┬────────────────────────────────┘
                            │ provider.proto（Health/Invoke/Cancel）
┌───────────────────────────▼────────────────────────────────┐
│ x86 Provider Plane（仿真/运控）                             │
│  · Scene Supervisor：start/stop/status/reset、等 Health     │
│  · MuJoCo 世界 + ProviderKit                                │
│  · 进程、端口、GPU/显示、崩溃重启归本机                     │
└────────────────────────────────────────────────────────────┘
```

## 3. 三个场景

| scene_id | MuJoCo 世界 | 机器人 | 当前状态 |
|---|---|---|---|
| `piper-pick` | `build/models/piper-pick-scene.xml` | piper | Provider 可运行（50052），当前未启用 |
| `ur5-pick` | `build/models/ur5-pick-scene.xml` | ur5e | 当前 .121 已运行（50053），.147 绑定此场景 |
| `joint-handoff` | `build/scenes/handoff_lab/handoff_lab_joint.xml` | piper、ur5e、unitree_go2、humanoid_static（静态） | 有场景文件与 scenario 脚本，**尚无 Provider/SceneProfile 声明** |

联合世界不是“三个独立 Provider”；它是一个 Provider 进程承载一个多机器人世界。

## 4. Scene Catalog（建议新增）

每个场景一份签名声明，部署期生成，运行期只读：

```yaml
scene_id: joint-handoff
provider_id: iraf/handoff-lab
model: build/scenes/handoff_lab/handoff_lab_joint.xml
simulation: true
endpoint: 10.203.247.121:50054
display: window
robots:
  - id: piper
    kind: arm
    profile: profiles/piper_mujoco.yaml
    resource_id: joint-piper
    capabilities: [pick_object, place_object, move_joint, stop]
  - id: ur5e
    kind: arm
    profile: profiles/ur5_mujoco.yaml
    resource_id: joint-ur5
    capabilities: [pick_object, place_object, move_joint, stop]
  - id: unitree_go2
    kind: quadruped
    profile: profiles/go2_mujoco.yaml
    resource_id: joint-go2
    capabilities: [stand, locomote, dock_for_handoff, stop]
  - id: humanoid_static
    kind: humanoid
    role: scene_entity
    capabilities: []
```

单机器人场景同理，只写一个 robot。

## 5. 边缘路由

当前限制：一个边缘 Runtime 只绑定一个 `IRAF_PROFILE` + 一个 Provider endpoint；
运行中不能切换 Profile（AGENTS 2.6）。

建议的 SceneRegistry：

```text
iraf_submit_task(text, scene_id="joint-handoff", resource_id="joint-piper")
  → SceneRegistry 取 manifest
  → 校验 scene/profile/safety/capabilities
  → ControlAuthority 按 resource 取租约
  → provider.proto Invoke(endpoint, lease)
```

没有 SceneRegistry 的短期做法：

- 一个场景一套边缘配置 + 一个 MCP 端点：
  `piper:8765`、`ur5:8766`、`joint:8767`；
- AgentOS/上位机按目标选择 MCP URL；
- 每套配置的 Profile/Safety 固定，切换是部署动作，不是运行时参数。

单板 2 核不建议同时跑三套 edge runtime；按场景切换或使用独立 VM。

## 6. Provider 生命周期

x86 本地 supervisor（不暴露给 AgentOS）：

```bash
provider-supervisor status
provider-supervisor up piper|ur5|joint
provider-supervisor down
provider-supervisor reset
```

职责：

- 停旧世界、起新世界；一个时刻一个世界；
- 等 `Health.ready=true`，不是盲 sleep；
- 端口、显示、资源检查；
- 崩溃重启、日志与证据；
- 输出当前 `scene_id`、`resource_id`、endpoint、Profile digest。

启动顺序：

```text
supervisor up <scene>
  → MuJoCo + ProviderKit 监听
  → Health ready / inventory 正常
  → 边缘 SceneRegistry 可见
  → AgentOS 提交任务
```

Provider 不在/未 ready 时，边缘必须直接拒绝，不得自动拉起、不得重试风暴。

## 7. 联合世界的额外约束

1. **单 Provider、单 ControlAuthority**
   禁止三个独立 Provider 同时控制同一个 MuJoCo 世界；租约、fencing、控制源会冲突。
2. **多 resource 租约**
   一个 Provider 内可按 `joint-piper` / `joint-ur5` / `joint-go2` 分租约，
   但控制权仍由同一个 Authority 统一发放。
3. **SceneProfile**
   当前 `RobotProfile` 是单机器人假设；联合世界需要：
   - 新增 `SceneProfile`（多机器人清单 + 每机器人 Profile）；或
   - 一个复合 Profile，把 capabilities 声明到各 resource。
   不补这层，边缘 Policy/Skill 无法判定“当前控制的是谁”。
4. **资源消耗**
   联合世界比单臂重；2 核 x86 只跑一个世界。不要同时跑 piper、ur5、joint。

## 8. 错误码与失败路径

| 场景 | 返回 |
|---|---|
| Provider 未启动 / 端口拒绝 | `IRAF-TRANSPORT-FAILED` |
| Provider Health 未 ready | `IRAF-PROVIDER-UNAVAILABLE` |
| scene_id 未注册 | `IRAF-SCENE-UNKNOWN` |
| scene 与 resource 不匹配 | `IRAF-POLICY-DENIED` |
| 旧 fencing token / 终态重发 | `IRAF-EXECUTION-CONFLICT` |
| 安全事件 | `SafetyEvent` → `QUARANTINED`，复位前不释放 |

## 9. 落地路线

### 现在（不改核心）

- piper / ur5 各自 Provider + 各自边缘配置/MCP 端点；
- 联合世界单独 Provider + 单独边缘配置/MCP 端点；
- x86 systemd/supervisor 保证“先起 Provider、Health ready”；
- AgentOS 只选对应 MCP URL；
- Provider 不可用时明确报错，不自动启动。

### 下一步（框架化）

- 新增 Scene Catalog + SceneRegistry；
- 新增 SceneProfile / 多 resource 声明；
- MCP 增加 `scene_id` / `resource_id`；
- x86 本地 `provider-supervisor`（status/up/down/reset）；
- 边缘按 scene 加载签名 Profile/Policy，并做 Health 门禁。

### 长期（可选，需 ADR/安全评审）

- 如确实需要 AgentOS 触发“启动场景”，不让 AgentOS 直连 x86：
  在 x86 放受控 Lifecycle Broker（mTLS、签名命令、白名单、审计），
  由 harness/运维授权调用；这不是 provider.proto 的现有范围。

## 10. 验收

- `scene_id` → manifest → endpoint/profile 能一一对应；
- Provider 未起时任务显式失败，不产生 execution 成功；
- Provider ready 后任务成功，`iraf_get_task` 可回查；
- 联合世界同一时刻只有一个控制源；
- 图内/文档结论标注 `simulation=true`。

## 11. 当前实机状态（2026-11-06）

本机已实现 `scripts/scene_supervisor.py`：
`status / up piper|ur5|g1 / down`，up 会停其他世界并等 Provider Health ready；
`joint` 在 SceneProfile/Provider 单元补齐前明确返回“未实现”。

```
.147 边缘：profile=profiles/ur5_mujoco.yaml
           endpoint=10.203.247.121:50053
           iraf-mcp-http active :8765
.121 x86：iraf-provider@ur5 active :50053（enabled）
           iraf-provider@piper failed / disabled
           iraf-provider@g1 inactive / disabled
```

因此当前 AgentOS 只能控制 UR5；piper 与 joint-handoff 需要启动对应 Provider 并切换
边缘 profile/endpoint（或使用独立 MCP 端点）。

## 12. 不做什么

- 不给 AgentOS x86 SSH/进程权限；
- 不让边缘 IRAF 启动/停止对方 MuJoCo；
- 不在运行中热改 Profile/SafetyPolicy；
- 不把联合世界拆成三个独立 Provider 同时写一个 MuJoCo 世界；
- 不把 scene 切换做成隐式默认值或字符串猜测。
