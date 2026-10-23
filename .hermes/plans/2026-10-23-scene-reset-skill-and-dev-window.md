# 场景复位做成 IRAF 技能 + 开发语义默认出 MuJoCo 窗口（2026-10-23）

## 0. 用户诉求（原话要点）

1. 「场景复位是不是也应该有一个 IRAF 的 skill，用来清理当前的某一个 mujoco 或者所有 mujoco」
2. 「"抓取 box" 这种机器人开发语义，应该默认就启动 x86 的 mujoco 来演示，方便用户开发调试并反馈」

## 1. 两个诉求必须分层，否则会互相污染

| 诉求 | 归属 | 为什么不能混 |
|---|---|---|
| 复位场景状态 | **IRAF Skill（能力面）** | 它改的是"物理世界状态"，必须过 TaskFlow→Skill Runtime→Policy→Provider，要有租约、事件记录、可审计 |
| provider 不存在/没窗口时自动拉起 | **开发环境运维（deploy/scripts）** | 它是进程生命周期，不是机器人动作；放进 Skill 就等于让 LLM 能起进程（越界） |

反过来也不行：把"复位"留在 shell 脚本里（现状 dev_reset_provider.py），
平台侧就永远学不会"复位"这个语义，用户只能找运维敲命令 —— 这正是要消掉的旁路。

## 2. `reset_scene` Skill 设计

```
name: reset_scene
safetyClass: maintenance        # 不产生运动，但会改状态；必须独占资源
requires: [reset_scene]
preconditions: [safety.estop == false]
inputSchema:
  scope: enum[target, all]      # 单个目标 / 全部
  target_id: string             # scope=target 时必填
timeoutSeconds: 30
recovery: [stop]
intentExamples: ["复位场景", "清理 mujoco", "把场景恢复初始", "reset scene"]
intentTrigger: {natural_language: true, note: "开发/调试语义；生产环境按声明禁用"}
providers: [{name: common_reset_scene_sim, type: python_adapter,
             entrypoint: "iraf_skills.common.scene:ResetSceneProvider",
             selector: {mode: simulation}, priority: 100}]
```

关键点：

- **必须持租约**（同一 `piper_mujoco` 资源）。运行中的任务会撞 `IRAF-RESOURCE-BUSY` —— 这是
  正确保护，不是缺陷（今天 place_object 连发两次已实测过这条）。
- `scope=all` 的实现是**在进程内复位**（`mj_resetDataKeyframe(model, data, 0)` + 清渲染缓存/指标），
  **不重启 provider 进程** ⇒ **窗口不灭**，开发体验才成立。这是它与 `dev_reset_provider.py --reset`
  （重启进程）的本质区别：前者是技能，后者是运维。
- provider 侧新增能力名 `reset_scene`，在 `remote/provider_service.py` 的 `_POSITIONAL` 登记
  `("scope", "target_id")`，并**不要**放进 `_NO_LEASE`。
- 事件库照常记录：`PENDING → VALIDATING → RUNNING → SUCCEEDED`，`resolved_skill=reset_scene`
  ⇒ "谁在什么时候把场景清了"可审计。

## 3. "抓取默认出窗口" —— 声明驱动，不做隐式魔法

铁律 2.6（不可变配置优先，禁止运行中隐式修改）与 1.5（不可用时显式失败）都指向：
**不能偷偷起服务**。所以做成"开发档声明 + 守护进程读声明"：

```yaml
# profiles/piper_mujoco.yaml（仅 verification: development 生效）
dev:
  provider_display: window      # window | headless
  autostart_provider: true      # 生产 profile 必须为 false
```

- `scripts/dev_provider_supervisor.py --profile piper_mujoco`：
  读声明 → 探测 50052 与当前 display mode → 不可达或模式不符时按声明拉起（window 模式，
  复用 dev_reset_provider.py 的环境构造：:0 / XAUTHORITY / LIBGL_ALWAYS_SOFTWARE / IRAF_VIEWER_DISPLAY_MODE=interactive）
  → 每次动作写日志（谁、何时、为什么拉起）。
- 非 development 或不写 `dev:` 段 ⇒ 行为与今天完全一致（不拉起、缺 provider 就显式失败）。
- 不做成 systemd 常驻：它是开发辅助，按需跑（避免在板卡/生产上无故起窗口进程）。

## 4. 实施顺序

1. **reset_scene 技能**（契约 → provider 能力 → 单测 → 升 0.3.15 → 上板 → 一句话验证）
   验收：AgentOS 里说"复位场景"→ 事件库出现 `resolved_skill=reset_scene` 的 SUCCEEDED；
   窗口不消失；随后"抓取 box_01"仍 SUCCEEDED。
2. **dev 声明 + supervisor**（profile 声明 → 脚本 → 冷启动验收）
   验收：先杀掉 provider，再在 AgentOS 说"抓取 box_01"→ supervisor 按声明拉起窗口版
   → 任务 SUCCEEDED 且 xwininfo 能看到 MuJoCo 窗口。
3. （可选，第三）place 场景：把 tray_01 纳入一份 handoff 场景声明，解决"放进托盘"必失败。

## 5. 口径

- simulation=true，不构成真机能力证据。
- reset_scene 是**开发/调试**能力；生产 profile 不启用（与 autostart 同理）。
- 不允许 LLM 直接起进程/改配置：它只能提交任务（reset_scene 是任务），进程生命周期属运维。
