# s05 `accept_payload` 交付计划（载荷确认 / DOG_CONFIRM）

状态：**进行中第 1 步**（计划已落盘）
创建：2026-09-28
上游：s01 stand / s02 dock / s03 pick / **s04 place_in_tray 已通过验收**（见
`docs/debug/2026-09-24-joint-model-dog-arm.md` §11.23(41)(h)，报告 `passed=True`、4 步 SUCCEEDED）

## 1. 目标与边界

目标：在联合世界（Go2 + Piper 同一份 MJCF）里，交接完成后由**四足侧**确认"载荷确实落在接收体上、
且整链处于静止安全状态"，作为交接链的最后一步验收（s05）。

不做（本轮明确排除）：
- 不引入真机/硬件载荷传感器（板卡不在场）；仿真里"载荷传感"= **接收体与载荷的接触事实 + 载荷落位**，
  与 s04 的 `payload_in_tray` 同口径，但**由四足侧技能独立复核**（不是复用臂侧证据 ⇒ 否则等于自己证明自己）。
- 不改 s01–s04 的任何判据与声明。

## 2. 契约（先写契约，再写实现 —— AGENTS.md 2.1）

新增 `skills/accept_payload/`：

- `skill.yaml`：`safetyClass: monitoring`（**不产生物理动作** ⇒ 不进 MOTION_CAPABILITIES）；
  `preconditions: [safety.estop == false]`；`timeoutSeconds: 60`（覆盖墙钟）；
  `providers: [{type: python_adapter, entrypoint: "...:AcceptPayloadProvider", selector: {mode: simulation}}]`
- `input.json`（`additionalProperties: false`）：
  - `payload_id`（必填，string）：要确认的载荷（本场景 `box_01`）
  - `place_target_id`（必填，string）：预期承载它的接收体（本场景 `tray_01`）
- `output.json`（`additionalProperties: false`）：
  - 必填 `skill`/`accepted`(const true)/`payload_id`/`place_target_id`/`confirmation`(enum: `payload_confirmed`)
  - `evidence`（必填：`payload_on_target`、`payload_low_z_m`、`target_top_z_m`、`resting_gap_m`、
    `contact_geoms`、`payload_center_m`、`target_center_m`、`offset_from_target_center_m`、
    `last_speed_mps` + 诊断键 `runtime_source`/`phase_trace`）

判据口径（**全是事实，不设力阈值**）：
1. `payload_on_target`：载荷的**最低点**落在接收体承载面（`target_top_z_m ± gap_tolerance_m`）内，
   且载荷与接收体**存在接触**；
2. 载荷中心相对接收体中心的水平偏移 ≤ 声明阈值；
3. 静止确认：本步结束时的**末速** ≤ 声明阈值（`max_speed_m_s`）。

阈值全部来自声明（`config/scene.schema.json` 的 s05 判据名单 + `scenario.yaml` 的 s05 `criteria`），
实现层不写默认值。

## 3. 实现步骤（每步可验收）

| 步骤 | 内容 | 验收 |
|---|---|---|
| 1 | 本计划 + 清理死代码 | 计划落盘；`py_compile` + 相关单测通过 |
| 2 | 写契约三件套（skill.yaml / input.json / output.json）+ 场景/profile/策略/适配器四处能力同步（`accept_payload`） | `scene_check` exit 0；契约 JSON 可被 `jsonschema` 校验 |
| 3 | 后端实现 `accept_payload`：在**四足侧**后端上按 `place_targets` + `targets` 复核载荷落位与接触，输出证据 | 单测：成功路径 + 失败路径（载荷不在承载面上 / 无接触 / 末速超阈）各 1 条 |
| 4 | `AcceptPayloadProvider`（`SkillRejected` on 证据不成立；后端未实现即拒绝） | 单测同上 |
| 5 | 场景 `s05_confirm_payload` 从"待交付"改为启用（params + 判据），跑 `nominal --world joint` | 报告 `passed=True`、**s01–s05 全 SUCCEEDED**、`failed_checks=0` |
| 6 | 文档 §11.23(42) + 提交（Conventional Commits） | 提交信息含实测数字；工作树干净 |

## 4. 风险与预先判死

- **风险 R1**：四足侧后端与臂侧后端共享同一株植物，读取模型事实时的锁/时序 —— 已有
  `_yield_residency_to_step` 机制可复用；若出现竞态，按 §11.11 的四条路径并排量化后修。
- **风险 R2**：s05 判据若与 s04 的同名量口径不一致 ⇒ 会出现"臂说放下了、狗说没有"。
  **预先判定**：两者必须都能从**同一份实测 FK + 接触**推出；若不一致，先统一口径再继续（不得两边各写一套）。
- **风险 R3**：把 s05 做成"复述 s04 证据" ⇒ 违反"不伪造成功"。**预先判定**：s05 必须**独立重新测量**
  （新采样、四足侧模型视角），并在证据里给出与 s04 不同的量化量（如 `resting_gap_m`、末速）。

## 5. 下一步（本轮先做第 2 步）

契约三件套 + 四处能力同步；完成后立刻跑 `scene_check` 与单测，再进入第 3 步实现。
