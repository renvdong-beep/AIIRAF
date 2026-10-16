# 持续执行队列（MPC 专项 → 框架内可用）—— 已授权，不再逐项确认

- 授权：使用者 2026-09-22（"持续执行…不要再询问"）。边界：只 add 自己负责的路径；
  禁 `git stash/checkout/add -A`；每步必须有可复跑判据 + 证据落盘；能力仅在验收通过后回填。
- 推进规则：每轮至少完成一项并提交；卡住时先降级为"记录 + 证伪条件"，不停在该项上反复试；
  **不提交未验证的代码**（本段该纪律拦下 8 处真错）。

## A6a-④（MPC 解 → 执行器）分块进度

| # | 事项 | 状态 | 证据 |
|---|---|---|---|
| ① | 足端力→关节力矩 + B1 载荷（纯映射层） | **DONE** | `0a308bf`；单测 15 项 |
| ② | binding 层：`ProviderRuntime` 裁定 → `_run_control` 的 `torque_provider` 口子（`MpcUnavailableError`，绝不复用旧解） | **DONE** | `fb3b21d`；`torque_hook.py` 38 项单测 + 契约 §4 补两行 |
| ③ | 契约 §4 失败路径**逐条**（真子进程 + 真协议 + 真缩放） | **DONE** | `a3ef947`；`scripts/verify_mpc_torque_hook.py` 6 用例 **21 断言全绿**（退出码 0），报告 `build/acceptance/mpc-a6a4-torque-hook/report.json` |
| ②b | **状态 → QP 请求**（`state_bridge`）+ `locomote()` 去掉显式拒绝，走 TaskFlow→SkillRuntime→PolicyGateway 正式路径（`_run_control` 一行不改） | **口径已固定，实现待做** | 事实与唯一来源见 `docs/debug/2026-09-23-a6a4-state-bridge-facts.md`（质量/COM/惯量/角速度来源已双路线互证到 1.11e-16；上游 4 条必须复刻的语义已抄录） |
| ④ | 回归：站立态 `stop` 与 loopback 三基准逐位不变（`0.279953602548388 / 3.302184116303541e-05 / 0.0038248382123762478`） | 待做 | 只在 ②b 接进 `locomote()` 后才有意义（当前 ②b 未接入，既有路径未被触碰：③ 的 21 断言与 281 项 mpc 单测全绿） |

## 本轮（2026-09-23 下午）新增

- 提交：`fb3b21d`（② binding 层）、`a3ef947`（③ 真子进程失败路径验收）。
- `ProviderRuntime` 三处修正（均由验收/单测抓出）：`client_error/response_reason` 只在**本次更新失败**时记录
  （否则成功判词"解可用…"会被带到"解龄过期"的失败原因上，报告自相矛盾）；本次更新失败时判词加前缀
  「本次更新失败（状态分类 X）」（`freshness` 的"尚未求解"分不清"从没解过"与"解过但本次失败"）；
  急停拍不调子进程。
- 事实固定：`docs/debug/2026-09-23-a6a4-state-bridge-facts.md`。

## 维护债（不随本变更改，需要专门窗口）

| # | 事项 | 为什么不顺手改 | 判据 |
|---|---|---|---|
| S1 | `~/.hermes/skills/robotics/robot-arm-simulation-integration/SKILL.md` 顶到 100K 硬上限，新增被拒；`references/tooling-pitfalls.md` 无链接 | 需一次专门重构窗口（改前备份） | ✅ **已结清（2026-09-24）**：147959 B → **89811 B**（外移 4 段到 references/ + 补 4 条索引链接，备份 `SKILL.md.bak-20260924`）；本地引用 0 缺失（另 4 条是跨技能指向，目标存在） |
| S4 | `vendor/agilex_piper/piper_description` 是**符号链接**（→ `~/piper_ros/src/piper_description`）⇒ 臂的厂商资产**未入库**、其锁写的是本机绝对路径（host-specific）⇒ 换机/新克隆无法复现臂侧仿真，联合产物可移植性受此限制 | 需要把资产实体入库 + 重写为 U1 锁 + 许可证 BOM（独立工作项，涉及体积与授权决策） | 新克隆上 `build_scene.py --attach piper` 与臂侧验收都能复现；锁内无本机绝对路径 |
| S3 | 该技能 `references/` 已有 ~160 个文件、**大量近重复**（`upstream-*-fidelity` 14 个、`qp-*` 20 个、`capability-declaration-*` 8 个、`*-probe-fidelity` 7 个；今天又新增的 `self-referential-verification-traps.md` 与既有 `vacuous-acceptance-and-self-referential-frames.md` 重叠） | 合并要逐份读、判断互补还是覆盖 ⇒ 需专门窗口；**新增条目时先按主题 grep 是否已有同族文件**（本次没做，属自伤） | 每个主题 ≤1 个文件（或主文件 + 明确分工的附录），并在 SKILL.md 索引里写明分工 |
| S2 | `unitree_go2.robot_mass_kg()` = `sum(body_mass)` 含**场景游离物体**（`box_01` 0.04 kg，parent=world）⇒ 质量 15.596408 而本体实为 15.556408，`mg` 偏大 0.3924 N（0.256%），且随场景变化 | 改它会整体位移**已冻结**的平衡路径验收基线 ⇒ 必须改 + 重新取证一起做 | ✅ **已结清（提交 97db7f4）**：子树口径落地（`state_bridge.robot_subtree_mass_kg`）；实测 `robot_mass_kg` 15.596408000 → **15.556408000 kg**（Δ = 0.04 kg = `box_01`，可逐项对账），`tray_01` 0.35 kg 在子树内计入是对的；loopback 三条基准 + 四工况 5/5 **逐位不变**（比原判据更强：基线差异 = 0）；单测 5 项 |

## 已知不承诺项（写进报告，避免结论超出证据）
- 转弯 70.0% / 侧行 120.4% 的跟踪属**传力效率上限**（成因=FR/RL 对角对），A6a 不承诺其达标。
- A4/A5 里"切向世界 y 分量是否反号"依赖符号约定，`(Jᵀ)⁻¹τ` 的整体符号未定死 ⇒ 不引用。
- MPC 质量/惯量按 **trunk 子树**计：挂在 `world` 下的物体（含被夹持物）不计入 ⇒ 携带工况必须把载荷
  挂进 trunk 子树或声明额外载荷（A8 的输入）。
