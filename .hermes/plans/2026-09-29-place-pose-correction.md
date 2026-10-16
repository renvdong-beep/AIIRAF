# 计划：放下点按**运行期实测托盘位姿**纠偏（消除"偏移余量只剩 1.7 mm"的验收脆弱点）

状态：待执行（B 轮已清债，见提交 `c277547` / `f574fd`）
起点：HEAD `c277547`，工作树干净，`nominal --world joint` 绿。

## 1. 问题（带实测数字）

托盘改为随狗运动（`pose.mount: unitree_go2/tray_frame`）后，放置偏移由**狗的实际停靠位姿**决定，
而放置四段航点是**构建期**按"名义停靠位姿"解出的 ⇒ 停靠误差直接变成放置偏移。

三轮同一场景实测（判据 `max_offset_from_tray_center_m = 0.06`）：

| 轮次 | `place_offset_from_tray_center_m` | `accept_offset_from_target_center_m` | `accept_last_speed_mps` |
|---|---|---|---|
| A（无窗口） | 0.052625625 | 0.052586728 | 0.003442571 |
| B（interactive_viewer） | — | 0.058340468 | 0.007412233 |
| C（无窗口） | 0.056927398 | 0.056955541 | 0.006272556 |

⇒ 偏移余量最薄 **1.7 mm**（0.06 − 0.058340468）⇒ 属"会随机变红"的验收脆弱点（末速余量 2.6 mm/s 尚可）。
停靠误差本身已实测为 `dock_translation_error_m = 0.02856927288102236`（该路径"参数空间已穷尽"，
见 skill `references/docking-and-low-speed-positioning.md`）⇒ 只能在**放置侧**纠偏。

## 2. 方案（声明驱动，不调阈值、不写死数字）

原则：放置点的**基准**必须来自运行期实测；构建期解仍保留（它给的是关节空间解的**形状**），
纠偏量与纠偏启用由声明给出，缺声明即显式失败。

1. **测量**：`place_object` 在搬运段开始前读取接收体的**实测位姿**（`tray_01` 的 body xpos/xquat），
   与报告里的 `nominal_pose_m`/`nominal_quaternion_wxyz` 比较 ⇒ 得到 `pose_delta`（位置 + 姿态）。
2. **纠偏**：把该 delta 变换到**臂基座系**，作为 `above`/`descend` 两段关节解的**增量目标**，
   按声明的方式施加：
   - `lateral_only`（推荐首选）：只纠水平两轴（停靠误差以水平为主，实测 0.0286 m）；
   - `full_pose`：位置三轴 + 绕竖直轴偏航。
   方式与容差上限由 `config/piper_simulation_baseline.yaml` 的 `grasp.place_pose_correction:
   {mode: lateral_only, max_lateral_m: …}` 声明；超上限即**显式拒绝**（不静默截断、不静默照做）。
3. **自证**：报告新增 `place_alignment.correction_applied_m`、`correction_source:
   runtime_fk_minus_nominal`，并在 s04 判据里继续只看**实测偏移**（判据口径不变）。
4. **反例保护**（铁律 1.5）：纠偏不可用时（接收体无 mount、或 delta 超上限）必须显式失败，
   不得退回"按名义位姿照放"。

## 2b. D1 已完成（提交 `7f56dec`）：声明化 + 只测不施，**delta 已实测**

- 声明：`grasp.place_pose_correction: {mode: measure_only, max_lateral_m: 0.05}`；
  合法模式 `off | measure_only | lateral_only | full_pose`，**未实现的模式声明了即显式失败**。
- 共享纯函数 `payload_facts.resolve_place_pose_correction`（唯一口径 + 7 项单测，含 3 条拒绝路径）。
- 观测面：scene 报告 `place_correction_lateral_m` / `place_correction_vertical_m` / `place_correction_applied`。
- **实测（nominal --world joint）**：横向 **0.033814698 m**、竖向 **−0.012792153 m**（托盘实测比名义低
  12.8 mm，现由落稳窗吸收）、`place_offset_from_tray_center_m` 0.051933449、末速 0.005050205，整链绿。
- 分解：`0.0519 ≈ 停靠横向 0.0338 ⊕ 约 18 mm 系统性偏差` ⇒ **施加纠偏后预期 offset ≈18 mm（余量 ≈42 mm）**。

## 2d. D2 结论（已提交）：施加被实测判定有害，**真因转到搬运侧**

- 施加 `lateral_only` ⇒ s05 末速 0.011856192 > 0.01（只纠水平 ⇒ 落距仍是名义高度的 12.8 mm）；
  改 `full_pose` ⇒ 末速 0.007864205 ✓ 但 place_offset 0.051933449 → **0.059755439**（余量 0.28 mm）⇒ 有害。
- 相对纠偏（nominal→live）为什么救不了：**构建期解本身是对的** —— `place_nominal_solution_lateral_m
  = 8.251e-06`；停靠项仅 lateral 0.0337 / vertical −0.0128。
- **真因**：载荷相对**夹口**横向偏移 —— 放置开始时 0.032725241，after_descend 时 **0.055144315**
  （`pad − tray_top` 只有 0.019525514；`pad_z − payload_low = 0.056346` 说明载荷吊在夹口下方属正常）
  ⇒ **搬运中渐进增大 +22 mm**，对应 §11.23(43) 的"载荷挂陈旧 anchor"机制。
- **逐段轨迹（本轮补测）把真因拆成两段**：`start` 时载荷已偏 0.032794（**抓取偏心**，pick 缺这条
  不变式：只量"臂到位"，不量"载荷是否在夹口轴线上"）；`transit` 段再沿单一基座轴**纯侧滑 +0.02145**
  （§11.23(43) 的陈旧 anchor 机制）；`above`/`descend` 段刚性跟随不再恶化。
- ⇒ **下一步工作项（按序）：① pick 侧补"载荷-夹口轴线偏移"判据（现在必红）；② 修抓取偏心；
  ③ 压 transit 侧滑（carry_constraint 的锚点跟随/节拍/solref）；④ 之后再启用 full_pose 只管停靠项。**
  本计划的施加路径保留为 `mode: off|measure_only` 的观测与 fail-closed 机械（已实现、已测）。

## 2c. （原计划）施加（`lateral_only`）——具体设计（暂缓，原因见 §2d）

1. **取臂基座系**：从 `wrist_body` 沿 parent 链**上溯到 parent==world 的那一节**（模型驱动，
   不写死 `piper_*` 名），其 `xmat` 即安装姿态 R ⇒ `delta_local = Rᵀ·delta_world`（旋转不改变模长，
   与 D1 的 `lateral_m` 自洽，可交叉验证）。
2. **改目标、不改结构**：`above`/`descend` 两段的关节目标按"世界系平移 delta（只取水平两轴）"
   用 `iraf_core.kinematics.solve_position_ik` **重新求解**，**种子 = 构建期名义解**（同分支；
   这正是"运行时现解 IK 会落错分支"的规避方式）；`transit` 保持名义（载荷离承载面还远）。
3. **自证（全部 fail-closed）**：求解残差 ≤ 声明容差；求解后**工具轴与名义同半球**（dot ≥ 0，
   防翻分支）；`pad_mid` 相对名义的水平位移与 delta 的差 ≤ 声明容差；任何一条不满足即
   **显式失败并回退到 `measure_only` 行为**（记录原因，不静默照放）。
4. **顺序**：先 `mode: measure_only` 跑 1 次确认 delta 稳定；再切 `lateral_only` 连跑 3 次，
   验收 `place_offset_from_tray_center_m` 上界 ≤0.035 且离散度收窄；任何一次 s01–s05 变红即
   回退 `measure_only` 并记录（不硬推）。


- 同一场景连跑 **3 次**：`place_offset_from_tray_center_m` 的上界必须显著小于 0.06（目标 ≤0.035），
  且离散度（max−min）明显小于现在（现在 0.058340468 − 0.052625625 = 0.005714843）。
- 契约：`skills/place_object/place_object.output.json` 的 `place_alignment` 补两个键（先改契约再改代码）。
- 回归：s01–s05 全绿 + 子集 1524 项全绿；纠偏关闭（声明 `mode: off`）时与现在**逐位一致**。
- 文档：`docs/debug/2026-09-24-joint-model-dog-arm.md` 新增 §11.23(48)（含三轮基线数字）。

## 4. 边界

- 不动停靠控制（该路径已实测穷尽）；不改判据阈值（阈值是验收口径，不是调节旋钮）。
- 不改臂的运动学时序（四段回放的结构不变，只改目标）。
- 若实测发现纠偏会把载荷带出托盘（`payload_in_tray` 变 false），立即回退并记录，不硬推。
