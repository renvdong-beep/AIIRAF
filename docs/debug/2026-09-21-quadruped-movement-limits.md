# 四足移动安全边界与停止语义声明落地（步骤 01，战役 `iraf-24h-2`）

- 日期：2026-09-21　　步骤：`plans/iraf-24h-2/01-ADR-0008 定稿 + 移动安全边界声明 + 停止语义矩阵落地.md`
- 目标：把"能走、能转、能到点"所需的**移动语义边界**与**停止语义矩阵**写成声明，并让实现层
  消费它们（越界/缺键/未知停止模式一律显式失败），为步骤 02 的步态控制器提供可声明的边界。

## 1. 先量再写：本步用到的实测数字

复跑：`PYTHONPATH=src /usr/bin/python3 build/iraf-24h-2/01/probe-workspace.py`
（原始输出 `build/iraf-24h-2/01/probe-workspace.txt`）

| 量 | 实测值 | 用途 |
|---|---|---|
| 场景台面 `workbench` 半边长（编译后 box geom） | 0.8 m（`scene.yaml` `terrain.workbench.half_size_m` 一致） | 工作空间矩形上界的来源 |
| Go2 躯干碰撞盒（`base_link` 唯一 box geom） | 半长 0.1881 m、半宽 0.04675 m、半高 0.057 m | 台面边缘余量推导 |
| 关键帧 `home` 下的 `base_link` 位置 | (0, 0, 0.27) m | 与既有 `stand.height_target_m` 一致（未改） |
| 模型 `timestep` | 0.002 s | 控制频率 100 Hz ⇒ 每控制周期 5 个物理步（未改） |

**推导**：`workspace_m` 取 ±0.5 m ⇒ 台面边缘到躯干边缘余量 0.8 − 0.5 = 0.30 m（> 躯干半长 0.1881 m），
余下 0.11 m 留给步态摆动与状态读数延迟。0.5 是**保守取整下界**（"不许越出"的边界），不是期望行程。

## 2. 落地内容

1. `profiles/safety/quadruped_lab.yaml`（移动边界的**唯一**来源）：
   `max_accel_mps2=0.5`、`allow_in_place_turn=true`、`max_tilt_moving_deg=15.0`、
   `workspace_m={x/y ∈ [−0.5, 0.5]}`、`watchdog_timeout_ms=100.0`；并新增 `spec.stop_modes`
   （`damped_hold` / `torque_zero_release` 并列，各自必须写明 `applies_to` 适用路径）。
   站立语义的四个键（`max_speed_mps` / `max_yaw_rate_rad_s` / `max_base_translation_m` / `max_tilt_deg`）
   **原样保留**。
2. `config/go2_loopback.yaml`：新增 `locomotion` 段（`ramp_s` / `control_frequency_source` /
   `heartbeat_period_ms` / `watchdog_action` / `damped_hold.{deceleration_source, hold_pose_source}`）。
   该段**不含任何速度/加速度/位移阈值**——阈值只在安全策略里声明一次。
3. `src/iraf_skills/quadruped.py`：
   - `load_movement_limits`：按类型分别校验（正数值 / 布尔 / 矩形 min<max）；
   - `load_stop_modes` + `_validate_stop_modes`：两层语义必须并列登记，未知模式/未知键/空 `applies_to` 均失败；
   - `enforce_velocity_limits(..., movement=…, current_velocity=…, ramp_s=…)`：在既有速度/角速度上限之外
     追加**加速度上限**（按声明斜坡推算）与**原地转弯开关**；参数不足时显式失败，不假定"从零起步"；
   - `enforce_state_freshness`：状态年龄 > `watchdog_timeout_ms` ⇒ 拒绝下发新指令；
   - `check_workspace_moving`：对**实测**位置比对声明矩形 + 移动中倾角（不含偏航，沿用步骤 15 的教训）；
   - `resolve_stop_mode`：移动→`damped_hold`；急停/安全事件→`torque_zero_release`（唯一入口）；
     未移动→机型声明的站立语义；**常规 stop 请求 `torque_zero_release` 一律被拒**；
   - `load_locomotion_declaration`：解析 `locomotion` 段每个"来源"的点号路径，并做两条跨文件自洽门禁：
     `ramp_s × max_accel_mps2 ≥ max_speed_mps`、`heartbeat_period_ms ≤ watchdog_timeout_ms`。
4. `tests/unit/test_quadruped_movement_limits.py`：37 例（正向对照 + 负向），覆盖上表全部门禁。
5. `docs/adr/0008-…md`：状态草案 → **已接受**，补齐口径表、S2 行程口径修正与落地状态。

## 3. 设计决定与理由（含一次"改门禁还是改前提"的判断）

1. **移动语义独立成键，而不是塞进站立键**：`LIMIT_KEYS`（4 个）是"站着有没有被带走"，
   新增键是"能不能按这个速度/方向走"。除语义不同外，类型也不同（布尔 `allow_in_place_turn`、
   映射 `workspace_m` 都不能套用"必须为正数"的规则）。因此**新增独立 loader**
   （`load_movement_limits`），既有 `load_quadruped_limits` 的返回形状**不变**——
   它被既有验收脚本 `scripts/verify_quadruped_skills.py` 与既有契约测试消费，
   改形状会连带改动既有断言，而本步并无"站立语义变了"的事实。两个 loader 共用
   `_load_safety_spec`（同一份 YAML 只解析一次、kind 校验只有一处）。
2. **停止语义：只新增移动语义，不动站立语义**。移动中的 `stop` = `damped_hold`（本步交付解析与门禁）；
   站立态仍是机型声明 `stop.mode: torque_zero_release`（步骤 15/17 已验收的**事实**，
   末速 0.003964950131491022 m/s
   ——⚠ 该数字测于"关键帧四足穿台 18.372 mm"的旧场景，对齐后重测为 **0.0038248382123762478 m/s**，
   旧值自战役 `iraf-24h-2` 提交 `2043111` 起被取代，见 `docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md` §11）。
   硬规则要求"停止语义不得混用"，本步落实为：
   **调用方不得自行选择语义**（`resolve_stop_mode` 是唯一入口）、**非急停路径请求失能停机必须被拒**。
   把站立语义也改成 `damped_hold` 属**另一个变更**（会改动已验收的 stand/stop 数字，触碰逐位一致门禁），
   已在 ADR 决策 3 显式登记为待人工决策项，不在本步偷带。
3. **同一事实只声明一次**：控制频率不写第二份数字，`locomotion.control_frequency_source` 写点号路径
   （`control.frequency_hz`）并由门禁解析；看门狗超时只在安全策略声明，机型声明只声明
   `heartbeat_period_ms` 与"超时后取哪种语义"。跨文件自洽由门禁保证，而不是靠人记得同步。
4. **`applies_to` 是必需的**：只写模式名而不写适用路径的 `stop_modes` 会被拒绝——
   否则"矩阵"退化成两个标签，无法判断某条路径该用哪种语义。

## 4. 本轮踩到的坑（自伤型，已修）

- **`patch` 的 `old_string` 落在 docstring 内部时，必须把闭合 `"""` 一并纳入上下文**：
  第一次插入模块说明时漏了闭合引号，工具把新增段落**当成**docstring 结尾，导致后续正文
  变成 Python 语句（lint 立刻报 `SyntaxError: invalid character '（' (U+FF08)`）。
  教训与 skill 中"编辑工具陷阱"同族：改 docstring 后必须立刻 `python3 -m ast`/lint 验证，
  发现后按字节修回（本次两处小补丁即恢复），不留半成品。

## 5. 诚实边界（未证明的部分）

- 本步交付的是**调度前门禁 + 声明 + 正/负路径证据**，**未接线到运行时**：门禁的调用方是
  步骤 02 的踏步控制器与步骤 03/04 的移动/到点路径。全量单测与 `profile_check --quadruped` 只能证明
  "声明自洽 + 门禁行为正确"，**不能**证明"狗会走"。
- `max_tilt_moving_deg=15°`（源自 ADR 草案建议）与 `max_accel_mps2=0.5`（控制预算推导）**尚无实测复核**，
  必须由步骤 02/03 的实测数值确认；超限时修控制器，不放宽阈值。
- `workspace_m` 由 `handoff_lab` 场景的台面声明推导，**换场景/换本体必须重算**（不是通用常量）。
- 全部结论 `simulation: true`；目标端/真机验收 DEFERRED（板卡不在场）。

## 6. 复跑命令

```
PYTHONPATH=src /usr/bin/python3 build/iraf-24h-2/01/probe-workspace.py
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_quadruped_movement_limits -v   # 期望 37 例全绿
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml; echo exit=$?
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_skill_ttl_gate 2>&1 | tail -3
PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit
```
