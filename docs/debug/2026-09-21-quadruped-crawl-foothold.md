# 四足落足点规划（迈步式 crawl）专项：调试记录

- 专项决策来源：2026-09-21 人工选 **C**（改判落足点规划），计划见 `.hermes/plans/2026-09-21_quadruped-crawl-foothold.md`
- 前序：战役 `iraf-24h-2`（`docs/progress/2026-09-21-quadruped-locomotion.md`、`docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md`）
- 全部结论 `simulation=true`；真机与目标端 `DEFERRED`（板卡不在场）

---

## 1. 步骤 01（声明段 + 契约门禁）：先落声明，实现未动

### 1.1 目标与范围

按「声明先于实现」（AGENTS.md 5.3 / 铁律 1）先落地 `gait.foothold` 声明段与全套门禁，
**不改任何步态行为**：本步只让「落点规划」有唯一的声明来源与 fail-closed 校验。

### 1.2 交付

| 层 | 文件 | 内容 |
|---|---|---|
| 声明 | `config/go2_loopback.yaml`（`gait` 段内新增 `foothold`，行 422~437） | `mode: static`（落点恒为中立足端 = 现行「原地踏步」行为）；注释写明 `per_phase` 的物理依据（静态余量 −0.000227483 m、重心到支撑三角形形心需平移 0.079~0.081 m）与「未实现不得声明」 |
| 解析 | `src/iraf_adapters/unitree/gait.py` | 常量 `FOOTHOLD_MODES` / `FOOTHOLD_MODE_KEYS` / `REQUIRED_FOOTHOLD_KEYS`；`_load_foothold`（按模式判必需键与**多余键**、相位环顺序、方向单位化、两个水平位移源互斥）；窄入口 `load_foothold_declaration`；抽出 `phase_groups_of` 供两条路径共用相位分组 |
| 门禁 | `scripts/profile_check.py` | **门禁 9**：用窄入口独立复核 `gait.foothold`，登记 `mode`/`stride_m`/`phase_order`/`sway.amplitude_m`，`wave` 缺段即失败 |
| 契约测试 | `tests/unit/test_foothold_contract.py`（新增，27 例） | 正例（生产 static / per_phase / 归一化 / `sway=0` 合法侧）、负例（缺段/缺键/非法 mode/static 带无定义键/缺 stride/环形顺序错/零矢量/trot+per_phase/互斥违规）、窄入口独立性 |

### 1.3 关键设计点（为什么这样切）

1. **按模式判必需键与多余键**：`static` 只允许 `mode`（落点参数在该模式下无定义 ⇒ 写进声明等于假声明）；
   `per_phase` 必需 `stride_m`/`phase_direction_map`/`phase_order`/`ramp_s`/`smooth_s`。与 `sway` 的
   「对某步态类型无定义即声明失败」同取向。
2. **相位环顺序门禁**：`phase_order` 必须等于按 `phase_offset` 升序的腿序列（实测 `['FL','FR','RR','RL']`）。
   顺序错会让落点方向逐窗口翻转 —— 典型静默失效（门禁全绿、机器人朝反方向走），该缺陷族在 `sway`
   的隔离实验里踩过一次（前序调试记录 §9）。
3. **两个水平位移源互斥**：`per_phase`（移动落点）与 `sway.amplitude_m > 0`（平移机身）不得同时启用，
   否则失稳无法归因。合法侧（`sway` 段保留、幅度为 0）有正例对照，证明该门禁不是恒真门禁。
4. **窄入口而不复用完整解析**：落足点门禁只消费 `gait` 段自身（腿标识 + 相位偏移）。
   理由见 1.5 的缺陷 2。

### 1.4 验收与证据（`build/iraf-24h-3/step01/`）

| 项 | 命令 | 结果 |
|---|---|---|
| 契约测试 | `PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_foothold_contract` | **Ran 27 / OK** |
| 门禁正/负向对照（10 例） | `/usr/bin/python3 build/iraf-24h-3/step01/check_foothold_gate.py` | **用例 10，通过 10，失败 0**（退 0）；报告 `foothold-gate-negative.txt` |
| 门禁 9 登记字段 | 同上，生产声明 | `{"declared": true, "mode": "static", "stride_m": null, "phase_order": null, "sway_amplitude_m": 0.06}` |
| 全量单测 | `PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit` | **Ran 1078 / failures=1 / errors=4 / skipped=4** |
| **逐位一致（本步最关键）** | `PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml` | **assessment 41 个可比字段差异 0 条**；20 项判据、13 条失败清单逐项相同 |

逐位一致的对照方式：改动前 19:34 的生产报告快照为 `pre-change-report.json`，改动后重跑同一条验收，
逐字段比对（MuJoCo 定步长仿真 ⇒ 数字可复算）。关键数字（改动前 → 改动后，逐位相同）：

- `height_mean_m` −52.96097353575173 → −52.96097353575173
- `max_tilt_deg` 176.44460017100423 → 176.44460017100423
- `max_displacement_m` 6.6266884683075515 → 6.6266884683075515
- `min_base_height_m` −215.6303346878428 → −215.6303346878428
- `max_tracking_error_rad` 0.3111402167153192 → 0.3111402167153192
- `ctrl_saturated_samples` 7486 → 7486
- `support_legs` {min 0.06818181818181818, max 0.6444444444444444} → 逐位相同

回归口径：战役基线 `844/1/4/4` → 本轮 `1078/1/4/4`，失败集合**同批 5 条**（4 项 loader ERROR +
`test_vision_processing.test_depth_projection_and_invalid_filter`），**新增 0 / 消失 0**。判据与阈值一字未改。

### 1.5 本步踩到并修掉的三个缺陷（都不是理论问题，是实测触发）

1. **缺必需键以 `KeyError` 泄漏**：首版只校验公共键 `mode`，`per_phase` 缺 `stride_m`/`phase_order` 等
   会在后续按下标取值时抛 `KeyError` —— 那是**崩溃**而不是声明层显式失败（调用方拿不到失败原因，
   也无法映射退出码）。修法：必需键**按模式**先判齐（公共键 ∪ 该模式专属键），再进入取值。
   由 `test_missing_required_keys` 触发。
2. **门禁 9 误报（设计问题）**：首版门禁用完整解析 `load_gait_declaration(declaration, profile.joints)`，
   而生产消费路径（`unitree_go2.py:419` / `_gait_parameters`）用的是**模型的** `joint_order`，且是
   **惰性**解析（装配期只校验通用键，刻意让「没有步态声明的本体」也能装配）。夹具 Profile 的关节名与
   真实 Go2 不同 ⇒ 两份既有用例 `test_cli_json_contract` / `test_positive_control` 由通过变失败
   （`Ran 1072 / failures=3`）。修法：抽出窄入口 `load_foothold_declaration`，只消费 `gait` 段自身；
   并新增 `test_does_not_depend_on_joint_bindings` 锁死该性质（bogus 关节名下窄入口仍通过、宽入口失败）。
3. **负向对照 harness 自身的缺陷（证据方法问题）**：`N1_missing_section` 的 mutate 写成
   `lambda d: d["gait"].pop("foothold")` —— `pop` 返回被弹出的**值**，落盘声明退化成 `{"mode": "static"}`，
   用例命中「缺顶层键」而不是「缺 foothold」，但**退出码同样是 1** ⇒ 「退出码正确」不足以证明门禁命中了
   目标缺陷。已改为正常函数并保留关键词断言（`期望退 1 + 失败原因命中关键词`）。

### 1.6 复跑命令

```
# 契约测试
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_foothold_contract

# 门禁 9 正/负向对照（10 例，退 0 = 全部按预期）
/usr/bin/python3 build/iraf-24h-3/step01/check_foothold_gate.py

# 机型声明自检（门禁 1~9）
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml

# 全量单测（计数应与 1078/1/4/4 一致）
PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit

# 逐位一致对照（应与 build/iraf-24h-3/step01/pre-change-report.json 逐字段相同）
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml
```

### 1.7 诚实边界（本步）

- 本步**没有让机器狗走得更好**：`foothold.mode` 仍是 `static`，行为与改动前逐位一致；
  `locomote` / `navigate_to` 仍未声明（未回填 Profile）。
- 「落足点规划可行」**未被证明**：本步只建立声明与门禁；`per_phase` 的目标生成与摆动相落点尚未实现，
  最大未知仍是**黏滑**（前序设计包 P4：失稳驱动可能是位置级目标本身）——要到步骤 05 的 L1 实跑才有数字。
- 步骤 01 的 `per_phase` 声明值（`stride_m` 等）**尚无实测标定**：`0.079~0.081 m` 来自几何/分配器推导，
  实际取值由步骤 05 的扫描实测确定，届时在声明里写明取值依据（同 `sway.amplitude_m` 的做法）。

---

## 2. 步骤 02（相位相关落点插值）：落点真正进入目标角，且 `static` 逐位不变

### 2.1 目标与范围

把落点从「声明里的字段」变成**目标生成的实际输入**（否则声明是死声明），同时保证
`mode=static` 的目标角与改动前**逐位一致**（既有原地踏步路径零回归）。

### 2.2 交付

| 层 | 位置 | 内容 |
|---|---|---|
| 机制 | `src/iraf_adapters/unitree/gait.py::foothold_offset_m(params, code, elapsed_s)` | 返回该腿水平落点偏移 `(dx, dy)`（躯干系）。`static` ⇒ **精确的** `(0.0, 0.0)`；`per_phase` ⇒ 支撑相保持「上一周期摆动结束时落下的等级」，摆动相按升余弦过渡到「本周期等级」，等级逐周期 `±1` 交替 |
| 接线 | `gait_joint_targets`（x 目标 `... - sway[0] + foothold_x`、y 目标 `damping[1] - sway[1] + foothold_y`） | 把落点接到 `leg_solve` 的 `(px, py)`；`foot_offset` 仍只负责抬脚 z |

### 2.3 机制的两个设计点（为什么这样定义）

1. **周期边界必须连续**：支撑相保持的是「上一周期摆动结束落下的等级」（= `−sign(cycle)`），
   摆动相从该等级过渡到本周期等级（= `+sign(cycle)`）。这样在 `u = duty`（摆动起点）与
   `u → 1`（摆动终点）两处都与相邻窗口衔接，不产生阶跃。首版写成「支撑相 = 本周期等级」，
   在周期边界会跳变一个步幅（2·stride = 0.16 m），属会直接甩翻机身的缺陷。
2. **两周期闭环**：等级逐周期交替 ⇒ 落点在 2 个周期后回到同一位置，足迹闭环、净漂移 0
   （"原地"语义），而支撑三角形的**形状**逐相位改变 —— 这正是要测的东西。
3. **过渡时长可声明**：`smooth_s` 超过摆动窗口长度时按窗口长度计（**不越窗**）；幅度按 `ramp_s`
   线性建立（`0` = 首次抬腿前满幅）。摆动窗口定义与 `sway` 共用 `sway_windows`（窗口只由
   `duty_factor` 与相位偏移决定）。

### 2.4 验收与证据（`build/iraf-24h-3/step02/`）

| 项 | 结果 |
|---|---|
| 契约测试 | `Ran 35 / OK`（含 `test_static_offset_is_exact_zero`、`test_static_joint_targets_are_bit_identical_to_previous_formula`） |
| 数学级逐位一致 | `static` 下 4 条腿 × 40 个时刻，目标角与「改动前表达式」**逐位相等**（`assertEqual`，非近似） |
| **实测逐位一致** | 同一条生产验收（`verify_go2_gait_in_place.py`）改动前 19:34 快照 vs 改动后：assessment **41 个字段差异 0 条**，20 项判据与 13 条失败清单逐项相同 |
| **存活性（分支真被执行）** | 把声明换成 `per_phase`（`build/iraf-24h-3/step02/per-phase-probe.yaml`）后同一条验收：**41 个字段中 27 个变化** ⇒ 落点确实进入了控制回路，不是死代码 |
| 全量单测 | `Ran 1086 / failures=1 / errors=4 / skipped=4`（基线同批 5 条，新增 0 / 消失 0） |
| 生产报告恢复 | `build/acceptance/go2-trot-in-place/report.json` 已按生产命令重跑（`report_path` 指回生产路径），未留 `--report` 覆盖痕迹 |

`static` → `per_phase` 的关键数字对比（同一条验收、同一场景）：

| 指标 | `static` | `per_phase`（首轮，未调参） |
|---|---|---|
| `height_mean_m` | −52.96097353575173 | 0.0778459128507211 |
| `height_std_m` | 63.78010487892247 | 0.0311160555974537 |
| `min_base_height_m` | −215.6303346878428 | 0.06700140292746969 |
| `max_displacement_m` | 6.6266884683075515 | 0.41062571823587596 |
| `max_tilt_deg` | 176.44460017100423 | 179.97546530052387 |
| `max_pitch_deg` | 86.10174714239811 | 14.043828174104641 |
| `ctrl_saturated_samples` | 7486 | 8449 |
| `per_leg.FL.steady_stance_fraction` | 0.09 | 0.0 |

**读法（诚实）**：`per_phase` 首轮**仍翻倒**（`max_tilt_deg` 179.97546530052387°），
且 `FL` 的稳态支撑相比例掉到 `0.0` ⇒ 该腿在整段里没有形成有效接触（侧向落点把足端带离地面）。
本步只交付机制；`support_legs` 耦合与摆动落点的接地是步骤 03，L1 静态可稳的判定与调参是步骤 05。
**不得**用本步数字宣称任何稳定性改善。

### 2.5 本步踩到的缺陷

1. **周期边界跳变**（见 2.3 第 1 点）：支撑相等级取错，边界处跳 2·stride。
2. **测试自身的两个断言错误**（记账，避免下次误判为"实现回归"）：
   ① `static` 逐位一致用例里把 `sway_offset_m` 在循环外只算了一次，而它与相位相关 ⇒ 出现
   `0.2229... vs 0.1806...` 的假差异；② 「摆动窗内单调」写成 `all(delta <= 1e-12) and all(delta >= -1e-12)`
   （实为「近似不变」），且把落点方向的符号写反。两处都先用**只读探针**
   （`build/iraf-24h-3/step02/probe_foothold_trajectory.py`）量出真实轨迹后才改断言，
   没有为了通过测试放宽任何门禁。

### 2.6 复跑命令

```
# 契约测试（含逐位一致与存活性的单元层）
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_foothold_contract

# 落点轨迹只读探针（打印 FL 腿一个周期的落点）
/usr/bin/python3 build/iraf-24h-3/step02/probe_foothold_trajectory.py

# 实测逐位一致（static）：应与 build/iraf-24h-3/step01/pre-change-report.json 逐字段相同
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml

# 存活性（per_phase）：应与上一条数字**不同**
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py \
    --config build/iraf-24h-3/step02/per-phase-probe.yaml \
    --report build/iraf-24h-3/step02/report-per-phase.json
```

### 2.7 诚实边界（本步）

- `foothold.mode` 生产值仍为 `static`（`per_phase` 只在证据区的声明副本里启用）⇒
  生产路径**无行为变化**，`locomote` / `navigate_to` 仍未声明。
- `per_phase` 的落点方向与 `stride_m = 0.08` 是**待标定的初值**（方向为契约测试用的占位，
  几何依据只有「重心到支撑三角形形心需平移 0.079~0.081 m」这一条推导），必须由步骤 05 的
  扫描实测确定；本步没有声称该取值可用。

---

## 3. 步骤 03（落点可达性测量 + 相位约定修复）：抓到并修掉一个真缺陷

人工指令：**「2 优先、1 并行」** —— 2 = 测落点可达性（不判整机稳定性），1 = 补对照消掉隔离实验的混淆项。

### 3.1 任务 1（并行）：混淆项消掉了，而且给出一个强不变量

补两条**合法**对照（不改任何门禁语义）：`static + sway 0` 与 `per_phase(stride 0) + sway 0`
（后者是扫描里同一格的重跑，用于确定性核对）。证据 `build/iraf-24h-3/step03/control-summary.{txt,json}`：

| 配置 | exit | failed | height_mean_m | max_tilt_deg | max_displacement_m |
|---|---|---|---|---|---|
| 生产 static（sway 0.06） | 5 | 13 | −52.96097353575173 | 176.44460017100423 | 6.6266884683075515 |
| C1 `static + sway 0` | 5 | 12 | **−78.17415831383013** | **155.90719322558672** | **5.646352472569747** |
| C2 `per_phase(stride 0) + sway 0` | 5 | 12 | **−78.17415831383013** | **155.90719322558672** | **5.646352472569747** |
| 扫描 `stride 0`（首次） | 5 | 12 | −78.17415831383013 | 155.90719322558672 | 5.646352472569747 |

两条结论：
1. **C1 与 C2 逐位相同（11/11 字段）** ⇒ 零位移时 `per_phase` 通路与 `static` 通路**逐位等价**：
   落点机制在「不产生位移」时不引入任何行为差异（比契约测试更强的内部一致性证据）。
2. 扫描里 `stride 0` 对照的翻倒**归因于「去掉 sway 幅度 0.06」**，不是落点机制 ——
   混淆项消除。确定性核对：C2 与扫描同一格 11/11 字段逐位相同 ⇒ 仿真可复算，逐位比对方法成立。

### 3.2 任务 2：落点可达性 / 可兑现性（不判整机稳定性）

新增采样量 `foot_trunk_m`（足端**接触几何**在**躯干系**的位置；世界系会随机身漂移而量不出落点误差），
测量只用生产入口的产物，计划落点由声明用 `foothold_offset_m` 同一函数算出。探针
`build/iraf-24h-3/step03/probe_reachability.py`，输出 `reachability.txt` / 重跑 `reachability-rerun.txt`。

**有效性前提（必须先看）**：只统计 `tilt_deg ≤ 15°`（= `max_tilt_moving_deg`）的样本：

| 配置 | 有效样本 | 首次越限时刻 |
|---|---|---|
| `per_phase` stride 0.01 + sway 0 | 234 / 1000 | **1.1800 s** |
| `static + sway 0` | 212 / 1000 | 1.1200 s |
| 生产 `static`（sway 0.06） | 322 / 1000 | 2.3500 s |

⇒ 整机在 1.12~2.35 s 内就超过 15°：下面的落点数字**只覆盖前约 1.5 个步态周期**。

**发现的缺陷（相位约定符号）**：首轮测量里 `FR`/`RL` 的支撑相样本中出现「未达等级」的过渡帧
（n=28 / n=21），而 `FL`/`RR` 没有。判据：`foothold_offset_m` 写成 `raw = elapsed/period − offset`，
而生产相位约定（`leg_phase`）是 `+ offset` ⇒ 落点交替窗口相对抬腿窗口错开 **2·offset**；
`2·0.0 = 0`、`2·0.5 = 1 ≡ 0`（FL/RR 恰好对齐），`2·0.25 = 0.5`、`2·0.75 = 1.5 ≡ 0.5`（FR/RL 错半个周期）
—— **与实测签名逐条吻合**。已修（`+ offset`），修复后重跑：**FR/RL 的过渡帧消失**，四腿误差带收敛。

**落点兑现（修复后，`stride_m = 0.01` ⇒ 相邻周期落点差应为 `2·stride = 0.020000 m`）**：

| 腿 | 实测落点差 | 声明 | 落点误差 | 占声明量 |
|---|---|---|---|---|
| FL | (+0.000425, −0.021787) | (0, −0.020000) | 0.001837 m | 9.2 % |
| FR | (+0.003661, +0.016756) | (0, +0.020000) | 0.004891 m | 24.5 % |
| RR | (+0.003476, −0.016567) | (0, −0.020000) | 0.004885 m | 24.4 % |
| RL | (+0.002717, +0.018662) | (0, +0.020000) | 0.003029 m | 15.1 % |

读法：落点**方向与量级都对**（y 分量实测 16.6~21.8 mm vs 声明 20 mm），残差 1.8~4.9 mm
（不足 5 mm），其中含约 3 mm 的 x 向串扰（声明为 0）。⇒ **Q2 的答案是「能兑现，误差 ≤ 5 mm」**。

**Q1（摆动腿是否真的落地）**：有效窗内支撑相样本的接触达标率 `FL 56.3% / FR 50.3% / RR 43.4% / RL 52.7%`
（阈值 2.0 N），接触力峰值 399.618~425.321 N（≈ 2.6~2.8 倍 mg = 153.000762480 N，说明是"砸"下去的）。
⇒ 有近一半的支撑相帧**没有有效接触** —— 腿没踩实，这是步骤 03 之后要解决的下一件事。

### 3.3 本步记账的两个缺陷

1. **实现**：`foothold_offset_m` 的相位约定符号（见 3.2），修前会静默地把落点交替错半个周期。
2. **探针自己**：首版 `planned_gap` 写成 `−2·stride·方向`（正确为 `+2·stride·方向`）⇒ 打印出的
   「误差 0.041839/0.046597/0.042217/0.045710 m」里混进了 `2·stride = 0.020000 m` 的常量偏置。
   按正确符号重算的真实误差就是上表的 1.8~4.9 mm。**该偏置已修**，且这条再次印证：
   「数字能打印出来」不等于「数字是对的」。

### 3.4 复跑命令

```
# 任务 1：消混淆项的对照（3 次运行）
/usr/bin/python3 build/iraf-24h-3/step03/control_confound.py

# 任务 2：落点可达性（3 次运行，输出 reachability.txt）
/usr/bin/python3 build/iraf-24h-3/step03/probe_reachability.py

# 契约测试（含相位约定鲁棒性）
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_foothold_contract

# 全量单测
PYTHONPATH=src /usr/bin/python3 -m unittest discover -s tests/unit -t tests/unit
```

### 3.5 诚实边界（本步）

- 落点数字只在**未失稳窗口**（≤ 1.18 s ≈ 1.5 个周期）内成立；整机在 1.12~2.35 s 内已越 15°，
  **本步不构成任何稳定性结论**。
- `stride_m = 0.01` 是**测量用**取值（为了把落点误差与失稳分开），不是标定值；
  生产 `foothold.mode` 仍为 `static`。
- 接触达标率不足 100% 与 399.618~425.321 N 的冲击峰值只作为**待处理事实**登记，未归因。
- 全量单测 `Ran 1086 / failures=1 / errors=4 / skipped=4`（与基线同批 5 条，新增 0 / 消失 0）；
  判据与阈值一字未改。


