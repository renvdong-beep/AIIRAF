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

---

## 4. 步骤 04（触地冲击/踩不实：声明层 2×2 扫描 + 伴随性判别）：声明层手段到此见底

人工指令：**「1」**（修"踩不实 / 砸地"）—— 只动声明与摆动轨迹语义，有界 4 次运行，零实现改动。

### 4.1 扫描设计（2×2，全部只用现成声明键）

| 轴 | 取值 | 依据 |
|---|---|---|
| `gait.swing_profile` | `sine`（现状） / `cosine` | `sine` 在触地时垂直速度非零；`cosine` 两端高度与**速度**都为 0 |
| `gait.foothold.smooth_s` | 0.2（= 摆动窗全长，现状） / 0.1（窗长一半） | 0.1 ⇒ 水平过渡在摆动相**前半段**完成，后半段以新落点垂直下落（落点先就位、再落地）——**不需要改机制**，`smooth_s` 的语义本就是"过渡时长，超出窗长按窗长计" |

固定：`mode = per_phase`、`stride_m = 0.01`（测量用）、`sway.amplitude_m = 0`（互斥门禁要求）、占位方向。
证据 `build/iraf-24h-3/step04/scan-summary.{txt,json}`。

### 4.2 结果（四格全 `exit=5 / failed=12`：**没有任何判据变绿**）

| 格子 | profile | smooth_s | height_mean_m | max_tilt_deg | 有效窗/首次越限 | 接触达标率区间 | 接触力峰值区间 (N) |
|---|---|---|---|---|---|---|---|
| A | sine | 0.2 | −70.16621288588475 | 161.46858183353564 | 234/1000 @1.1800000000000008 | 43.4~56.3 % | 399.618~425.321 |
| B | cosine | 0.2 | −71.22548782819125 | 177.28092137622377 | 257/1000 @2.4699999999999496 | 48.0~58.1 % | 391.906~428.883 |
| C | sine | 0.1 | −72.19868753807658 | 173.40168725989076 | 238/1000 @1.1500000000000008 | 48.1~55.1 % | 367.827~437.496 |
| D | cosine | 0.1 | −85.5581516543018 | 173.11031267765136 | 206/1000 @1.1300000000000008 | 45.6~54.5 % | 406.663~452.899 |

- A 与步骤 03 重跑**逐项一致**（234/1000 @1.1800000000000008、FL 98/174=56.3%、落点误差 0.001837 m）⇒ 同配置可复算。
- 接触达标率：A 43.4~56.3 %、B 48.0~58.1 %、C 48.1~55.1 %、D 45.6~54.5 % —— 最好与最差相差约 1.7 个百分点，
  而单腿支撑相样本只有 147~197 个 ⇒ **落在噪声量级，不能宣称 cosine / 提前过渡改善了踩实**。
- **接触力峰值没有下降**（最差的 D 反而 452.899 N > A 的 425.321 N）⇒ "砸地"未被这两个手段减轻。

### 4.3 "踩不实"是失稳的伴随现象还是独立缺陷？—— 判别结果：**分辨率不足以判定**

零新增验收运行的判别（`probe_contact_availability.py`，输出 `contact-availability.txt`）：

- 有效窗（tilt ≤ 15°）内：支撑相帧 702，其中无有效接触 347（**49.4 %**）；逐腿 FL 43.7 % / FR 49.7 % / RR 56.6 % / RL 47.3 %。
- **0.1 s 分箱**：前 0.4 s 几乎全接触（0.0 s 箱 2/27=7.4 %，0.1~0.4 s 全为 0.0 %），**从 0.5 s 起跳到 20.0 %**
  并在其后各箱维持 50.0~95.2 %；同期倾角从 0.000° 涨到 4.734°（0.5 s）→ 14.295°（1.1 s）。
  ⇒ 无接触与倾角增长**在同一个 0.1 s 箱里同时开始**，分箱粒度不足以分先后。
- **改用原始 100 Hz 样本**（`contact-availability-samples.json`）：
  · 最早的无接触帧 **t = 0.0300 s**（FL，当时倾角 **0.4910°**、接触力 0.000 N）
  · 倾角首次 >0.5° 在 0.0400 s、>1.0° 在 0.1300 s、>2.0° 在 0.3500 s、>5.0° 在 0.6000 s、>15.0° 在 1.1800 s
  ⇒ 最早无接触**先于**倾角显著增长约 0.01~0.57 s；但 0.03 s 处只有前两帧（启动瞬态，
  `gait.ramp_s = 1.0` 的起坡段），**不能**据此断言因果。
- 全时段（不加倾角门限，含失稳后段）逐腿无接触率 86.7~88.9 %（FL 659/760、FR 662/758、RR 659/741、RL 646/741）。

**结论（不夸大）**：不支持"无接触只是翻倒的伴随现象"（它在倾角还只有 0.5~2° 时就已出现），
但也**没有**证明"踩不实导致翻倒" —— 两者在 0.5 s 起同现，现有量测无法定因果。
定因果需要**干预实验**（把基座固定、隔离整机失稳，只看腿部落点/接触是否恢复）——
那属实现改动（场景/适配器层需要"基座固定"能力），本轮未做。

### 4.4 到期口径判读（沿用战役 2 的纪律：不为同一层反复烧轮次）

声明层可用变量（`swing_profile` × `smooth_s`）已在**一个有界 2×2 内试完**：
失败集合四格完全相同（`failed=12`）、接触率与冲击峰值变化在噪声量级。
⇒ **"踩不实/砸地"不是这两个声明层变量能解决的**，继续在声明层调参＝空转。
本步据此收口，**不**再追加同层扫描；下一步是二选一的人工决策（见 §4.5）。

### 4.5 复跑命令与下一步选项

```
/usr/bin/python3 build/iraf-24h-3/step04/scan_swing_and_transition.py      # 2×2 扫描（4 次运行）
/usr/bin/python3 build/iraf-24h-3/step04/probe_contact_availability.py     # 伴随性判别（1 次运行）
```

- **（a）干预实验**：固定基座隔离整机失稳，判定"踩不实"是否独立可修。需要实现改动
  （基座固定能力要有声明来源，不能写死在脚本里），成本约 1~2 步。
- **（b）收口**：把 C 判 `DEFERRED`。现状：落点机制**可兑现**（误差 1.8~4.9 mm，见 §3.2），
  但整机的"踩不实 + 1.2 s 内失稳"是一个**未分离的复合问题**，声明层已试完。
- **（c）先补专项验收入口** `scripts/verify_go2_crawl.py`（把落点误差 ≤ 5 mm、接触达标率、
  利用周期净漂移/峰位移落成声明化判据），再谈后续 —— 让每一步都有独立判据，而不是靠探针数字。

### 4.6 诚实边界（本步）

- 四格全部 `simulation=true`；`locomote` / `navigate_to` 仍未声明；生产 `foothold.mode` 仍为 `static`。
- 本步**零实现改动**（只改证据区的声明副本），判据与阈值一字未改。
- 接触率、落点误差、冲击峰值都在**有效窗（≤1.18 s 或 tilt ≤ 15°）**内成立，不构成稳定性结论。

---

## 5. 步骤 05（专项验收入口 `verify_go2_crawl.py` + 声明化判据）：判据先于调参

人工指令：**「c」** —— 先补专项验收入口，把落点误差 / 接触达标率 / 周期净漂移 / 峰位移
落成**声明化判据**（含负向对照），让后续每一步按声明判据判，而不是靠探针数字。

### 5.1 交付

| 层 | 位置 | 内容 |
|---|---|---|
| 声明 | `config/go2_loopback.yaml` → `gait.foothold.verification` | 6 个必需键：`report` / `duration_s` / `landing_error_max_m` / `min_stance_contact_rate` / `net_drift_per_cycle_m` / `peak_body_excursion_m`。**两种 mode 都必须声明**（判据描述「crawl 要达到什么」，与当前生效模式无关，生产 static 也如实登记） |
| 判据 | `src/iraf_adapters/unitree/gait.py::assess_crawl` | 13 项判据：`min_base_height_m` / `max_tilt_deg` / `ctrl_saturated_samples` / `net_drift_per_cycle_m` / `peak_body_excursion_m` + 逐腿 `leg_*_landing_error_m`（4）+ 逐腿 `leg_*_stance_contact_rate`（4） |
| 入口 | `scripts/verify_go2_crawl.py` | 与原地入口**同一套采样与生产控制路径**，退出码沿用 0/1/2/3/4/5；含 `--self-check`（合成样本，不跑仿真） |
| 契约测试 | `tests/unit/test_foothold_contract.py` | `Ran 45 / OK`（新增 `FootholdVerificationDeclarationTest` 与 `AssessCrawlFailClosedTest`） |

### 5.2 阈值取值依据（每条都在声明里写明，**没有**为通过而设）

| 键 | 值 | 依据 |
|---|---|---|
| `landing_error_max_m` | 0.000227 | 静态支撑余量实测 **−0.000227483 m** 的量级：落点误差大于它，「把重心移进支撑三角形」的计划就被误差本身吞掉 |
| `min_stance_contact_rate` | 1.0 | 静态可稳要求任意时刻 ≥3 条腿承载（`min_stance_legs = 3`）⇒ 声明为支撑相的帧必须真的接触 |
| `net_drift_per_cycle_m` | 0.005 | 落点位移量 `2·stride_m` 的 1/4；漂移超过它即说明足迹没有闭环（定标条件：首次稳定 crawl 时按实测复核） |
| `peak_body_excursion_m` | 0.10 | 与 ADR-0008 §3 的到点判据（S4 位置误差 0.10 m）同量级，便于二期接力口径一致 |
| `duration_s` | 10.0 | 与原地验收同量级（12.5 个步态周期） |

### 5.3 判据自检（`--self-check`，合成样本，不跑仿真）：**8 / 8 通过，退 0**

好样本必须全过；落点误差超限 / 接触率不足 / 净漂移超限 / 倾角超限 必须各自挂掉对应判据；
缺 `foot_trunk_m`、稳态窗内只有一个落点等级、`static` 模式 必须显式失败（不写半份报告）。

自检里我踩到的三个缺陷（都是"判据看起来有效其实无效/无效其实有效"的同类问题）：
1. **相位浮点边界**：合成样本用构造时的 `elapsed` 算相位，判据用 `time_s − 首帧 time_s` ⇒
   恰落在 `duty` 边界上的帧被两边判成不同相位，好样本也挂在 `leg_*_stance_contact_rate` 上（3 例 FAIL）。
   修法：合成样本改成**两遍构造**，第二遍用与判据完全相同的表达式算相位。
2. **共模偏移不可见**（判据的真实盲区）：注入「所有帧同加常量」时落点判据看不见 ——
   因为该判据测的是**位移兑现**（两等级之差），常量在差分里抵消。
   这条**不是 bug 而是口径**：落足点规划只通过位移改变支撑三角形，差分口径才是与机制对应的量。
   已写进 `assess_crawl` docstring 与声明注释，并登记「中立位绝对一致性当前无判据覆盖」的缺口。
3. **双轴注入**：沿两个轴都加误差时 `hypot(0.0002, 0.0002) = 0.000283 > 0.000227` ⇒ 好样本被判挂。
   修法：误差按**方向**缩放注入（单轴、量级清晰）。

### 5.4 真实验收基线（`build/iraf-24h-3/step05/`，`per_phase` stride 0.01、sway 0）

13 项判据 **13 项失败**，`GO2_CRAWL_ACCEPTANCE_FAILED`，退 5（诚实基线，未放宽任何阈值）：

| 判据 | 实测 | 阈值 |
|---|---|---|
| `min_base_height_m` | −259.880329 m | ≥ 0.15 |
| `max_tilt_deg` | 161.468582° | ≤ 15.0 |
| `ctrl_saturated_samples` | 7842 | = 0 |
| `net_drift_per_cycle_m` | 0.3444890117772212 m/周期 | ≤ 0.005 |
| `peak_body_excursion_m` | 3.6128285110136105 m | ≤ 0.10 |
| `leg_FL_landing_error_m` | 0.013265 m | ≤ 0.000227 |
| `leg_FR_landing_error_m` | 0.006960 m | ≤ 0.000227 |
| `leg_RR_landing_error_m` | 0.012416 m | ≤ 0.000227 |
| `leg_RL_landing_error_m` | 0.007054 m | ≤ 0.000227 |
| `leg_FL_stance_contact_rate` | 0.037500 | ≥ 1.0 |
| `leg_FR_stance_contact_rate` | 0.043818 | ≥ 1.0 |
| `leg_RR_stance_contact_rate` | 0.027375 | ≥ 1.0 |
| `leg_RL_stance_contact_rate` | 0.037097 | ≥ 1.0 |

**与 §3.2 探针数字的差异必须说清（不是矛盾）**：§3.2 的 1.8~4.9 mm 与 43.7~56.6% 是
**tilt ≤ 15° 有效窗**内的数字（用于定位机制），本表的 0.0070~0.0133 m 与 2.7~4.4% 是
**整个稳态窗**（10 s，含翻倒后段）的数字。判据**不给任何豁免**：机器已经翻了，其它判据一起挂是
fail-closed 的正确行为；诊断用的有效窗数字另记，不作为判据口径。

### 5.5 负向对照（入口层）

```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_crawl.py --config config/go2_loopback.yaml
# ⇒ 退 2：「本入口只验收 foothold.mode=per_phase；当前 mode='static'。static 模式下落点恒为
#        中立足端、没有可验收的落点内容，不得判为通过。」
```
即生产 `static` 声明**不能被**本入口判为通过 —— 防的是「零位移 ⇒ 落点误差 0 ⇒ 恒真通过」这条退化路径。

### 5.6 复跑命令

```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_crawl.py --self-check          # 判据自检，退 0
/usr/bin/python3 build/iraf-24h-3/step05/make_declaration.py                       # 生成 per_phase 声明副本
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_crawl.py \
    --config build/iraf-24h-3/step05/per-phase-crawl.yaml \
    --report build/iraf-24h-3/step05/report.json                                  # 真实验收，当前退 5
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_foothold_contract       # 45 例
```

### 5.7 诚实边界（本步）

- 专项验收**当前不通过**（13/13 失败），本步只交付判据与入口，**没有**改善任何数字。
- 既有「原地」20 项判据与全部阈值**一字未改**，两条入口并存、互不放宽；
  生产 `foothold.mode` 仍为 `static`，`locomote` / `navigate_to` 未声明。
- 判据口径是**全稳态窗**（不给 tilt 豁免）；`landing_error` 只测位移兑现，**共模偏移不可见**
  （已在 5.3 第 2 条与声明注释里登记为已知盲区与缺口）。
- 回归：全量单测 `Ran 1096 / failures=1 / errors=4 / skipped=4`（与战役基线同批 5 条：
  4 项 loader ERROR + `test_vision_processing.test_depth_projection_and_invalid_filter`，新增 0 / 消失 0）；
  判据自检 8/8 退 0；入口负向对照退 2。既有「原地」20 项判据与全部阈值一字未改。
- 全部结论 `simulation=true`；真机/目标端 `DEFERRED`。

---

## 6. 步骤 06（方向图案扫描）：假说排除，且得到一个反直觉结果

人工指令：**「1」** —— 扫方向图案，判决「翻倒以滚转为主是否由占位方向（纯侧向 ±y）造成」。
有界 3 格（只改 `phase_direction_map`，stride 0.01、sway 0、smooth_s 0.2），用**生产判据入口**
`scripts/verify_go2_crawl.py` 跑（判据口径 = 全稳态窗；诊断口径 = tilt ≤ 15° 有效窗）。
证据 `build/iraf-24h-3/step06/scan-summary.{txt,json}`。

### 6.1 结果

| 图案 | exit | failed | roll_max（全程 / 有效窗） | pitch_max（全程 / 有效窗） | max_tilt_deg | 净漂移/周期 | 有效窗/首次越限 |
|---|---|---|---|---|---|---|---|
| `P_y_lateral`（现占位） | 5 | 13 | 176.492° / 13.669° | 88.536° / 11.165° | 161.468582 | 0.344489 | 234/1000 @1.1800000000000008 |
| `P_x_uniform`（纯前后） | 5 | 13 | **179.611°** / 14.276° | 81.923° / 8.730° | 173.831585 | 0.745258 | 218/1000 @0.9600000000000007 |
| `P_diag_out`（对角外张） | 5 | 13 | 179.987° / 13.799° | 88.634° / 11.051° | 178.096772 | 0.462855 | 243/1000 @1.7600000000000013 |

逐腿（有效窗内，接触达标率 / 接触力峰值 / 落点误差）：

| 图案 | FL | FR | RR | RL |
|---|---|---|---|---|
| `P_y_lateral` | 0.5632 / 409.865 N / 0.001837 m | 0.5028 / 425.321 N / 0.004891 m | 0.4341 / 399.618 N / 0.004885 m | 0.5266 / 420.658 N / 0.003029 m |
| `P_x_uniform` | 0.5444 / 434.320 N / 0.004258 m | 0.4500 / 468.630 N / 0.006225 m | 0.5617 / 421.611 N / 0.005417 m | 0.4847 / 381.840 N / **0.015664 m** |
| `P_diag_out` | 0.6011 / 430.385 N / 0.003632 m | 0.4541 / 420.329 N / 0.004386 m | 0.4526 / 401.299 N / 0.008840 m | 0.5614 / 381.960 N / 0.005917 m |

### 6.2 判读（按事先写死的口径）

- **假说排除**：换成纯前后方向后 roll **没有**塌下来（179.611° vs 176.492°），pitch 也只从 88.536° 略降到 81.923°；
  三档**全部** roll 主导（全程 roll 176~180°、pitch 82~89°）⇒ 「翻倒由占位方向造成」**不成立**。
- **反直觉结果（登记，别凭直觉调参）**：占位方向（纯侧向）在**落点兑现**上反而**最好**
  （误差 0.001837~0.004891 m），纯前后最差（0.004258~0.015664 m，RL 达 0.015664 m —— 是侧向的 5 倍）。
  ⇒ 「占位方向差」这个直觉是错的；方向图案确实影响落点精度，但方向与失稳类别无关。
- 三档 `failed` 全部 **13**、`exit=5` ⇒ 判据全挂，方向不改变任何判据的结论；
  有效窗 0.9600000000000007~1.7600000000000013 s（`P_diag_out` 撑最久，`P_x_uniform` 最短），
  差异不足以构成"某方向更稳"的证据（无重复次数、样本 218~243）。

### 6.3 到期口径判读（第二次触发）

声明层可用变量到此**全部试完**：`swing_profile` × `smooth_s`（步骤 04）、`stride_m`（步骤 03）、
`phase_direction_map`（本步）。三组扫描**无一**改变失败类别（每次 `failed` 都不变、都是 13 项）。 
⇒ 声明层已见底，**不再追加**同层扫描。根因不在声明层这两个变量的取值上，而在更底层
（腿本身的接触建立 / 整机失稳机制），要定因果只能做**干预实验**。

### 6.4 复跑命令

```
/usr/bin/python3 build/iraf-24h-3/step06/scan_directions.py     # 3 格方向图案扫描（3 次运行）
```

### 6.5 诚实边界（本步）

- 三格都 `simulation=true`、都 `exit=5 / failed=13`；生产 `foothold.mode` 仍为 `static`。
- 本步零实现改动（只改证据区的声明副本）；判据与阈值一字未改。
- 有效窗数字（roll 13.669~14.276°、pitch 8.730~11.165°）说明：**在 tilt 越限之前，roll 与 pitch
  各自都还在 15° 内**，越限是两者的合成量 —— 这条与"失稳从 0.5 s 起同步出现接触丢失"（§4.3）
  是同一条线索的两个侧面，仍**未**定位根因。

---

## 7. 步骤 07（干预实验：基座 weld）：设计缺陷暴露，两项判别口径均作废

起因：使用者问「现在能 MuJoCo 查看效果了吗」。先做了可视化（`scripts/view_go2_gait.py` +
`build/iraf-24h-3/step07/display-{crawl,static}.yaml`，`duration_s` 压到 3 s）：
crawl 渲染 627 帧 / 300 采样、static 632 帧 / 300 采样，`display_only: true`；
窗口在 `:0` 上确认可见（`xwininfo` 报 `"MuJoCo : go2" 1280x720`），自限时退出、无驻留进程。
随后按上一轮写死的分支条件直接开做**干预实验**（weld 固定基座）。

### 7.1 器材与做法（零仓库改动）

从生产场景 XML 复制一份，**只插入 `<weld body1="base_link" />`**（MuJoCo 的 weld 无 body2
即焊到世界），声明副本只改 `model.file` / `model.builder` 指向它；同一条专项验收、
同一份判据与阈值。器材在证据区（`build/iraf-24h-3/step08/`），明确标注为**验证器材、非能力声明**。

### 7.2 结果

| 配置 | exit | failed | max_tilt_deg | min_base_height_m | 净漂移/周期 | 峰位移 | `ctrl_saturated_samples` |
|---|---|---|---|---|---|---|---|
| `control_base_free` | 5 | 13 | 161.468582 | −259.880329 | 0.3444890117772212 | 3.6128285110136105 | 7842 |
| `intervention_base_fixed` | 5 | **9** | **0.086955** | 0.3060332038487615 | **8.58758186441274e-05** | **0.0010366524102600541** | 6485 |

（`failed` 从 13 降到 9：`min_base_height_m` / `max_tilt_deg` / `net_drift_per_cycle_m` /
`peak_body_excursion_m` 四项变绿 —— 但见 7.3，这不是能力的证据。）

### 7.3 必须先说清的三处问题（我先错了两处）

1. **快速读数被污染（我错）**：两次运行写的是**同一个** `samples.json`（`report_path.with_name("samples.json")`），
   我据此得出"对照组 FL 全 0 接触"是**错的** —— 那是固定基座那次的数据。
2. **实验设计缺陷（我错）**：weld 把机身重量接到焊点上 ⇒ **接触力必然 ≈ 0**，
   所以"接触达标率 0.0000"**主要**是器材造成的，**不能**当作"腿的缺陷"证据。
   因此上一步写死的分支判据（"接触率 ≥0.95 → ..."）**设计有误、无法回答原问题**。
   同理，`max_tilt_deg 0.086955°` 是**焊点强制的**，不是"目标几何不产生倾覆"的证据；
   只有 `net_drift` / `peak_excursion` 变小是器材的必然结果，不能当结论。
3. **派生的"到地间隙"判别无效（我错）**：用 `(base_height + foot_trunk_z) − 球半径 0.022`
   反推足端是否贴地，结果与接触力**完全不相关**：0~0.5 s 内 36/36 个"离地 >2 mm"的帧
   仍报 ≥2 N，穿透帧 55/55 有力 —— 说明该派生量**不可用**（躯干系→世界系的反推遗漏了
   姿态/偏移，2 mm 量级的判别承受不住）。
   ⇒ **结论**：以后要测"是否真的踩到地"，必须直接用**模型自身的接触状态**
   （接触点数 / 穿透量，例如从 `mjData` 的接触列表按 geom 取），不能靠位置反推。

### 7.4 本步仍然可用的两项收获

1. **落点误差中有一部分与整机失稳无关**：基座固定时（机身不能翻、不能漂），四腿落点误差仍有
   **0.001324~0.003314 m**（6~15 倍于声明上限 0.000227 m），且 `ctrl_saturated_samples` 仍有 **6485 / 10000**。
   ⇒ 腿部层的落点兑现**本身**不达标（不能全部归因于"机器翻了"）。
2. **力矩触顶是普遍现象（新线索）**：自由基座 7842/10000、固定基座 6485/10000 的采样都存在执行器
   限幅截断。此前只把"饱和"当"目标越界"的附属登记，未当成独立线索 —— 它现在指向
   「目标的力矩需求普遍超出执行器能力（或 PD 增益/重力前馈与声明的目标不匹配）」。

### 7.5 复跑命令

```
/usr/bin/python3 build/iraf-24h-3/step08/base_fixed_probe.py     # weld 前后对照（2 次运行）
DISPLAY=:0 XAUTHORITY=<桌面授权> MUJOCO_GL=glfw PYTHONPATH=src \
    /usr/bin/python3 scripts/view_go2_gait.py \
    --config build/iraf-24h-3/step07/display-crawl.yaml --seconds 30     # 可视化（自限时）
```

### 7.6 诚实边界（本步）

- 全部结果 `simulation=true`；生产 `foothold.mode` 仍为 `static`，`locomote` / `navigate_to` 未声明；
  仓库**零改动**（器材与被改声明副本都在 gitignored 的证据区）。
- **本步没有回答"踩不实是否独立"**：原设计（weld）无法回答，且两项替代判别口径均被实测判为不可用。
- 有效窗内的旧数字（§3.2 的 43.7~56.6%、1.8~4.9 mm）仍然只有"未失稳窗口"内的意义，未变。






