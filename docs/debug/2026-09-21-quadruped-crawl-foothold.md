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
