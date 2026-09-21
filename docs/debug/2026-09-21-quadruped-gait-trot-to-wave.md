# 四足步态：trot 判决 → 决策改 wave → wave 落地实测（步骤 02）

- 日期：2026-09-21　机器人：宇树 Go2 EDU（仿真）　场景：`build/scenes/handoff_lab/handoff_lab.xml`
- 结论级别：`simulation=true`；真机/目标端验收 **DEFERRED**（板卡不在场）
- 相关：ADR-0008 决策 1（首期=准静态 wave，trot → 二期）、`plans/iraf-24h-2/02-*.md`

## 1. 症状

动态 trot（duty 0.5）19 条判据中 11 条失败（已入库，提交 681eaaa）：机身塌到 −38.67 m、
倾角 149.01°、位移 3.58 m、控制量饱和 7306 次采样。26 组参数扫描（步频×步高×kd）与阻尼/截断
对照**每一组都倒下**，失败形态相同 ⇒ 判定「调参不是出路」，改走准静态 wave（ADR-0008 决策 1）。

本轮把声明改成 wave（duty 0.75、四相位 0/0.25/0.5/0.75）后重跑原地踏步验收：**仍未通过**，
但暴露出一条与实现无关的**声明缺陷**（见 §3）。

## 2. 实测证据链（命令 + 原始数字）

### 2.1 中立足形的穿台深度（决定步高的下界）

```
PYTHONPATH=src /usr/bin/python3 build/iraf-24h-2/02/probe_neutral_clearance.py build/scenes/handoff_lab/handoff_lab.xml
→ 证据 build/iraf-24h-2/02/probe-neutral-clearance.txt
```

- 关键帧 `home`、基座 z = 0.270000000000；**四条腿全部接触台面且穿透 `dist = −0.018372500 m`**
  （`FL/FR/RL/RR` 各一条接触，共 8 条接触里 4 条是足端）。
- 足端球：`r = 0.022`、球心 z = 0.003627500 ⇒ 最低点 z = −0.018372500（台面顶面 z = 0）。
- 结合 trot 扫描的实测（抬脚 ≤ 0.01 m 从未离地、≥ 0.02 m 才离地）⇒ 摆动相要「真的离地」，
  命令抬起量必须明显大于 ~0.017 m。这是 `step_height_m` 从 0.05 提到 0.08 的**实测依据**。

### 2.2 正式声明（wave, sine, 步高 0.08, 1.25 Hz）验收：**不通过**

```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml
→ 证据 build/iraf-24h-2/02/acceptance-wave-run1.txt（exit=5）
```

| 量 | 实测 | 声明判据 |
|---|---|---|
| `height_mean_m` | −62.23686161508254 | 0.27 ± 0.03 |
| `height_std_m` | 71.50775636419188 | ≤ 0.02 |
| `min_base_height_m` | −240.03824818702557 | ≥ 0.15 |
| `max_tilt_deg` | 135.95309870935603 | ≤ 15 |
| `max_displacement_m` | 巨大 | ≤ 0.05 |
| `ctrl_saturated_samples` | 7579 | == 0 |
| `max_tracking_error_rad` | 0.5300321885478345 | ≤ 0.25 |
| `support_legs` min/max | 0.06818181818181818 / 0.5833333333333334 | ≥ 2.6 |
| 四腿支撑相 | 0.11 / 0.105 / 0.142 / 0.124 | \|x − 0.75\| ≤ 0.10 |

失败判据 12/20 条；`leg_*_clear_swing_cycles` 与 `phase_sequence_structure` **通过**（12/12）。

### 2.3 时间线（证据 `build/iraf-24h-2/02/timeline-wave-run1.txt`）

```
t=0.210  z=0.27807  tilt= 0.96°   sat=0  contacts 43.8/25.2/65.7/36.2 N
t=0.410  z=0.27058  tilt= 4.20°   sat=3  contacts 56.5/42.0/ 0.0/75.4
t=0.610  z=0.26719  tilt= 8.03°   sat=3  contacts  0.0/76.9/33.3/189.1
t=2.810  z=0.23069  tilt=19.63°   sat=6  contacts  0.0/302.9/ 0.0/ 0.0
t=3.210  z=-0.13389 tilt=109.63°  sat=7  contacts 全 0（脱离台面，自由落体）
```

- 单足接触力峰值 200~450 N（机体重量 153 N）⇒ **足端在砸台面**；
- 第一个摆动周期（0.41 s）机身就倾 4.20°，之后每 0.2 s（= 一个摆动窗口）一次符号交替的
  滚转振荡，逐步放大到 19.63° 后走出台面；
- 即「接触序列正确但机身站不住」，与 trot 判决**同形**。

### 2.4 对照试验：换 `cosine`（落地速度为零）+ 步高 0.10 → **同形失败**

```
PYTHONPATH=src /usr/bin/python3 build/iraf-24h-2/02/make_trial_config.py cosine 0.10 1.25 build/iraf-24h-2/02/trial-cosine-h010.yaml
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py \
  --config build/iraf-24h-2/02/trial-cosine-h010.yaml --report build/iraf-24h-2/02/trial-cosine-h010-report.json
→ 证据 build/iraf-24h-2/02/trial-cosine-run.txt
```

`height_mean_m −67.61332065178927`、`min_base_height_m −253.58113165758192`、
`max_tilt_deg 175.3677966148357`、`ctrl_saturated_samples 7822`、
`support_legs` 0.06818181818181818 / 0.5416666666666667、四腿支撑相 0.10/0.12/0.146/0.125。

**两组取值失败形态相同（同方向、同数量级）⇒ 被调的自变量（摆动轨迹形状 / 步高）不是原因**，
按战役规则停手，不再扫参数网格。

## 3. 根因（两条，互相独立）

### 根因 A（**可证明的声明缺陷**）：`min_swing_fraction` 与 `duty_factor` 在 duty = 0.75 处互斥

- 判据 `support_legs_profile`（本轮新增）与 ADR 决策 1 都要求「任意时刻 ≥ 3 条腿支撑」
  ⇒ 同一时刻至多 1 条腿离地 ⇒ 对单腿而言「离地比例 a ≤ 1/4 = 1 − duty」。
- 判据 `leg_*_clear_swing_cycles` 要求「一个周期内支撑相比例 ≤ 1 − min_swing_fraction = 0.75」
  ⇒ 「离地比例 a ≥ 0.25」。
- 两式合并 ⇒ **a 必须恰好等于 0.25**：该腿的离地窗口必须**铺满整个摆动窗口**，包括起落瞬间；
  且 ① 落地瞬间接触力必须立刻 ≥ 2 N、② 离地瞬间必须立刻 < 2 N（零卸载时间）。
  这与「连续轨迹两端高度为 0」+「任何有限卸载阈值」矛盾
  ⇒ **该判据在 duty = 0.75 下不可满足**（不是实现没调好）。
- 判据本身没有写反、也不是恒失败：合成序列的正例对照在把边界归属做成确定性后能通过
  （§4.2），说明它只是**零余量**。
- 处理方式（**未私自改**）：按战役规则登记为待人工决策，见 `plans/iraf-24h-2/00-STATUS.json` 的 02 note 与
  `config/go2_loopback.yaml` 里 `min_swing_fraction` 旁的 ⚠ 注释。

### 根因 B（**实测，未定位到根因**）：wave 下机身仍然失稳

- 与 trot 同形：单足接触力 200~450 N（砸台面）、第一个摆动周期即 4.20° 倾角、
  滚转每 0.2 s 符号交替并放大、最后走出台面。
- 已排除：摆动轨迹形状（sine/cosine 两组同形）、步高（0.08/0.10 同形）、
  接触序列结构（实测 `phase_sequence_structure` 通过）、相位/占空比声明（自洽门禁全过）。
- **未证明**：是「机身位移被支撑腿位形锁定」这类结构问题（trot 的同源结论），
  还是接触模型/初始穿台冲击/落地整形的组合。下一步应量的是：离地瞬间的支撑腿位形是否真的锁定
  （逐帧对比支撑腿足端世界坐标 vs 目标）、以及穿台 0.0184 m 的初始冲击是否在第一个周期就注入能量。

## 4. 本轮修复（与验收同文交付）

### 4.1 相位结构判据的**方向错误**（wave 才暴露）

`phase_sequence_structure` 把「后一组相位环按声明相位差平移后与前一组合」写成了 `+shift`，
而腿的相位是 `(u + offset) % 1`（offset 越大窗口在绝对相位轴上越**早**）⇒ 正确关系是
`current[k] == reference[(k + shift) % bins]`（本轮已改正）。
trot 的 Δ = 0.5 在环上**自逆**（`+10 ≡ −10 (mod 20)`），所以这个方向错误在 trot 下完全不可见。
证据：改正前三个相邻对的 `max_abs_diff` 全为 `1.0`，改正后为
`0.006651884700665134 / 0.011961722488038284 / 0.021052631578947323`
（`build/iraf-24h-2/02/diagnose-phase-structure.txt` 与 `…-fixed.txt`）。

### 4.2 合成夹具的相位边界浮点假象

`_samples` 夹具在 `duty = 0.75`、周期 0.8 s、采样 0.01 s 时每 0.04 s 就有一个采样点正好落在
支撑/摆动边界上，而 `1.25·t` 与 `(1.25·t + 0.25) % 1` 的浮点结果会一个落在边界内、一个落在外
（实测 t = 0.40：`u = 0.50000000000000000` 判支撑、`v = 0.75000000000000000` 判摆动；
证据 `build/iraf-24h-2/02/probe-phase-boundary.txt`）⇒ 合成序列出现「同一边界、两条腿归属相反」，
同时打掉相位结构与离地判据。夹具改为按边距归属（`phase < stance_frac − 1e-9`）。
**这不是放宽判据**：阈值一字未改，改的是夹具在边界上的确定性。

### 4.3 泛化与薄包装（声明驱动，不新增第二套实现）

- `gait.py`：`GAIT_KINDS = ("trot", "wave")`；占空比下限与相位结构**按 kind 分派**
  （wave 下限 `(n−1)/n` 由腿数推导，不写死 0.75）；`phase_groups` 改为「按偏移升序的组列表」；
  `trot_joint_targets` → `gait_joint_targets`、`assess_trot` → `assess_gait`（旧名留薄包装）。
- `unitree_go2.py`：`trot_in_place` → `gait_in_place`（旧名薄包装）；
  `scripts/verify_go2_trot_in_place.py` → `scripts/verify_go2_gait_in_place.py`（旧名薄包装、转发退出码）。
- 新增判据 `support_legs_profile`（把「两条腿同时离地」拦下）。

## 5. 复现命令

```bash
cd /home/coretek/AIIRAF
PYTHONPATH=src /usr/bin/python3 build/iraf-24h-2/02/probe_neutral_clearance.py build/scenes/handoff_lab/handoff_lab.xml
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml; echo exit=$?
/usr/bin/python3 build/iraf-24h-2/02/timeline.py build/acceptance/go2-trot-in-place/samples.json 0.2
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_gait_contract          # 51 例 OK
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml; echo exit=$?   # 0
```

## 6. 诚实边界（未证明 / 未完成）

- 步态**未达到**「原地踏步稳定」：本轮交付的是「wave 路线落地 + 判据泛化 + 缺陷定位」，
  不是可用的 wave 控制器；`locomote` 能力**未回填** Profile（验收未通过，铁律 2）。
- 根因 B 只在「失败形态」层面定性，**未定位**到具体结构与相位；没有做任何平衡器/额外控制器。
- 根因 A 是**声明缺陷**，修法需要人工决策（改判据口径 / 改 duty / 加余量），本轮不改。
- 目标端/真机、ROS 2/Nav2/避障/地形越障均不在本步范围；结论只对 `simulation=true` 成立。
