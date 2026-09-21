# 02 参数化步态：原地踏步（首期 wave，声明 + 控制器 + 验收）

- 状态：IN_PROGRESS（trot 路线已给出完整判决并入库；**2026-09-21 决策改走 wave**）
- 预估：3~5 个 tick　　归属：本窗口（四足线）
- 依据：ADR-0008（决策 1 已于 2026-09-21 修订：**首期步态 = 准静态 wave**，动态 trot → 二期优化）

## 已完成的实测结论（不要再重复走 trot 调参）

- 动态 trot（duty 0.5）**19 条判据中 11 条失败**：`height_mean_m −38.66691503360235`、
  `height_std_m 50.77553945155333`、`min_base_height_m −174.8562035777835`、`max_tilt_deg 149.00568880619002`、
  `max_displacement_m 3.5763946259283177`、`ctrl_saturated_samples 7306`、`max_tracking_error_rad 0.3101749935977546`、
  四腿稳态支撑相 0.1111/0.1344/0.1789/0.1311（声明 duty 0.5）。
- **26 组参数扫描**（步频×步高×kd）与阻尼/截断上限对照已证明**调参不是出路**；
  `gait.stabilization.enabled=false` 是实测最优档（开阻尼各项更差）——已按此落声明，**判据与阈值未动**。
- 已入库资产（保留，二期 trot 复用）：`src/iraf_adapters/unitree/gait.py`、`unitree_go2.py::trot_in_place`、
  `scripts/verify_go2_trot_in_place.py`、`tests/unit/test_gait_contract.py`、提交 681eaaa。

## 目标（本步重定义）

把 `gait.kind` 从 `trot` 改为 **`wave`**（准静态四相位步态）：duty 0.75、每腿相位偏移 0/0.25/0.5/0.75、
一次只抬一条腿 ⇒ 任意时刻 ≥3 腿支撑；**并实现"逐相位重心转移"**（2026-09-21 决策 A）：支撑腿按相位
改变长度/落点以把重心投影移进当前支撑三角形。原地踏步验收（判据结构不变）全绿后收口。

### 为什么必须带重心转移（已实测，不再重复量）

对称矩形足迹下对角腿连线恒过足迹中心，三腿支撑三角形交集 = 中心一点 ⇒ 支撑余量
`-0.000227483 m`（≈0）；"常量重心平移"可行域为**空集**（两相位所需平移夹角 180.00°）。
⇒ 稳定性不能靠"更宽的固定站姿"，必须逐相位移动躯干/重心。
`support_legs_profile` 门禁（任意相位平均支撑腿数 ≥ 2.6 = duty 0.75×4 − 容差）保持为硬判据。

## 前置

- 步骤 01 完成（移动边界、停止语义、`locomotion` 段已声明）。
- 本步的 trot 判决已入库（上文），不再重跑 trot 调参。
- **足–地接触前提（2026-09-21 授权 A′，必须先做）**：实测关键帧 `home` 下四足足端球穿入台面
  **18.372 mm**（证据 `probe-neutral-clearance.txt`，dist=-0.018372500）、单腿接触力高达 **409 N**
  （站立量级应 ~38 N）。在修掉这个前提之前，任何步态/重心转移结论都不可信。
  要求：① 场景生成按实测把初始位姿抬到足端球最低点贴台面（消除穿透）并写入场景报告；
  ② 声明 `stance_clearance_m`（>0）使中立目标不穿透；③ 接触力探针证明静态四腿 ≈ mg/4；
  ④ 重跑同一隔离实验（步高 0.001 m）与 wave。**判据与阈值不动**；
  ⑤ 修正后 stand/stop 数字会变——必须重测并如实标注"战役 1 的旧数字被取代"，不得静默改数。

## 涉及文件（提交时只 add 这些路径）

- 改：`config/go2_loopback.yaml` 的 `gait` 段：`kind: wave`、`duty_factor: 0.75`、四腿
  `phase_offset` 改为 0/0.25/0.5/0.75、`frequency_hz` 与 `step_height_m` 按 wave 的实测重定；
  **新增 `sway` 段**（决策 A 必需）：`amplitude_m`（躯干/重心逐相位位移幅度）、
  `axis`（先横向 ±y，必要时再叠前后 ±x）、`phase_map`（各腿抬起前把重心推向对侧支撑三角形）、
  `smooth_s`（过渡时长，避免阶跃）。全部数值来自实测扫描，注释写明依据；
  `verification` 判据与阈值**不动**。
- 改：`src/iraf_adapters/unitree/gait.py`：`GAIT_KINDS` 增加 `wave`；相位/支撑判定支持 4 相位；
  支撑多边形检查（≥3 腿支撑）作为 wave 的显式自洽门禁；trot 分支保留。
- 改：`src/iraf_adapters/unitree/unitree_go2.py`：`trot_in_place` 泛化为 `gait_in_place`
  （按 `gait.kind` 分派），旧名保留为薄包装（避免破坏既有调用）。
- 改：`scripts/verify_go2_trot_in_place.py` → 更名 `scripts/verify_go2_gait_in_place.py`，
  旧名留薄包装（打印 `[deprecated]` 后转发）。
- 改：`tests/unit/test_gait_contract.py`（wave 的正/负路径：相位不自洽、duty 越界、支撑腿数不足）。
- 改：`docs/debug/2026-09-21-quadruped-gait-trot-to-wave.md`（新增：trot 判决 → 决策 → wave 设计）。

## 步骤

1. 声明改 `kind: wave` / `duty_factor: 0.75` / 相位 0,0.25,0.5,0.75；自洽门禁要求
   `duty ≥ (n_legs-1)/n_legs`（wave 的静态稳定条件：≥3 腿支撑），不足即失败。
2. 实现 wave 相位与摆动轨迹（支撑腿位形锁定、摆动腿抬升-前移-落地；机身高度由支撑腿维持）。
2b. 实现**逐相位重心转移**：抬起某腿前，把躯干重心投影推向"剩余三腿"三角形的内心方向
   （幅度/时序取声明 `sway`），使 `support_legs_profile` 与支撑余量判据真正成立；
   实现只允许改**支撑腿的目标位形**（长度/落点），不得新增力矩级反馈或姿态控制器
   （那属二期的 trot+平衡器路线，需要 ADR 决策）。
3. 先跑 5 s 短验收看趋势（高度/倾角/饱和），再跑 10 s 正式验收。
4. 若仍不稳：按**声明**做最小必要调整（步频/步高/机身高度），每轮留证据；**不得**改判据阈值。
5. 全绿后：报告 `passed: true` → 归档证据 → 提交 → 台账置 DONE。

## 验收（数字全部来自声明；未达标即 FAILED，不放宽）

```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_gait_in_place.py --config config/go2_loopback.yaml; echo exit=$?   # 期望 0
PYTHONPATH=src /usr/bin/python3 -m unittest tests.unit.test_gait_contract 2>&1 | tail -3                                  # 期望 OK
PYTHONPATH=src /usr/bin/python3 scripts/profile_check.py --quadruped config/go2_loopback.yaml; echo exit=$?               # 期望 0
```

判据（结构不变，仅 duty 目标随声明变化）：10 s 无跌倒（`min_base_height_m` ≥ 声明）、
`height_mean_m` 在容差内、`height_std_m` ≤ 声明、`max_tilt_deg` ≤ 移动期上限、
`ctrl_saturated_samples = 0`、`max_tracking_error_rad` ≤ 声明、四腿支撑相 ≥ `duty − duty_tolerance`、
`clear_swing_cycles` ≥ 下限、对角/四相位结构正确、`max_displacement_m` ≤ 0.05。

## 证据落盘

`build/acceptance/go2-trot-in-place/report.json`（沿用既有声明路径；报告内 `config.gait.kind` 应为 `wave`）、
`build/iraf-24h-2/02/acceptance-*.txt`、`build/iraf-24h-2/02/trace-wave-*.json`

## 提交信息

`feat: 首期步态改为准静态 wave（原地踏步验收通过）`

## 失败 / 阻塞处理

- 若 wave 仍不稳：**先量**支撑腿位形是否真的锁定（接触力/关节跟踪），再决定改声明还是改实现；
  每轮留证据，禁止用"看起来在动"过关，也禁止放宽判据。
- 若需要机身稳定器：说明 wave 的静态稳定性假设不成立，必须回到 ADR 并给出新决策，不得私自加。
- 桌面观看（可选，只用于看）：`DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority MUJOCO_GL=glfw
  PYTHONPATH=src /usr/bin/python3 scripts/view_go2_gait.py --config config/go2_loopback.yaml --seconds 30`
