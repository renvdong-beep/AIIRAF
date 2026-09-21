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
一次只抬一条腿 ⇒ 任意时刻 ≥3 腿支撑、**静态稳定、不需要平衡器**；原地踏步验收（判据结构不变）全绿后收口。

## 前置

- 步骤 01 完成（移动边界、停止语义、`locomotion` 段已声明）。
- 本步的 trot 判决已入库（上文），不再重跑 trot 调参。

## 涉及文件（提交时只 add 这些路径）

- 改：`config/go2_loopback.yaml` 的 `gait` 段：`kind: wave`、`duty_factor: 0.75`、四腿
  `phase_offset` 改为 0/0.25/0.5/0.75、`frequency_hz` 与 `step_height_m` 按 wave 的实测重定
  （注释写明依据；`verification` 判据与阈值**不动**）。
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
