# s06 到位残差：下压段分段纠偏（`grasp_pose_correction.descend_splits`）

日期：2026-09-30　状态：实施中　依据：`docs/debug/2026-09-24-joint-model-dog-arm.md` §11.56

## 1. 问题（实测）

s06（UR5e 卸载抓取）的"到位"判据是 `pose_tolerance_m = 0.005 m`，而实测残差在
**0.0012~0.0083 m** 之间间歇（单轮不算数，必须连跑看通过率）。

3 轮连跑（`build/diagnostics/handoff-round-{1,2,3}.log`）实测：

| 轮 | 到位残差 (m) | 判据 0.005 | 载荷漂移率 (m/s) |
|----|--------------|-----------|------------------|
| 1  | 0.002941875  | pass | 0.000093 / 0.000084 |
| 2  | 0.003333336  | pass | 0.000088 / 0.000080 |
| 3  | 0.003338753  | pass | 0.000086 / 0.000084 |

演示轮（历史）出现 0.007822 m（`delta = [+0.003382, -0.000522, +0.007034]`，z 向多 7 mm）⇒ 翻车。

## 2. 根因（已定量）

**残差 = 载荷漂移率 × 曝光窗口**，不是噪声、不是解算精度：

- 载荷在狗背托盘里以 0.080~0.093 mm/s **单向**漂移（接触求解器蠕变）；
- 最后一次纠偏（t=1082.79）到门禁（t=1115.71）之间隔着 **32.9 s 仿真时间**；
- 0.085 mm/s × 33 s ≈ 2.8 mm —— 与实测 2.94 / 3.33 / 3.34 mm 对上；
- 纠偏**自身**残差只有 0.35 mm 级 ⇒ 残差几乎全部是"解算基准过期 33 秒"。

## 3. 为什么窗口有 33 秒（时间放大，已判死）

s06 的 `duration_ms: 8000`，`phase_ms = 8000//5 = 1600 ms`（每相位标称 1.6 s），
但实测相邻相位推进 **27.2~32.0 s** ⇒ 约 **17x**（27.28 / 1.6）。

新增探针通路 `IRAF_DEBUG_GUEST_STEPS=1` → `PICK_STEP_ACCT`（本次实测，`guest-steps-round1.log`）：

```
ur5_mujoco DESCEND   calls=801 req=4000 act=9519 scope_amp=2.38
                     small(count=1)=800 次 req=800 act=6314 (7.89x)  max_over=24 步
ur5_mujoco APPROACH  calls=801 req=4000 act=9463 scope_amp=2.37
                     small(count=1)=800 次 req=800 act=6258 (7.82x)  max_over=24 步
```

**判死：不是记账 bug。** `_wait_for_guest_steps` 里 `target = before + count` 正确，
`before/after` 记账一致（settle 段 req=3200 → act≈3205，1.02x 精确）。
真因是 **owner 领先**：

- 臂是 **guest**，时间由 owner（Go2 驻留线程，`stand` 连续循环）推进；
- `wait_until` 的轮询间隔 = `timestep` = 2 ms，而 owner 实测约 **3900 步/s**
  ⇒ 每次唤醒植物已前进 **~7.8 步** ⇒ 逐拍 `count=1` 的轨迹循环被放大 **7.8x**；
- 相位总账 2.37x（记账内）+ guest 计算间隙（`_set_controls`/五次多项式/采样）里
  owner 自走未记账的步 ⇒ 相位总放大 **3.4x**（27.28 s / 8 s）。

即：**臂的控制节拍被 owner 的步进速率"拖长"**；标称 `duration_ms` 在联合世界里只是下界。

## 4. 修法（只动内部停止条件，不动 0.005 判据）

新增声明 `grasp_pose_correction.descend_splits: K`（正整数，缺省 1 = 逐位不变）：

- 把下压段拆成 K 个子段；**每个子段之前**用**新鲜的实测目标**重解剩余下压（IK 解算
  不推进植物 ⇒ 几乎不占仿真时间，实测 PICK_CORRECTION 时刻与上一相位 dump 只差 0.01 s）；
- 只有最后一个子段结束进入稳定窗口与门禁；
- 下压前半段指腹离载荷还有 8~16 cm（实测 APPROACH 末距离 0.160 m，DESCEND 末 0.0029 m）
  ⇒ 中途重解安全。

**预期**：曝光窗口 ≈ 27.2/K + 0.9 s ⇒ 残差 ≈ 2.8/K mm。

**K 的反推**（不放宽判据）：取历史残差分布上界 0.008228 m，要求最坏 ≤ 内部容差
`residual_tolerance_m = 0.002 m` ⇒ K ≥ 0.008228 / 0.002 = 4.11 ⇒ **K = 5**
（窗口 ≈ 5.4+0.9 = 6.3 s ⇒ 预期最坏 ≈ 1.6 mm，留 ~1.2x 余量）。

## 5. 验收与回归

1. 单轮：`IRAF_DEBUG_GUEST_STEPS=1 IRAF_DEBUG_PICK=1` 跑 s06，门禁残差与 `PICK_STEP_ACCT`
   子段数（= K）符合预期；
2. 多轮：`scripts/probe_handoff_handover.py --rounds N` 连跑，**通过率**（判据 0.005）
   与最坏残差分布；K 按实测分布调（不放宽 0.005）；
3. 回归：Piper（s03/s04）不声明 `descend_splits` ⇒ 行为逐位不变；
4. 声明链三处同步：`GRASP_POSE_CORRECTION_KEYS`（运行期允许键）、臂生成器转发校验、
   基线 `config/ur5_simulation_baseline.yaml`。

## 6. 不做的事

- 不放宽 `pose_tolerance_m = 0.005`；
- 不改 owner 的推进速率 / 不动 `dt` / 不动 PD 增益（历史 §11.6 的稳定性口径）；
- 不在下压**之后**纠偏（§11.31：指腹已在载荷两侧，越纠越偏）。
