# 02 参数化 trot：原地踏步（声明 + 控制器 + 验收）

- 状态：TODO　　预估：3~5 个 tick　　归属：本窗口（四足线）
- 依据：ADR-0008、`.hermes/plans/2026-09-21_quadruped-locomotion-and-goto.md`

## 目标
实现声明驱动的对角小跑控制器，先在原地踏步上证明四腿协调与机身稳定（不带位移）。

## 前置
步骤 01 完成（`locomotion` 与移动上限已声明）。

## 涉及文件（提交时只 add 这些路径）
- 新增：`src/iraf_adapters/unitree/gait.py`（参数化 trot：相位/步高/占空比/机身高度/斜坡，全部入参）
- 改：`config/go2_loopback.yaml`（新增 `gait` 段：步频、步高、占空比、相位偏移、摆动轨迹形状）
- 新增：`scripts/verify_go2_trot_in_place.py`
- 新增：`tests/unit/test_gait_contract.py`

## 步骤
1. 声明 `gait` 段（步频 Hz、步高 m、占空比、对角相位等），缺键即失败。
2. 控制器：把步态相位映射到 12 个关节的目标角（足端轨迹→关节角的解析式只写在实现层一处），再经既有 PD + 重力前馈（不新写第二套控制律）。
3. 原地踏步：位移目标恒为 0，只做支撑/摆动切换；采样机身高度、倾角、四腿接触力。
4. 验收判据全部来自声明：持续 10 s 无跌倒、机身高度波动 ≤ 声明、接触序列正确（每腿有明确支撑相）。
5. 负向：步频为 0、占空比越界、步高为负 → 显式失败。

## 验收（数字全部来自声明；未达标即 FAILED，不放宽）
```
PYTHONPATH=src /usr/bin/python3 scripts/verify_go2_trot_in_place.py --config config/go2_loopback.yaml; echo exit=$?  # 期望 0
# 报告含：duration_s / height_std_m / max_tilt_deg / 每腿支撑相统计 / ctrl_saturated_samples
```
报告中 `simulation: true`；`ctrl_saturated_samples` 必须为 0（饱和即判据失败）。

## 证据落盘
`build/acceptance/go2-trot-in-place/report.json`

## 提交信息
`feat: 新增参数化 trot 原地踏步与稳定性验收`

## 失败 / 阻塞处理
若无法稳定：先做参数扫描（步频×步高×kd）并留证据，再决定改声明还是改控制律；禁止把「能站住」写成「能走」。
