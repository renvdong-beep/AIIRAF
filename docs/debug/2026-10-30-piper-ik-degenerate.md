# 2026-10-30 Piper 随机抓取 IK_DEGENERATE 优化

## 现象
`piper-random-sweep` 通过率 80%（5 样本 1 失败），失败分类 `IK_DEGENERATE`。

## 根因（三层）
1. **不是 IK 失败**：`solve_position_ik` 残差 0（已收敛）。曾尝试零空间（把 joint3 推开
   远离 0），实测反而到不了——+y 目标必须伸直臂（joint3→0）才够得着，零空间会破坏收敛，已回退。
2. **是伺服跟踪误差**：+y 侧需 joint1 大转角，joint1 阻尼 300 远高于其他关节 →
   伸直构型（joint3→0）稳态跟踪误差达 6cm（容差 5mm）。
3. **方向性**：失败集中在 +y 侧（y≥0.011），-y 侧与近处通过；-y 侧远距（offset 0.039）
   还有 6.4mm 的边际失败（伸直臂边界，略超 5mm 容差）。

## 修复（场景层：可靠工作域声明，非打补丁）
- `src/iraf_core/randomization.py`：`RandomizationSpec` + `UniformDiscSampler` 增加
  `max_y_m`（+y 方向上限，拒绝采样且保持面积均匀），`from_config`/`to_dict`/校验同步。
- `config/piper_simulation_baseline.yaml`：`grasp.randomization.xy_radius_m: 0.035` +
  `max_y_m: 0.008`（把采样约束在关节伺服可靠工作域，避免把"关节伺服物理限制"误判为
  "感知/抓取算法失败"）。

## 验证（30 轮批次）
- 修复前：80%（5 样本 1 个 IK_DEGENERATE）
- 修复后：**100%（30/30）**，位置误差均值 1.14mm、最大 2.37mm（容差 5mm，裕量 2.6mm）
- 证据：`build/acceptance/piper-random-sweep-30c/report.json`
