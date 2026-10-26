# piper 抓"机械臂左侧"的方块：runtime 纠偏只覆盖小偏差，大位移必须构建期声明（2026-10-26）

## 1. 结论先行
- piper 的场景**没有** `grasp_pose_correction`，抓取只能用构建期按声明位置解出的关节角 ⇒ 方块一挪必拒。
- 给它补上纠偏机制**是可行的**（本次实测纠偏确实执行了），但两条声明门禁把"15 cm 换位"挡了回来：
  1. `实测目标与名义解相差 0.150000000 m 超过声明上限 0.070000000 m ⇒ 拒绝`
  2. 把上限抬到 0.20 后再跑：`抓取段纠偏把夹爪姿态带歪了（approach 相位）：腕→夹持区轴偏 28.780° > 声明 8.000° ⇒ 拒绝`
- ⇒ runtime 纠偏的口径是**小偏差**（mm 级、UR5 的预算 0.07 是从验收域 0.06 反推的），
  不是"把目标搬到别处"。要把 piper 的方块放到左侧并抓，**正确做法是构建期把目标声明在左侧**，
  由构建器用位姿约束解出那一处的抓取关节（与 UR5 参考姿态求解器同路），而不是放宽 runtime 判据。

## 2. 本次改动（留作后续构建期方案的基础设施）
- `scripts/build_piper_pick_scene.py`：支持 `grasp.pad_boxes` 与 `grasp.grasp_pose_correction` 透传
  （声明为可选；没有声明时行为与改动前逐位一致）。
- `scripts/probe_piper_left_pick_scene.py`：把方块挪到左侧(y=+0.15)并补纠偏声明的**探针**（产物在 build/models/probe/，不是交付物）。
- `config/piper_simulation_baseline.yaml` 的纠偏声明已**回退**：当前 piper 报告的 gripper 里没有
  `grasp_positions`（构建器输出的是 hold/lift 形状），在这种报告上开纠偏会让 pick 直接报
  「grasp_positions 里没有可解的臂关节键」—— 所以不能只加声明。

## 3. 实测记录
```
probe 场景（box y=+0.150）+ correction(max_correction_m=0.07)：
  FAILED 实测目标与名义解相差 0.150000000 m 超过声明上限 0.070000000 m
probe 场景 + max_correction_m=0.20：
  FAILED 腕→夹持区轴偏 28.780° > 声明 8.000°
正常场景（box y=0，无纠偏）：
  SUCCEEDED wall=4.35 s（回归正常）
```

## 4. 下一步（正确的构建期方案，约 1-2 小时）
1. 让 piper 报告重新具备 `grasp_positions/approach_positions/home_positions`（当前构建器不产出；需按 UR5 的参考姿态求解路径补齐，
   或确认产出这些键的那个构建器/基线组合）；
2. 在同一构建器里把目标挡在左侧（target_position 由参考解给出），构建时用位姿约束解 IK ⇒ 报告天然声明"左侧"；
3. 校验：新建 `piper-left` 场景实例（iraf-provider@piper-left，独立端口），跑一次抓取验收 + 像素差证据。
