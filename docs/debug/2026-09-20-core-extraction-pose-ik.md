# 2026-09-20 Step 2：把位姿型 IK、离台配平、重力前馈提到 core

计划：`.hermes/plans/2026-09-20_120000-generality-step2-core-extraction.md`
（`.hermes/` 为工作区状态，不入库）
关联：AGENTS.md 6.3（新机型接入不得复制核心代码）、6.4（统一金路径）

## 1. 动机

接入 UR5e 时，同一类逻辑被写成了"一半在 core、一半在机型脚本"：

| 能力 | 迁移前 | 迁移后 |
| --- | --- | --- |
| 位置型 IK（中点目标） | `iraf_core.kinematics.solve_position_ik` | 不变 |
| 6 维位姿约束 IK（工具指向 + 开合轴 + 夹持区中点） | `scripts/build_ur5_baseline.py` 的 `OrientedGraspSolver`（约 200 行） | `iraf_core.kinematics.solve_pose_ik` |
| 指尖离台间隙配平 | 脚本内联循环 | `iraf_core.kinematics.balance_tip_clearance` |
| 伺服重力前馈（qfrc_bias + 静态保持自证） | 脚本内 `_gravity_hold_ctrl`（约 130 行） | `iraf_core.kinematics.gravity_hold_ctrl` |

后果：下一个机型（Franka / GCR5）必须再抄一遍这三块。脚本由 994 行降到 731 行，
其中剩下的部分只有"读配置 → 调 core → 门禁 → 落盘证据"。

## 2. core 新增的接口与口径

- `tool_pose_from_axes(pointing_axis, spread_axis)`：由"工具指向 = 法兰 +z、
  开合轴 = 法兰 +x"构造目标旋转；两轴不正交即显式失败（静默正交化会得到
  一个"看起来合理但谁也不满足"的目标）。
- `solve_pose_ik(model, data, target_position, target_rotation, arm_joints,
  points, flange_site, iterations, tolerance_m, orientation_tolerance, damping,
  step_limit)`：位置误差取 `points` 的算术平均（与位置型 IK **同一口径**），
  姿态误差取法兰 site 的旋转向量，拼 6 维后做阻尼最小二乘。
  返回既有 `IkResult`；`extra` 携带 `rotation_error_rad`、`best_position_error_m`、
  `best_iteration`、`target_rotation`。
  与旧实现一致：迭代耗尽时返回**当前**迭代状态（不伪造收敛），最优残差另行留证。
- `balance_tip_clearance(...)`：沿注入的方向抬高目标直到指尖最低点达标；
  `solve` 由调用方注入闭包（求解器与机型都无关）；
  未收敛时返回 `cleared=False` 而**不抛错** —— 是否接受由调用方门禁决定，
  避免把策略判断下沉进核心库。
- `gravity_hold_ctrl(model, arm_joints, hold_positions, hold_ms, tolerance_rad)`：
  `qfrc_bias`（qvel=0）→ τ，`Δctrl = τ / gainprm[0]`（增益从模型读），
  随后做一次静态保持仿真自证（稳态误差 ≤ tolerance）。
  **自建 MjData**，不修改调用方状态。
  仍然明确禁止用 `mj_inverse`：它按执行器力限截断（实测 shoulder_lift
  稳定返回 -150.000 N·m，恰好等于官方力矩限值且符号相反）。
- `lowest_mesh_point_z` 增加 **box 分支**（旋转后的半尺寸投影），
  使 2F-85 的 pad box 与网格走同一口径。

## 3. 逐位一致验证（重构的安全网）

重构前后各生成一次参考姿态证据，逐字段比对：

```
差异 19 处，全部为：
  joint_positions / target_m / feedforward / orientation_error_deg / grip_region … 完全相同
  finger_center_m            差 1e-10 m 量级（sum/n 与 np.mean 的浮点结合律差异）
  gripper_axis_world         差 1e-9（round(x, 9) 边界）
  clearance_trace[0].rotation_error_rad  新增字段（有意）
```

对照文件：`build/ur5-pose-before-core-extraction.json`（重构前快照，位于 build/ 不入库）。

## 4. 回归结果

```
UR5e  verify_ur5_pick.py    SUCCEEDED  位置误差 0.000119m  抬升 0.041208m  grip_region 口径
Piper verify_piper_pick.py  SUCCEEDED  力 0.240676/0.244112N  抬升 0.087184m
                                       抓取点偏差 0.002826042685491991（与重构前逐位相同）
单元测试 258 项（新增 14 项 core 契约测试），失败项仍为此前已有的 1 项 + 4 项导入错误
```

新增测试 `tests/unit/test_kinematics_pose_ik.py` 用自建最小 MJCF（单自由度滑轨）
覆盖：目标旋转构造的正/负路径、位姿 IK 收敛与"迭代耗尽不伪造收敛"、
配平的抬高循环与未收敛导出、重力前馈 = τ/gain、无重力时前馈为零、
不修改调用方状态、未知关节与缺目标位形的显式失败。

## 5. 遗留

1. **Piper 侧尚未切到同一套 core**：`scripts/build_piper_baseline.py` 仍只用
   `solve_position_ik` + 自己的抓取姿态逻辑。下一步把它也切到 `solve_pose_ik`
   与 `balance_tip_clearance`，先逐项对齐残差再废弃旧路径。
2. **统一入口**未做：`scripts/build_baseline.py --baseline <robot>.yaml`、
   `scripts/verify_pick.py --baseline --profile`（铁律 6.4 的金路径）。
3. `gravity_hold_ctrl` 目前放在 `kinematics.py`：它读模型的 `qfrc_bias` 与
   `gainprm`，属"仿真执行器模型"而非纯运动学。若框架侧倾向独立模块
   （如 `iraf_core/servo.py`），迁移成本很低（纯函数 + 一个测试文件）。
