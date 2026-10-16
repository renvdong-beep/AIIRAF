# IRAF 泛化欠债清理 · Step 2：把求解能力提到 core

- 计划时间：2026-09-20
- 前置已完成的提交：176e5d2（夹爪契约泛化）、85d0073（UR5e 适配）、9091651（去隐式默认值）、2f2bd1b（视觉声明式）
- 目标读者：本窗口（UR5 线）与框架窗口

## 1. 目标

让"接入一台新机械臂"只需要 profiles/ + config/ + 一个薄壳脚本，不需要复制算法。
当前同一类逻辑一半在 core、一半在机型脚本里：

| 能力 | 现在在哪 | 应该在哪 |
| --- | --- | --- |
| 位置型 IK（中点目标） | `src/iraf_core/kinematics.py` ✓ 已在 core | core |
| 6 维位姿约束 IK（工具指向 + 开合轴 + 夹持区中点） | `scripts/build_ur5_baseline.py`（`OrientedGraspSolver`） | core |
| 指尖离台间隙配平（沿后退方向迭代抬高目标） | `scripts/build_ur5_baseline.py` | core |
| 伺服重力前馈（qfrc_bias + 静态保持自证） | `scripts/build_ur5_baseline.py`（`_gravity_hold_ctrl`） | core |
| 机型声明（关节名/指腹 geom/夹持区/后退方向/限位） | `config/*.yaml` + `profiles/*.yaml` ✓ 已是数据 | 不变 |

判据（引用 AGENTS.md 6.3）：新机型接入若需要复制/修改 Runtime 核心代码，先重构接口。
现在 `build_ur5_baseline.py` 已有 993 行，其中算法部分约 500 行属于"下一个机型要再抄一遍"。

## 2. 现状与假设

- `src/iraf_core/kinematics.py` 已依赖 MuJoCo 并只做参数注入（无机型名），
  因此把位姿型 IK 放进去与现有分层一致（core 承载"平台无关的数值契约"）。
- UR5e 抓取验收当前状态：`SUCCEEDED`，位置误差 0.000119m；Piper 同 `SUCCEEDED`。
  本次重构**不得**改变这两个数字（逐位对齐即回归判据）。
- 现有安全/契约门禁必须全部保留：工具指向门禁、法兰高于台面门禁、
  夹持区中点一致性门禁、左右 pad 高度差门禁、姿态偏差门禁、指尖离台门禁、
  重力前馈静态保持自证（残余 ≤1e-3 rad）。
- 假定框架窗口不在改 `src/iraf_core/kinematics.py`（用户已授权继续）。

## 3. 方案

### 3.1 core 新增（`src/iraf_core/kinematics.py`）

1. `TargetPose`（dataclass）：`position`（世界系 3 维）+ `rotation`（3×3 目标旋转矩阵）。
2. `build_tool_pose(pointing_axis, spread_axis)`：由"工具指向 = 法兰 +z（法兰→夹持区中点）"
   与"开合轴 = 法兰 +x（左 pad → 右 pad）"构造目标旋转矩阵；两轴必须正交，否则显式失败。
3. `solve_pose_ik(model, data, target_pose, arm_joints, points, flange_site, iterations,
   tolerance_m, orientation_tolerance, damping, step_clip)`：
   - 位置误差取 `points` 的均值点（复用 `_jacobian_for_points`，与位置型 IK 同一口径）；
   - 姿态误差取法兰 site 的旋转向量（`mju_mat2Quat` + `mju_quat2Vel` 两步）；
   - 阻尼最小二乘、关节限位裁剪、`mj_normalizeQuat`、按最小位置残差保留 best；
   - 返回 `IkResult`（`extra` 携带 `rotation_error_rad`、两轴偏差角、迭代数）。
4. `balance_tip_clearance(...)`：给定"沿后退方向移动目标 + 求解"的闭包，迭代抬高直到
   指尖最低点 ≥ 台面 + 间隙；保留完整 trace；不收敛即显式失败（不做静默兜底）。
5. `gravity_hold_ctrl(model, arm_joints, hold_positions, hold_ms, tolerance_rad)`：
   - `qfrc_bias`（qvel=0）→ τ，`Δctrl = τ / gainprm[0]`（增益从模型读，不写死）；
   - 自证：施加补偿后静态保持 `hold_ms`，稳态关节误差必须 ≤ `tolerance_rad`；
   - 自建 MjData，不污染调用方状态（避免"留证数组是内部缓冲区视图"类错误）；
   - 返回 `(offsets, evidence)`，evidence 含重力矩、增益、残余误差。

### 3.2 脚本变薄（`scripts/build_ur5_baseline.py`）

保留：读配置、组装探测场景、调用 core、全部门禁、证据落盘。
删除：`OrientedGraspSolver` 类、`_gravity_hold_ctrl`、配平循环实现，改为调用 core。
允许保留"UR5 专用"的部分仅限：配置字段名、场景探测（复用 `build_robot_pick_scene`）。

### 3.3 后续（不在本步）

- Piper 侧（`scripts/build_piper_baseline.py`）切到同一套 core；先逐项对齐残差再废弃旧路径。
- 统一入口 `scripts/build_baseline.py --baseline <robot>.yaml`、`scripts/verify_pick.py`。
- 框架侧决策：`factory.verify_backend_contract` 双向 → 单向，并新增
  "声明 visual_pick ⇒ 必须有 vision 来源"校验（见 `docs/debug/2026-09-20-declarative-vision-provider.md`）。

## 4. 涉及文件

- 改：`src/iraf_core/kinematics.py`（新增 3 组能力，保留既有 API 逐位不变）
- 改：`scripts/build_ur5_baseline.py`（改为调用 core；删除重复实现）
- 新增：`tests/unit/test_kinematics_pose_ik.py`（core 求解器正/负路径）
- 新增：`docs/debug/2026-09-20-core-extraction-pose-ik.md`（证据与口径说明）
- 不变：`profiles/ur5_mujoco.yaml`、`config/ur5_simulation_baseline.yaml`、
  `scripts/build_robot_pick_scene.py`、后端 `mujoco_backend.py`

## 5. 验证

1. `PYTHONPATH=src python3 scripts/build_ur5_baseline.py`
   → 参考姿态残差、姿态偏差、夹持区中心误差、前馈残余必须与重构前**逐位一致或更优**；
   对照基线：`build/calibration/ur5-baseline-pose.json` 现有数值
   （grasp 残差 3e-08m、姿态偏差 0.0065°、grip_center_error 6.6e-08m、前馈 4 组、
   worst_residual_rad 0.0）。
2. `PYTHONPATH=src python3 scripts/verify_ur5_pick.py` → `SUCCEEDED`，位置误差 ≤0.005m
   （当前 0.000119m）、抬升 ≥0.02m、双侧接触。
3. `PYTHONPATH=src python3 scripts/verify_piper_pick.py ...` → `SUCCEEDED`（回归）
4. `PYTHONPATH=src python3 -m unittest discover -s tests/unit -t tests/unit`
   → 失败项仅允许既有 1 项 + 4 项导入错误。
5. 单独跑新测试文件，确认 core 求解器的拒绝路径（零向量轴、非正交轴、缺 geom、
   关节不足）都显式失败。

## 6. 风险与开放问题

- **重构风险**：位姿型 IK 的收敛行为对阻尼、步长裁剪、best 保留策略敏感；
  迁移时按"逐位一致"要求做，一旦数字变化先回退再分析，不放宽门禁。
- **接口口味**：core 现有 IK 返回 `IkResult`（frozen dataclass）；位姿型沿用同一类型，
  姿态残差放 `extra`，避免再加一种返回形状。
- **开放问题**：`gravity_hold_ctrl` 是否该留在 core —— 它读模型的 `qfrc_bias` 与
  `gainprm`，属"仿真执行器模型"而非运动学。若框架侧认为应归
  `src/iraf_core/servo.py`，本步先放 `kinematics.py` 并在调试记录里登记待议。
