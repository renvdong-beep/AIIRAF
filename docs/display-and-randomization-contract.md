# 显示与随机化：多机器人构型泛化契约

本文把"随机摆放 → 参考姿态求解 → 门禁 → 抓取 → 可视化"这条流程规范为
**与机型无关的契约**，并说明新增机器人时需要做什么、不需要做什么。

前置阅读：`AGENTS.md`（铁律）、`docs/piper-generic-integration.md`（适配入口）。

---

## 1. 分层与职责

| 层 | 位置 | 职责 | 机型相关？ |
|---|---|---|---|
| 契约层 | `src/iraf_core/kinematics.py` | 位置型阻尼最小二乘 IK；关节名/末端几何全部参数注入 | 否 |
| 契约层 | `src/iraf_core/randomization.py` | 圆盘面积均匀采样、失败归因分类、统计量 | 否 |
| 适配层 | `src/iraf_adapters/mujoco/viewer_runner.py` | 显示编排、快照并发、请求组装 | 否 |
| 装配层 | `src/iraf_adapters/factory.py` | 装配期契约校验（capabilities ↔ 实现） | 否 |
| 配置层 | `profiles/*.yaml`、`config/*.yaml` | 关节名、限位、夹爪位形、相机、容差 | **是** |
| 脚本层 | `scripts/*.py` | 选场景、传参、落报告 | 否 |

**原则**：机型差异只允许出现在 `profiles/` 与 `config/`。
任何脚本里出现关节名、body 名、几何名的字面量，都属于待修复的耦合。

---

## 2. 随机化契约

### 2.1 采样语义（必须可复现）

`UniformDiscSampler(spec, seed)` 的行为：

- **面积均匀**：`r = R·sqrt(u)`，`E[r] = 2R/3`（不是 `R/2`，半径均匀会让点向圆心聚集）；
- **yaw**：默认固定为 `0`，取自 `RandomizationSpec.yaw_choices_deg`；
- **z 贴台面**：`z = workbench.top_z_m + half_size`，不独立随机；
- **可复现**：同一 `seed` 必须产出逐位相同的采样序列。

### 2.2 批量入口的采样语义（踩坑点）

**每条批量路径必须用"每轮独立子种子 `base_seed + index`"，不要用一个 sampler 贯穿多轮。**

原因：单 sampler 贯穿时，第 N 轮的结果依赖前 N−1 轮消耗掉的随机数个数，
一旦某轮发生重采样，后续轮次全部错位 —— 两条脚本即使 seed 相同，
采到的几何也不同，通过率无法对比。实测曾因此误判"显示路径有 bug"。

报告里必须同时写入 `base_seed` 与每轮 `round_seed`，便于跨路径核对。

### 2.3 失败分类（铁律 5：失败即显式）

采样阶段**不做可达性预筛**：退化构型也记为一次合法采样，由下游门禁拒绝并归因。

`classify_rejection()` 的类别：

| 类别 | 含义 |
|---|---|
| `IK_DEGENERATE` | 关节空间退化（|joint3| 接近 0，臂近伸直、雅可比秩亏） |
| `IK_NOT_CONVERGED` | 指尖离台配平未收敛 |
| `ALIGNMENT_TOLERANCE` | 夹爪对齐门禁不达标 |
| `POSITION_TOLERANCE` | 抓取位姿与目标位置不一致 |
| `FINGER_TABLE_COLLISION` | 指尖扎入工作台 |
| `FINGER_CONTACT` | 张开手指与场景接触 |
| `VISION_FAILED` | 视觉检测失败 |
| `MOTION_FAILED` | 接触力/抬升不达标 |
| `OTHER` | 未归类（保留原文，不静默丢弃） |

### 2.4 已知的可达性边界（实测，Piper 标称点）

以标称抓取点为圆心、半径 50mm 的圆盘内：

| 半径区间 | 通过率 |
|---|---|
| 0–10mm | 100% |
| 10–20mm | 100% |
| 20–30mm | 50% |
| 30–40% | 33% |
| 40–50mm | 50% |

失败全部落在 `|joint3| ∈ [0.03, 0.25]` 的退化带，且集中在 +y 侧
（需更大 joint1 转角；`config/piper_multi_target.yaml` 已记录 joint1 阻尼 300
导致大转角稳态误差可达 2cm 以上）。

**结论**：需要稳定演示时把 `grasp.randomization.xy_radius_m` 收到 `0.02`；
要根治需在参考姿态求解引入退化门禁或关节空间正则项（属 `pick_object` 六阶段重构范畴）。

### 2.5 感知精度（实测，25 次采样）

通过样本的位置误差：均值 `0.963mm`、标准差 `0.419mm`、最大 `1.773mm`；
姿态误差均值 `0.880°`、最大 `1.480°`。对照 `acceptance.pose_tolerance_m = 0.005`
有 3.23mm 余量，**阈值可维持**。

---

## 3. 显示契约

### 3.1 三档显示能力

| 入口 | 可见内容 | 适用 |
|---|---|---|
| `run_interactive` | 抓取**完成后**的末态 | 确认结果 |
| `run_offscreen` | 抓取完成后导出帧序列 | 无图形会话的 CI |
| `run_interactive_live` | **全过程**（HOME→APPROACH→DESCEND→GRIP→LIFT） | 演示、排查阶段问题 |

### 3.2 "只有一帧"的根因与并发契约（重要）

`RobotBackend.display_lock()` 返回的**就是物理步进那把锁**。抓取每个
timestep 都要抢它；若渲染循环也在同一把锁上 `viewer.sync()`，
`realtime=True` 时渲染会被饿死 —— 现象就是**整个抓取过程只显示一帧**。

`run_interactive_live` 因此遵守三条约束，缺一即退化：

1. 渲染只读 `SnapshotMirror` 持有的副本，抓取期间**绝不同步后端 data**；
2. 仅在**拷贝快照的瞬间**持锁，且拷贝本身不耗时；
3. 窗口生命周期在单次调用内闭合。

### 3.3 快照实现的两个实测陷阱

| 陷阱 | 实测结论 |
|---|---|
| `mj_copyData` | MuJoCo 3.3.3 的 Python 绑定**不存在**该函数 → 必须用 `mj_getState`/`mj_setState` |
| 缓冲区长度 | 必须恰为 `mj_stateSize(model, mjSTATE_FULLPHYSICS)`（Piper 场景为 30） |
| **mocap 不在 FULLPHYSICS 中** | 实测 `carried=False`；而抓取用 mocap body（`grasp_anchor`）驱动抬升约束，**漏掉会让渲染错位** → 必须显式复制 `mocap_pos`/`mocap_quat` |

`SnapshotMirror` 已把这三点封装好，调用方不需要重复踩。

### 3.4 真实时序与租约的冲突

`realtime=True` 下整条抓取约 **92 秒**，超出既有技能的 30 秒租约。因此：

- 显示用 skill `display_pick`：`timeoutSeconds: 180`，`duration_ms` 上限 180000；
- 显示用策略 `profiles/safety/display_showcase.yaml`：`max_duration_ms: 180000`；
- `build_grasp_request` 的 `deadline_unix_ms` 按时长放大（`duration*12 + 60s`）。

**验收口径完全不变**：验收链仍走 `visual_pick` + `simulation_lab.yaml`（30s），
显示链路是独立旁路，两者互不影响。

---

## 4. 适配一台新机器人：照抄清单

### 4.1 必须做

1. **写 Profile**（可用向导生成，见 §5）：

   ```yaml
   spec:
     joints: [...]                  # 全部关节（含夹爪驱动）
     joint_limits: {...}            # 每关节 [low, high]
     capabilities: [move_joint, pick_object, visual_pick, stop]
     joint_roles: {..., <夹爪驱动1>: gripper_drive_left, <夹爪驱动2>: gripper_drive_right}
     gripper:
       drive_joints: [..., ...]     # 恰好两个
       open_positions: {...}
       closed_positions: {...}
       pad_offset_m: <数值>
     home: {...}
     camera: {lookat_m: [x, y, z], ...}
     manipulation: {targets: {...}}
   ```

2. **写基线 config**：声明 `model.source`、`arm_joints`、`bodies.wrist`、
   `finger_geoms`、`grasp.finger_center_xy_m`、`grasp.solver`、
   `grasp.randomization.xy_radius_m`、`acceptance.*`。

3. **实现 Backend**（实现 `RobotBackend` 契约）：

   ```
   from_config(config, profile, authority) -> backend
   move_joint(positions, duration_ms, lease)
   pick_object(target_id, grasp_pose, duration_ms, lease)
   stop(lease)
   step(count=1)
   ```
   加上显示/生命周期方法：`start_continuous` / `stop_continuous` /
   `is_continuous` / `render_frames` / `home_pose` / `hold_current_pose` /
   `simulation_status` / `display_lock`，
   以及供 Policy 求值前置条件的 `runtime_inventory()`
   （至少含 `safety.estop` 与 `manipulation.target_visible`）。

4. **装配期校验**：`factory.load_backend` 会交叉核对
   `capabilities` ↔ 实现、并反向检查"实现了运动能力但未声明"。
   不一致会在**装配期**报 `BackendContractError`，不会拖到运行期。

5. **IK 等价性验证**：仿照 `scripts/verify_ik_equivalence.py`，
   用同一初值/目标/参数比对 `iraf_core.kinematics` 解与既有实现，
   要求逐位一致（`1e-12`）。

6. **跑验收链**：无视觉真值 → 单目标视觉闭环 → 多目标未知姿态。

### 4.2 不需要做

- **不需要改 `viewer_runner.py`**：显示契约只依赖
  `model` / `data` / `display_lock` 三个公开成员；
- **不需要改 `kinematics.py`**：关节名与几何全部参数注入；
- **不需要改 `randomization.py`**：随机化只做几何，不做运动学；
- **不需要改任何脚本**：脚本只选场景与传参。

---

## 5. 用向导生成适配工件（面向外部客户）

`engineering.robot_adapter` 是一个**构建期工程向导**，不是机器人操作 skill：

- 独立能力命名空间 `engineering.*`，`safetyClass: build_tooling`，`preconditions: []`；
- **不调用任何运动方法、不申请运动租约**（有单测断言）；
- 使用独立 Profile `profiles/engineering_tooling.yaml`
  （只声明向导能力，不含运动能力，因此该 Runtime 无法执行运动 skill）
  与独立策略 `profiles/safety/engineering_tooling.yaml`。

提交问卷 → 产出四件工件：

| 工件 | 内容 |
|---|---|
| `<robot>_profile.yaml` | RobotProfile（已通过既有加载器校验） |
| `<robot>_backend.py` | Backend 骨架（契约签名 + `NotImplementedError`，不猜测实现） |
| `<robot>_baseline.yaml` | 基线 config 骨架 |
| `<robot>_validation_report.json` | 校验报告 + 人工待办清单 |

**工件默认拒绝覆写**（铁律 5）；需要覆写必须显式声明 `allow_overwrite`。

---

## 6. 复现命令

```bash
# 契约验证：IK 等价性
MUJOCO_GL=egl PYTHONPATH=src:scripts python3 scripts/verify_ik_equivalence.py

# 装配期契约校验（真实后端 + 反例）
MUJOCO_GL=egl PYTHONPATH=src:scripts python3 scripts/verify_backend_assembly.py

# 随机摆放验证（N 次采样，产出通过率与误差分布）
MUJOCO_GL=egl PYTHONPATH=src:scripts python3 scripts/verify_piper_random_pick.py \
    --samples 25 --seed 20260916

# 随机抓取（无头，单次，可复现）
MUJOCO_GL=egl PYTHONPATH=src:scripts python3 scripts/run_piper_random_pick.py --seed 7

# 随机抓取（可视化，真实时序完整过程）
MUJOCO_GL=glfw DISPLAY=:0 PYTHONPATH=src:scripts \
    python3 scripts/run_piper_random_view.py --rounds 5 --seed 20260917 \
    --realtime --seconds 15

# 工程向导（生成适配工件）
MUJOCO_GL=egl PYTHONPATH=src:scripts python3 scripts/verify_robot_adapter_wizard.py
```

---

## 7. 已知限制与后续排期

| 项 | 状态 | 说明 |
|---|---|---|
| `grasp.yaw_deg` 夹爪朝向对齐 | **未接线** | 字段被写入但无人读取；本期 yaw 固定为 0，随机 yaw 不改变抓取几何。留待 `pick_object` 六阶段重构 |
| joint3 退化导致的不可达 | **未根治** | 半径 >20mm 时通过率降到 ~40%。需在参考姿态求解引入退化门禁，或改 IK 目标函数 |
| `pick_object` 六阶段提取为 `GraspActuator` | **未排期** | 用户明确"后期再完成" |
| 倾斜目标（roll/pitch ≠ 0）姿态对齐 | **未实现** | 会被 `validate_grasp_pose` 全量拦截 |
