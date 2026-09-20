# 2026-09-20 UR5e 抓取不可达：工具指向被写成"后退方向"（含 mj_inverse 力限截断坑）

关联文档：`docs/ur5-adaptation-status.md`（9/17 的阻塞记录）、
`docs/hybrid-model-assembly-pitfalls.md`、`config/ur5_simulation_baseline.yaml`、
`profiles/ur5_mujoco.yaml`

## 1. 现象

UR5e + 2F-85 受控仿真抓取验收一直 `FAILED`：

```
末端未到达目标抓取位姿: distance=0.314034m tolerance=0.005000m
delta=[+0.156745, -0.081362, +0.259670]   qpos.shoulder_lift=-0.5276（目标 -0.0440）
```

9/17 的文档把阻塞点记为"DESCEND 段跟踪误差 0.498 rad"，并推断根因是
"IK 把 pad1 geom 中心当抓取点，整套 pad 相对方块下沉 34mm"。
本次复现后确认该推断**不是根因的完整解释**：pad 与方块的**水平**几何
（间隙 25.7mm/侧）在 4 点中点口径下已经正确，问题在竖直方向且更靠前。

## 2. 排查链路与证据

### 2.1 `scripts/probe_approach_contact.py`（逐毫秒碰撞时间线）

三段轨迹（与后端 `_move_trajectory` 同算法：quintic + settle）逐步记录
跟踪误差、方块接触对、夹持区中点：

```
HOME_HOLD  box=[-0.55, -0.134, 0.025]（未动）  最大误差 0.016969 rad
APPROACH   box 在 t=1284ms 开始位移，段末 [-0.5637, -0.0673, 0.0334]
DESCEND    box 段末 [-0.7094, -0.0527, 0.0509]，位移 180.8mm  最大误差 0.483600 rad
```

关键发现：**方块在 APPROACH 段就被推动了**，而 DESCEND 的 0.48 rad
跟踪误差只是接触反力的结果，不是伺服增益不足的原因。

### 2.2 `scripts/probe_contact_geometry.py`（接触对身份）

首次非台面接触的完整记录：

```
phase=APPROACH t=1278.000ms
box_01_geom <-> <geom#27>(body=wrist_2_link) dist=-0.00039
接触点 pos=[-0.56794, -0.134, 0.04981]（方块顶面附近）
同一瞬间夹持区中点 = [-0.580119, -0.134003, 0.343883]
```

`geom#26/#27` 是 UR5e 官方 `wrist_2_link` 的碰撞胶囊（半径 0.04m，
见 `scripts/probe_geom_identity.py`）。**腕部比 pad 早 0.3m 到达方块**，
物理上等价于"工具轴朝上、夹爪挂在腕部下方并朝上托"。

### 2.3 `scripts/probe_tool_axis.py`（工具轴方向，判定性证据）

```
姿态        法兰局部 z 轴(世界系)        法兰 z     wrist_2 z   pad 中点 z   轴指向
home      [-0.0001 -0.0000 +1.0000]   0.35053    0.25054     0.48500     朝上
approach  [-0.0001 +0.0000 +1.0000]   0.05053   -0.04946     0.18500     朝上
grasp     [-0.0001 +0.0002 +1.0000]  -0.10947   -0.20946     0.02500     朝上
lift      [-0.0000 -0.0000 +1.0000]  -0.02947   -0.12947     0.10500     朝上

pad 相对法兰的 z 偏置 = +0.13447 m（pad 在法兰**上方** 134mm）
```

结论：抓取位姿要求法兰 z = **-0.109 m**、wrist_2 z = -0.209 m，
即腕部必须在台面以下 20cm —— **位姿不可达**。这不是精度问题，
而是目标姿态本身反了。

## 3. 根因

`config/ur5_simulation_baseline.yaml` 的 `grasp.approach_direction: [0,0,1]`
语义是**预抓取后退方向**（抓取点 → 预抓取点，本场景竖直向上）。
`scripts/build_ur5_baseline.py` 的 `OrientedGraspSolver` 曾把它**直接**
当作法兰 z 轴（"夹爪指向"），于是优化目标变成"夹爪朝上"：

```python
# 修复前
z_axis = self.direction          # direction = approach_direction = [0,0,1] → 朝上
```

后果链条：法兰 z 朝上 → 夹持区（法兰 +z 侧 134mm）被抬到法兰上方 →
要把 pad 送到方块中心就必须把法兰压到台面以下 → 实际执行时
`wrist_2_link` 碰撞体先压在方块顶面 → shoulder_lift 被顶住且力矩饱和
（实测误差 0.48 rad，远超纯 PD 静差上限 150N/2000 = 0.075 rad）→
验收报 0.314m。

**为什么姿态门禁没拦住**：门禁比较的是"实测夹爪指向 vs 期望方向"，
而期望方向本身就是反的，所以偏差报 0.008°，看起来完美。

## 4. 修复

1. `OrientedGraspSolver`：参数与属性改名为 `pointing`（工具指向），
   目标旋转取 `z_axis = pointing`；调用方显式传
   `pointing_direction = -retreat_direction`。两者语义在 docstring 与
   配置注释里写明"互为反向、不可互相替代"。
2. 新增**防复发门禁**（同一次构建内显式失败）：
   - 夹持区必须落在法兰沿工具指向的一侧：`(grip_mid - flange)·pointing > 0`；
   - 抓取位姿下法兰必须高于台面：`flange_z > workbench.top_z_m`。
   这两条与"姿态偏差"正交，符号再反会立刻失败。
3. 证据新增 `tool_pointing_direction_world` / `retreat_direction_world` /
   `grip_along_tool_axis_m` / `flange_position_m`，两类方向同时留证；
   `approach_direction_world` 保留旧字段名（语义 = 后退方向），
   因为 `scripts/build_robot_pick_scene.py` 用它推导 `pad_offset_axis`。
4. 修掉一个"留证值被后续改写"的隐患：`data.site_xpos[...]` 是 MuJoCo
   内部缓冲区**视图**，取值时必须 `.copy()`，否则后面 `_gravity_hold_ctrl`
   改写 qpos 时留证值会一起变（实测把 grasp 的法兰 z 记成了 home 的 0.6195）。

## 5. 验证

### 5.1 修复后几何

```
姿态        法兰局部 z 轴(世界系)        法兰 z     wrist_2 z   pad 中点 z   轴指向
home      [-0.0001 -0.0000 -1.0000]   0.61947    0.71946     0.48500     朝下
approach  [-0.0002 -0.0000 -1.0000]   0.31947    0.41945     0.18500     朝下
grasp     [+0.0002 -0.0000 -1.0000]   0.15947    0.25949     0.02500     朝下
lift      [+0.0002 +0.0000 -1.0000]   0.23947    0.33949     0.10500     朝下

grip_along_tool_axis_m = +0.134470   flange_position_z(grasp) = 0.159470
夹持区中心与目标点误差 6.6e-08 m     姿态偏差 0.0065°（门禁 3°）
```

### 5.2 三段跟踪与方块状态（`scripts/probe_tracking.py`）

```
HOME_HOLD 0.014382 rad   APPROACH 0.014414 rad   DESCEND 0.015229 rad
三条段的 box_01 全部 = [-0.55, -0.134, 0.025]（未被推动、无碰撞）
```

对照修复前：0.016969 / 0.190713 / 0.483600 rad，方块被推走 180.8mm。

### 5.3 验收门禁

| | 修复前 | 修复后 |
| --- | --- | --- |
| DESCEND 后末端误差 | 0.314034 m | 0.016639 m |
| 容差 | 0.005 m | 0.005 m |

残差 16.6mm 的性质已变化：不再是接触反力造成的饱和，而是**纯 PD 静差**
（见下节）。

## 6. 排查中发现的第二个独立缺陷：mj_inverse 被执行器力限截断

9/18 为"重力前馈"加了 `_gravity_hold_ctrl`（用 `mj_inverse` 求静态力），
产出的增量是：

```
home      shoulder_lift +0.075 rad   elbow -0.075 rad   wrist_* ±0.056 rad
```

`0.075 = 150N/2000`、`0.056 = 28N/500`，**恰好等于 UR5e 官方力矩限值**。
逐项排除（`scripts/probe_inverse_dynamics.py`，A~E 五种口径，含关闭全部接触）
后确认：数值不随接触状态变化，是 `mj_inverse` 本身按 `forcerange` 截断的结果，
而且符号也错（`-150.000`，作为前馈会把臂推向反方向）。
手算与仿真实测都表明该位形只需约 30 N·m。

改用 `qfrc_bias`（qvel=0 即"保持静止所需广义力"，无截断、无接触污染）
后四姿态的重力矩与 ctrl 增量：

```
home      shoulder_lift -28.06 N·m → -0.01403 rad    elbow -19.72 → -0.00986   wrist_1 -2.41 → -0.00483
approach  shoulder_lift -28.46 N·m → -0.01423 rad    elbow -19.36 → -0.00968   wrist_1 -2.41 → -0.00483
grasp     shoulder_lift -33.26 N·m → -0.01663 rad    elbow -15.03 → -0.00751   wrist_1 -2.41 → -0.00483
lift      shoulder_lift -30.54 N·m → -0.01527 rad    elbow -17.49 → -0.00874   wrist_1 -2.41 → -0.00483
```

`_gravity_hold_ctrl` 同步改为：只读模型（自己构造 MjData，不动调用方状态、
不污染留证数组）、按 `gainprm[0]` 折算、并用**一次静态保持仿真**做自证
（`worst_residual_rad = 0.0`，门限 1e-3 rad）。

### 前馈效果（`scripts/probe_gravity_feedforward.py`）

```
段          无前馈误差(m)   有前馈误差(m)   容差内
HOME_HOLD  0.013331       0.000119        是
APPROACH   0.013642       0.000119        是
DESCEND    0.008643       0.000119        是
```

残余偏差是 z 方向 -0.119mm 的常数项（容差 5mm），三段一致。

## 7. 适用范围与遗留

- 本条修的是**仿真里的姿态语义与伺服静差**；真机 UR5e 的位置伺服内部
  已做重力补偿，不需要外部前馈，但"工具指向 = 后退方向的反向"这一条
  在真机同样成立。
- 重力前馈已接线（见第 8 节）。

## 8. 重力前馈接线与第三个缺陷：抓取点口径不一致

前馈算出来后只写进 `build/calibration/ur5-baseline-pose.json` 是不生效的
（这正是 9/18 那次"修了但没效果"的原因）。本次按**显式契约**接线：

| 层 | 改动 |
| --- | --- |
| 参考姿态求解器 `scripts/build_ur5_baseline.py` | 四姿态各自算 `gravity_feedforward`（`qfrc_bias` 口径）+ 静态保持自证 |
| 场景生成器 `scripts/build_robot_pick_scene.py` | 把前馈按"关节名 → ctrl 通道名"翻译后写入 report 的 `gripper.gravity_feedforward`；键与各段位置指令逐个校验 |
| 后端 `src/iraf_adapters/mujoco/mujoco_backend.py` | 解析该字段（未知段名/未知通道/超限/非有限数一律显式失败）；`_move_trajectory(..., ctrl_offsets=...)` 逐段加到 ctrl；`_pick_ctrl_offsets(phase)` 取值 |

`GRAVITY_FEEDFORWARD_LIMIT_RAD = 0.1` 作为单关节上限，只用于拦截"单位/符号写错"
（误填 N·m、方向写反），不作为正常取值约束（实测最大 0.017 rad）。

接线后验收残差 0.016639m → **0.009494m**，仍超 5mm 容差。此时关节角已经
完全到位（`shoulder_lift` 误差 2.5e-11 rad），因此剩余 9.5mm 不可能来自伺服：
它是**抓取点口径不一致** —— 求解器把"4 个 pad box 的中点"对齐到目标，
而运行时对齐门禁只用左右 **pad1** 的中点，两者相差 9.375mm：

```
IK 口径   = mean(rq2f85_left_pad1, left_pad2, right_pad1, right_pad2) = [-0.55, -0.134, 0.025]  ← 等于目标
门禁口径  = mean(rq2f85_left_pad1, rq2f85_right_pad1)                 = [-0.55, -0.134, 0.015625]
差值      = -9.375mm（z），且随工具指向翻转而变号
```

修复：把夹持区定义（`gripper.pad_boxes`）也透传给后端，门禁的"抓取点"改为
**配置声明的夹持区中点**，与 IK、参考姿态证据三方同口径；未声明时回退到
左右代表接触面（Piper 既有行为逐位不变）。门禁证据新增
`grasp_point_source`（`grip_region` / `finger_pair`）与 `grip_region_geoms` 留证。

### 最终验收结果（`build/acceptance/ur5-pick/report.json`）

```
status = SUCCEEDED          confirmation = contact
命中目标      target_body = box_01
双侧接触      bilateral_contact = true   法向力 17.452N / 16.053N（限 0.2N）  不平衡比 1.087（限 4.0）
抬升位移      lifted = true   lift_delta_m = 0.041208 m（限 0.02）
位置误差      center_distance_m = 0.000119 m（容差 0.005）  grasp_point_source = grip_region
```

回归验证：

- Piper 抓取验收 `verify_piper_pick.py` 仍为 `SUCCEEDED`
  （`grasp_point_source = finger_pair`，即回退路径未被改动；
  `lift_delta_m = 0.087184`、位置误差 0.002826m，与改动前一致）。
- 单元测试 229 项（原 220 + 新增 9 项契约回归），
  失败项仍只有改动前就存在的 `test_vision_processing` 1 项与 4 项导入错误。

### 复现命令

```
cd ~/AIIRAF
PYTHONPATH=src python3 scripts/build_ur5_baseline.py
PYTHONPATH=src python3 scripts/verify_ur5_pick.py
PYTHONPATH=src python3 -m unittest discover -s tests/unit -t tests/unit
```

