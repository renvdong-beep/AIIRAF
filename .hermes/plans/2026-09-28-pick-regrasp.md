# 计划：pick 增加"重新抓取（regrasp）"相位 —— 先抬离台、再合爪到载荷腰部

状态：**第 1–4 步已实施并实测 ⇒ 该方案在本几何下不可行（结论已归档）**。
- 已落地：契约字段（`lift_attitude_max_deg`/`regrasp`）、求解器 `pre_lift`/`regrasp` 两位形、
  构建器写入报告、后端四相位（PRE_LIFT→REGRASP_OPEN→REGRASP_DESCEND→REGRASP_CLOSE）+ 姿态证据、
  解析层白名单透传；`grasp.regrasp.enabled: false` 为当前状态（与改动前逐位一致）。
- 实测：相位表全部执行，但方块在 PRE_LIFT（声明 0.04 m）后**只升到 z=0.0338（9 mm）**
  ⇒ 首次"压上缘"抓取连 4 cm 竖直抬升都撑不住（§11.23(27) 的翻滚）⇒ **regrasp 无法到达**。
- 第 5 步（回归测试）不必为本方案补；**下一步转向**：声明化的**搬运约束**
  （臂侧世界既有 `box_01_lift_constraint` + `require_friction_lift: false`）接进联合世界。
- 前置调查与全部量化结论见 `docs/debug/2026-09-24-joint-model-dog-arm.md` §11.23(15)(27)(28)(29)。

## 0. 为什么需要它（已定案的机制，不是猜测）

- 夹爪只压住方块的**上缘**（`pad_offset_m` 让指腹中心高于方块顶面 ~4 mm）；
- 于是夹持对**转动**几乎没有阻力矩：实测抬升中载荷姿态从 0° 一路转到 **~120°**，最后翻出夹口
  （双侧接触力 5.7 N **正常**，姿态才是判读依据 —— 先看 `quat` 再看力）；
- 几何是否被迫：指腹长 **5.2 cm** vs 方块高 **5 cm** ⇒ 在**台面上**夹腰部必然让指尖进台面 ⇒
  被几何排除（`finger_tip_z_m = 0.005829938`、`tip_clearance_m = 0.005`）；
- 斜抓（闭合轴与方块面成 58.357073°）之所以能抬 8 cm：4 mm **过盈**把方块卡死、转不动 ⇒
  用"过盈"换"抗转"，代价是搬运段沿夹口轴**楔出**；
- "面抓 + 深闭合"已实测**否掉**：面抓在闭爪位 0.011~0.020 **全部托不住**（越用力越把载荷往外挤）。

## 1. 目标（可验收）

在**同一个联合世界**里，`s03_pick` 用**面夹**（闭合轴垂直于方块面）把方块抬起 ≥ 0.02 m 并保持
≥ 0.5 s，且：
- `lift_delta_m ≥ min_lift_delta_m(0.02)`（结束时刻仍在夹口里）
- `lift_peak_delta_m ≈ lift_delta_m`（**不再出现"抬起后滑出"**：两者接近说明没有中途丢失）
- 抬升段载荷姿态变化 ≤ 5°（新的**抗转**判据；这条要新增契约字段，见 §4）

## 2. 相位设计（新增 4 个相位，全部声明化）

```
HOME → APPROACH → DESCEND(抓取位形, 夹爪张开)
     → [到位门禁]
     → GRIP_OPEN → GRIP_CLOSE(上缘夹, 与现状相同)
     → PRE_LIFT(竖直抬升 pre_lift_m, 让方块离台)         ← 新增
     → REGRASP_OPEN(松开一档 regrasp_open_m)             ← 新增
     → REGRASP_DESCEND(下探 regrasp_depth_m 到方块腰部)   ← 新增
     → REGRASP_CLOSE(合到 closed_positions, 真正的面夹)   ← 新增
     → LIFT(到 lift_positions)
```

## 3. 声明（全部进基线 `config/piper_simulation_baseline.yaml`，缺声明即构建失败）

```yaml
grasp:
  regrasp:
    enabled: true                 # 关掉即回到现有单次抓取（行为逐位不变，便于对照）
    pre_lift_m: 0.04              # 第一次抬升高度（> 方块半高，确保离台）
    depth_m: 0.025                # 下探深度（到方块腰部）
    open_m: 0.004                 # 松开量（夹爪指令增量，保证下探时不挤住载荷）
    duration_ms: 2000             # 每段时长
```

## 4. 实施清单（顺序固定；每步跑绿再下一步）

1. **契约先行**：`skills/pick_object/pick_object.output.json` 的 evidence 增加
   `lift_attitude_max_deg`（number≥0，抬升段载荷姿态相对初始的最大偏角）与
   `regrasp`（object：`applied`/`pre_lift_m`/`depth_m`）；`config/scene.schema.json` 同步
   判据名（若新增判据）。
2. **求解器**（`scripts/build_piper_baseline.py`）：新增 `build_regrasp_poses(...)`，
   按 `pre_lift_m`/`depth_m` 在**任务空间**算两个位形（pre_lift = 抓取点竖直 +`pre_lift_m`；
   regrasp = 抓取点竖直 +(pre_lift_m − depth_m)），返回关节解 + IK 残差；缺参数即报错。
3. **构建器**（`scene_builder._joint_reference_resolution` + `build_piper_pick_scene`）：
   按声明调用并写入报告 `gripper.pre_lift_positions` / `gripper.regrasp_positions` /
   `gripper.regrasp`（含出处）；同时把姿态判据阈值写进报告。
4. **后端**（`mujoco_backend.pick_object`）：按 `gripper.regrasp.enabled` 走上面相位表；
   每个新相位都记 `self._log_pick_phase(...)` 并进 trace；抬升段常驻 sampler 里跟踪
   **姿态偏角**（用 `data.xquat[target]` 对抬升初始姿态求夹角）⇒ 写进证据。
5. **回归测试**：`tests/unit/` 增加 ①缺声明即失败 ②`enabled: false` 时与现状逐位一致
   ③相位表按声明顺序执行（用桩 report）。
6. **验收**：`nominal --world joint`（s03 必须 SUCCEEDED 且 `lift_peak ≈ lift_delta`、
   `lift_attitude_max_deg ≤ 5`）→ 再临时启用 s04 跑搬运段（届时会撞上 §11.23(11) 的
   `transit→above` 穿 `base_link` 2.286 cm —— 一并按航点修）。

## 5. 验证仪器（都已存在，直接用）

- `IRAF_DEBUG_PICK=1 IRAF_DEBUG_PICK_STRIDE=1`：抬升段逐样本（载荷位姿/指腹 qpos+ctrl/接触对+力）；
- `build/iraf-a6a14/offline_pick_sequence_probe.py`：逐相位回放探针（**只能证否**，见技能 §15）；
- `build/iraf-a6a14/gripper_clamp_calibration_probe.py`：闭爪位→力的接触标定（载荷钉住）。
