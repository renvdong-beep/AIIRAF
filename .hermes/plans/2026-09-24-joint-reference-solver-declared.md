     1|# 联合世界的臂参考姿态：按声明重解（2026-09-24 续 · 第 2 轮）
     2|
     3|## 0. 本轮起点（上一轮提交 8de1f78）
     4|
     5|- 联合世界 `--world joint` 已可用：s01 stand、s02 dock 绿（0.028125012624229208 m），
     6|  植物驻留 `cycles=16`、`failed_cycles=[]`。
     7|- 卡点 s03：`IRAF-EXECUTION-FAILED 末端未到达目标抓取位姿: distance=0.369305m tolerance=0.005000m`。
     8|- 已量化（`build/iraf-a6a14/joint-reference-resolve-probe.json`）：位姿与 pad_offset 必须**一起重解**：
     9|  继承位姿+继承 pad 0.367696068 / 重解位姿+继承 pad 0.018389447 / 重解位姿+重解 pad **5.681e-06**。
    10|
    11|## 1. 本轮必做（用户给定的三步）
    12|
    13|1. 联合模式参考姿态求解器**写进声明**（`robots[].reference_solver: {module, entry, baseline}`），
    14|   构建器**按声明**调用 —— 不在 `scene_builder` 里写 piper 特例（AGENTS.md 6.3）。
    15|2. 重解结果写进联合报告的 `gripper.{home,approach,grasp,lift}_positions`（关节名按既有规则加前缀）
    16|   与 `pad_offset_m`，并标注 `reference_pose_source: resolved_for_joint_model`。
    17|3. 重建联合场景 → 复跑 `nominal --world joint`：`reference_pose_check.distance_m` 预期 ~5.681e-06 ≤ 0.005，
    18|   再看 s03 三项判据。
    19|
    20|## 2. 本轮新发现的同一类风险（必须先量再决定）
    21|
    22|联合模型的臂来自 **Profile 声明的厂商 MJCF**（`--attach` 用 `spec.model.file`），
    23|而臂自己场景注入的是**声明增益**（`config/piper_simulation_baseline.yaml: scene.arm_position_kp`）。
    24|实测两处 kp 完全不同：
    25|
    26|| 关节 | 联合模型（厂商 MJCF） | 臂自己场景（注入） |
    27||---|---|---|
    28|| joint1 | 10000 | 450 |
    29|| joint2 | 2000 | 200 |
    30|| joint3 | 500 | 200 |
    31|| joint4 | 50 | 200 |
    32|| joint5 | 20 | 200 |
    33|| joint6 | 5 | 200 |
    34|
    35|重力前馈增量 = τ_g / kp ⇒ 继承来的前馈（按 kp=200 算）在联合模型里**按增益比例失真**
    36|（实测与联合模型重算值的最大差 0.016225157 rad，位于 joint5）。这与 pad_offset 是**同一类错误**：
    37|"场景专属数据被静默继承"。因此本轮把前馈也按声明重解（`feedforward_entry`），
    38|并把静态保持残余（core 判据）写进证据 —— 通过与否都用数字说话。
    39|
    40|## 3. 执行队列（每步一个提交，跑绿再走下一步）
    41|
    42|| # | 事项 | 验收 |
    43||---|---|---|
    44|| A1 ✔ | `scenes/handoff_lab/scene.yaml` + `config/scene.schema.json`：声明 `reference_solver`（含 `feedforward_entry`），且 `manipulation_report ⇒ 必须声明 placement + reference_solver`（fail-closed） | `scripts/scene_check.py` exit 0；schema 测试绿 |
    45|| A2 ✔ | `src/iraf_adapters/unitree/scene_builder.py`：按声明加载求解器（文件路径动态导入，不写机型名），把目标换算到**臂基座系**重解，覆写四段 `*_positions`、`pad_offset_m`、`gravity_feedforward`，写入 `reference_pose_source` 与 `reference_pose_resolution` 留痕 | 联合报告 FK 自检距离 ~5.681e-06；单本体产物逐位不变 |
    46|| A3 | 重建联合场景 + `nominal --world joint` | s03 三项判据（`pose_tolerance_m` ≤0.005 / `min_lift_delta_m` ≥0.02 / `require_bilateral_contact`）+ 驻留 `cycles ≥1` |
    47|| A4 | `docs/debug/2026-09-24-joint-model-dog-arm.md` §11.6 + 本文件回写实测 | 数字与报告一致 |
    48|
    49|## 4. 已知风险（本轮可能落到的下一条卡点）
    50|
    51|即便位姿与前馈都按联合模型重解，**厂商增益下 grasp 姿态是否"可静态保持"**是另一件事：
    52|kp=20（joint5）相对该姿态的重力梯度可能不足 ⇒ 残余不随前馈消失。探针
    53|`build/iraf-a6a14/joint_reference_feedforward_probe.py` 同时量两个模型 × 四个姿态的静态残余，
    54|并预测"重解姿态 + 重解前馈保持 1600 ms 后"的门禁距离；若该数字 > 0.005 m，
    55|则下一条卡点是**臂动力学口径**（联合模型必须用与已验收臂场景相同的声明增益），
    56|修法同样是声明化（Profile/机型声明给出执行器刚度），不在构建器里硬编码。
    57|
## 5. 第 2 轮回写实测（提交前）

- A1/A2 完成：`reference_solver` 声明化 + 构建器按声明重解 ⇒ 联合报告
  `reference_pose_check.distance_m = 5.663e-06`（容差 0.005，within=true），原 0.367696068。
  `resolved_pad_offset_m = 0.011452003`（继承 0.029835769）；`local_xy_radius_m = 0.519711458`。
  `scene_check` exit 0；`tests/unit/test_joint_name_map.py` 11 项绿（含 2 条新回归）。
  臂场景产物逐位不变（before/after 三份 diff 全空）。
- A3 部分：`nominal --world joint` ⇒ s01/s02 绿、s03 仍红 `distance=0.274949m`
  （delta=[0.24114787436338767, 0.09218743459485515, 0.09458519690813935]，末态 joint5 −1.0358268081644664 /
  joint6 −0.5718125594197493）。残差已换向 ⇒ 位姿重解生效，卡点 = **臂动力学口径**（kp 表见 §11.6）。
- 故本轮**不**接入 `feedforward_entry`（会让联合构建在静态保持判据处直接失败）。
- A4 已写：docs §11.6 + §12.0。
- 下一轮（B）：执行器刚度口径声明化（Profile 声明 → 构建器按声明注入
  `kp = max(arm_position_kp, damping×ratio)`），判据 = 四个参考姿态在联合模型上的静态保持残余 ≤ tolerance_rad。
