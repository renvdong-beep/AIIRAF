# IRAF 泛化欠债清理 · 总计划（剩余步骤）

- 制定时间：2026-09-20
- 已完成：`docs/debug/2026-09-20-*` 四份记录对应的 7 笔提交
  （夹爪契约泛化、UR5e 适配、去隐式默认值、视觉声明式、Step 2 提 core）
- 之前的单步计划：`.hermes/plans/2026-09-20_120000-generality-step2-core-extraction.md`

## 判断依据（为什么顺序是这样）

审计结论（见对话记录）把泛化欠债按严重度排序：

1. **两条管线分叉**：`build_piper_baseline.py`(472) / `build_ur5_baseline.py`(730)
   与 `build_piper_pick_scene.py`(559) / `build_robot_pick_scene.py`(624) 各一套实现。
   新机型走新路径、老机型走老路径，必然漂移（工具指向符号事故只发生在新路径上）。
2. **可复用能力曾散落机型脚本** —— 已由 Step 2 收敛到 core，但 Piper 侧尚未切过来，
   分叉仍然存在。
3. **验收入口按机型分叉**（`verify_piper_*.py` / `verify_ur5_pick.py`），
   与铁律 6.4 要求的"金路径"不符。
4. **框架侧门禁挡住诚实声明**（factory 能力契约双向）。

## 步骤 A：Piper 侧切到同一套 core（最高优先）

目标：Piper 参考姿态求解不再自持算法，改用 `iraf_core.kinematics` 的
`solve_pose_ik` / `balance_tip_clearance` / `lowest_mesh_point_z`，
使"位置型 IK"之外的能力也只剩一份实现。

- 涉及文件：`scripts/build_piper_baseline.py`（主）、必要时 `scripts/build_piper_pick_scene.py`
- 安全网（**必须先做**）：把当前 Piper 参考姿态与抓取验收数值快照到 `build/`
  （`piper-baseline-pose.json`、`acceptance/piper-pick/report.json`），
  重构后逐字段比对；判据：`joint_positions` 完全一致，
  `finger_center_m` 差 ≤1e-9 m，姿态/抬升/接触力数值不变。
- 验收：
  1. `build/piper_baseline` 重新生成场景成功，残差门禁全部通过；
  2. `verify_piper_pick.py` SUCCEEDED，`lift_delta_m=0.087184`、
     位置误差 `0.002826042685491991`（逐位一致）；
  3. `verify_piper_visual_pick.py` SUCCEEDED（视觉链路未受影响）；
  4. 单元测试无新增失败。
- 不做：不改 Piper 的场景几何、不改 clamp/门禁阈值、不动 `config/piper_*.yaml`
  的参数值（只允许在确有必要时补注释）。

## 步骤 B：统一入口（金路径）

目标：新机型接入 = 填 profile + config + 跑统一命令，而不是挑机型脚本。

- 新增 `scripts/build_baseline.py --baseline <robot>.yaml --scene <path>`
  （内部按配置选择生成器：`scene.generator` 或按 profile 名约定；两个现有生成器
  先作为实现细节保留，入口统一）。
- 新增 `scripts/verify_pick.py --baseline <robot>.yaml --profile <robot>.yaml
  [--target-id] [--duration-ms]`，取代 `verify_ur5_pick.py` /
  `verify_piper_pick.py` 的重复壳；旧脚本保留为薄包装（打印一行 deprecation 提示），
  避免既有文档/CI 立刻失效。
- 验收：新入口对 Piper 与 UR5e 都能复现现有验收数值；文档命令更新。

## 步骤 C：框架侧门禁与诚实声明（需框架窗口确认）

- `factory.verify_backend_contract` 由双向（class ⇄ profile）改为单向
  （declared ⊆ implemented），理由：一个后端类服务多台机器人，能力子集不同。
- 新增装配期校验：`profile.capabilities` 含 `visual_pick` ⇒ 后端配置必须有
  `vision.evidence_file` 或 `vision.detector`，否则装配即失败。
- 落地后：从 `profiles/ur5_mujoco.yaml` 移除 `visual_pick`（UR5 无视觉 Provider）。

## 步骤 D：已知缺口（可并行、按需）

1. UR5e 视觉：检测器 + 相机标定 + 视觉验收，兑现 `visual_pick` 声明。
2. `scripts/view_piper_mujoco.py`、`scripts/verify_viewer_chain.py` 未透传 `vision`。
3. `tcp_offset_from_link6_m`（Piper 命名）待调用方迁移后移除。
4. `gravity_hold_ctrl` 归属：现放 `kinematics.py`，可议迁 `iraf_core/servo.py`。

## 每步的共同纪律

- 先快照 → 再改 → 逐位比对 → 门禁 + 验收 + 单测 → 写调试记录 → 单独提交。
- 提交只 add 自己负责的路径；不执行 `git stash/checkout/reset/add -A`。
- 单步改动尽量小而可审查；不得放宽任何既有门禁来"让测试通过"。
