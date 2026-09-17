# manipulator_grasp 借鉴与 Piper 工程化实施计划

## 目标

借鉴 `/home/nando/manipulator_grasp` 的视觉抓取、碰撞过滤和轨迹规划思路，形成可复用的 AIIRAF Piper/MuJoCo 抓取闭环。外部项目只作为算法参考，不直接复制 UR5、GraspNet 或硬件路径。

## 任务分解

### 阶段 1：轨迹规划基础（已完成）

- [x] 提取五次多项式位置插值，支持关节和笛卡尔阶段的统一时间参数。
- [x] 输入输出使用有限数值校验，禁止轨迹超时或负时长。
- [x] 为插值器补充单元测试和边界测试。

验收：起点/终点位置、速度、加速度满足边界条件；异常输入被拒绝。

### 阶段 2：视觉抓取接口

- [x] 保留 RGB/深度检测和相机到基座坐标转换。
- [x] 将外部项目的点云采样、碰撞过滤抽象为可替换 Provider。
- [x] 输出目标位姿、置信度、坐标系和证据文件。

验收：目标坐标误差、深度有效性、标定版本均可审计。
说明：视觉检测（`scripts/detect_piper_target.py`，EGL 离屏渲染）输出
`build/calibration/piper-vision-target.json`，包含 `vision_world_position_m`、
`pixel_center`、`depth_m`、`frame_id` 与 `source`，由 `visual_pick` 技能读取。

### 阶段 3：Piper 末端与 IK

- [x] 用实测指尖网格几何标定 TCP：抓取点相对指尖 geom 中心沿接近方向偏移 `pad_offset_m`（本次标定 0.0298m）。
- [x] IK 求解遵守 MJCF 关节限位（阻尼伪逆 + 限位裁剪）。
- [x] 预抓取、下降、夹爪闭合使用独立阶段，不得互相覆盖关节控制。

验收：末端到达目标抓取位姿，双指接触力达到阈值。

### 阶段 4：抓取搬运闭环

- [x] HOME_HOLD → APPROACH → DESCEND → GRIP_OPEN → GRIP_CLOSE → 接触力校验 → LIFT。
- [x] 接触失败禁止抬升，目标位移通过物理状态验证（抬升前后目标 Z 位移）。
- [ ] Viewer、HTTP、Skill Runtime 共用同一 Backend 状态机。

验收：成功报告包含接触力、抬升位移、目标最终位置和运行阶段证据。

### 阶段 5：工程化回归与复用

- [x] 增加无 GUI 验收脚本和最小场景。
- [x] 将控制参数移入 `config/piper_simulation_baseline.yaml`，禁止散落硬编码。
- [x] 更新中文调试记录、运行手册和复现命令。

验收：CI 可运行单元测试和仿真验收，失败时不产生伪成功证据。

## 外部项目借鉴边界

- 借鉴：五次多项式轨迹、笛卡尔直线段、点云预处理、碰撞过滤流程。
- 不直接复用：UR5 环境、UR5 关节值、固定相机外参、GraspNet 权重路径和硬件控制接口。
- 所有迁移代码必须经过 Piper MJCF、AIIRAF Safety Policy 和 Runtime Contract 校验。

## 受控仿真基线

模型来源与全部几何/控制参数集中在 `config/piper_simulation_baseline.yaml`：

- 模型：`vendor/agilex_piper/piper_description/mujoco_model/piper_description.xml`，
  由 `vendor/agilex_piper/source-lock.json` 逐文件记录 sha256，构建时校验不一致即失败。
- 参考姿态：`scripts/build_piper_baseline.py` 在生成的探测场景上用阻尼伪逆 IK 求解
  home / approach / grasp / lift 四组关节角，残差必须小于门禁容差的 0.1 倍。
- 场景生成：`scripts/build_piper_pick_scene.py` 负责相对网格路径、目标贴台高度、
  指尖摩擦、位置增益和参考姿态注入。

### 关键几何标定结论（本次实测）

1. **指尖 geom 中心不是抓取点**：Piper 手指网格约 100mm 长，网格中心比指腹接触区高约 30mm。
   直接把方块中心当作指尖中心会让指尖扎入工作台 19.5mm，并把 link6 挤入方块 15.7mm。
   现在由构建器按 `grasp.tip_clearance_m`（5mm）自动配平抓取高度，偏移量写入
   `gripper.pad_offset_m` 并在对齐门禁中减掉。
2. **夹爪最大开度 70mm**：`joint7/8` 行程为 0.035m，60mm 方块完全张开时两侧只剩 5mm 余量，
   下降阶段会刮碰目标。目标改为 50mm 方块（`target.half_size_m: 0.025`），余量 10mm。
3. **闭合不能命令到行程末端**：位置控制夹爪命令到 0 会以巨大接触力挤压目标并让仿真发散。
   `gripper.closed` 取 0.023（约 2mm 挤压量）。
4. **位置增益上限**：`scene.arm_position_kp` 取 200。800 在无阻尼的 2ms 步长下发散
   （实测 `max|qvel|` 达到 240rad/s，目标被弹出场景），不要仅凭"稳态误差大"继续调高。

## 当前实施状态（视觉抓取闭环已通过）

在 `10.203.247.145:~/AIIRAF`（MuJoCo 3.3.3，EGL 离屏渲染）上的最新结果：

### 无视觉真值抓取（`pick_object`）

- `status: SUCCEEDED`，`confirmation: contact`（未使用焊接夹具，抬升完全由双指摩擦承担）
- 双指接触：`bilateral_contact=true`，法向力 0.243N / 0.246N（阈值 0.2N）
- 抬升位移：`lift_delta_m=0.084181`（阈值 0.02），`lifted=true`
- 抓取位姿：`center_distance_m=0.002246`（容差 0.005）

### 视觉抓取闭环（`visual_pick`）

- `status: SUCCEEDED`，`confirmation: contact`
- 视觉检测：相机固定，内参来自标定文件（`focal_px=292.11`，`principal_point_px=[319.83, 239.51]`）
- 视觉坐标误差 **1.15mm**（不使用任何硬编码偏移）
- 视觉坐标直接作为抓取点喂给 `pick_object`，由 `pad_offset_m` 标定换算到指尖中心
- 接触/抬升证据与无视觉真值验收一致

复现命令：

```
cd ~/AIIRAF
# 1. 构建受控基线场景（含固定相机）
python3 scripts/build_piper_baseline.py --baseline config/piper_simulation_baseline.yaml \
  --scene build/models/piper-pick-scene.xml
# 2. 相机标定（外参 + 内参）
PYTHONPATH=src python3 scripts/verify_camera_calibration.py \
  --scene build/models/piper-pick-scene.xml
# 3. 无视觉真值抓取验收
PYTHONPATH=src python3 scripts/verify_piper_pick.py \
  --source vendor/agilex_piper/piper_description/mujoco_model/piper_description.xml \
  --scene build/models/piper-pick-scene.xml \
  --output build/acceptance/piper-pick \
  --baseline config/piper_simulation_baseline.yaml --duration-ms 12000
# 4. 视觉抓取闭环验收（自动先跑视觉检测）
PYTHONPATH=src python3 scripts/verify_piper_visual_pick.py \
  --source vendor/agilex_piper/piper_description/mujoco_model/piper_description.xml \
  --scene build/models/piper-pick-scene.xml \
  --output build/acceptance/piper-visual-pick \
  --baseline config/piper_simulation_baseline.yaml --duration-ms 12000
# 5. 深度链路体检（多目标验收准入门槛）
PYTHONPATH=src python3 scripts/verify_camera_depth.py \
  --scene build/models/piper-multi-scene.xml \
  --baseline config/piper_multi_target.yaml --target-id box_red
# 6. 多目标 / 未知姿态验收（逐目标按 ID 抓取）
PYTHONPATH=src python3 scripts/verify_piper_multi_target_pick.py \
  --baseline config/piper_multi_target.yaml \
  --output build/acceptance/piper-multi-target-pick --duration-ms 12500
# 7. 单元测试
PYTHONPATH=src python3 -m unittest discover -s tests/unit -t tests/unit -v
```


单元测试：共 120 项（原 92 项 + 多目标 / 姿态拟合 28 项），
抓取、标定与姿态估计相关全部通过；`test_vision_processing` 1 项失败与
`test_agentos_bridge` / `test_coding_worker_contract` / `test_runtime_grpc` /
`test_runtime_http_health` 4 项导入错误为本次改动之前已存在的问题，与抓取链路无关。

下一步：在保持固定相机与标定内参可审计的前提下扩展多目标 / 未知姿态场景。
（已完成，见下节）


### 扩展进度

- [x] `pad_offset_m` / `pad_offset_axis` 配置解析单元测试（`test_piper_visual_pick.PadOffsetConfigTests`，4 项）
- [x] `VisualPickProvider` 证据流单元测试（`test_piper_visual_pick.VisualPickProviderTests`，3 项）
- [x] 相机标定单元测试（`test_camera_calibration`，8 项：外参刚体拟合 + 内参真值恢复 + 边界拒绝）
- [x] 相机固定在支架上并对准抓取点，外参与内参均可标定
- [x] 删除 `detect_piper_target.py` 中硬编码的 `[0.003676726, -0.001277797, 0.0]` 偏移
- [x] 多目标 / 未知姿态场景验收（详见下节）

## 多目标 / 未知姿态验收

### 6DoF 获取路径：RGB-D 深度通道

| 路径 | 结论 |
| --- | --- |
| **A. RGB-D（采纳）** | 相机已标定，深度渲染 + 内参反投影即得相机系点云；纯色方块无纹理也不受影响 |
| B. 多视角立体 | 方块是纯色无纹理，立体匹配无法建立可靠对应，否决 |
| C. 标记点（AprilTag / 贴图 PnP） | 精度高但等于把姿态答案贴在物体上，削弱未知姿态验收的意义，保留为降级备选 |

选 A 的额外好处：只加"深度读取 + 反投影 + 拟合"，完全复用既有内参与外参标定，
不触碰 `calibrate_camera_to_base` / `_calibrate_intrinsics`。

### 目标区分机制：颜色为主、ID 为辅

- MJCF 层：每个目标生成独立 `material`（rgba），命名沿用 `box_*`；
- 感知层：按归一化颜色距离分割得到每目标掩码，掩码 ∧ 有效深度 → 该目标点云；
- 语义层：配置给出 `id → rgba` 映射，检测结果携带 `id`，
  上层按"抓红色方块"这类语义请求，避免"检测顺序即 ID"的脆弱性。

### 姿态拟合：顶面平面拟合 + 平面内最小外接矩形

俯视单目只能看到顶面与部分侧壁，直接对全部可见表面做 PCA / 模板配准
会被"可见面偏置"带偏。稳健流程：

1. 掩码内深度点反投影到相机系 → 外参变换到世界系；
2. 基于**到点云中心的距离**做 MAD 去噪；
3. RANSAC 拟合**最大支撑平面**（顶面）→ 法向 `n`；
4. 顶面点投影到平面内，用旋转卡壳求**最小面积外接矩形**
   → 中心 `c`、边长 `(w, h)`、面内主轴 `a`；
5. 校验 `(w, h) ≈ 2 * half_size`，超出容差即抛错，不做静默兜底；
6. 顶面中心沿 `-n` 偏移 `half_size` 得立方体中心；
7. 构造旋转矩阵 z = n、x = a（正交化）、y = z × x，做 90° 对称消歧；
8. 输出四元数（wxyz）与残差指标。

### 抓取位姿运行时求解

- 抓取点 = 拟合出的立方体中心，叠加既有 `pad_offset_m`（指尖中点 → 抓取点换算）；
- 接近方向 = 目标顶面法向 `n`（不再是配置里写死的 `[0,0,1]`）；
- 降级链：法向与竖直夹角超过 `grasp.max_tilt_deg` 时回退竖直抓取，
  并在证据里标注 `grasp_mode: fallback_vertical`；
- `validate_grasp_pose` 兜底拒绝碰台 / 碰目标的解。

### 排查过程中定位到的四个真实缺陷

多目标场景把单目标路径中被掩盖的问题全部暴露出来，逐一修复：

1. **MuJoCo 深度渲染输出已经是米制线性距离**，不是 [0,1] 的 OpenGL 深度缓冲。
   初版按 `z_ndc` 反算导致点云数值被彻底破坏（出现 -1759m 等荒谬值）；
   实际最大值等于 `zfar`（113.16m）即为此结论的直接证据。
   另外无几何像素被写成精确的 `1.0`（不是 `zfar`），必须单独掩膜剔除。
2. **逐轴独立 MAD 在多面体点云上失效**：俯视倾斜方块时，某一轴上有约一半的点
   恰好聚在极窄的带上，该轴 MAD 接近 0，门限收缩到亚毫米并把同属目标的另一半
   点全部误剔除（实测只剩 43/315 点）。改为基于"到点云中心距离"的各向同性判据。
3. **`joint1` 的位置增益被压到低于自身阻尼**：场景生成器把所有臂关节 kp 统一设为
   `arm_position_kp = 200`，而 `joint1` 的 `damping = 300`，
   阻尼主导后执行器永远到不了目标角（实测只走到目标的 41%~62%）。
   改为 `kp = max(arm_position_kp, damping * 1.5)`，`joint1` 因此取 450，
   稳态误差降到 0.0001 rad。
4. **稳定窗口按"段时长"缩放开销过大**：`_move_trajectory` 的稳定时间原为
   `min(2000, duration//2)`，而调用方传入的是每段时长（总时长 / 5），
   实际只有约 2 秒，`joint1` 这类高阻尼关节远未收敛。改为按段时长成比例放宽
   （上限 16 秒）。

此外还修正了两处设计问题：`gravcomp` 只给"被抓取的那个目标"
（否则干扰物被擦碰后会长期漂浮，实测被顶到 0.14m 高且不落回）；
姿态比较必须折叠正方体的 90° 对称等价类
（否则同一几何朝向被误报为 89.7° 偏差）。

### 验收结果

```
TARGET box_red    passed=True  pos_err=0.001224m  ori_err=0.94°  mode=pose_adaptive  status=SUCCEEDED
TARGET box_green  passed=True  pos_err=0.001212m  ori_err=0.71°  mode=pose_adaptive  status=SUCCEEDED
TARGET box_blue   passed=True  pos_err=0.002026m  ori_err=0.82°  mode=pose_adaptive  status=SUCCEEDED
```

判据沿用既有四项（命中指定 ID + 双指接触 + 抬升位移 + 坐标误差阈值），
未新增判据语义：

- **命中目标**：Skill 回传的 `target_body` 必须等于请求的 `target_id`；
- **双指接触**：左右指法向力均 ≥ `min_normal_force_n`（0.2N），
  实测 0.242 / 0.243 N，力不平衡比 1.007；
- **抬升位移**：`lift_delta_m ≥ min_lift_delta_m`（0.02m），实测 0.0865 m；
- **坐标误差**：检测位置 ≤ 5mm、姿态角 ≤ 10°，实测最大 2.03mm / 0.94°。

深度链路准入门槛（`verify_camera_depth.py`）：
水平误差 1.93mm、高度误差 0.10mm、法向角 1.17°（容差 4mm / 4mm / 5°），
顶面 ROI 内 168 点。

该脚本的 `--min-points` 默认值与 `--half-size` 真值解析已修正：

- **`--min-points` 原为 300，与脚本自身注释矛盾**：注释已写明 50mm 方块在 1m 视距下
  顶面只有约 15x15 像素、±4mm 窄带内实测仅约 170 点，300 的门槛必然误报
  "顶面 ROI 内点数不足"。现改为 100，与物理量级一致。
  注意这是**门禁参数**而非精度判据 —— 通过与否只由几何误差决定，
  点数门槛只用于拦截"相机完全没对准"这类无效输入。
- **`--half-size` 原只从顶层 `target` 段读取**：多目标配置的半尺寸真值在
  `targets[]` 各元素或顶层 `target` 中，而请求的 `target_id` 可能是 `box_red`。
  原实现会把 `box_01` 的半尺寸套到 `box_red` 上，真值错位。
  现按 `target_id` 精确匹配 `targets[]`，未声明时回退顶层；
  两者都找不到时**显式报错**，不允许套用其它目标的真值。
- **解析顺序**：`target_id` 必须先于 `half_size` 解析，否则 `target_id` 仍为 `None`
  时会匹配失败并误报"配置里没有真值"。

### 已知边界

- **倾斜目标（r/p ≠ 0）**：夹爪尚未实现绕进近轴的姿态对齐，
  倾斜 50mm 方块进近时张开的手指会与棱角干涉并把目标推走（实测推开 57mm）。
  当前配置将目标 r/p 置 0，并通过 `validate_grasp_pose` 显式拦截这类姿态，
  不做静默通过。
- **目标布局约束**：相邻目标中心间距需 ≥ 100mm（方块 50mm、夹爪张开内距 70mm），
  且 y 偏移宜控制在 ±150mm 内 —— `joint1` 阻尼高，大角度构型下
  抓取点会出现 1cm 量级偏差（实测 y = 180mm 时 5.55mm、y = 110mm 时 7.78mm）。

复现命令（在多目标验收之前先跑深度体检）：

```
# 6. 深度链路体检（多目标验收的准入门槛）
# target-id 与 half-size 均可缺省：目标取 targets[] 首个，
# 半尺寸按该 id 从 targets[] / 顶层 target 自动解析。
PYTHONPATH=src python3 scripts/verify_camera_depth.py \
  --scene build/models/piper-multi-scene.xml \
  --baseline config/piper_multi_target.yaml
# 7. 多目标 / 未知姿态验收（逐目标按 ID 抓取）
PYTHONPATH=src python3 scripts/verify_piper_multi_target_pick.py \
  --baseline config/piper_multi_target.yaml \
  --output build/acceptance/piper-multi-target-pick --duration-ms 12500
```

多目标验收逐目标生成场景（目标位置严格取自配置，不被搬到参考点）、
按该目标位置重新求解参考关节姿态，再走 Skill Runtime 调用 `visual_pick(target_id)`。


## 视觉精度治理（关键结论）

原先视觉坐标依赖一笔写死的偏移 `[0.003676726, -0.001277797, 0.0]`，
只在 `(0.19, 0)` 这单一位置凑出 1.2mm 误差。定量实验定位到两个根因：

1. **相机是 `targetbody` 模式**，始终锁定目标 → 相机位姿随目标移动，
   内参不可标定，且方块恒落在图像中心附近，`focal` 方向无约束。
   跨视野实测误差 **3.3mm 均值 / 4.6mm 最大**，且随目标位置不可预测。
2. **掩码质心 ≠ 几何中心**：斜视时方块可见面亮度不均，
   质心被暗侧面拉偏约 1 像素，经 1m 视距放大即数毫米。

治理措施：

- 相机改为 `mode="fixed"`，位姿由 `scene.camera`（位置 + 注视抓取点）声明，
  四元数由构建器 `_look_at_quat` 计算，光轴精确指向抓取点（投影 320.0/240.0）。
- 新增 `calibrate_camera_to_base`：外参用 Kabsch 迭代刚体拟合（精度 1e-15m），
  内参用 Levenberg-Marquardt 求 `focal_px` 与 `principal_point_px`
  （标定值 292.1 / [319.83, 239.51]，与理论 296.4 吻合，残差 1.08px）。
- 检测与标定统一改用**掩码包围盒中心**表示目标几何中心。
- 检测脚本删除硬编码偏移，改用标定文件内参；相机非 fixed 模式时直接拒绝检测。

结果：视觉坐标误差 **1.15mm**（`[-1.15, -0.09, 0]`），不再依赖任何手工魔数。
