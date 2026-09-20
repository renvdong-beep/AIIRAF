# UR5e + 2F-85 抓取适配进展报告

## 一、已完成并验证

### 1. 模型组装（`scripts/assemble_ur5e_2f85.py`）

从 Menagerie 官方模型组装 UR5e + Robotiq 2F-85，**两个静默致命坑已修复**：

| 坑 | 症状 | 修复 |
| --- | --- | --- |
| A. 同名网格被复用 | 夹爪基座变成机械臂基座（顶点数 10899→6532） | mesh 的 `file` + `name` 都加前缀、复制文件、同步改写 `geom/mesh=` |
| B. mesh scale 被 default 覆盖 | 网格保持毫米单位 → 质量放大 **1e9 倍** → 夹爪完全驱不动 | 每个 `<mesh>` 元素**显式**写 `scale`，并加编译后自检门禁 |

**自检门禁**（编译后立即断言，不通过就报错）：
```
夹爪网格 max span 0.094146 m  (限 0.2)
夹爪 body max mass 0.777441 kg (限 5.0)
```

**决策落地**（q1/q2/q3）：
- **forcerange 回退官方 ±5N** ✓ — 实测官方值足够：修好网格后行程 84.729mm，与官方单模型 84.795mm 偏差仅 **0.5%**
- pinch site 保留原名别名（`rq2f85_pinch` + `pinch` 并存）✓
- 诊断脚本全部保留 + 本文档

### 2. 物理正确性验证（对等对照）

```
官方 2f85.xml 单独加载：ctrl 0→255，gap 0.0854→0.0004，行程 84.795mm，严格单调
组装后同一模型：        ctrl 0→255，gap 0.0851→0.0004，行程 84.729mm，严格单调
driver 全程落在 range [0, 0.8] 内，无 QACC 发散告警
```

### 3. Profile 与基线（新增文件）

- `profiles/ur5_mujoco.yaml` — 显式声明 `gripper.type: tendon`、
  driver 关节（0..0.8 rad）、home、相机
- `config/ur5_simulation_baseline.yaml` — 工作台/目标/抓取/夹爪/场景/验收
- `scripts/check_ur5_profile.py` — profile 与基线交叉一致性校验（**PASS**）

### 4. 机器人无关化改造（后端 6 处补丁）

`scripts/patch_backend_generalization.py` 用"精确替换 + 命中数断言"实现，
避免静默改错。原实现里写死 Piper 名字的地方全部改为读配置声明：

| 位置 | 原写死 | 改为 |
| --- | --- | --- |
| `_set_gripper_controls` | `{"joint7","joint8"}` 过滤 | 直接用调用方传入的夹爪通道字典 |
| `_grasp_alignment_evidence` | `piper_left_finger` geom | 读 `gripper.left_finger_geom`（缺省仍是 Piper 名） |
| 同上 `joint_qpos` | `joint1..joint6` | 从 profile 的 arm 角色关节推导 |
| `_log_pick_phase` / `calibrate_grasp` | `link6/link7/link8` | `wrist_body` / `*_finger_body` |
| `_joint_qpos` | 只按 actuator 名查 | 兼容关节名与执行器名 |
| `_parse_manipulation_config` | 只白名单已知字段 | 透传 `wrist_body` / `*_finger_geom` |

**Piper 兼容性（已验证）**：
后端补丁**前后单测结果逐项一致** —— 均为 `Ran 220 tests, failures=1, errors=4`。
- 补丁过程中一度引入 3 项新错误（`test_mujoco_backend_metrics` 用
  `SimpleNamespace` 替身 Model，没有 `nkey` 字段，直接取属性抛
  AttributeError），已改为 `getattr(self.model, "nkey", 0)` 探测后消除；
- 剩余 4 项 error 是 4 个模块导入错误（`test_agentos_bridge` /
  `test_coding_worker_contract` / `test_runtime_grpc` /
  `test_runtime_http_health`），1 项 failure 是 `test_vision_processing`，
  均为本次改动**之前就存在**（项目历史文档已记录）。

### 5. 机器人无关场景生成器（新增 `scripts/build_robot_pick_scene.py`）

机器人身份全部来自基线配置，脚本内不含机型名称：
`model.arm_joints` / `model.finger_geoms` / `model.bodies` /
`scene.inject_arm_position_gains`。

**关键泛化点**：
- **关节名 → 执行器名翻译**：后端按 actuator 名索引 ctrl，
  Piper 两者同名、UR5e 不同名（`shoulder_pan` vs `shoulder_pan_joint`）
- UR5e **不注入** `arm_position_kp`（官方 `<general>` 自带阻尼位置伺服，
  覆盖会破坏官方标定）
- keyframe 覆盖为抓取起始位形 + 目标位姿（消除跨台面扫掠）

### 6. 带姿态约束的抓取逆解（`scripts/build_ur5_baseline.py`）

**问题**：UR5e 是 6 自由度通用臂，同一 pad 位置有无穷多组关节角。
只约束位置时求解器落到"夹爪横着伸出"的病态解：
```
开合轴 = [0.749, 0.143, -0.647]   (z 分量 -0.65，斜插向下)
两 pad 高度差 60.5mm，夹爪指向与 -Z 夹角 71.3°
```

**走不通的两条路**（都已实测排除）：
- 零空间姿态修正用"pad 中点 - 法兰"位置差雅可比 → 两者同挂腕部末端树，
  偏导数完全抵消（范数 ~1e-16），指向偏差恒为 90.000° 不收敛
- 改用旋转雅可比 → 指向可收敛，但开合轴卡在 41~45°（左右 pad 旋转雅可比
  几乎相同，任何"两指相减"构造都退化）

**最终方案**：把姿态期望写成法兰的**目标旋转矩阵**，
对 6 维位姿误差 `[位置(3); 旋转(3)]` 做阻尼最小二乘。

```python
法兰约定（实测官方 home）：z 轴 = 夹爪指向，x 轴 = 开合轴
11 次迭代收敛：
  pad 中点误差 4.5e-07 m
  夹爪指向偏差 0.000°     开合轴偏差 0.000°
  两 pad 高度差 0.000000 m
```

生产实现里四个参考姿态（home/approach/grasp/lift）**全部解出**：
```
grasp 残差 2.9e-08 m   姿态偏差 0.0086°   两 pad 高度差 1.4e-05 m
```

### 7. 发现并修复的两个执行期问题

| 问题 | 证据 | 修复 |
| --- | --- | --- |
| HOME 段跨台面扫掠 | `probe_home_transit.py`：零位→HOME 插值 t=0.33 时 pad_z=**-0.0047m**（穿台面），沿途把方块顶到 z=5.3m | 后端 `mj_resetDataKeyframe(model, data, 0)` 按 keyframe 初始化位形 |
| keyframe 目标段为零 | 方块被搬回世界原点，报 `距离 0.567m` | keyframe 里同时写入目标自由关节的位置+四元数 |

## 二、当前阻塞点

> **2026-09-20 更新（结论已修正）**：本节原先记录的两条推断
> （"DESCEND 跟踪误差 0.498 rad 是伺服/增益问题"、"根因是 pad1 中心当抓取点
> 导致整套 pad 下沉 34mm"）都**不是根因**。实测根因是
> **工具指向被写成了后退方向**：法兰 z 轴朝上、夹持区落在法兰上方 134mm，
> 抓取位姿要求法兰压到台面以下（z=-0.109m）而不可达，实际执行时
> `wrist_2_link` 碰撞体先压在方块顶面上。完整证据链、修复与验证见
> `docs/debug/2026-09-20-ur5-tool-axis-flip.md`。
> 修复后三段跟踪误差为 0.0144 / 0.0144 / 0.0152 rad，方块全程未被推动；
> 验收残差由 0.314m 一路降到 **0.000119m 并 SUCCEEDED**：
> 0.314m（工具指向反了）→ 0.0166m（姿态修正后，纯 PD 重力静差）→
> 0.0095m（前馈接线后，暴露抓取点口径不一致）→ 0.000119m（三方同口径）。
> Piper 验收与 229 项单测均无回归。
> 以下两节保留为历史记录，不再作为当前结论。

### DESCEND 段的跟踪误差（0.498 rad）

三段轨迹复现（`probe_tracking.py`）显示跟踪误差随夹爪逼近方块而**单调增大**：

```
HOME_HOLD  最大跟踪误差 0.017 rad   box_01=[-0.550, -0.134, 0.025]
APPROACH   最大跟踪误差 0.210 rad   box_01=[-0.570, -0.067, 0.034]
DESCEND    最大跟踪误差 0.498 rad   box_01=[-0.714, -0.053, 0.052]
```

说明 pad 在下降过程中**推挤方块**，反作用力把 `shoulder_lift` 顶住。

### 根因（`probe_pad_vs_block.py` 量出的真实几何）

câ
```
left_pad1  内表面 x=-0.60070  方块左面 x=-0.575  → 间隙 0.0257 m  ✓ 水平方向正确
right_pad1 内表面 x=-0.49930  方块右面 x=-0.525  → 间隙 0.0257 m  ✓
pad 开合间距 0.1014 m  vs  方块 0.050 m          → 每侧余量 25.7mm  ✓

但竖直方向：
pad1 底面 z=+0.01562  方块顶面 z=+0.05000  → pad 底面比块顶**低 34.4mm**
pad2 底面 z=-0.00313                       → pad2 扎进台面 3.1mm
```

**水平方向完全正确，问题在高度**：
IK 把 **pad1 geom 中心**对齐到方块中心（z=0.025），
但 2F-85 的 pad 是**两个上下排列的 box**（`pad1` 在局部 +z、`pad2` 在 -z），
其中点并不是"夹持中心"。结果整套 pad 相对方块**下沉了约 34mm**，
`pad2` 甚至穿过台面，下降时下侧 pad 先撞到方块侧面。

## 三、下一步（待你确认）

需要把"抓取点"的定义从 `pad1 中心` 改成 **pad1/pad2 的重叠夹持区中心**，
并相应调整 `pad_offset_m`。两条可选路径：

| 方案 | 做法 | 代价 |
| --- | --- | --- |
| **A. 用 pad1/pad2 四点中点作为 IK 目标** | 契约层支持多个点求中点，把 4 个 pad box 都传进去，得到真正的夹持区中心 | 需要改 `build_ur5_baseline.py` 的求解目标；契约层不用改 |
| **B. 保持 pad1 中心为目标，配置 `pad_offset_m` 补偿** | 在 yaml 里给一个显式的竖直偏移量，把整套 pad 抬高 34mm | 改动最小，但偏移量是"魔数"，换块高就失效 |
| **C. 用 pinch site 作为 IK 目标** | 官方 `pinch` site 在 `base` 上 pos="0 0 0.145"，是夹爪的标准参考点 | 需要量出 pinch 与夹持区的偏移，语义更标准 |

我倾向 **A**（几何自洽、不引入魔数），但也可以先做 **C** 再验证。
