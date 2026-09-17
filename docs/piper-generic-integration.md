# Piper 内容梳理与 IRAF 通用化集成说明

本文档记录「把 Piper 收敛为 IRAF 通用框架下一等实现」的梳理结论、迁移映射、
通用接口说明与遗留项。目标是让机械臂能力 arm-agnostic：换机械臂只需替换
`profiles/` 与 `adapters/` 中的实现，`iraf_core/`、`iraf_skills/` 不作改动。

## 1. 通用化的边界（依据 AGENTS.md）

`AGENTS.md` 铁律第 5 条：**TaskFlow、Skill Runtime、Policy 等核心逻辑不得依赖
某块板、某个 ROS topic、GPU/NPU SDK 或发行版私有库；硬件差异只能放在
`adapters/` 与 `profiles/`。**

因此本次通用化不是"新建一层抽象"，而是让 Piper 严格落回既有边界：

1. 补齐既有 `RobotBackend` Protocol（`src/iraf_adapters/backend.py`）；
2. 把机器人差异下沉到 `RobotProfile`（`profiles/`）与场景报告；
3. 显示入口与验收脚本走既有 `factory.load_backend` 动态装配；
4. 不动 `iraf_core/`、`iraf_skills/`。

## 2. 耦合点清单

调研时的 Piper 耦合分布（`grep -ric piper`）：

| 位置 | 耦合数 | 分类 | 结论 |
| --- | --- | --- | --- |
| `src/iraf_core/` | 0 | — | 已 arm-agnostic，保持不动 |
| `src/iraf_skills/` | 0 | — | 已 arm-agnostic，保持不动 |
| `src/iraf_adapters/mujoco/pose_estimation.py` | 0 | — | 纯算法 |
| `src/iraf_adapters/mujoco/depth_channel.py` | 0 | — | 纯算法 |
| `src/iraf_adapters/mujoco/mujoco_backend.py` | 11 | **适配层合理耦合** | Piper 是当前唯一 MuJoCo 实现，位置正确 |
| `scripts/verify_piper_*.py` | 13~20 | **验收脚本专属** | 脚本名即场景名，属正常 |
| `scripts/build_piper_*.py` | 9~17 | **场景构建专属** | 同上 |
| `scripts/view_piper_mujoco.py` | 8 → 2 | **已收敛** | 内联魔数已全部迁出 |
| `skills/piper/basic.py` | — | **遗留垫片** | 已移除（见第 5 节） |

**核心结论**：框架内核零耦合，Piper 耦合本就集中在正确位置（`adapters/` + `scripts/`）。
本次工作的实质是**消除脚本层的魔数**与**补齐契约**。

## 3. 通用接口说明

### 3.1 `RobotBackend` 契约（`src/iraf_adapters/backend.py`）

既有 4 个方法签名**不得改动**（多处按位置参数依赖）：

```python
def move_joint(self, positions: dict, duration_ms: int, lease) -> None: ...
def pick_object(self, target_id: str, grasp_pose: dict, duration_ms: int, lease) -> dict: ...
def stop(self, lease) -> None: ...
def step(self, count: int = 1) -> dict: ...
```

本次**新增**的可视化与生命周期契约：

| 方法 | 用途 | 说明 |
| --- | --- | --- |
| `start_continuous()` | 启动连续步进 | 长周期仿真生命周期 |
| `stop_continuous(timeout=2.0)` | 停止连续步进 | 幂等 |
| `is_continuous()` | 查询运行状态 | 布尔 |
| `render_frames(count=1)` | 取 RGB 帧序列 | 内部加锁，返回 numpy 数组列表 |
| `home_pose()` | 取 Home 位姿 | 优先 profile 的 `home` 段，回退 manipulation |
| `hold_current_pose()` | 锁存末态位形 | 防止 Runtime 收尾后塌回零位 |
| `simulation_status()` | 运行指标快照 | 已有实现，纳入契约 |
| `display_lock()` | 显示同步互斥 | **必须**：`viewer.sync()` 会复制 `mjData`，须与步进互斥 |

`display_lock()` 的必要性有实测依据：不加锁时 MuJoCo 报
`mj_copyDataVisual: attempting to copy mjData while stack is in use`。

### 3.2 `RobotProfile` 结构化段（`profiles/piper_mujoco.yaml`）

新增字段**全部可选**，追加在 dataclass 末尾并带默认值，缺失时回退既有平铺语义：

| 字段 | 作用 | 消除的魔数 |
| --- | --- | --- |
| `joint_roles` | 关节语义（arm / gripper_drive_left / gripper_drive_right） | 原来靠 `joint7`/`joint8` 字面量判断指侧 |
| `gripper.drive_joints` | 夹爪驱动关节 | `_set_gripper_controls` 内硬编码的 `{"joint7","joint8"}` |
| `gripper.open_positions` / `closed_positions` | 开合位形 | viewer 内联字典 |
| `gripper.pad_offset_m` / `pad_offset_axis` / `max_tilt_deg` | 抓取点偏移与倾角上限 | 接近轴回退阈值 |
| `home` | Home 位姿 | viewer 内联的 `{"joint2":0.65,...}` |
| `camera` | 初始视角 | viewer 内联的 `cam.lookat/distance/azimuth/elevation` |
| `manipulation` | 目标与容差 | viewer 内联的 targets 字典 |

### 3.3 场景报告（`build/models/*.json`）

场景旁挂报告由 `scripts/build_piper_baseline.py` 产出，含 IK 求得的
`approach_positions` / `grasp_positions` / `lift_positions` / `pad_offset_m`，
以及 `targets[]` 的目标真值。**显示入口直接复用该报告，不重算 IK、不内联魔数。**

## 4. 迁移映射

| 迁移项 | 迁移前 | 迁移后 |
| --- | --- | --- |
| Home 位姿 | `view_piper_mujoco.py` 内联字面量 | `profile.home` + 场景报告 `home_positions` |
| 相机视角 | 内联 `cam.lookat/distance/azimuth/elevation` | `profile.camera` |
| 夹爪指体 | 内联 `link7` / `link8` | 场景报告 `gripper.left/right_finger_body` |
| 开合位形 | 内联字典 | `profile.gripper` + 场景报告 |
| 抓取参数 | 内联 `manipulation` 全量字典 | `_manipulation_from_scene_report()` |
| 后端装配 | `import MujocoBackend` 直接实例化 | `build_runtime_from_env()` → `factory.load_backend` |
| 私有成员访问 | `backend._set_controls` / `_set_gripper_controls` / `_data_lock` | `move_joint` / `hold_current_pose` / `display_lock` |
| 目标定位 | 硬编码 `box_01` | 场景报告 `target_id`（支持多目标） |
| 遗留垫片 | `skills/piper/basic.py` | 已移除，PYTHONPATH 同步清理 |

## 5. 遗留垫片清理

`skills/piper/basic.py` 自述"Piper 不再拥有独立 Skill 实现"，仅转发到 `motion`。
移除依据：

1. 未在任何 `skill.yaml` 中注册为 provider；
2. 部署用 systemd service 的 `PYTHONPATH` 均未包含 `skills/piper`；
3. `tools/coding_worker.py` 与 `tools/orchestrate_cycle.py` 仅在 PYTHONPATH
   字符串中提及，自身无 `import` 依赖。

已移除该目录并同步修正上述两个脚本的 PYTHONPATH，校验 `REMAINING_REFS 0`。

## 6. 可视化架构与两个实测约束

`src/iraf_adapters/mujoco/viewer_runner.py` 是通用显示入口，不依赖任何具体机械臂。

**约束一：窗口必须与抓取串行化，不能只靠互斥锁。**

`mujoco.viewer.launch_passive` 会创建**自己的 GLFW 渲染线程**，该线程在任意
时刻调用 `mjv_updateScene`（内部 `mj_copyDataVisual`）读取 `MjData`。

关键实测结论：**即使调用方用 `display_lock()` 包住自己的 `sync()`，也无法
阻止 viewer 内部渲染线程与抓取线程的 `mj_step`/`mj_forward` 交错**——
实测三次中有两次报
`mj_copyDataVisual: attempting to copy mjData while stack is in use`。

因此当前设计为两阶段串行：

1. **阶段一**：抓取在**无窗口**状态下完成（约 2 秒），锁存末态；
2. **阶段二**：再打开窗口，渲染抓取结果并保持末态，可继续步进。

改造后连续三次验收全部通过，竞态消除。代价是抓取执行期间窗口不显示过程，
这是 `mujoco.viewer` 平台限制下的必要取舍，已如实记录而非掩盖。

**约束二：显示的实时性与技能租约预算存在固有冲突。**
实测 `realtime=True` 时每步 2.42ms，`pick_object` 需约 38000 步 → 约 92 秒，
远超技能声明的 30 秒 TTL；而 `realtime=False` 时约 2 秒即完成（SUCCEEDED）。
因此显示链采用 `realtime=False` 让抓取在仿真时间内快速完成，**不通过放宽后端
时序语义或延长租约来掩盖问题**。

显示路径判定：`interactive_viewer`（真实 GLFW 窗口）/ `offscreen_frames`
（EGL 离屏降级）/ `unavailable`（显式失败）。降级时报告如实标注 `fallback_reason`。

注意：在**将要开启 `launch_passive` 的同一进程内**，不得先做 GLFW 试创建窗口
（会残留 GL 上下文状态，导致数据拷贝报错），故显示判定使用
`resolve_display_mode(allow_probe=False)`，仅依据 X11 socket 连通性。

## 7. 验证结果

### 7.1 显示链验收（新增）

```
display_mode : interactive_viewer   (GLFW 3.4.0 GLX @ DISPLAY=:0)
phases       : HOME → HOLD
hold_pose    : True
status       : SUCCEEDED
lift_delta   : 0.087224 m
bilateral    : True
grasp_mode   : pose_adaptive
左/右指法向力 : 0.247 / 0.241 N
passed       : True
```

### 7.2 既有验收链回归（零回归，数值与基线一致）

| 验收项 | 结果 |
| --- | --- |
| 单目标无视觉抓取 | exit=0，SUCCEEDED |
| 单目标视觉闭环 | exit=0，SUCCEEDED |
| 深度链路体检 | exit=0，passed=true（水平 2.76mm / 高度 0.35mm / 法向 3.25°） |
| 多目标 / 未知姿态 | exit=0，3/3（pos_err ≤ 2.03mm，ori_err ≤ 0.94°） |

### 7.3 单元测试

- 新增 `tests/unit/test_viewer_contract.py`：**22 项全部通过**
- 抓取/标定/姿态/契约专项：**85 项全部通过**
- 全量 137 项：136 通过，1 失败（`test_vision_processing`）——
  经 `git diff --stat` 确认 `vision` 相关文件**未出现在本次改动中**，
  该失败为改动前固有基线问题。

## 8. 复现命令

```bash
cd <仓库根目录>
export PYTHONPATH=src

# 显示通道准入探测（独立进程，可做 GLFW 试创建）
python scripts/probe_viewer_display.py --output build/acceptance/viewer-display-probe/report.json

# 交互式查看器（需 DISPLAY，远程主机为 :0）
export DISPLAY=:0 MUJOCO_GL=glfw
python scripts/view_piper_mujoco.py --seconds 40

# 显示链验收（自动判定显示路径并产出报告）
python scripts/verify_viewer_chain.py --seconds 30

# 离屏降级路径（无图形会话时）
MUJOCO_GL=egl python scripts/verify_viewer_chain.py --mode offscreen --frames 24

# 既有验收链回归
export MUJOCO_GL=egl
python scripts/verify_piper_pick.py --source vendor/agilex_piper/piper_description/mujoco_model/piper_description.xml --scene build/models/piper-pick-scene.xml --output build/acceptance/piper-pick --baseline config/piper_simulation_baseline.yaml --duration-ms 12000
python scripts/verify_piper_visual_pick.py --source vendor/agilex_piper/piper_description/mujoco_model/piper_description.xml --scene build/models/piper-pick-scene.xml --output build/acceptance/piper-visual-pick --baseline config/piper_simulation_baseline.yaml --duration-ms 12000
python scripts/verify_camera_depth.py --scene build/models/piper-multi-scene.xml --baseline config/piper_multi_target.yaml --target-id box_red --min-points 100 --top-band-m 0.008
python scripts/verify_piper_multi_target_pick.py --baseline config/piper_multi_target.yaml --output build/acceptance/piper-multi-target-pick --duration-ms 12500

# 单元测试
python -m pytest tests/unit/test_viewer_contract.py -q
```

## 9. 遗留项（未解决，如实记录）

1. **抓取阶段的完整六阶段时间线未纳入显示报告。**
   `pick_object` 内部阶段日志仅在 `IRAF_DEBUG_PICK=1` 时输出，显示报告的
   `phases` 目前只记录显示层编排的 `HOME` / `HOLD`。若要精确呈现
   APPROACH/DESCEND/GRIP/LIFT 的时间线，需把后端阶段日志提升为结构化事件
   并经公开接口暴露——属于契约扩展的后续工作。

2. **`pick_object` 的 `realtime=True` 路径在长时间抓取下会超出租约 TTL。**
   根因是稳定窗口按 `duration_ms * 4` 放大（上限 16 秒），叠加实时逐步等待后
   总耗时约 92 秒 > 技能声明 30 秒。显示链已通过 `realtime=False` 规避，
   但**后端时序语义本身未改动**——如需支持长时真实时仿真，应调整稳定窗口
   策略或按场景校准技能 `timeoutSeconds`，属需要评测的独立议题。

3. **倾斜目标（r/p ≠ 0）姿态对齐仍未实现。**
   夹爪尚未绕接近轴做姿态对齐，进近时张开的手指会与棱角干涉（实测推开 57mm），
   当前由 `validate_grasp_pose` 显式拦截。这是上一阶段的既有边界，本次未推进。

4. **`skills/piper` 移除后 `build/` 下的历史产物未清理。**
   旧的 `.pyc` 位于已删除目录内，已随目录一并移除；但 `build/` 下的
   `iraf-viewer*.db` 等运行产物属正常输出，未纳入版本控制。
