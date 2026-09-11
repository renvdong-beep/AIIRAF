# `pick_object` 第一批契约实现记录

## 目标

在不伪造 MuJoCo 或真机抓取成功的前提下，让 IRAF 从 `move_joint` 推进到
`pick_object`。本批只建立公共输入输出契约、Provider 确认边界和 Runtime
拒绝路径，不把当前正式 Piper Profile 标记为已具备抓取能力。

## 实现

- 新增 `skills/pick_object/`，输入包含目标 ID、带坐标系的抓取位姿和有界时长。
- 输出只有在 Backend 返回目标一致、`grasped=true` 且确认类型为 `contact`、
  `constraint` 或 `gripper_state` 时才允许成功。
- 新增平台无关 `PickObjectProvider`；Backend 未实现抓取、目标不匹配、未确认抓取、
  确认类型不受信或位姿非有限/四元数未归一化时全部 fail-closed。
- Skill 前置条件要求急停未触发且目标当前可见。
- 扩展公共 Backend Protocol，但暂不修改 `profiles/piper_mujoco.yaml` 和正式安全策略；
  当前 Runtime 不会误报抓取能力。

## 验证

新增单元测试覆盖：受信仿真确认成功、目标不可见、Profile 能力缺失、Backend 未确认
抓取和非有限位姿。

远端固定 Conda 环境执行：

```bash
python -m unittest discover -s tests/unit -p 'test_*.py'
```

结果为 `Ran 77 tests`、`OK`。当前提交仍属于契约和安全边界完成，不是 MuJoCo
真实抓取验收完成。

## 第二批：Piper 双指接触

运行服务实际使用的 AgileX Piper MJCF 来自 `piper_ros`，模型内已包含双指夹爪：
`joint7` 行程为 `0..0.035m`，`joint8` 行程为 `-0.035..0m`，并分别配置位置执行器。
此前 Profile 把两个关节都配置为 `0..0.08m`，已按 MJCF 修正。

新增 `scripts/build_piper_pick_scene.py`，从只读 Piper MJCF 自动生成开发抓取场景，
把 mesh 路径解析为绝对路径，并在 `build/` 中加入 `box_01` 立方体。立方体是本项目
直接定义的 MuJoCo 基础几何体，不复制来源不明的第三方网格。当前场景关闭重力，只用于
验证目标定位、夹爪开合和双指接触链路；在完成带重力升举测试前，不声称具备稳定搬运能力。

MuJoCo Backend 新增声明式 `manipulation.targets` 和 `manipulation.gripper` 配置。
`pick_object` 仅在目标存在、请求位姿与目标一致且 `link7`、`link8` 都与目标产生真实
MuJoCo contact 时返回成功。单侧接触、目标未知、坐标系错误或位姿偏差超限均失败。

完整 Runtime 验收入口：

```bash
PYTHONPATH=src python scripts/verify_piper_pick.py \
  --source /path/to/piper_description.xml
```

场景、场景清单和执行报告分别写入 `build/models/` 与
`build/acceptance/piper-pick/`。

## Ubuntu Viewer

Ubuntu 主机存在 XFCE/Xorg 图形会话（`DISPLAY=:0`）。MuJoCo Viewer 首次启动失败的
根因是 Conda `libstdc++.so.6` 缺少系统 Mesa LLVM 所需的 `GLIBCXX_3.4.30`。使用
系统 C++ ABI 和 Mesa DRI 后，Viewer 已实测返回 `VIEWER_OK`：

```bash
DISPLAY=:0 \
XAUTHORITY=/run/user/1000/gdm/Xauthority \
LIBGL_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri \
MESA_LOADER_DRIVER_OVERRIDE=iris \
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
/home/coretek/miniconda3/envs/mujoco_graspnet/bin/python \
  scripts/view_piper_mujoco.py --seconds 0
```

脚本只负责 Viewer 和物理画面显示；任务动作仍必须经 IRAF Runtime、Skill、Policy
和 Backend 执行，不能把 Viewer 作为旁路控制入口。

重力搬运验收中发现 Piper 原始网格在当前姿态下的纯摩擦夹持会滑脱。为避免掩盖真实
限制，场景增加“接触后激活”的 MuJoCo connect 约束：只有先观察到 `link7`、`link8`
与目标双侧接触，才激活约束并执行抬升。验收成功的确认类型为 `constraint`，报告同时
保留 `bilateral_contact` 和 `constraint_activated`，不宣称纯摩擦抓取已经通过。

## 13. 接触力闭环与掉块根因

此前的判定只检查 `link7`、`link8` 是否产生 contact，然后直接激活搬运约束；这能证明
接触链路存在，但不能证明夹爪已经形成可承载的夹持力，因此会出现“判定成功、抬升后掉块”。

现在 Backend 使用 MuJoCo `mj_contactForce` 读取每个接触的六维接触力，取接触坐标系的
法向分量 `force[0]`，分别取左右手指接触法向力的最大值：

```text
F_left  = max(contactForce(link7, box)[0])
F_right = max(contactForce(link8, box)[0])
imbalance = max(F_left, F_right) / max(min(F_left, F_right), 1e-9)
```

抓取通过条件为：双侧 contact、左右法向力都不低于 `min_normal_force_n`（默认 0.2N）、
`imbalance` 不超过 `max_force_imbalance_ratio`（默认 4.0）。只有通过该力门禁后才允许
激活 `lift_constraint`；否则 fail-closed，报告 `force_ok=false` 及实测力值。

原默认目标半边长 0.018m 时，实测左指约 2.883N、右指约 0.580N，受力比约 4.97:1，
因而正确判定为不可承载抓取。经过尺寸扫描，半边长 0.030m 的目标能让指腹形成稳定
双侧接触，实测约 12.045N/12.134N、受力比 1.007:1，并完成 0.182m 抬升。开发验收
场景默认改为 0.030m，同时保留 `--target-half-size` 供小目标 IK/指尖几何调参；不能
通过放宽力门禁来掩盖小目标的掉块问题。
