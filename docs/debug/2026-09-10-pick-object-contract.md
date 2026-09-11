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

重力搬运验收中发现 Piper 原始网格在当前姿态下的纯摩擦夹持会滑脱。为避免掩盖真实
限制，场景增加“接触后激活”的 MuJoCo connect 约束：只有先观察到 `link7`、`link8`
与目标双侧接触，才激活约束并执行抬升。验收成功的确认类型为 `constraint`，报告同时
保留 `bilateral_contact` 和 `constraint_activated`，不宣称纯摩擦抓取已经通过。
