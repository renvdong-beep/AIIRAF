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
抓取和非有限位姿。真实 MuJoCo 抓取仍需配置带目标物体和夹爪接触信息的 MJCF，并在
Backend 中实现基于接触/约束状态的确认后才能启用正式 Profile。

远端固定 Conda 环境执行：

```bash
python -m unittest discover -s tests/unit -p 'test_*.py'
```

结果为 `Ran 77 tests`、`OK`。当前提交仍属于契约和安全边界完成，不是 MuJoCo
真实抓取验收完成。
