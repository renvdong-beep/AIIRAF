# demo3：第三台机型接入示例（干跑）

一台**合成**的 4 自由度臂（q1 偏航 + q2/q3/q4 俯仰）+ 平行双指滑轨夹爪，
用于验证 IRAF 的接入路径是否真的"只填声明"，以及门禁会不会拒绝不自洽的声明。

刻意采用与既有两台机型完全不同的命名（`q1..q4` / `pad_l,pad_r` / `pinch_l,pinch_r`），
这样任何"名字恰好相同所以能跑通"的耦合都会暴露。

## 文件

| 文件 | 作用 |
| --- | --- |
| `demo3.xml` | 合成 MJCF：4 自由度臂 + 双指滑轨夹爪 + flange site + home keyframe |
| `demo3_profile.yaml` | RobotProfile：关节、限位、角色、夹爪驱动方式、home、相机 |
| `demo3_baseline.yaml` | 基线：模型来源、臂关节、指腹 geom、夹爪位形、抓取/验收参数、光源、build 段 |

## 当前状态

```
scripts/profile_check.py --baseline examples/demo3_arm/demo3_baseline.yaml
  → PROFILE_CHECK_PASSED（零代码即可通过）

scripts/build_baseline.py --baseline examples/demo3_arm/demo3_baseline.yaml
  → 在位姿/一致性门禁处显式失败：
    夹持区中心与抓取目标不一致 |measured - target| ≈ 8~17mm
```

原因（详见 `docs/debug/2026-09-20-third-robot-dry-run.md`）：该臂是平面构型，
"工具必须朝下/水平 + 目标在台面高度"这一组约束要求的目标点不在它的可达集内，
求解器在不可达时会发散到关节限位，门禁据此拒绝 —— **这是预期行为**，
它保证不会产出"看起来收敛"的坏场景。

## 想让它跑通

1. 用实测扫描确定可达集（示例：扫 q2/q3 网格，筛"工具指向满足要求 + 夹持区高度合适"的
   (x,z) 点集），把 `grasp.finger_center_xy_m` 改到集合内的点；
2. 或把模型改成工作空间覆盖台面的构型（加长连杆 / 调整腕部偏置），
   使声明与运动学自洽。

## 这个示例的价值

- 证明接入不需要写 py：profile / baseline / 模型三份声明即可推进到运动学求解；
- 演示门禁的拒绝行为：不可达 / 不自洽的声明会被显式拒绝，并给出可定位的中文原因；
- 回归安全网：由它暴露的"法兰局部轴假设"修掉后，UR5e 的参考姿态逐位不变。
