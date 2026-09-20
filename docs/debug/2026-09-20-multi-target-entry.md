# 2026-09-20 统一入口扩展：多目标 + 视觉包装脚本

## 1. 改了什么

`scripts/verify_pick.py` 现在覆盖三种用法：

```
--baseline <cfg>                          # 单目标 pick_object
--baseline <cfg> --skill visual_pick      # 视觉链路（同一套四判据）
--baseline <cfg> --all-targets --skill visual_pick   # 多目标：逐个重建场景并各自执行
```

- 新增 `--all-targets`：按基线/场景声明的 `targets[]` 逐个**重建场景**（搬运约束与
  参考姿态是按目标生成的）并各自执行，报告里每个目标一份执行证据，
  `passed = 所有目标 SUCCEEDED`。
- 两个构建器的 `build()` 新增 `target_id` 参数（"按目标重建"成为一等能力）；
  Piper 侧原先用配置顶层的 `target.id` 覆盖调用方传入的 id，已改为"参数优先"。
- `scripts/verify_piper_visual_pick.py` 降级为**薄包装**（打印 deprecated 后转发，
  保留旧默认值与"给 --baseline 即重建"的语义）。

## 2. 验证

单目标四项验收**数值逐位不变**（重构未触碰动作链路）：

```
UR5e  pick_object  SUCCEEDED  17.451911/16.053349N  抬升 0.041208m  偏差 0.00011876256957761978
UR5e  visual_pick  SUCCEEDED  同上（视觉位姿）
Piper pick_object  SUCCEEDED  0.240676/0.244112N   抬升 0.087184m  偏差 0.002826042685491991
Piper visual_pick  SUCCEEDED  0.002912920845322836（走统一入口 / 旧包装两条路径一致）
```

多目标（`config/piper_multi_target.yaml`，3 个目标）：

```
box_red    SUCCEEDED  命中 box_red  力 0.244495/0.243678N  抬升 0.08734m  偏差 0.0029280527598886984
box_green  FAILED
box_blue   FAILED
```

## 3. 多目标为什么只过了一个（诚实记录）

旧的多目标验收脚本（`verify_piper_multi_target_pick.py`）在调用
`build_reference_poses` **之前**还会做两件编排：

1. **按目标位姿推导抓取参数**：从 `targets[]` 里取该目标的位置与顶面法向，
   覆写 `grasp.finger_center_xy_m` / `approach_direction` / `pregrasp_direction` / `yaw_deg`；
2. **生成抬高的 home 位姿**（`_raised_home_pose`），避免 HOME 阶段扫过其它目标。

这两段编排仍在机型脚本里，没有进通用层。因此统一入口的 `--all-targets`
目前只适用于"目标位姿已在配置中显式声明、且抓取参数对每个目标都成立"的场景 ——
`box_red` 恰好满足（它是顶层 `target` 声明的那一个），绿色/蓝色不满足。

**结论**：`verify_piper_multi_target_pick.py` **暂不删除**，它仍承担"未知位姿自适应
多目标"验收；统一入口的 `--all-targets` 先作为"声明式多目标"入口，两者语义不同，
文档里分别标注，避免用弱验收替换强验收。

下一步（把编排移入通用层，使其真正统一）：
- 在 `build_robot_baseline.build_reference_poses` 中支持"按 target 条目推导抓取参数"
  （配置开关，例如 `grasp.derive_from_target: true`，默认关闭保持既有行为）；
- 把 `_raised_home_pose` 一并移入通用层；
- 判据：box_red/green/blue 三个目标的命中、双指接触、抬升与误差全部复现旧脚本的数值。

## 4. 复现

```
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/piper_simulation_baseline.yaml --skill visual_pick
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/piper_multi_target.yaml --all-targets --skill visual_pick
```
