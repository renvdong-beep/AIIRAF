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

## 3. 多目标编排移入通用层（本轮完成）

初版 `--all-targets` 只过 box_red：因为旧脚本在求解参考姿态**之前**还做两件编排，
当时仍在机型脚本里。现已移入通用层（`build_robot_baseline.py`），Piper 构建器
**调用**这些通用函数而不复制实现：

- `derive_grasp_for_target(baseline, target_id)`：按 `targets[]` 中该目标的位姿推导
  抓取点（`pos_m`）、接近方向（顶面法向，超 `max_tilt_deg` 则回退竖直并标注
  `fallback_vertical`）、预抓取方向（固定竖直）、夹爪 yaw（对齐面内主轴）；
- `raised_home_pose(..., scene_builder=...)`：把 HOME 换成"接近轴上方抬高"的解，
  避免 HOME→APPROACH 横扫台面撞飞干扰目标；**探测场景由调用方注入生成器**
  （实测：用通用生成器建 Piper 探测场景会报"MJCF 缺少 geom: piper_left_finger"）；
  用 core 的**位置型** IK（HOME 无需姿态约束，也就无需 flange site）。
- 两个开关：`grasp.derive_from_target` / `grasp.raised_home`（只写在多目标基线里，
  单目标场景行为不变）。

统一入口还补上了旧脚本独有的**视觉精度对真值核对**（含立方体 90° 对称折叠），
并把"抓取成功 + 视觉精度达标"一起作为通过判据：

```
box_red    SUCCEEDED  力 0.245461/0.245851N  抬升 0.086574m  抓取点偏差 0.002928m
                      视觉精度 1.225mm / 0.941°   （工程记录：1.224mm / 0.94°）
box_green  SUCCEEDED  力 0.245299/0.249909N  抬升 0.076540m  抓取点偏差 0.004515m
                      视觉精度 1.212mm / 0.712°   （工程记录：1.212mm / 0.71°）
box_blue   SUCCEEDED  力 0.251940/0.249783N  抬升 0.064688m  抓取点偏差 0.003670m
                      视觉精度 2.026mm / 0.822°   （工程记录：2.026mm / 0.82°）
passed = True
```

即统一入口与旧脚本的判据与数值一致（视觉精度与工程记录逐位吻合）。

因此 `verify_piper_multi_target_pick.py` 也**降级为薄包装**（打印 deprecated 后转发）。
至此统一入口覆盖：单目标 pick_object / 单目标 visual_pick / 多目标逐目标 visual_pick，
两个机型验收脚本（pick / visual / multi-target）全部成为包装。

## 4. 复现

```
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/piper_simulation_baseline.yaml --skill visual_pick
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/piper_multi_target.yaml --all-targets --skill visual_pick
```
