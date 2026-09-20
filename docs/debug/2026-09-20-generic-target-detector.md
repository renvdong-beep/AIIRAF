# 2026-09-20 Step D1.1：视觉检测器去机型化（含 UR5e 零代码复用验证）

计划：`.hermes/plans/2026-09-20_123000-generality-roadmap.md`
关联：AGENTS.md 6.3（新机型接入不得复制核心代码）、6.5（只依赖能力，不依赖具体设备路径）。

## 1. 做了什么

检测器的**编排逻辑**原先住在 `scripts/detect_piper_targets.py` 里，虽然它依赖的全是
适配层通用原语（`depth_channel.render_rgbd/camera_pose/unproject`、
`pose_estimation.estimate_cube_pose`），但脚本名、默认输出路径、schema 字符串
都绑在 Piper 上。

- 新增 `src/iraf_adapters/mujoco/target_detection.py`：把编排逻辑（颜色分割、
  连通域、内参反投影、位姿装配、证据落盘）下沉到适配层，
  相机名与 schema 变成参数（默认 `overhead_camera` / `iraf.vision-targets/v1`）。
- 新增 `scripts/detect_targets.py`：通用 CLI（`--model/--config/--output/--extrinsics/--camera/--width/--height`）。
- `scripts/detect_piper_targets.py` 降级为薄包装（打印 deprecated 后转发，保留旧默认值）。
- 证据 schema 由 `iraf.piper-vision-targets/v1` 统一为 `iraf.vision-targets/v1`；
  后端按**结构**（是否有 targets 数组）区分单/多目标格式，不看 schema 字符串，
  因此历史证据文件仍可读。
- `config/piper_simulation_baseline.yaml` 的 `detector.command` 改指向通用检测器。

## 2. 等价性

```
通用检测器 vs 原 Piper 检测器（同一场景、同一检测配置、同一标定文件）：
  证据差异仅 1 处 —— schema_version 字符串（有意统一）
  targets[0] 视觉位置 = [0.189001, -1.4e-05, 0.025158]（逐位相同）
Piper 视觉验收 verify_piper_visual_pick.py：SUCCEEDED
  视觉位置 [0.18900127, -1.4268e-05, 0.025158242]｜抓取点偏差 0.002912920845322836
  抬升 0.087184m｜力 0.240676/0.244112N（与改动前逐位相同）
Piper 抓取验收：SUCCEEDED；单元测试 263 项，失败项仍为既有 1 + 4
```

## 3. UR5e 零代码复用验证（泛化是否成立的真正检验）

用通用检测器直接跑 UR5 场景（检测配置由后端按场景旁挂报告生成，未写任何 UR5 专用代码）：

```
MUJOCO_GL=egl python3 scripts/detect_targets.py \
  --model build/models/ur5-pick-scene.xml \
  --config build/calibration/ur5-detection-config.json \
  --output build/calibration/ur5-vision-target.json

估计位置 [-0.550372, -0.132274, 0.026086]   真值 [-0.55, -0.134, 0.025]
误差约 2.1mm（0.37 / 1.73 / 1.09 mm），掩码 448 点，位姿残差 0.34mm
```

即：UR5e 的视觉链路**不需要新增任何视觉代码**，只差"相机标定 + 配置声明 +
验收"三件事（D1.2）。这正面回答了本次泛化工作的核心问题。

## 4. 顺带修掉一个隐患：相机标定不得跨机型复用

原实现的标定默认路径是 `build/calibration/camera_to_base.json`，而该文件里是
"**Piper 的**相机相对 Piper 基座"的外参。若 UR5e 沿用它，会得到一组
"看起来合理但实际错误"的坐标 —— 这类错误不会报错，只会让抓取精度莫名变差。

处理（契约扩展，向后兼容）：

- 新增 `vision.detector.calibration_file` 声明；
- 命令占位符新增 `{calibration}`；
- **装配期校验**：命令里用了 `{calibration}` 但没声明 `calibration_file` → 显式失败；
- Piper 配置补上 `calibration_file: build/calibration/camera_to_base.json` 并把
  `--extrinsics {calibration}` 写进命令；UR5 在 D1.2 声明自己的标定文件。

## 5. 遗留（D1.2）

1. UR5e 相机标定证据（外参 + 内参）→ `build/calibration/ur5-camera-to-base.json`；
2. `config/ur5_simulation_baseline.yaml` 增加 `vision` 段；
3. 视觉验收纳入统一入口（给 `scripts/verify_pick.py` 增加 `--skill visual_pick`）；
4. 验收通过后把 `visual_pick` 加回 `profiles/ur5_mujoco.yaml`。
