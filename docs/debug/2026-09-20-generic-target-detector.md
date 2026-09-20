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

## 6. Step D1.2：UR5e 视觉接入完成（2026-09-20 续）

D1.1 之后 UR5e 已能"零代码"跑检测器，D1.2 补上标定、声明与验收三件事。

### 6.1 相机标定脚本去机型化

`scripts/verify_camera_calibration.py` 的算法本就通用，但身份绑死在 Piper 上：

| 原实现 | 改为 |
| --- | --- |
| 写死 body `box_01` / geom `box_01_geom` | `--target-id`（缺省 `box_01`） |
| 写死 `profiles/piper_mujoco.yaml` | `--profile`，缺省读基线 `build.profile`（不再隐式假设机型） |
| 写死 resource `piper-mujoco` | `profile.name + "-mujoco"` |
| schema `iraf.piper-camera-extrinsics/v1` | `iraf.camera-extrinsics/v1` |

**并修掉一条写死的机型假设门禁**：原判据 `200 < focal_px < 400` 是 Piper 相机
（292px）留下的区间，UR5e 的 404.8px 被判 `CALIBRATION_FAILED`，而几何上完全正确
（由 fovy=60° 推出的理论焦距 415.7px，相对偏差仅 2.6%）。
改为**以模型自带 fovy 推出的理论焦距为基准**的判据：

```
focal_expected = (height/2) / tan(fovy/2)
门禁：相对偏差 ≤ --focal-tolerance(0.15) 且 像素残差 RMS ≤ --residual-px(2.0)
```

两台机型实测均通过：

```
UR5e   focal 404.83 vs 理论 415.7（偏差 2.6%）  残差 RMS 1.29px  外参 max_error 2.2e-15 m
Piper  focal 292.11 vs 理论 296.4（偏差 1.4%）  残差 RMS 1.08px  外参 max_error 1e-15 m
```

### 6.2 UR5e 视觉声明与验收

`config/ur5_simulation_baseline.yaml` 新增 `vision` 段（与 Piper 同结构，
检测器同一个通用脚本，只是 `calibration_file` 指向 UR5e 自己的标定）：

```yaml
vision:
  evidence_file: build/calibration/ur5-vision-target.json
  refresh: always
  detector:
    command: ["{python}", "scripts/detect_targets.py", "--model", "{model}",
              "--config", "{config}", "--output", "{evidence}",
              "--extrinsics", "{calibration}"]
    calibration_file: build/calibration/ur5-camera-to-base.json
```

统一入口新增 `--skill visual_pick`（`scripts/verify_pick.py`），
四判据与 `pick_object` 完全一致，只是位姿来自视觉证据而不是场景真值。

### 6.3 结果

```
检测精度（UR5e，用自己的标定，独立于验收链路）：
  视觉位置 [-0.551008, -0.131610, 0.025408]   真值 [-0.55, -0.134, 0.025]
  误差 2.63mm（掩码 448 点，位姿残差 0.34mm）
  对照：用模型自带内参时为 2.10mm —— 拟合内参残差 1.3px 带来约 0.5mm 额外偏差，
  两者都远小于 5mm 容差。若要提高精度可增加采样或直接采用模型内参。

四条验收全部 SUCCEEDED：
  UR5e  visual_pick  SUCCEEDED  视觉位置误差 2.6mm  双指 17.451911/16.053349N
                                抬升 0.041208m  抓取点偏差 0.000118763（与真值路径一致）
  UR5e  pick_object  SUCCEEDED  同上数值
  Piper pick_object  SUCCEEDED  0.240676/0.244112N  抬升 0.087184m  偏差 0.002826043
  Piper visual_pick  SUCCEEDED  偏差 0.002912921（视觉链路）
单元测试 263 项，失败项仍为既有 1 项 + 4 项导入错误。
```

`profiles/ur5_mujoco.yaml` 的 `visual_pick` 能力**已加回**：相机、独立标定、
通用检测器、config vision 段与视觉验收证据齐备，不再是无凭据的声明。

### 6.4 复现命令

```
PYTHONPATH=src python3 scripts/build_baseline.py --baseline config/ur5_simulation_baseline.yaml
MUJOCO_GL=egl PYTHONPATH=src python3 scripts/verify_camera_calibration.py \
    --scene build/models/ur5-pick-scene.xml \
    --baseline config/ur5_simulation_baseline.yaml \
    --extrinsics-output build/calibration/ur5-camera-to-base.json \
    --output build/acceptance/ur5-camera-calibration
PYTHONPATH=src python3 scripts/verify_pick.py --baseline config/ur5_simulation_baseline.yaml \
    --skill visual_pick --output build/acceptance/ur5-visual-pick
```
