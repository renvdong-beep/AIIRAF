# 2026-09-20 视觉入口声明式化：后端不再含任何机型路径

关联：AGENTS.md 铁律 6.2（禁止隐式默认值与环境变量）、6.5（只依赖能力，不依赖
具体模型名/设备路径）、5.3（参数集中在版本化配置）。上一步同类治理见
`docs/debug/2026-09-20-backend-implicit-defaults-removal.md`。

## 1. 问题

`MujocoBackend.visual_pick` 把 Piper 的部署细节写死在适配器里：

| 位置 | 原实现 |
| --- | --- |
| 证据文件 | 缺省 `build/calibration/piper-vision-target.json` |
| 检测器入口 | 写死 `scripts/detect_piper_target.py` / `detect_piper_targets.py`，并按"多目标优先、否则单目标"自行兜底选择 |
| 参数真源 | 写死 `config/piper_multi_target.yaml`（且"文件存在就复用、解析失败就静默忽略"） |
| 刷新开关 | 隐式环境变量 `IRAF_REFRESH_VISION`，缺省 `1` |

后果与上一步同类：换机器人后要么静默复用了别机型的检测器，要么报错指向一个
与调用方无关的路径；而"改了参数却没生效"（静默忽略解析失败）无法察觉。

## 2. 契约：`vision` 段（全部可选）

后端配置（`MujocoBackend.from_config` 的入参，与 `model_path` / `manipulation` 同级）：

```yaml
vision:
  evidence_file: build/calibration/piper-vision-target.json   # 证据文件（目标 world 位姿）
  refresh: always | on_missing | never                        # 默认 always
  detector:
    command: ["{python}", "scripts/detect_piper_targets.py",
              "--model", "{model}", "--config", "{config}", "--output", "{evidence}"]
    config_file: config/piper_multi_target.yaml               # 可选：参数真源（其 depth 段覆盖内置默认值）
    config_output: build/calibration/detection-config.json     # 可选：生成物落盘位置
```

- 占位符：`{python}`（当前解释器）、`{model}`、`{evidence}`、`{config}`、`{target_id}`。
  **未知占位符在装配期即失败**（`{modle}` 这类拼写错误若被容忍，检测器会带着默认参数
  跑出一份"看起来成功"的错误证据）。
- `refresh` 语义：`always` 每次抓取前刷新；`on_missing` 仅证据缺失时刷新；
  `never` 只读现有证据。环境变量 `IRAF_REFRESH_VISION`（0/1）保留为**调试用显式覆盖**，
  且只接受 0/1，其他值报错。
- **未声明 `vision` 时**：不刷新，且 `visual_pick` 必须由请求显式给出 `vision_file`，
  否则显式失败：
  `未提供视觉证据路径：请在请求参数里给 vision_file，或在后端配置的 vision.evidence_file 中声明（后端不提供机型默认路径）`。
- 证据文件仍是"场景事实"，检测器输入配置中的 `targets`/颜色由场景旁挂报告
  （`<scene>.json`）生成，参数真源才由 `config_file` 声明 —— 单目标与多目标共用一条链路。

## 3. 产者侧（声明怎么流到后端）

```
config/*.yaml 的 vision 段
  → scripts/build_*_pick_scene.py 写入场景 report 的 scene["vision"]
    → 各调用脚本透传 "vision": scene.get("vision")
      → MujocoBackend.from_config → _parse_vision_config
```

已同步：`config/piper_simulation_baseline.yaml`、`config/piper_multi_target.yaml`
（新增 vision 段）、两个场景生成器、以及四个 `visual_pick` 调用方
（`verify_piper_visual_pick.py`、`verify_piper_multi_target_pick.py`、
`run_piper_random_pick.py`、`run_piper_random_view.py`）。

## 4. 验证

```
Piper 视觉验收 verify_piper_visual_pick.py  SUCCEEDED
  证据 refreshed=True  refresh_mode=always  source=pose_estimation
  schema=iraf.piper-vision-targets/v1  evidence_file=<配置声明的路径>
  双指接触 0.2407 / 0.2441 N   抬升 0.087184 m（与改动前一致）
Piper 抓取验收 verify_piper_pick.py        SUCCEEDED（0.002826 m，finger_pair 口径）
UR5e  抓取验收 verify_ur5_pick.py          SUCCEEDED（0.000119 m，grip_region 口径）
UR5e  视觉路径未声明时：显式失败（实测错误文案见第 2 节），未伪造成功
单元测试 244 项（原 231 + 新增 13 项视觉契约回归），失败项仍为既有 1 + 4
```

## 5. 遗留与决策点

1. **能力契约是双向的**：`factory.verify_backend_contract` 要求"后端实现了的能力
   必须在 profile 声明"，否则装配期报
   `Backend 实现了未声明的运动能力…: visual_pick`。
   因此本步**没有**从 `profiles/ur5_mujoco.yaml` 去掉 `visual_pick`：
   后端类同时服务多台机器人、各自能力子集不同，这条双向约束会挡住"UR5 无视觉"的
   诚实声明。建议框架侧改为单向（declared ⊆ implemented），并新增
   "声明 visual_pick ⇒ 必须声明 vision 来源"的装配期校验；
   等这项框架决策落地后，再从 UR5 profile 移除该能力。
   现状：对 UR5e 提交 visual_pick 会显式失败（不会伪造成功），但失败点在运行期而非策略层。
2. `scripts/view_piper_mujoco.py`、`scripts/verify_viewer_chain.py` 未透传 `vision`
   （它们不调用 `visual_pick`）；将来若在这些入口启用视觉抓取，需要一并透传。
3. UR5e 侧仍无相机标定与视觉抓取证据，属已知缺口（见
   `docs/debug/2026-09-20-ur5-tool-axis-flip.md` 与 profile 注释）。
