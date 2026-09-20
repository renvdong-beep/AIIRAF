# vendor/unitree_go2 许可证 BOM（物料清单）

- 适用范围：本目录**已锁定**的 22 个文件（`unitree_robots/go2` 子树），逐件 SHA-256 见 `source-lock.json`
- 上游：`https://github.com/unitreerobotics/unitree_mujoco`
- 锁定 commit：`1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`（2026-09-07，`Merge pull request #131 from keeprobot/main`）
- 完整性口径：逐件 SHA-256 复算 + `git blob sha1 == git ls-tree oid` 交叉校验（抓取与校验命令见 `docs/debug/`）

## 1. 上游许可证

| 项 | 取值 | 证据 |
|---|---|---|
| SPDX | `BSD-3-Clause` | 上游 `LICENSE` 首行 `BSD 3-Clause License` |
| 版权行 | `Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")` | 上游 `LICENSE` 第 3 行 |
| `LICENSE` 文本 SHA-256 | `a5d73fc4aca9074e3e6fe0b1a0ba763cf9514b2249b7390ed20fe8d53630bf25` | 复算（1559 字节） |
| `LICENSE` git blob SHA-1 | `42d2e648c8881ea075a3bc386c91669290b2e386` | 与上游 `git ls-tree HEAD LICENSE` 逐位一致 |

上游 `readme.md` / `readme_zh.md` 未再声明其它许可证；`unitree_mujoco` 仓库主许可证为 BSD-3-Clause。

## 2. 本锁定子树（22 件，29 091 323 字节）

| 类别 | 件数 | 字节 | 文件 |
|---|---|---|---|
| MJCF 模型 | 3 | 37 655 | `go2.xml`（本体，12 个 `<motor>`、`imu` site、41 个 sensor）、`scene.xml`（平地场景）、`scene_terrain.xml`（地形场景，含 2 个 hfield） |
| 网格资产 | 16 | 28 409 145 | `assets/{base_0..4, hip_0/1, thigh_0/1, thigh_mirror_0/1, calf_0/1, calf_mirror_0/1, foot}.obj` |
| 贴图 / 高度场 | 3 | 644 523 | `height_field.png`、`unitree_hfield.png`（地形高度场）、`Go2.png`（上游文档配图，仅为溯源保留） |

以上 22 件均来自同一 commit，来源与字节均由 `source-lock.json` 记录；未混入任何第三方来源资产。

## 3. 本目录内的**非**资产文件（不参与上游许可，随本仓库许可）

| 文件 | 说明 |
|---|---|
| `source-lock.json` | 本仓库生成的锁文件（声明），非上游资产 |
| `LICENSE-BOM.md` | 本文件，非上游资产 |

## 4. 未覆盖范围（显式登记，禁止当成已审查）

- 上游 `simulate/`、`simulate_python/`、`example/ros2/`、`terrain_tool/`、`doc/` 未锁定：其中 `simulate/src/joystick/LICENSE-2.0.txt`、`simulate/src/lodepng/LICENSE` 属**第三方**许可证，本轮未纳入审查。
- 上游其它本体目录（`g1`、`h1`、`h1_2`、`h2`、`r1`、`b2`、`b2w`、`go2w`、`a2`、`as2`）未锁定；人形资产按 ADR-0007 仅作为后续静态模型候选，未进入本轮范围。
- `unitree_sdk2` / `unitree_ros2` / 固件 / Go2 EDU SKU 的许可证与二次开发授权**未收集**（ADR-0004 验证门禁 1 的其余部分仍待办）。
- 逐项资产审查（每个 `.obj` 与贴图的独立来源证明）**未完成**。

## 5. 分发门禁

**逐项审查未完成即不得对外分发资产。** 本目录资产当前只允许在**本仓库内部仿真链路**使用；
对外分发（含打包给第三方、随产品交付、公网镜像）必须先完成第 4 节全部未完成项，
并把上表的 `review_state`（`source-lock.json` 内）由 `pending_per_asset_review` 改为已审查状态。
`review_state` 是 fail-closed 字段：**未变更即视为禁止分发**。

## 6. 只读与可移植性

- 本目录资产按步骤要求设为只读（`chmod a-w`）；`source-lock.json` 与 `LICENSE-BOM.md` 保持可写，便于重新生成锁。
- **git 只保存可执行位、不保存写位**：新克隆的工作树里资产是**可写**的。因此可移植的完整性门禁是
  `scripts/verify_vendor_lock.py` 的逐件 SHA-256 复算（默认即执行）；只读位只在 `--require-readonly`
  下作为**本地状态**门禁。两者都必须通过才算"未被本地修改"。

## 7. 实测旁证（MuJoCo 3.3.3，`simulation=true`）

锁定后的最小集可直接编译，无需任何上游额外文件：

| 模型 | 加载 | nq | nv | nu | nbody | ngeom | ncam | nsensor | timestep |
|---|---|---|---|---|---|---|---|---|---|
| `go2.xml` | ok | 19 | 18 | 12 | 18 | 56 | 0 | 41 | 0.002 |
| `scene.xml` | ok | 19 | 18 | 12 | 18 | 65 | 0 | 41 | 0.002 |
| `scene_terrain.xml` | ok | 19 | 18 | 12 | 18 | 162 | 0 | 41 | 0.002 |

`ncam = 0` 再次确认：**厂商模型不含相机/雷达**，传感器必须由 IRAF 场景构建器按声明注入（步骤 13），
厂商文件保持只读、不得把传感器写进厂商 MJCF。以上数字来自本机实测，不得表述为真机能力。
