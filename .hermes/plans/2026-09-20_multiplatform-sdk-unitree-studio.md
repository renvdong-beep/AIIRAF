# IRAF 三项扩展的架构梳理与分阶段计划（SDK 跨架构交付 / 宇树场景交互 / 语音 Studio）

- 制定时间：2026-09-20
- 目标读者：本窗口（SDK 与宇树线）与框架窗口
- 前置：本轮泛化欠债清理（`.hermes/plans/2026-09-20_123000-generality-roadmap.md`）已完成步骤 A/B/C；工作区另有 5 个未提交文件属"多目标统一入口"这一步，归另一窗口，本计划不动它们
- 配套设计文档：
  - `docs/iraf-multiplatform-sdk-design.md`（需求 1）
  - `docs/iraf-unitree-scenario-interaction-design.md`（需求 2）
  - `docs/iraf-voice-studio-design.md`（需求 3，后期）

## 0. 本轮已完成的侦察（事实，非假设）

实测与本设计相关的关键约束（详见需求 1 文档 §2）：

- 开发端 x86_64，`Python 3.10.12`，本机即 `10.203.247.145`；Conda 环境 `mujoco_graspnet` 为 `Python 3.9.21`（服务当前用它启动）。
- `docker 29.1.3`（x86_64）**无 buildx**；`registry-1.docker.io` 不可达 → 多架构 OCI 当前不可执行。
- `qemu-aarch64-static` 与 binfmt `qemu-aarch64` 已就绪；无 `aarch64-linux-gnu-gcc`。
- `pypi.org` 不可达；aliyun / 清华镜像可达；aarch64 wheel 实测可得（mujoco 492、numpy 1088、grpcio 953、protobuf 212、pyyaml 210、pillow 1224；jsonschema 为纯 Python）。
- 板卡 `10.203.247.72`、`10.203.247.86` 的 22/9119 **均不通** → aarch64 真机安装验收当前阻塞。
- `unitree_mujoco`（HEAD `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`，2026-09-07）已探测：`unitree_robots/` 含 go2/go2w/g1/h1/h1_2/h2/r1/b2/b2w/a2/as2 等 MJCF；`go2.xml` 有 12 个 `<motor>` + imu site + 力矩传感器，**无相机/雷达**；`simulate_python/unitree_sdk2py_bridge.py` 提供 MuJoCo↔SDK2 桥。
- 仓库现状：`sdk/`、`profiles/boards/`、`scenes/` 均**不存在**；`deploy/` 只有 systemd 单元与 `install-iraf-orchestrator.sh`；根级两张总架构 SVG **无 `.dot` 源**（`docs/diagrams/` 下只有两张实现图有 `.dot`），`dot` 工具链已在 `/usr/bin/dot`。

## 1. 阶段划分与依赖

```text
M1 交付骨架（需求1 L1）      M2 宇树单体（需求2 U1-U3）      M3 宇树能力与场景交互（U4-U6）
        |                              |                                |
        +-------------> M4 交接集成演示（Piper + Go2，S1/S2 交互） <------+
                                       |
                            M5 Studio 文本闭环（需求3 V1-V2）
                                       |
                            M6 语音 + 边缘 NPU（需求3 V3-V4，后期）
```

M1 与 M2 可并行（文件边界完全不重叠）：M1 只碰 `deploy/sdk/`、`config/sdk/`、`profiles/boards/`、`sdk/`；M2 只碰 `vendor/unitree_*`、`scenes/`、`src/iraf_adapters/unitree/`、`skills/{stand,stop,locomote}/`。

## 2. M1：交付骨架（需求 1，L1 路线）

| 步 | 内容 | 涉及文件 | 验收（必须可复跑） |
|---|---|---|---|
| M1.1 | 产物矩阵与 BoardProfile 契约：schema + 两个板卡的 `unverified` 声明 | 新增 `config/sdk/package_matrix.yaml`、`profiles/boards/e300.yaml`、`profiles/boards/firefly_rk3588.yaml`、`docs/adr/0006-sdk-cross-arch-delivery.md` | `profile_check` 能读 BoardProfile 并对 `unverified` 显式拒绝 |
| M1.2 | SDK 层最小实现：错误码→中文诊断、客户端封装（复用既有 gRPC/HTTP 公共契约，无旁路） | 新增 `src/iraf_sdk/`、`tests/unit/test_sdk_errors.py` | 单测通过；拒绝路径有用例 |
| M1.3 | 打包脚本：`build_sdk.sh` → wheel + runtime bundle + `manifest.json` + 校验和 | 新增 `deploy/sdk/build_sdk.sh` | 退出码 0；manifest 内 SHA-256 全部复算一致 |
| M1.4 | 离线 wheelhouse：`fetch_wheelhouse.sh`（按声明标签抓取） | 新增 `deploy/sdk/fetch_wheelhouse.sh` | 抓取物平台标签全部匹配；遇到 `win_arm64` 等错标签必须失败 |
| M1.5 | 板级 bundle 组装 + 目标端脚本 | 新增 `deploy/sdk/package_board_bundle.sh`、`deploy/sdk/install.sh`、`deploy/sdk/verify.sh`、`deploy/sdk/uninstall.sh` | `unverified` 未加 `--allow-unverified` → 退出码 2；篡改文件 → 退出码 4 |
| M1.6 | 传输与部署：`deploy.sh`（ssh 与 media 双通道，含 `--dry-run`） | 新增 `deploy/sdk/deploy.sh`、`docs/debug/2026-09-20-sdk-cross-arch.md` | 本机 `--dry-run` 全绿；ssh 通道标注"待板卡可达" |
| M1.7 | 文档与图：更新 `docs/iraf-engineering-design.md` §3/§4 的产物与脚本入口；新增 `docs/diagrams/iraf-sdk-delivery.dot` → `.svg` | 改 `docs/iraf-engineering-design.md`、新增 dot/svg | 图文一致（`AGENTS.md` 6.6） |

M1 明确不做：OCI 多架构镜像、签名服务（只留占位）、目标端编译。

## 3. M2：宇树单体与场景包（需求 2，U1-U3）

| 步 | 内容 | 涉及文件 | 验收 |
|---|---|---|---|
| M2.1 | vendor 锁定：`unitree_mujoco` / `unitree_sdk2_python` 指定 commit + 许可证 BOM | 新增 `vendor/unitree_mujoco/`（只读）、`vendor/unitree_go2/source-lock.json` | `source-lock.json` 记录 commit/哈希/许可证；模型文件哈希可复算 |
| M2.2 | 场景包契约与 schema：`scenes/<id>/scene.yaml` + `scenario.yaml` | 新增 `scenes/handoff_lab/`、`docs/adr/0007-unitree-scenario-interaction.md` | schema 校验 + 缺字段显式失败 |
| M2.3 | 场景构建器：厂商 MJCF 只读，注入 camera/lidar/托盘/地形/光照（声明式） | 新增 `scripts/build_scene.py`（按 `scene.yaml` 分派）、`profiles/unitree_go2_mujoco.yaml` | 生成 MJCF 可加载；传感器实测统计（雷达点数、相机内参 vs 模型 fovy 相对误差）落证据 |
| M2.4 | Go2 loopback：MuJoCo 步进 + 站立/停止/状态读取 smoke（不接 IRAF） | 新增 `scripts/verify_go2_loopback.py` | 躯干高度/姿态/速度曲线 + 稳定时长数字；无 GUI 可复现 |
| M2.5 | `QuadrupedAdapter` 契约 + `UnitreeGo2Adapter`，接入 `profile_check` 与 factory | 新增 `src/iraf_adapters/unitree/`、`tests/unit/test_quadruped_adapter.py` | declared ⊆ implemented；未知厂家字段不得穿透 |

## 4. M3：宇树能力与场景交互（U4-U6）

| 步 | 内容 | 验收 |
|---|---|---|
| M3.1 | Skill：`stand` / `stop` / `locomote`（走 TaskFlow→SkillRuntime→Policy），含拒绝路径 | 成功 + 拒绝（越界速度、过期状态、无权限）各一份证据 |
| M3.2 | S2 脚本化场景：`scripts/scenario.py run --scene … --scenario nominal` | 场景步骤自动执行，出 `build/acceptance/<scene>/report.json` |
| M3.3 | `navigate` / `dock_for_handoff` / `accept_payload` + 互锁负向测试 | 停靠误差 ≤ 30 mm、偏航 ≤ 2 deg、速度为零；互锁拒绝率 100% |
| M3.4 | S1 命令式交互（复用 `view_mujoco.py` 管线） | 交互命令 → Skill → viewer 可见状态变化；拒绝路径同样可见 |

## 5. M4/M5/M6

- M4 交接集成：`delivery_handoff` 端到端（Piper + Go2），复用 `docs/iraf-piper-quadruped-simulation.md` 的指标表做判据。
- M5 Studio 文本闭环（需求 3 V1-V2）：`iraf studio` CLI → AgentOSBridge → 执行 → 回执；语音暂不引入。
- M6 语音 + 边缘 NPU（V3-V4）：需先有板卡可达与 `npu: verified`，否则只能标"未验证"。

## 6. 每步共同纪律（沿用既有约定）

1. 先快照证据 → 再改 → 逐位比对 → 门禁/验收/单测 → 中文调试记录 → 单独提交。
2. 提交只 `add` 自己负责的路径；不执行 `git stash/checkout/reset/add -A`。当前工作区有另一窗口的 5 个未提交文件，禁止触碰。
3. 不放宽任何既有门禁来"让测试通过"；阈值只来自声明与实测。
4. 新增可复跑能力必须落成脚本/CLI，不写一次性手工手册（`AGENTS.md` 5.2）。
5. 架构边界变化必须在同一变更更新架构图与 ADR（`AGENTS.md` 6.6）；根级两张总 SVG 无 `.dot` 源，属已知欠债，先出 Mermaid/Graphviz 源再导出。
6. 仿真结果一律带 `simulation=true`；未在板卡验收的能力不得表述为"已在边缘运行"。

## 7. 需要用户确认的决策点（原始选项保留以便回溯）

**已确认（2026-09-20，1.A / 2.A更新 / 3.A / 4.B / 5.A / 6.A / 7.A）**：见三份设计文档的「决策记录」小节。

1. SDK 形态：仅 Python（建议）/ +C++ 头文件（本轮不做二进制）/ +只读 gRPC 封装。
2. wheelhouse 来源：aliyun 镜像（建议）/ 内网私有源 / 板厂清单。
3. 目标端 Python 版本由谁确认：先写 `unverified`（建议）/ 提供板卡实测信息 / 等板卡可达再定。
4. 宇树首期本体范围：只 Go2（铁律 6.8）/ Go2 + 人形静态模型（建议）/ Go2 + 人形运动（需 ADR）。
5. 场景包位置：`scenes/`（建议）/ 复用 `examples/`。
6. 交互形态优先级：先 S2 脚本化（建议）/ 先 S1 命令式。
7. 语音定位：仅输入法（建议）/ 允许只读查询 / 允许低风险动作。

## 8. 本计划的执行落地位置

- 计划文件夹：`plans/iraf-24h/`（**已入库**，随引导提交 `5df5d05`）；机器台账 `plans/iraf-24h/00-STATUS.json` 是唯一权威，人工摘要 `plans/iraf-24h/00-日志.md`。
- 执行规则：`plans/iraf-24h/00-执行规则.md`（含 §0 范围调整 x86-first、硬规则、无人值守注意事项、阻塞清单）。
- 本战役范围：只做开发端 x86_64 可取证部分（SDK L1、脚本、场景包与 MuJoCo、Go2 loopback、适配器与技能、图文档）；目标端验收统一 `DEFERRED`（板卡不在），禁用 dry-run/mock 冒充。
- 文件边界（本窗口 vs 另一窗口）：见 `docs/iraf-multiplatform-sdk-design.md` §13。

