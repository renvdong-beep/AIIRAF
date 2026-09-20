# IRAF 工程化设计

**状态**：实施基线（v0.1）  
**范围**：Ubuntu、CentOS、openEuler 开发/仿真；E300、S600、FIREFLY 边缘部署  
**依据**：`agentos-robot-application-framework-design.md`

**详细设计入口**：

- [总体架构图](../agentos机器人应用框架-更新版.svg)
- [IRAF 详细技术架构图](../IRAF详细技术架构图.svg)
- [SDK 跨架构交付图](diagrams/iraf-sdk-delivery.svg)（源文件 `diagrams/iraf-sdk-delivery.dot`）
- [宇树场景栈图](diagrams/iraf-unitree-scene-stack.svg)（源文件 `diagrams/iraf-unitree-scene-stack.dot`）
- [IDL、Registry 与 Runtime](iraf-idl-runtime-detailed-design.md)
- [TaskFlow 与 Policy Gateway](iraf-taskflow-policy-detailed-design.md)
- [边缘适配、数据闭环与 AgentOS 接入](iraf-edge-data-detailed-design.md)
- [Piper 与机器狗协同仿真](iraf-piper-quadruped-simulation.md)
- [Piper 多模式控制与 MuJoCo](iraf-piper-multimode-control.md)
- [竞品与开源能力借鉴分析](iraf-competitive-landscape.md)
- [ADR-0004：宇树 Go2 EDU](adr/0004-unitree-go2-edu.md)
- [AgentOS / Hyper 构型兼容设计](iraf-agentos-hyper-compatibility.md)
- [ADR-0005：AgentOS / Hyper 兼容](adr/0005-agentos-hyper-compatibility.md)

## 1. 工程目标与非目标

IRAF 为 AgentOS 提供机器人领域执行面：应用提交任务，TaskFlow 编排 Skill，Skill Runtime 在策略和本体约束内路由能力提供者，并将结果、事件和可追溯数据返回。

不在 IRAF 内实现 Hypervisor、内核/BSP、硬实时主站或驱动器。它们通过受控 Capability Provider 接入。首期的成功标准是同一份 TaskFlow、Skill 合约和 Robot Profile 语义可在 Linux 仿真和边缘目标上运行，而不是让每块板运行相同的二进制或强行具备同一算力。

兼容方向必须明确：IRAF 通过 `AgentOSBridge` 北向承接 AgentOS 的任务、身份委托、生命周期、状态与事件；通过 `HyperProfile` 南向适配 `MQ50-E300-3VM-LPR`、`MQ50-E300-3VM-LPP`、`FIREFLY-RK3588-2VM-LR` 三个当前正式发布构型。VM 数量、OS 类型和角色映射由签名发布元数据决定，不在代码中硬编码。S600 尚无对应正式构型，保持 `unverified`。

## 2. 可实施架构

```text
AgentOS / Application
          | protobuf/gRPC: TaskRequest, SkillGoal, Feedback
     IRAF Control Plane
 TaskFlow | Registry | Runtime | Policy Gateway | Observability
          | validated capability goal / safety event
     IRAF Edge Plane
 Capability Router -> ROS 2 adapters -> Nav2/MoveIt/Skill providers
                   -> local real-time gateway -> master/driver
```

控制面运行在通用 Linux 进程/容器中，采用 Protobuf + gRPC/HTTP 作为管理与开发接口；边缘 ROS 2 适配采用 DDS。运控到通信主站只允许本机的固定格式、时间戳、序列号和超时保护通道，不能通过 gRPC、AgentOS 或模型服务形成实时闭环。

| 模块 | 责任 | 首选实现 | 禁止事项 |
|---|---|---|---|
| `iraf-idl` | 公共消息、状态与错误码 | Protobuf、buf/protoc | 以 ROS topic 作为唯一公共契约 |
| `taskflow` | 状态机/行为树、取消、重试、接管 | C++20 核心，Python SDK | 直接调设备 |
| `skill-runtime` | 验证、路由、生命周期 | C++20 | 以模型输出代替策略检查 |
| `policy-gateway` | schema/RBAC/范围/碰撞与安全策略 | C++20，规则配置 | 调用方提升安全边界 |
| `capability-adapters` | ROS 2、仿真、主站受控适配 | C++20，ROS 2 | 把板型判断泄漏给业务层 |
| `model-provider` | local/remote/cloud/replay 推理 | gRPC client | 进入 L0/L1 闭环 |
| `control-authority` | traditional/VLA/learned/teleop 单写者仲裁 | C++20 + lease/fencing | 多 Provider 同时写控制通道 |
| `observability` | 追踪、事件、回放索引、审计 | OpenTelemetry-compatible | 记录密钥或未脱敏数据 |

## 3. 目标仓库和产物

```text
api/proto/                 # 可版本化的 canonical IDL
core/{taskflow,runtime,policy,registry,world}/
sdk/{cpp,python}/
config/sdk/                 # SDK 产物矩阵与板级交付的集中声明（唯一事实来源）
adapters/{ros2,sim,hil,master,model}/
skills/{navigate,pick_object,place_object,stop,recover}/
profiles/{robots,boards,safety}/
deploy/{compose,helm,systemd}/
deploy/sdk/                 # 跨架构 SDK 构建、wheelhouse 抓取、板级 bundle 组装与部署脚本
tests/{unit,contract,simulation,hil,hardware}/
tools/{profile-check,release-evidence}/
docs/
```

`config/sdk/`、`deploy/sdk/` 与 `profiles/boards/` 是 M1.7 之后新增的交付边界：前者是唯一声明来源，中者是入口层脚本 + 仅用标准库的实现层，后者是 `BoardProfile`（部署能力，不含业务语义）。三者与 ADR-0006 的分级交付对应，当前只承诺 L1（纯 Python SDK + 离线 wheelhouse + shell 脚本）。

| 已交付路径 | 已实现入口 | 当前状态（2026-09-20） |
|---|---|---|
| `config/sdk/package_matrix.yaml` + `package_matrix.schema.json` | 被 `build_sdk.sh`/`fetch_wheelhouse.sh`/`package_board_bundle.sh` 读取 | 已落地；`index_url` 指向可达镜像，可声明替换 |
| `profiles/boards/{e300,firefly_rk3588}.yaml` | `scripts/profile_check.py --board <id>` | 已落地；未实测字段一律 `unverified`，`--board` 默认 exit=2 |
| `deploy/sdk/build_sdk.sh` | `--dry-run` / `--verify` / `--allow-unverified` | 已落地；产物可复算（连续两次构建 SHA-256 逐位相同） |
| `deploy/sdk/fetch_wheelhouse.sh` | 标准库直读 PEP 503 索引 | 已落地；`pip download` 在无人值守会话被审批门拦截，故实现偏差已记入调试记录 |
| `deploy/sdk/package_board_bundle.sh` / `install.sh` / `verify.sh` / `uninstall.sh` | 组装 → 安装 → 自检 → 受控卸载 | 已落地；目标端真实安装 **DEFERRED**（板卡不在场） |
| `deploy/sdk/deploy.sh` | `--transport media` / `--transport ssh` | 已落地；ssh 全链路本机未实测，只验证命令构造与可达性门禁 |

首期 IDL 固定六类对象：`Observation`、`RobotState`、`SkillGoal`、`SkillFeedback`、`SafetyEvent`、`TaskStatus`。每条执行请求必须带 `correlation_id`、调用者身份、截止时间、Robot Profile 和 Safety Policy 的版本/摘要；反馈必须带单调状态序号和终态原因。

单次 Skill execution 为 `PENDING -> VALIDATING -> RUNNING -> SUCCEEDED`，异常终态为 `FAILED/ABORTED/CANCELLED/SAFETY_STOP`。终态不可复活或回写成功；重试、恢复和人工接管由 TaskFlow 创建新的 execution 并保留因果关联。

## 4. 跨发行版与架构策略

支持以“可重复构建的目标矩阵”定义，不能以开发机上一次运行成功宣称支持。IRAF 支持等级不沿用 ROS REP-2000 的 Tier 名称，避免把项目自测与 ROS 官方平台支持混淆。

| 环境 | IRAF 支持级别 | 交付方式 | 必测内容 |
|---|---|---|---|
| Ubuntu 22.04 amd64/arm64 | S1 原生认证 | 原生包 + Jammy/Humble Dev Container | 单元/契约/ROS 2 Humble/MuJoCo 仿真/HIL |
| Ubuntu 24.04 amd64/arm64 | C1 兼容宿主 | 运行同一 Jammy/Humble OCI | 单元/契约/无头 MuJoCo/DDS/设备透传；GUI/GPU 单独验收 |
| Ubuntu 24.04 amd64/arm64 | E0 原生预览 | Noble/Jazzy 独立镜像 | 公共核心、IDL、Ros2Adapter 兼容；未通过场景矩阵前不承诺生产 |
| CentOS Stream 9 amd64 | S1 容器开发 | 锁定 OCI 环境；宿主 CLI | 单元/契约/无头仿真/宿主兼容 |
| openEuler 24.03 LTS amd64/arm64 | S1 容器开发 | 锁定 OCI 环境；RPM/宿主 CLI | 单元/契约/无头仿真/宿主兼容 |
| E300、S600、FIREFLY | S2 逐板放行 | 目标 OCI 或 systemd bundle + BSP adapter | 冒烟、HIL、故障与安全验收 |

Hyper 构型也必须逐项放行：E300 LPR（Linux/Linux RT/RTOS）、E300 LPP（Linux/双 Linux RT）和 FIREFLY 2VM-LR（Linux/Linux RT）分别生成兼容报告。2VM 构型允许运控与主站同 VM，但不允许合并进程、权限、设备所有权或安全责任。

原生认证组合固定为 Ubuntu 22.04 + ROS 2 Humble + MuJoCo。ROS 2 Humble 对 Ubuntu 22.04 amd64/arm64 提供 Tier 1 支持；Ubuntu 24.04 的 ROS 官方原生目标是 Jazzy，因此 IRAF 用 Jammy/Humble 容器提供 24.04 宿主兼容，并以独立 Noble/Jazzy lane 准备原生迁移，不在 24.04 上混装 Humble。Piper 使用 `agilexrobotics/agx_arm_ros` ROS 2 分支；机器狗使用宇树 Go2 EDU、`unitree_ros2` 和 `unitree_mujoco`；所有上游锁定 commit。MuJoCo/ROS 2 桥在 M0 通过 spike 锁定，TaskFlow 与 Skill 不依赖桥的内部接口。参考资料：[ROS REP-2000](https://www.ros.org/reps/rep-2000.html)、[MuJoCo](https://github.com/google-deepmind/mujoco)、[MuJoCo ros2_control](https://github.com/ros-controls/mujoco_ros2_control)、[AgileX ROS 2 驱动](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)、[Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2)、[Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)。

构建输出按 `linux/amd64`、`linux/arm64` 以及实际 BSP 要求的架构发布多架构 OCI manifest。C1/S1 容器支持表示能够在锁定的 Ubuntu 22.04 容器中开发和运行无头 MuJoCo 仿真，不表示 ROS 官方为宿主提供 Humble 原生二进制包；原生 GUI/GPU 加速另行验证。所有非实时用户态核心模块坚持 C++20/POSIX，避免绑定发行版私有库；ROS 2 发行版差异、MuJoCo、GPU/NPU、Piper/Go2 SDK 和现场总线 SDK 均留在 adapter 层。每个产物生成 SBOM、镜像 digest、依赖清单和 ABI/IDL 兼容报告。

E300、S600、FIREFLY 的精确 SKU 仍须确认，故本文不对未验证组合做硬件断言。进入 S2 前必须填写并由硬件/BSP 负责人签字的板级清单：架构、OS/内核、容器运行时、可用 RAM/存储、GPU/NPU runtime、EtherCAT/AUTBUS 支持、PTP/时钟、急停链路、驱动版本、镜像 digest。

## 5. Profile 和适配边界

`RobotProfile` 描述能力、关节/传感器、可达空间与策略边界；`BoardProfile` 描述部署能力，不描述业务语义；`SafetyPolicy` 描述不可由外部覆盖的限制。启动时 `profile-check` 读取 OS、架构、内核、设备、时钟和适配器版本，生成 capability inventory；任何不满足 `requires` 的 Skill 在 `VALIDATING` 拒绝。

```yaml
apiVersion: iraf.intewell.io/v1
kind: BoardProfile
metadata: {name: e300, version: 0.1.0}
spec:
  status: unverified
  target: {os: pending, arch: pending, kernel: pending}
  runtime: {oci: pending, ros2: pending}
  adapters: {master: pending, gpu_npu: pending}
  evidence: {owner: bsp-team, acceptance_report: pending}
```

由此可先在 Ubuntu/CentOS/openEuler 建立可信开发闭环，再以同一清单逐板启用硬件功能；不能为赶进度把 `pending` 当成可用能力。

## 6. 安全、降级与可观测性

Policy Gateway 的执行顺序固定为：身份/RBAC、IDL schema、技能前置条件、参数范围、Robot Profile、Safety Policy、Capability 可用性、执行授权。模型服务失联只影响未开始的语义规划或策略选择；已开始运动遵循本地控制器，按策略停留、暂停、恢复或人工接管。设备/看门狗/急停故障必须生成 `SafetyEvent` 并优先停止。

每次调用记录 task/skill/provider/model/controller/profile/policy 版本、输入摘要、策略决定、状态迁移、时间戳、故障和 trace ID。远程模型通道采用 mTLS、短超时、限流、健康检查、版本匹配与断网回退；密钥仅由部署环境提供。

## 7. 质量门禁与验收

每次合并至少通过格式化、静态分析、单元测试、IDL 破坏性变更检查和仿真/契约测试。变更 runtime、provider 或 profile 时再运行故障注入；变更板级/实时接口时必须附 HIL/真机证据。首期验收包括：跨 Ubuntu 与目标边缘执行同一 TaskFlow；同一 Skill Provider 切换不变更合约；模型断连、设备失联与传感器异常的受控降级；以及完整调用可追溯。

## 8. 易用与泛化设计

IRAF 的使用者不应从 ROS topic、板级 SDK 或部署脚本开始。框架提供“应用开发者只面对能力，集成者才面对适配器”的两层体验：应用开发者组合 TaskFlow 和标准 Skill；算法工程师实现 Skill Provider；机器人集成商填写 Robot/Board/Safety Profile；平台工程师维护镜像、适配器和发布矩阵。

| 使用者 | 最小输入 | 框架自动提供 | 不需要知道 |
|---|---|---|---|
| 应用开发者 | TaskFlow、目标物、目的地、优先级 | schema 校验、Skill 路由、状态/错误、回放 | ROS topic、驱动、板型 |
| 算法工程师 | Provider、输入输出合约、资源需求 | 生命周期、取消、策略检查、可观测性 | AgentOS 调度细节、部署路径 |
| 集成商 | URDF/传感器、Robot/Board Profile | 能力探测、契约测试、适配器模板 | 业务 TaskFlow 实现 |
| 运维人员 | 签名发布包、目标 Profile | 依赖预检、版本摘要、健康/回滚信息 | 控制算法内部细节 |

### 8.1 金路径与脚手架

首期 CLI/模板的交付目标如下；命令在实现前只是产品契约，不可在发布说明中表述为已可用：

```text
iraf init delivery-demo            # 创建 SDK、TaskFlow、Profile 与测试骨架
iraf dev up --scenario handoff-lab # 启动 Piper/机器狗仿真与可观测组件
iraf skill scaffold inspect_item   # 创建 Provider、IDL 映射、单测与 manifest
iraf profile check --robot piper_lab --board <profile>
iraf test scenario handoff-lab     # 执行成功和故障注入用例
iraf package --profile <profile>   # 输出 SBOM、签名摘要与可部署包
```

每个 Skill 包包含 `skill.yaml`、输入输出 JSON Schema/IDL 引用、最小示例、Provider 模板、单元测试、契约测试和故障场景。`RobotProfile`、`BoardProfile`、`SafetyPolicy` 均提供 schema、中文字段说明和 `profile check` 静态验证。SDK 把通用错误码映射为可操作中文诊断，例如缺少能力、版本不匹配、前置条件不满足或安全策略拒绝。

#### 8.1.1 当前可复跑的脚本入口（已实现，与上表的 `iraf` CLI 契约分列）

上面的 `iraf …` 是产品契约，**尚未实现**，不得在发布说明中表述为已可用。M1.7 战役已落地的等价入口是脚本（`PYTHONPATH=src`，解释器口径 `/usr/bin/python3`）：

| 能力 | 已实现入口（可复跑） |
|---|---|
| 声明校验（机型/板卡/四足） | `scripts/profile_check.py --baseline <cfg>` · `--board <id>` · `--quadruped` |
| 机型基线与 IK 证据 | `scripts/build_baseline.py --baseline <cfg>` |
| 抓取验收（唯一权威数字） | `scripts/verify_pick.py --baseline <cfg> [--rebuild] [--skill visual_pick]` |
| 显示/演示（数字**不是**验收数字） | `scripts/view_mujoco.py --baseline <cfg> [--live]` |
| 场景包契约 | `scripts/scene_check.py --scene scenes/handoff_lab [--require-resolved-refs] [--require-model]` |
| 场景包 → 可加载 MJCF | `scripts/build_scene.py --scene <id> --robot <id>` |
| 传感器验收（相机/雷达/IMU） | `scripts/verify_scene_sensors.py --scene <id>` |
| 四足 loopback（站立/停止/状态） | `scripts/verify_go2_loopback.py --config config/go2_loopback.yaml` |
| S2 脚本化场景与故障注入 | `scripts/scenario.py list` · `scripts/scenario.py run --scene <id> --scenario <id>` |
| 厂商资产按锁重取/校验 | `scripts/fetch_vendor_assets.py --verify-lock` |
| SDK 打包与产物校验 | `bash deploy/sdk/build_sdk.sh [--dry-run] [--verify]` |
| 离线 wheelhouse 抓取 | `bash deploy/sdk/fetch_wheelhouse.sh [--dry-run] [--verify]` |
| 板级 bundle 组装 | `bash deploy/sdk/package_board_bundle.sh --board <id>` |
| 目标端安装/自检/卸载 | `bash deploy/sdk/install.sh` · `verify.sh [--bundle <tar.gz>] [--root <dir>] [--dry-run]` · `uninstall.sh --root <dir>` |
| 双通道部署 | `bash deploy/sdk/deploy.sh --transport media|ssh` |

目标端（边缘板卡）相关入口只在本机以演练/`--dry-run` 取证，真实安装与 `/health` 一律 **DEFERRED**，详见 §4 与 ADR-0006。

### 8.2 泛化约束

泛化不等于抽象一切。核心仅抽象稳定的任务、Skill、能力、状态、策略和配置；运动控制、传感器、仿真器、模型和板卡差异保留在 Provider/adapter/profile。新本体接入的目标是“新增声明和适配器，不修改 TaskFlow/Runtime”；新场景的目标是“新增 TaskFlow 和参数，不复制控制逻辑”。
