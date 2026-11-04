# IRAF v1.8 架构基线

## 图纸

- `iraf-refined-architecture.png`：总体架构图（由 `agentos机器人应用框架-更新版.dot` 生成），说明 AgentOS worker、IRAF 智控域/运控域、MCP 北向鉴权、Provider 与仿真/真机后端的模块边界。
- `iraf-detailed-interface-security.png`：详细接口/安全架构图（由 `IRAF详细技术架构图.dot` 生成），说明 MCP 鉴权、TaskFlow、Skill Runtime、Policy Gateway、ControlAuthority 的 lease/fencing/幂等、SafetyEvent 隔离与证据链。

## 工程边界

IRAF 是 AgentOS 到机器人 Skill 的应用框架，不替代实时控制器、BSP、驱动、现场总线主站或 Hypervisor。AgentOS 只能通过 AgentOSBridge 和版本化公共服务接入；业务请求按 `TaskFlow -> SkillRuntime -> Policy Gateway -> Provider` 的契约边界处理。Provider 分为传统算法控制、VLA 和强化学习等类型，ROS 2/MoveIt2/ros2_control 属于传统算法控制 Provider；MuJoCo 和真机属于后端适配环境。

## 当前基础与实施路线

当前已具备可复用 MuJoCo 仿真代码、ROS 控制代码、Piper 抓取场景闭环和机器人 Profile 统一规约基础。

- 一阶段：在 Ubuntu 22.04 + ROS 2 Humble + MuJoCo 上完成公共契约、Registry、Skill Query、Tool/Skill Manager、TaskFlow、Runtime、Policy 和单体仿真；板卡侧 AgentOS 通过容器运行 aarch64 的 Intewell-Agent RPM。
- 二阶段：完成 classical/VLA/RL 多模式 Provider、交接闭环、HIL 和边缘适配。
- 三阶段：完成 World Model、模型 Provider、遥操作和数据闭环。

所有 Skill 必须声明 schema、能力依赖、前置/成功条件、错误码、超时、取消、恢复、安全等级和版本。仿真必须声明 `simulation=true`，不得把仿真结果表述为真机实时能力。

## 当前实现图纸（2026-09-02）

- `docs/diagrams/iraf-current-implementation-overview.png`：当前模块边界、Runtime 唯一入口、Profile/Policy、Skill Registry、Provider/Backend 及后续能力。
- `docs/diagrams/iraf-minimal-chain.png`：已实测的中文意图到 MuJoCo 最小链路，包含拒绝路径和当前安全边界。
- 同目录的 `.dot` 为可审查源文件，`.svg` 为可缩放版本。

## 新增交付边界图纸（2026-09-20，M1.7 战役 iraf-24h）

| 图 | 源文件 | 说明 |
|---|---|---|
| SDK 跨架构交付图 | `docs/diagrams/iraf-sdk-delivery.dot` → `.svg` | 开发端 x86_64 → 产物（wheel / runtime bundle / 离线 wheelhouse / 板级 bundle）→ 目标端 aarch64 → 证据；标注 L1/L2/L3 分级与当前可达级别、演练证据与目标端证据的分野。依据 ADR-0006 与 `docs/iraf-multiplatform-sdk-design.md`。 |
| 宇树场景栈图 | `docs/diagrams/iraf-unitree-scene-stack.dot` → `.svg` | 场景包 → 场景构建器 → adapter → 技能层 → 运行时；标注厂商 MJCF 只读（按 `source-lock.json` 对账）、注入式传感器、以及「人形仅静态模型」。依据 ADR-0004、ADR-0007 与 `docs/iraf-unitree-scenario-interaction-design.md`。 |

两图均由 `dot -Tsvg`（本机 graphviz 2.43.0）从 `.dot` 源生成，源文件与 SVG 同时入库，可直接复跑：

```bash
dot -Tsvg docs/diagrams/iraf-sdk-delivery.dot       -o docs/diagrams/iraf-sdk-delivery.svg
dot -Tsvg docs/diagrams/iraf-unitree-scene-stack.dot -o docs/diagrams/iraf-unitree-scene-stack.svg
```

图内所有结论均为开发端（x86_64）可复现的部分；目标端真实安装、`/health`、AgentOS 联通为 **DEFERRED（延后，不是失败）**，因为边缘板卡不在场。

## 根级两张总图的源文件（2026-11-04 补齐）

根级两张总图已补 `.dot` 源，不再属于“只能手工改 SVG”的欠债。图为 v0.4.1 口径
（MCP Bearer 鉴权、AgentOS worker 共享 config、ProviderKit、智控域/运控域分离）：

| 图 | 源文件 | 生成命令 |
|---|---|---|
| 总体架构 | `agentos机器人应用框架-更新版.dot` | `dot -Tsvg ... -o agentos机器人应用框架-更新版.svg`；`dot -Tpng -Gdpi=140 ... -o iraf-refined-architecture.png` |
| 详细技术架构 | `IRAF详细技术架构图.dot` | `dot -Tsvg ... -o IRAF详细技术架构图.svg`；`dot -Tpng -Gdpi=140 ... -o iraf-detailed-interface-security.png` |

两张图与 `docs/mcp-binding.md`、`docs/adr/0011-mcp-northbound-auth.md`、
`docs/upper-sdk-guide.md`、`docs/provider-sdk-guide.md` 同口径；图内所有执行结论仍为
`simulation=true`，不构成真机/实时能力证据。

`docs/diagrams/iraf-current-implementation-overview.{dot,svg,png}` 与
`iraf-minimal-chain.{dot,svg,png}` 已有 `.dot` 源，不属本欠债。

