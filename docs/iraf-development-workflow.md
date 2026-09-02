# IRAF 开发与验证流程

## 每个功能的流程

```text
需求/验收场景
  -> Skill/TaskFlow/Profile/IDL 影响评估
  -> 设计评审（安全等级、失败与接管路径）
  -> 实现与单元测试
  -> IDL 契约测试
  -> 仿真故障注入
  -> HIL/板级验证（如适用）
  -> 证据归档与发布
```

## 运行模式

| 模式 | 环境 | 允许能力 | 明确限制 |
|---|---|---|---|
| `dev-sim` | Ubuntu 22.04 + Humble + MuJoCo 原生；Ubuntu 24.04/CentOS/openEuler 运行锁定 Jammy/Humble OCI | SDK、TaskFlow、Skill、无头仿真、回放、模型 stub/remote | 24.04 不宣称 Humble 原生；跨宿主 GUI/GPU/设备透传逐项验收 |
| `dev-next` | Ubuntu 24.04 + Jazzy 独立实验镜像 | 公共核心、IDL、Adapter 兼容和迁移回归 | 未通过 Piper/Go2/场景矩阵，不进入生产发布 |
| `dev-hil` | Linux + 部分真实控制/通信设备 | 契约、适配器、故障注入 | 受控工装，非生产任务 |
| `edge-prod` | 已验收的 E300/S600/FIREFLY | 已签名 profile 的真实能力 | 仅支持清单中验证过的设备/版本 |

所有 API、日志和 UI/CLI 必须暴露当前模式。`dev-sim` 不能伪装为 `hardware=true`。

## CI 流水线

1. `lint`: 格式化、许可证、敏感信息扫描、依赖锁定检查。
2. `contract`: Protobuf breaking-change 检查，生成 C++/Python SDK，并运行 current/previous 兼容测试。
3. `unit`: TaskFlow、Policy、Registry、Profile、错误状态覆盖。
4. `sim`: 容器化仿真执行成功、取消、超时、模型不可用、传感器异常、设备失联。
5. `package`: 多架构 OCI、SBOM、签名、镜像 digest。
6. `hardware`: 仅受保护 runner 触发；归档 BoardProfile、BSP/镜像版本、HIL 日志与验收报告。
7. `agentos-hyper`: 运行 AgentOS N/N-1 契约和 LPR/LPP/LR 三套部署 profile；检查发布元数据、HyperProfile 与运行时回读一致。

## 架构变更同步门禁

PR 必须填写 `architecture-impact: yes/no`。选择 `yes` 或变更 `core/`、公共 `api/`、Provider/adapter 边界、Profile、平台矩阵或安全执行路径时，必须在同一 PR 更新 `agentos机器人应用框架-更新版.svg`、`IRAF详细技术架构图.svg` 和对应 ADR。CI 的 `docs-architecture` job 校验两张 SVG 的 XML、文档链接和 ADR 状态；评审人确认图中的模块、调用方向、信任边界和文档一致后才可合并。

## 事件与故障处理

| 事件 | Runtime 动作 | 必须记录 |
|---|---|---|
| Provider 超时 | `FAILED`，按 Skill 策略恢复/接管 | deadline、provider、重试次数 |
| 模型不可用 | 禁止新语义决策；已运行闭环局部受控 | provider 状态、降级路径 |
| 设备失联 | 主站看门狗受控停机，发 `SafetyEvent` | 设备、最后心跳、停机结果 |
| 急停 | 立即 `SAFETY_STOP`，资源隔离至控制器确认安全并受控复位 | 急停源、时间、safe-state ack、授权复位者 |
| Profile 不匹配 | `VALIDATING` 拒绝执行 | 期望/实际 capability inventory |

## 发布流程

发布候选必须锁定 IDL、Skill、Profile、Safety Policy、模型、控制器和 OCI digest。发布说明列出平台支持矩阵与已知限制；仅具有通过硬件验收证据的 E300/S600/FIREFLY 组合可以标记为生产支持。
