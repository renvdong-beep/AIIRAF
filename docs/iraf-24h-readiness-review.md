# IRAF 24 小时连续开发审查

**审查角色**：产品经理 + 项目经理  
**结论**：可以启动 M0；M1/M2 必须依次满足前置门禁，不得并行绕过契约与安全决策。不得在板卡/BSP 信息缺失时承诺 E300、S600、FIREFLY 的生产部署日期或实时性能。

## 详细设计复审（2026-08-28）

**复审结论：有条件通过。** 已完成 IDL/Runtime、TaskFlow/Policy、边缘适配/数据闭环和 Piper-机器狗场景的编码前设计，并用 ADR 固化身份、资源、Provider 和平台基线。当前只放行 M0；M1 在 M0 smoke/ADR 签署后启动，M2 在完整 proto 与契约测试通过后启动，M3 在 Piper/机器狗版本 spike 完成后启动。

仍未放行的事项：AgentOS 精确接口版本、Go2 EDU 精确 SKU/固件与许可证 BOM、Piper/Go2 锁定 commit 的兼容性 spike、三个 Hyper 构型的真实通道/HIL 证据、S600 正式 release profile、HIL 工装与安全责任人。它们不阻塞平台无关核心，但分别阻塞 M2 集成、对应 Provider、三构型和 S2 发布。

## 产品经理审查

| 项目 | 结论 | 证据/要求 |
|---|---|---|
| 用户价值 | 通过统一 TaskFlow/Skill 降低机器人应用重复开发 | 已在设计中定义跨仿真/真机目标 |
| MVP 边界 | 清晰 | 首期完成四服务 IDL、Runtime、Policy、Profile、8 个交接基础 Skill 与仿真 |
| 安全价值 | 清晰且不可降级 | 模型不进入实时环；策略与 Profile 强制执行 |
| 多平台诉求 | 可实现，但须按能力矩阵承诺 | Ubuntu 22.04/Humble/MuJoCo 原生 S1；Ubuntu 24.04 宿主 C1、Jazzy 原生 E0；CentOS/openEuler 容器 S1；三板卡逐板 S2 |
| 体验/可观测性 | 需要作为 MVP 必选项 | trace、状态、错误码、回放不可后置 |

产品验收问题：同一配送 TaskFlow 是否能在至少一个仿真环境与一个通过验收的边缘 profile 上执行？调用者是否能理解失败原因并发起恢复或接管？模型断网是否有可预测且安全的用户可见状态？这些问题全部回答“是”才可宣布 MVP。

## 项目经理审查

| 风险/依赖 | 当前状态 | 所有人 | 下一步 |
|---|---|---|---|
| E300/S600/FIREFLY 硬件/BSP 清单 | 阻塞 S2 放行 | 硬件/BSP 负责人 | 提供版本化清单与设备访问 |
| AgentOS 接口/认证方式 | 待确认 | AgentOS 负责人 | 评审 IDL、身份与生命周期接口 |
| LPR/LPP/LR 发布 manifest 与跨域 transport | 阻塞 M5 | Release/Hypervisor | 提供构型制品、通道 API 和版本回读样本 |
| ROS 2/仿真器版本 | 已决策、待 spike | 运控负责人 | 验证 Humble/MuJoCo/Piper 与 ROS 桥锁定 commit |
| EtherCAT/AUTBUS 主站接口 | 待确认 | 通信主站负责人 | 定义受控目标与 SafetyEvent 接口 |
| CI runner 与 HIL 工装 | 待准备 | DevOps/测试负责人 | 创建 protected runner 和证据归档 |

## 前 24 小时排程

| 时间窗 | 交付物 | 负责人角色 | 退出条件 |
|---|---|---|---|
| 0-3h | 签署 Humble/MuJoCo、Go2 EDU ADR，确认责任人与采购/SKU 输入 | 架构师 + 产品/运控/BSP | 决策与未决项有 owner |
| 3-9h | monorepo、CI、Ubuntu 22.04/Humble/MuJoCo Dev Container | 平台工程师 | amd64 无头 MuJoCo smoke green |
| 9-15h | 四类服务 proto、错误码、`buf lint/build` | Runtime 工程师 | proto 可编译且无 breaking issue |
| 15-20h | C++/Python SDK 生成、认证 interceptor 骨架 | Runtime/安全工程师 | 身份不可由请求伪造 |
| 20-24h | MuJoCo ROS 桥、CentOS/openEuler 宿主 smoke、M0 报告 | 平台/仿真工程师 | 桥选型和三宿主证据进入 RAID |

## Go / No-Go 门槛

**Go**：M0 平台 smoke 和 ADR 签署完成；M1 IDL 与兼容策略通过；M2 显式拒绝越权参数与缺失能力，并通过 fencing token/隔离资源测试；M3/M4 仿真达到量化验收。

**No-Go**：以 mock success 掩盖未实现 Provider；让远程模型调用控制实时设备；没有 profile/BSP 证据而将板卡标成生产支持；或缺少急停、失联、超时的终态处理。

## 决策请求

在进入 M5 前，项目必须指定每个 E300、S600、FIREFLY 的精确 SKU、CPU 架构、OS/BSP 版本、容器/ROS 2 支持、加速器与现场总线配置，并指定可用的 HIL/真机窗口。没有这些输入，团队仍可完成平台无关核心与仿真，但不能完成边缘侧验收。
