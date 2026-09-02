# IRAF 边缘适配、数据闭环与 AgentOS 接入详细设计

**状态**：编码前基线  
**实现范围**：`adapters/`、`core/world/`、`deploy/`、`tools/`

## 1. Capability Adapter 与 ROS 2 映射

Adapter 是唯一可了解 ROS 2 topic/action、厂家 SDK 和现场总线 API 的模块。它实现 `SkillProvider`，将 `ValidatedGoal` 映射到 ROS 2 action，并把 feedback/result 反映回 Runtime。ROS 2 namespace 由 Profile 提供，禁止硬编码。

| IRAF Skill | ROS 2 Action 目标 | 输入映射 | 关键反馈 |
|---|---|---|---|
| `detect_object` | `/<ns>/detect_object` | object/ROI | confidence、Pose、frame、timestamp |
| `pick_object` | `/<ns>/pick_object` | Pose、grasp policy | phase、gripper、collision check |
| `place_object` | `/<ns>/place_object` | `tray_frame` Pose | release、payload confirmation |
| `navigate` | `/<ns>/navigate_to_pose` | map Pose、constraints | distance、velocity、replan |
| `dock_for_handoff` | `/<ns>/dock` | dock Pose、tolerance | pose error、stable、speed |
| `stop` | `/<ns>/stop` | reason | controller acknowledged |

Adapter 必须在 action 开始前检查 frame、时间戳、新鲜度和 capability inventory；取消请求须转为 ROS 2 cancel，并在 Profile 规定的 `cancelAckMs` 内收到确认，否则发送 SafetyEvent。主站 adapter 只接受经 Policy 约束的目标，使用本地 IPC/共享内存/确定性总线；不把 gRPC 消息直接转发至 EtherCAT/AUTBUS。

Provider 默认以独立进程部署，通过版本化 gRPC/ROS 2 Action 契约与 Runtime 通信，避免 Ubuntu、CentOS、openEuler 之间的 C++ ABI、libstdc++ 与厂家 SDK 冲突。只有随 Runtime 同工具链构建、同镜像发布的内置 Provider 才能使用进程内 C++ 接口；禁止加载第三方 C++ `.so` 作为通用插件。Provider manifest 必须声明 protocol version、进程/镜像 digest、健康端点、资源和权限。

ROS 2 基线固定维护 DDS QoS Profile：命令/Action 使用 reliable + bounded history；高频传感器按能力使用 best-effort；SafetyEvent 使用 reliable、transient-local 和独立 callback group。所有时间戳统一为同步后的系统时间并携带 source clock；TF 发布者必须在 RobotProfile 中唯一声明，重复发布同一 frame pair 直接拒绝启动。Profile 同时定义 Observation TTL、最大允许时钟偏差与 `cancelAckMs` 的平台上限，SafetyPolicy 只能收紧这些值。

## 2. Piper/机器狗集成细节

`PiperAdapter` 管理 `piper.arm`、`piper.gripper` 和 `piper.eye_in_hand_camera`；`QuadrupedAdapter` 管理 `quadruped.base`、`quadruped.camera`、`quadruped.lidar`、`quadruped.tray`。交接时由 `handoff_zone` 独占锁串行化两个 Provider。唯一跨本体接口是经 TF 校验的 `tray_frame`、标准 RobotState 和 Skill 结果，禁止 Piper 直接发布机器狗底盘命令或反向调用。

Piper 的 classical、VLA、learned policy、teleop 和 replay Provider 必须经过 `ControlAuthorityManager` 获取唯一写权限。策略模式与执行后端正交：同一 Provider 合约可连接 MuJoCoAdapter、HIL 或 PiperHardwareAdapter。VLA/学习策略输出 ActionProposal，经标准化、Policy Gateway 与 Safety Projector 后才进入 MoveIt2/ros2_control 确定性执行器。

每个 adapter 提供 `Health()`：连接、最后状态时间、控制模式、急停状态、时间同步、传感器新鲜度、软件版本。Health 低于 profile 阈值时 Registry 将其摘除；已执行任务按 TaskFlow 指定的安全路径处理。

## 3. 世界模型与数据存储

世界模型只存“可用于决策且带来源/置信度/时效”的状态，不替代 ROS 2 原始传感器日志。实体表为 `Robot`、`Object`、`Place`、`Human`、`Task`、`Zone`；关系表为 `located_in`、`held_by`、`on_tray`、`blocked_by`。每条 Observation 包含 source、sequence、frame、timestamp、confidence、TTL、schema version。过期 Observation 不可作为 TaskFlow 前置条件成功依据。

```text
Observation -> schema/frame/time check -> WorldModel upsert -> StateChanged event
Skill feedback/SafetyEvent ------------------------------^                 |
TaskFlow query <- typed snapshot / prediction (non-real-time) <------------+
```

首期使用 PostgreSQL（生产）与 SQLite（单机仿真）的同一逻辑模型，但不假定两者 SQL 完全等价。EventStore/WorldRepository 通过端口接口隔离数据库，迁移只使用两端验证过的 SQL 子集；每个 migration 和并发/事务语义必须跑 PostgreSQL/SQLite conformance suite。时序 telemetry/大对象只写对象存储，事件数据库保留 digest、URI、访问策略和回放索引。`ReplayManifest` 冻结场景、TaskFlow、Skill/Provider、Robot/Board/Safety Profile、模型、控制器、镜像 digest、输入摘要和事件序列。

## 4. AgentOS、公共服务与模型 Provider

公共 API 明确拆分为四个版本化服务：

| 服务 | 责任 | 主要方法 |
|---|---|---|
| `TaskService` | 业务任务入口 | `Submit/Cancel/Resume/Get/Watch` |
| `SkillRuntimeService` | 受控 Skill 执行 | `Execute/Cancel/GetExecution` |
| `WorldModelService` | 类型化状态查询 | `GetSnapshot/Query/WatchChanges` |
| `EventService` | 审计和回放索引 | `ListEvents/GetReplayManifest` |

AgentOS 默认只调用 `TaskService`、`WorldModelService` 和只读事件接口；直接调用 `SkillRuntimeService` 需要显式 `skill.execute.direct` 权限。所有服务从 mTLS/JWT interceptor 获取 `AuthenticatedContext`，流式订阅使用单调 sequence、resume token、心跳和慢消费者上限，断线后按 token 续传。AgentOS 无主站或设备凭据。

AgentOS 私有协议由独立 `AgentOSBridge` 映射到上述公共服务。启动前调用 `GetCompatibility` 交换 protocol major/minor、feature bits、IDL digest 和构建摘要；服务端至少运行当前/上一主版本的契约测试，不兼容 major 必须拒绝。AgentOS 的 `Drain/Shutdown` 只能触发 Runtime 定义的停止流程，不得绕过资源租约或直接终止主站安全闭环。

模型 Provider 统一为 `Generate`、`Embed`、`Infer` 三个带认证上下文、deadline、model version、token/资源上限的异步接口。只允许在 TaskFlow 生成、语义感知、候选目标/技能选择和失败重规划使用；Provider 返回值必须经 schema 与 Policy Gateway 验证。

远程 Provider 的失败策略：到 deadline 立即返回明确错误；TaskFlow 可选择本地/回放 Provider、等待、接管或失败。不得因远程模型不可用而阻塞 `stop`、运行中的局部控制或 SafetyEvent 处理。

## 5. Profile 检查、部署与验收证据

`profile-check` 在部署前采集 OS、架构、内核、OCI runtime、ROS 2、适配器、GPU/NPU、网络/时间同步和设备 inventory，并与 BoardProfile 的已签名声明比较。输出 JSON 和中文摘要；缺项、版本不符或 `unverified` 时非零退出。生产 bundle 由 OCI digest、SBOM、签名、Profile/Policy/Skill manifest 组成，systemd/compose 只引用 digest，不引用浮动标签。

签名体系必须定义离线根信任、环境级签发密钥、允许的 signer、证书/密钥轮换、吊销列表和回滚保护。Runtime 只加载 TrustStore 允许且版本不低于防回滚计数器的 bundle；开发自签名只能在 `dev-sim` profile 中启用。

每个板级验收包最少包含：命令与版本、ProfileCheck 输出、DDS/adapter 冒烟、HIL 成功/失败场景、急停/失联结果、48 小时稳定性记录和已知限制。稳定性门槛为非预期运动/重复命令/事件丢失/未恢复进程崩溃均为 0，资源租约和线程无持续增长，预热后 RSS 增长不超过 5%，所有故障都有终态。E300、S600、FIREFLY 未完成此证据前只能标记开发支持。

`HyperProfile` 还必须匹配 AICICD Release 的 board、`vm_profile`、VM 类型、Hypervisor/内核/RTOS commit 和 `/etc/iaros-version`。E300 LPR、E300 LPP、FIREFLY 2VM-LR 分别验收；详细部署与跨域通道约束见 `iraf-agentos-hyper-compatibility.md`。S600 在正式 release profile 建立前不得进入 S2。

## 6. 端到端测试矩阵

| 层 | 自动化测试 | 必须故障 |
|---|---|---|
| IDL/SDK | 兼容、序列化、错误码 | 未知字段/旧客户端 |
| Runtime/Policy | 状态、幂等、租约、规则 | 拒绝、deadline、急停 |
| Piper/机器狗仿真 | 交接、导航、回放 | 遮挡、抓取失败、对接误差、雷达失效 |
| HIL/板级 | 健康、通信、受控停止 | 设备失联、cancel 超时、急停 |
| 发布 | 签名、SBOM、profile match | 篡改/版本不匹配 |
| AgentOS / Hyper | N/N-1 契约、三构型部署、版本回读 | VM0 失联、跨域断连、构型错配、迟到命令 |
