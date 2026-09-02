# IRAF 与 AgentOS / Hyper 构型兼容设计

**状态**：编码前基线  
**适用对象**：AgentOS、IRAF、Intewell Hypervisor、E300、FIREFLY-RK3588；S600 待正式发布构型

## 1. 兼容边界

从逻辑分层看，IRAF 对北向兼容 AgentOS，对南向适配 Hyper 承载的智控、运控和通信主站角色。兼容不等于把 AgentOS 或 Hypervisor 编进 IRAF：AgentOS 通过版本化服务调用 IRAF；Hypervisor 负责 VM、CPU、内存和外设隔离，IRAF 只按已签名部署 Profile 把组件放到正确域，并使用批准的跨域通道。

```text
AgentOS
  -> AgentOSBridge（身份委托、任务、生命周期、事件、版本协商）
IRAF Control Plane（TaskFlow / Registry / Policy / World Model）
  -> SkillExecutionChannel（语义化、可重试、非硬实时）
IRAF Motion Plane（ROS 2 / MoveIt2 / Nav2 / WBC / 控制适配）
  -> MotionCommandChannel（固定结构、时间戳、序列、超时）
IRAF Master Adapter（EtherCAT / AUTBUS / 厂家设备接口）
```

AgentOS 不获得 ROS 2 控制 topic、厂家 SDK、主站或设备凭据；IRAF 不实现 AgentOS 的调度/记忆/模型治理，也不实现 Hypervisor 的 VM 管理、BSP 或硬实时主站。

## 2. AgentOS 兼容契约

`AgentOSBridge` 是独立北向适配层，不允许 AgentOS 私有对象进入 IRAF canonical IDL。它映射到四个公共服务：

| AgentOS 能力 | IRAF 接口 | 兼容要求 |
|---|---|---|
| 任务提交/取消/恢复 | `TaskService` | 幂等键、deadline、调用方身份、TaskFlow 版本 |
| 世界状态读取/订阅 | `WorldModelService` | typed snapshot、sequence、TTL、resume token |
| 直接 Skill 调用 | `SkillRuntimeService` | 默认禁用；需 `skill.execute.direct` scope |
| 审计与回放 | `EventService` | 只读、分页/续传、租户隔离 |
| 生命周期 | `Health/Ready/Drain/Shutdown` | drain 后拒绝新任务，运行任务进入定义的完成或安全停止路径 |
| 身份委托 | `AuthenticatedContext` | mTLS/JWT；audience、tenant、subject、scope、expiry、credential ID |
| 版本协商 | `GetCompatibility` | protocol major/minor、feature bits、IDL digest、runtime/build digest |

兼容策略为当前主版本 `N` 和上一主版本 `N-1` 的客户端契约测试；major 不兼容时拒绝启动或请求，minor/feature 不支持时显式降级，不静默忽略安全语义。AgentOS Dev Runtime 和边缘 AgentOS 必须使用同一 IDL、认证和状态迁移测试，禁止固定返回值的假 AgentOS 作为集成验收。

## 3. 三个 Hyper 发布构型

构型事实源来自 AICICD Release 的 `packaging_configs` 和随制品生成的版本清单。VM 编号以发布 profile 为准；用户手册中的显示编号或默认 IP 只用于现场诊断，不可写入 IRAF 业务代码。

| 发布构型 | VM 拓扑 | IRAF 部署映射 | 必测差异 |
|---|---|---|---|
| `MQ50-E300-3VM-LPR` | VM0 Linux；VM1 Linux RT；VM2 RTOS | VM0：AgentOS + IRAF Control/Data；VM1：Motion Plane/ROS 2；VM2：Master Adapter/现场总线服务 | Linux RT ↔ RTOS 通道、RTOS 看门狗、安全状态确认、AUTBUS/EtherCAT 事实 |
| `MQ50-E300-3VM-LPP` | VM0 Linux；VM1 Linux RT；VM2 第二 Linux RT | VM0：AgentOS + IRAF Control/Data；VM1：Motion Plane；VM2：Master Adapter/主站服务 | 双 Linux RT 的服务隔离、CPU/内存上限、主站周期与故障隔离 |
| `FIREFLY-RK3588-2VM-LR` | VM0 Linux；VM1 Linux RT | VM0：AgentOS + IRAF Control/Data；VM1：Motion Plane 与 Master Adapter 分进程/权限部署 | 合并域资源竞争、进程故障隔离、设备独占、最坏时延 |

三者共享同一 TaskFlow、IRAF canonical IDL、Skill manifest、RobotProfile 和 SafetyPolicy 语义，但不要求同一可执行文件、相同 ROS 2 安装或相同跨域 transport。每个构型独立产出 bundle、SBOM、签名和验收报告。

`S600` 当前没有出现在上述三项正式 AICICD `packaging_configs` 中，因此保持 `BoardProfile.status: unverified`。只有新增正式 board baseline、Hyper config、组件来源、assembler 和独立验证后，才能进入 IRAF S2 发布矩阵。

## 4. HyperProfile

IRAF 增加独立 `HyperProfile`，引用而不复制 AICICD Release 的构型事实：

```yaml
apiVersion: iraf.intewell.io/v1
kind: HyperProfile
metadata:
  name: mq50-e300-3vm-lpr
  version: 0.1.0
spec:
  releaseProfile: MQ50-E300-3VM-LPR
  releaseVersion: pending
  hypervisorDigest: pending
  domains:
    control: {vm: vm0, osRole: linux, services: [agentos-bridge, iraf-runtime, world-model]}
    motion: {vm: vm1, osRole: linux_rt, services: [ros2-adapter, motion-provider]}
    master: {vm: vm2, osRole: rtos, services: [master-adapter, safety-watchdog]}
  channels:
    task: {class: managed, transport: pending, authenticated: true}
    motion: {class: bounded, transport: pending, timestamped: true, watchdog: true}
    safety: {class: priority, transport: pending, acknowledgement: required}
  evidence: {profileCheck: pending, hilReport: pending}
```

启动时 `profile-check` 对比 HyperProfile 与制品中的 board、`vm_profile`、VM 类型、Hypervisor/内核/RTOS commit、`/etc/iaros-version`、CPU/内存/设备分配和通道能力。任何不匹配都禁止 `edge-prod`，不得自动回退到另一构型。

## 5. 跨域通道铁律

1. AgentOS/TaskFlow 到运控只传 Task/Skill 语义和受约束目标，不传电机/力矩命令。
2. 运控到主站采用固定布局消息，必须包含 schema version、sequence、monotonic timestamp、deadline 和 integrity check。
3. 主站看门狗、急停和安全状态确认不依赖 AgentOS、gRPC、DDS、远程模型或 VM0 存活。
4. 管理网络中断不能阻断本地停止；恢复连接不得重放过期命令或复活已终止 execution。
5. LPP/LPR 不因 VM2 OS 不同而改变 Skill 结果语义；差异封装在 Master Adapter 和 HyperProfile。
6. 2VM-LR 中 Motion/Master 即使同 VM，也必须使用不同进程、用户/能力集、CPU/内存限额和设备独占策略。

## 6. 开发与验收

Ubuntu Dev Runtime 用三个容器/进程命名空间模拟 control、motion、master 角色，并提供 LPR、LPP、LR 三套 compose profile。该模拟只验证部署、契约、权限、失联和重启语义，不宣称等价于 Hypervisor 隔离或实时性能。

每个正式构型至少通过：AgentOS N/N-1 合约、任务 drain/restart、身份/权限、跨域断连、迟到/重复命令、时钟偏差、Motion 崩溃、Master 崩溃、VM0 失联、急停、授权复位、24 小时压力和 48 小时稳定性测试。发布证据必须关联 AICICD release version、三个组件 commit/digest、IRAF bundle digest 和硬件 SKU。
