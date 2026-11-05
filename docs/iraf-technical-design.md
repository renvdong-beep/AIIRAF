# IRAF 技术设计说明

> 适用版本：v0.4.1（MCP 北向鉴权 + 上位机/南向 SDK + AgentOS worker 绑定）
> 事实来源：`docs/iraf-v1.8-architecture-baseline.md`、`docs/iraf-engineering-design.md`、`docs/iraf-multiplatform-sdk-design.md`、`docs/adr/0011-mcp-northbound-auth.md`、`AGENTS.md`
> 全程 `simulation=true`，v1 只做仿真，不构成真机/实时能力证据。

---

## 1. 定位

IRAF（Intewell Robot Application Framework）是 AgentOS 到机器人 Skill 的**应用框架**，不替代实时控制器、BSP、驱动、现场总线主站或 Hypervisor。AgentOS 只能经 `AgentOSBridge` 和版本化公共服务接入；业务请求按 `TaskFlow → SkillRuntime → Policy Gateway → Provider` 契约边界处理。

## 2. 分层架构（L0–L8）

- **L0 调用方/语义层**：上位机 harness、AgentOS chat（agentos-worker）、IRAF CLI/语音。
- **L1 接入与身份**：`iraf-mcp-http.service :8765`（streamable-http /mcp，5 个任务级工具）、stdio MCP、`BearerAuthMiddleware`、token 注册表（SHA-256/600/热撤销）、AgentOS 共享内容层。身份只来自受信 transport。
- **L2 任务编排**：TaskFlow/AgentOSBridge、模型网关（受约束意图解析）、TaskDispatcher（request/correlation/idempotency/deadline）、Skill 契约。
- **L3 技能运行**：Skill Manager（版本选择/能力依赖/前置成功条件）、Skill Runtime（校验/执行/超时/取消/恢复）、Skill Registry（skills/<name>/skill.yaml）。
- **L4 安全策略**：Policy Gateway（AuthenticatedContext 判定）、RobotProfile（joints/limits/capabilities）、SafetyPolicy（allowed_skills/max_duration/simulation_only）、SafetyEvent→QUARANTINED。
- **L5 执行控制**：ControlAuthority（execution_id/lease TTL/单控制源）、fencing token（单调/旧 token 拒绝/终态拒绝新控制）、idempotency store、SqliteExecutionStore。
- **L6 能力适配**：RemoteProviderBackend / 本地 MujocoBackend、CapabilityProviderService（Health/Invoke/Cancel/LeaseEnvelope）。
- **L7 Provider**：x86 MuJoCo Provider（:50052/:50053）、客户 ProviderKit（南向 SDK）、运控域实时 Provider（L0/L1，v1 不验收 HIL）。
- **L8 交付/证据**：iraf_sdk wheel + runtime bundle、systemd 常驻、mcp-audit.jsonl + execution store + journal。

## 3. 三域分工

- **智控域（非实时）**：AI Agent、VLA、世界模型、任务规划；输出受约束的 Skill Goal，不输出关节/力矩/总线报文。
- **运控域（实时）**：ROS 2、导航/操作、WBC/MPC、状态估计、Skill Runtime；确定性控制。
- **通信主站域（实时）**：EtherCAT/AUTBUS 主站、设备聚合、周期控制帧、看门狗、受控停机。

关键约束：HTTP/网络推理/AgentOS 不得进入 L0/L1 闭环；仿真只在 x86；四套凭据（上位机/AgentOS/Provider/模型网关）互不复用。

## 4. 安全铁律（9 条，不可违反）

1. LLM/VLM/VLA/远程服务/业务应用不得直接发送电机、关节、力矩、现场总线或驱动命令。
2. 所有外部请求必须经过 `TaskFlow → Skill Runtime → Policy Gateway → Capability Provider`，禁止旁路。
3. 安全上限只来自已签名 RobotProfile 与 SafetyPolicy；调用参数只能收紧，不能放宽。
4. 实时控制路径必须本机、确定、有界、带时间戳和看门狗；网络推理/HTTP/gRPC/AgentOS 不得进入 L0/L1 闭环。
5. 模型/网络/传感器/Provider/设备不可用时只能显式失败、安全停机、恢复或人工接管；禁止伪造成功。
6. 急停、设备失联和安全事件优先级高于任务成功、重试和模型指令；终态不得被回写为成功。
7. 仿真必须声明 simulation=true；不得把仿真/Mock 表述为真机/实时能力。
8. 身份与角色只能来自受信 transport 的 AuthenticatedContext；不得信任请求体的 caller/role/tenant。
9. SafetyEvent 后相关资源必须进入 QUARANTINED，确认 safe state 且授权复位前不得释放。

## 5. 身份-安全-归属三层解耦（MCP 北向鉴权，0.4.0）

| 层 | 回答 | 载体 | 独立性 |
|---|---|---|---|
| 身份层（L1） | 你是谁 | Bearer token（transport）/stdio env；只存 SHA-256、文件 600、热撤销 | token 可撤销/轮换，不影响安全 |
| 安全层（L4） | 你能做什么 | 签名 RobotProfile/SafetyPolicy | token 不是安全上限 |
| 归属层（L5/L8） | 你做了什么 | execution 绑定 subject + get_for_subject 过滤 + audit JSONL | 他人 execution_id 返回 NOT_FOUND（不泄露存在性） |

配套：scope=task.submit/task.read + resource_id 绑定；状态按 subject 分桶；无注册表且未开 INSECURE 时拒绝启动（退出 2）；INSECURE 只允许绑 loopback。

## 6. SDK 与跨架构交付

- 交付分级：L1 纯 Python + 预编译 aarch64 wheel（可执行，本轮承诺）；L2 OCI 镜像（不可执行）；L3 板级原生包（需板厂）。
- 三产物：SDK wheel / runtime bundle / board bundle；离线优先（PyPI 不可达）；profile-check 失败即不启动（fail-closed）；install 幂等；uninstall 只回滚本 bundle。
- 构建目标单一：`aarch64-manylinux_2_28-cp311`（openEuler 24.03 / Python 3.11 / glibc 2.38）。

## 7. 能力与技能模型

- 16 个 Skill（move_joint/pick_object/place_object/visual_pick/accept_payload/dock_for_handoff/stand/stop/locomote/reset_scene + calibrate_grasp/calibrate_camera_to_base/display_pick + engineering.robot_adapter/engineering.inspect_robot）。
- 17 项能力词表（CAPABILITY_METHODS），能力名 → Backend 同名方法；`declared ⊆ implemented` 装配期校验。
- 12 个运动 skill 的 providers 支持 `selector:{mode:simulation|hardware}` 双条目（profile.simulation=false 时路由 hardware，后端由 factory backend_entrypoint 决定 RemoteProviderBackend/x86 SDK 或本地 MuJoCo）。

## 8. VLA 训练与边缘推理设计

- 策略 Provider（classical/vla_guided/learned_policy/teleop/replay）× 执行后端（mujoco/hardware/hil）正交。
- VLA/学习策略只输出 `ActionProposal`（目标物体/末端位姿/夹爪动作/短时动作块/置信度），不得输出 CAN 帧/电机电流/绕过安全投影的关节命令；最终轨迹插值/限位/碰撞/停止由确定性执行器负责。
- 数据闭环：传统控制成功轨迹 → VLA/策略训练数据（脱敏、版本化、区分仿真/真机）；ReplayManifest 冻结场景+模型+Profile+输入摘要+事件序列。
- 模型 Provider 统一 `Generate`/`Embed`/`Infer` 三异步接口，只用于任务生成/语义感知/技能选择/失败重规划；远程超时即明确错误，不得阻塞 stop/局部控制/SafetyEvent。
- 落地状态：仅北向意图解析（`openai_intent.py`）已实现；VLA/学习策略训练、南向 Infer 三接口、智能域远端 Provider 后端仍是设计图纸、代码空白（首期 M4 只要求 classical@mujoco 全链 + vla_guided@mujoco 受约束 ActionProposal 演示）。

## 9. 部署拓扑与板卡

- 板卡 profile：e300 / firefly_rk3588 / topeet_rk3588 / intewell_aros_lab。
- Hyper 构型：MQ50-E300-3VM-LPR / MQ50-E300-3VM-LPP / FIREFLY-RK3588-2VM-LR；2VM 合并不合并安全边界；S600 未有 release profile 前禁止标记 S2。
- 仿真只部署 x86（install_profile_check 拒绝 adapters_sim + arch != x86_64）。

## 10. 显示契约（AGENTS §6.12）

每个新机器人适配器必须实现 `render_frames`/`display_lock`/`render_settings`（与 MujocoBackend/UnitreeGo2Adapter 同契约），离屏渲染需 `MUJOCO_GL=egl`（import mujoco 前设置），禁止 headless。
