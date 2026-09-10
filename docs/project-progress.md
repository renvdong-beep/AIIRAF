# IRAF 项目进度基线：2026-09-01 至 2026-09-03

> 本文是本阶段跨机器协作的唯一进度摘要。后续 AI、开发机、Runtime 主机或边缘板卡接手时，应先阅读本文，再查看 `docs/debug/` 下的专项记录。

## 1. 机器与运行边界

| 角色 | 主机 | 目录/服务 | 当前职责 |
|---|---|---|---|
| 本机开发参考 | 当前工作区 | `/home/nando/AI-APPLICATION-FRAMEWORK` | IRAF 框架结构、契约、测试和文档基线 |
| Runtime 仿真主机 | `10.203.247.145`，用户 `coretek` | `/home/coretek/AIIRAF`，`iraf-runtime.service`，HTTP `127.0.0.1:8765` | AgentOS/HTTP Runtime、SkillRuntime、策略、安全审计和 Piper MuJoCo |
| 边缘 AI 板卡 | `10.203.247.86`，用户 `root` | `qwen-vllm.service`，HTTPS `:9119/v1` | Qwen3-0.6B 意图解析，不拥有运动或安全最终权限 |

运行边界遵循 IRAF：AI/应用层只能提交意图和技能；Linux-RT/控制层负责规划控制；RTOS/现场总线侧保留最终安全和执行权限。当前远端链路是 `development simulation`，不能视为真实硬件闭环。

## 2. 2026-09-01 完成的架构与基础能力

### 2.1 IRAF 分层目录和兼容迁移

- 建立 canonical 包：`src/iraf_core`、`src/iraf_adapters`、`src/iraf_skills`、`src/iraf_tools`。
- 适配器按职责拆分为 AgentOS、gRPC、HTTP、模型、MuJoCo、ROS 2。
- 将 Robot Profile、Skill、Safety Policy、消息和执行边界纳入框架层。
- 原有 `adapters/`、`skills/common/`、`tools/` 兼容入口保留，没有删除旧兼容代码，避免现有脚本和部署立即失效。
- 更新 `pyproject.toml`、部署配置、README、gRPC 开发文档和框架对齐状态。

### 2.2 Runtime、权限和执行一致性

- 增加 Runtime authority 的租约 TTL、fencing 和过期校验。
- 增加数字化 SemVer 技能版本约束。
- 保持执行幂等、取消和 stop 的安全收敛行为。
- AgentOS 意图先经过 Profile/Skill/Policy 校验，再进入 Backend，禁止绕过安全桥直接驱动仿真或设备。

### 2.3 持久化事件和安全隔离

- SQLite 执行存储增加 `executions`、`execution_events`、`safety_events`、`safety_event_history`。
- `SafetyQuarantine` 支持跨 Runtime 实例读取同一安全事件，恢复必须提供事件 ID、控制器确认和操作主体。
- 新增事件 proto、生成代码和 canonical gRPC `EventService.ListEvents`，支持执行、资源、时间范围和分页查询。
- 恢复历史保留，便于事后审计和故障分析。

### 2.4 AgentOS/Qwen 意图桥接

- AgentOS Bridge 根据当前 Robot Profile 注入显式关节集合和参数契约，避免模型输出非法字段。
- OpenAI-compatible provider 增加提示词约束和语义校验。
- 修复 Qwen 曾输出 `{"positions":{"positions":0.2}}` 的结构错误，当前可解析为合法 `move_joint` 意图。
- 边缘板 `qwen-vllm.service` 从异常 watchdog 状态恢复，当前提供 Qwen3-0.6B 的 HTTPS OpenAI-compatible 接口。
- 服务增加 `Restart=always`、重启间隔和启动限流配置，并保留修改前备份。

## 3. 2026-09-02 完成的最小链路与 MuJoCo 生命周期

### 3.1 最小动作链路

已验证以下两条链路：

```text
直接任务：HTTP /v1/tasks
  -> TaskDispatcher
  -> SkillRuntime
  -> Policy/Authority/Store
  -> common_motion_sim
  -> Piper MuJoCo

AgentOS 意图：HTTP /v1/intents
  -> Qwen3-0.6B
  -> AgentOSBridge
  -> 同一 TaskDispatcher 和 Runtime 链路
  -> Piper MuJoCo
```

验证脚本：`scripts/verify_minimal_chain.py --mode direct|intent|both`。最终 `--mode both` 中 direct task 和 AgentOS intent 均为 `SUCCEEDED`。

### 3.2 连续 MuJoCo 仿真

此前动作只在请求期间步进，不能表达长期运行的仿真服务。本次改为框架生命周期管理：

- 新增 `src/iraf_adapters/mujoco/supervisor.py` 的 `MujocoSimulationSupervisor`。
- `MujocoBackend` 增加单一后台步进线程，按 MuJoCo `model.opt.timestep` 执行连续 `mj_step`。
- HTTP Runtime 和 gRPC Runtime 启动时启动 Supervisor，退出时停止并等待线程收敛。
- `move_joint` 在连续模式下只设置控制目标并等待动作时长，不再创建第二套物理循环。
- `stop` 保留取消事件、控制量归零和安全停止行为。
- `/health` 增加 `continuous_simulation: true`，可由部署和监控确认连续仿真是否生效。
- `scripts/render_piper_mujoco.py` 通过 Runtime、SkillRuntime、Backend 和 Supervisor 工作；新增 `--forever` 生成持续更新的 `piper-live.png`，没有复制独立动作控制逻辑。

### 3.3 Piper 画面验证

framework-owned 渲染入口已生成：

- `piper-initial.png`
- `piper-final.png`
- `piper-motion.gif`
- `render-report.json`，schema 为 `iraf.piper-mujoco-view/v3`，记录 execution、profile、policy、provider 和 Supervisor 状态。

Ubuntu 系统 MuJoCo/EGL 渲染已成功。远端 Conda MuJoCo 绑定因 EGL 驱动不支持 `EGL_PLATFORM_DEVICE` 无法创建无头渲染上下文，但这不影响 Runtime 的无窗口物理步进。交互式 Viewer 仍需要 X11、VNC 或桌面会话。

### 3.4 2026-09-03 仿真诊断与执行回放

- MuJoCo 连续循环增加目标/有效步频、步进耗时、超限、失败和最近错误指标；`/health` 在步进失败后返回降级状态。
- 故障注入仅允许在显式启用的独立仿真实例中使用；步进失败后控制量归零并锁存，必须显式清除才能重启。
- EventService 增加确定性 `ReplayManifest`，冻结请求、结果、Profile、Policy、Skill、Provider 和事件摘要，不保存原始凭据与 Skill 输出。
- AgentOS Provider 失败持久化为 `PENDING -> VALIDATING -> FAILED`，不触发动作后端；成功意图以只追加方式记录模型身份、意图摘要和 resolved Skill。
- 一键证据包同时验证 direct、在线 Qwen intent、故障回放、两条成功回放和 Runtime 健康状态。

## 4. 当前验证证据

- Runtime 服务：`iraf-runtime.service` 为 `active`。
- 服务重启次数：`NRestarts=0`。
- 健康检查：`continuous_simulation=true`、`intent_enabled=true`。
- 单元测试：`44 tests`，`unittest discover` 全部通过。
- 最小链路：`/v1/tasks` 和 `/v1/intents` 均 `SUCCEEDED`。
- AgentOS 模型：`Qwen3-0.6B`。
- 一键证据：九项门禁全部通过，manifest 和两份成功回放均提供 SHA-256 校验。
- 仿真循环：目标 500 Hz，验收与长期服务中的有效步频稳定在约 460 Hz。
- Piper 渲染：Supervisor 模式执行成功并产出 PNG/GIF。
- 长期模式：`--forever` 已进入 `RUNNING`，可由 Ctrl-C 停止。

## 5. 关键代码入口

- Runtime 核心：`src/iraf_core/runtime.py`
- 安全隔离：`src/iraf_core/safety.py`
- 执行和事件存储：`src/iraf_core/store.py`
- AgentOS 桥：`src/iraf_adapters/agentos/bridge.py`
- MuJoCo Backend：`src/iraf_adapters/mujoco/mujoco_backend.py`
- MuJoCo Supervisor：`src/iraf_adapters/mujoco/supervisor.py`
- HTTP Runtime：`src/iraf_adapters/http/runtime_http.py`
- gRPC Runtime：`src/iraf_adapters/grpc/server.py`
- 最小链路验证：`scripts/verify_minimal_chain.py`
- 一键仿真证据：`scripts/verify_development_simulation.py`
- 意图失败回放：`scripts/verify_intent_failure_replay.py`
- 执行回放导出：`scripts/export_replay_manifest.py`
- Piper 画面验证：`scripts/render_piper_mujoco.py`

## 6. 已知限制与未完成项

1. 当前仍是 development simulation，真实 ROS 2、Hyper IPC、RTOS/现场总线和硬件在环尚未接入。
2. Profile 签名、生产 mTLS、RTOS SafetyEvent 回读和真实设备负向测试仍需补齐。
3. gRPC 当前为开发仿真入口，生产网络暴露和认证策略需按部署环境继续收敛。
4. EGL/Viewer 的图形环境需要单独部署；无头 Runtime 物理循环不依赖窗口。
5. 旧兼容层暂不删除，待所有调用方迁移、兼容测试和发布分支确认后再制定下线计划。

## 7. 下一阶段优先级

1. 将当前 Piper MuJoCo Backend 与真实 ROS 2/Linux-RT 控制接口建立同一 Skill/Policy 契约。
2. 接入 RTOS motion permit、heartbeat、fieldbus authority 和 fail-closed 负向测试。
3. 增加生产部署的 mTLS、Profile 签名校验、事件远端回读和 HIL 验证证据。
4. 将当前 JSON 仿真指标接入统一 Observability/Prometheus，并定义告警阈值与 retention。
5. 在所有调用方完成迁移后，再评估删除旧兼容入口。

## 8. 2026-09-03 微信日报摘要

1. 完成 MuJoCo 连续步进指标、健康降级、受控故障注入和失败后归零锁存。
2. 完成 execution replay manifest、导出校验和 gRPC 查询契约。
3. 完成 AgentOS Provider 失败的无动作回放，以及成功意图的模型与 Skill 身份固化。
4. 将 direct 与在线 Qwen intent 成功链路及两份 replay 纳入一键仿真证据包。
5. 完成 44 项测试和九项开发仿真门禁；边界仍明确为 `simulation_only=true`。

## 9. 2026-09-10 进度更新

- `codex/replay-evidence-tests-001` 当前提交为 `e392737`，工作区干净。
- 回放清单已加强嵌套身份字段白名单和执行元数据校验，避免凭据、token、key 等字段进入证据。
- ROS 2 命令/状态边界已补充输入校验：租约先校验，命令拒绝未知/非有限关节值和非正时长，状态拒绝重复/未知/非有限数据。
- 单元测试已由 44 项增至 68 项，`unittest discover` 全部通过。
- 一键开发仿真在当前网络条件下 direct、故障回放、执行回放和 Runtime 健康检查通过；在线 Qwen 意图链路因到 `10.203.247.86:9119` 的 `No route to host` 失败，不能标记为全门禁通过。
- GitHub 推送仍受运行环境 DNS/SSH 出站限制影响，待网络恢复后执行 `git push -u origin codex/replay-evidence-tests-001`。

## 10. 2026-09-10 控制门进展

- 新增 `RtosMotionGate`：运动前必须具备新鲜单调 heartbeat、短期 motion permit 和匹配 fencing token。
- 心跳超时、序号回退/重复、permit 字段异常、资源或 token 不匹配、permit 过期、Provider 链路异常均 fail-closed。
- 新增 4 项负向/成功回归测试；全量单元测试增至 72 项并全部通过。
- 当前控制门仍是平台无关核心契约，下一步接入 ROS 2/Linux-RT 适配器；`stop` 路径保持独立，确保异常时仍可安全停机。

## 11. 2026-09-10 `pick_object` 第一批实现

- 新增 `pick_object` 输入输出 Schema 和仿真 Provider 契约，开始从关节运动推进到抓取能力。
- 抓取成功必须由 Backend 返回目标一致的接触、约束或夹爪状态确认；不得仅因命令已发送而返回成功。
- 目标不可见、能力缺失、后端未确认、非有限位姿或非归一化四元数均 fail-closed。
- 正式 Piper Profile 暂不声明 `pick_object`，需要真实 MJCF 目标、夹爪接触确认和仿真证据完成后再启用。
- 远端固定 Conda 环境全量单元测试由 72 项增至 77 项，全部通过。

## 12. 2026-09-10 Piper MuJoCo 双指接触抓取

- 定位到运行服务实际使用的 AgileX Piper MJCF；模型包含 `joint7`、`joint8` 双指、
  对应 mesh 和位置执行器，不需要重新猜测夹爪结构。
- 修正 Piper Profile 的夹爪行程，并在真实双指 contact 验收后开放开发仿真的
  `pick_object` 能力和 SafetyPolicy 准入。
- 新增可复跑场景生成器，在 `build/` 内生成带 `box_01` 基础几何体的 MJCF，保持
  Piper 原模型和安装目录只读。
- Backend 抓取成功必须同时观察到目标与左右手指接触，并校验目标 ID、world 位姿和
  容差；不能用“控制命令已发出”代替抓取成功。
- 当前证据范围是零重力接触夹取，不包含升举和搬运；下一步增加重力、工作台、IK 接近、
  抬升后目标位移与持续接触验收。
- 远端全量单元测试增至 80 项并全部通过；真实 Piper MJCF 完整 Runtime 验收为
  `SUCCEEDED`，证据报告位于 `build/acceptance/piper-pick/report.json`。
