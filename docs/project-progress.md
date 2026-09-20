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
- 第二阶段完成：场景加入重力和工作台；抓取过程先验证双侧接触，再激活受控搬运约束，
  执行抬升并校验目标高度增加。最新证据为目标抬升 `0.197722m`、`lifted=true`、
  `constraint_activated=true`。这证明了可审计的抓取搬运链路，不等价于真实硬件摩擦力
  和负载能力认证。
- Ubuntu 图形界面已实测可打开 MuJoCo Viewer。由于 Conda 自带 C++ ABI 与系统 Mesa
  驱动不兼容，启动时必须使用系统 `libstdc++`、Mesa DRI 路径和 `DISPLAY=:0`；已固化
  `scripts/view_piper_mujoco.py` 作为统一入口。
- Viewer 抓取演示已启用实时物理步进，默认动作时长调整为 12 秒，并支持
  `--pick-duration-ms` 调整，便于逐段观察张开、闭合和抬升过程。
- 接触力闭环已加入：Backend 通过 `mj_contactForce` 读取左右指法向力，并以最小法向力
  0.2N、最大受力不平衡比 4.0 作为抬升门禁。当前场景实测左右约 2.559N/0.156N、
  比值约 16.4:1，验收会 fail-closed；这暴露了真实的夹持力分配问题，后续需调整 IK
  接近位姿、指尖接触几何、摩擦参数和执行器力，而不是伪造成功。
- 参数扫描确认 0.030m 半边长方块可形成约 12.045N/12.134N 的平衡夹持（比值 1.007:1），
  默认开发验收目标已调整为该尺寸；小目标仍必须通过独立 IK/接触几何验收。

## 13. 2026-09-20 `iraf-24h` 24 小时战役：SDK 跨架构交付与宇树场景/技能

> 本节只写事实与实测数字；完整汇总见 `docs/progress/2026-09-20-sdk-unitree-24h.md`，逐步台账见 `plans/iraf-24h/00-STATUS.json`。

- 范围调整为 **x86-first**：边缘板卡不在场（`<边缘板卡A>` / `<边缘板卡B>` 的 22 与 9119 端口实测不通），
  目标端验收统一标记 `DEFERRED`（延后，不是失败）；**未**用 dry-run、mock 或历史数据冒充目标端证据。
- 20 步中 19 步完成（01–19），收尾步骤 20 产出本节与汇总文档；无 `BLOCKED` 步骤。
- SDK 层：新增纯 Python `iraf_sdk`（零第三方依赖，子进程导入纯净性证明不 import `iraf_core`/`iraf_adapters`），
  错误码映射 19 项（IDL §5 表 9 / `src` 抛出点 15 / 仅实现 9，逐项标 `provenance`）。
- 打包与安装：`build_sdk.sh` 产出 wheel + runtime bundle，`manifest` 逐文件条目 112，两次构建 SHA-256 **逐位相同**；
  板级 bundle 当前 **47 478 361 字节 / 26 成员 / sha256 `43f61c48…`**；`install.sh` / `verify.sh` / `uninstall.sh` / `deploy.sh`
  均有 `--dry-run` 与负向退出码（篡改 → 4、缺声明 → 2、用法 → 1），`uninstall` 依据安装记录删除
  （`file_count_record=161 == file_count_actual=161`）。
- 离线 wheelhouse：真实抓取 **7 个 wheel / 47 620 905 字节**，全部通过 `cp310 + manylinux_2_28_aarch64` 标签校验；
  已知缺口：传递依赖 10 条仅 1 条覆盖（9 条缺失），当前 wheelhouse 不足以支撑离线安装，需决策后回填矩阵。
- 宇树线：厂商 `unitree_mujoco` 资产按 commit `1eb6642e…` 锁定（22 件 / 29 091 323 字节，逐件 SHA-256 + git blob SHA-1 交叉校验）；
  三模型编译实测 `ncam=0` ⇒ **厂商 MJCF 不含相机/雷达，传感器必须由场景构建器按声明注入**。
- 场景包 `scenes/handoff_lab/` + 构建器生成模型实测 `nq 26 / nv 24 / nu 12 / ncam 1 / nsite 3 / nbody 20 / njnt 14`；
  传感器验收 15 条判据全过：`camera.fovy_rel_error=0.010427987255182231`、`lidar.points=167`、
  `lidar.miss_fraction=0.5361111111111111`、`imu.acc.rel_error=2.0889954113422363e-16`，台面命中距离与声明解析求交偏差 **0.0**。
- Go2 loopback（`simulation: true`）：`height_mean_m=0.2801007638069918`、`height_std_m=7.351396160228674e-05`、
  `hold_seconds=7.499999999999341`、`max_attitude_error_deg=0.05779130222480239`、`ctrl_saturated_samples=0`；
  适配器 stand 1.000 s → 基座高度 `+0.010216000 m`，力矩上限独立复算 = 模型 `ctrlrange`（hip/thigh ±23.7、calf ±45.43 N·m）。
- 技能层：`stand` 墙钟 `0.39167014486156404 s`、`stop` 墙钟 `0.5178679858800024 s` 均 `SUCCEEDED`；
  7/7 拒绝用例命中预期错误码（未认证、死信超时、能力缺失、越界速度、越出限位、幂等冲突、策略拒绝）。
  `locomote` **不声明能力**（首期无步态控制器），其拒绝路径即能力门禁证据。
- S2 脚本化入口 `scripts/scenario.py`：`stand_stop` 场景 `passed=true`，stand 墙钟 `0.3899381598457694 s` /
  仿真时间推进 `7.999999999999341 s` / 末速 `3.0387117402068175e-05 m/s`；故障注入 `injected=true / fake_success=false / verified=true`。
- 图与文档：新增 `docs/diagrams/iraf-sdk-delivery.{dot,svg}`（34 911 字节）与 `iraf-unitree-scene-stack.{dot,svg}`
  （43 683 字节），`.dot` 重新生成与入库 SVG **逐字节一致**；ADR-0006（跨架构交付分级 L1/L2/L3）与 ADR-0007（宇树场景交互）已落地。
- 回归：全量单测 `263 → 794`（新增 **531** 例），`failures=1 / errors=4 / skipped=4` 与战役基线**逐项一致**；
  4 项导入 `ERROR` + `test_vision_processing` 1 项 `FAIL` 为战役前既有缺陷，未并入基线、未放宽门禁掩盖。
- 未实现/未验证（不表述为已支持）：人形运动能力（仅静态模型）、语音/Studio、S1 命令式与 S3 遥操作、
  目标端真实安装与 `/health`、AgentOS 真机联通、OCI 多架构路线（无 buildx 且镜像源不可达）、板级原生包（无 aarch64 交叉工具链）。
