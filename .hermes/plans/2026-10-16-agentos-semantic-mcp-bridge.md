# 计划（修正版）：板上 AgentOS 语义 → 板上 IRAF TaskFlow → x86 只做 MuJoCo 显示

日期：2026-10-16　分支：main　授权：用户「基于这个开始执行」

> 本文件**取代**同日早先那版（MCP server 放 x86、板卡只当语义前端）。修正理由见 §1，用户明确纠正。

## 1. 口径（用户原话，2026-10-16）

> 「板上我有 agentos cli 应该是板上的 cli 发出语义，然后在板上的 iraf 部分发起 taskflow，x86 端的只是最后的 mujoco 显示」

⇒ **被否决的旧方案**：把 IRAF 执行放 x86、板卡只发 MCP 请求。
⇒ **正确分工**：

```
[板卡 aarch64]  AgentOS CLI（语义）
                  └─► 板上 IRAF：TaskFlow → Skill Runtime → PolicyGateway → 审计/事件
                        └─► Capability Provider  = 远端仿真后端（新）
                              │ 跨机（gRPC / 版本化公共服务）
                              ▼
[x86 本机]  仿真 Capability Provider 服务（MuJoCo 物理 + 感知 + 渲染）
                              └─► MuJoCo 窗口出现在本机桌面（DISPLAY=:0）
```

### 1.1 必须讲清的物理事实（影响接口切面）

"x86 只做显示"的**字面**读法（板上传关节角、x86 只回放）不成立：MuJoCo 引擎、
`target_detection.py`、`pose_estimation.py` 全在 x86 —— 板上不装仿真（用户 2026-10-15 口径）。
因此 x86 端的角色是**仿真 Capability Provider**：不参与策略、决策、审计（那些在板上），
只把板上发来的**能力调用**在 MuJoCo 里执行出来并渲染给人看。

⇒ 接口切面 = **capability 级**（`move_joint` / `pick_object` / `place_object` / `stop` / `step` …），
与 `src/iraf_adapters/backend.py: RobotBackend` Protocol 的方法集一致（`factory.verify_backend_contract` 用的就是它）。
⇒ `pick_object(target_id, grasp_pose, duration_ms, lease)`：视觉与规划留在 x86（属仿真域能力），板上只发 target_id。

## 2. 事实基线（2026-10-16 只读实测）

| 事实 | 值 | 证据 |
|---|---|---|
| 板 → 本机 | 10.203.247.221 → 10.203.247.121，**0% 丢包，rtt 0.190 ms** | `/tmp/board_agentos_recon.sh` §F |
| 板 → 模型网关 | 192.168.100.89:3000 **REACHABLE** | 同上 §G |
| 板上 AgentOS | `intewell-agent-cli`（Hermes 系）：`mcp add <n> --url/--command`、`--auth header`、`skills`、`tools` | 同上 §A/§B/§C |
| 板上模型 | `model.default=deepseek-flash` / `provider=intewell` / `base_url=https://api.deepseek.com` | `/root/.intewell/config.yaml` |
| 板上 IRAF | **未安装**（无 `/opt/iraf-sdk`、无 `iraf_sdk`、无 unit） | `/tmp/iraf_board_status.sh` |
| 板上 MCP 注册 | 空 | 同上 |
| 现有 IDL | 只有**任务级** `SkillRuntimeService{Execute,Cancel,GetExecution}` + `EventService` ⇒ **无 Provider 级服务** | `api/proto/iraf/v1/{runtime,events}.proto` |
| 现有跨域范例 | `iraf_adapters/domain_stub.py`（控制域 Provider 桩，323 行）+ `ros2/command_backend.py`（44 行薄后端） | 仓库 |
| 缺失层 | **远端 Provider 后端（client 侧）** + 对应的 Provider 服务端 ⇒ 正是 `edge-intel.backend_entrypoint: unverified` 缺的那层 | 记忆/技能陷阱 19 |
| x86 能力实现 | `MujocoBackend`（5099 行）：`pick_object/visual_pick/place_object/move_joint/stop/step/start_continuous/stop_continuous/render_frame` 全在；`from_config(config, profile, authority)` 符合 factory 契约 | `src/iraf_adapters/mujoco/mujoco_backend.py` |
| x86 显示已验证 | `VIEWER_WINDOW_OPENED True / VIEWER_FRAMES 1678 / exit 0 / 墙钟 45 s` | `/tmp/piper-live-rerun.log` |

## 3. 分工与落点

| 组件 | 放哪 | 形态 |
|---|---|---|
| AgentOS CLI（语义理解） | 板卡 | 已有 `intewell-agent-cli` |
| TaskFlow / Skill Runtime / PolicyGateway / 事件审计 | **板卡** | 装 `edge-intel` 角色（`deploy/sdk/install-iraf.sh`） |
| Provider 级 IDL | 仓库 `api/proto/iraf/v1/provider.proto`（新增） | 先改 IDL（铁律 2.1） |
| 仿真 Capability Provider **服务端** | **x86** | 包 `MujocoBackend` + 主线程窗口渲染 |
| 远端 Provider **客户端后端** | 板卡（代码在仓库） | 实现 `RobotBackend` 同名方法，转发 RPC |
| MuJoCo 窗口 | **x86** | `DISPLAY=:0` + `XAUTHORITY` + `MUJOCO_GL=glfw` |

## 4. 分期与验收（每期给数字证据）

- **阶段 1｜本机自证（同机两进程）**
  产物：`api/proto/iraf/v1/provider.proto` + 桩；`src/iraf_adapters/remote/provider_service.py`（服务端）；
       `src/iraf_adapters/remote/provider_backend.py`（客户端后端）；`scripts/iraf_provider_serve.py`（入口）。
  验收：x86 起服务端（带窗口）→ 客户端进程调 `pick_object(box_01)` → **本机窗口出现并渲染**，
        返回证据（`bilateral_contact` / 力 / 抬升）；拒绝路径：未知能力 / 过期 fencing token / 超时。
- **阶段 2｜跨机（板卡装 IRAF + 连 x86）**
  验收：板卡 `install-iraf.sh --role edge-intel` exit 0 + 六路残留可清；板上 runtime 用远端 backend 跑通一次
        `pick_object`，x86 侧出窗；板上事件存储留有 execution 记录。
- **阶段 3｜板上 AgentOS 语义端到端**
  验收：板上 CLI 输入「启动piper机械臂进行抓取仿真」⇒ 板上 TaskFlow 发起 ⇒ x86 窗口出现；
        记录：板上会话日志（工具调用）+ 板上 execution_id + x86 窗口帧数/墙钟 + 截图。
- **阶段 4｜拒绝路径 + 文档**
  未知技能 / 越权 / 板→x86 断链 / 取消；`docs/` 中文操作手册 + 调试记录。

## 5. 边界（写死）

- 凭据只从环境变量来，不落盘、不进命令行、不进 git。
- 板上 `simulation_only: true`（板卡未签字）；一切仿真结论标 `simulation=true`（铁律 1.7）。
- 跨机能力调用必须带 **execution_id + 幂等键 + 租约/fencing token + deadline + 取消路径**（铁律 2.12）。
- x86 服务端**不做策略判定**（策略在板上）；但**拒绝过期 fencing token 与终态执行的控制请求**（铁律 2.12）。
- 网络故障只允许显式失败 / 安全停机，**禁止伪造成功**（铁律 1.5）。

## 7. 进度快照（2026-10-16，main 领先 origin/main 41 笔）

> **追加（同日，用户交办 5 项已全部完成）**
> ① 跨机安全事件传播 `222c3c7`/`08b7c86`/`2104ffa`（core 定义 `SafetyEventReported`、runtime 捕获后隔离本侧资源、
>    服务端状态判定 + 隔离锁存 + Invoke 前置拒绝、客户端转 core 异常；11 例用例）
> ② IDL 破坏性变更检查 `99c3c7b`（`scripts/idl_compat_check.py`，15 例；真实仓库 HEAD→0 项、对比 948c3fa→6 项兼容新增）
> ③ 铁律改 v1 仿真范围 `9ca8b21`/`4a7c1e4`（AGENTS.md §0.1 + 铁律 2.9/§4/6.8 + 设计文档同步；诚实性条款保留）
> ④ 跨机断链/取消实测 `bd39ea6`/`cc555ab`（断链：退出码 1 + `IRAF-TRANSPORT-FAILED` + 中文诊断、无 traceback 穿透；
>    取消：执行中 `accepted=true,state=7`、抓取仍完成 `grasped=True` 墙钟 10.1 s、终态后 `accepted=false`）
> ⑤ 记录债 `1351f16`（`.hermes/plans/` 入库，36 份）
> 最终全量基线：`FAILED=1`（仅已知环境差异 `test_target_verify_evidence`）、`ERROR=2`（既有 sys.path 问题），无本轮引入的失败。
> 本轮共逼出 4 个真问题（含我自己的遗漏：新增 `correlation_id` 读取破坏既有替身用例，`887f093` 修）。

| 阶段 | 状态 | 证据 / 产物 |
|---|---|---|
| 1 本机自证（两进程跨机） | **完成** | `PROVIDER_LISTEN 127.0.0.1:50054`；跨机 visual_pick 墙钟 6.2 s（带窗）/4.2 s；`bilateral_contact true`，力 0.253531/0.244902 N，`force_imbalance_ratio 1.035233`；窗口 `0x5c00007 "MuJoCo : piper_description" 1280x720`；录像 `/tmp/provider-pick.mp4`（60285 B） |
| 2 仓库侧（声明 + 依赖核对） | **完成** | `12d1f04`：edge-intel 增 `adapters_remote` + 回填 `RemoteProviderBackend`（`verification` 仍 `unverified`）；`install_profile_check` 66 项全过；wheelhouse 已含 `grpcio-1.84.0`/`protobuf-7.36.2`（aarch64 cp311，与本机版本一致） |
| 2 端到端预演（本机模拟板上） | **完成** | `状态 SUCCEEDED` / `execution b704263a-b862-5c83-93ea-9128ffd44c54` / `解析技能 visual_pick`；`--list-skills` 13 项 exit 0 |
| 2 板卡写入（装 edge-intel） | **待放行** | `install-iraf.sh --role edge-intel` |
| 3 板上 AgentOS 语义绑定 | **待定路线** | ① 板上 shell 工具直跑 `iraf-agentos --text "…"`（零额外依赖）② MCP stdio 通道（需把 `mcp` 加入 aarch64 wheelhouse）。已探明板上支持 `-z/--oneshot PROMPT` + `--yolo`；`tools`/`skills` 子命令需交互终端 |
| 4 拒绝路径 + 文档 | **部分完成** | 11 例拒绝路径单测（`d6842f4`）；调试记录 `docs/debug/2026-10-16-remote-provider-cross-machine.md`（`c5b925c`，8 条坑位） |

**本轮实测逼出并修掉的真缺陷**（都已提交，供后续避免重犯）：
1. `9229521` 运行时清单必须**跨机取回** —— `iraf_core/runtime.py:93` 用 `backend.runtime_inventory()`
   求值 skill preconditions（`safety.estop` / `manipulation.target_visible`）；用本地假清单放行 = 静默绕过前置条件。
2. `7314c5f`（阶段 0）live 显示路径未透传 `display_mode` ⇒ 窗口从未创建却报成功。
3. 服务端 SIGTERM 只置标志不停服 ⇒ `kill` 无效（已修，headless 分支同时 `server.stop()`）。

**已知环境差异（不是缺陷）**：用装了 `grpcio` 的解释器跑全量时，
`test_target_verify_evidence.py::test_health_stub_positive_control_marks_verified` 会失败 ——
该夹具用 HTTP stub 冒充 gRPC 目标，探针在有 grpc 时会走协议层判定并**正确地**报"协议层无响应"。
用系统 python3（无 grpc，降级走 TCP 判定）跑则通过；基线 `failures=0`、`errors=4`（4 个已知 proto 桩 loader error）。

## 8. 已知风险

1. **板上装 IRAF** 需依赖 wheelhouse（aarch64 cp311 已有 `build/wheelhouse/aarch64-manylinux_2_28-cp311`，18 wheel）。
2. **gRPC 依赖**：板上要有 `grpcio`；无则退化为 HTTP（`iraf_adapters/http/runtime_http.py` 已有 HTTP 服务端雏形）。
3. **窗口线程模型**：MuJoCo 窗口必须主线程 + GLFW；服务端用"主线程渲染循环 + 工作线程执行 capability + `display_lock`"（`run_request_live` 已验证该模型）。
4. **板卡地址会变**（实测 .74 → .221）⇒ 迁移期禁用 IP，全部从声明/环境取。
5. `.hermes/plans/` 被 `.gitignore` 忽略 ⇒ 本计划无版本控制（记录债）。
