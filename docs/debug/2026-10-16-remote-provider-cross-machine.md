# 远端 Capability Provider（跨机）实测与坑位 — 2026-10-16

目标口径（用户当日明确）：
**板卡 AgentOS 发语义 → 板上的 IRAF 发起 TaskFlow → x86 只做最后的 MuJoCo 显示。**

```
[板卡 aarch64] AgentOS CLI（语义）
                 └─► IRAF Runtime：TaskFlow → Skill Runtime → PolicyGateway → 审计
                       └─► RemoteProviderBackend（本文件对应实现）
                             │  gRPC: CapabilityProviderService（版本化公共服务）
                             ▼
[x86 仿真主机] iraf_provider_serve.py ──► MujocoBackend（物理 + 感知） ──► MuJoCo 窗口（本机桌面）
```

分层纪律：`SkillRuntimeService`（任务级，服务端自己跑策略链）**≠**
`CapabilityProviderService`（能力级，策略在调用方）。两者都属版本化公共服务，混用即旁路。

## 1. 本轮落地的文件

| 文件 | 作用 |
|---|---|
| `api/proto/iraf/v1/provider.proto` | 能力级跨机契约（Health / Invoke / Cancel + `LeaseEnvelope` + `inventory`） |
| `src/iraf_adapters/remote/provider_service.py` | x86 侧 servicer + `ExecutionRegistry`（租约/fencing/幂等/终态拒绝） |
| `src/iraf_adapters/remote/provider_backend.py` | 板上侧后端（实现 `RobotBackend` 同名方法，转发 RPC） |
| `scripts/iraf_provider_serve.py` | 服务端入口（fail-closed token 校验 + 主线程窗口） |
| `tests/unit/test_remote_provider_service.py` | 11 例拒绝路径与租约语义（纯 stdlib，不依赖桩） |

装配（板上）：`IRAF_BACKEND_ENTRYPOINT=iraf_adapters.remote.provider_backend:RemoteProviderBackend`，
`IRAF_BACKEND_CONFIG={"endpoint":"<x86>:50052","token_env":"IRAF_PROVIDER_TOKEN","timeout_s":180}`。
token 只从环境变量来；无 token 时服务端**拒绝启动**（fail-closed）。

## 2. 实测数字（本机两进程跨机预演，2026-10-16）

- 服务端：`PROVIDER_LISTEN 127.0.0.1:50054` ｜ `PROVIDER_CAPABILITIES 5 move_joint,pick_object,place_object,stop,visual_pick`
- 显示：`PROVIDER_DISPLAY_MODE interactive_viewer` ｜ `VIEWER_GL software_llvmpipe window=1280x720`
  窗口：`0x5c00007 "MuJoCo : piper_description" ("MuJoCo" "MuJoCo") 1280x720 @ +5+355`
- 跨机 `visual_pick`：墙钟 **6.2 s**（带窗）/ **4.2 s**（无窗）
  `provider_execution_id exec-d3f10a7a877648b0afc34a4af9871d59`（**服务端生成**）
  `bilateral_contact true` ｜ 法向力 0.253531 / 0.244902 N ｜ `force_imbalance_ratio 1.035233` ｜ `lift_peak_step 1667`
- 完整返回键：`['confirmation','evidence','grasped','provider_execution_id','target_id']`，`grasped=True`
  ⇒ skill 层判据（`VisualPickProvider` 要求 `grasped is True`）**PASS**
- 端到端预演（本机模拟板上 + 桩意图模型）：
  `状态 SUCCEEDED` ｜ `execution b704263a-b862-5c83-93ea-9128ffd44c54` ｜ `解析技能 visual_pick`
- 画面在变（x11grab 抽帧）：mean 53.66 → 53.65 → **54.63**，唯一色 7460 → 7453 → **9292**

## 3. 坑位（每条都是实测踩出来的）

1. **`.local` 里的旧 protobuf 会把服务端带崩**。用户级 site-packages（`~/.local/lib/python3.10/site-packages`）
   的 protobuf 版本旧，报 `cannot import name 'runtime_version' from 'google.protobuf'`。
   注意：把 `.local` 放进 `PYTHONPATH` **也**不行 —— `PYTHONPATH` 条目排在 venv 的 site-packages **之前**。
   正解：在 venv 的 site-packages 放一个 `.pth`（追加到 `sys.path` 末尾，让 venv 版本优先），
   `PYTHONPATH` 只留 `src:build/generated/python`。
2. **venv 需要三样**：`grpcio`（跨机）、`grpcio-tools`（生成桩）、`pyyaml`（`iraf_core.profile` 依赖）。
   `mujoco` 不必装进 venv（用 `.pth` 从 `.local` 取）。
3. **`import` 期不能拉桩**。桩是生成物（`build/generated/python`），服务端把桩**注入** servicer
   （`build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2)`），核心模块顶层不 import 桩，
   否则"没跑过 gen_proto_stubs"就会让整个包 import 失败。
4. **SIGTERM 必须同时停服**。只置渲染循环标志不够：headless 分支阻塞在 `wait_for_termination()`，
   表现为 `kill` 无效、只能 `kill -9`。修法：handler 里 `stop_loop.set()` **且** `server.stop(grace=1)`。
   实测修后打印 `PROVIDER_STOPPED` 正常退出。
5. **运行时清单必须跨机取回**（本轮最重要的一条）。`iraf_core/runtime.py:93` 拿
   `backend.runtime_inventory()` 求值 skill preconditions（`safety.estop == false`、
   `manipulation.target_visible == true`）。客户端若返回本地静态说明 ⇒ 直接
   `IRAF-PRECONDITION-FAILED: 缺少运行时状态: safety.estop`。
   修法：Health 增 `Struct inventory`（远端真实清单）。**取不到就显式失败** ——
   用假清单放行等于静默绕过前置条件。
6. **MuJoCo 场景是有状态的**。常驻服务端抓过一次目标就被移走，第二次调同一目标必然
   `抓取位姿与目标位置不一致: distance=0.092186m tolerance=0.005000m`（这是**正确**的 fail-closed，
   不是缺陷）。复跑要么换新实例/新端口，要么后续补"场景复位"能力。
7. **诊断脚本别截断输出**。`json.dumps(...)[:2000]` 曾让我误判"跨机返回丢了 grasped"，
   实际是截断。核对契约完整性要 `sorted(resp.outputs.keys())`，不要看被截断的 JSON 片段。
8. **`pkill -f <脚本名>` 会杀掉自己**：执行该命令的 shell 命令行里含同样字符串 ⇒ 自匹配（实测 exit -15）。
   用 `pgrep -f "[s]cripts/xxx.py"` 的字符类技巧，或按 PID kill。

## 4. 复现命令

```bash
cd /home/coretek/AIIRAF
# 0) 桩（生成物）与 venv
PYTHON=build/mcp-venv/bin/python bash scripts/gen_proto_stubs.sh
# 1) 服务端（带窗口；token 只从环境来）
IRAF_PROVIDER_PORT=50054 IRAF_PROVIDER_TOKEN=smoke-token bash /tmp/iraf_provider_up.sh --window
# 2) 板上式装配的端到端（本机替身）
IRAF_BOARD_PROVIDER_ENDPOINT=127.0.0.1:50054 bash /tmp/iraf_board_sim.sh --text "视觉抓取方块"
# 3) 拒绝路径单测
PYTHONPATH=src build/mcp-venv/bin/python -m pytest tests/unit/test_remote_provider_service.py -q
```

> 边界：x86 侧一律 `simulation=true`；板上 `simulation_only: true`（板卡未签字，AGENTS 1.7）。
> 本记录只证"本机两进程跨机可用 + 板上式装配可跑通"，**不构成真机/板卡能力证据**。
