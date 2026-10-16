# IRAF × AgentOS 交互式 CLI 使用说明

> 适用版本：`main` ≥（本次提交）
> 一句话：**在 AgentOS 的 CLI 场景里，用一句自然语言驱动 IRAF 的 Skill，链路不绕层、失败显式。**

---

## 1. 它是什么（与 HTTP/gRPC 服务端**同一层**）

```
自然语言 ─► AgentOSBridge.execute_intent()
              ├─► IntentProvider（受约束解码到 skill schema，不是自由文本）
              └─► TaskDispatcher ─► SkillRuntime.execute()
                     ├─► PolicyGateway（rbac/schema/profile/safety/capability/precondition）
                     └─► Capability Provider（控制域）
```

> ⚠ **硬边界（AGENTS 铁律 1）**：LLM/CLI **不得**直接下发电机/关节/力矩/总线命令。
> 本 CLI 只做「取文本 → 建 canonical 请求 → 交给 Bridge → 如实渲染结果」，
> **不驱动后端、不写判据、不绕过策略**。

| 相关文件 | 职责 |
|---|---|
| `src/iraf_adapters/agentos/cli.py` | CLI 核心（请求构建 / 身份 / 渲染 / 退出码） |
| `scripts/agentos_cli.py` | 仓库根入口（把 `src/` 挂到 path 后转发） |
| `pyproject.toml` | 注册 `iraf-agentos` 命令（安装后可直接敲） |

---

## 2. 先装配 Runtime 环境（缺项即显式失败，不猜）

```bash
export PYTHONPATH=src

# —— Execution 侧（6 项必需）——
export IRAF_PROFILE=/opt/iraf/profiles/unitree_go2_mujoco.yaml
export IRAF_SAFETY_POLICY=/opt/iraf/profiles/safety/quadruped_lab.yaml
export IRAF_SKILL_ROOT=/opt/iraf/skills
export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.unitree.unitree_go2:UnitreeGo2Adapter
# ⚠ 四足适配器要「JSON 对象里带 declaration（机型声明路径）」；臂侧则是 model_path 的 JSON，
#   形状随 Profile 声明走 —— 手写副本会静默过期（见 docs/debug/2026-09-20-*）。
export IRAF_BACKEND_CONFIG='{"declaration":"config/go2_loopback.yaml"}'
export IRAF_EVENT_STORE=/var/lib/iraf/iraf-runtime.db

# —— 身份 ——
export IRAF_RUNTIME_SUBJECT=agentos-demo        # 或用 --subject 显式给

# —— 意图模型（OpenAI 兼容端点；两键齐全才启用）——
export IRAF_AGENTOS_MODEL_URL=https://<边缘模型>/v1
export IRAF_AGENTOS_MODEL=<model-id>
export IRAF_AGENTOS_TOKEN=<部署下发凭据>          # 可选
export IRAF_AGENTOS_VERIFY_TLS=true              # 可选（缺省 true）
export IRAF_AGENTOS_TIMEOUT_SECONDS=10           # 可选
```

> 生产请用 `/etc/iraf-runtime.env`（`chmod 600`，**不入库**）；`deploy/iraf-runtime.env` 是模板。

---

## 3. 三种用法

```bash
# ① 交互 REPL（人用）
iraf-agentos
#    或（未安装时，仓库根）
PYTHONPATH=src python3 scripts/agentos_cli.py

# ② 单次文本（脚本用）
iraf-agentos --text "让机器狗站起来"
iraf-agentos --text "停止机器人" --deadline-seconds 30

# ③ 机器模式（**AgentOS CLI 绑定用**）：stdin 收一条 JSON → stdout 出一条 JSON
echo '{"text":"四足站立保持","deadline_seconds":60}' | iraf-agentos --json
```

REPL 内建命令：`:skills`（列出可用 Skill）· `:whoami`（身份与 scope）· `:q`（退出）。

`--list-skills` 可非交互列出可用 Skill 与一句意图示例。

### 退出码

| 码 | 含义 |
|---|---|
| `0` | 执行成功（evidence 满足 skill 输出 schema） |
| `1` | 执行失败或策略拒绝（`IRAF-POLICY-DENIED` / `IRAF-EXECUTION-FAILED` …） |
| `2` | 用法/环境错误（缺 Runtime 环境、缺意图模型、参数非法） |
| `3` | 意图解析失败（`IRAF-INTENT-PARSE-FAILED`） |
| `4` | 意图 Provider 不可达 |

---

## 4. 与 AgentOS CLI 的绑定方式

IRAF 的 CLI 把自己暴露成**一个可组合的子进程**，AgentOS 侧只需：

**方式 A（推荐，最松耦合）—— 把 `iraf-agentos --json` 当工具调用**

```python
# AgentOS 侧（示意）：一次工具调用 = 起进程 + 喂一行 JSON + 收一行 JSON
import json, subprocess

def call_iraf(text, deadline_seconds=60):
    proc = subprocess.run(
        ["iraf-agentos", "--json"],
        input=json.dumps({"text": text, "deadline_seconds": deadline_seconds}) + "\n",
        text=True, capture_output=True, timeout=deadline_seconds + 30,
    )
    result = json.loads(proc.stdout.strip().splitlines()[-1])
    return proc.returncode, result          # 0=成功；1/3/4=显式失败（含 error_code）
```

约定（**最小、可组合**）：
- **入**：`{"text": "...", "deadline_seconds": 120, "resource_id": "...", "request_id": "..."}`（后两个可选）
- **出**：与 Bridge 返回值**同构**的 JSON，含 `status` / `error_code` / `execution_id` / `evidence`
- **失败不改变进程约定**：仍输出一行 JSON，只是退出码非 0 —— 让 AgentOS 能统一处理

**方式 B —— AgentOS 直接调 IRAF 运行时常驻服务**（不经本 CLI）

IRAF 已有常驻服务：`iraf-runtime-http`（:8765）/ `iraf-runtime-grpc`（:50051）。
AgentOS 侧把「意图 → IRAF 任务」接上即可（同一 `AgentOSBridge` 语义）。
契约见 `docs/iraf-agentos-hyper-compatibility.md` §2：AgentOS 能力 ↔ IRAF 四个公共服务
（TaskService / WorldModelService / SkillRuntimeService / EventService + 生命周期 + 身份委托 + 版本协商）。

> **建议**：开发/调试期用方式 A（一行命令、看得见证据）；生产期用方式 B（常驻、可观测、可审计）。
> 两者**走同一条 SkillRuntime 链**，不产生第二套语义。

---

## 5. 完整示例（本仓实测，可照抄）

下面这段在一台装了 Go2 仿真环境的机器上**真实跑通**（模型端点用本地 stub 代替）：

```bash
cd /home/coretek/AIIRAF
export PYTHONPATH=src
export IRAF_PROFILE=$PWD/profiles/unitree_go2_mujoco.yaml
export IRAF_SAFETY_POLICY=$PWD/profiles/safety/quadruped_lab.yaml
export IRAF_SKILL_ROOT=$PWD/skills
export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.unitree.unitree_go2:UnitreeGo2Adapter
export IRAF_BACKEND_CONFIG='{"declaration":"config/go2_loopback.yaml"}'
export IRAF_EVENT_STORE=/tmp/cli-demo.db
export IRAF_RUNTIME_SUBJECT=agentos-demo
export IRAF_AGENTOS_MODEL_URL=http://127.0.0.1:8077/v1
export IRAF_AGENTOS_MODEL=demo-model

python3 scripts/agentos_cli.py --text "让机器狗站起来"
```

**输出（实测）**：

```
状态      : SUCCEEDED
execution : c9b99283-7627-5edd-bed3-c503d74963a6
解析技能  : stand
意图模型  : demo-model (openai-compatible-intent)
```

**机器模式（实测，截取）**：

```bash
echo '{"text":"四足站立保持","deadline_seconds":60}' | python3 scripts/agentos_cli.py --json
```
```json
{"status":"SUCCEEDED","resolved_skill":"stand","intent_provider":{"model":"demo-model"},
 "result":{"accepted":true,"skill":"stand","evidence":{
   "simulation":true,"capability":"stand","duration_ms":3000.0,"control_cycles":300,
   "final_state":{"base_position_m":[-0.0070, -2.9e-08, 0.27999],
                  "joint_positions_rad":{"FL_thigh_joint":0.8974, "...":0.0}}}}}
```

**停机（实测）**：

```
$ python3 scripts/agentos_cli.py --text "停止机器人"
状态      : SUCCEEDED
解析技能  : stop
```

**拒绝路径（实测）——说不清就显式失败，不猜着执行**：

```
$ echo '{"text":"煮一杯咖啡并递给我","deadline_seconds":60}' | python3 scripts/agentos_cli.py --json
{"status":"FAILED","error_code":"IRAF-INTENT-PARSE-FAILED",
 "reason":"模型意图不符合 Skill schema: {'skill':'stand','parameters':{'nonsense':1}} is not valid under any of the given schemas"}
# exit=3
```

> 注：`--list-skills` 会列出当前 `IRAF_SKILL_ROOT` 下所有已登记的 Skill（示例机器上 13 个）。
> 若某句意图解析出的 skill **不在** `IRAF_SAFETY_POLICY.allowed_skills` 里，
> 会得到 `IRAF-POLICY-DENIED: SafetyPolicy 未允许该 Skill` —— 这是**正确的安全边界**，
> 换策略（如 `quadruped_lab` 允许 `stand/stop/locomote/dock_for_handoff/accept_payload`）即可。

---

## 6. 常见问题

| 现象 | 原因与处置 |
|---|---|
| `缺少 Runtime 环境：IRAF_*` | 上表 6 项没配齐；CLI **不给默认值**（避免猜路径） |
| `缺少意图模型：请设置 IRAF_AGENTOS_MODEL_URL 与 IRAF_AGENTOS_MODEL` | 未启用自然语言解析；CLI **不做"无校验直通"降级**（那会变成零校验后门） |
| `缺少调用方身份` | 给 `--subject` 或设 `IRAF_RUNTIME_SUBJECT` |
| `IRAF-POLICY-DENIED: SafetyPolicy 未允许该 Skill` | 换/改 `IRAF_SAFETY_POLICY` 的 `allowed_skills`（安全边界，不是 bug） |
| `IRAF-INTENT-PARSE-FAILED` | 该文本无法映射到任何已声明 Skill + 合法参数；换更明确的说法，或先 `--list-skills` |
| `DeclarationError: 适配器配置必须是声明路径…` | `IRAF_BACKEND_CONFIG` 形状与该机型不符（四足要 `declaration`；臂要 `model_path` 等） |
| 结果 JSON 里 `evidence` 为空且 `status=FAILED` | 失败步骤的 evidence 不进报告；用 `--json` 看 `error_code` 与 `reason` |

---

## 7. 相关文档

- `docs/skill-app-development-guide.md` §4 —— **用自然语言对话开发的协议**（6 项输入 / 6 项产物）
- `docs/iraf-agentos-hyper-compatibility.md` —— AgentOS 兼容契约与四个公共服务
- `docs/deploy-topology-and-domain-split.md` —— 控制域 / 智能域拓扑与跨域铁律
- `docs/agentos-robot-application-framework-design.md` —— 三域职责与 TaskFlow/Skill 设计

---

## 8. 语义 → 抓取显示（`scripts/agentos_view.py`，2026-10-16 新增）

**它解决什么**：`iraf-agentos` 是**无显示**的语义执行入口，`scripts/view_mujoco.py` 是**程序化**的显示入口；
"用一句 AgentOS 语义触发抓取并在 MuJoCo 里看见"需要把两者接起来。本入口就是这条薄桥（不旁路任何一层）：

```
自然语言 ─► IntentProvider（受约束解码到 Skill 契约）─► 校验+打印解析结果
          ─► scripts/view_mujoco.py（SkillRuntime → Policy/Authority → Provider(MuJoCo)，窗口先开、边抓边看）
```

```bash
export PYTHONPATH=src
export IRAF_PROFILE=profiles/piper_mujoco.yaml
export IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml
export IRAF_SKILL_ROOT=skills
export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend
export IRAF_BACKEND_CONFIG="$(PYTHONPATH=src python3 scripts/emit_backend_config.py \
    --scene build/models/piper-pick-scene.xml --compact)"
export IRAF_EVENT_STORE=/tmp/iraf-agentos-view.db
export IRAF_RUNTIME_SUBJECT=agentos-demo

# 意图模型（两键齐全才启用；缺了会显式失败，不做无校验直通）
export IRAF_AGENTOS_MODEL_URL=http://127.0.0.1:8077/v1     # 真端点换成你们的 OpenAI 兼容端点
export IRAF_AGENTOS_MODEL=stub-intent                      # 真模型名

python3 scripts/agentos_view.py --text "启动piper机械臂进行抓取仿真"
```

**本机实测（2026-10-16，Piper）**
```
PARSED_SKILL   visual_pick
PARSED_PARAMS  {'target_id': 'box_01'}
PARSED_TARGET  box_01
INTENT_PROVIDER openai-compatible-intent/stub-intent
SKILL_MAPPING  解析=visual_pick → 显示通道执行=display_pick（同一 Backend 闭环；display_pick 仅放宽时长上限）
DISPLAY_CHANNEL scripts/view_mujoco.py --baseline config/piper_simulation_baseline.yaml --live \
                --skill display_pick --target-id box_01 --pick-duration-ms 3000 --seconds 25
VIEWER_WINDOW_OPENED True   VIEWER_FRAMES 2284   VIEWER_OK   exit=0
```
抓取**进行中**窗口即存在（`WM_CLASS="MuJoCo"`，1280x720），即"边抓边看"。

**规则与边界**
- 解析出的 skill 必须属于**可显示集合**（`display_pick`/`visual_pick`/`pick_object`），否则**显式失败**（退出码 3）；不做"兜底猜一个"。
- 显示通道统一用 `display_pick`：它与 `visual_pick` 走**同一条** Backend 闭环，只把时长上限放宽（见 `skills/display_pick/skill.yaml` 自述）。
- 解析出的 `target_id`（若有）透传给显示通道。
- **离线桩** `scripts/dev_stub_intent_model.py`：OpenAI 兼容、只做最小受约束解码（关键词→skill、只填语义可知参数），
  日志/报告里必须标明"桩，不是真 LLM"；换成真模型**只改上面两个环境变量**，代码与流程不变。
- 仿真结论一律 `simulation=true`；本入口不驱动电机/关节、不写判据、不绕过 Policy。

