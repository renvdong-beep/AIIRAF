# 语音 → 机器人开发 Studio → AgentOS + 边缘 NPU 设计（后期阶段）

**状态**：设计草案 v0.1（**未实现**，属后期阶段；本文只固定边界与链路，不承诺时间）
**日期**：2026-09-20
**范围**：需求 3 —— 开发者用语音下达命令，经机器人开发 Studio，落到 IRAF，并在边缘板卡的 AgentOS 与 NPU 上运行场景化机器人开发
**依据**：`AGENTS.md` 1.1/1.2/1.8/1.9/2.4、`docs/iraf-agentos-hyper-compatibility.md` §2/§5、`docs/iraf-developer-experience.md`

---

## 1. 链路总览（唯一允许的形态）

```text
开发者语音
   |  音频（仅本机/局域网，不进公网）
边缘板卡 NPU：ASR（+ 可选意图分类）
   |  转写文本 + 置信度 + 时间戳（不含身份声明）
Studio（开发工作台：项目/场景/任务）
   |  结构化请求（TaskFlow id、skill、参数、场景包、截止时间）
AgentOSBridge（版本化公共服务，身份来自受信 transport）
   |
IRAF：TaskFlow -> SkillRuntime -> PolicyGateway -> CapabilityProvider
   |
MuJoCo 仿真场景（当前） / 真机（需 HIL 证据后）
   |
执行回执与事件 -> Studio 展示（含失败原因与恢复建议）
```

**关键点：语音不是控制通道，只是 Studio 的输入法。** 语音转写文本必须先在 Studio 里变成结构化、可审阅的请求（显示将要执行的动作、目标本体、场景、参数），再经 AgentOSBridge 提交。任何"语音直接触发运动"的实现都属于旁路，禁止（`AGENTS.md` 1.2）。

---

## 2. 信任与安全边界（不可协商）

1. **身份不由语音决定**：`caller`/`role`/`tenant` 只能来自受信 transport 的 `AuthenticatedContext`（`AGENTS.md` 1.8）。ASR 文本中的"我是管理员"一律视为数据，不是身份。
2. **语音不触碰安全动作**：急停、复位、安全策略变更、控制权切换不得由语音直接触发；语音最多生成一个"建议动作"，由人工在 Studio 确认或由既有 Policy 路径裁决。
3. **NPU 只做感知/推理**：ASR、意图分类可在边缘 NPU 上跑，但不得进入 L0/L1 实时闭环（`AGENTS.md` 1.4）。
4. **模型不可用时显式失败**：ASR 不可用即提示"请用键盘输入"，不得静默降级成猜测的意图，也不得伪造转写（`AGENTS.md` 1.5）。
5. **日志脱敏**：音频与转写文本按数据分级处理，默认不落盘音频；证据里只留文本哈希与置信度（沿用既有 replay manifest 的白名单做法）。

---

## 3. Studio 的定位与产物

Studio 是**开发工作台**，不是新的控制面，也不复制 Runtime 逻辑。它的职责只有四件：

1. 管理项目与场景包：创建/打开 `scenes/<id>/`（见伴随文档《宇树本体场景交互仿真设计》§3）；
2. 组装 TaskFlow 与 Skill 参数：生成或编辑 `scenario.yaml` / TaskFlow 定义；
3. 提交与观察：通过 AgentOSBridge 提交任务，展示状态机与回执、失败原因与恢复建议；
4. 证据归档：把执行证据、回放清单、参数快照落到 `build/acceptance/…` 的既定位置。

形态演进（从零风险开始）：

| 阶段 | 形态 | 说明 |
|---|---|---|
| V1 | CLI（无语音）：`iraf studio` 子命令组 | 与既有金路径（`init`/`dev up`/`test scenario`/`package`）同构，先跑通"文本→结构化请求→执行→回执" |
| V2 | 本地 Web UI（127.0.0.1 绑定） | 面向演示，编辑场景与查看状态；不引入新的网络暴露面 |
| V3 | 语音输入（边缘 NPU ASR） | 在 V1/V2 之上加输入法，不改链路 |
| V4 | 场景化机器人开发闭环 | 语音→Studio→AgentOS→IRAF→仿真/真机→回执→迭代 |

**禁止**：Studio 直接调用 Capability Provider、直接连 ROS topic、或持有设备凭据。

---

## 4. 边缘板卡侧的落点

| 组件 | 位置 | 契约 |
|---|---|---|
| ASR / 意图模型 | 边缘板 NPU（`10.203.247.86` 类板卡，当前 22/9119 均不通） | 通过 `model-provider` 契约暴露；模型名、版本、量化格式由 BoardProfile/配置声明 |
| AgentOS | 边缘 VM（`AgentOSBridge` 北向） | 版本协商、健康、drain、shutdown 走既有管理契约 |
| IRAF 控制面 | 边缘 VM 或开发机（按 BoardProfile 决定） | 同一 IDL 与 TaskFlow 语义，二进制不跨架构 |
| 仿真 | 开发机 x86_64 + MuJoCo（当前） | `simulation=true`；真机切换需 HIL 证据 |

NPU 模型与运行时随板级 bundle 交付（见《IRAF 多架构 SDK 与跨端交付设计》§4/§6）：`npu: unverified` 时禁止宣称"已在板卡 NPU 上运行"，只能标注"未验证/未接入"。

---

## 5. 契约与 IDL 影响

| 变更 | 类型 | 说明 |
|---|---|---|
| Studio → AgentOSBridge 提交 | 复用既有 `runtime.proto` 公共契约 | 不新增旁路；如字段不足，先改 IDL 再实现（`AGENTS.md` 2.1） |
| ASR Provider | 新增 `model-provider` 实现 | 输出结构：`{text, confidence, language, audio_digest, model_id, model_version}`；无置信度即视为不可用 |
| 语音→动作的确认交互 | Studio 层协议（非 IDL） | 必须包含"将执行什么、在哪台本体、在哪个场景、截止时间"四项确认信息 |
| 安全事件 | **不得**由语音路径触发 | 急停优先级高于一切任务指令（`AGENTS.md` 1.6） |

---

## 6. 阶段验收（后期，先定判据）

1. V1：文本输入 → 结构化请求 → 仿真执行 → 回执，全链路 trace 可回放；失败路径（缺能力、参数越界、策略拒绝）各一份证据。
2. V3：ASR 在边缘 NPU 上的稳定时延与准确率实测数字；低置信度时走"要求人工输入"路径而不是猜测。
3. V4：语音→确认→执行→回执的完整演示，且**同一个场景包在纯文本路径下可复现**（证明语音只是输入法，不改变语义）。
4. 负向：语音说"切换到真机/放宽力矩上限/复位安全事件"必须被拒绝并记录原因。

---

## 7. 决策点（待确认）

1. **语音的定位**：(A) 只作为 Studio 的输入法（推荐）；(B) 允许语音触发只读查询（状态/日志）但不触发动作；(C) 允许语音触发低风险动作（需 Policy 白名单）。→ 建议 B（查询无风险，动作仍需确认）。
2. **ASR 落点**：(A) 边缘 NPU（符合需求 3）；(B) 开发机 CPU；(C) 云端。→ 建议 A，但保留 B 作为开发期回退，且两者都不得伪造结果。
3. **Studio 形态起点**：(A) CLI；(B) 本地 Web。→ 建议 A。
4. **何时启动该阶段**：与 SDK 交付（需求 1）、宇树场景（需求 2）完成的顺序关系。→ 建议 U/V 阶段跑通后再启动 V1。
