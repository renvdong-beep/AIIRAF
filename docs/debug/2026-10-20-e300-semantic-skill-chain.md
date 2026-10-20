# E300 双域：语义 → Skill → 跨机执行 首次打通（2026-10-19/20）

机器：智控域 10.203.247.147（Intewell R1.2.1 / aarch64 / Python 3.11.6 / glibc 2.38）
      运控域 10.203.247.149（同 OS，独立 VM，vm-1-net0 桥接到 br0）
      x86 仿真 Capability Provider：10.203.247.121:50052（headless，offscreen_frames）

## 1. 链路结论（有实测输出）

```
板上语义（LLM 意图解析）
  → IRAF TaskFlow / Skill Runtime / PolicyGateway（都在板上）
  → gRPC 跨机调用 x86 Capability Provider（Bearer token 鉴权）
  → MuJoCo 真跑（piper_pick 场景）
  → fail-closed 返回精确误差
```

最后一次实测（板上 CLI，`--text "抓取 box_01"`）：

```
resolved_skill: pick_object
adapter: agentos-intent       intent_provider: openai-compatible-intent 1.2.0 (deepseek-flash)
status: FAILED  error_code: IRAF-PROVIDER-ERROR
reason: 末端未到达目标抓取位姿: distance=0.014450m tolerance=0.005000m
        delta=[0.004229494155511354, 5.415238105131032e-06, 0.013817039392045947]
        qpos={'joint1': 3.784e-05, 'joint2': 1.1861, 'joint3': -0.12330,
              'joint4': 1.525e-06, 'joint5': -0.12034, 'joint6': -3.308e-08}
simulation: true
```

判读：**语义解析、策略判定、跨机鉴权、能力执行都已成立**（臂真的动了，qpos 有值）；
剩下的失败是"末端到不了抓取位姿"——误差 14.45 mm，其中 Z 向贡献 13.82 mm，容差 5 mm。
这是模型/标定层的残差问题，不是集成层问题。它与既有记录
`docs/debug/2026-09-24-joint-model-dog-arm.md §11.67`（residual_tolerance_m 已判死、
下一步判别实验是 align_max_attempts 3→6）是同一族现象。

## 2. 为打通链路修掉的四个真缺陷（均已入仓）

1. **网关不支持 `response_format=json_schema`**（400 `This response_format type is unavailable now`）
   ⇒ `src/iraf_adapters/model/openai_intent.py` 做降级链 `json_schema → json_object → none`，
   **降级不放宽校验**（仍用同一 JSON Schema）。逐字段探针：`scripts/probe_intent_gateway.py`。
2. **我把 `_validate_profile_semantics` 挤成嵌套函数**（插入新方法时结构错位），板上报
   `AttributeError: 'OpenAICompatibleIntentProvider' object has no attribute ...`
   ⇒ 修正类结构；补 `tests/unit/test_intent_provider.py`（8 例，含直接调 `parse()` 的用例）。
3. **`max_tokens` 写死 256 被推理型模型吃光**：`finish_reason=length`、`completion_tokens=256`、
   `content_len=0` ⇒ "模型未返回严格 JSON 意图"。改为可配（默认 1024，
   `IRAF_AGENTOS_MAX_TOKENS` 覆盖），并让报错带 `finish_reason/completion_tokens/content_len`。
4. **探针脚本把两个秘密混用**：`IRAF_PROVIDER_TOKEN`（跨机 Provider）与 `IRAF_AGENTOS_TOKEN`
   （模型网关）本是两回事，混用导致 provider 侧 `UNAUTHENTICATED`。探针已拆开。

部署侧另修两条（见 `deploy/sdk/board_deploy.sh`）：`config/install/` 含目录时必须 `scp -r`；
卸载路径不再强制 `--wheelhouse`。

## 3. 装机状态（两域分机，各自独立）

```
智控域 .147  edge-intel   /opt/iraf-sdk/0.2.0        /etc/iraf/edge-intel.env
                          iraf-runtime-grpc.service  CPUQuota=150% MemoryMax=1G  disabled/inactive
                          env: PROFILE=profiles/piper_mujoco.yaml
                               SAFETY_POLICY=profiles/safety/simulation_lab.yaml
                               RUNTIME_SUBJECT=board-e300-intel
                               endpoint=10.203.247.121:50052
                          --list-skills → 13 个技能
运控域 .149  edge-motion  /opt/iraf-sdk-motion/0.2.0 /etc/iraf-motion/edge-motion.env
                          iraf-control-provider.service CPUQuota=100% MemoryMax=512M disabled/inactive
                          （真机控制入口未定义 ⇒ 仍是 readiness 桩，铁律 6.8）
```

## 4. 环境事实（排查用）

- 智控域**原本没有默认路由**（只有 10.203.247.0/24 直连），到不了模型网关 192.168.100.89
  ⇒ 实测报 `Network is unreachable`。本次临时补：
  `ip addr del 10.203.247.147/24 dev br0 && ip addr add 10.203.247.147/23 dev br0 &&
   ip route add default via 10.203.246.1 dev br0`
  **注意这是临时命令，重启即失** ⇒ 要持久化必须写进该域的网络配置。
- br0 的端口是 eth2 + vm-0-net0 + vm-1-net0（`/sys/class/net/br0/brif/` 可查；
  `bridge link show` 在这台机器上输出为空，别据此误判"没挂进桥"——我误判过一次）。
- 板卡→x86 Provider：`tcp_50052=OPEN`，connect≈0.88 ms。
- 两个域都跑 Intewell R1.2.1、Python 3.11.6、glibc 2.38，与 `profiles/boards/e300.yaml`
  声明的 cp311/manylinux_2_28_aarch64 一致。

## 5. 未闭合

### 5.1 已拿到 SUCCEEDED 的闭环证据（2026-10-20 复测）

板上 CLI，`--text "停止机器人"`：
```
execution_id: 838347b7-dbf1-5606-9be6-096a5c2a2e64
resolved_skill: stop      status: SUCCEEDED      error_code: ""      退出码 0
```
更早一次 `--text "把 joint1 移动到 0.1 弧度"` 也拿到过 `status: SUCCEEDED` / `resolved_skill: move_joint` / 退出码 0。
⇒ 「自然语言 → Skill → 板上策略 → 跨机 Provider 执行」在这条链上是通的，且终态如实。

同一轮修掉的第四个缺陷：`stop` 被执行层拒绝
`IRAF-CAPABILITY-UNKNOWN 能力 'stop' 未登记参数形状` —— 根因是
`provider_service._POSITIONAL` 只登记了 `emergency_stop`，没有技能名 `stop`。
已补 `"stop": ()`，并加 `tests/unit/test_capability_registry.py` 做"技能目录 ↔ 能力表"静态对账
（AST 解析，不需要 grpc）。**注意：provider 是 x86 上的常驻进程，改完必须重启它才生效**——
本轮就因此多失败了一次（板上装了新代码但旧 Provider 仍在跑）。

### 5.2 已闭合：pick_object 的残差根因是**缺省时长**，不是标定

板上 `抓取 box_01` 之前报 `末端未到达目标抓取位姿 distance=0.014450m tolerance=0.005000m`。
打开 `IRAF_DEBUG_PICK=1` 抓到相位账本：

```
PICK_INPUTS {"duration_ms": 1000, "phase_ms_each": 200, ...}
PICK_TRACE  HOME_HOLD  center_distance_m=0.163883  (起点)
PICK_TRACE  APPROACH   center_distance_m=0.071105
PICK_TRACE  DESCEND    center_distance_m=0.014450   ← 门禁读数，仍 > 0.005
```

根因：`src/iraf_skills/common/manipulation.py` 里 `duration_ms` 缺省 **1000 ms**，
而 backend 按 5 段切分（`phase_ms = duration_ms // 5`）⇒ 每段只有 **200 ms**；
而 `mujoco_backend` 相位注释里的标定值是**每段 1600 ms（总 8000 ms）**。
即"代码默认值比标定值短 8 倍"，臂根本没有收敛时间。**不是标定残差，也不是容差问题。**

修法（commit 8854a1a）：新增 `PICK_NOMINAL_DURATION_MS = 8000`、`PLACE_NOMINAL_DURATION_MS = 6400`
（分别对应 5 段 / 4 段 × 1600 ms），把缺省值改为标定值；**判据 tolerance 未动**。

修后实测（关闭 debug，干净复跑，板上 `--text "抓取 box_01"`）：

```
status: SUCCEEDED      resolved_skill: pick_object      simulation: true
execution_id: 1e8171e9-160e-5c00-a307-e7767bb2b2e1
policy: 1.0.0  decision: 8abded60-f884-468e-b2b6-76befad0cb8f
accepted: true         confirmation: "constraint"       target_id: box_01
pose_source: live_target_body
grasp_alignment.center_distance_m = 9.387527253937005e-06   ← 9.39 µm（容差 5 mm）
bilateral_contact = true    force_ok = true
lift_delta_m = 0.092124     lift_peak_delta_m = 0.080084     lift_attitude_max_deg = 21.470857
regrasp.applied = true      regrasp.force_imbalance_ratio = 1.0155
```

即残差从 **14.45 mm → 9.39 µm**，`grasped/lifted` 均为 true，终态 SUCCEEDED。

补充：此前还出现过一次 `distance=0.163883m` 且 `qpos` 全 0 的异常，是在同一"相位过短"条件下
发生的首调异常；改缺省时长后 1 次 debug 复跑 + 1 次干净复跑**都 SUCCEEDED**，未再复现
（样本仅 2 次，若再出现需单独追）。

### 5.3 其余未闭合
1. ~~pick_object 残差~~ —— **已闭合**，见 5.2（根因是缺省时长，不是标定容差）。
2. 智控域默认路由是临时命令，未持久化（重启会再次 `Network is unreachable`）。
3. 运控域 edge-motion 仍是桩；真机入口未定义，不能声称运控能力可用。
4. 语义层目前在**板上 IRAF CLI 内部**用 LLM 解析（`IRAF_AGENTOS_*`），板上 AgentOS 仍未安装；
   与"AgentOS 发语义"的最终形态还差一层绑定（MCP/子进程，见技能阶段 4）。
5. `visual_pick` 也会被模型选中（"抓取 box_01" 曾被解析成 visual_pick）——
   意图歧义属模型行为，若要固定技能需在提示词/契约层收敛，勿用放宽校验解决。
