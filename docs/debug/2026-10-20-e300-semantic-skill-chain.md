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

1. `pick_object` 残差 14.45 mm（Z 向 13.82 mm）> 5 mm 容差 ⇒ 抓取本身未成功；
   下一步按 §11.67 的判别实验：`align_max_attempts 3→6`，**不放宽 residual 判据**。
2. 智控域默认路由是临时命令，未持久化（重启会再次 `Network is unreachable`）。
3. 运控域 edge-motion 仍是桩；真机入口未定义，不能声称运控能力可用。
4. 语义层目前在**板上 IRAF CLI 内部**用 LLM 解析（`IRAF_AGENTOS_*`），板上 AgentOS 仍未安装；
   与"AgentOS 发语义"的最终形态还差一层绑定（MCP/子进程，见技能阶段 4）。
5. `visual_pick` 也会被模型选中（"抓取 box_01" 曾被解析成 visual_pick）——
   意图歧义属模型行为，若要固定技能需在提示词/契约层收敛，勿用放宽校验解决。
