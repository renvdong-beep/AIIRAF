# IRAF 南向 Provider SDK 使用指南

版本：0.5.1（2026-11-06）  
适用对象：在自己的 x86 Ubuntu / 运控域上实现 **Capability Provider** 的团队。  
边界：Provider 只执行"单个能力调用"，**不做策略判定**；策略与技能选择在边缘 IRAF。
一切仿真结论 `simulation=true`，不构成真机/实时能力证据。

## 1. 它在链路中的位置

```text
上位机 harness ──北向──> 边缘 IRAF Runtime/Policy
                              │  provider.proto (CapabilityProviderService)
                              ▼
                    你的 Provider（本 SDK）
                    Health / Invoke / Cancel
```

边缘只会调用你**声明过且实现过**的能力；未声明/未实现/未知能力一律拒绝。

## 2. 安装

```bash
python3 -m pip install 'iraf_sdk-0.5.1-py3-none-any.whl[provider]'
# protobuf 桩（本交付包的 stubs/，或边缘 SDK 的 <version>/lib）加入 PYTHONPATH：
export PYTHONPATH=<交付包>/stubs:/opt/iraf-sdk/current/lib:$PYTHONPATH
```

`iraf_sdk.provider` 顶层只依赖标准库；`grpcio` 与桩只在 `build_servicer()/serve()`
时惰性导入，因此纯逻辑测试不需要 grpc。

## 3. 实现一个后端（duck typing）

```python
class MyMujocoBackend:
    # 必须：声明能力（只会暴露这里列出的能力）
    capabilities = {"move_joint", "pick_object", "read_state"}
    display_mode = "offscreen_frames"        # 或 interactive_viewer / unavailable

    # 建议：策略前置条件依赖这份清单，必须如实
    def runtime_inventory(self):
        return {
            "mode": "simulation",
            "safety": {"estop": False},
            "manipulation": {"target_visible": True},
        }

    # 能力方法：参数名与 provider.proto 的 inputs 一致；签名里有 lease 就注入
    def move_joint(self, positions, duration_ms, lease):
        # 在这里驱动你的 MuJoCo/控制器
        return {"moved": True, "fencing": lease.fencing_token}

    def pick_object(self, target_id, grasp_pose, duration_ms, lease):
        return {"lifted": True, "target_body": target_id}

    def read_state(self, lease):
        return {"qpos": [...]} 
```

约束：

- 能力词表固定（`iraf_sdk.provider.CAPABILITY_METHODS`，与边缘 `factory.CAPABILITY_METHODS`
  逐项一致，有单测防漂移）；新增能力必须先改 `api/proto/iraf/v1/provider.proto`。
- 返回值必须是 dict；返回非 dict 会被判 `IRAF-PROVIDER-ERROR`，禁止伪造成功。
- 不要在这里做策略判断或放宽安全上限；Provider 只执行，不决定"该不该做"。
- 显示契约（AGENTS 6.12）：实现 `render_frames` / `display_lock` / `render_settings`，
  无头场景在 `import mujoco` 之前设 `MUJOCO_GL=egl`。

## 4. 启动服务

```python
from iraf_sdk.provider import ProviderKit

kit = ProviderKit(
    backend=MyMujocoBackend(),
    token=token,                         # 与边缘装配单里的 IRAF_PROVIDER_TOKEN 一致
    provider_id="acme/mujoco",
    subject="acme-provider",
    resource_id="piper-a",               # 与边缘 resource_id 对应
    simulation=True,                     # 仿真档必须 True
    idempotency_db="/var/lib/acme/provider-idem.db",  # 建议：重启后幂等仍有效
)
kit.serve(bind="0.0.0.0:50052")
```

边缘侧装配单里声明这个端点：

```bash
IRAF_BACKEND_ENTRYPOINT=iraf_adapters.remote.provider_backend:RemoteProviderBackend
IRAF_BACKEND_CONFIG='{"endpoint":"<provider>:50052","token_env":"IRAF_PROVIDER_TOKEN","timeout_s":180,"subject":"edge-runtime"}'
```

并在 RobotProfile 的 `capabilities` 里只声明你真实实现的能力（装配期会校验声明 ⊆ 实现）。

## 5. 安全不变量（SDK 已强制，客户改不了）

| 场景 | 结果 |
|---|---|
| 缺 execution_id / idempotency_key | `IRAF-INPUT-INVALID` |
| 缺 deadline 或已过期 | `IRAF-INPUT-INVALID` / `IRAF-DEADLINE-EXCEEDED` |
| fencing token 回退 | `IRAF-EXECUTION-CONFLICT` |
| 终态 execution 收到新控制 | `IRAF-EXECUTION-CONFLICT` |
| 同 execution_id + idempotency_key 重放 | 返回既有结果（`idempotent_replay=true`），不重复执行 |
| 未声明 / 未实现 / 未知能力 | `IRAF-POLICY-DENIED` / `IRAF-CAPABILITY-NOT-IMPLEMENTED` / `IRAF-CAPABILITY-UNKNOWN` |
| Bearer token 错 | gRPC `UNAUTHENTICATED` |
| 后端抛异常 | `IRAF-PROVIDER-ERROR`，不会变成成功 |

Health 返回：`ready / simulation / backend / capabilities / provider / fencing_token /
display_mode / window_opened / detail / inventory`。边缘用 `inventory` 求值策略前置条件，
所以 `safety.estop`、`manipulation.target_visible`、`mode` 必须如实。

## 6. 自检与测试

本机已验证（`build/harness/provider_kit_smoke.py`）：

```text
HEALTH True True ['move_joint', 'read_state'] offscreen_frames
INVOKE True True {"moved": true}
REPLAY True {"moved": true}
AUTH rejected UNAUTHENTICATED
NO_DEADLINE False IRAF-INPUT-INVALID
```

单元测试：`PYTHONPATH=src python3 -m pytest -q tests/unit/test_provider_sdk.py`
（能力词表一致性、lease/fencing、幂等、后端异常、输出契约、持久化幂等）。

## 7. 未完成（交付前必须知道）

- 没有独立的 `iraf-provider-selftest` / conformance kit；目前用单测与冒烟脚本。
- Provider 的 gRPC 传输当前是 `add_insecure_port`；对外生产必须由 TLS/mTLS 网关或
  后续版本补齐（AGENTS 2.7）。
- 帧回传 RPC 尚未定义（`provider.proto` 只有 Health/Invoke/Cancel）。
- Provider manifest 的签名/注册流程仍由部署期声明完成，没有在线自助注册。
- 本 SDK 不做实时性保证；运控域实时段仍需独立验收。
