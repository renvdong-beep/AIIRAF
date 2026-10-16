# -*- coding: utf-8 -*-
"""远端 Capability Provider 的**客户端后端**（板卡侧，2026-10-16）。

它把 `RobotBackend` 契约的每个能力方法转成一次跨机 `Invoke`。
装上它之后，**板上跑的是完整 IRAF**（TaskFlow → Skill Runtime → PolicyGateway → 这里），
x86 只把能力在 MuJoCo 里执行出来并显示 —— 这就是用户口径下
`edge-intel.backend_entrypoint` 缺的那一层。

契约（与 `src/iraf_adapters/backend.py` 的 Protocol 同名同义）
--------------------------------------------------------------
`from_config(config, profile, authority)` + `profile.capabilities` 里每个能力都要有同名方法
（`factory.verify_backend_contract` 会在**装配期**校验，不通过即失败）。

配置（来自声明，禁止业务代码硬编码）
-----------------------------------
```yaml
endpoint: "10.203.247.121:50052"     # x86 仿真 Provider 服务端
token_env: "IRAF_PROVIDER_TOKEN"      # token 只从环境变量读（不落盘、不进命令行）
timeout_s: 180                        # 单次能力调用的 gRPC 截止时间
resource_id: "robot-arm"              # 可选：资源 id（缺省由服务端定）
subject: "board-runtime"              # 审计用调用方标识（身份由受信 transport 决定）
```

边界（铁律）
------------
- **不伪造成功**：服务端返回 `accepted=false` 或抛错 → 抛 `RemoteProviderError`（带显式错误码），
  绝不返回一个看起来成功的字典；
- **不做策略判定**：策略在板上（调用方），这里只转发；
- 每次调用都带 `correlation_id` / `idempotency_key` / `deadline` / `resource_id` / `fencing_token`。
"""
from __future__ import annotations

import os
import uuid
from typing import Any, Dict, Optional


class RemoteProviderError(RuntimeError):
    """远端拒绝或失败（带显式错误码；调用方据此决定重试/安全停机，**不得**当成成功）。"""

    def __init__(self, code: str, message: str, retryable: bool = False):
        super().__init__("[%s] %s" % (code, message))
        self.code = code
        self.retryable = retryable


class RemoteProviderBackend:
    """跨机 Capability Provider 后端（客户端侧）。"""

    def __init__(self, endpoint: str, token: str, *, timeout_s: float = 180.0,
                 resource_id: Optional[str] = None, subject: str = "board-runtime",
                 provider_hint: str = "iraf-adapters/mujoco") -> None:
        if not endpoint:
            raise ValueError("endpoint 必填（远端 Provider 地址，来自声明）")
        if not token:
            raise ValueError("token 必填（只从环境变量来；缺 token 即拒绝装配，fail-closed）")
        self.endpoint = str(endpoint)
        self._token = str(token)
        self._timeout_s = float(timeout_s)
        self._resource_id = resource_id
        self._subject = subject
        self._provider_hint = provider_hint

    # ---------- 装配契约 ----------
    @classmethod
    def from_config(cls, config, profile, authority, **kwargs):  # noqa: D401
        """`factory.load_backend` 的装配入口（config 来自部署声明）。"""
        cfg = dict(config or {})
        token_env = str(cfg.get("token_env") or "IRAF_PROVIDER_TOKEN")
        token = os.environ.get(token_env) or ""
        endpoint = str(cfg.get("endpoint") or os.environ.get("IRAF_PROVIDER_ENDPOINT") or "")
        return cls(endpoint, token,
                   timeout_s=float(cfg.get("timeout_s") or 180.0),
                   resource_id=cfg.get("resource_id"),
                   subject=str(cfg.get("subject") or "board-runtime"))

    # ---------- 运行时清单（可发现性） ----------
    def runtime_inventory(self) -> Dict[str, Any]:
        """运行时清单**跨机取回**（不是本地静态说明）。

        为什么必须跨机：`iraf_core/runtime.py:93` 拿这份清单去求值 skill 的 preconditions
        （如 `safety.estop == false`、`manipulation.target_visible == true`）。
        板上若用本地假清单，就等于用"猜的状态"给物理动作放行 —— 那是静默绕过前置条件。
        取不到就显式失败，不放行。
        """
        health = self.health()
        inventory = dict(health.get("inventory") or {})
        if not inventory:
            raise RemoteProviderError(
                "IRAF-PROVIDER-INVENTORY-UNAVAILABLE",
                "远端 Provider 未提供运行时清单；拒绝用本地假清单放行物理动作")
        inventory.setdefault("mode", "simulation")
        inventory["backend"] = "remote-provider"
        inventory["endpoint"] = self.endpoint
        inventory["provider"] = self._provider_hint
        inventory["simulation"] = True   # 跨机 Provider 是仿真域；不得表述为真机能力
        return inventory

    def health(self) -> Dict[str, Any]:
        """探活（只读）。返回远端 Provider 的能力清单与显示事实。"""
        from google.protobuf.json_format import MessageToDict
        stub, channel = self._open()
        try:
            from iraf.v1 import provider_pb2
            md = (("authorization", "Bearer " + self._token),)
            resp = stub.Health(provider_pb2.ProviderHealthRequest(),
                               timeout=min(self._timeout_s, 10.0), metadata=md)
            return {
                "ready": bool(resp.ready),
                "simulation": bool(resp.simulation),
                "backend": resp.backend,
                "capabilities": list(resp.capabilities),
                "display_mode": resp.display_mode,
                "window_opened": bool(resp.window_opened),
                "fencing_token": int(resp.fencing_token),
                "detail": resp.detail,
                "inventory": (MessageToDict(resp.inventory)
                              if resp.HasField("inventory") else {}),
            }
        finally:
            channel.close()

    # ---------- 能力方法（与 RobotBackend 契约同名同义） ----------
    def move_joint(self, positions, duration_ms, lease) -> None:
        self._invoke("move_joint", {"positions": positions, "duration_ms": int(duration_ms)}, lease)

    def pick_object(self, target_id, grasp_pose, duration_ms, lease) -> Dict[str, Any]:
        return self._invoke("pick_object", {"target_id": target_id, "grasp_pose": grasp_pose,
                                            "duration_ms": int(duration_ms)}, lease)

    def place_object(self, place_target_id, payload_id, duration_ms, lease) -> Dict[str, Any]:
        return self._invoke("place_object", {"place_target_id": place_target_id,
                                             "payload_id": payload_id,
                                             "duration_ms": int(duration_ms)}, lease)

    def visual_pick(self, inputs, lease) -> Dict[str, Any]:
        return self._invoke("visual_pick", {"inputs": inputs}, lease)

    def accept_payload(self, payload_id, place_target_id, lease) -> Dict[str, Any]:
        return self._invoke("accept_payload", {"payload_id": payload_id,
                                               "place_target_id": place_target_id}, lease)

    def stop(self, lease) -> None:
        self._invoke("stop", {}, lease)

    def step(self, count: int = 1) -> Dict[str, Any]:
        return self._invoke("step", {"count": int(count)}, None)

    def read_state(self) -> Dict[str, Any]:
        return self._invoke("read_state", {}, None)

    # ---------- 内部 ----------
    def _open(self):
        import grpc
        from iraf.v1 import provider_pb2_grpc
        channel = grpc.insecure_channel(self.endpoint)
        return provider_pb2_grpc.CapabilityProviderServiceStub(channel), channel

    def _invoke(self, capability: str, inputs: Dict[str, Any], lease) -> Any:
        from google.protobuf.json_format import MessageToDict, ParseDict
        from google.protobuf.timestamp_pb2 import Timestamp
        from iraf.v1 import provider_pb2

        correlation_id = "rpc-" + uuid.uuid4().hex
        envelope = provider_pb2.LeaseEnvelope(
            correlation_id=correlation_id,
            idempotency_key=correlation_id,          # 单次调用即一次逻辑动作
            resource_id=self._resource_id or "",
            caller_subject=self._subject,
            controller="board-runtime",
            fencing_token=int(getattr(lease, "fencing_token", 0) or 0),
        )
        if lease is not None and getattr(lease, "resource", None):
            envelope.resource_id = str(lease.resource)
        deadline = Timestamp()
        deadline.GetCurrentTime()
        deadline.seconds += int(self._timeout_s)
        envelope.deadline.CopyFrom(deadline)

        call = provider_pb2.CapabilityCall(capability=capability, lease=envelope)
        ParseDict(inputs, call.inputs)

        stub, channel = self._open()
        try:
            md = (("authorization", "Bearer " + self._token),)
            resp = stub.Invoke(call, timeout=self._timeout_s, metadata=md)
            if not resp.accepted:
                err = resp.error
                if err.code == "IRAF-SAFETY-QUARANTINED":
                    # 远端被控对象已隔离（铁律 1.6/1.9）：抛 core 定义的异常，
                    # 由 SkillRuntime 捕获后在**本侧**也隔离该资源（跨机安全事件不做单向传导）。
                    from iraf_core.safety import SafetyEventReported
                    details = dict(err.details or {})
                    raise SafetyEventReported(
                        details.get("resource") or self._resource_id or "",
                        details.get("reason") or err.message_zh or "远端报告安全事件",
                        details.get("kind") or "SAFETY_EVENT")
                raise RemoteProviderError(err.code or "IRAF-PROVIDER-REJECTED",
                                          err.message_zh or "远端 Provider 拒绝（无消息）",
                                          retryable=bool(err.retryable))
            payload = MessageToDict(resp.outputs) if resp.HasField("outputs") else {}
            payload.setdefault("provider_execution_id", payload.get("provider_execution_id"))
            return payload
        finally:
            channel.close()
