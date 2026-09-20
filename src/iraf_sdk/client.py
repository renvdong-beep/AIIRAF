"""IRAF SDK 客户端最小层：只走公共契约，不开旁路（纯标准库，跨架构可独立安装）。

公共入口白名单（本文件**只**允许调用这些入口）：

- HTTP 开发适配器（`src/iraf_adapters/http/runtime_http.py`，需 `IRAF_HTTP_DEVELOPMENT=true`）：
  `POST /v1/tasks`（提交任务）、`GET /v1/tasks/{id}`（查询执行）；
- gRPC（`api/proto/iraf/v1/runtime.proto` / `events.proto`）：
  `iraf.v1.SkillRuntimeService.Execute / GetExecution / Cancel`、
  `iraf.v1.EventService.ListEvents / GetReplayManifest`。

铁律约束（AGENTS.md 1.1 / 1.2 / 1.8 / 2.3 / 2.4）：

1. **无旁路**：任务必经 `TaskFlow -> SkillRuntime -> Policy Gateway -> Capability Provider`。
   本文件不 import `iraf_core`，因此物理上无法构造 `AuthenticatedContext`、无法直连
   Provider、后端、设备或现场总线。
2. **身份只来自受信 transport**：客户端只发送 Bearer 凭据，上下文由服务端构造；
   请求体中的 caller/role/tenant 不被信任，SDK 也不提供任何"自声明身份"参数。
3. **无无限等待**：`timeout_s` 为构造必填参数（无默认值），每次调用都传递截止时间。
4. **无自动重试**：失败响应只做判读（`iraf_sdk.raise_for_status` / `IrafError`），
   重试必须由 TaskFlow 或调用方发起并新建 execution。
5. **纯标准库导入**：模块级只 import 标准库与 `iraf_sdk.errors`；`grpc` 与生成的
   `iraf.v1.*_pb2*` stub 全部按需惰性导入，故 SDK 可在 aarch64 目标端单独安装，
   亦不把 mujoco 等重依赖带进 import 链。

最小示例（HTTP 开发适配器，纯标准库）：

    from iraf_sdk import HttpSkillClient, raise_for_status

    client = HttpSkillClient(endpoint="http://127.0.0.1:8765", token="<token>", timeout_s=35)
    result = client.submit(
        skill="move_joint",
        profile={"name": "piper", "version": "1.0.0", "digest": "<sha256>"},
        safety_policy={"name": "lab", "version": "1.0.0", "digest": "<sha256>"},
        parameters={"positions": {"j1": 0.1}, "duration_ms": 500},
    )
    raise_for_status(result)  # 失败终态抛 IrafError，携带中文诊断与下一步建议
"""

import importlib
import json
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from datetime import datetime, timezone
from typing import Mapping, Optional, Sequence

from .errors import TRANSPORT_FAILED_CODE, describe

__all__ = [
    "HTTP_TASKS_PATH",
    "MAX_EVENT_LIMIT",
    "SdkConfigError",
    "SdkUnsupportedError",
    "SdkTransportError",
    "identity_of",
    "deadline_after",
    "canonical_execute_body",
    "execute_request_from_body",
    "status_from_state",
    "result_from_feedback",
    "result_from_snapshot",
    "event_to_dict",
    "replay_manifest_to_dict",
    "HttpSkillClient",
    "GrpcSkillClient",
]

HTTP_TASKS_PATH = "/v1/tasks"
# 服务端上界：src/iraf_adapters/grpc/events_grpc.py:62（limit > 1000 直接拒绝）
MAX_EVENT_LIMIT = 1000

_IDENTITY_KEYS = ("name", "version", "digest")


class SdkConfigError(ValueError):
    """SDK 使用方式/配置错误（缺端点、缺凭据、缺超时、身份三元组不全）。

    禁止隐式默认值：任何缺失都显式失败（铁律 1.5 不得伪造成功、2.3 不允许吞掉错误）。
    """


class SdkUnsupportedError(SdkConfigError):
    """所选传输/适配器不支持该操作；禁止用旁路或本地模拟顶替。"""


class SdkTransportError(RuntimeError):
    """SDK ↔ Runtime 之间的传输/协议层错误（服务端未给出 IDL error_code）。

    该异常由**客户端**产生：网络不可达、超时、HTTP 非 2xx 且响应体无 error_code、
    响应不是合法 JSON 等。它不代表服务端已收到或未收到请求，因此
    `reason` 里明确写出这一限制，提醒调用方先确认是否已执行，再决定是否重试。
    """

    def __init__(self, message_zh: str, *, http_status: int = 0, payload: Optional[Mapping] = None):
        self.code = TRANSPORT_FAILED_CODE
        self.http_status = http_status
        self.payload = dict(payload) if isinstance(payload, Mapping) else None
        self.diagnosis = describe(TRANSPORT_FAILED_CODE, message_zh)
        super().__init__(self.diagnosis.format_zh())


# --------------------------------------------------------------------------
# 参数与身份校验（fail-closed）
# --------------------------------------------------------------------------
def _require_text(value, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SdkConfigError(f"{field} 必须是非空字符串，实际：{value!r}（SDK 不提供隐式默认值）")
    return value.strip()


def _require_timeout(value, field: str = "timeout_s") -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise SdkConfigError(f"{field} 必须是 > 0 的秒数，实际：{value!r}（禁止无超时等待）")
    return float(value)


def _require_mapping(value, field: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise SdkConfigError(f"{field} 必须是对象（JSON object），实际：{type(value).__name__}")
    return value


def identity_of(value, field: str) -> dict:
    """把 RobotProfile / SafetyPolicy 的声明身份规范化为公共三元组 name/version/digest。

    既接受映射，也接受带同名属性的对象（例如 `iraf_core.profile.load_robot_profile`
    返回的 RobotProfile）。SDK **不读取 Profile 文件**：身份必须来自调用方已取得的已签名声明，
    缺任何一项都显式失败（铁律 1.3 安全上限只来自已签名 Profile/SafetyPolicy）。
    """
    if isinstance(value, Mapping):
        source = value
    else:
        source = {key: getattr(value, key, None) for key in _IDENTITY_KEYS}
    identity = {}
    for key in _IDENTITY_KEYS:
        item = source.get(key)
        if not isinstance(item, str) or not item.strip():
            raise SdkConfigError(
                f"{field}.{key} 缺失或为空（实际 {item!r}）：身份三元组必须完整来自已签名声明，"
                "禁止空身份或猜测默认值"
            )
        identity[key] = item.strip()
    return identity


def deadline_after(timeout_s, now_ms: Optional[float] = None) -> str:
    """把超时预算换算为公共契约使用的 ISO-8601 UTC 截止时间（`...Z`）。"""
    budget = _require_timeout(timeout_s)
    base = time.time() if now_ms is None else float(now_ms) / 1000.0
    return datetime.fromtimestamp(base + budget, timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_execute_body(
    skill,
    *,
    profile,
    safety_policy,
    parameters=None,
    request_id=None,
    idempotency_key=None,
    correlation_id=None,
    skill_version_constraint="",
    resource_id=None,
    controller="iraf-sdk",
    deadline_s,
) -> dict:
    """构造 `ExecuteSkillRequest` 的规范 JSON 体（HTTP 与 gRPC 共用同一份逻辑）。

    字段名与 `api/proto/iraf/v1/skill.proto` 的 JSON 映射一致（camelCase），
    且只含契约字段：`ParseDict(..., ignore_unknown_fields=False)` 会拒绝多余字段
    （见 `src/iraf_adapters/http/runtime_http.py:46`）。
    """
    skill_name = _require_text(skill, "skill")
    profile_identity = identity_of(profile, "profile")
    safety_identity = identity_of(safety_policy, "safety_policy")
    inputs = {} if parameters is None else _require_mapping(parameters, "parameters")
    request = _require_text(request_id if request_id is not None else str(uuid.uuid4()), "request_id")
    idempotency = _require_text(idempotency_key if idempotency_key is not None else request, "idempotency_key")
    correlation = _require_text(correlation_id if correlation_id is not None else request, "correlation_id")
    resolved_resource = resource_id if resource_id else profile_identity["name"]
    return {
        "requestId": request,
        "idempotencyKey": idempotency,
        "goal": {
            "correlationId": correlation,
            "skillName": skill_name,
            "skillVersionConstraint": skill_version_constraint or "",
            "inputs": dict(inputs),
            "deadline": deadline_after(deadline_s),
            "robotProfile": dict(profile_identity),
            "safetyPolicy": dict(safety_identity),
            "labels": {
                "resource_id": _require_text(resolved_resource, "resource_id"),
                "controller": _require_text(controller, "controller"),
            },
        },
    }


# --------------------------------------------------------------------------
# IDL stub 与消息转换（惰性导入；纯消息层不需要 grpc）
# --------------------------------------------------------------------------
def _stub(module_name: str):
    try:
        return importlib.import_module("iraf.v1." + module_name)
    except ImportError as exc:
        raise SdkConfigError(
            f"缺少 IDL stub iraf.v1.{module_name}：请先生成 proto stub 并把生成目录加入 PYTHONPATH"
            "（deploy/sdk/build_sdk.sh 负责生成；生成目录 build/generated/python），"
            f"原始错误：{exc}"
        ) from exc


def _grpc_module():
    try:
        return importlib.import_module("grpc")
    except ImportError as exc:
        raise SdkConfigError(
            "gRPC 客户端需要 grpcio（可选依赖）：请按 config/sdk/package_matrix.yaml 声明安装 "
            "grpcio（当前环境未安装）。若只需控制面开发联通，可改用 HttpSkillClient。"
            f"原始错误：{exc}"
        ) from exc


def execute_request_from_body(body: Mapping) -> object:
    """把规范 JSON 体转成 `ExecuteSkillRequest`（供 gRPC Execute 使用）。"""
    skill_pb2 = _stub("skill_pb2")
    json_format = importlib.import_module("google.protobuf.json_format")
    request = skill_pb2.ExecuteSkillRequest()
    json_format.ParseDict(body, request, ignore_unknown_fields=False)
    return request


def status_from_state(state_value) -> str:
    """把 `SkillState` 枚举值转为运行时状态字符串（`SKILL_STATE_RUNNING` -> `RUNNING`）。"""
    skill_pb2 = _stub("skill_pb2")
    name = skill_pb2.SkillState.Name(int(state_value))
    return name[len("SKILL_STATE_"):] if name.startswith("SKILL_STATE_") else name


def result_from_feedback(message) -> dict:
    """把 `SkillFeedback` 归一化为与 `SkillRuntime._result` 同形的结果字典。"""
    json_format = importlib.import_module("google.protobuf.json_format")
    return {
        "execution_id": message.execution_id,
        "status": status_from_state(message.state),
        "sequence": int(message.sequence),
        "phase": message.phase,
        "error_code": message.error.code,
        "reason": message.error.message_zh,
        "retryable": bool(message.error.retryable),
        "result": json_format.MessageToDict(message.outputs),
    }


def result_from_snapshot(message) -> dict:
    """把 `ExecutionSnapshot` 归一化为结果字典（error_code 字段始终存在）。"""
    return {
        "execution_id": message.execution_id,
        "status": status_from_state(message.state),
        "sequence": int(message.last_sequence),
        "skill": {
            "name": message.skill.name,
            "version": message.skill.version,
            "digest": message.skill.digest,
        },
        "provider": {"name": message.provider.name, "version": message.provider.version},
        "error_code": message.error.code,
        "reason": message.error.message_zh,
    }


def event_to_dict(message) -> dict:
    """把 `EventRecord` 转为可打印字典（不含原始载荷与凭据）。"""
    return {
        "event_id": message.event_id,
        "object_id": message.object_id,
        "execution_id": message.execution_id,
        "sequence": int(message.sequence),
        "kind": message.kind,
        "status": message.status,
        "reason": message.reason,
        "occurred_at_ms": int(message.occurred_at.seconds) * 1000 + int(message.occurred_at.nanos) // 1_000_000,
        "recovered_at_ms": int(message.recovered_at.seconds) * 1000 + int(message.recovered_at.nanos) // 1_000_000,
        "recovered_by": message.recovered_by,
    }


def replay_manifest_to_dict(message) -> dict:
    """把 `ReplayManifest` 转为可打印字典（含每条事件，便于一次取证）。"""
    json_format = importlib.import_module("google.protobuf.json_format")
    data = json_format.MessageToDict(message, preserving_proto_field_name=False)
    data["events"] = [event_to_dict(item) for item in message.events]
    return data


# --------------------------------------------------------------------------
# HTTP 传输（开发适配器）
# --------------------------------------------------------------------------
class HttpSkillClient:
    """`POST /v1/tasks`、`GET /v1/tasks/{id}` 的薄封装（开发仿真 HTTP 适配器）。

    该适配器**未提供**取消与事件查询端点：对应方法显式抛 `SdkUnsupportedError`，
    而不是在本地伪造结果，也不走旁路（需要时请使用 `GrpcSkillClient`）。
    """

    def __init__(self, endpoint, token, *, timeout_s, controller="iraf-sdk", default_resource_id=None):
        self.endpoint = _require_text(endpoint, "endpoint").rstrip("/")
        self.token = _require_text(token, "token")
        self.timeout_s = _require_timeout(timeout_s)
        self.controller = _require_text(controller, "controller")
        self.default_resource_id = default_resource_id

    # -- 公共契约调用 -------------------------------------------------------
    def submit(
        self,
        skill,
        *,
        profile,
        safety_policy,
        parameters=None,
        request_id=None,
        idempotency_key=None,
        correlation_id=None,
        skill_version_constraint="",
        resource_id=None,
        controller=None,
        deadline_s=None,
        timeout_s=None,
    ) -> dict:
        """提交任务；返回服务端原始结果字典（失败终态请用 `raise_for_status` 判读）。"""
        body = canonical_execute_body(
            skill,
            profile=profile,
            safety_policy=safety_policy,
            parameters=parameters,
            request_id=request_id,
            idempotency_key=idempotency_key,
            correlation_id=correlation_id,
            skill_version_constraint=skill_version_constraint,
            resource_id=resource_id,
            controller=controller or self.controller,
            deadline_s=self.timeout_s if deadline_s is None else deadline_s,
        )
        return self._request("POST", HTTP_TASKS_PATH, body, timeout_s=timeout_s)

    def get_execution(self, execution_id, *, timeout_s=None) -> dict:
        """查询执行状态。"""
        identifier = _require_text(execution_id, "execution_id")
        return self._request("GET", HTTP_TASKS_PATH + "/" + urllib.parse.quote(identifier, safe=""), None, timeout_s=timeout_s)

    def cancel(self, execution_id, reason, *, timeout_s=None) -> dict:
        raise SdkUnsupportedError(
            "HTTP 开发适配器（src/iraf_adapters/http/runtime_http.py）未提供取消端点："
            "请使用 GrpcSkillClient.cancel（iraf.v1.SkillRuntimeService.Cancel）。"
            "SDK 不会用本地模拟或旁路路径顶替该能力。"
        )

    def list_events(self, **kwargs) -> dict:
        raise SdkUnsupportedError(
            "HTTP 开发适配器未提供事件查询端点：请使用 GrpcSkillClient.list_events"
            "（iraf.v1.EventService.ListEvents）。禁止用本地缓存或历史数据顶替事件证据。"
        )

    def get_replay_manifest(self, execution_id, *, timeout_s=None) -> dict:
        raise SdkUnsupportedError(
            "HTTP 开发适配器未提供回放清单端点：请使用 GrpcSkillClient.get_replay_manifest"
            "（iraf.v1.EventService.GetReplayManifest）。"
        )

    # -- 传输层 -------------------------------------------------------------
    def _request(self, method: str, path: str, body: Optional[Mapping], *, timeout_s=None) -> dict:
        url = self.endpoint + path
        budget = self.timeout_s if timeout_s is None else _require_timeout(timeout_s)
        headers = {"Authorization": "Bearer " + self.token, "Accept": "application/json"}
        data = None
        if body is not None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=budget) as response:
                raw = response.read()
                status = getattr(response, "status", 0)
        except urllib.error.HTTPError as exc:
            payload = self._decode_quiet(exc.read())
            raise SdkTransportError(
                f"HTTP {exc.code} 调用 {method} {url} 失败，服务端响应体未包含 IDL error_code"
                f"（响应：{payload}）。请核对端点路径、Bearer 凭据与适配器启用开关"
                "（IRAF_HTTP_DEVELOPMENT=true）；该错误码由客户端生成，不代表请求未被服务端处理。",
                http_status=int(exc.code),
                payload=payload if isinstance(payload, Mapping) else None,
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise SdkTransportError(
                f"HTTP 传输失败：{method} {url} -> {exc}。请核对地址、端口、网络与 TLS；"
                "该错误码由客户端生成，不能据此判断服务端是否已收到请求。"
            ) from exc
        payload = self._decode_quiet(raw)
        if not isinstance(payload, Mapping):
            raise SdkTransportError(
                f"HTTP {status} 调用 {method} {url} 返回的不是 JSON 对象（原始内容：{raw[:200]!r}）",
                http_status=int(status or 0),
            )
        return dict(payload)

    @staticmethod
    def _decode_quiet(raw) -> object:
        try:
            return json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError, AttributeError):
            return None


# --------------------------------------------------------------------------
# gRPC 传输（公共契约的规范入口）
# --------------------------------------------------------------------------
class GrpcSkillClient:
    """`iraf.v1.SkillRuntimeService` 与 `iraf.v1.EventService` 的薄封装。

    `grpc` 与生成的 `*_pb2_grpc` stub 在首次调用时惰性导入：缺少 grpcio 时给出
    中文可操作提示（`SdkConfigError`），而不是在 import SDK 时就失败。
    """

    def __init__(self, target, token, *, timeout_s, options: Optional[Sequence] = None):
        self.target = _require_text(target, "target")
        self.token = _require_text(token, "token")
        self.timeout_s = _require_timeout(timeout_s)
        self.options = tuple(options) if options else ()
        self._channel = None

    # -- 公共契约调用 -------------------------------------------------------
    def submit(
        self,
        skill,
        *,
        profile,
        safety_policy,
        parameters=None,
        request_id=None,
        idempotency_key=None,
        correlation_id=None,
        skill_version_constraint="",
        resource_id=None,
        controller="iraf-sdk",
        deadline_s=None,
        timeout_s=None,
    ) -> dict:
        """提交任务并返回终态反馈（`Execute` 为 unary-stream：取最后一条终态）。"""
        body = canonical_execute_body(
            skill,
            profile=profile,
            safety_policy=safety_policy,
            parameters=parameters,
            request_id=request_id,
            idempotency_key=idempotency_key,
            correlation_id=correlation_id,
            skill_version_constraint=skill_version_constraint,
            resource_id=resource_id,
            controller=controller,
            deadline_s=self.timeout_s if deadline_s is None else deadline_s,
        )
        runtime_pb2_grpc = _stub("runtime_pb2_grpc")
        stub = runtime_pb2_grpc.SkillRuntimeServiceStub(self._grpc_channel())
        budget = self.timeout_s if timeout_s is None else _require_timeout(timeout_s)
        terminal = None
        for feedback in stub.Execute(execute_request_from_body(body), timeout=budget, metadata=self._metadata()):
            terminal = feedback
        if terminal is None:
            raise SdkTransportError(
                f"gRPC Execute 未返回任何反馈（{self.target}）：服务端可能提前断开或返回了空流；"
                "请检查服务端日志与凭据，并用 GetExecution 复核是否已产生执行。"
            )
        return result_from_feedback(terminal)

    def get_execution(self, execution_id, *, timeout_s=None) -> dict:
        identifier = _require_text(execution_id, "execution_id")
        skill_pb2 = _stub("skill_pb2")
        runtime_pb2_grpc = _stub("runtime_pb2_grpc")
        stub = runtime_pb2_grpc.SkillRuntimeServiceStub(self._grpc_channel())
        snapshot = stub.GetExecution(
            skill_pb2.GetExecutionRequest(execution_id=identifier),
            timeout=self._budget(timeout_s),
            metadata=self._metadata(),
        )
        return result_from_snapshot(snapshot)

    def cancel(self, execution_id, reason, *, timeout_s=None) -> dict:
        identifier = _require_text(execution_id, "execution_id")
        reason_text = _require_text(reason, "reason")
        skill_pb2 = _stub("skill_pb2")
        runtime_pb2_grpc = _stub("runtime_pb2_grpc")
        stub = runtime_pb2_grpc.SkillRuntimeServiceStub(self._grpc_channel())
        response = stub.Cancel(
            skill_pb2.CancelExecutionRequest(execution_id=identifier, reason=reason_text),
            timeout=self._budget(timeout_s),
            metadata=self._metadata(),
        )
        # 服务端 `CancelExecutionResponse` 只有 accepted/state：这里不编造 error_code。
        # 该返回字典**不是**任务结果形状，调用方必须按 `accepted` 分支判读。
        return {
            "accepted": bool(response.accepted),
            "status": status_from_state(response.state),
            "reason": reason_text,
        }

    def list_events(
        self,
        *,
        execution_id=None,
        object_id=None,
        since_unix_ms=0,
        until_unix_ms=0,
        limit=100,
        page_token="",
        timeout_s=None,
    ) -> dict:
        """查询事件（只读）。`limit` 上界与服务端一致（<= 1000）。"""
        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0 or limit > MAX_EVENT_LIMIT:
            raise SdkConfigError(f"limit 必须是 1..{MAX_EVENT_LIMIT} 的整数，实际：{limit!r}（服务端上界见 events_grpc.py）")
        events_pb2 = _stub("events_pb2")
        events_pb2_grpc = _stub("events_pb2_grpc")
        stub = events_pb2_grpc.EventServiceStub(self._grpc_channel())
        response = stub.ListEvents(
            events_pb2.ListEventsRequest(
                execution_id=execution_id or "",
                object_id=object_id or "",
                since_unix_ms=int(since_unix_ms),
                until_unix_ms=int(until_unix_ms),
                limit=int(limit),
                page_token=page_token or "",
            ),
            timeout=self._budget(timeout_s),
            metadata=self._metadata(),
        )
        return {
            "events": [event_to_dict(item) for item in response.events],
            "next_page_token": response.next_page_token,
        }

    def get_replay_manifest(self, execution_id, *, timeout_s=None) -> dict:
        identifier = _require_text(execution_id, "execution_id")
        events_pb2 = _stub("events_pb2")
        events_pb2_grpc = _stub("events_pb2_grpc")
        stub = events_pb2_grpc.EventServiceStub(self._grpc_channel())
        manifest = stub.GetReplayManifest(
            events_pb2.GetReplayManifestRequest(execution_id=identifier),
            timeout=self._budget(timeout_s),
            metadata=self._metadata(),
        )
        return replay_manifest_to_dict(manifest)

    # -- 传输层 -------------------------------------------------------------
    def close(self) -> None:
        if self._channel is not None:
            self._channel.close()
            self._channel = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
        return False

    def _budget(self, timeout_s) -> float:
        return self.timeout_s if timeout_s is None else _require_timeout(timeout_s)

    def _metadata(self):
        # 身份只经受信 transport 传递：客户端只发凭据，不发送任何自声明身份字段。
        return (("authorization", "Bearer " + self.token),)

    def _grpc_channel(self):
        grpc = _grpc_module()
        if self._channel is None:
            self._channel = grpc.insecure_channel(self.target, options=self.options)
        return self._channel
