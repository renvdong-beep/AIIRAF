"""IRAF SDK 错误码 → 中文可操作诊断（纯标准库；不依赖 mujoco / grpc / iraf_core）。

契约来源（唯一事实来源，禁止在本文件之外另立错误码）：

1. `api/proto/iraf/v1/skill.proto` 的 `IrafError`：`code` / `message_zh` / `retryable`；
2. `docs/iraf-idl-runtime-detailed-design.md` §5「标准错误码与测试」错误码表（含"是否重试"）；
3. `docs/iraf-api-contract-catalog.md` 中已声明的流控类错误码；
4. `src/iraf_core/*`、`src/iraf_adapters/*` 的实际抛出点（每个码在 `ErrorSpec.emitter`
   逐条登记 `路径:行号`，行号以 git `0fdb7d8` 为准；`tests/unit/test_sdk_errors.py` 会校验
   "IDL 表里的每个码都在本映射内" 与 "src/ 里出现的每个 IRAF- 码都在本映射内"）。

设计要点（fail-closed，对应 AGENTS.md 铁律 1.5 / 2.3 / 2.4）：

1. **未知错误码不得静默返回原文**：必须显式标注「未知错误码」，服务端原文只放进
   `server_reason`，绝不冒充诊断结论，也绝不按已知码自动重试。
2. 终态失败（FAILED/ABORTED/SAFETY_STOP/CANCELLED）必须携带 `error_code` 字段；
   **字段缺失**即契约违反，显式抛 `SdkContractError`（不允许默认成功、不允许吞掉错误）。
   `error_code` 字段存在但为空字符串表示"显式无错误"（例如取消成功的响应），不算失败。
3. `provenance` 逐码标注来源（`idl+code` / `idl-only` / `api-catalog-only` / `code-only`），
   契约漂移可直接审计：`idl-only` 表示契约已声明但实现未落地，不得据此假设可触发。
4. `retryable` 只在有依据时为 True；SDK **不自动重试**（无无限重试）。重试必须由
   TaskFlow/调用方发起，并创建新的 execution（铁律 2.4 idempotency / 终态不可复活）。
5. 服务端若显式给出 `retryable`，以服务端为准（`server_retryable`），契约默认值退居
   `spec.retryable`；两者不一致时在诊断文本中并列显示，便于发现契约漂移。
"""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

__all__ = [
    "PROVENANCE_IDL_AND_CODE",
    "PROVENANCE_IDL_ONLY",
    "PROVENANCE_API_CATALOG_ONLY",
    "PROVENANCE_CODE_ONLY",
    "PROVENANCE_LABELS",
    "UNKNOWN_CODE_LABEL",
    "TERMINAL_FAILURE_STATUSES",
    "NON_TERMINAL_STATUSES",
    "TRANSPORT_FAILED_CODE",
    "ErrorSpec",
    "Diagnosis",
    "IrafError",
    "SdkContractError",
    "ERROR_SPECS",
    "KNOWN_ERROR_CODES",
    "describe",
    "spec_for",
    "is_retryable",
    "raise_for_status",
]

ERROR_PREFIX = "IRAF-"
UNKNOWN_CODE_LABEL = "未知错误码"
TRANSPORT_FAILED_CODE = "IRAF-TRANSPORT-FAILED"

# 终态（`api/proto/iraf/v1/skill.proto` SkillState 的终态集合，不含 UNSPECIFIED）
TERMINAL_FAILURE_STATUSES = frozenset({"FAILED", "ABORTED", "CANCELLED", "SAFETY_STOP"})
NON_TERMINAL_STATUSES = frozenset({"PENDING", "VALIDATING", "RUNNING"})
SUCCESS_STATUSES = frozenset({"SUCCEEDED"})

PROVENANCE_IDL_AND_CODE = "idl+code"
PROVENANCE_IDL_ONLY = "idl-only"
PROVENANCE_API_CATALOG_ONLY = "api-catalog-only"
PROVENANCE_CODE_ONLY = "code-only"

PROVENANCE_LABELS = {
    PROVENANCE_IDL_AND_CODE: "IDL §5 错误码表已声明，且仓库实现中已有抛出点",
    PROVENANCE_IDL_ONLY: "IDL 已声明，实现尚未落地：不得据此假设当前一定能触发",
    PROVENANCE_API_CATALOG_ONLY: "API 契约目录已声明，实现尚未落地",
    PROVENANCE_CODE_ONLY: "实现已抛出，但 IDL §5 错误码表尚未收录（契约缺口，需补表）",
    "unregistered": "未登记：既不在 IDL 错误码表，也不在本仓实现中（按未知错误码处理，不猜测）",
}


@dataclass(frozen=True)
class ErrorSpec:
    """单个错误码的契约声明与中文诊断。"""

    code: str
    summary_zh: str
    suggestion_zh: str
    retryable: bool
    retry_policy_zh: str
    provenance: str
    emitter: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Diagnosis:
    """一次诊断结果；`known=False` 表示该错误码未登记（未知错误码）。"""

    code: str
    known: bool
    summary_zh: str
    suggestion_zh: str
    retryable: bool
    retry_policy_zh: str
    provenance: str
    server_reason: str
    server_retryable: Optional[bool] = None
    contract_retryable: Optional[bool] = None

    def to_dict(self) -> dict:
        return {
            "code": self.code,
            "known": self.known,
            "summary_zh": self.summary_zh,
            "suggestion_zh": self.suggestion_zh,
            "retryable": self.retryable,
            "retry_policy_zh": self.retry_policy_zh,
            "provenance": self.provenance,
            "server_reason": self.server_reason,
            "server_retryable": self.server_retryable,
            "contract_retryable": self.contract_retryable,
        }

    def format_zh(self) -> str:
        """人可读的中文诊断（含错误码、诊断/下一步、重试判读、服务端原文、来源）。"""
        if self.known:
            lines = [f"[{self.code}] {self.summary_zh}", f"  下一步：{self.suggestion_zh}"]
        else:
            lines = [
                f"[{self.code}] {UNKNOWN_CODE_LABEL}",
                f"  诊断：{self.suggestion_zh}",
            ]
        retry_text = "可重试" if self.retryable else "不可重试"
        lines.append(f"  重试：{retry_text}（{self.retry_policy_zh}）")
        if (
            self.server_retryable is not None
            and self.contract_retryable is not None
            and self.server_retryable != self.contract_retryable
        ):
            lines.append(
                f"  契约漂移提示：服务端上报 retryable={self.server_retryable}，"
                f"与 IDL 契约默认值 {self.contract_retryable} 不一致"
            )
        lines.append(f"  服务端原文：{self.server_reason or '（无）'}")
        lines.append(f"  来源：{self.provenance}（{PROVENANCE_LABELS.get(self.provenance, '')}）")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# 错误码表：19 个码 = IDL §5 表中的 9 个（其中 7 个实现已落地）+ API 目录 1 个
# + 实现已抛出但表格未收录的 9 个。顺序与 IDL §5 表一致，便于人工比对。
# ---------------------------------------------------------------------------
ERROR_SPECS: Tuple[ErrorSpec, ...] = (
    # --- IDL §5 错误码表（9 项）------------------------------------------------
    ErrorSpec(
        code="IRAF-INPUT-INVALID",
        summary_zh="参数不符合 Skill manifest 的 inputs schema",
        suggestion_zh=(
            "按该 Skill manifest 的 inputs schema 逐字段修正参数（必填项、类型、枚举、取值范围、"
            "未知字段）；schema 可通过 `iraf skill describe` 或 skills/<skill>/<skill>.output.json 获取。"
            "这属于校验失败，重试同一参数必然再次被拒。"
        ),
        retryable=False,
        retry_policy_zh="否（IDL §5：参数不符合 manifest schema）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=(
            "src/iraf_core/policy.py:18",
            "src/iraf_core/policy.py:21",
            "src/iraf_core/policy.py:23",
            "src/iraf_adapters/http/runtime_http.py:49",
        ),
    ),
    ErrorSpec(
        code="IRAF-POLICY-DENIED",
        summary_zh="RBAC / RobotProfile / SafetyPolicy 任一环节拒绝",
        suggestion_zh=(
            "按 reason 定位拒绝环节并逐项核对：调用方 roles 是否含 task.submit；RobotProfile 与 "
            "SafetyPolicy 的 name/version/digest 是否与已签名声明一致；Skill 是否在 "
            "safety_policy.allowed_skills 内；duration_ms 是否超过 min(manifest.timeout, "
            "max_duration_ms)；仿真声明是否满足（未验证配置只能用于 simulation=true）。"
            "这是策略判定，原地重试无效。"
        ),
        retryable=False,
        retry_policy_zh="否（IDL §5：RBAC/Profile/Safety Policy 拒绝）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=(
            "src/iraf_core/policy.py:17",
            "src/iraf_core/policy.py:24",
            "src/iraf_core/policy.py:32",
            "src/iraf_core/runtime.py:213",
        ),
    ),
    ErrorSpec(
        code="IRAF-PRECONDITION-FAILED",
        summary_zh="状态、标定或能力等前置条件不满足",
        suggestion_zh=(
            "读 manifest.preconditions，用运行时清单对照缺失键（如 safety.estop、mode、标定状态）；"
            "先满足前置条件（复位急停、进入正确模式、完成标定/标定文件到位），再创建新的 execution "
            "重试；前置由环境决定，盲目重试无效。"
        ),
        retryable=False,
        retry_policy_zh="视 Skill 而定（本 SDK 取保守值 False，由 TaskFlow 决定）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=("src/iraf_core/policy.py:38", "src/iraf_core/policy.py:41", "src/iraf_core/policy.py:44"),
    ),
    ErrorSpec(
        code="IRAF-SKILL-PROVIDER-UNAVAILABLE",
        summary_zh="无健康 Provider（未注册 / 版本不匹配 / 能力缺失）",
        suggestion_zh=(
            "依次确认：Skill 是否已注册且版本满足 skill_version_constraint；Provider 是否健康；"
            "RobotProfile 是否声明了 manifest.requires 的全部能力（能力缺失也用本码返回）。"
            "Provider 恢复后在 deadline 内创建新 execution 重试。"
        ),
        retryable=True,
        retry_policy_zh="是，受 deadline 限制（IDL §5）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=("src/iraf_core/policy.py:30", "src/iraf_core/runtime.py:88"),
    ),
    ErrorSpec(
        code="IRAF-RESOURCE-BUSY",
        summary_zh="独占资源已被其他执行租用",
        suggestion_zh=(
            "目标资源（如 piper.arm、quadruped.base）已被其他 execution 持有租约；按 TaskFlow 策略"
            "等待释放或重排，并只使用带有效期的最新 fencing token（旧 token 会被 adapter 拒绝）。"
            "禁止无限等待。"
        ),
        retryable=True,
        retry_policy_zh="是，受 TaskFlow 策略限制（IDL §5）",
        provenance=PROVENANCE_IDL_ONLY,
        emitter=(),
    ),
    ErrorSpec(
        code="IRAF-IDEMPOTENCY-CONFLICT",
        summary_zh="同一幂等键对应了不同请求",
        suggestion_zh=(
            "该 idempotency_key 已绑定另一份请求摘要；不要复用该键。若确需重发，请使用新的 "
            "idempotency_key 与新的 execution（同一键 + 同摘要会直接返回历史结果）。"
        ),
        retryable=False,
        retry_policy_zh="否（IDL §5：同一幂等键对应不同请求）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=("src/iraf_core/runtime.py:68", "src/iraf_core/runtime.py:230"),
    ),
    ErrorSpec(
        code="IRAF-DEADLINE-EXCEEDED",
        summary_zh="任务截止时间已过（运行超时）",
        suggestion_zh=(
            "deadline 已过期；重新评估动作时长与超时预算（duration_ms、timeout_seconds）后再由 "
            "TaskFlow 创建新 execution 并关联原 execution_id；不要原地重试。"
        ),
        retryable=False,
        retry_policy_zh="否，转恢复/接管（IDL §5）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=("src/iraf_core/policy.py:19",),
    ),
    ErrorSpec(
        code="IRAF-CANCELLED",
        summary_zh="执行已被调用方或编排器取消",
        suggestion_zh=(
            "取消来源可能是调用方、编排器或 gRPC 客户端断连（服务端会自动取消）；确认来源后再决定"
            "是否新建 execution。取消是终态，不得被回写为成功。"
        ),
        retryable=False,
        retry_policy_zh="否（IDL §5：由调用者/编排器取消）",
        provenance=PROVENANCE_IDL_AND_CODE,
        emitter=("src/iraf_core/runtime.py:174",),
    ),
    ErrorSpec(
        code="IRAF-SAFETY-STOP",
        summary_zh="安全事件中断了执行",
        suggestion_zh=(
            "资源须进入 QUARANTINED，等待控制器确认 safe state 且授权人员复位后才能释放或重新调度"
            "（铁律 1.9）；禁止自动重试，禁止把终态改回成功。当前实现以 IRAF-SAFETY-QUARANTINED + "
            "终态 SAFETY_STOP 表达同一语义。"
        ),
        retryable=False,
        retry_policy_zh="否，须受控复位（IDL §5）",
        provenance=PROVENANCE_IDL_ONLY,
        emitter=(),
    ),
    # --- API 契约目录已声明（1 项）-------------------------------------------
    ErrorSpec(
        code="IRAF-STREAM-LAGGED",
        summary_zh="事件流消费者滞后，事件被丢弃（流控保护）",
        suggestion_zh=(
            "用响应中给出的可恢复 page_token 重新拉取；调大消费者缓冲或缩小订阅范围（execution_id/"
            "object_id/时间窗）；慢消费者不得拖垮 Runtime。"
        ),
        retryable=False,
        retry_policy_zh="否；会话可恢复（重拉，不重放动作）",
        provenance=PROVENANCE_API_CATALOG_ONLY,
        emitter=(),
    ),
    # --- 实现已抛出但 IDL §5 表格未收录（9 项，契约缺口）---------------------
    ErrorSpec(
        code="IRAF-UNAUTHENTICATED",
        summary_zh="缺少受信传输身份，或凭据被拒绝",
        suggestion_zh=(
            "身份只能来自受信 transport（mTLS / 经验证 JWT / 受信网关），请求体中自声明的 caller、"
            "role、tenant 一律不被信任（铁律 1.8）；检查 Bearer 凭据、subject 与 transport 配置。"
        ),
        retryable=False,
        retry_policy_zh="否；未通过身份校验，重试同一凭据无效",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=(
            "src/iraf_core/policy.py:16",
            "src/iraf_core/runtime.py:52",
            "src/iraf_core/runtime.py:209",
        ),
    ),
    ErrorSpec(
        code="IRAF-EXECUTION-CONFLICT",
        summary_zh="同一 execution 已处于活动状态",
        suggestion_zh=(
            "该 execution_id 上已有活动执行，禁止并发下发；先用 GetExecution 查询状态，确认前一执行"
            "到达终态后再决定是否新建 execution。"
        ),
        retryable=False,
        retry_policy_zh="否；需先确认既有执行终态",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_core/runtime.py:59",),
    ),
    ErrorSpec(
        code="IRAF-EXECUTION-NOT-ACTIVE",
        summary_zh="目标 execution 不是活动态（不存在或已终态）",
        suggestion_zh=(
            "用 GetExecution 查询执行现状；终态执行不得再取消或接受控制请求。若需重做，请由 TaskFlow "
            "创建新的 execution 并关联原 execution_id。"
        ),
        retryable=False,
        retry_policy_zh="否；终态不可复活",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_core/runtime.py:140", "src/iraf_core/runtime.py:145"),
    ),
    ErrorSpec(
        code="IRAF-EXECUTION-FAILED",
        summary_zh="执行失败（后端 / Provider 抛出异常）",
        suggestion_zh=(
            "读 reason 与事件流定位失败环节（后端异常、场景/配置缺失、运动学不可达、标定错误等）；"
            "修复后由 TaskFlow 新建 execution，禁止把失败终态改写为成功（铁律 1.5）。"
        ),
        retryable=False,
        retry_policy_zh=(
            "IDL §5 未收录；实现层统一上报 retryable=false"
            "（src/iraf_adapters/grpc/runtime_grpc.py:38），是否重试由 TaskFlow 决定（须新建 execution）"
        ),
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_core/runtime.py:126",),
    ),
    ErrorSpec(
        code="IRAF-SAFETY-QUARANTINED",
        summary_zh="安全事件导致资源隔离，后续动作被拒绝",
        suggestion_zh=(
            "按铁律 1.9 处理：控制器确认 safe state 且授权复位前，相关资源保持在 QUARANTINED，不得释放"
            "或重新调度；禁止自动重试，禁止以重试绕过隔离。"
        ),
        retryable=False,
        retry_policy_zh="否；须受控复位",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_core/runtime.py:119",),
    ),
    ErrorSpec(
        code="IRAF-CANCEL-STOP-FAILED",
        summary_zh="取消请求未能让设备安全停止",
        suggestion_zh=(
            "立即人工介入：确认设备实际状态与 safe state，必要时触发急停；该码表示「取消但没停下」，"
            "绝不可按成功处理，也不得自动重试。"
        ),
        retryable=False,
        retry_policy_zh="否；须人工介入确认设备状态",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_core/runtime.py:151", "src/iraf_core/runtime.py:174"),
    ),
    ErrorSpec(
        code="IRAF-INTENT-PARSE-FAILED",
        summary_zh="自然语言意图无法解析为合法的 Skill 请求",
        suggestion_zh=(
            "检查 AgentOS 意图提供方输出是否符合可解析 schema（技能名、必填参数、参数类型）；补齐后"
            "重新提交意图；解析失败不得静默降级为默认动作（铁律 1.5）。"
        ),
        retryable=False,
        retry_policy_zh="否；需先修正意图或提供方输出",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_adapters/agentos/bridge.py:43",),
    ),
    ErrorSpec(
        code="IRAF-INTERNAL",
        summary_zh="运行时/适配器内部未分类错误",
        suggestion_zh=(
            "携带 reason 与 correlation_id 报障，并检查对应适配器日志；这是未分类错误，不要据此自动"
            "重试（可能重复物理动作）。"
        ),
        retryable=False,
        retry_policy_zh="否；须先定位内部错误原因",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("src/iraf_adapters/http/runtime_http.py:50",),
    ),
    ErrorSpec(
        code=TRANSPORT_FAILED_CODE,
        summary_zh="客户端侧传输失败（网络/超时/TLS），或服务端未返回 IDL 错误码",
        suggestion_zh=(
            "检查端点地址、端口、网络与 TLS 配置；该码由客户端生成，**不代表服务端已收到请求**。"
            "重试前先用幂等键与 GetExecution 确认是否已执行，避免重复物理动作。"
        ),
        retryable=True,
        retry_policy_zh="是（需人工判断：先确认请求是否已送达，再用新 execution 重试）",
        provenance=PROVENANCE_CODE_ONLY,
        emitter=("scripts/verify_minimal_chain.py:52",),
    ),
)

ERROR_SPECS_BY_CODE = {spec.code: spec for spec in ERROR_SPECS}
KNOWN_ERROR_CODES: Tuple[str, ...] = tuple(spec.code for spec in ERROR_SPECS)


class SdkContractError(ValueError):
    """SDK 与服务端契约不一致（响应缺少必需字段、字段类型不符）。

    这是"显式失败"而不是"降级为成功"：调用方必须能看到契约被破坏。
    """


class IrafError(Exception):
    """带中文诊断的 IRAF 失败（错误码来自服务端或客户端传输层）。"""

    def __init__(self, diagnosis: Diagnosis, payload: Optional[Mapping] = None):
        super().__init__(diagnosis.format_zh())
        self.diagnosis = diagnosis
        self.payload = dict(payload) if isinstance(payload, Mapping) else None

    @property
    def code(self) -> str:
        return self.diagnosis.code

    @property
    def retryable(self) -> bool:
        return self.diagnosis.retryable

    @property
    def reason(self) -> str:
        return self.diagnosis.server_reason

    @property
    def known(self) -> bool:
        return self.diagnosis.known

    def to_dict(self) -> dict:
        return {"error_code": self.code, "reason": self.reason, "diagnosis": self.diagnosis.to_dict()}

    @classmethod
    def from_payload(cls, payload: Mapping) -> "IrafError":
        """由失败的终端响应构造。成功响应/缺字段属于误用，显式抛 SdkContractError。"""
        if not isinstance(payload, Mapping):
            raise SdkContractError(f"响应必须是对象（JSON object），实际类型：{type(payload).__name__}")
        code = payload.get("error_code")
        if code is None:
            raise SdkContractError("失败响应缺少 error_code 字段：契约要求终态携带 error_code（禁止默认成功）")
        if not isinstance(code, str) or not code.strip():
            raise SdkContractError(f"error_code 必须是非空字符串，实际：{code!r}")
        server_reason = payload.get("reason") or ""
        if not isinstance(server_reason, str):
            server_reason = str(server_reason)
        diagnosis = describe(code.strip(), server_reason, payload.get("retryable"))
        return cls(diagnosis, payload)


def spec_for(code: str) -> Optional[ErrorSpec]:
    """返回错误码的契约声明；未登记返回 None。"""
    if not isinstance(code, str):
        return None
    return ERROR_SPECS_BY_CODE.get(code.strip())


def describe(code: str, server_reason: str = "", server_retryable: Optional[bool] = None) -> Diagnosis:
    """把错误码翻译为中文诊断 + 下一步建议。

    未知错误码：显式标注「未知错误码」，保留服务端原文，不猜测、不自动重试。
    """
    if not isinstance(code, str) or not code.strip():
        raise SdkContractError(f"错误码必须是非空字符串，实际：{code!r}")
    normalized = code.strip()
    reason = server_reason if isinstance(server_reason, str) else str(server_reason or "")
    spec = ERROR_SPECS_BY_CODE.get(normalized)
    if spec is None:
        return Diagnosis(
            code=normalized,
            known=False,
            summary_zh=UNKNOWN_CODE_LABEL,
            suggestion_zh=(
                "该错误码未在 IDL §5 错误码表、API 契约目录与本仓实现中登记，SDK 无法给出诊断结论："
                "既未按任何已知码解释，也未自动重试。请先核对服务端版本与 IDL 兼容性"
                "（api/proto/iraf/v1/*.proto 与 iraf_sdk.KNOWN_ERROR_CODES），确认后再决定是否重试。"
            ),
            retryable=False,
            retry_policy_zh="未知码不做任何重试假设（保守取 False）",
            provenance="unregistered",
            server_reason=reason,
            server_retryable=server_retryable if isinstance(server_retryable, bool) else None,
            contract_retryable=None,
        )
    effective_retryable = server_retryable if isinstance(server_retryable, bool) else spec.retryable
    return Diagnosis(
        code=spec.code,
        known=True,
        summary_zh=spec.summary_zh,
        suggestion_zh=spec.suggestion_zh,
        retryable=effective_retryable,
        retry_policy_zh=spec.retry_policy_zh,
        provenance=spec.provenance,
        server_reason=reason,
        server_retryable=server_retryable if isinstance(server_retryable, bool) else None,
        contract_retryable=spec.retryable,
    )


def is_retryable(code: str) -> bool:
    """契约默认的重试判读；未知码一律 False（不猜测）。"""
    spec = spec_for(code)
    return bool(spec and spec.retryable)


def raise_for_status(payload: Mapping) -> Mapping:
    """按公共契约判读一次任务响应：失败即抛 `IrafError`，否则原样返回。

    规则（与 `src/iraf_core/runtime.py::SkillRuntime._result` 的响应形状一致）：

    - `error_code` 存在且非空 ⇒ 失败，抛 `IrafError`（含中文诊断）；
    - `error_code` 存在且为空字符串 ⇒ 显式无错误（例如取消成功的响应），正常返回；
    - `error_code` **字段缺失** ⇒ 契约违反，抛 `SdkContractError`（禁止默认成功）；
    - 非终态（PENDING/VALIDATING/RUNNING）且无 error_code ⇒ 正常返回，由调用方继续查询。
    """
    if not isinstance(payload, Mapping):
        raise SdkContractError(f"响应必须是对象（JSON object），实际类型：{type(payload).__name__}")
    status = payload.get("status")
    if not isinstance(status, str) or not status.strip():
        raise SdkContractError(f"响应缺少合法的 status 字段：实际 {status!r}")
    status = status.strip()
    known_status = TERMINAL_FAILURE_STATUSES | NON_TERMINAL_STATUSES | SUCCESS_STATUSES
    if status not in known_status:
        raise SdkContractError(
            f"未知 status：{status!r}；仅接受 {sorted(known_status)}（skill.proto SkillState 的终态/中间态）"
        )
    if "error_code" not in payload:
        raise SdkContractError(
            f"status={status} 的响应缺少 error_code 字段：契约要求显式给出（空字符串表示无错误）"
        )
    code = payload["error_code"]
    if code is None:
        raise SdkContractError(
            f"status={status} 的响应把 error_code 写成了 null：契约要求字符串（空字符串表示无错误，"
            "null 属于字段类型违约）"
        )
    if isinstance(code, str) and not code.strip():
        return payload
    if not isinstance(code, str):
        raise SdkContractError(f"error_code 必须是字符串，实际：{code!r}")
    raise IrafError.from_payload(payload)
