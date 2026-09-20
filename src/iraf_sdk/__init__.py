"""IRAF Python SDK（纯 Python，跨架构；决策 1.A 本轮只做 Python SDK）。

分层定位（`docs/iraf-multiplatform-sdk-design.md` §4 产物 1）：

- `iraf_sdk.errors`：错误码 → 中文可操作诊断（唯一错误码映射，来源见该模块 docstring）；
- `iraf_sdk.client`：既有公共契约的薄封装（HTTP 开发适配器 / gRPC），无旁路；
- 本模块只做再导出，**不 import** mujoco / grpc / iraf_core / iraf_adapters，
  因此 `pip install iraf_sdk` 后可在目标端独立使用（aarch64 目标端只需标准库）。

导入本包不会加载任何重依赖：

    import sys, iraf_sdk
    assert "mujoco" not in sys.modules and "grpc" not in sys.modules

典型用法见 `iraf_sdk.client` 的模块 docstring（含最小可运行示例）。
"""

from .errors import (
    ERROR_SPECS,
    KNOWN_ERROR_CODES,
    TRANSPORT_FAILED_CODE,
    UNKNOWN_CODE_LABEL,
    Diagnosis,
    ErrorSpec,
    IrafError,
    SdkContractError,
    describe,
    is_retryable,
    raise_for_status,
    spec_for,
)
from .client import (
    HTTP_TASKS_PATH,
    MAX_EVENT_LIMIT,
    GrpcSkillClient,
    HttpSkillClient,
    SdkConfigError,
    SdkTransportError,
    SdkUnsupportedError,
    canonical_execute_body,
    deadline_after,
    identity_of,
)

__version__ = "0.2.0"

__all__ = [
    "__version__",
    "ERROR_SPECS",
    "KNOWN_ERROR_CODES",
    "TRANSPORT_FAILED_CODE",
    "UNKNOWN_CODE_LABEL",
    "Diagnosis",
    "ErrorSpec",
    "IrafError",
    "SdkContractError",
    "SdkConfigError",
    "SdkTransportError",
    "SdkUnsupportedError",
    "describe",
    "is_retryable",
    "raise_for_status",
    "spec_for",
    "HTTP_TASKS_PATH",
    "MAX_EVENT_LIMIT",
    "GrpcSkillClient",
    "HttpSkillClient",
    "canonical_execute_body",
    "deadline_after",
    "identity_of",
]
