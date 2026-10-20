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
    event_to_dict,
    execute_request_from_body,
    identity_of,
    replay_manifest_to_dict,
    result_from_feedback,
    result_from_snapshot,
    status_from_state,
)

# 版本单一来源（铁律 5.3）：真实值只在 pyproject.toml 的 project.version。
# 这里从**已安装元数据**读取，避免再复制一份版本号字面量（原先三处硬编码，
# 2026-10-20 固化时合并）。源码树未安装时给 dev 标记，不冒充发布版本。
try:
    from importlib.metadata import PackageNotFoundError as _PNF, version as _pkg_version

    try:
        __version__ = _pkg_version("intewell-iraf")
    except _PNF:
        # 直接从源码树运行（未 pip install）时，回到唯一来源 pyproject.toml 读，
        # 而不是复制一个版本号字面量（否则又会与 pyproject 分叉）。
        __version__ = "0.0.0+dev"
        try:
            import re as _re
            from pathlib import Path as _Path

            for _parent in _Path(__file__).resolve().parents:
                _pyproject = _parent / "pyproject.toml"
                if _pyproject.is_file():
                    _match = _re.search(r'(?m)^version\s*=\s*"([^"]+)"',
                                        _pyproject.read_text(encoding="utf-8"))
                    if _match:
                        __version__ = _match.group(1)
                    break
        except Exception:  # noqa: BLE001 - 版本兜底失败不应阻断导入
            pass
except ImportError:  # pragma: no cover - Python < 3.8 不在支持范围
    __version__ = "0.0.0+dev"

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
    "event_to_dict",
    "execute_request_from_body",
    "identity_of",
    "replay_manifest_to_dict",
    "result_from_feedback",
    "result_from_snapshot",
    "status_from_state",
]
