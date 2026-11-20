"""VLA 适配层：HTTP 推理客户端 + 动作安全护栏。

设计边界（与 IRAF 铁律一致）：
- 本层只做“观测 -> 动作提案”和“提案过滤”，不直接发执行器命令；
- 动作必须由 Policy Gateway / Controller Gate 再校验；
- VLA 不可用时显式失败，回退由上层决定。
"""
from .client import VlaClient, VlaError, VlaResult
from .guard import ActionGuard, GuardResult, VlaLimits

__all__ = ["VlaClient", "VlaError", "VlaResult", "ActionGuard", "GuardResult", "VlaLimits"]
