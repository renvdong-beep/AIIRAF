"""实时性能测试 Provider（工程/诊断类）。

与其他 Provider 的关键差异：
- 不调用 backend（构造参数只为满足 Runtime 统一签名），不申请运动租约，不产生任何物理动作；
- 实际执行在 **RTOS/运控域**：由本 Provider（边缘侧）通过 SSH 执行固定命令 cyclictest；
  RTOS 内**不安装**任何 IRAF 进程，避免干扰实时性；
- 目标地址/账号/密钥来自配置（`IRAF_RT_TARGET_*`），不写死。
"""
from __future__ import annotations

import os

from iraf_adapters.tooling.rt_latency import RtTarget, RtTestError, run_test


class RtLatencyProvider:
    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease):
        target = RtTarget.from_env()
        if inputs.get("target_host"):
            target.host = str(inputs["target_host"]).strip()
        if inputs.get("target_user"):
            target.user = str(inputs["target_user"]).strip()
        try:
            result = run_test(
                duration_s=int(inputs.get("duration_s") or 10),
                interval_us=int(inputs.get("interval_us") or 200),
                priority=int(inputs.get("priority") or 240),
                histogram_us=int(inputs.get("histogram_us") or 100),
                target=target,
            )
            result["kind"] = "diagnostic"
            return result
        except RtTestError as exc:
            return {"status": "FAILED", "error_code": exc.code, "reason": exc.detail,
                    "kind": "diagnostic"}