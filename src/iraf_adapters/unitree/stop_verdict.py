"""`stop`(damped_hold) 报告的**判读口径**（唯一实现，多个消费者共用）。

为什么单独一层：这个口径已经错过两次，两次都是"按**不存在的键**判定"⇒ 把成功判成失败
（假失败，等价于伪造状态，AGENTS.md 铁律 1.5/1.6）：

  · `dock_for_handoff` 的保持段：曾按 `static_entered` 判定 —— 该键是 **loopback 验收脚本**
    按样本自己算的，`damped_hold` 报告里根本没有 ⇒ 实测末速 2.461973e-04 m/s 却被判"未进入静止段"；
  · `locomote` 的 `MpcUnavailableError` 失败路径：同一个 `static_entered` ⇒ `damped_hold` 真正
    停稳也会把终态写成 `FAILED`。

本层只做一件事：从报告里**实际存在的**键判定"静态保持是否成立"，缺键即**显式失败**
（不猜、不默认成功）。
"""

from __future__ import annotations

__all__ = ["StopReportError", "static_hold_failure"]


class StopReportError(KeyError):
    """`stop` 报告不符合契约（缺判定键）。缺键不得被当作"成功"。"""


#: 判定"静态保持是否成立"所必需的键（`damped_hold` 报告实测含这两个）。
REQUIRED_KEYS = ("failure_reason", "final_speed_mps")


def static_hold_failure(report):
    """返回静态保持的失败原因；`None` = 保持成立。

    规则：
      * `failure_reason` 非空字符串 ⇒ 返回它（这是 `stop` 自己给出的失败判定）；
      * 空串 / `None` ⇒ 保持成立，返回 `None`；
      * 缺 `failure_reason` 或缺 `final_speed_mps` ⇒ `StopReportError`（**不假定成功**）。
    """
    if not isinstance(report, dict):
        raise StopReportError("stop 报告必须是字典，实际 %r" % (type(report).__name__,))
    missing = [key for key in REQUIRED_KEYS if key not in report]
    if missing:
        raise StopReportError(
            "stop 报告缺少判定键 %s（实际键：%s）：缺键不得当作成功"
            % (missing, sorted(report))
        )
    reason = report.get("failure_reason")
    if reason is None:
        return None
    reason = str(reason)
    return reason if reason else None
