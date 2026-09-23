"""Provider 运行时编排（A6a-3）：把「独立进程客户端 + 协议 + 新鲜度门禁」缝成一个 `step()`。

契约：`docs/debug/2026-09-23-locomote-provider-integration-spec.md` §2/§3/§4。

设计要点（每条对着契约或铁律）：
  · **更新率与消费分离**：每 `ticks_per_update` 个控制拍才真正调一次子进程；其余拍走**消费侧**
    门禁（复用 `freshness.decide`）⇒ 生产 50 Hz、消费 100 Hz（契约 §3）。
  · **失败即显式失败**：子进程超时/崩溃/响应非法由 `MpcProcessClient` 统一映射为
    `damped_hold` 且 `z=None`；本层再把门禁裁定叠加上去，**绝不复用上一拍的解**（契约 §2）。
  · **决策可追溯**：每次返回带 `decision/reason/flags/diagnostics`（解龄、状态分类、迭代数、
    求解耗时、是否越界），供报告与告警引用。
  · 时间可注入（`now_fn`）⇒ 单测可复现、无 flakiness。
"""

from __future__ import annotations

import time

from . import protocol as pr
from .freshness import DECISION_OK, decide

__all__ = ["ProviderRuntime"]


class ProviderRuntime:
    """一次控制拍的 MPC 消费逻辑（不碰控制器；调用方据 `decision` 决定用解或走 damped_hold）。"""

    def __init__(self, client, ticks_per_update=2, now_fn=None):
        if int(ticks_per_update) < 1:
            raise ValueError("ticks_per_update 必须 ≥ 1，实际 %r" % (ticks_per_update,))
        self._client = client
        self._ticks_per_update = int(ticks_per_update)
        self._now = now_fn or (lambda: time.perf_counter() * 1e3)
        self._tick = 0
        self._last_ok_ms = None
        self._solution = None
        self._last_status = "unknown"
        self.stats = {"steps": 0, "updates": 0, "skips": 0, "holds": 0, "releases": 0}

    # ---- 只读 ----
    @property
    def has_solution(self):
        return self._solution is not None and self._last_ok_ms is not None

    @property
    def solution(self):
        return self._solution

    def invalidate(self, reason=""):
        """作废当前解（急停/安全事件；契约 §4）。"""
        self._solution = None
        self._last_ok_ms = None
        return {"invalidated": True, "reason": reason}

    # ---- 主入口 ----
    def step(self, request, emergency=False, timeout_ms=None):
        """推进一拍。返回 dict(decision, reason, flags, z, diagnostics)。

        `request` 为**原问题** QP 数据（缺字段时由协议校验拒绝）。
        """
        self.stats["steps"] += 1
        do_update = (self._tick % self._ticks_per_update) == 0
        self._tick += 1

        if do_update:
            self.stats["updates"] += 1
            resp, err = self._client.call(request, timeout_ms=timeout_ms)
            self._last_status = str(resp.get("status_class", "unknown"))
            if err is None and resp.get("decision") == DECISION_OK:
                self._solution = list(resp["z"])
                self._last_ok_ms = self._now()
            else:
                # 显式失败：**不保留**旧解（契约 §2 禁止"继续跑旧解"）
                self._solution = None
                self._last_ok_ms = None
            solve_ms = resp.get("solve_ms")
            source = "subprocess"
        else:
            self.stats["skips"] += 1
            solve_ms = None
            source = "held"

        age_ms = None if self._last_ok_ms is None else (self._now() - self._last_ok_ms)
        verdict = decide(age_ms=age_ms, qp_status_class=self._last_status,
                         solve_ms=solve_ms, emergency=emergency)
        if verdict["decision"] == DECISION_OK:
            z = self._solution
        else:
            z = None
            if verdict["decision"] == "damped_hold":
                self.stats["holds"] += 1
            else:
                self.stats["releases"] += 1
                self._solution = None
                self._last_ok_ms = None
        diagnostics = {"age_ms": age_ms, "status_class": self._last_status,
                       "solve_ms": solve_ms, "source": source, "tick": self._tick - 1,
                       "ticks_per_update": self._ticks_per_update}
        return {"decision": verdict["decision"], "reason": verdict["reason"],
                "flags": verdict["flags"], "z": z, "diagnostics": diagnostics}
