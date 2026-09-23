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
        self._last_error = None
        self._last_response_reason = ""
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

    def will_update(self, emergency=False):
        """**只读**预测：下一次 `step(...)` 是否会真的调子进程（更新拍）。

        与 `step()` 内部的 `do_update` 是**同一份事实**（同一个式子），供调用方决定
        "本拍要不要构造请求"（384 变量的 QP 不该在非更新拍白造）。
        急停拍不更新（见 `step` 的注释）⇒ 这里也必须返回 False，否则预测与实际会错拍。
        """
        return (not emergency) and (self._tick % self._ticks_per_update) == 0

    # ---- 主入口 ----
    def step(self, request, emergency=False, timeout_ms=None):
        """推进一拍。返回 dict(decision, reason, flags, z, diagnostics)。

        `request` 为**原问题** QP 数据（缺字段时由协议校验拒绝）。
        """
        self.stats["steps"] += 1
        # 急停拍**不**调子进程（铁律 6.6：急停优先级最高 ⇒ 不再花一次求解预算/超时预算）；
        # 它直接落到 decide(emergency=True) ⇒ torque_zero_release 并作废当前解。
        do_update = ((self._tick % self._ticks_per_update) == 0) and not emergency
        self._tick += 1

        if do_update:
            self.stats["updates"] += 1
            resp, err = self._client.call(request, timeout_ms=timeout_ms)
            self._last_status = str(resp.get("status_class", "unknown"))
            # 失败**原因**必须可追溯（契约 §4 四类失败要能区分）：`err` 是客户端层的判定
            # （超时/子进程退出/响应非法/请求非法），`resp["reason"]` 是子进程层的中文原因。
            # 只留 freshness 的判词会把"为什么"丢掉（本层原先即如此）。
            self._last_error = err
            self._last_response_reason = str(resp.get("reason", ""))
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
                       "ticks_per_update": self._ticks_per_update,
                       # 最近一次**更新拍**的失败原因（held 拍沿用上一次的值，便于追因）
                       "client_error": self._last_error,
                       "response_reason": self._last_response_reason}
        return {"decision": verdict["decision"], "reason": verdict["reason"],
                "flags": verdict["flags"], "z": z, "diagnostics": diagnostics}
