"""MPC Provider 内核（第 4 块）：把「等价缩放 + 原生求解 + 新鲜度门禁」串成一次可验收的求解步骤。

设计要点（都对着铁律）：
  · **求解器注入**：本内核不 import osqp/scipy，而是接受 `solver_fn`（真实实现是
    `osqp_native.solve_native`）⇒ 负向用例可在**无求解器**环境端到端跑；跨进程部署时该函数
    由独立进程侧提供，私有类型不穿透边界。
  · **失败即显式失败**：求解器抛异常或状态分类非可用 ⇒ 返回 `decision=damped_hold` 且
    **不输出任何解**（绝不复用上一次的解，也不把终态写成成功）。
  · **新鲜度门禁**：解龄由调用方给的 `now_ms` 与上一次成功求解时刻算出，判定交给 `freshness.decide`。
  · 时间与求解器都可注入 ⇒ 单测可复现、无 flakiness。
"""

from __future__ import annotations

import time

from .freshness import MPC_PERIOD_MS, decide
from . import qp_scaling as qs

__all__ = ["ProviderCore"]


class ProviderCore:
    """一次求解 = 缩放 → 求解 → 回代 → 新鲜度判定。

    参数
    ----
    solver_fn : callable(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba, prev, settings)
                → dict(x, y, status, status_class, iter, setup_ms, solve_ms)
                （真实实现：`osqp_native.solve_native`）
    now_fn    : 返回当前时刻（ms）的 callable，默认 `time.perf_counter()*1e3`
    """

    def __init__(self, solver_fn, now_fn=None):
        if not callable(solver_fn):
            raise TypeError("solver_fn 必须可调用（真实实现：osqp_native.solve_native）")
        self._solver = solver_fn
        self._now = now_fn or (lambda: time.perf_counter() * 1e3)
        self._prev = None            # 上一次成功求解的 (x, y)，仅用于**热身**，绝不用于"跑旧解"
        self._last_ok_ms = None      # 上一次成功求解的时刻（ms）

    # ---- 只读属性，便于验收与日志 ----
    @property
    def has_solution(self):
        return self._last_ok_ms is not None

    @property
    def last_ok_ms(self):
        return self._last_ok_ms

    def invalidate(self, reason=""):
        """作废当前解（如收到急停/安全事件）。之后必须重新求解才可能有可用解。"""
        self._prev = None
        self._last_ok_ms = None
        return {"invalidated": True, "reason": reason}

    def latest(self, now_ms=None, emergency=False):
        """**消费者侧**取解：返回最新解及其新鲜度判定（> 40 ms 或无解 ⇒ 不得使用）。

        为什么必须有这个接口：生产（`step`，50 Hz）与消费（控制环 100 Hz）速率不同 ⇒
        "本拍该不该用这份解"是**消费侧**的问题。`step()` 只负责解出来并记时间戳；
        真正决定"用不用/是否 `damped_hold`"的是本方法。
        """
        now = self._now() if now_ms is None else float(now_ms)
        age = None if self._last_ok_ms is None else (now - self._last_ok_ms)
        out = decide(age_ms=age, qp_status_class="ok" if self.has_solution else "unknown",
                     emergency=emergency)
        out["age_ms"] = age
        return out

    def step(self, qp, emergency=False, settings=None):
        """执行一步。

        `qp` 为**原问题**数据：`h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba`
        （键名即上面这些）；返回 dict：
          decision / reason / flags / z（原问题变量，仅 decision=ok 时存在）/
          status_class / iter / solve_ms / total_ms / age_ms
        """
        t_start = self._now()
        need = ("h_diag", "g", "a_rows", "a_cols", "a_vals", "lbx", "ubx", "lba", "uba")
        missing = [k for k in need if k not in qp]
        if missing:
            out = decide(age_ms=None, qp_status_class="unknown", emergency=emergency)
            out.update({"z": None, "status_class": "unknown", "iter": float("nan"),
                        "solve_ms": float("nan"), "total_ms": 0.0, "age_ms": None})
            out["reason"] = "请求缺少字段 %s ⇒ 显式失败" % missing
            return out

        d = qs.build_scale_diag(qp["h_diag"])
        h_s = qs.scale_hessian_diag(qp["h_diag"], d)
        g_s = qs.scale_linear_cost(qp["g"], d)
        a_s = qs.scale_matrix_nonzeros(qp["a_vals"], qp["a_cols"], d)
        lbx_s, ubx_s = qs.scale_box_bounds(qp["lbx"], qp["ubx"], d)

        status_class, solve_ms, it, z = "exception", float("nan"), float("nan"), None
        try:
            res = self._solver(h_s, g_s, qp["a_rows"], qp["a_cols"], a_s, lbx_s, ubx_s,
                               qp["lba"], qp["uba"], self._prev, settings)
            status_class = str(res.get("status_class", "unknown"))
            solve_ms = float(res.get("solve_ms", float("nan")))
            it = float(res.get("iter", float("nan")))
            if status_class in ("ok", "ok_inaccurate"):
                z = qs.from_scaled_variables(res["x"], d)
                self._prev = (res["x"], res["y"])          # 仅供下次热身
                self._last_ok_ms = self._now()
        except Exception as exc:  # noqa: BLE001  —— 任何求解器异常都走显式失败，不吞
            status_class = "exception"
            z = None
            exc_msg = str(exc).splitlines()[0][:120]
        else:
            exc_msg = ""

        age_ms = None if self._last_ok_ms is None else (self._now() - self._last_ok_ms)
        out = decide(age_ms=age_ms, qp_status_class=status_class, solve_ms=solve_ms,
                     emergency=emergency)
        if out["decision"] != "ok":
            z = None                                       # 显式失败：不输出解（禁止跑旧解）
        out.update({"z": z, "status_class": status_class, "iter": it, "solve_ms": solve_ms,
                    "total_ms": self._now() - t_start, "age_ms": age_ms})
        if exc_msg:
            out["reason"] = "%s（求解器异常：%s）" % (out["reason"], exc_msg)
        return out
