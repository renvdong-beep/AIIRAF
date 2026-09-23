"""验收脚本用的**可注入求解器**（只在脚本/子进程里使用；生产求解器是 `osqp_native.solve_native`）。

用法（由 `scripts/verify_mpc_torque_hook.py` 通过环境变量注入到 worker 子进程）：
  · `IRAF_MPC_SOLVER=_mpc_probe_solver:solve_native`
  · `IRAF_FAKE_STATUS`：`ok`（默认）/ `ok_inaccurate` / `max_iter` / `infeasible` / `raise` / `hang`
  · `IRAF_PROBE_FAIL_AFTER`：**子进程内**前 N 次调用返回 ok，第 N+1 次起按 `IRAF_FAKE_STATUS` 行为
    （默认 0 = 永不失败）。为什么用"第几次调用"而不是改父进程的环境变量：子进程的环境在
    `Popen` 时就固定了 —— 改父进程的 `os.environ` **影响不到**已在跑的子进程（这点实测过，
    首版脚本就是因此把"第一次 ok、第二次失败"的用例写成了永远不会失败）。
  · `IRAF_PROBE_X`：JSON 数字列表 —— 直接作为**缩放空间**的解向量返回（不设即显式失败，不猜）
  · `IRAF_PROBE_HANG_S`：`hang` 时睡眠秒数（默认 3.0）

为什么把 x 做成注入值：验收要断言"设计好的足端力 → 实际载荷"，而中间隔着
「缩放 → 子进程 → JSON → 回代」四道；把 x 写死在本文件里就等于把断言对象也写死了。
"""

from __future__ import annotations

import json
import os
import time

import numpy as np

#: 本子进程已服务的调用次数（每个子进程一份；重启后归零，这是**刻意**的：重启即新进程）。
_CALLS = {"n": 0}


def solve_native(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba, prev=None, settings=None):
    _CALLS["n"] += 1
    fail_after = int(os.environ.get("IRAF_PROBE_FAIL_AFTER", "0") or 0)
    failing = bool(fail_after) and _CALLS["n"] > fail_after
    status = os.environ.get("IRAF_FAKE_STATUS", "ok") if failing else "ok"
    if status == "raise":
        raise RuntimeError("假求解器按 IRAF_FAKE_STATUS=raise 抛错（验证异常路径）")
    if status == "hang":
        time.sleep(float(os.environ.get("IRAF_PROBE_HANG_S", "3.0")))
    raw = os.environ.get("IRAF_PROBE_X")
    if raw is None:
        raise RuntimeError("未提供 IRAF_PROBE_X：本假求解器不猜解向量（显式失败）")
    x = np.asarray(json.loads(raw), dtype=float).reshape(-1)
    n = len(h_diag)
    if x.size != n:
        raise RuntimeError("IRAF_PROBE_X 长度 %d ≠ 变量数 %d" % (x.size, n))
    return {
        "x": x,
        "y": np.zeros(n + len(lba), dtype=float),
        "status": status,
        "status_class": "ok" if status == "ok" else status,
        "iter": 60.0,
        "setup_ms": 0.5,
        "solve_ms": 1.9972,
    }
