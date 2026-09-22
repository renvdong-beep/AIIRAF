"""测试用假求解器（仅用于注入 `IRAF_MPC_SOLVER`，不参与生产路径）。

行为由环境变量 `IRAF_FAKE_STATUS` 控制：
  · 未设置 / "ok" ⇒ 返回 ok，并在**缩放空间**给出 w = [0.5, 0.5]；
  · 其它字符串   ⇒ 原样作为 status_class 返回（用于验证坏状态路径）；
  · "raise"      ⇒ 抛异常（验证异常路径）。
"""

import os

import numpy as np


def solve_native(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba, prev=None, settings=None):
    status = os.environ.get("IRAF_FAKE_STATUS", "ok")
    if status == "raise":
        raise RuntimeError("假求解器按 IFAF_FAKE_STATUS=raise 抛错")
    n = len(h_diag)
    return {
        "x": np.full(n, 0.5, dtype=float),
        "y": np.zeros(n + len(lba), dtype=float),
        "status": status,
        "status_class": "ok" if status == "ok" else status,
        "iter": 60.0,
        "setup_ms": 0.5,
        "solve_ms": 1.9972,
    }
