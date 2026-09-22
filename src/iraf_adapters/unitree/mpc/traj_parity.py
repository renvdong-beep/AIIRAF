"""保值摊销的**判据工具**：把"逐位一致"做成可断言的比较（第 3 块 `traj_amortize` 的前置）。

背景（`docs/debug/2026-09-22-mpc-qp-latency.md` §15）：外部 MPC 的 `_continuousDynamics` 每次调用都在
16 次循环里计算 `Ac` / `Bc` / `gc`，而这三个数组**全工程没有任何读取方**（只填不算用）。
去掉它们可把参考轨迹生成 P50 从 4.7170 ms 降到 2.9958 ms，且**被消费的 14 个数组逐位一致**。

为什么先做工具：摊销属于"重构实时路径"，其验收判据是**逐位一致**（不是"看起来一样"）。
本模块提供该判据的可复跑实现，供移植/改造后一键断言：

  · `compare_array`：单数组判定（`tobytes()` 严格比较 + 形状/NaN 处理 + 首个不一致索引）
  · `assert_bit_identical`：整组数组的**门禁**（不通过即抛错，附中文差异清单）
  · `summarize`：一行摘要，便于写进验收报告

注意语义：`tobytes()` 比较对 NaN 也成立（同一位模式即相等），这正符合"重构不得改变数值输出"的要求；
而 `np.array_equal` 对 NaN 会判不等 ⇒ 本模块**不用**它做逐位判据。
"""

from __future__ import annotations

import numpy as np

__all__ = ["compare_array", "assert_bit_identical", "summarize"]


def compare_array(name, a, b):
    """比较同名数组：返回 `dict(name, shape_ok, dtype_ok, bit_equal, max_abs_diff, first_diff)`。

    - `bit_equal`：`a.tobytes() == b.tobytes()`（严格逐位；NaN 位模式相同也算相等）
    - `max_abs_diff`：形状/类型不同或含 NaN 时为 `nan`
    - `first_diff`：第一个不一致的扁平索引（无差异为 `None`）
    """
    xa = np.asarray(a)
    xb = np.asarray(b)
    out = {"name": name, "shape_a": tuple(xa.shape), "shape_b": tuple(xb.shape),
           "dtype_a": str(xa.dtype), "dtype_b": str(xb.dtype),
           "shape_ok": xa.shape == xb.shape, "dtype_ok": xa.dtype == xb.dtype,
           "bit_equal": False, "max_abs_diff": float("nan"), "first_diff": None}
    if not out["shape_ok"]:
        return out
    out["bit_equal"] = xa.tobytes() == xb.tobytes()
    try:
        fa = xa.astype(np.float64, copy=False).reshape(-1)
        fb = xb.astype(np.float64, copy=False).reshape(-1)
        diff = np.abs(fa - fb)
        out["max_abs_diff"] = float(np.nanmax(diff)) if diff.size else 0.0
        nz = np.flatnonzero(diff > 0)
        out["first_diff"] = int(nz[0]) if nz.size else None
    except (TypeError, ValueError):
        out["max_abs_diff"] = float("nan")
    return out


def assert_bit_identical(pairs, context=""):
    """门禁：整组 `(name, a, b)` 必须逐位一致；否则抛 `AssertionError`（含中文差异清单）。

    返回逐项结果列表（全部通过时用于写报告）。
    """
    results = [compare_array(n, a, b) for n, a, b in pairs]
    bad = [r for r in results if not r["bit_equal"]]
    if bad:
        lines = []
        for r in bad[:10]:
            if not r["shape_ok"]:
                lines.append("  · %s：形状不一致 %s vs %s" % (r["name"], r["shape_a"], r["shape_b"]))
            else:
                lines.append("  · %s：非逐位一致（最大绝对差 %.3e，首个不一致索引 %s）"
                             % (r["name"], r["max_abs_diff"], r["first_diff"]))
        raise AssertionError("保值性门禁未通过%s：%d/%d 个数组不一致\n%s"
                             % (("（%s）" % context) if context else "", len(bad), len(results),
                                "\n".join(lines)))
    return results


def summarize(results, context=""):
    """一行摘要：`上下文: N 个数组逐位一致（形状/类型均相同）`。"""
    n = len(results)
    shapes_ok = all(r["shape_ok"] for r in results)
    dtypes_ok = all(r["dtype_ok"] for r in results)
    return "%s%s：%d 个数组逐位一致（形状%s、类型%s）" % (
        ("%s " % context) if context else "", "", n,
        "一致" if shapes_ok else "不一致", "一致" if dtypes_ok else "不一致")
