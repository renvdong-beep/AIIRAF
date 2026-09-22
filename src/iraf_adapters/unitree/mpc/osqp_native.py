"""原生 OSQP 调用层（避开 CasADi 包装）—— 第 2 块落地。

为什么：R2 实测（`docs/debug/2026-09-22-mpc-qp-latency.md` §11.2/§16）同一批真实 QP 下
CasADi 的 OsqpInterface 与原生 OSQP 纯求解耗时之比为 **1.04**，但**缩放后**走 CasADi 仍
4.1580 ms/拍、原生只要 1.4307 ms ⇒ **包装费 ≈2.7 ms/拍**，成为缩放后的主项。
本模块只负责"把 QP 交给原生 OSQP 并取回解"，不含任何 MPC 语义（语义在上层 `provider`）。

**依赖策略（硬性，铁律 3/4）**：`osqp` / `scipy` 只在函数体内**延迟导入**；缺失时抛
`RuntimeError` 并给出中文原因 ⇒ 仓库主环境（无求解器）跑 pytest 不会因缺依赖而失败，
而真正需要求解时**显式失败、不静默降级**。

本文件里的**纯 numpy** 部分（三元组→CSC、约束行拼接、状态分类、热身向量校验）可在
无求解器环境完整单测（见 `tests/unit/test_unitree_mpc_osqp_native.py`）。
"""

from __future__ import annotations

import importlib
import numpy as np

__all__ = [
    "SETTINGS_BASELINE",
    "resolved_settings",
    "csc_from_triplets",
    "stack_identity_rows",
    "concat_bounds",
    "classify_status",
    "check_warm_start",
    "solve_native",
]

# 与 R2 实测所用配置逐项一致（`centroidal_mpc.OPTS['osqp']` + 顶层热身开关）
SETTINGS_BASELINE = {
    "eps_abs": 1e-4,
    "eps_rel": 1e-4,
    "max_iter": 1000,
    "polish": False,
    "adaptive_rho": True,
    "check_termination": 10,
    "adaptive_rho_interval": 25,
    "scaled_termination": True,
    "warm_starting": True,
}

# OSQP 返回状态 → 我们的显式失败分类（缺省落到 unknown，绝不当作成功）
_STATUS_MAP = {
    "solved": "ok",
    "solved inaccurate": "ok_inaccurate",
    "primal infeasible": "infeasible",
    "dual infeasible": "unbounded",
    "maximum iterations reached": "max_iter",
    "run time limit reached": "time_limit",
    "non-convex": "nonconvex",
    "unsolved": "unknown",
}


def _require(modname: str):
    """延迟导入依赖；缺失时**显式失败**并给出中文可操作原因。"""
    try:
        return importlib.import_module(modname)
    except ImportError as exc:  # pragma: no cover - 由单测用假模块名覆盖
        raise RuntimeError(
            "本模块需要 `%s`（MPC Provider 的独立进程环境内）；当前环境缺失 ⇒ 显式失败，不降级。"
            "安装示例：pip install osqp scipy（或按 BoardProfile 的 wheelhouse 安装）" % modname
        ) from exc


def resolved_settings(overrides=None) -> dict:
    """在 `SETTINGS_BASELINE` 上叠加覆盖项；未知键**显式失败**（避免拼错后静默无效）。"""
    out = dict(SETTINGS_BASELINE)
    for k, v in (overrides or {}).items():
        if k not in out:
            raise ValueError("未知的 OSQP 设置项 %r（允许项：%s）" % (k, sorted(out)))
        out[k] = v
    return out


def csc_from_triplets(rows, cols, vals, shape):
    """三元组 → CSC（**保留显式零**，以维持固定稀疏模式，便于后续 `update()`）。

    为什么自己算而不依赖 scipy：组装是纯索引运算，放在这里可被无求解器环境单测；
    真正的求解入口再用 scipy/csc 对象（延迟导入）。
    """
    rows = np.asarray(rows, dtype=np.int64).reshape(-1)
    cols = np.asarray(cols, dtype=np.int64).reshape(-1)
    vals = np.asarray(vals, dtype=float).reshape(-1)
    nrow, ncol = int(shape[0]), int(shape[1])
    if not (rows.shape == cols.shape == vals.shape):
        raise ValueError("rows/cols/vals 长度不一致：%s/%s/%s" % (rows.shape, cols.shape, vals.shape))
    if rows.size and (rows.min() < 0 or rows.max() >= nrow):
        raise ValueError("行号越界：rows ∈ [%d, %d]，形状 %s" % (rows.min(), rows.max(), shape))
    if cols.size and (cols.min() < 0 or cols.max() >= ncol):
        raise ValueError("列号越界：cols ∈ [%d, %d]，形状 %s" % (cols.min(), cols.max(), shape))
    order = np.lexsort((rows, cols))          # 先列后行 ⇒ CSC 顺序
    rows, cols, vals = rows[order], cols[order], vals[order]
    indptr = np.zeros(ncol + 1, dtype=np.int64)
    np.add.at(indptr, cols + 1, 1)
    indptr = np.cumsum(indptr)
    return indptr, rows.astype(np.int64), vals.astype(float)


def stack_identity_rows(n, a_rows, a_cols, a_vals):
    """构造 OSQP 需要的 `[I; A]` 约束（盒约束必须显式作为行给出）。

    `A` 的稀疏模式**逐项保留**（含显式零），使 `rows/cols` 在时间推进中保持不变 ⇒ 可用 `update()`。
    返回 `(rows, cols, vals, n_total_rows)`。
    """
    a_rows = np.asarray(a_rows, dtype=np.int64).reshape(-1)
    a_cols = np.asarray(a_cols, dtype=np.int64).reshape(-1)
    a_vals = np.asarray(a_vals, dtype=float).reshape(-1)
    if not (a_rows.shape == a_cols.shape == a_vals.shape):
        raise ValueError("A 三元组长度不一致")
    i_rows = np.arange(n, dtype=np.int64)
    i_cols = np.arange(n, dtype=np.int64)
    i_vals = np.ones(n, dtype=float)
    rows = np.concatenate([i_rows, a_rows + n])
    cols = np.concatenate([i_cols, a_cols])
    vals = np.concatenate([i_vals, a_vals])
    return rows, cols, vals, n + int(a_rows.max() + 1 if a_rows.size else 0)


def concat_bounds(lbx, ubx, lba, uba):
    """盒约束与行约束的 `l/u` 拼接；`±inf` 原样传递，并校验 `l ≤ u`（含 NaN 检查）。"""
    lbx = np.asarray(lbx, dtype=float).reshape(-1)
    ubx = np.asarray(ubx, dtype=float).reshape(-1)
    lba = np.asarray(lba, dtype=float).reshape(-1)
    uba = np.asarray(uba, dtype=float).reshape(-1)
    if lbx.shape != ubx.shape:
        raise ValueError("lbx/ubx 维度不匹配：%s vs %s" % (lbx.shape, ubx.shape))
    if lba.shape != uba.shape:
        raise ValueError("lba/uba 维度不匹配：%s vs %s" % (lba.shape, uba.shape))
    l = np.concatenate([lbx, lba])
    u = np.concatenate([ubx, uba])
    if np.isnan(l).any() or np.isnan(u).any():
        raise ValueError("边界含 NaN：不允许（NaN 会让求解器行为未定义）")
    bad = np.flatnonzero(l > u)
    if bad.size:
        raise ValueError("存在 l > u 的行（前 5 个索引 %s）⇒ 问题自相矛盾" % bad[:5].tolist())
    return l, u


def classify_status(status) -> str:
    """OSQP 状态 → 显式分类；未知状态归 `unknown`（**绝不视为成功**）。"""
    s = str(status).strip().lower().replace("_", " ")
    if s in _STATUS_MAP:
        return _STATUS_MAP[s]
    for key, val in _STATUS_MAP.items():      # 容忍前后缀（如 "problem not solved"）
        if key in s:
            return val
    return "unknown"


def check_warm_start(prev, n_vars: int, n_cons: int):
    """热身向量校验：`x` 长度 = 变量数；`y` 长度 = 约束行数（盒行 + 行约束）。"""
    if prev is None:
        return None
    x, y = prev
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    if x.shape != (n_vars,):
        raise ValueError("热身 x 长度 %s ≠ 变量数 %d" % (x.shape, n_vars))
    if y.shape != (n_cons,):
        raise ValueError("热身 y 长度 %s ≠ 约束行数 %d" % (y.shape, n_cons))
    return x, y


def solve_native(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba, prev=None,
                 settings=None):
    """用**原生 OSQP** 求解一个已缩放好的 QP：`min ½ zᵀHz + gᵀz` s.t. `l ≤ [z; Az] ≤ u`。

    返回 `dict(x, y, status, status_class, iter, setup_ms, solve_ms)`；
    `status_class != "ok"` 时由上层走显式失败路径（本函数不抛成功假象）。

    依赖：`osqp`、`scipy`（延迟导入；缺失显式失败）。
    """
    import time as _time

    osqp = _require("osqp")
    sp = _require("scipy.sparse")

    h = np.asarray(h_diag, dtype=float).reshape(-1)
    g = np.asarray(g, dtype=float).reshape(-1)
    if h.shape != g.shape:
        raise ValueError("h_diag 与 g 维度不匹配：%s vs %s" % (h.shape, g.shape))
    n = h.size
    rows, cols, vals, m_total = stack_identity_rows(n, a_rows, a_cols, a_vals)
    l, u = concat_bounds(lbx, ubx, lba, uba)
    if l.size != m_total:
        raise ValueError("l/u 长度 %d ≠ 约束行数 %d（盒行 %d + 行约束 %d）"
                         % (l.size, m_total, n, l.size - n))
    indptr, indices, data = csc_from_triplets(rows, cols, vals, (m_total, n))
    A = sp.csc_matrix((data, indices, indptr), shape=(m_total, n))
    P = sp.csc_matrix(np.diag(h))
    st = resolved_settings(settings)
    warm = check_warm_start(prev, n, m_total)

    solver = osqp.OSQP()
    t0 = _time.perf_counter()
    solver.setup(P, g, A, l, u, **st)
    t_setup = (_time.perf_counter() - t0) * 1e3
    if warm is not None:
        solver.warm_start(warm[0], warm[1])
    t1 = _time.perf_counter()
    res = solver.solve()
    t_solve = (_time.perf_counter() - t1) * 1e3
    info = res.info
    return {
        "x": np.asarray(res.x, dtype=float).reshape(-1),
        "y": np.asarray(res.y, dtype=float).reshape(-1),
        "status": str(getattr(info, "status", "unknown")),
        "status_class": classify_status(getattr(info, "status", "unknown")),
        "iter": float(getattr(info, "iter", float("nan"))),
        "setup_ms": t_setup,
        "solve_ms": t_solve,
    }
