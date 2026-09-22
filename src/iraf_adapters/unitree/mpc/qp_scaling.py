"""等价变量缩放：把 QP 的数值形态改好，**不改解**。

背景（实测依据）：本工程外部 MPC 的代价矩阵 `R = 1e-5·I`、状态代价 `Q = diag([1,1,50,10,20,1,2,2,1,1,1,1])`
⇒ `H = 2·diag(Q, R)` 的对角元素跨度 `2e-5 ~ 1e2`（比例 5×10⁶）；OSQP 是一阶（ADMM）法，
条件数直接决定迭代数。实测（真实连续 QP 序列、原生 OSQP）：

    变体        iter P50   solve P50
    as-is          340      9.8146 ms
    unit-H          60      1.4307 ms      ← 本模块实现的变换
    （证据：docs/debug/2026-09-22-mpc-qp-latency.md §13；全过程 §11~§17）

变换（严格等价，解不变）：令 `z = D⁻¹ w`，`D = diag(√H_ii)`，则

    P' = D⁻¹ H D⁻¹   （本工程 H 为对角 ⇒ P' = I）
    q' = D⁻¹ q
    A' = A D⁻¹       （等价于"每个非零元按其列号除以 d_j"）
    l' = [D·lbx ; lba]、u' = [D·ubx ; uba]      （行约束不变；盒约束随变量一起缩放）
    回代 z = D⁻¹ w  （对偶：盒约束的对偶量 / d，行约束的对偶量不变）

`D` 为正对角（`H` 正定 ⇒ `H_ii > 0`），故 ±inf 与 0 在乘法下语义保持。

本模块刻意**只依赖 numpy**（不引入 scipy/casadi/osqp）：框架核心与适配层的其它部分不需要求解器依赖，
单测可以在没有任何求解器的环境里验证等价性（见 `tests/unit/test_unitree_mpc_qp_scaling.py`）。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "build_scale_diag",
    "to_scaled_variables",
    "from_scaled_variables",
    "scale_linear_cost",
    "scale_hessian_diag",
    "scale_matrix_nonzeros",
    "scale_box_bounds",
    "unscale_box_duals",
]


def build_scale_diag(h_diag) -> np.ndarray:
    """由 Hessian 对角构造缩放因子 `d = √H_ii`（逐元素）。

    参数
    ----
    h_diag : 形状 (n,) 的 Hessian 对角元素

    返回
    ----
    形状 (n,) 的 `d`，全部为正。

    失败路径（显式失败，不静默）：
    - 出现非有限值或非正值 ⇒ `ValueError`（`H` 非正定或数据损坏时不得继续，避免"看似跑通"）
    """
    h = np.asarray(h_diag, dtype=float).reshape(-1)
    if h.size == 0:
        raise ValueError("h_diag 为空：无可缩放变量")
    if not np.all(np.isfinite(h)):
        raise ValueError("h_diag 含非有限值（NaN/Inf）：H 必须有限")
    if np.any(h <= 0.0):
        bad = np.flatnonzero(h <= 0.0)[:5].tolist()
        raise ValueError("h_diag 含非正值（H 非正定），索引示例 %s" % bad)
    return np.sqrt(h)


def to_scaled_variables(z, d) -> np.ndarray:
    """变量前向映射：`w = D·z`（**乘**）。

    注意方向：变量是 `z = D⁻¹ w`（原变量 → 缩放变量是乘 `D`），
    而**代价向量**相反（`q' = D⁻¹ q`，见 `scale_linear_cost`）。
    首版把两者写成同一个"除"，被单测
    `test_returns_scale_is_bit_exact` 抓出 ⇒ 已分离为两个函数。
    """
    z = np.asarray(z, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if z.shape != d.shape:
        raise ValueError("维度不匹配：z %s vs d %s" % (z.shape, d.shape))
    return z * d


def from_scaled_variables(w, d) -> np.ndarray:
    """变量回代：`z = D⁻¹ w`（**除**）。"""
    w = np.asarray(w, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if w.shape != d.shape:
        raise ValueError("维度不匹配：w %s vs d %s" % (w.shape, d.shape))
    return w / d


def scale_linear_cost(g, d) -> np.ndarray:
    """线性代价：`q' = D⁻¹ q`（**除**）。"""
    g = np.asarray(g, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if g.shape != d.shape:
        raise ValueError("维度不匹配：g %s vs d %s" % (g.shape, d.shape))
    return g / d


def scale_hessian_diag(h_diag, d) -> np.ndarray:
    """`H'_ii = H_ii / d_i²`。取 `d = √H_ii` 时结果恒为 1（对角归一）。"""
    h = np.asarray(h_diag, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if h.shape != d.shape:
        raise ValueError("维度不匹配：h %s vs d %s" % (h.shape, d.shape))
    return h / (d * d)


def scale_matrix_nonzeros(nonzeros, col_index, d) -> np.ndarray:
    """`A' = A D⁻¹`：每个非零元按其**列号**除以 `d_j`。

    参数
    ----
    nonzeros  : 形状 (nnz,) 的稀疏矩阵非零元（列主序，与 `col_index` 同序）
    col_index : 形状 (nnz,) 的列号数组（CasADi `Sparsity.get_triplet()` 的第二项即此义）
    d         : 形状 (n,) 的缩放因子
    """
    nz = np.asarray(nonzeros, dtype=float).reshape(-1)
    cols = np.asarray(col_index, dtype=np.int64).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if nz.shape != cols.shape:
        raise ValueError("非零元与列号长度不一致：%s vs %s" % (nz.shape, cols.shape))
    if cols.size and (cols.min() < 0 or cols.max() >= d.size):
        raise ValueError("列号越界：cols ∈ [%d, %d]，d 长度 %d"
                         % (cols.min(), cols.max(), d.size))
    return nz / d[cols]


def scale_box_bounds(lbx, ubx, d):
    """盒约束随变量一起缩放：`l' = D·l`、`u' = D·u`（`D` 为正 ⇒ ±inf 语义保持）。"""
    lbx = np.asarray(lbx, dtype=float).reshape(-1)
    ubx = np.asarray(ubx, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if lbx.shape != d.shape or ubx.shape != d.shape:
        raise ValueError("维度不匹配：lbx %s / ubx %s / d %s" % (lbx.shape, ubx.shape, d.shape))
    return lbx * d, ubx * d


def unscale_box_duals(lam_scaled, d) -> np.ndarray:
    """盒约束对偶量回代：`λ = D·λ′`（**乘**）。

    推导：原问题 KKT 的平稳性 `H z + g + λ_up − λ_lo = 0`；缩放后
    `P′ w + q′ + λ′_up − λ′_lo = 0`，代入 `w = D z`、`P′ = D⁻¹HD⁻¹`、`q′ = D⁻¹g` 得
    `D⁻¹(H z + g) + λ′_up − λ′_lo = 0` ⇒ 左乘 `D` ⇒ `λ = D·λ′`（行约束对偶量不变，不在本函数内）。
    前向映射为 `λ′ = λ / d`（由调用方按此式给出）。
    """
    lam = np.asarray(lam_scaled, dtype=float).reshape(-1)
    d = np.asarray(d, dtype=float).reshape(-1)
    if lam.shape != d.shape:
        raise ValueError("维度不匹配：lam_x %s vs d %s" % (lam.shape, d.shape))
    return lam * d
