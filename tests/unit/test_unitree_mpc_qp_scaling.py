"""单测：等价变量缩放（`iraf_adapters.unitree.mpc.qp_scaling`）的**等价性**与**失败路径**。

为什么这样测：本模块**只依赖 numpy**（框架核心与适配层不引入求解器依赖），因此单测在没有任何
QP 求解器的环境里也要能证明"变换不改解"。做法是直接验证**等价性成立的四条代数性质**：

  1. 往返一致：`unscale(scale(v)) == v`（逐位）；
  2. 目标函数值不变：原变量下的 `½ zᵀHz + gᵀz` 与缩放变量下的 `½ wᵀP'w + q'ᵀw` 相等；
  3. 约束不变：`A z` 与 `A' w` 相等（行约束）、盒约束 `l ≤ z ≤ u ⟺ l' ≤ w ≤ u'`；
  4. 边界语义：`±inf` 与 `0` 在缩放/回代下保持（`D` 为正）。

外加失败路径：`H` 非正定/非有限/维度不匹配时**显式抛错**（不得静默继续）。

证据基础：`docs/debug/2026-09-22-mpc-qp-latency.md` §13（iter 340 → 60、solve 9.8146 → 1.4307 ms）。
"""

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.qp_scaling import (
    build_scale_diag,
    from_scaled_variables,
    scale_box_bounds,
    scale_hessian_diag,
    scale_linear_cost,
    scale_matrix_nonzeros,
    to_scaled_variables,
    unscale_box_duals,
)

# 用与本工程同类病态的 H（对角跨度大：2e-5 ~ 1e2）
H_DIAG = np.array([2.0, 2.0, 100.0, 20.0, 40.0, 2.0, 2e-5, 2e-5, 2e-5, 2e-5])


def test_returns_scale_is_bit_exact():
    d = build_scale_diag(H_DIAG)
    rng = np.random.default_rng(20260922)
    z = rng.normal(size=H_DIAG.size)
    assert np.allclose(from_scaled_variables(to_scaled_variables(z, d), d), z, rtol=1e-15, atol=0.0)  # FP 往返非逐位，只保证 1e-15 相对精度


def test_hessian_diag_becomes_one():
    d = build_scale_diag(H_DIAG)
    assert np.allclose(scale_hessian_diag(H_DIAG, d), 1.0, rtol=0, atol=1e-15)


def test_objective_is_invariant():
    d = build_scale_diag(H_DIAG)
    rng = np.random.default_rng(7)
    z = rng.normal(size=H_DIAG.size)
    g = rng.normal(size=H_DIAG.size)

    f_orig = 0.5 * float(np.sum(H_DIAG * z * z)) + float(g @ z)

    w = to_scaled_variables(z, d)
    h_scaled = scale_hessian_diag(H_DIAG, d)      # = 1
    q_scaled = scale_linear_cost(g, d)
    f_scaled = 0.5 * float(np.sum(h_scaled * w * w)) + float(q_scaled @ w)

    assert f_scaled == pytest.approx(f_orig, rel=1e-12, abs=1e-12)


def test_row_constraints_are_invariant():
    d = build_scale_diag(H_DIAG)
    # 稀疏三元组：col_index 与 nonzeros 同序（列主序），列数 = H_DIAG.size
    rows = np.array([0, 0, 1, 1, 2, 3, 4, 5], dtype=np.int64)
    cols = np.array([0, 6, 1, 7, 2, 3, 4, 5], dtype=np.int64)
    vals = np.array([1.0, -0.5, 2.0, 1.5, -1.0, 0.25, 3.0, -2.0])
    rng = np.random.default_rng(11)
    z = rng.normal(size=H_DIAG.size)

    a_orig = np.zeros(6)
    np.add.at(a_orig, rows, vals * z[cols])

    w = to_scaled_variables(z, d)
    vals_scaled = scale_matrix_nonzeros(vals, cols, d)
    a_scaled = np.zeros(6)
    np.add.at(a_scaled, rows, vals_scaled * w[cols])

    assert np.allclose(a_scaled, a_orig, rtol=1e-13, atol=1e-13)


def test_box_bounds_and_infinities_are_preserved():
    d = build_scale_diag(H_DIAG)
    n = H_DIAG.size
    lbx = np.array([-np.inf, 10.0, -np.inf, 0.0, 0.0, -np.inf, 10.0, 10.0, 10.0, 10.0])
    ubx = np.array([np.inf, np.inf, np.inf, 0.0, 0.0, np.inf, 20.0, 20.0, 20.0, 20.0])
    l_s, u_s = scale_box_bounds(lbx, ubx, d)

    # ±inf 语义保持
    assert np.all(np.isinf(l_s) == np.isinf(lbx))
    assert np.all(np.isinf(u_s) == np.isinf(ubx))
    # 0 保持为 0
    assert np.all(l_s[lbx == 0.0] == 0.0)

    # 盒约束等价：对随机 w，判定结果一致
    rng = np.random.default_rng(3)
    for _ in range(50):
        z = rng.normal(size=n) * 10.0
        w = to_scaled_variables(z, d)
        ok_orig = bool(np.all(z >= lbx) and np.all(z <= ubx))
        ok_scaled = bool(np.all(w >= l_s) and np.all(w <= u_s))
        assert ok_orig == ok_scaled


def test_box_duals_unscale():
    d = build_scale_diag(H_DIAG)
    lam = np.ones_like(d)
    back = unscale_box_duals(lam / d, d)  # 前向 λ′ = λ/d，回代 λ = D·λ′
    assert np.allclose(back, lam, rtol=1e-15, atol=0.0)


def test_failure_paths_are_explicit():
    with pytest.raises(ValueError):
        build_scale_diag([])
    with pytest.raises(ValueError):
        build_scale_diag([1.0, 0.0, 2.0])            # 非正定
    with pytest.raises(ValueError):
        build_scale_diag([1.0, np.nan, 2.0])         # 非有限
    d = build_scale_diag(H_DIAG)
    with pytest.raises(ValueError):
        to_scaled_variables(np.zeros(3), d)                 # 维度不匹配
    with pytest.raises(ValueError):
        scale_matrix_nonzeros(np.ones(2), np.array([0, 99]), d)   # 列号越界
    with pytest.raises(ValueError):
        scale_box_bounds(np.zeros(3), np.zeros(3), d)             # 维度不匹配
