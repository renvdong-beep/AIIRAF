"""单测：`iraf_adapters.unitree.mpc.osqp_native` 的**纯 numpy 部分**与**依赖缺失的显式失败**。

设计意图：本层必须能在**没有任何 QP 求解器**的环境里被测试（铁律 3/4：求解器依赖只在
Provider 的独立进程环境里出现）。因此单测只覆盖组装/分类/校验这类纯逻辑，
并显式验证"缺依赖 ⇒ RuntimeError 且原因可读"，而不是让 pytest 静默跳过。
"""

import numpy as np
import pytest

from iraf_adapters.unitree.mpc import osqp_native as on


def test_csc_from_triplets_orders_by_column_and_keeps_explicit_zeros():
    rows = np.array([1, 0, 1, 0])
    cols = np.array([0, 1, 0, 0])
    vals = np.array([5.0, 2.0, 0.0, 1.0])          # 含显式 0（第 3 项）
    indptr, indices, data = on.csc_from_triplets(rows, cols, vals, (2, 2))
    assert indptr.tolist() == [0, 3, 4]
    assert indices.tolist() == [0, 1, 1, 0]         # 列 0 的三项按行号排序，再列 1
    assert data.tolist() == [1.0, 5.0, 0.0, 2.0]    # 显式 0 保留（模式固定）
    assert data.size == 4


def test_stack_identity_rows_builds_box_then_linear_rows():
    n = 3
    a_rows = np.array([0, 1])
    a_cols = np.array([1, 2])
    a_vals = np.array([-0.5, 1.25])
    rows, cols, vals, m_total = on.stack_identity_rows(n, a_rows, a_cols, a_vals)
    assert m_total == 5                              # 3 行盒 + 2 行线性
    assert rows[:3].tolist() == [0, 1, 2] and cols[:3].tolist() == [0, 1, 2]
    assert np.allclose(vals[:3], 1.0)
    assert rows[3:].tolist() == [3, 4]               # A 行整体下移 n
    assert cols[3:].tolist() == [1, 2]


def test_concat_bounds_preserves_infinity_and_zero():
    lbx = np.array([-np.inf, 0.0, 10.0])
    ubx = np.array([np.inf, 0.0, np.inf])
    lba = np.array([-np.inf, 3.0])
    uba = np.array([0.0, 3.0])
    l, u = on.concat_bounds(lbx, ubx, lba, uba)
    assert np.isneginf(l[0]) and np.isposinf(u[0])
    assert l[1] == 0.0 and u[1] == 0.0               # 摆动腿：0 = 0 语义保持
    assert l[3] == -np.inf and u[3] == 0.0           # 摩擦锥：≤ 0
    assert l[4] == u[4] == 3.0                       # 等式行


def test_concat_bounds_rejects_nan_and_inconsistent():
    with pytest.raises(ValueError):
        on.concat_bounds(np.array([np.nan]), np.array([1.0]), np.array([]), np.array([]))
    with pytest.raises(ValueError):
        on.concat_bounds(np.array([2.0]), np.array([1.0]), np.array([]), np.array([]))  # l > u
    with pytest.raises(ValueError):
        on.concat_bounds(np.array([0.0, 0.0]), np.array([1.0]), np.array([]), np.array([]))


def test_classify_status_maps_explicitly():
    assert on.classify_status("solved") == "ok"
    assert on.classify_status("solved inaccurate") == "ok_inaccurate"
    assert on.classify_status("primal infeasible") == "infeasible"
    assert on.classify_status("maximum iterations reached") == "max_iter"
    assert on.classify_status("maximum_iterations_reached") == "max_iter"
    assert on.classify_status("something new") == "unknown"      # 绝不视为成功


def test_resolved_settings_rejects_unknown_key():
    st = on.resolved_settings({"eps_abs": 1e-3})
    assert st["eps_abs"] == 1e-3 and st["max_iter"] == 1000
    with pytest.raises(ValueError):
        on.resolved_settings({"eps_ab": 1e-3})                   # 拼错必须显式失败


def test_check_warm_start_validates_lengths():
    assert on.check_warm_start(None, 3, 5) is None
    x, y = on.check_warm_start((np.zeros(3), np.zeros(5)), 3, 5)
    assert x.shape == (3,) and y.shape == (5,)
    with pytest.raises(ValueError):
        on.check_warm_start((np.zeros(4), np.zeros(5)), 3, 5)
    with pytest.raises(ValueError):
        on.check_warm_start((np.zeros(3), np.zeros(4)), 3, 5)


def test_missing_dependency_fails_explicitly_with_chinese_reason():
    with pytest.raises(RuntimeError) as ei:
        on._require("osqp_definitely_missing_module_xyz")
    msg = str(ei.value)
    assert "osqp_definitely_missing_module_xyz" in msg and "显式失败" in msg


def test_solve_native_raises_when_osqp_absent(monkeypatch):
    """在无求解器环境下调用求解入口 ⇒ RuntimeError（而不是 ImportError 泄漏或静默返回）。"""
    real_import = on.importlib.import_module

    def fake_import(name, *a, **k):
        if name == "osqp":
            raise ImportError("no osqp")
        return real_import(name, *a, **k)

    monkeypatch.setattr(on.importlib, "import_module", fake_import)
    with pytest.raises(RuntimeError) as ei:
        on.solve_native(h_diag=[1.0], g=[0.0], a_rows=[], a_cols=[], a_vals=[],
                        lbx=[-1.0], ubx=[1.0], lba=[], uba=[])
    assert "需要 `osqp`" in str(ei.value)
