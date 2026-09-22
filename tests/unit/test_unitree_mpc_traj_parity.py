"""单测：`traj_parity`（保值摊销的逐位一致门禁）—— 含 NaN、-0.0、形状/类型不一致等边界。"""

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.traj_parity import assert_bit_identical, compare_array, summarize


def test_identical_arrays_pass():
    a = np.linspace(-1.0, 1.0, 17).reshape(17, 1)
    r = compare_array("x", a, a.copy())
    assert r["bit_equal"] and r["shape_ok"] and r["dtype_ok"]
    assert r["max_abs_diff"] == 0.0 and r["first_diff"] is None


def test_one_ulp_difference_is_not_bit_equal():
    a = np.array([1.0, 2.0, 3.0])
    b = a.copy()
    b[1] = np.nextafter(b[1], np.inf)          # 只差 1 ulp
    r = compare_array("x", a, b)
    assert r["bit_equal"] is False
    assert r["first_diff"] == 1
    assert r["max_abs_diff"] > 0.0


def test_nan_same_bit_pattern_counts_as_equal():
    a = np.array([1.0, np.nan, 3.0])
    b = np.array([1.0, np.nan, 3.0])
    assert compare_array("x", a, b)["bit_equal"] is True      # 位模式相同
    c = np.array([1.0, np.nan, 3.0 + 1e-12])
    assert compare_array("x", a, c)["bit_equal"] is False


def test_shape_and_dtype_mismatch_are_reported():
    r = compare_array("x", np.zeros((3, 1)), np.zeros((3,)))
    assert r["shape_ok"] is False and r["bit_equal"] is False
    r2 = compare_array("x", np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float64))
    assert r2["shape_ok"] and not r2["dtype_ok"] and not r2["bit_equal"]


def test_gate_passes_and_reports_summary():
    pairs = [("Ad", np.eye(3), np.eye(3)), ("Bd", np.zeros((2, 3)), np.zeros((2, 3)))]
    res = assert_bit_identical(pairs, context="参考轨迹")
    assert len(res) == 2
    assert "2 个数组逐位一致" in summarize(res, "参考轨迹")


def test_gate_raises_with_chinese_diff_list():
    a = np.array([1.0, 2.0])
    b = np.array([1.0, 2.0 + 1e-15])
    with pytest.raises(AssertionError) as ei:
        assert_bit_identical([("r_fl_foot_world", a, b)], context="参考轨迹")
    msg = str(ei.value)
    assert "保值性门禁未通过" in msg and "r_fl_foot_world" in msg and "最大绝对差" in msg


def test_gate_raises_on_shape_mismatch_with_clear_reason():
    with pytest.raises(AssertionError) as ei:
        assert_bit_identical([("contact_table", np.zeros((4, 16)), np.zeros((4, 15)))])
    assert "形状不一致" in str(ei.value)
