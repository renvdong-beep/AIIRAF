"""`mpc.qp_builder.cost_diagonal`：H 对角、尺寸/nnz=384、与声明自洽及门禁。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from iraf_adapters.unitree.mpc.qp_builder import (STATE_DIM, INPUT_DIM, cost_diagonal,
                                                  state_cost_weights)

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config" / "go2_locomote.yaml"


def _model():
    if not CONFIG.is_file():
        pytest.skip("需要仓库内的真实声明文件")
    return (yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {})["mpc_model"]


def test_real_declaration_gives_384_diagonal_with_384_nonzeros():
    model = _model()
    out = cost_diagonal(model)
    assert out["horizon"] == 16
    assert out["size"] == 16 * (STATE_DIM + INPUT_DIM) == 384
    assert out["nnz"] == 384                      # 纯对角 ⇒ 非零元个数 = 尺寸
    assert out["h_diag"].shape == (384,)


def test_diagonal_layout_is_states_first_then_inputs():
    """布局 = 先 192 个状态项、后 192 个输入项（上游实测确认，**不是**每拍 [Q,R] 交替）。"""
    model = _model()
    out = cost_diagonal(model)
    d = out["h_diag"]
    n = out["horizon"]
    assert np.array_equal(d[:n * 12], np.tile(2.0 * out["q_diag"], n))
    assert np.array_equal(d[n * 12:], np.tile(2.0 * out["r_diag"], n))
    assert d[12] == pytest.approx(2.0)        # = 2·q₀（状态段仍在延续）
    assert d[192] == pytest.approx(2.0e-05)   # 输入段起点
    # 与"每拍交替"布局必须不同（防回归：首版就是错成那样）
    alt = np.tile(np.concatenate([2.0 * out["q_diag"], 2.0 * out["r_diag"]]), n)
    assert not np.array_equal(d, alt)
    # 声明值本身：q 最大项 50（状态 z）、r 全 1e-5 ⇒ 对角跨度 5e6（病态来源，如实登记）
    assert d.max() == pytest.approx(100.0)
    assert d.min() == pytest.approx(2.0e-5)
    assert d.max() / d.min() == pytest.approx(5.0e6)


def test_state_cost_weights_repeat_q_diag_per_step():
    model = _model()
    w = state_cost_weights(model)
    assert w.shape == (16 * 12,)
    assert np.array_equal(w[:12], np.array(model["q_diag"], dtype=float))
    assert np.array_equal(w[:12], w[12:24])


def test_matches_captured_upstream_qp_inputs_bitwise():
    """与截获的上游真实 QP 入参逐位比对（证据文件缺席则跳过，不伪造通过）。"""
    import json
    path = ROOT / "build" / "research" / "mpc-repo" / "qp_inputs.json"
    if not path.is_file():
        pytest.skip("缺少截获证据 %s" % path)
    captured = json.loads(path.read_text(encoding="utf-8"))
    ours = cost_diagonal(_model())["h_diag"]
    for qp in captured["qps"]:
        theirs = np.asarray(qp["h_diag"], dtype=float)
        assert theirs.shape == ours.shape
        assert theirs.tobytes() == ours.tobytes()


@pytest.mark.parametrize("bad", [
    None,
    {},
    {"q_diag": [1.0] * 12, "r_diag": [1e-5] * 12},
    {"q_diag": [1.0] * 11, "r_diag": [1e-5] * 12, "horizon": 16},
    {"q_diag": [1.0] * 12, "r_diag": [1e-5] * 11, "horizon": 16},
    {"q_diag": [-1.0] + [1.0] * 11, "r_diag": [1e-5] * 12, "horizon": 16},
    {"q_diag": [1.0] * 12, "r_diag": [1e-5] * 12, "horizon": 0},
    {"q_diag": [float("nan")] + [1.0] * 11, "r_diag": [1e-5] * 12, "horizon": 16},
])
def test_invalid_model_raises(bad):
    with pytest.raises((ValueError, TypeError)):
        cost_diagonal(bad)


# ---- 盒约束（_compute_bounds 语义移植）----

from iraf_adapters.unitree.mpc.qp_builder import box_bounds  # noqa: E402


def _mask(value=1, n=None):
    n = _model()["horizon"] if n is None else n
    return np.full((4, n), value, dtype=int)


def test_box_bounds_shapes_and_infinities():
    lb, ub = box_bounds(_mask(1), _model())
    assert lb.shape == ub.shape == (384,)
    assert np.all(np.isposinf(ub[192:]))          # 力段上界默认 +inf（支撑腿法向不设上界）
    assert np.all(np.isneginf(lb[:192]))          # 状态段无盒约束


def test_all_swing_forces_are_pinned_to_zero():
    lb, ub = box_bounds(_mask(0), _model())
    assert np.all(lb[192:] == 0.0) and np.all(ub[192:] == 0.0)


def test_stance_sets_only_normal_lower_bound_per_leg_and_step():
    m = _model()
    lb, ub = box_bounds(_mask(1), m)
    for k in range(int(m["horizon"])):
        for leg in range(4):
            base = 192 + 12 * k + 3 * leg
            assert lb[base] == -np.inf and lb[base + 1] == -np.inf   # 切向不设盒约束
            assert lb[base + 2] == pytest.approx(10.0)               # 法向下界（声明 min_normal_force_n）
            assert ub[base + 2] == np.inf
    # 腿序与 LEG_ORDER 同源：第 192+2 项 = FL 的法向
    from iraf_adapters.unitree.mpc.contact import LEG_ORDER
    assert list(LEG_ORDER) == ["FL", "FR", "RL", "RR"]
    assert lb[192 + 2] == pytest.approx(10.0)


def test_mixed_contact_per_leg():
    m = _model()
    mask = _mask(1, int(m["horizon"]))
    mask[2, :] = 0                                 # RL 全程摆动
    lb, ub = box_bounds(mask, m)
    for k in range(int(m["horizon"])):
        base_rl = 192 + 12 * k + 3 * 2
        assert lb[base_rl:base_rl + 3].tolist() == [0.0, 0.0, 0.0]
        assert ub[base_rl:base_rl + 3].tolist() == [0.0, 0.0, 0.0]
        assert lb[192 + 12 * k + 2] == pytest.approx(10.0)          # FL 仍支撑


@pytest.mark.parametrize("bad", [
    {"min_normal_force_n": 10.0},
    {"horizon": 16},
    {"horizon": 16, "min_normal_force_n": -1.0},
])
def test_box_bounds_missing_or_invalid_model_keys(bad):
    with pytest.raises(ValueError):
        box_bounds(_mask(1), bad)


def test_box_bounds_invalid_contact_table():
    m = _model()
    with pytest.raises(ValueError):
        box_bounds(np.zeros((3, 16), dtype=int), m)
    with pytest.raises(ValueError):
        box_bounds(np.zeros((4, 5), dtype=int), m)
    bad = _mask(1)
    bad[0, 0] = 2
    with pytest.raises(ValueError):
        box_bounds(bad, m)


def test_box_bounds_match_captured_upstream_bitwise():
    import json
    path = ROOT / "build" / "research" / "mpc-repo" / "qp_inputs.json"
    if not path.is_file():
        pytest.skip("缺少截获证据 %s" % path)
    captured = json.loads(path.read_text(encoding="utf-8"))
    m = _model()
    for qp in captured["qps"]:
        if "contact_table" not in qp:
            pytest.skip("证据文件未含 contact_table（需重跑探针）")
        lb, ub = box_bounds(np.asarray(qp["contact_table"], dtype=int), m)
        assert lb.tobytes() == np.asarray(qp["lbx"], dtype=float).tobytes()
        assert ub.tobytes() == np.asarray(qp["ubx"], dtype=float).tobytes()


# ---- 线性代价 g ----

from iraf_adapters.unitree.mpc.qp_builder import linear_cost  # noqa: E402


def test_linear_cost_shape_and_force_block_zero():
    m = _model()
    x_ref = np.ones((12, int(m["horizon"])))
    g = linear_cost(x_ref, m)
    assert g.shape == (384,)
    assert np.all(g[192:] == 0.0)                       # 力段不参与代价


def test_linear_cost_is_column_major_and_q_weighted():
    m = _model()
    n = int(m["horizon"])
    x_ref = np.zeros((12, n))
    x_ref[2, 0] = 1.0                                   # 只激励第 0 拍的 z（q=50）
    g = linear_cost(x_ref, m)
    assert g[2] == pytest.approx(-2.0 * 50.0)
    assert np.count_nonzero(g) == 1
    # 列优先：第 k 拍的状态放在 k*12 起（不是"按行拍平"）
    x_ref2 = np.zeros((12, n))
    x_ref2[0, 1] = 1.0
    assert np.flatnonzero(linear_cost(x_ref2, m)).tolist() == [12]


@pytest.mark.parametrize("bad_x", [np.zeros((12, 5)), np.zeros((11, 16)), np.zeros((13, 16))])
def test_linear_cost_invalid_x_ref(bad_x):
    with pytest.raises(ValueError):
        linear_cost(bad_x, _model())


def test_linear_cost_missing_horizon():
    with pytest.raises(ValueError):
        linear_cost(np.zeros((12, 16)), {"q_diag": [1.0] * 12})


def test_linear_cost_matches_captured_upstream_bitwise():
    import json
    path = ROOT / "build" / "research" / "mpc-repo" / "qp_inputs.json"
    if not path.is_file():
        pytest.skip("缺少截获证据 %s" % path)
    captured = json.loads(path.read_text(encoding="utf-8"))
    m = _model()
    checked = 0
    for qp in captured["qps"]:
        if "x_ref" not in qp:
            pytest.skip("证据文件未含 x_ref（需重跑探针）")
        g = linear_cost(np.asarray(qp["x_ref"], dtype=float), m)
        assert g.tobytes() == np.asarray(qp["g"], dtype=float).tobytes()
        checked += 1
    assert checked >= 20


# ---- 摩擦锥块 ----

from iraf_adapters.unitree.mpc.qp_builder import friction_rows  # noqa: E402

#: 上游模块常量 MU（与其 MJCF 足端 0.8 配对）；我们声明的是 0.4（与厂商 MJCF 0.4 配对）。
UPSTREAM_MU = 0.8


def test_friction_rows_count_layout_and_coefficients():
    m = _model()
    out = friction_rows(_mask(1), m)
    assert out["n_rows"] == 4 * 4 * int(m["horizon"]) == 256   # 4 腿 × 4 面 × N
    trip = list(zip(out["rows"], out["cols"], out["vals"]))
    # 第一拍第一条腿的 4 个面：+fx−μfz、−fx−μfz、+fy−μfz、−fy−μfz（列基准 baseU = 192）
    mu = float(m["mu"])
    assert trip[0:2] == [(0, 192 + 0, 1.0), (0, 192 + 2, -mu)]
    assert trip[2:4] == [(1, 192 + 0, -1.0), (1, 192 + 2, -mu)]
    assert trip[4:6] == [(2, 192 + 1, 1.0), (2, 192 + 2, -mu)]
    assert trip[6:8] == [(3, 192 + 1, -1.0), (3, 192 + 2, -mu)]
    # 第二条腿从列 192+3 起、行 4 起
    assert trip[8][0] == 4 and trip[8][1] == 192 + 3


def test_friction_upper_bound_zero_only_for_stance():
    m = _model()
    n = int(m["horizon"])
    out = friction_rows(_mask(1), m)
    assert np.all(out["upper_bound"] == 0.0) and np.all(np.isneginf(out["lower_bound"]))
    swing_all = friction_rows(_mask(0), m)
    assert np.all(np.isposinf(swing_all["upper_bound"]))
    mask = _mask(1, n)
    mask[3, 0] = 0                                     # 第 0 拍 RR 摆动
    out2 = friction_rows(mask, m)
    assert out2["upper_bound"][((0 * 4) + 3) * 4:((0 * 4) + 3) * 4 + 4].tolist() == [np.inf] * 4
    assert out2["upper_bound"][0] == 0.0               # FL 同拍仍支撑


@pytest.mark.parametrize("bad", [
    np.zeros((3, 16), dtype=int), np.zeros((4, 5), dtype=int),
])
def test_friction_rows_bad_contact_table(bad):
    with pytest.raises(ValueError):
        friction_rows(bad, _model())


def test_friction_rows_bad_model():
    with pytest.raises(ValueError):
        friction_rows(_mask(1), {"horizon": 16})
    with pytest.raises(ValueError):
        friction_rows(_mask(1), {"horizon": 16, "mu": 0.0})
    bad = _mask(1)
    bad[1, 1] = 3
    with pytest.raises(ValueError):
        friction_rows(bad, _model())


def test_friction_rows_match_captured_upstream_bitwise():
    """判据用**上游 MU=0.8**（探针跑的是上游代码，其 mu 是模块常量）。

    这恰好固化一条 ADR §6.5 的事实：摩擦系数按"同侧配对"——我们声明 0.4（配厂商 MJCF 足端 0.4），
    上游 0.8（配它自己的 MJCF）。所以"同语义"比对必须把 mu 换成 0.8；本函数在 0.4 下产出
    的 fz 系数是 −0.4，已由上一用例固定。
    """
    import json
    path = ROOT / "build" / "research" / "mpc-repo" / "qp_inputs.json"
    if not path.is_file():
        pytest.skip("缺少截获证据 %s" % path)
    captured = json.loads(path.read_text(encoding="utf-8"))
    model = dict(_model())
    model["mu"] = UPSTREAM_MU
    base = int(model["horizon"]) * 12
    checked = 0
    for qp in captured["qps"]:
        if "contact_table" not in qp:
            pytest.skip("证据文件未含 contact_table（需重跑探针）")
        out = friction_rows(np.asarray(qp["contact_table"], dtype=int), model)
        mine = {(r + base, c): v for r, c, v in zip(out["rows"], out["cols"], out["vals"])}
        up = {(r, c): v for r, c, v in zip(qp["a_rows"], qp["a_cols"], qp["a_vals"]) if r >= base}
        assert set(mine) == set(up)
        assert all(mine[k] == up[k] for k in up)
        assert out["upper_bound"].tobytes() == np.asarray(qp["uba"], dtype=float)[base:].tobytes()
        checked += 1
    assert checked >= 20
