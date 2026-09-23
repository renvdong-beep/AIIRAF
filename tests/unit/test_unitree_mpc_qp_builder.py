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
