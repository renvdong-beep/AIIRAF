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


def test_diagonal_layout_matches_upstream_block_order():
    model = _model()
    out = cost_diagonal(model)
    d = out["h_diag"]
    # 每拍 24 项：前 12 = 2·q_diag，后 12 = 2·r_diag；跨拍重复
    for step in range(out["horizon"]):
        block = d[step * 24:(step + 1) * 24]
        assert np.array_equal(block[:12], 2.0 * out["q_diag"])
        assert np.array_equal(block[12:], 2.0 * out["r_diag"])
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
