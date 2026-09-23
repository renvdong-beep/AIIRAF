"""单测：`dynamics.discrete_dynamics`（A6a-3b 第②步第一块）—— 结构性质 + 公式逐项复核。

说明：本文件验证的是**结构性质与公式取法**（不是"与我自己的实现一致"那种同义反复）；
与研究侧实测值的**逐位比对**是下一步（`traj_parity` 门禁，见移植清单 §2），
在此之前本模块不得接进 `provider_runtime`。
"""

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.dynamics import GRAVITY, discrete_dynamics, skew, yaw_avg_rotation

DT = 1.0 / 3.0 / 16.0        # GAIT_T/16
MASS = 15.206408             # 厂商 MJCF 实测值
I_COM = np.diag([0.1, 0.1, 0.2])


def _feet(n, scale=1.0):
    """构造 (4,3,n) 的足端世界位置（四足对称分布）。"""
    base = np.array([[0.19, 0.047, -0.213], [0.19, -0.047, -0.213],
                     [-0.19, 0.047, -0.213], [-0.19, -0.047, -0.213]], dtype=float)
    return np.repeat(base[:, :, None], n, axis=2) * scale


def test_skew_is_antisymmetric_with_correct_entries():
    v = np.array([1.0, 2.0, 3.0])
    S = skew(v)
    assert np.allclose(S, -S.T)
    assert np.allclose(S @ v, np.zeros(3))          # 叉乘自身为零
    assert S[0, 1] == -v[2] and S[1, 0] == v[2]


def test_yaw_avg_rotation_uses_horizon_average_yaw():
    rpy = np.zeros((3, 4))
    rpy[2, :] = [0.0, 0.1, 0.2, 0.3]                 # 平均偏航 = 0.15
    RzT = yaw_avg_rotation(rpy)
    c, s = np.cos(0.15), np.sin(0.15)
    assert np.allclose(RzT, np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]))
    assert np.isclose(np.linalg.det(RzT), 1.0)


def test_ad_blocks_match_upstream_convention():
    Ad, _Bd, _gd = discrete_dynamics(DT, MASS, I_COM, np.zeros((3, 2)), _feet(2))
    assert Ad.shape == (12, 12)
    assert np.allclose(Ad[0:3, 6:9], DT * np.eye(3))          # p ← v
    assert Ad[6:9, 0:3].sum() == 0.0                          # v 不受 p 影响
    # 其余位置保持**单位阵**（上游 Ad 也以 I12 为底；我首版误写成"全零" ⇒ 被本用例抓出）
    mask = np.ones((12, 12), dtype=bool)
    mask[0:3, 6:9] = False
    mask[3:6, 9:12] = False
    assert np.allclose(Ad[mask], np.eye(12)[mask])


def test_gd_has_half_dt2_in_position_and_dt_in_velocity():
    _Ad, _Bd, gd = discrete_dynamics(DT, MASS, I_COM, np.zeros((3, 1)), _feet(1))
    assert np.allclose(gd[0:3, 0], 0.5 * GRAVITY * DT * DT)
    assert np.allclose(gd[6:9, 0], GRAVITY * DT)
    assert np.allclose(gd[3:6, 0], 0.0) and np.allclose(gd[9:12, 0], 0.0)


def test_bd_blocks_and_shapes():
    n = 3
    _Ad, Bd, _gd = discrete_dynamics(DT, MASS, I_COM, np.zeros((3, n)), _feet(n))
    assert Bd.shape == (n, 12, 12)
    Bp = (0.5 * DT * DT / MASS) * np.eye(3)
    Bv = (DT / MASS) * np.eye(3)
    for i in range(n):
        for leg in range(4):
            sl = slice(3 * leg, 3 * leg + 3)
            assert np.allclose(Bd[i][0:3, sl], Bp)            # p ← f
            assert np.allclose(Bd[i][6:9, sl], Bv)            # v ← f
        # ω ← f 与 rpy ← f 与足端力臂相关
        assert not np.allclose(Bd[i][9:12, 0:3], 0.0)
        assert not np.allclose(Bd[i][3:6, 0:3], 0.0)


def test_bd_omega_block_uses_inertia_inverse_times_skew():
    n = 1
    feet = _feet(n)
    _Ad, Bd, _gd = discrete_dynamics(DT, MASS, I_COM, np.zeros((3, n)), feet)
    W0 = np.linalg.inv(I_COM) @ skew(feet[0, :, 0])
    assert np.allclose(Bd[0][9:12, 0:3], DT * W0)
    assert np.allclose(Bd[0][3:6, 0:3], 0.5 * DT * DT * (yaw_avg_rotation(np.zeros((3, 1))) @ W0))


@pytest.mark.parametrize("dt,mass", [(0.0, MASS), (-0.02, MASS), (DT, 0.0), (DT, -1.0)])
def test_non_positive_inputs_are_rejected(dt, mass):
    with pytest.raises(ValueError):
        discrete_dynamics(dt, mass, I_COM, np.zeros((3, 1)), _feet(1))


@pytest.mark.parametrize("rpy,feet", [((3,), _feet(2)), ((4, 2), _feet(2)),
                                     ((3, 2), _feet(3))])
def test_shape_mismatches_are_rejected(rpy, feet):
    with pytest.raises(ValueError):
        discrete_dynamics(DT, MASS, I_COM, np.zeros(rpy), feet)
