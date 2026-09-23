"""`mpc.reference` 的状态相关轨迹：形状、钳位、旋变换、线性偏航与参数校验。"""

from __future__ import annotations

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.reference import reference_state_trajectory, yaw_rotation


def _state(x, y, z=0.27, yaw=0.0):
    s = np.zeros(12)
    s[0], s[1], s[2] = x, y, z
    s[5] = yaw
    return s


def _call(**kw):
    args = dict(initial_x_vec=_state(0.0, 0.0), com_pos_world=(0.0, 0.0, 0.27), yaw=0.0,
                n_steps=4, time_step=0.0208333, vx_body=0.5, vy_body=0.0, z_des=0.27,
                yaw_rate=0.0, pos_des_world=(0.1, 0.0, 0.27), max_pos_error=0.1)
    args.update(kw)
    return reference_state_trajectory(**args)


def test_shapes_and_dtype():
    pos, vel, rpy, omega, _ = _call()
    for arr in (pos, vel, rpy, omega):
        assert arr.shape == (3, 4)
        assert arr.dtype == np.float64


def test_time_vector_starts_at_one_dt():
    pos, _, _, _, _ = _call(vx_body=1.0, n_steps=3, time_step=0.5)
    assert np.allclose(pos[0], [0.1 + 0.5, 0.1 + 1.0, 0.1 + 1.5])


def test_z_is_overridden_by_z_des():
    _, _, _, _, p_des = _call(z_des=0.33, pos_des_world=(0.0, 0.0, 0.99))
    assert p_des[2] == pytest.approx(0.33)


def test_clamp_both_directions_on_x_and_y():
    _, _, _, _, p = _call(com_pos_world=(0.0, 0.0, 0.27), pos_des_world=(0.5, -0.5, 0.27),
                          max_pos_error=0.1)
    assert p[0] == pytest.approx(0.1)
    assert p[1] == pytest.approx(-0.1)
    # 阈值内不动
    _, _, _, _, q = _call(pos_des_world=(0.05, -0.05, 0.27), max_pos_error=0.1)
    assert q[0] == pytest.approx(0.05) and q[1] == pytest.approx(-0.05)


def test_zero_threshold_pins_to_com():
    _, _, _, _, p = _call(pos_des_world=(0.5, 0.5, 0.27), max_pos_error=0.0)
    assert p[0] == pytest.approx(0.0) and p[1] == pytest.approx(0.0)


def test_velocity_is_rotated_into_world():
    _, vel, _, _, _ = _call(yaw=np.pi / 2, vx_body=0.4, vy_body=0.3)
    assert np.allclose(vel[:, 0], [-0.3, 0.4, 0.0], atol=1e-15)
    assert np.allclose(vel[:, 3], vel[:, 0])


def test_yaw_rate_only_touches_z_axis():
    _, _, rpy, omega, _ = _call(yaw=0.2, yaw_rate=0.5, n_steps=2, time_step=0.1)
    assert rpy[0].tolist() == [0.0, 0.0] and rpy[1].tolist() == [0.0, 0.0]
    assert np.allclose(rpy[2], [0.2 + 0.05, 0.2 + 0.10])
    assert omega[2, 0] == pytest.approx(0.5)
    assert omega[0, 0] == 0.0 and omega[1, 0] == 0.0


def test_yaw_rotation_is_right_handed():
    R = yaw_rotation(np.pi / 2)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-15)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-15)


@pytest.mark.parametrize("bad", [
    {"initial_x_vec": np.zeros(11)},
    {"n_steps": 0},
    {"time_step": 0.0},
    {"time_step": -0.1},
    {"max_pos_error": -1e-9},
])
def test_invalid_inputs_raise(bad):
    with pytest.raises(ValueError):
        _call(**bad)
