"""`mpc.reference.foot_reference_trajectory`：逐拍状态机语义（离地/触地/未变）、
落足点公式（nominal + drift + rot − base_pos）、腿序来源与入参门禁。"""

from __future__ import annotations

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.contact import LEG_ORDER
from iraf_adapters.unitree.mpc.reference import foot_reference_trajectory

HIP = {"FL": (0.19, 0.047, 0.0), "FR": (0.19, -0.047, 0.0),
       "RL": (-0.19, 0.047, 0.0), "RR": (-0.19, -0.047, 0.0)}
SWING, STANCE, NOM_Z = 0.13333333333333333, 0.2, 0.02


def _call(masks, base_pos=None, vel=(0.0, 0.0, 0.0), yaw=0.0, yaw_rate=0.0, n_cols=None):
    n = masks.shape[1] if n_cols is None else n_cols
    if base_pos is None:
        base_pos = np.tile(np.array([0.0, 0.0, 0.27]).reshape(3, 1), (1, n))
    from iraf_adapters.unitree.mpc.reference import yaw_rotation
    return foot_reference_trajectory(masks, base_pos, vel, yaw_rotation(yaw), yaw_rate,
                                     HIP, SWING, STANCE, NOM_Z)


def test_hand_computed_takeoff_then_touchdown_then_hold():
    # FL: 支撑 → 摆动(i=1,离地) → 摆动(i=2,未变) → 支撑(i=3,触地)
    masks = np.array([
        [1, 0, 0, 1],
        [1, 1, 1, 1],
        [1, 1, 1, 1],
        [1, 1, 1, 1],
    ])
    out = _call(masks)
    pred = (SWING + 0.5 * STANCE) / 2.0
    # 无速度、无偏航：td = [hip_x, hip_y, 0.02]（机身 xy = 0），参考 = td − base_pos
    td = np.array([HIP["FL"][0], HIP["FL"][1], NOM_Z]) - np.array([0.0, 0.0, 0.27])
    assert np.allclose(out["FL"][:, 1], [0.0, 0.0, 0.0])          # 离地当拍置零
    assert np.allclose(out["FL"][:, 2], 0.0)                       # 掩码未变 ⇒ 仍为上一拍（零）
    assert np.allclose(out["FL"][:, 3], td)                        # 触地取离地时记录的落足点
    assert np.allclose(out["FR"], 0.0)                             # 其它腿全程支撑且第 0 拍即触地
    assert np.allclose(out["FR"][:, 0], np.array([-0.0, 0.0, 0.0]))  # 第 0 拍走触地分支（初值 2）
    assert np.allclose(out["FR"][:, 0], 0.0)
    assert abs(pred - (SWING + 0.5 * STANCE) / 2.0) == 0.0


def test_touchdown_records_state_at_takeoff_step_not_at_touchdown_step():
    masks = np.array([
        [1, 0, 0, 1],
        [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
    ])
    base = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.27, 0.27, 0.27, 0.27]])
    vel = (0.5, 0.0, 0.0)
    out = _call(masks, base_pos=base, vel=vel)
    pred = (SWING + 0.5 * STANCE) / 2.0
    assert np.allclose(out["FL"][:, 3], np.array([HIP["FL"][0] + 0.5 * pred,
                                                  HIP["FL"][1], NOM_Z - 0.27]))
    # 机身位置在触地拍变了，但参考仍是离地拍（i=1）算出的值 ⇒ 换 base_pos 不影响已记录值
    base2 = base.copy()
    base2[0, 3] = 0.123
    out2 = _call(masks, base_pos=base2, vel=vel)
    assert np.allclose(out["FL"][:, 3], out2["FL"][:, 3])


def test_rotation_correction_term_and_sign():
    masks = np.array([[1, 0, 1]] + [[1, 1, 1]] * 3)
    out = _call(masks, yaw=0.0, yaw_rate=2.0)
    pred = (SWING + 0.5 * STANCE) / 2.0
    dtheta = 2.0 * pred
    r_xy = np.array([HIP["FL"][0], HIP["FL"][1]])       # nominal[:2] − base[:2]（base xy = 0）
    expect = np.array([r_xy[0] - dtheta * r_xy[1], r_xy[1] + dtheta * r_xy[0], NOM_Z - 0.27])
    assert np.allclose(out["FL"][:, 2], expect)
    # 反号即被这条例行测试抓住
    wrong = np.array([r_xy[0] + dtheta * r_xy[1], r_xy[1] - dtheta * r_xy[0], NOM_Z - 0.27])
    assert not np.allclose(out["FL"][:, 2], wrong)


def test_hip_offset_is_rotated_by_current_yaw_not_trajectory_yaw():
    masks = np.array([[1, 0, 1]] + [[1, 1, 1]] * 3)
    yaw = np.pi / 2
    out = _call(masks, yaw=yaw)
    off = np.array(HIP["FL"])
    rotated = np.array([-off[1], off[0], off[2]])      # R_z(90°) @ off
    assert np.allclose(out["FL"][:, 2], rotated - np.array([0.0, 0.0, 0.27]) + np.array([0, 0, NOM_Z]))
    assert not np.allclose(out["FL"][:, 2], off - np.array([0.0, 0.0, 0.27]) + np.array([0, 0, NOM_Z]))


def test_per_step_base_velocity_supported_and_takeoff_column_used():
    masks = np.array([[1, 0, 1]] + [[1, 1, 1]] * 3)   # FL 在 i=1 离地、i=2 触地
    # (3,N) 形式：x 行在 i=1 处为 0.5（离地拍）⇒ 应取该列，而不是第 0 列（0.9）或第 2 列（0.1）
    vel = np.array([[0.9, 0.5, 0.1], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    out = _call(masks, vel=vel)
    pred = (SWING + 0.5 * STANCE) / 2.0
    expect = np.array([HIP["FL"][0] + 0.5 * pred, HIP["FL"][1], NOM_Z - 0.27])
    assert np.allclose(out["FL"][:, 2], expect)
    for wrong_col in (0.9, 0.1):
        assert not np.allclose(out["FL"][:, 2],
                               np.array([HIP["FL"][0] + wrong_col * pred, HIP["FL"][1],
                                         NOM_Z - 0.27]))


@pytest.mark.parametrize("bad", [
    {"masks": np.zeros((3, 4), dtype=int)},
    {"hip_offsets": {k: v for k, v in HIP.items() if k != "RL"}},
    {"hip_offsets": dict(HIP, XX=(0.0, 0.0, 0.0))},
    {"swing_time_s": 0.0},
    {"swing_time_s": -1.0},
    {"stance_time_s": 0.0},
    {"base_pos_traj": np.zeros((3, 5))},
    {"base_vel_body": np.zeros((3, 5))},
    {"r_z": np.eye(2)},
])
def test_invalid_inputs_raise(bad):
    masks = np.array([[1, 0, 1]] + [[1, 1, 1]] * 3)
    kwargs = dict(masks=masks, base_pos_traj=np.tile([0.0, 0.0, 0.27], (3, 1)),
                  base_vel_body=(0.0, 0.0, 0.0), r_z=np.eye(3), yaw_rate_des=0.0,
                  hip_offsets=HIP, swing_time_s=SWING, stance_time_s=STANCE, nominal_z_m=NOM_Z)
    kwargs.update(bad)
    with pytest.raises(ValueError):
        foot_reference_trajectory(**kwargs)


def test_garbage_mask_value_first_column_hits_unreachable_guard():
    """掩码含非 0/1 值（上游初值 2 的语义）时，第 0 怕"未变"分支必须显式失败而非读 [-1]。"""
    masks = np.array([[2, 1]] + [[1, 1]] * 3)
    with pytest.raises(ValueError):
        _call(masks)


def test_rows_follow_leg_order():
    assert list(LEG_ORDER) == ["FL", "FR", "RL", "RR"]
    masks = np.array([[1, 0, 1]] + [[1, 1, 1]] * 3)
    out = _call(masks)
    assert list(out.keys()) == list(LEG_ORDER)
