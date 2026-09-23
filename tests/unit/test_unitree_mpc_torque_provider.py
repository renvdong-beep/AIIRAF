"""`mpc.torque_provider`：腿序映射、Jᵀ 乘法、B1 载荷与入参门禁。"""

from __future__ import annotations

import numpy as np
import pytest

from iraf_adapters.unitree.mpc.contact import LEG_ORDER
from iraf_adapters.unitree.mpc.torque_provider import (foot_forces_matrix, joint_torques,
                                                       torque_provider_payload)


def _eye_jt():
    return {code: np.eye(3) for code in LEG_ORDER}


def test_foot_forces_matrix_accepts_dict_and_arrays():
    forces = {code: np.array([1.0, 2.0, 3.0]) for code in LEG_ORDER}
    m = foot_forces_matrix(forces)
    assert m.shape == (4, 3) and np.allclose(m[0], [1.0, 2.0, 3.0])
    flat = np.arange(12, dtype=float)
    assert np.allclose(foot_forces_matrix(flat), foot_forces_matrix(flat.reshape(4, 3)))


def test_identity_jacobian_passes_forces_through_in_leg_order():
    forces = {code: np.zeros(3) for code in LEG_ORDER}
    forces["RL"] = np.array([10.0, 0.0, -5.0])
    tau = joint_torques(forces, _eye_jt())
    assert tau.shape == (12,)
    assert np.allclose(tau[6:9], [10.0, 0.0, -5.0])       # RL 在 LEG_ORDER 的第 2 位 ⇒ 分量 6..8
    assert np.allclose(np.delete(tau, [6, 7, 8]), 0.0)    # 只有 RL 发力 ⇒ 其余为零


def test_hand_computed_jacobian_product():
    jt = _eye_jt()
    jt["FL"] = np.array([[2.0, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, -1.0]])
    forces = {code: np.zeros(3) for code in LEG_ORDER}
    forces["FL"] = np.array([3.0, 4.0, 5.0])
    tau = joint_torques(forces, jt)
    assert np.allclose(tau[0:3], [6.0, 2.0, -5.0])


def test_zero_forces_give_zero_torques():
    tau = joint_torques({code: np.zeros(3) for code in LEG_ORDER}, _eye_jt())
    assert np.all(tau == 0.0)


def test_payload_is_negative_of_jacobian_product():
    """载荷里的 `balance_torque_nm` 必须是**执行器支撑力矩** = −Jᵀ·f（地面反力 f 取 +z 向上）。

    符号依据（实测，探针 `build/iraf-a6a4/jt_sign_probe.py`）：τ=−Jᵀf 撑住机身
    （Δh=+0.001714 m、四腿法向合力 116.025 N）；τ=+Jᵀf 把足端卸掉（Δh=−0.000563 m、合力 0.0）。
    本用例把该符号钉成回归：`joint_torques` 保持 +Jᵀf 的纯映射，载荷构造取负。
    """
    forces = {code: np.zeros(3) for code in LEG_ORDER}
    forces["FL"] = np.array([1.0, 2.0, 3.0])
    forces["RR"] = np.array([0.0, 0.0, 38.250191])
    payload = torque_provider_payload(forces, _eye_jt(), np.zeros(12))
    direct = joint_torques(forces, _eye_jt())
    assert np.allclose(payload["balance_torque_nm"], -direct)
    assert np.allclose(payload["balance_torque_nm"][0:3], [-1.0, -2.0, -3.0])
    assert payload["balance_torque_nm"][11] == pytest.approx(-38.250191)


def test_payload_shape_and_keys():
    payload = torque_provider_payload({code: np.zeros(3) for code in LEG_ORDER}, _eye_jt(),
                                      np.zeros(12))
    assert set(payload) == {"balance_torque_nm", "position_weight"}
    assert payload["balance_torque_nm"].shape == (12,)
    assert payload["position_weight"].shape == (12,)


def test_payload_does_not_mutate_caller_weight():
    weight = np.full(12, 0.25)
    payload = torque_provider_payload({code: np.zeros(3) for code in LEG_ORDER}, _eye_jt(), weight)
    payload["position_weight"][0] = 0.9
    assert weight[0] == pytest.approx(0.25)


@pytest.mark.parametrize("bad_weight", [None, np.zeros(11), np.zeros(13),
                                        np.full(12, 1.5), np.full(12, -0.1),
                                        np.array([np.nan] + [0.0] * 11)])
def test_payload_rejects_bad_position_weight(bad_weight):
    with pytest.raises((ValueError, TypeError)):
        torque_provider_payload({code: np.zeros(3) for code in LEG_ORDER}, _eye_jt(), bad_weight)


@pytest.mark.parametrize("bad_forces", [
    {code: np.zeros(3) for code in LEG_ORDER if code != "RR"},          # 缺腿
    dict({code: np.zeros(3) for code in LEG_ORDER}, XX=np.zeros(3)),    # 多腿
    np.zeros((3, 3)),
    np.zeros(11),
    {code: np.array([np.inf, 0.0, 0.0]) for code in LEG_ORDER},
])
def test_rejects_bad_foot_forces(bad_forces):
    with pytest.raises(ValueError):
        joint_torques(bad_forces, _eye_jt())


def test_rejects_bad_jacobian():
    forces = {code: np.zeros(3) for code in LEG_ORDER}
    bad_missing = {code: np.eye(3) for code in LEG_ORDER if code != "FL"}
    with pytest.raises(ValueError):
        joint_torques(forces, bad_missing)
    bad_shape = _eye_jt()
    bad_shape["FR"] = np.eye(4)
    with pytest.raises(ValueError):
        joint_torques(forces, bad_shape)
    bad_extra = dict(_eye_jt(), ZZ=np.eye(3))
    with pytest.raises(ValueError):
        joint_torques(forces, bad_extra)


# ---------- 转置回归（2026-09-23：漏转置只把「水平」搞反，单位矩阵掩盖了它） ----------

#: 生产实测的前右腿平动雅可比（行 = 世界 x,y,z；列 = (hip, thigh, calf)）。
#: 来源：`build/iraf-a6a6/sign_axis_probe.py` A 段（解析值 vs 数值微分 max|Δ| = 1.324e-07）。
FR_JACOBIAN = np.array([[0.000000, -0.264806, -0.132403],
                        [0.264806, 0.000000, 0.000000],
                        [-0.095500, 0.000000, -0.166849]])


def test_fr_jacobian_is_asymmetric_so_identity_tests_cannot_catch_transpose():
    """前提检查：生产雅可比**非对称** ⇒ 用单位矩阵的用例根本验证不了转置。

    这正是缺陷能长期潜伏的原因：`_eye_jt()` 下 `J·f == Jᵀ·f`。
    """
    assert not np.allclose(FR_JACOBIAN, FR_JACOBIAN.T)


def test_jacobian_transpose_from_transposes():
    """接线入口必须给 **Jᵀ**：`jacobian_transpose_from(J) == J.T`（且不改原矩阵）。"""
    from iraf_adapters.unitree.mpc.torque_provider import jacobian_transpose_from

    transposed = jacobian_transpose_from(FR_JACOBIAN)
    np.testing.assert_allclose(transposed, FR_JACOBIAN.T, atol=0.0)
    transposed[0, 0] = 123.0                       # 返回的是副本：不许污染调用方矩阵
    assert FR_JACOBIAN[0, 0] == 0.000000
    with pytest.raises(ValueError):
        jacobian_transpose_from(np.eye(4))
    with pytest.raises(ValueError):
        jacobian_transpose_from(np.full((3, 3), np.nan))


def test_horizontal_torques_flip_sign_if_transpose_is_missed():
    """回归本体：非对称雅可比下，漏转置会把**水平**方向的力矩符号搞反。

    实测依据（同上探针，FR 腿）：
      · f=(40,0,0)：J·f = (0, +10.5922, −3.8200)  vs Jᵀ·f = (0, −10.5922, −5.2961)（thigh 反号）
      · f=(0,40,0)：J·f = (−10.5922, 0, 0)        vs Jᵀ·f = (+10.5922, 0, 0)（hip 反号）
      · f=(0,0,38.25)：两者**同号**（calf 项完全相同）⇒ 竖直方向看不出缺陷
    ⇒ 本用例既钉住「水平必须用 Jᵀ」，也钉住「竖直两种写法同号」（说明为什么它潜伏）。
    """
    from iraf_adapters.unitree.mpc.torque_provider import jacobian_transpose_from

    jt = {code: np.eye(3) for code in LEG_ORDER}
    jt["FR"] = jacobian_transpose_from(FR_JACOBIAN)

    def torques_for(force):
        forces = {code: np.zeros(3) for code in LEG_ORDER}
        forces["FR"] = np.array(force, dtype=float)
        return joint_torques(forces, jt)[3:6]

    for axis_force in ([40.0, 0.0, 0.0], [0.0, 40.0, 0.0]):
        correct = FR_JACOBIAN.T @ np.array(axis_force)
        missed = FR_JACOBIAN @ np.array(axis_force)
        np.testing.assert_allclose(torques_for(axis_force), correct, atol=1e-12)
        # 漏转置的那一支必须与正确值显著不同（否则本用例抓不到缺陷）
        assert not np.allclose(correct, missed, atol=1e-6), axis_force

    vertical = [0.0, 0.0, 38.25]
    correct_v = FR_JACOBIAN.T @ np.array(vertical)
    missed_v = FR_JACOBIAN @ np.array(vertical)
    np.testing.assert_allclose(torques_for(vertical), correct_v, atol=1e-12)
    # 竖直方向：两支**逐分量同号**（这就是缺陷潜伏的原因，写进测试防复发）
    assert np.all(np.sign(correct_v) == np.sign(missed_v)), (correct_v, missed_v)
    assert not np.allclose(correct_v, missed_v, atol=1e-9)   # 但数值仍不同（hip 项）
