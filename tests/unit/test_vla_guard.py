"""VLA 动作护栏单元测试（不依赖 torch / 网络）。"""
from __future__ import annotations

import numpy as np

from iraf_adapters.vla.guard import ActionGuard, VlaLimits


def test_accept_in_range():
    g = ActionGuard()
    res = g.check([0.0, 0.8, -0.1, 0.0, -0.1, 0.0, 0.02])
    assert res.accepted
    assert not res.clamped
    assert np.allclose(res.action, [0.0, 0.8, -0.1, 0.0, -0.1, 0.0, 0.02])


def test_dim_mismatch_rejected():
    g = ActionGuard()
    res = g.check([0.0] * 6)
    assert not res.accepted
    assert any("dim=" in v for v in res.violations)


def test_joint_clamped():
    g = ActionGuard()
    res = g.check([9.0, 0.8, -0.1, 0.0, -0.1, 0.0, 0.02])
    assert not res.accepted
    assert res.action[0] == VlaLimits().joint_max[0]
    assert any("joint1" in v for v in res.violations)


def test_rate_limit():
    g = ActionGuard()
    res = g.check([0.5, 0.8, -0.1, 0.0, -0.1, 0.0, 0.02],
                  previous=[0.0, 0.8, -0.1, 0.0, -0.1, 0.0, 0.02])
    assert not res.accepted
    assert abs(res.action[0] - 0.0) <= VlaLimits().max_step_rad + 1e-9
    assert "rate-limit" in res.violations


def test_gripper_clamped():
    g = ActionGuard()
    res = g.check([0.0, 0.8, -0.1, 0.0, -0.1, 0.0, 9.9])
    assert not res.accepted
    assert res.action[6] == VlaLimits().gripper_max
    assert any("gripper" in v for v in res.violations)


def test_non_finite_rejected():
    g = ActionGuard()
    res = g.check([0.0, float("nan"), 0.0, 0.0, 0.0, 0.0, 0.02])
    assert not res.accepted
    assert "non-finite" in res.violations
