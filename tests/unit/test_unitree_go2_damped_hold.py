"""单测：`damped_hold` 的判据逻辑与分派（不依赖完整仿真夹具的部分）。

为什么要测这些：`damped_hold` 是**安全路径**，其中两处最容易做歪、且一旦做歪后果最重：
  1. "连续达标"的判定——**单次低于容差不算达标**（否则机器人在减速过程中一瞬抖动就被判成功）；
  2. 停机模式分派——未知模式必须**显式失败**，且 `damped_hold` 必须真的被路由过去（不静默退回失能停机）。

完整仿真级用例（移动中停止、超时失败、安全事件、loopback 回归）需要既有仿真夹具，属下一批。
"""

import numpy as np
import pytest

from iraf_adapters.unitree.unitree_go2 import (
    SUPPORTED_STOP_MODES,
    UnitreeGo2Adapter,
)
from iraf_adapters.unitree.quadruped import CommandRejectedError


class _Stub:
    """最小替身：只提供被判方法用到的属性，不构造 MuJoCo/Profile。"""

    control_hz = 200.0

    def __init__(self):
        self.routed = []

    def damped_hold(self, lease, execution_id=None):
        self.routed.append(("damped_hold", lease, execution_id))
        return {"stop_mode": "damped_hold"}

    def _stop_torque_zero_release(self, lease, execution_id=None):
        self.routed.append(("torque_zero_release", lease, execution_id))
        return {"stop_mode": "torque_zero_release"}


def test_supported_stop_modes_now_include_both():
    assert "damped_hold" in SUPPORTED_STOP_MODES
    assert "torque_zero_release" in SUPPORTED_STOP_MODES


def test_dispatch_routes_each_mode_and_rejects_unknown():
    stub = _Stub()
    assert UnitreeGo2Adapter.stop(stub, "L", "E", mode=None)["stop_mode"] == "torque_zero_release"
    assert UnitreeGo2Adapter.stop(stub, "L", "E", mode="torque_zero_release")["stop_mode"] == \
        "torque_zero_release"
    assert UnitreeGo2Adapter.stop(stub, "L", "E", mode="damped_hold")["stop_mode"] == "damped_hold"
    assert [r[0] for r in stub.routed] == ["torque_zero_release", "torque_zero_release", "damped_hold"]
    with pytest.raises(CommandRejectedError):
        UnitreeGo2Adapter.stop(stub, "L", "E", mode="coast")          # 未知模式必须显式失败
    with pytest.raises(CommandRejectedError):
        UnitreeGo2Adapter.stop(stub, "L", "E", mode="")               # 空串同样拒绝


def test_default_mode_keeps_legacy_behaviour():
    """`mode` 省略时行为不变（既有调用 `stop(lease, execution_id)` 必须走失能停机）。"""
    stub = _Stub()
    UnitreeGo2Adapter.stop(stub, "L", "E")
    assert stub.routed == [("torque_zero_release", "L", "E")]


def test_continuous_below_requires_unbroken_run():
    """连续达标判定：中间断一次就不得算达标（这是最容易做歪的一处）。"""
    probe = _Stub()
    probe.control_hz = 10.0                       # 1 样本 = 0.1 s
    f = UnitreeGo2Adapter._continuous_below
    # 需要连续 0.5 s = 5 个样本
    assert f(probe, [0.01] * 5, 0.05, 0.5) is True
    assert f(probe, [0.01] * 4, 0.05, 0.5) is False          # 时长不足
    assert f(probe, [0.01] * 4 + [0.9] + [0.01] * 4, 0.05, 0.5) is False   # 中间断了
    assert f(probe, [0.9] + [0.01] * 5, 0.05, 0.5) is True   # 断在开头不算断
    assert f(probe, [0.05] * 9, 0.05, 0.5) is False          # 恰好等于阈值**不算**低于（严格小于）
    assert f(probe, [], 0.05, 0.5) is False                  # 无样本不得判达标


def test_continuous_below_uses_control_frequency():
    """窗口长度按控制频率换算：同样 5 个样本，100 Hz 下只等于 0.05 s ⇒ 不达标。"""
    probe = _Stub()
    probe.control_hz = 100.0
    f = UnitreeGo2Adapter._continuous_below
    assert f(probe, [0.01] * 5, 0.05, 0.5) is False
    assert f(probe, [0.01] * 50, 0.05, 0.5) is True


def test_safety_event_probe_is_fail_safe_by_default():
    """`_safety_event_active` 在取不到状态时按"无事件"处理（不误报安全事件）。"""
    probe = _Stub()
    assert UnitreeGo2Adapter._safety_event_active(probe, None) is False


def test_safety_event_probe_reads_common_interfaces():
    class _Estop:
        def __init__(self, v):
            self.triggered = v

    probe = _Stub()
    probe.estop = _Estop(True)
    assert UnitreeGo2Adapter._safety_event_active(probe, None) is True
    probe.estop = _Estop(False)
    assert UnitreeGo2Adapter._safety_event_active(probe, None) is False
