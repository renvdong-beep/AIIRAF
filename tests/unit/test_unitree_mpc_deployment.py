"""单测：`deployment`（A6a-2）—— 部署参数的读取、整除校验与**与新鲜度门禁的自洽核对**。

覆盖点（每条对着一份契约/铁律）：
  · 正常读取：字段齐、派生量正确（period_ms / ticks_per_update / stale_limit_ms / worker_command）；
  · 缺项 ⇒ 显式失败（不得静默取默认值，铁律 5.3）；
  · 非法值：空字符串、solver 格式、非正超时/更新率/周期数 ⇒ 显式失败；
  · **整除关系**：100 Hz 控制频率下 update_hz=50 通过；48（不整除）必须失败；
  · **自洽核对**：staleness_periods × period_ms 必须等于 `freshness.STALE_LIMIT_MS`，
    否则两处各写一个数字 ⇒ 失败（例如 update_hz=25 时 2×40=80 ms ≠ 40 ms，必须报错）；
  · 与真实声明文件联调：`config/go2_locomote.yaml` 的 `mpc_provider` 段必须能被本模块接受。
"""

from pathlib import Path

import pytest
import yaml

from iraf_adapters.unitree.mpc.deployment import load_deployment
from iraf_adapters.unitree.mpc.freshness import STALE_LIMIT_MS

REPO = Path(__file__).resolve().parents[2]


def _good():
    return {"worker_module": "iraf_adapters.unitree.mpc.worker",
            "solver": "iraf_adapters.unitree.mpc.osqp_native:solve_native",
            "call_timeout_ms": 200.0, "update_hz": 50.0, "staleness_periods": 2.0}


def test_loads_and_derives():
    dep = load_deployment(_good(), control_frequency_hz=100.0)
    assert dep.period_ms == pytest.approx(20.0)
    assert dep.ticks_per_update == 2
    assert dep.stale_limit_ms == pytest.approx(STALE_LIMIT_MS)
    assert dep.worker_command("/usr/bin/python3") == [
        "/usr/bin/python3", "-m", "iraf_adapters.unitree.mpc.worker"]
    assert dep.as_dict()["update_hz"] == 50.0


@pytest.mark.parametrize("missing", ["worker_module", "solver", "call_timeout_ms", "update_hz",
                                     "staleness_periods"])
def test_missing_key_fails_explicitly(missing):
    doc = _good()
    del doc[missing]
    with pytest.raises(ValueError) as ei:
        load_deployment(doc, control_frequency_hz=100.0)
    assert missing in str(ei.value) and "不得硬编码" in str(ei.value)


def test_empty_module_and_bad_solver_format_are_rejected():
    with pytest.raises(ValueError):
        load_deployment(dict(_good(), worker_module="   "), control_frequency_hz=100.0)
    with pytest.raises(ValueError):
        load_deployment(dict(_good(), solver="osqp_native"), control_frequency_hz=100.0)


@pytest.mark.parametrize("key,value", [("call_timeout_ms", 0.0), ("call_timeout_ms", -1.0),
                                       ("update_hz", 0.0), ("update_hz", -50.0),
                                       ("staleness_periods", 0.0)])
def test_non_positive_values_are_rejected(key, value):
    with pytest.raises(ValueError):
        load_deployment(dict(_good(), **{key: value}), control_frequency_hz=100.0)


def test_update_hz_must_divide_control_frequency():
    assert load_deployment(dict(_good(), update_hz=50.0), control_frequency_hz=100.0).update_hz == 50.0
    # update_hz=25 ⇒ 周期 40 ms；要保持门禁上限仍是 40 ms（单一事实来源）⇒ staleness_periods 必须为 1.0
    dep25 = load_deployment(dict(_good(), update_hz=25.0, staleness_periods=1.0),
                            control_frequency_hz=100.0)
    assert dep25.ticks_per_update == 4 and dep25.stale_limit_ms == pytest.approx(STALE_LIMIT_MS)
    with pytest.raises(ValueError) as ei:
        load_deployment(dict(_good(), update_hz=48.0), control_frequency_hz=100.0)
    assert "必须整除" in str(ei.value)


def test_staleness_must_be_consistent_with_freshness_single_source():
    """两处各写一个数字 ⇒ 必须报错（本专项的口径纪律：数字只有一个来源）。

    反例：update_hz=25（周期 40 ms）却仍写 staleness_periods=2.0 ⇒ 上限 80 ms ≠ 40 ms。
    """
    with pytest.raises(ValueError) as ei:
        load_deployment(dict(_good(), update_hz=25.0), control_frequency_hz=100.0)
    msg = str(ei.value)
    assert "不自洽" in msg and "STALE_LIMIT_MS" in msg


def test_real_declaration_is_accepted():
    """与真实声明联调：`config/go2_locomote.yaml` 的 `mpc_provider` 段必须通过校验。"""
    doc = yaml.safe_load((REPO / "config" / "go2_locomote.yaml").read_text(encoding="utf-8"))
    profile = yaml.safe_load((REPO / "profiles" / "unitree_go2_mujoco.yaml").read_text(encoding="utf-8"))
    dep = load_deployment(doc["mpc_provider"], profile["spec"]["control_frequency_hz"])
    assert dep.update_hz == 50.0 and dep.ticks_per_update == 2
    assert dep.stale_limit_ms == pytest.approx(STALE_LIMIT_MS)
