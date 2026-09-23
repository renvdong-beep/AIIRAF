"""`mpc.contact` 接触表：腿序、半拍采样、对角签名、duty 计数与声明校验（复用 gait 判定）。"""

from __future__ import annotations

import numpy as np
import pytest

from iraf_adapters.unitree import gait as gait_mod
from iraf_adapters.unitree.mpc.contact import LEG_ORDER, contact_table, current_mask


def _params(kind="trot", duty=0.6, hz=3.0, offsets=None):
    offs = offsets or {"FL": 0.5, "FR": 0.0, "RL": 0.0, "RR": 0.5}
    legs = {code: {"phase_offset": offs[code]} for code in ("RR", "RL", "FR", "FL")}
    groups = {}
    for code, off in offs.items():
        groups.setdefault(off, []).append(code)
    return {
        "kind": kind,
        "frequency_hz": hz,
        "period_s": 1.0 / hz,
        "duty_factor": duty,
        "legs": legs,
        "phase_groups": [{"offset": o, "legs": sorted(v)} for o, v in sorted(groups.items())],
    }


def test_rows_follow_leg_order_and_ignore_dict_insertion_order():
    p = _params()
    dt = 1.0 / 300.0
    n = 4
    tab = contact_table(p, 0.0, dt, n)
    assert tab.shape == (4, n)
    # 逐格对照：第 i 行必须等于 LEG_ORDER[i] 这条腿用 gait 判定算出来的序列
    times = 0.0 + np.arange(n, dtype=float) * dt + dt / 2.0
    for i, code in enumerate(LEG_ORDER):
        expect = [1 if gait_mod.is_stance(p, gait_mod.leg_phase(p, code, t)) else 0 for t in times]
        assert tab[i].tolist() == expect
    # 注意：n 太小时两组可能全落在「四足重叠窗口」内而图案相同（见下一个用例），
    # 故"两组图案不同"这条断言放在整周期 64 采样的用例里。
    # 字典插入顺序不影响结果（腿序来自 LEG_ORDER，不来自迭代顺序）
    q = _params()
    q["legs"] = {code: q["legs"][code] for code in ("FL", "FR", "RL", "RR")}
    assert np.array_equal(contact_table(q, 0.0, dt, n), tab)


def test_trot_duty_06_has_four_leg_overlap_and_no_flight():
    """duty 0.6 > 0.5 ⇒ 存在**四足同时支撑**窗口，且**无腾空相**。

    分析式：两组支撑窗口各长 `duty`、错开半周期 ⇒ 交集两段、总长 `2·(duty − 0.5) = 0.2`。
    恒 2 足支撑只在 `duty ≤ 0.5` 时成立（本次先按 0.1 写，被本测试实测 0.1875 = 12/64 纠正）。
    """
    p = _params()
    tab = contact_table(p, 0.0, p["period_s"] / 64.0, 64)
    assert np.array_equal(tab[0], tab[3])   # FL == RR
    assert np.array_equal(tab[1], tab[2])   # FR == RL
    assert not np.array_equal(tab[0], tab[1])
    counts = tab.sum(axis=0)
    assert set(counts.tolist()) == {2, 4}          # 无腾空(0)、无单足(1)、无三足(3)
    assert (counts == 4).mean() == pytest.approx(2 * (0.6 - 0.5), abs=2.0 / 64.0)
    # duty 计数：每条腿支撑占比 = duty
    for row in tab:
        assert abs(row.mean() - 0.6) <= 1.0 / 64.0


def test_half_step_sampling_is_explicit():
    p = _params(hz=3.0)
    dt = 0.4 * p["period_s"]          # 半拍 = 0.2 周期 ⇒ 采样点跨过支撑/摆动边界
    a = contact_table(p, 0.0, dt, 1, half_step=True)
    b = contact_table(p, 0.0, dt, 1, half_step=False)
    c = contact_table(p, dt / 2.0, dt, 1, half_step=False)
    assert a[:, 0].tolist() == [0, 1, 1, 0]    # FL/RR 相位 0.5+0.2 = 0.7 ≥ 0.6 ⇒ 摆动
    assert b[:, 0].tolist() == [1, 1, 1, 1]    # FL/RR 相位 0.5 ⇒ 支撑
    assert np.array_equal(a, c)                # 半拍 = 起点后移 dt/2


def test_current_mask_matches_single_point_table():
    p = _params()
    for t in (0.0, 0.07, 0.1234, 0.29):
        assert current_mask(p, t).tolist() == contact_table(p, t, 0.0, 1, half_step=False)[:, 0].tolist()


def test_dtype_and_shape():
    tab = contact_table(_params(), 0.1, 0.01, 5)
    assert tab.dtype == np.int32 and tab.shape == (4, 5)


def test_rejects_non_trot_kind():
    with pytest.raises(gait_mod.DeclarationError):
        contact_table(_params(kind="wave"), 0.0, 0.01, 2)


def test_rejects_missing_leg():
    p = _params()
    del p["legs"]["RL"]
    with pytest.raises(gait_mod.DeclarationError):
        contact_table(p, 0.0, 0.01, 2)


def test_rejects_wrong_phase_structure():
    p = _params()
    p["phase_groups"] = [{"offset": o, "legs": [c]} for o, c in enumerate(LEG_ORDER)]
    with pytest.raises(gait_mod.DeclarationError):
        contact_table(p, 0.0, 0.01, 2)


def test_rejects_empty_horizon():
    with pytest.raises(ValueError):
        contact_table(_params(), 0.0, 0.01, 0)
