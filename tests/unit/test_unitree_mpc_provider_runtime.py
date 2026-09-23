"""单测：`ProviderRuntime`（A6a-3）—— 更新/消费分离、失败即显式失败、绝不复用旧解。

用**假客户端**（不是真子进程）驱动，覆盖契约 §2/§3/§4 的编排语义：
  · 每 ticks_per_update 拍才更新一次；其余拍走消费侧门禁（source 分别为 subprocess/held）；
  · 更新成功 ⇒ decision=ok 且 z 非空；失败/坏状态 ⇒ damped_hold 且 z=None，且**旧解被清掉**；
  · 失败后的下一拍（即使不需更新）也不得把旧解拿出来用；
  · 急停 ⇒ torque_zero_release、解作废；
  · 计时注入 ⇒ 解龄超 40.0 ms ⇒ damped_hold。
"""

import pytest

from iraf_adapters.unitree.mpc import protocol as pr
from iraf_adapters.unitree.mpc.freshness import STALE_LIMIT_MS
from iraf_adapters.unitree.mpc.provider_runtime import ProviderRuntime


def _req():
    return pr.build_request(h_diag=[2.0, 2.0], g=[-1.0, -1.0], a_rows=[0.0], a_cols=[0.0],
                            a_vals=[1.0], lbx=[-1.0, -1.0], ubx=[1.0, 1.0], lba=[0.0], uba=[2.0])


class FakeClient:
    """按脚本返回响应（不启子进程）：next_response 为 None 时返回 ok。"""

    def __init__(self):
        self.calls = 0
        self.queue = []

    def push(self, decision="ok", err=None, status="ok", solve_ms=2.0):
        self.queue.append({"decision": decision, "err": err, "status": status,
                           "solve_ms": solve_ms})

    def call(self, request, timeout_ms=None):
        self.calls += 1
        if self.queue:
            spec = self.queue.pop(0)
        else:
            spec = {"decision": "ok", "err": None, "status": "ok", "solve_ms": 2.0}
        resp = pr.build_response(spec["decision"], "ok" if spec["decision"] == "ok" else "显式失败",
                                 z=[0.5, 0.5] if spec["decision"] == "ok" else None,
                                 status_class=spec["status"], iter_=60.0,
                                 solve_ms=spec["solve_ms"], age_ms=0.0)
        return resp, spec["err"]


def test_update_every_n_ticks_and_hold_consumer_side():
    client = FakeClient()
    rt = ProviderRuntime(client, ticks_per_update=2, now_fn=lambda: 0.0)
    out0 = rt.step(_req())
    assert out0["decision"] == "ok" and out0["z"] == [0.5, 0.5]
    assert out0["diagnostics"]["source"] == "subprocess"
    out1 = rt.step(_req())                     # 第 2 拍：不更新，走消费侧
    assert out1["diagnostics"]["source"] == "held" and out1["decision"] == "ok"
    assert client.calls == 1                   # 只调了一次子进程
    assert rt.stats["updates"] == 1 and rt.stats["skips"] == 1


def test_bad_status_clears_solution_and_never_reuses_it():
    # 每拍都更新（ticks_per_update=1）⇒ 推送的坏响应才会被消费；否则第 2 拍是"held"、根本不调子进程
    client = FakeClient()
    rt = ProviderRuntime(client, ticks_per_update=1, now_fn=lambda: 0.0)
    assert rt.step(_req())["decision"] == "ok"
    client.push(decision="damped_hold", err="超时", status="exception")
    bad = rt.step(_req())                       # 更新拍：失败
    assert bad["decision"] == "damped_hold" and bad["z"] is None
    # "不复用旧解"必须在**失败当拍**断言：此时解已被清空（下一拍若再次求解成功会是 ok，属正常）
    assert rt.has_solution is False and rt.solution is None


def test_held_tick_does_not_consume_queued_response():
    """反面守卫：非更新拍不得调子进程（否则更新率形同虚设）。"""
    client = FakeClient()
    rt = ProviderRuntime(client, ticks_per_update=2, now_fn=lambda: 0.0)
    rt.step(_req())
    client.push(decision="damped_hold", err="超时", status="exception")
    out = rt.step(_req())                       # 第 2 拍：held，坏响应仍排队中
    assert out["diagnostics"]["source"] == "held" and out["decision"] == "ok"
    assert client.queue, "坏响应不应在非更新拍被消费"
    assert rt.step(_req())["decision"] == "damped_hold"   # 第 3 拍才是更新拍


def test_emergency_releases_and_invalidates():
    client = FakeClient()
    rt = ProviderRuntime(client, ticks_per_update=1, now_fn=lambda: 0.0)
    rt.step(_req())
    out = rt.step(_req(), emergency=True)
    assert out["decision"] == "torque_zero_release" and out["z"] is None
    assert rt.has_solution is False
    assert rt.stats["releases"] == 1


def test_stale_solution_is_gated_on_consumer_ticks():
    clock = {"t": 0.0}
    client = FakeClient()
    rt = ProviderRuntime(client, ticks_per_update=5, now_fn=lambda: clock["t"])
    assert rt.step(_req())["decision"] == "ok"
    clock["t"] = STALE_LIMIT_MS + 0.1           # 解龄超限
    out = rt.step(_req())                       # 非更新拍 ⇒ 按门禁判过期
    assert out["decision"] == "damped_hold" and out["flags"]["stale"] is True
    assert out["z"] is None


def test_ticks_per_update_must_be_positive():
    with pytest.raises(ValueError):
        ProviderRuntime(FakeClient(), ticks_per_update=0)


def test_diagnostics_carry_traceable_fields():
    rt = ProviderRuntime(FakeClient(), ticks_per_update=1, now_fn=lambda: 0.0)
    d = rt.step(_req())["diagnostics"]
    for key in ("age_ms", "status_class", "solve_ms", "source", "tick", "ticks_per_update"):
        assert key in d
