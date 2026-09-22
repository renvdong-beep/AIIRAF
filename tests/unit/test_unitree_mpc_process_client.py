"""单测：`process_client`（进程外壳）—— 超时/崩溃/自相矛盾响应都必须落 `damped_hold` 且无解。

用**假工作进程**（内联 `python3 -c ...`）代替真实求解器 ⇒ 无需 osqp/scipy 也能端到端测进程失败语义。
"""

import json
import sys
import time

import pytest

from iraf_adapters.unitree.mpc import protocol as pr
from iraf_adapters.unitree.mpc.process_client import MpcProcessClient

PY = sys.executable


def _req():
    return pr.build_request(h_diag=[2.0, 2.0], g=[-1.0, -1.0], a_rows=[0.0], a_cols=[0.0],
                            a_vals=[1.0], lbx=[-1.0, -1.0], ubx=[1.0, 1.0],
                            lba=[0.0], uba=[2.0])


GOOD_WORKER = [
    PY, "-c",
    "import sys,json;"
    "json.loads(sys.stdin.readline());"
    "print(json.dumps({'version':'iraf.mpc.v1','decision':'ok','reason':'解可用',"
    "'z':[0.5,0.5],'status_class':'ok','iter':60,'solve_ms':1.9972,'age_ms':0.0,'flags':{}}),"
    "flush=True)",
]
CRASH_WORKER = [PY, "-c", "import sys; sys.exit(3)"]
HANG_WORKER = [PY, "-c", "import time; time.sleep(30)"]
CONTRADICTORY_WORKER = [
    PY, "-c",
    "import sys,json;"
    "json.loads(sys.stdin.readline());"
    "print(json.dumps({'version':'iraf.mpc.v1','decision':'ok','reason':'假的',"
    "'z':None,'status_class':'ok','iter':1,'solve_ms':1.0,'age_ms':0.0,'flags':{}}),flush=True)",
]


def test_ok_path_returns_solution():
    with MpcProcessClient(GOOD_WORKER) as c:
        resp, err = c.call(_req())
    assert err is None and resp["decision"] == "ok"
    assert resp["z"] == [0.5, 0.5] and resp["iter"] == 60.0


def test_crash_yields_damped_hold_without_solution():
    client = MpcProcessClient(CRASH_WORKER)
    try:
        resp, err = client.call(_req())
    finally:
        client.close()
    assert resp["decision"] == "damped_hold" and resp["z"] is None
    assert err in ("子进程退出", "子进程不可写")
    assert client.stats["crashes"] >= 1


def test_timeout_yields_damped_hold_and_kills_child():
    client = MpcProcessClient(HANG_WORKER, timeout_ms=300.0)
    try:
        t0 = time.perf_counter()
        resp, err = client.call(_req())
        dt_ms = (time.perf_counter() - t0) * 1e3
    finally:
        client.close()
    assert resp["decision"] == "damped_hold" and resp["z"] is None
    assert err == "超时" and client.stats["timeouts"] == 1
    assert dt_ms < 2000.0, "超时后必须尽快返回（实测 %.1f ms）" % dt_ms


def test_contradictory_response_is_rejected():
    """对端自称 ok 却给 null 解 ⇒ 拒收并按失败处理（不得伪造成功）。"""
    client = MpcProcessClient(CONTRADICTORY_WORKER)
    try:
        resp, err = client.call(_req())
    finally:
        client.close()
    assert resp["decision"] == "damped_hold" and resp["z"] is None
    assert err == "响应非法" and client.stats["protocol_errors"] == 1


def test_invalid_request_is_rejected_before_sending():
    with MpcProcessClient(GOOD_WORKER) as c:
        resp, err = c.call({"version": "iraf.mpc.v1"})       # 缺字段
    assert resp["decision"] == "damped_hold" and resp["z"] is None
    assert err == "请求不满足协议" and c.stats["protocol_errors"] == 1


def test_no_stale_solution_is_ever_reused():
    """负向守卫：先成功后失败，失败那次的返回**绝不能**带上一次的解。"""
    client = MpcProcessClient(GOOD_WORKER)
    try:
        ok1, err1 = client.call(_req())
        assert err1 is None and ok1["z"] is not None
        client._cmd = HANG_WORKER                                 # 换成会超时的工作进程
        client.close()
        client._timeout_ms = 300.0
        bad, err2 = client.call(_req())
        assert bad["decision"] == "damped_hold" and bad["z"] is None
        assert err2 == "超时"
    finally:
        client.close()


def test_stats_track_failure_conditions():
    with MpcProcessClient(CRASH_WORKER) as c:
        c.call(_req())
        c.call(_req())
    assert c.stats["calls"] == 2 and c.stats["crashes"] >= 2 and c.stats["restarts"] >= 2
