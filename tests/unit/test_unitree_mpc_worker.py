"""端到端：`process_client` → `protocol` → 子进程 `worker` → `provider_core` → **假求解器**。

用模块路径注入假求解器（`IRAF_MPC_SOLVER=_fake_mpc_solver:solve_native`）⇒ 全链路不需要 osqp/scipy。
这是第 4 块的"活体验证"：真实子进程、真实 JSON 边界、真实失败语义。

假求解器行为由环境变量 `IRAF_FAKE_STATUS` 驱动（ok / max_iter / raise 等）。
"""

import os
import sys

import pytest

from iraf_adapters.unitree.mpc import protocol as pr
from iraf_adapters.unitree.mpc.process_client import MpcProcessClient

PY = sys.executable
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "src"))


def _worker_cmd():
    return [PY, "-m", "iraf_adapters.unitree.mpc.worker"]


def _env(monkeypatch, status=None):
    monkeypatch.setenv("PYTHONPATH", os.pathsep.join([SRC_DIR, TESTS_DIR]))
    monkeypatch.setenv("IRAF_MPC_SOLVER", "_fake_mpc_solver:solve_native")
    if status is None:
        monkeypatch.delenv("IRAF_FAKE_STATUS", raising=False)
    else:
        monkeypatch.setenv("IRAF_FAKE_STATUS", status)


def _req():
    return pr.build_request(h_diag=[2.0, 2.0], g=[-1.0, -1.0], a_rows=[0.0], a_cols=[0.0],
                            a_vals=[1.0], lbx=[-1.0, -1.0], ubx=[1.0, 1.0],
                            lba=[0.0], uba=[2.0])


def test_end_to_end_ok_path(monkeypatch):
    _env(monkeypatch)
    with MpcProcessClient(_worker_cmd(), timeout_ms=5000.0) as c:
        resp, err = c.call(_req())
    assert err is None, resp
    assert resp["decision"] == "ok" and resp["z"] is not None
    assert resp["status_class"] == "ok"
    # 解龄是"求解完成到读回"的真实耗时 ⇒ 只能约束范围，不能断言等于 0
    assert resp["age_ms"] is not None and 0.0 <= resp["age_ms"] < 5.0


@pytest.mark.parametrize("status", ["max_iter", "infeasible", "unknown"])
def test_end_to_end_bad_status_is_explicit_failure(monkeypatch, status):
    _env(monkeypatch, status)
    with MpcProcessClient(_worker_cmd(), timeout_ms=5000.0) as c:
        resp, _ = c.call(_req())
    assert resp["decision"] == "damped_hold"
    assert resp["z"] is None, "跨进程边界上也不得带解（禁止伪造成功）"
    assert resp["status_class"] == status


def test_end_to_end_solver_exception_is_explicit_failure(monkeypatch):
    _env(monkeypatch, "raise")
    with MpcProcessClient(_worker_cmd(), timeout_ms=5000.0) as c:
        resp, _ = c.call(_req())
    assert resp["decision"] == "damped_hold" and resp["z"] is None
    assert "异常" in resp["reason"]


def test_end_to_end_rejects_bad_request_without_crashing(monkeypatch):
    _env(monkeypatch)
    bad = dict(_req())
    del bad["a_vals"]
    with MpcProcessClient(_worker_cmd(), timeout_ms=5000.0) as c:
        resp, err = c.call(bad)
        assert resp["decision"] == "damped_hold" and resp["z"] is None
        assert err == "请求不满足协议"
        # 子进程必须还活着：紧接着发一条合法请求仍应成功（不因一次坏请求而崩）
        good, err2 = c.call(_req())
        assert err2 is None and good["decision"] == "ok"
