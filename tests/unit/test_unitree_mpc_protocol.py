"""单测：`protocol`（跨进程边界）—— 版本、类型纯净、契约冲突、伪造成功守卫。"""

import json

import numpy as np
import pytest

from iraf_adapters.unitree.mpc import protocol as pr


def _req_arrays():
    return dict(h_diag=np.array([2.0, 2.0]), g=np.array([-1.0, -1.0]),
                a_rows=np.array([0]), a_cols=np.array([0]), a_vals=np.array([1.0]),
                lbx=np.array([-1.0, -1.0]), ubx=np.array([1.0, 1.0]),
                lba=np.array([0.0]), uba=np.array([2.0]))


def test_request_roundtrip_is_json_pure():
    msg = pr.build_request(**_req_arrays())
    text = pr.encode(msg)                       # 含 numpy 私有类型时这里就会失败
    back = pr.decode(text, validate="request")
    assert back["version"] == pr.PROTOCOL_VERSION
    assert back["h_diag"] == [2.0, 2.0] and back["a_vals"] == [1.0]
    assert not any(isinstance(v, (np.ndarray, np.generic)) for v in back.values())


def test_request_rejects_numpy_object_dtype_and_dict():
    with pytest.raises(ValueError):
        pr.build_request(**dict(_req_arrays(), g={"x": 1}))                # dict 非法
    with pytest.raises(ValueError):
        pr.build_request(**dict(_req_arrays(), g=np.array(["a", "b"])))    # 非数值
    with pytest.raises(ValueError):
        pr.build_request(**dict(_req_arrays(), g=None))


def test_request_rejects_length_mismatch_and_nan():
    a = _req_arrays()
    a["lbx"] = np.array([-1.0])                 # 与变量数不符
    with pytest.raises(ValueError):
        pr.validate_request(pr.build_request(**a))
    b = _req_arrays()
    b["g"] = np.array([np.nan, 0.0])
    with pytest.raises(ValueError):
        pr.validate_request(pr.build_request(**b))


def test_request_rejects_wrong_version_and_missing_fields():
    msg = pr.build_request(**_req_arrays())
    msg["version"] = "iraf.mpc.v0"
    with pytest.raises(ValueError):
        pr.validate_request(msg)
    msg2 = pr.build_request(**_req_arrays())
    del msg2["a_vals"]
    with pytest.raises(ValueError):
        pr.validate_request(msg2)


def test_response_ok_carries_solution():
    r = pr.build_response("ok", "解可用", z=[0.5, 0.5], status_class="ok", iter_=60,
                          solve_ms=1.9972, age_ms=0.0, flags={"overran": False})
    text = pr.encode(r)
    back = pr.decode(text, validate="response")
    assert back["z"] == [0.5, 0.5] and back["iter"] == 60.0
    assert json.loads(text)["solve_ms"] == 1.9972


@pytest.mark.parametrize("decision", ["damped_hold", "torque_zero_release"])
def test_response_failure_must_not_carry_solution(decision):
    with pytest.raises(ValueError):
        pr.build_response(decision, "显式失败", z=[0.1, 0.2])       # 构造期就拒
    ok = pr.build_response(decision, "显式失败")
    assert ok["z"] is None


def test_validate_response_rejects_contradiction_from_other_side():
    """负向守卫：对端发来"失败却带解"的消息，本侧**拒收**（不给伪造成功留口子）。"""
    bad = {"version": pr.PROTOCOL_VERSION, "decision": "damped_hold", "reason": "过期",
           "z": [0.1, 0.2], "status_class": "ok", "iter": None, "solve_ms": None,
           "age_ms": None, "flags": {}}
    with pytest.raises(pr.ResponseError):
        pr.validate_response(bad)
    bad2 = dict(bad, decision="ok", z=None)
    with pytest.raises(pr.ResponseError):
        pr.validate_response(bad2)
    bad3 = dict(bad, decision="ok", z=[0.0, 0.0], reason="   ")
    with pytest.raises(pr.ResponseError):
        pr.validate_response(bad3)


def test_response_rejects_bad_decision_and_version():
    with pytest.raises(ValueError):
        pr.build_response("succeeded", "x")              # 非受控词表
    bad = {"version": "iraf.mpc.v0", "decision": "ok", "reason": "x", "z": [0.0]}
    with pytest.raises(pr.ResponseError):
        pr.validate_response(bad)
