"""跨进程协议（第 4 块）：MPC Provider 与调用方之间**只传普通 JSON 类型**。

铁律依据：
  · 铁律 3/6.9：AgentOS/上层不得让私有类型（`casadi.DM`/scipy 稀疏对象/numpy 数组）穿透公共边界；
    跨发行版 Provider 默认独立进程，通过版本化接口通信 ⇒ 边界上只有
    **数字列表 / 字符串 / 布尔 / None**。
  · 铁律 6.6：终态不得回写为成功 ⇒ 响应里 `decision != "ok"` 时 **`z` 必须为 `null`**，
    且校验函数在遇到"decision 失败却带解"这类自相矛盾的消息时**拒收**（不给伪造成功留口子）。

本模块只做**编解码 + 校验**，不做求解；与 `provider_core` 组合即构成进程外壳两侧。
"""

from __future__ import annotations

import json
import math

__all__ = [
    "PROTOCOL_VERSION",
    "REQUEST_KEYS",
    "ResponseError",
    "build_request",
    "validate_request",
    "encode",
    "decode",
    "build_response",
    "validate_response",
]

PROTOCOL_VERSION = "iraf.mpc.v1"

REQUEST_KEYS = ("h_diag", "g", "a_rows", "a_cols", "a_vals", "lbx", "ubx", "lba", "uba")

_DECISIONS = ("ok", "damped_hold", "torque_zero_release")


class ResponseError(ValueError):
    """响应不合契约（跨边界拒收）。"""


def _as_list(x, name, where):
    """把入参规约为"数字列表"；**拒绝**非 JSON 可序列化类型（numpy 标量、DM 等）。"""
    if x is None:
        raise ValueError("%s：%s 不得为 None" % (where, name))
    if isinstance(x, (str, bytes)) or isinstance(x, (dict,)):
        raise ValueError("%s：%s 类型非法（%s）⇒ 边界上只允许数字列表"
                         % (where, name, type(x).__name__))
    try:
        seq = list(x)
    except TypeError:
        raise ValueError("%s：%s 不可迭代（%s）⇒ 边界上只允许数字列表"
                         % (where, name, type(x).__name__)) from None
    out = []
    for v in seq:
        try:
            f = float(v)
        except (TypeError, ValueError):
            raise ValueError("%s：%s 含非数值元素（%s）" % (where, name, type(v).__name__)) from None
        out.append(f)
    return out


def build_request(h_diag, g, a_rows, a_cols, a_vals, lbx, ubx, lba, uba,
                  version=PROTOCOL_VERSION, meta=None):
    """构造请求消息（全部规约为数字列表；`meta` 只允许 JSON 标量/字符串）。"""
    msg = {"version": version}
    for name, val in (("h_diag", h_diag), ("g", g), ("a_rows", a_rows), ("a_cols", a_cols),
                      ("a_vals", a_vals), ("lbx", lbx), ("ubx", ubx), ("lba", lba), ("uba", uba)):
        msg[name] = _as_list(val, name, "请求")
    if meta is not None:
        msg["meta"] = {str(k): (v if isinstance(v, (int, float, str, bool)) or v is None
                                else str(v)) for k, v in dict(meta).items()}
    return msg


def validate_request(msg, version=PROTOCOL_VERSION):
    """校验请求：版本、字段齐备、长度自洽、数值有限；返回规范化后的副本。"""
    if not isinstance(msg, dict):
        raise ValueError("请求必须是字典（收到 %s）" % type(msg).__name__)
    if msg.get("version") != version:
        raise ValueError("协议版本不匹配：收到 %r，期望 %r" % (msg.get("version"), version))
    missing = [k for k in REQUEST_KEYS if k not in msg]
    if missing:
        raise ValueError("请求缺少字段：%s" % missing)
    out = {"version": version}
    for k in REQUEST_KEYS:
        out[k] = _as_list(msg[k], k, "请求")
    n = len(out["h_diag"])
    for k in ("h_diag", "g", "lbx", "ubx"):
        if len(out[k]) != n:
            raise ValueError("长度不一致：%s 长度 %d ≠ 变量数 %d" % (k, len(out[k]), n))
    for k in ("a_rows", "a_cols", "a_vals"):
        if not (len(out[k]) == len(out["a_rows"])):
            raise ValueError("A 三元组长度不一致：%s=%d 与 a_rows=%d"
                             % (k, len(out[k]), len(out["a_rows"])))
    if len(out["lba"]) != len(out["uba"]):
        raise ValueError("lba/uba 长度不一致：%d vs %d" % (len(out["lba"]), len(out["uba"])))
    for k, v in out.items():
        if k == "version":
            continue
        if any((isinstance(x, float) and (math.isnan(x))) for x in v):
            raise ValueError("字段 %s 含 NaN ⇒ 拒收（NaN 会让求解行为未定义）" % k)
    return out


def build_response(decision, reason, z=None, status_class="unknown", iter_=None,
                   solve_ms=None, age_ms=None, flags=None, version=PROTOCOL_VERSION):
    """构造响应。**契约**：`decision != "ok"` 时 `z` 必须为 None（不得带解）。"""
    if decision not in _DECISIONS:
        raise ValueError("非法 decision=%r，允许 %s" % (decision, list(_DECISIONS)))
    if decision != "ok" and z is not None:
        raise ValueError("契约冲突：decision=%r 却带了解 ⇒ 拒绝构造（不得伪造成功）" % decision)
    return {
        "version": version, "decision": decision, "reason": str(reason),
        "z": None if z is None else [float(v) for v in z],
        "status_class": str(status_class),
        "iter": None if iter_ is None else float(iter_),
        "solve_ms": None if solve_ms is None else float(solve_ms),
        "age_ms": None if age_ms is None else float(age_ms),
        "flags": {str(k): bool(v) for k, v in dict(flags or {}).items()},
    }


def validate_response(msg, version=PROTOCOL_VERSION):
    """校验响应；与请求同等的严格度，并**拒收"失败却带解"**的消息。"""
    if not isinstance(msg, dict):
        raise ResponseError("响应必须是字典（收到 %s）" % type(msg).__name__)
    if msg.get("version") != version:
        raise ResponseError("协议版本不匹配：收到 %r" % (msg.get("version"),))
    d = msg.get("decision")
    if d not in _DECISIONS:
        raise ResponseError("非法 decision=%r" % (d,))
    z = msg.get("z")
    if d != "ok" and z is not None:
        raise ResponseError("响应自相矛盾：decision=%r 却带解 ⇒ 拒收（防伪造成功）" % d)
    if d == "ok" and z is None:
        raise ResponseError("响应自相矛盾：decision=ok 却无解")
    if not str(msg.get("reason", "")).strip():
        raise ResponseError("响应缺少可读 reason（排查必须有据）")
    return msg


def encode(msg):
    """序列化为 JSON 文本（若含 numpy/DM 等私有类型，这里就会失败 —— 这正是边界的意义）。"""
    return json.dumps(msg, ensure_ascii=False, allow_nan=False)


def decode(text, validate="request"):
    """反序列化并可选校验；`validate ∈ {"request", "response", None}`。"""
    msg = json.loads(text)
    if validate == "request":
        return validate_request(msg)
    if validate == "response":
        return validate_response(msg)
    if validate is None:
        return msg
    raise ValueError("validate 参数非法：%r" % (validate,))
