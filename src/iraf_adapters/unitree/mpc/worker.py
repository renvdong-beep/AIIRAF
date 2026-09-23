"""Provider 的独立进程入口（第 4 块最后一件，薄层）。

职责：`stdin` 读一行请求 JSON → `provider_core.step()` → `stdout` 写一行响应 JSON。
所有重活都在被导入的模块里（`provider_core` / `qp_scaling` / `osqp_native`），本文件只做**接线**。

求解器来源可注入（便于测试与替换实现）：
  环境变量 `IRAF_MPC_SOLVER` = `"模块:可调用对象"`，默认
  `iraf_adapters.unitree.mpc.osqp_native:solve_native`（**唯一**需要 osqp/scipy 的地方，
  且只在子进程里导入 ⇒ 框架主环境不受影响）。

失败语义：请求非法或求解异常 ⇒ 回一条 `damped_hold` 响应（`z` 为 `null`），
**不退出、不静默**；由父侧 `process_client` 统一处理超时/崩溃。
"""

from __future__ import annotations

import importlib
import contextlib
import json
import os
import sys

from . import protocol as pr
from .provider_core import ProviderCore

DEFAULT_SOLVER = "iraf_adapters.unitree.mpc.osqp_native:solve_native"


def load_solver(spec):
    mod_name, _, attr = str(spec).partition(":")
    if not mod_name or not attr:
        raise ValueError("求解器规格必须是 '模块:可调用对象'，收到 %r" % (spec,))
    mod = importlib.import_module(mod_name)
    fn = getattr(mod, attr)
    if not callable(fn):
        raise ValueError("求解器 %s 不可调用" % spec)
    return fn


def main(argv=None):  # pragma: no cover - 由子进程测试覆盖（tests/unit/test_unitree_mpc_worker.py）
    spec = os.environ.get("IRAF_MPC_SOLVER", DEFAULT_SOLVER)
    core = ProviderCore(load_solver(spec))
    # **协议通道卫生（实测踩过）**：osqp 的 C 库在构造/求解时把启动横幅与迭代信息打到 **stdout**，
    # 而本进程的 stdout 就是"一行一条 JSON 响应"的协议通道 ⇒ 父侧读到的第一行变成横幅、被
    # `validate_response` 判为"响应非法"（实测：`client_error=响应非法`、每次调用 ~1.3 s）。
    # 因此把**求解器调用期间**的 stdout 重定向到 stderr，协议行只由 `real_stdout` 显式写出。
    real_stdout = sys.stdout
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = pr.validate_request(json.loads(line))
        except Exception as exc:  # noqa: BLE001
            resp = pr.build_response("damped_hold", "请求非法：%s" % exc, status_class="exception")
            real_stdout.write(pr.encode(resp) + "\n")
            real_stdout.flush()
            continue
        try:
            with contextlib.redirect_stdout(sys.stderr):
                out = core.step(req)
            resp = pr.build_response(out["decision"], out["reason"], z=out["z"],
                                     status_class=out["status_class"], iter_=out["iter"],
                                     solve_ms=out["solve_ms"], age_ms=out["age_ms"],
                                     flags=out["flags"])
        except Exception as exc:  # noqa: BLE001
            resp = pr.build_response("damped_hold", "内核异常：%s" % exc, status_class="exception")
        try:
            line_out = pr.encode(resp)
        except Exception as exc:  # noqa: BLE001 —— 序列化失败也必须回一条合法失败响应，绝不崩
            line_out = pr.encode(pr.build_response(
                "damped_hold", "响应序列化失败：%s" % exc, status_class="exception"))
        real_stdout.write(line_out + "\n")
        real_stdout.flush()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
