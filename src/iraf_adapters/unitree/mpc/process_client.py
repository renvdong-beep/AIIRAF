"""MPC Provider 的进程外壳（第 4 块最后一件）。

职责：把"求解"放到**独立子进程**里，父侧只通过 `protocol.py` 约定的 JSON 行通信。
为什么独立进程：铁律 3/4（跨发行版 Provider 默认独立进程；CasADi/OSQP/Pinocchio 不得进框架核心）
与铁律 6.6（终态不得回写为成功）。

父侧的失败语义（本模块的重点，全部有单测）：
  · 子进程**超时** ⇒ 杀掉子进程 + 返回 `damped_hold`，**不返回任何解**；
  · 子进程**崩溃/提前退出/输出非法** ⇒ 返回 `damped_hold`，**不返回任何解**；
  · 对端返回**自相矛盾**消息（失败却带解 / ok 却无解）⇒ 本侧**拒收**并按失败处理；
  · 本客户端**不缓存任何旧解** ⇒ 结构上不可能"继续跑旧解"。
"""

from __future__ import annotations

import json
import select
import subprocess

from . import protocol as pr

__all__ = ["MpcProcessClient", "ClientError"]


class ClientError(RuntimeError):
    """协议或进程层面的错误（对外一律映射为显式失败）。"""


class MpcProcessClient:
    """常驻子进程客户端：一次 `call` = 写一行请求、读一行响应。

    参数
    ----
    cmd        : 子进程命令行（列表），例如 ["/usr/bin/python3", "-m", "iraf_adapters.unitree.mpc.worker"]
    timeout_ms : 单次调用的墙钟超时（默认取 MPC 预算 20.0 ms 的合理倍数 200.0 ms；
                 生产里应由 Provider 配置给出，**不得**散落在业务代码里）
    """

    def __init__(self, cmd, timeout_ms=200.0):
        if not cmd:
            raise ValueError("cmd 不能为空")
        self._cmd = list(cmd)
        self._timeout_ms = float(timeout_ms)
        self._proc = None
        self.stats = {"calls": 0, "timeouts": 0, "crashes": 0, "protocol_errors": 0, "restarts": 0}

    # ---- 生命周期 ----
    def start(self):
        if self._proc is not None and self._proc.poll() is None:
            return self
        self._proc = subprocess.Popen(self._cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, text=True, bufsize=1)
        return self

    def close(self, timeout_s=2.0):
        if self._proc is None:
            return
        try:
            if self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
                    self._proc.wait(timeout=timeout_s)
        finally:
            self._proc = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- 主调用 ----
    def call(self, request_msg, timeout_ms=None):
        """发送一条请求并取回响应；任何异常路径都返回 `damped_hold` 且 `z=None`。

        返回 `(response_dict, error_reason_or_None)`。
        """
        self.start()
        budget = self._timeout_ms if timeout_ms is None else float(timeout_ms)
        self.stats["calls"] += 1
        try:
            payload = pr.encode(pr.validate_request(request_msg))   # 先校验再发（缺字段不得上线）
        except (TypeError, ValueError) as exc:
            self.stats["protocol_errors"] += 1
            return self._fail("请求不满足协议：%s" % exc), "请求不满足协议"

        try:
            self._proc.stdin.write(payload + "\n")
            self._proc.stdin.flush()
        except (BrokenPipeError, ValueError, OSError) as exc:
            self.stats["crashes"] += 1
            self._restart()
            return self._fail("子进程不可写（可能已崩溃）：%s" % exc), "子进程不可写"

        ready, _, _ = select.select([self._proc.stdout], [], [], budget / 1e3)
        if not ready:
            self.stats["timeouts"] += 1
            self._restart(kill=True)
            return self._fail("子进程超时 %.1f ms ⇒ 显式失败（不得继续跑旧解）" % budget), "超时"

        line = self._proc.stdout.readline()
        if not line:                                    # EOF ⇒ 崩溃/提前退出
            self.stats["crashes"] += 1
            self._restart()
            return self._fail("子进程提前退出（EOF）⇒ 显式失败"), "子进程退出"
        try:
            resp = pr.decode(line, validate="response")
        except Exception as exc:  # noqa: BLE001
            self.stats["protocol_errors"] += 1
            self._restart()
            return self._fail("响应不满足协议或自相矛盾：%s" % exc), "响应非法"
        if resp["decision"] != "ok":
            return resp, "子进程报告失败"
        return resp, None

    # ---- 内部 ----
    def _fail(self, reason):
        return pr.build_response("damped_hold", reason, z=None, status_class="exception")

    def _restart(self, kill=False):
        if self._proc is not None:
            try:
                if kill and self._proc.poll() is None:
                    self._proc.kill()
            except Exception:  # noqa: BLE001
                pass
            try:
                self._proc.wait(timeout=1.0)
            except Exception:  # noqa: BLE001
                pass
        self._proc = None
        self.stats["restarts"] += 1
        self.start()


def _smoke():   # pragma: no cover - 手动冒烟用
    print(json.dumps({"note": "用 MpcProcessClient(cmd=...) 实例化；worker 入口见 worker.py"}))


if __name__ == "__main__":  # pragma: no cover
    _smoke()
