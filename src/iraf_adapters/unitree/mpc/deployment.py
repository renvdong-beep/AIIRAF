"""MPC Provider 的**部署参数**读取与校验（A6a-2；铁律 5.3：不得在业务代码里硬编码）。

来源：`config/go2_locomote.yaml` 的 `mpc_provider` 段（版本化声明）。
本模块**只读配置**，不引入任何求解器依赖（osqp/scipy 只在 worker 子进程里）。

校验规则（缺项/非法值 ⇒ 显式失败，绝不静默取默认值）：
  · `worker_module` / `solver`：非空字符串，且 `solver` 形如 `"模块:可调用对象"`
  · `call_timeout_ms`：正数
  · `update_hz`：正数，且**必须整除** `control_frequency_hz`（否则控制拍与 MPC 拍无法对齐）
  · `staleness_periods`：正数（默认 2.0），用于把"解龄上限 = k × (1000/update_hz)"与
    `freshness.STALE_LIMIT_MS` 做**自洽核对**（不一致即失败，避免两处各写一个数字）
"""

from __future__ import annotations

from .freshness import MPC_PERIOD_MS, STALE_LIMIT_MS

__all__ = ["ProviderDeployment", "load_deployment", "REQUIRED_KEYS"]

REQUIRED_KEYS = ("worker_module", "solver", "call_timeout_ms", "update_hz", "staleness_periods")


class ProviderDeployment:
    """已校验的部署参数（不可变语义：构造后不再改）。"""

    def __init__(self, worker_module, solver, call_timeout_ms, update_hz, staleness_periods,
                 control_frequency_hz):
        self.worker_module = str(worker_module)
        self.solver = str(solver)
        self.call_timeout_ms = float(call_timeout_ms)
        self.update_hz = float(update_hz)
        self.staleness_periods = float(staleness_periods)
        self.control_frequency_hz = float(control_frequency_hz)

    @property
    def period_ms(self):
        return 1000.0 / self.update_hz

    @property
    def ticks_per_update(self):
        return int(round(self.control_frequency_hz / self.update_hz))

    @property
    def stale_limit_ms(self):
        return self.staleness_periods * self.period_ms

    def as_dict(self):
        return {
            "worker_module": self.worker_module,
            "solver": self.solver,
            "call_timeout_ms": self.call_timeout_ms,
            "update_hz": self.update_hz,
            "period_ms": self.period_ms,
            "ticks_per_update": self.ticks_per_update,
            "staleness_periods": self.staleness_periods,
            "stale_limit_ms": self.stale_limit_ms,
            "control_frequency_hz": self.control_frequency_hz,
        }

    def worker_command(self, python_executable):
        """独立进程的命令行（python -m worker_module）。"""
        return [str(python_executable), "-m", self.worker_module]


def _require(document, key, where):
    if not isinstance(document, dict) or key not in document:
        raise ValueError("%s 缺少 %s（部署参数必须集中声明，不得硬编码默认值）" % (where, key))
    return document[key]


def load_deployment(document, control_frequency_hz, where="config/go2_locomote.yaml mpc_provider"):
    """从**已解析的声明字典**读取并校验部署参数。

    `document` 为 `mpc_provider` 段本身；`control_frequency_hz` 取自机型 Profile
    （`profiles/unitree_go2_mujoco.yaml` 的 `control_frequency_hz`），用于整除关系校验。
    """
    raw = {k: _require(document, k, where) for k in REQUIRED_KEYS}

    for key in ("worker_module", "solver"):
        value = str(raw[key]).strip()
        if not value:
            raise ValueError("%s.%s 不得为空字符串" % (where, key))
    if ":" not in str(raw["solver"]):
        raise ValueError("%s.solver 必须形如 '模块:可调用对象'，实际 %r"
                         % (where, raw["solver"]))

    timeout = float(raw["call_timeout_ms"])
    if not timeout > 0:
        raise ValueError("%s.call_timeout_ms 必须为正数，实际 %r" % (where, raw["call_timeout_ms"]))

    update_hz = float(raw["update_hz"])
    if not update_hz > 0:
        raise ValueError("%s.update_hz 必须为正数，实际 %r" % (where, raw["update_hz"]))

    ctrl_hz = float(control_frequency_hz)
    if not ctrl_hz > 0:
        raise ValueError("control_frequency_hz 必须为正数，实际 %r" % (control_frequency_hz,))
    ratio = ctrl_hz / update_hz
    if abs(ratio - round(ratio)) > 1e-9:
        raise ValueError("%s.update_hz（%g）必须整除 control_frequency_hz（%g）：否则控制拍与 MPC 拍无法对齐"
                         % (where, update_hz, ctrl_hz))

    periods = float(raw["staleness_periods"])
    if not periods > 0:
        raise ValueError("%s.staleness_periods 必须为正数，实际 %r" % (where, raw["staleness_periods"]))

    dep = ProviderDeployment(raw["worker_module"], raw["solver"], timeout, update_hz, periods,
                             ctrl_hz)

    # 自洽核对：声明推算出的解龄上限必须与 freshness 的单一事实来源一致（避免两处各写一个数字）
    if abs(dep.stale_limit_ms - STALE_LIMIT_MS) > 1e-6:
        raise ValueError(
            "部署声明与新鲜度门禁不自洽：声明给出 %.6f ms（%g × %.6f），"
            "而 freshness.STALE_LIMIT_MS=%.6f ms（= %g × %.1f）⇒ 两处数字必须一致"
            % (dep.stale_limit_ms, periods, dep.period_ms, STALE_LIMIT_MS, 2.0, MPC_PERIOD_MS))
    return dep
