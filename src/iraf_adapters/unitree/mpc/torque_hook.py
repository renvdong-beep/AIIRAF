"""MPC 解 → 执行器级 B1 载荷（A6a-④ 第二块）：把 `ProviderRuntime` 的裁定接到
`_run_control` 的 `torque_provider(cycle_index, info)` 口子上（**`_run_control` 一行不改**）。

契约：`docs/debug/2026-09-23-locomote-provider-integration-spec.md` §1/§2/§4、
`docs/debug/2026-09-23-qp-builder-port-plan.md` §1.2（带 file:line 的接入契约）。

数据流（每次回调 = 一个控制拍）：
  `plan_fn()` 组本拍 QP 请求（含**本拍接触表**，只在更新拍调）→ `runtime.step()`
    ├─ `ok`                 ⇒ 取解 z 的**第一拍**足端力（12 维）→ `Jᵀ·f` → B1 载荷
    │                          `{"balance_torque_nm", "position_weight"}`（权重按本拍接触表逐关节）
    ├─ `damped_hold`        ⇒ 抛 `MpcUnavailableError`（调用方走 `stop(mode="damped_hold")`）
    └─ `torque_zero_release`⇒ 抛 `MpcUnavailableError`（急停 ⇒ 松力 + 作废解 + `SAFETY_STOP`）

**绝不复用旧解**：解只由 `ProviderRuntime` 持有，失败当拍即被清空（`step` 的既有语义）；
本模块不缓存任何解，也不缓存任何力矩。

本模块是**纯逻辑**：不碰模型/锁/控制器，**不写任何数字**——阈值来自 `freshness`，
解向量布局来自 `qp_builder` 的常量，`position_weight` 取值与腿→关节映射由调用方按**声明**给出。
外部依赖（请求构造、雅可比、急停状态、时钟、子进程客户端）全部注入
⇒ 负向用例可在**无求解器**环境端到端跑。
"""

from __future__ import annotations

import numpy as np

from .contact import LEG_ORDER
from .freshness import DECISION_DAMPED_HOLD, DECISION_OK, DECISION_TORQUE_ZERO_RELEASE
from .qp_builder import INPUT_DIM, STATE_DIM
from .torque_provider import foot_forces_matrix, torque_provider_payload

__all__ = ["MpcUnavailableError", "solution_foot_forces", "position_weight_vector", "MpcTorqueHook"]


class MpcUnavailableError(RuntimeError):
    """本拍**无可用解**（契约 §4）⇒ 调用方按 `decision` 决定安全动作。

    `decision ∈ {"damped_hold", "torque_zero_release"}`；`reason` 为中文可读原因（可直接进报告）；
    `diagnostics` 原样带上 `ProviderRuntime` 的诊断（解龄/状态分类/求解耗时/来源/拍号），便于排查。
    """

    def __init__(self, decision, reason, diagnostics=None):
        if decision not in (DECISION_DAMPED_HOLD, DECISION_TORQUE_ZERO_RELEASE):
            raise ValueError("MpcUnavailableError 的 decision 非法：%r" % (decision,))
        super().__init__("[%s] %s" % (decision, reason))
        self.decision = str(decision)
        self.reason = str(reason)
        self.diagnostics = dict(diagnostics or {})


def _leg_mask(contact_mask):
    """把接触表的一列归一为长度 4 的 0/1 数组（序 = `LEG_ORDER`）。

    接受 `{腿码: 0/1}` 或长度 4 的序列；**恰好覆盖**四条腿、取值只允许 0/1（否则显式失败）。
    """
    if isinstance(contact_mask, dict):
        missing = [code for code in LEG_ORDER if code not in contact_mask]
        extra = [code for code in contact_mask if code not in LEG_ORDER]
        if missing or extra:
            raise ValueError("contact_mask 必须恰好覆盖 LEG_ORDER=%s（缺 %s／多 %s）"
                             % (list(LEG_ORDER), missing, extra))
        values = [contact_mask[code] for code in LEG_ORDER]
    else:
        values = list(np.asarray(contact_mask).reshape(-1))
    if len(values) != len(LEG_ORDER):
        raise ValueError("contact_mask 长度必须为 %d（LEG_ORDER），实际 %d"
                         % (len(LEG_ORDER), len(values)))
    out = []
    for value in values:
        if value not in (0, 1, True, False):
            raise ValueError("contact_mask 只能含 0/1，实际 %r" % (value,))
        out.append(int(value))
    return np.asarray(out, dtype=int)


def solution_foot_forces(solution, horizon, step=0):
    """取解向量里第 `step` 拍的足端力 → `(4, 3)`（行序 `LEG_ORDER`）。

    布局事实**只来自 `qp_builder`**：决策向量 = `[每拍状态 STATE_DIM]×horizon` ‖
    `[每拍输入 INPUT_DIM]×horizon`（`assemble_qp` 的列序），力段起点 `horizon·STATE_DIM`，
    段内按 `LEG_ORDER` 排腿（与 `box_bounds`/`friction_rows` 的腿序同一事实）。
    """
    horizon = int(horizon)
    if horizon < 1:
        raise ValueError("horizon 必须 ≥1，实际 %r" % (horizon,))
    step = int(step)
    if not 0 <= step < horizon:
        raise ValueError("step 必须落在 [0, %d)，实际 %d" % (horizon, step))
    z = np.asarray(solution, dtype=float).reshape(-1)
    expected = horizon * (STATE_DIM + INPUT_DIM)
    if z.size != expected:
        raise ValueError("解向量长度必须为 horizon×(STATE_DIM+INPUT_DIM)=%d，实际 %d"
                         % (expected, z.size))
    base = horizon * STATE_DIM + step * INPUT_DIM
    return foot_forces_matrix(z[base:base + INPUT_DIM])


def position_weight_vector(contact_mask, joint_index_map, stance_weight, swing_weight):
    """逐关节位置权重（B1）：支撑腿 3 关节取 `stance_weight`，摆动腿取 `swing_weight`。

    · `contact_mask`：本拍接触表列（`{腿码: 0/1}` 或长度 4 的序列，序同 `LEG_ORDER`）；
    · `joint_index_map`：`{腿码: (hip, thigh, calf)}` —— 三个关节在 `joint_order` 里的下标，
      必须恰好覆盖 `LEG_ORDER`、12 个下标互不重复且恰好覆盖 `[0, STATE_DIM)`（B1 是 12 关节契约）；
    · 权重取值**只来自声明**：本函数只做 [0, 1] 门禁，**不给默认值**。
    """
    if sorted(str(code) for code in joint_index_map) != sorted(LEG_ORDER):
        raise ValueError("joint_index_map 必须恰好覆盖 LEG_ORDER=%s，实际 %s"
                         % (list(LEG_ORDER), sorted(str(c) for c in joint_index_map)))
    indices = []
    for code in LEG_ORDER:
        triple = tuple(joint_index_map[code])
        if len(triple) != 3:
            raise ValueError("joint_index_map[%s] 必须是 (hip, thigh, calf) 三元组，实际 %d 项"
                             % (code, len(triple)))
        for index in triple:
            value = int(index)
            if not 0 <= value < STATE_DIM:
                raise ValueError("joint_index_map[%s] 的下标 %r 越界（须落在 [0, %d)）"
                                 % (code, index, STATE_DIM))
            indices.append(value)
    if sorted(indices) != list(range(STATE_DIM)):
        raise ValueError("joint_index_map 的 12 个下标必须互不重复且覆盖 [0, %d)，实际 %s"
                         % (STATE_DIM, sorted(indices)))

    stance = float(stance_weight)
    swing = float(swing_weight)
    for label, value in (("stance_weight", stance), ("swing_weight", swing)):
        if not np.isfinite(value) or value < 0.0 or value > 1.0:
            raise ValueError("位置权重 %s=%r 必须落在 [0, 1]（取值只来自声明，本模块不给默认值）"
                             % (label, value))

    mask = _leg_mask(contact_mask)
    out = np.empty(STATE_DIM, dtype=float)
    for leg, code in enumerate(LEG_ORDER):
        out[list(joint_index_map[code])] = stance if mask[leg] == 1 else swing
    return out


class MpcTorqueHook:
    """`torque_provider(cycle_index, info)` 的 MPC 实现：可用解 ⇒ B1 载荷；否则抛 `MpcUnavailableError`。

    参数（全部注入，本层不认识模型/子进程实现）
    ----
    runtime      : `ProviderRuntime`（必须提供 `step()` 与只读 `will_update()`）
    plan_fn      : `() -> {"request": <协议请求>, "current_mask": <本拍接触表列>}`；**只在更新拍调**
    joint_index_map / stance_weight / swing_weight : 见 `position_weight_vector`（来自声明）
    jacobian_transpose_provider : `() -> {腿码: (3,3)}`（由调用方从既有平衡/步态路径取）
    horizon      : MPC 视界（`mpc_model.horizon`）；用于在解向量里定位力段
    emergency_fn : `() -> bool`（急停/安全事件；默认恒 False）
    timeout_ms   : 单次子进程调用超时（来自部署声明；None ⇒ 用客户端默认）
    """

    PLAN_KEYS = ("request", "current_mask")

    def __init__(self, runtime, plan_fn, joint_index_map, stance_weight, swing_weight,
                 jacobian_transpose_provider, horizon, emergency_fn=None, timeout_ms=None):
        if not callable(plan_fn):
            raise TypeError("plan_fn 必须可调用（() -> {request, current_mask}）")
        if not callable(jacobian_transpose_provider):
            raise TypeError("jacobian_transpose_provider 必须可调用（() -> {腿码: (3,3)}）")
        if not callable(getattr(runtime, "step", None)):
            raise TypeError("runtime 必须提供 step(request, emergency=..., timeout_ms=...)")
        if not callable(getattr(runtime, "will_update", None)):
            raise TypeError("runtime 必须提供只读 will_update(emergency=...)（防止 100 Hz 白造 QP）")
        self._runtime = runtime
        self._plan_fn = plan_fn
        self._jt_provider = jacobian_transpose_provider
        self._joint_index_map = {code: tuple(int(i) for i in joint_index_map[code])
                                 for code in LEG_ORDER}
        self._horizon = int(horizon)
        if self._horizon < 1:
            raise ValueError("horizon 必须 ≥1，实际 %r" % (horizon,))
        self._emergency = emergency_fn or (lambda: False)
        self._timeout_ms = None if timeout_ms is None else float(timeout_ms)
        # 装配期就把权重域与腿→关节映射门禁跑一遍（不等到运行到一半才失败）
        position_weight_vector([1, 1, 1, 1], self._joint_index_map, stance_weight, swing_weight)
        self._stance_weight = float(stance_weight)
        self._swing_weight = float(swing_weight)
        self._mask = None          # 与**当前被消费的解**同一拍的接触表列
        self.stats = {"calls": 0, "plans": 0, "payloads": 0, "unavailable": 0,
                      "overran_cycles": 0, "inaccurate_cycles": 0,
                      "last_decision": None, "last_reason": None, "last_cycle": None}

    # ---- 只读 ----
    @property
    def mask(self):
        """最近一次成功更新所用的接触表列（`None` ⇒ 还没有可用解）。"""
        return None if self._mask is None else np.array(self._mask, dtype=int)

    @property
    def horizon(self):
        return self._horizon

    # ---- 钩子本体 ----
    def __call__(self, cycle_index, info=None):
        self.stats["calls"] += 1
        self.stats["last_cycle"] = int(cycle_index)
        emergency = bool(self._emergency())
        update = bool(self._runtime.will_update(emergency=emergency))

        request = None
        mask = self._mask
        if update:
            try:
                plan = self._plan_fn()
                if not isinstance(plan, dict):
                    raise ValueError("plan_fn 必须返回映射，实际 %s" % type(plan).__name__)
                missing = [key for key in self.PLAN_KEYS if key not in plan]
                if missing:
                    raise ValueError("plan_fn 返回的映射缺少键 %s（本拍无法构造 QP）" % missing)
                request = plan["request"]
                mask = _leg_mask(plan["current_mask"])
            except Exception as exc:      # noqa: BLE001 —— 安全优先：本拍计划不可用也走 damped_hold
                self.stats["unavailable"] += 1
                self.stats["last_decision"] = DECISION_DAMPED_HOLD
                self.stats["last_reason"] = "本拍计划不可用：%s" % exc
                raise MpcUnavailableError(
                    DECISION_DAMPED_HOLD,
                    "本拍计划不可用（%s）：%s" % (type(exc).__name__, exc)) from exc
            self.stats["plans"] += 1

        out = self._runtime.step(request, emergency=emergency, timeout_ms=self._timeout_ms)
        # 验收证据要的"移动中 QP 越界次数"（config/go2_locomote.yaml 的 must_record）：
        # 逐拍累计 freshness 的 overran / inaccurate 标志（判词与阈值仍只在 freshness 里）。
        flags = out.get("flags") or {}
        if flags.get("overran"):
            self.stats["overran_cycles"] = int(self.stats.get("overran_cycles", 0)) + 1
        if flags.get("inaccurate"):
            self.stats["inaccurate_cycles"] = int(self.stats.get("inaccurate_cycles", 0)) + 1

        # 自洽校验：本拍是否真的调了子进程，必须与"本拍是否构造了计划"一致。
        # 不一致 ⇒ tick 计数被别的调用方推进过 ⇒ 显式失败（绝不按"看起来对"的解继续跑）。
        source = out.get("diagnostics", {}).get("source")
        if (source == "subprocess") != update:
            raise MpcUnavailableError(DECISION_DAMPED_HOLD,
                                      "内部不一致：source=%r 与本拍是否构造请求（%r）不符"
                                      % (source, update), out.get("diagnostics"))

        if out.get("decision") != DECISION_OK or out.get("z") is None:
            self.stats["unavailable"] += 1
            self.stats["last_decision"] = out.get("decision")
            reason = str(out.get("reason", ""))
            # 失败**原因**必须进报告（契约 §4 的四类失败要能区分）：把客户端判定与子进程原因
            # 一并带上；否则报告里只剩 freshness 的判词，看不到"超时/崩溃/响应非法"是哪一类。
            details = out.get("diagnostics", {})
            extra = [str(details[key]) for key in ("client_error", "response_reason")
                     if details.get(key)]
            if extra:
                reason = "%s；%s" % (reason, "／".join(extra))
            self.stats["last_reason"] = reason
            raise MpcUnavailableError(out.get("decision"), reason, details)

        try:
            forces = solution_foot_forces(out["z"], self._horizon)
        except (TypeError, ValueError) as exc:
            # 响应带解但解不可用（长度/数值非法）⇒ 按契约 §4 "响应非法" 走显式失败（原因原样保留）
            self.stats["unavailable"] += 1
            self.stats["last_decision"] = DECISION_DAMPED_HOLD
            self.stats["last_reason"] = "解不可用：%s" % exc
            raise MpcUnavailableError(DECISION_DAMPED_HOLD, "解不可用（形状/数值非法）：%s" % exc,
                                      out.get("diagnostics")) from exc

        payload = torque_provider_payload(forces, self._jt_provider(),
                                         position_weight_vector(mask, self._joint_index_map,
                                                                self._stance_weight,
                                                                self._swing_weight))
        # 饱和**前**的原始载荷峰值（诊断用）：用于区分"QP 给的力本身过大"与"混合/截断把它放大"
        # （实测：执行端峰力矩恒为 45.430 N·m = 模型上限 ⇒ 一直饱和，必须看清源头）。
        self.stats["last_payload_max_nm"] = float(
            np.max(np.abs(np.asarray(payload["balance_torque_nm"], dtype=float)))
        )
        self.stats["last_forces_max_n"] = float(np.max(np.abs(forces)))
        self.stats["last_forces_sum_z_n"] = float(np.sum(np.asarray(forces, dtype=float)[:, 2]))
        # 最近一次求解的耗时与迭代数（诊断：用于看清"哪一拍开始变慢"——
        # 实测运行末段稳定在 ~0.88 s 处超时，需要区分"求解变慢"与"状态发散"）
        diagnostics = out.get("diagnostics", {})
        self.stats["last_solve_ms"] = diagnostics.get("solve_ms")
        self.stats["last_status_class"] = diagnostics.get("status_class")
        self._mask = mask
        self.stats["payloads"] += 1
        self.stats["last_decision"] = DECISION_OK
        self.stats["last_reason"] = out.get("reason")
        return payload

    def summary(self):
        """给报告用的摘要（含最后一次的裁定与原因，便于把失败写进验收记录）。"""
        return {
            "horizon": self._horizon,
            "stance_weight": self._stance_weight,
            "swing_weight": self._swing_weight,
            "stats": dict(self.stats),
            "mask": None if self._mask is None else [int(v) for v in self._mask],
            "runtime": dict(self._runtime.stats),
        }
