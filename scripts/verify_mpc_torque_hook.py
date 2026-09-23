#!/usr/bin/env python3
"""A6a-④ 验收：契约 §4 的失败路径**逐条**（真子进程 + 真协议边界 + 真缩放）与正常拍。

判据来源：`docs/debug/2026-09-23-locomote-provider-integration-spec.md` §4/§6；
覆盖范围（诚实标注）：本脚本覆盖「门禁 → 进程边界 → 协议 → 缩放回代 → 解→B1 载荷」这一段，
**不含**「MuJoCo 状态 → QP 请求」那段（属 A6a-④ ②），故 `plan_fn` 给的是**合成请求**。

用例（每条都有可复跑的断言，失败即列出）：
  A 正常拍（真子进程，`status=ok`）：设计的足端力 → 载荷 = 手算 `Jᵀ·f`，权重按本拍接触表；
  B QP `max_iter`（契约 §4 第 1 条）：`damped_hold` + 原因里带状态分类 + **解被清空**；
  C 子进程超时（`hang` 3 s、调用超时 200 ms）：`damped_hold` + 原因带「超时」+ 客户端记 1 次超时；
  D 解龄过期（> 40.0 ms）：`damped_hold` + `flags.stale`，且**不再调子进程**；
  E 急停：`torque_zero_release` + 解作废 + 本拍**不调子进程**；
  F 不沿用旧解：失败后的下一拍（非更新拍）仍必须抛，绝不返回上一拍的力矩。

产出：`build/acceptance/mpc-a6a4-torque-hook/report.json`（+ 控制台逐条打印）；全绿 ⇒ 退出码 0。
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.unitree import balance as balance_module          # noqa: E402
from iraf_core.profile import load_robot_profile                     # noqa: E402
from iraf_adapters.unitree.mpc import protocol as pr                 # noqa: E402
from iraf_adapters.unitree.mpc import qp_scaling as qs               # noqa: E402
from iraf_adapters.unitree.mpc.contact import LEG_ORDER              # noqa: E402
from iraf_adapters.unitree.mpc.deployment import load_deployment     # noqa: E402
from iraf_adapters.unitree.mpc.freshness import STALE_LIMIT_MS       # noqa: E402
from iraf_adapters.unitree.mpc.process_client import MpcProcessClient  # noqa: E402
from iraf_adapters.unitree.mpc.provider_runtime import ProviderRuntime  # noqa: E402
from iraf_adapters.unitree.mpc.torque_hook import (MpcTorqueHook, MpcUnavailableError)  # noqa: E402

OUT = ROOT / "build/acceptance/mpc-a6a4-torque-hook"
LOCOMOTE_DECL = ROOT / "config/go2_locomote.yaml"
BASE_DECL = ROOT / "config/go2_loopback.yaml"
PROFILE_DECL = ROOT / "profiles/unitree_go2_mujoco.yaml"
SCRIPTS_DIR = Path(__file__).resolve().parent

#: MPC 视界（`mpc_model.horizon`，main() 里从声明读入后填充；不在这里写数字）。
HORIZON = None

#: 设计的足端力（LEG_ORDER = FL, FR, RL, RR，每腿 (fx, fy, fz)）；单位阵 Jᵀ ⇒ 力矩 = 力本身。
DESIGNED_FORCES = [10.0, 20.0, 30.0,
                   40.0, 50.0, 60.0,
                   70.0, 80.0, 90.0,
                   100.0, 110.0, 120.0]
MASK_FRONT_STANCE = {"FL": 1, "FR": 1, "RL": 0, "RR": 0}
INDEX_MAP = {"FL": (0, 1, 2), "FR": (3, 4, 5), "RL": (6, 7, 8), "RR": (9, 10, 11)}
IDENT_JT = {code: np.eye(3) for code in LEG_ORDER}
TOL = 1e-9


def _load_declarations():
    with open(LOCOMOTE_DECL) as handle:
        locomote = yaml.safe_load(handle)
    with open(BASE_DECL) as handle:
        base = yaml.safe_load(handle)
    # 控制频率走**机型 Profile 的正式加载器**（不是裸 yaml 取键：键在 `spec:` 下，
    # 且加载器会做「关节/限位/频率」的门禁；首版脚本按顶层键取 ⇒ KeyError）。
    profile = load_robot_profile(PROFILE_DECL)
    mpc_model = locomote["mpc_model"]
    deployment = load_deployment(locomote["mpc_provider"],
                                 profile.control_frequency_hz)
    balance = balance_module.load_balance_declaration(base)
    return mpc_model, deployment, balance


def _designed_solution(horizon, forces):
    """缩放**空间**的解向量：`w = d ⊙ z`（`d = √h_diag`）；回代后应精确得到设计的 z。"""
    n = horizon * 24
    h_diag = np.full(n, 2.0, dtype=float)
    z = np.zeros(n, dtype=float)
    base = horizon * 12
    z[base:base + 12] = forces
    d = qs.build_scale_diag(h_diag)
    return h_diag, (z * d)


def _request(h_diag):
    n = h_diag.size
    return pr.build_request(h_diag=h_diag.tolist(), g=[0.0] * n,
                            a_rows=[0], a_cols=[0], a_vals=[1.0],
                            lbx=[-1.0] * n, ubx=[1.0] * n, lba=[0.0], uba=[0.0])


def _env(status, solution, fail_after=0):
    env = dict(os.environ)
    paths = [str(ROOT / "src"), str(SCRIPTS_DIR)]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(paths)
    env["IRAF_MPC_SOLVER"] = "_mpc_probe_solver:solve_native"
    env["IRAF_FAKE_STATUS"] = status
    env["IRAF_PROBE_FAIL_AFTER"] = str(int(fail_after))
    env["IRAF_PROBE_X"] = json.dumps([float(v) for v in solution])
    env["IRAF_PROBE_HANG_S"] = "3.0"
    return env


class Harness:
    """一个用例 = 一个真子进程 + 一个 `ProviderRuntime` + 一个 `MpcTorqueHook`（时钟可注入）。

    `fail_after`：子进程内前 N 次求解返回 ok，之后按 `status` 行为（见 `_mpc_probe_solver.py`）。
    状态**只在子进程启动时**经环境变量注入（子进程环境不会被父进程后续修改影响）。
    """

    def __init__(self, deployment, balance, status="ok", forces=DESIGNED_FORCES,
                 ticks_per_update=None, fail_after=0):
        self.deployment = deployment
        self.horizon = int(HORIZON)
        self.clock = {"t": 1000.0}
        self.estop = {"latched": False}
        self.mask = dict(MASK_FRONT_STANCE)
        self.h_diag, self.solution = _designed_solution(self.horizon, forces)
        self.request = _request(self.h_diag)
        self.client = MpcProcessClient(
            [sys.executable, "-m", deployment.worker_module],
            timeout_ms=deployment.call_timeout_ms)
        self.runtime = ProviderRuntime(
            self.client,
            ticks_per_update=int(ticks_per_update or deployment.ticks_per_update),
            now_fn=lambda: self.clock["t"])
        self.weights = (float(balance["stance_weight_position"]),
                        float(balance["weight_position"]))
        self.hook = MpcTorqueHook(
            self.runtime, self._plan, INDEX_MAP, self.weights[0], self.weights[1],
            lambda: IDENT_JT, self.horizon, emergency_fn=lambda: self.estop["latched"],
            timeout_ms=deployment.call_timeout_ms)
        os.environ.update(_env(status, self.solution, fail_after))

    def _plan(self):
        return {"request": self.request, "current_mask": dict(self.mask)}

    def __enter__(self):
        self.client.start()
        return self

    def __exit__(self, *exc):
        self.client.close()
        return False

    def summary(self):
        return {"client": dict(self.client.stats), "runtime": dict(self.runtime.stats),
                "hook": self.hook.summary()}


def _check(checks, name, ok, detail=""):
    checks.append({"check": name, "ok": bool(ok), "detail": detail})


def case_a_ok(deployment, balance):
    checks, numbers = [], {}
    with Harness(deployment, balance) as h:
        payload = h.hook(0)
        torques = np.asarray(payload["balance_torque_nm"], dtype=float)
        weights = np.asarray(payload["position_weight"], dtype=float)
        expected = np.asarray(DESIGNED_FORCES, dtype=float)
        numbers["torques"] = [float(v) for v in torques]
        numbers["weights"] = [float(v) for v in weights]
        # 载荷是**执行器支撑力矩** = −Jᵀ·f（符号实测见 build/iraf-a6a4/jt_sign_probe.py：
        # τ=−Jᵀf 撑住机身 Δh=+0.001714 m、四腿法向合力 116.025 N；τ=+Jᵀf 把足端卸掉）。
        numbers["max_abs_diff_vs_hand_computed"] = float(np.max(np.abs(torques + expected)))
        _check(checks, "载荷 = −Jᵀ·f（逐位手算对照，单位 Jᵀ ⇒ 取负）",
               np.max(np.abs(torques + expected)) <= TOL,
               "max|Δ| = %.3e" % numbers["max_abs_diff_vs_hand_computed"])
        _check(checks, "权重 = 支撑腿 0.0 / 摆动腿 1.0（来自声明）",
               np.allclose(weights, [0.0] * 6 + [1.0] * 6),
               "stance=%.6f swing=%.6f" % h.weights)
        _check(checks, "真子进程被调用 1 次（协议边界成立）", h.client.stats["calls"] == 1,
               "calls=%d" % h.client.stats["calls"])
        _check(checks, "解可用时 has_solution=True", h.runtime.has_solution is True, "")
        # 50 Hz 求解 / 100 Hz 消费（契约 §3）：第 2 拍是**消费拍** ⇒ 必须复用同一份**新鲜**解，
        # 且不再调子进程。（"消费拍复用新鲜解"本身是正确行为；失败用例因此要显式设 ticks_per_update=1。）
        second = h.hook(1)
        _check(checks, "消费拍复用新鲜解且不再调子进程（更新率解耦）",
               np.allclose(np.asarray(second["balance_torque_nm"], dtype=float), -expected)
               and h.client.stats["calls"] == 1 and h.runtime.stats["skips"] == 1,
               "calls=%d skips=%d" % (h.client.stats["calls"], h.runtime.stats["skips"]))
        numbers["harness"] = h.summary()
    return checks, numbers


def case_b_qp_max_iter(deployment, balance):
    checks, numbers = [], {}
    # `ticks_per_update=1`：让**每一拍都是更新拍**，否则第 2 拍是消费拍（held）⇒ 会（正确地）
    # 复用那份新鲜解，失败路径根本不会被触发（首版即如此，被本用例抓出）。更新率语义由用例 A 覆盖。
    with Harness(deployment, balance, status="max_iter", fail_after=1,
                 ticks_per_update=1) as h:
        h.hook(0)                                          # 第 1 次求解 ok（先有可用解）
        try:
            h.hook(1)                                      # 第 2 次求解返回不可用状态（更新拍）
            _check(checks, "QP max_iter ⇒ 抛 MpcUnavailableError", False, "没有抛")
        except MpcUnavailableError as exc:
            numbers["reason"] = exc.reason
            numbers["decision"] = exc.decision
            numbers["status_class"] = exc.diagnostics.get("status_class")
            _check(checks, "QP max_iter ⇒ damped_hold", exc.decision == "damped_hold", exc.reason)
            _check(checks, "失败判词指出「本次更新失败」（不是含糊的「尚未求解」）",
                   "本次更新失败" in exc.reason and "max_iter" in exc.reason, exc.reason)
            _check(checks, "原因里带状态分类 max_iter（可追溯）",
                   "max_iter" in exc.reason or numbers["status_class"] == "max_iter",
                   "status_class=%r" % numbers["status_class"])
        _check(checks, "失败当拍解被清空（不沿用旧解）",
               h.runtime.has_solution is False and h.runtime.solution is None, "")
        numbers["harness"] = h.summary()
    return checks, numbers


def case_c_subprocess_timeout(deployment, balance):
    checks, numbers = [], {}
    with Harness(deployment, balance, status="hang", fail_after=1,
                 ticks_per_update=1) as h:                 # 同 B：每拍都是更新拍
        h.hook(0)                                          # 第 1 次求解 ok
        try:
            h.hook(1)
            _check(checks, "超时 ⇒ 抛 MpcUnavailableError", False, "没有抛")
        except MpcUnavailableError as exc:
            numbers["reason"] = exc.reason
            numbers["decision"] = exc.decision
            numbers["client_error"] = exc.diagnostics.get("client_error")
            _check(checks, "超时 ⇒ damped_hold", exc.decision == "damped_hold", exc.reason)
            _check(checks, "原因里带「超时」（失败分类可区分）", "超时" in exc.reason,
                   "client_error=%r" % numbers["client_error"])
        numbers["client_stats"] = dict(h.client.stats)
        _check(checks, "客户端记录 1 次超时并重启子进程", h.client.stats["timeouts"] == 1,
               json.dumps(numbers["client_stats"], ensure_ascii=False))
        _check(checks, "失败当拍解被清空", h.runtime.has_solution is False, "")
        numbers["harness"] = h.summary()
    return checks, numbers


def case_d_stale_solution(deployment, balance):
    checks, numbers = [], {}
    with Harness(deployment, balance, ticks_per_update=2) as h:
        h.hook(0)
        calls_before = h.client.stats["calls"]
        h.clock["t"] += STALE_LIMIT_MS + 0.1
        try:
            h.hook(1)
            _check(checks, "解龄过期 ⇒ 抛 MpcUnavailableError", False, "没有抛")
        except MpcUnavailableError as exc:
            numbers["reason"] = exc.reason
            numbers["flags"] = exc.diagnostics.get("age_ms")
            _check(checks, "解龄 > %.1f ms ⇒ damped_hold" % STALE_LIMIT_MS,
                   exc.decision == "damped_hold", exc.reason)
            _check(checks, "原因里带「过期」并给出解龄", "过期" in exc.reason,
                   "age_ms=%r" % exc.diagnostics.get("age_ms"))
        _check(checks, "过期拍不再调子进程（消费侧门禁生效）",
               h.client.stats["calls"] == calls_before,
               "calls %d → %d" % (calls_before, h.client.stats["calls"]))
        numbers["harness"] = h.summary()
    return checks, numbers


def case_e_emergency(deployment, balance):
    checks, numbers = [], {}
    with Harness(deployment, balance) as h:
        h.hook(0)
        calls_before = h.client.stats["calls"]
        h.estop["latched"] = True
        try:
            h.hook(1)
            _check(checks, "急停 ⇒ 抛 MpcUnavailableError", False, "没有抛")
        except MpcUnavailableError as exc:
            numbers["reason"] = exc.reason
            numbers["decision"] = exc.decision
            _check(checks, "急停 ⇒ torque_zero_release", exc.decision == "torque_zero_release",
                   exc.reason)
        _check(checks, "急停拍不调子进程（急停优先于拿新解）",
               h.client.stats["calls"] == calls_before,
               "calls %d → %d" % (calls_before, h.client.stats["calls"]))
        _check(checks, "急停后解作废", h.runtime.has_solution is False, "")
        numbers["harness"] = h.summary()
    return checks, numbers


def case_f_no_reuse_after_failure(deployment, balance):
    checks, numbers = [], {}
    with Harness(deployment, balance, status="infeasible", fail_after=1,
                 ticks_per_update=1) as h:                 # 同 B：每拍都是更新拍，才逼得出失败
        h.hook(0)
        for cycle in (1, 2):
            try:
                payload = h.hook(cycle)
                _check(checks, "第 %d 拍仍必须抛（绝不返回旧解）" % cycle, False,
                       "返回了载荷=%r" % (np.asarray(payload["balance_torque_nm"])[:3].tolist(),))
            except MpcUnavailableError as exc:
                _check(checks, "第 %d 拍仍必须抛（绝不返回旧解）" % cycle, True, exc.decision)
        numbers["harness"] = h.summary()
    return checks, numbers


CASES = (("A: 正常拍（真子进程）", case_a_ok),
         ("B: QP max_iter", case_b_qp_max_iter),
         ("C: 子进程超时", case_c_subprocess_timeout),
         ("D: 解龄过期", case_d_stale_solution),
         ("E: 急停", case_e_emergency),
         ("F: 不沿用旧解", case_f_no_reuse_after_failure))


def main():
    mpc_model, deployment, balance = _load_declarations()
    horizon = int(mpc_model["horizon"])
    report = {
        "scope": "契约 §4 失败路径 + 正常拍（真子进程/真协议/真缩放；plan 为合成请求）",
        "declarations": {
            "locomote": str(LOCOMOTE_DECL.relative_to(ROOT)),
            "base": str(BASE_DECL.relative_to(ROOT)),
            "profile": str(PROFILE_DECL.relative_to(ROOT)),
        },
        "deployment": deployment.as_dict(),
        "horizon": horizon,
        "stance_weight_position": float(balance["stance_weight_position"]),
        "weight_position": float(balance["weight_position"]),
        "stale_limit_ms": STALE_LIMIT_MS,
        "designed_forces": DESIGNED_FORCES,
        "cases": [],
    }
    failed = []
    global HORIZON
    HORIZON = horizon
    for title, fn in CASES:
        checks, numbers = fn(deployment, balance)
        ok = all(check["ok"] for check in checks)
        report["cases"].append({"case": title, "ok": ok, "checks": checks, "numbers": numbers})
        print("%-24s %s" % (title, "通过" if ok else "**失败**"))
        for check in checks:
            print("    [%s] %s%s" % ("ok" if check["ok"] else "FAIL", check["check"],
                                     ("；" + check["detail"]) if check["detail"] else ""))
            if not check["ok"]:
                failed.append("%s / %s" % (title, check["check"]))
    report["all_ok"] = not failed
    report["failed"] = failed
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print("\n断言 %d 项，失败 %d 项；报告：%s"
          % (sum(len(c["checks"]) for c in report["cases"]), len(failed),
             (OUT / "report.json").relative_to(ROOT)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
