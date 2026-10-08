#!/usr/bin/env python3
"""探针：双域链路在**本机进程内**的四例预检（不涉容器、不涉 gRPC）。

为什么需要：容器里跑 gRPC 一圈要几十秒，而"策略是否放行 / skill 报告是否满足输出 schema /
取消是否真的调了安全停机"这几件事在**进程内**就能判。先把这一层跑绿，容器只负责证明
"这条链能被版本化接口跨域调用"，而不是同时承担调 bug 的职责（AGENTS.md 5.2）。

四例（与容器验收同口径）：
  ① 正常：stand 全链通过，返回满足 `skills/stand/stand.output.json` 的证据
  ② 策略拒绝：Profile 摘要不匹配 ⇒ `IRAF-POLICY-DENIED`
  ③ 取消：执行中取消 ⇒ 终态 CANCELLED，且**确实调了 backend.stop**（安全停机证据）
  ④ Provider 不可达：注入 fault=unreachable ⇒ 显式失败（不得伪造成功）

用法（仓库根）：
  PYTHONPATH=src python3 scripts/probe_domain_split_local.py
退出码：0 四例全部符合预期；1 有不符合项。
"""

from __future__ import annotations

import pathlib
import sys
import tempfile
import threading
import time
import uuid

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from iraf_adapters.domain_stub import ControlDomainStubBackend  # noqa: E402
from iraf_core.authority import ControlAuthorityManager  # noqa: E402
from iraf_core.policy import AuthenticatedContext  # noqa: E402
from iraf_core.profile import load_robot_profile, load_safety_policy  # noqa: E402
from iraf_core.registry import SkillRegistry  # noqa: E402
from iraf_core.runtime import SkillRuntime  # noqa: E402
from iraf_core.store import SqliteExecutionStore  # noqa: E402

PROFILE_PATH = "profiles/unitree_go2_mujoco.yaml"
SAFETY_PATH = "profiles/safety/quadruped_lab.yaml"

FAIL: list = []


def check(label: str, ok: bool, detail: str = "") -> None:
    print("    [%s] %s%s" % ("✓" if ok else "✗", label, ("  —— " + detail) if detail else ""))
    if not ok:
        FAIL.append(label)


def build_runtime(fault: str, trace_path: pathlib.Path, latency_ms: int = 0, store_path=None):
    """按声明装配 Runtime：profile / safety / skill 根 / 桩 backend / sqlite 事件库。"""
    profile = load_robot_profile(str(REPO / PROFILE_PATH))
    safety = load_safety_policy(str(REPO / SAFETY_PATH))
    registry = SkillRegistry().load_directory(str(REPO / "skills"))
    authority = ControlAuthorityManager()
    backend = ControlDomainStubBackend.from_config(
        {"fault": fault, "latency_ms": latency_ms, "trace_file": str(trace_path)}, profile, authority)
    store = SqliteExecutionStore(str(store_path or (pathlib.Path(tempfile.mkdtemp()) / "events.sqlite")))
    return SkillRuntime(profile, safety, backend, registry, authority, store), profile, safety, backend


def make_request(skill: str, profile, safety, parameters=None, digest: "str | None" = None, deadline_s: int = 30):
    now_ms = int(time.time() * 1000)
    return {
        "request_id": str(uuid.uuid4()),
        "idempotency_key": str(uuid.uuid4()),
        "correlation_id": str(uuid.uuid4()),
        "skill": skill,
        "skill_version_constraint": "",
        "parameters": dict(parameters or {}),
        "deadline_unix_ms": now_ms + deadline_s * 1000,
        "profile_name": profile.name,
        "profile_version": profile.version,
        "profile_digest": digest if digest is not None else profile.digest,
        "safety_policy_name": safety.name,
        "safety_policy_version": safety.version,
        "safety_policy_digest": safety.digest,
        "resource_id": "go2_sim",
        "controller": "agentos",
    }


def case_ok(runtime, profile, safety, trace_path) -> None:
    print("  ① 正常路径（stand 全链：TaskFlow → SkillRuntime → Policy → Provider → 桩）")
    request = make_request("stand", profile, safety, {"duration_ms": 2000})
    result = runtime.execute(request, ctx())
    status = result.get("status")
    check("终态 SUCCEEDED", status == "SUCCEEDED", "status=%s error_code=%s reason=%s" % (status, result.get("error_code"), result.get("reason")))
    output = result.get("result") or {}
    check("返回 evidence 段", isinstance(output.get("evidence"), dict), "keys=%s" % sorted(output.keys()))
    evidence = output.get("evidence") or {}
    check("evidence.simulation 为真", evidence.get("simulation") is True, "simulation=%s" % evidence.get("simulation"))
    check("evidence.control_cycles 为正整数", isinstance(evidence.get("control_cycles"), int) and evidence.get("control_cycles", 0) > 0,
          "control_cycles=%s（duration_ms=2000 / 100 Hz）" % evidence.get("control_cycles"))
    check("evidence.final_state.joint_positions_rad 非空", bool((evidence.get("final_state") or {}).get("joint_positions_rad")),
          "12 关节来自 Profile home")
    provider = result.get("provider") or {}
    check("provider 已记录", bool(provider.get("name")), "provider=%s" % provider.get("name"))


def case_denied(runtime, profile, safety) -> None:
    print("  ② 策略拒绝（Profile 摘要不匹配 ⇒ IRAF-POLICY-DENIED）")
    request = make_request("stand", profile, safety, {"duration_ms": 2000}, digest="0" * 64)
    result = runtime.execute(request, ctx())
    check("终态 FAILED", result.get("status") == "FAILED", "status=%s" % result.get("status"))
    check("错误码为 IRAF-POLICY-DENIED", result.get("error_code") == "IRAF-POLICY-DENIED",
          "error_code=%s reason=%s" % (result.get("error_code"), result.get("reason")))


def case_cancel(profile, safety, trace_path) -> None:
    print("  ③ 取消（执行中取消 ⇒ 终态 CANCELLED + 确实调用安全停机）")
    runtime, _, _, backend = build_runtime("none", trace_path, latency_ms=4000)
    request = make_request("stand", profile, safety, {"duration_ms": 2000})
    execution_id = runtime.execution_id_for("agentos", request["idempotency_key"])
    holder = {}

    def run():
        holder["result"] = runtime.execute(request, ctx(), execution_id=execution_id)

    worker = threading.Thread(target=run, daemon=True)
    started = time.time()
    worker.start()
    # ⚠ 必须等 RUNNING（= 租约已取得、执行器已被占用）再取消：
    # 在 VALIDATING 阶段取消时**还没有任何执行器被占用**，此时"没调 stop"是正确语义而不是缺陷
    # （本探针第一版用固定 sleep(0.4) 就踩了这个坑：实测 0.56 s 才进 RUNNING，
    #   0.4 s 取消 ⇒ 判据误报"未安全停机"）。
    reached_running = False
    while time.time() - started < 15.0:
        snapshot = runtime.get(execution_id) or {}
        if snapshot.get("status") == "RUNNING":
            reached_running = True
            break
        time.sleep(0.02)
    check("执行已进入 RUNNING（租约已取得，执行器已被占用）", reached_running,
          "耗时 %.3f s" % (time.time() - started))
    response = runtime.cancel(execution_id, ctx(), reason="跨域验收：取消执行")
    worker.join(timeout=15)
    result = holder.get("result") or {}
    check("cancel 被接受", response.get("accepted") is True, "accepted=%s reason=%s" % (response.get("accepted"), response.get("reason")))
    check("终态 CANCELLED", result.get("status") == "CANCELLED", "status=%s error_code=%s" % (result.get("status"), result.get("error_code")))
    check("确实执行了安全停机（backend.stops 非空）", len(backend.stops) >= 1,
          "safe_stop 记录 %d 条：%s" % (len(backend.stops), backend.stops[:1]))
    check("取消不是成功（不得把 CANCELLED 回写成 SUCCEEDED）", result.get("status") != "SUCCEEDED", "status=%s" % result.get("status"))


def case_unreachable(profile, safety, trace_path) -> None:
    print("  ④ Provider 不可达（fault=unreachable ⇒ 显式失败）")
    runtime, _, _, backend = build_runtime("unreachable", trace_path)
    request = make_request("stand", profile, safety, {"duration_ms": 2000})
    result = runtime.execute(request, ctx())
    check("终态非 SUCCEEDED", result.get("status") != "SUCCEEDED", "status=%s" % result.get("status"))
    check("错误码为 IRAF-EXECUTION-FAILED", result.get("error_code") == "IRAF-EXECUTION-FAILED",
          "error_code=%s reason=%s" % (result.get("error_code"), result.get("reason")))
    check("无 result.evidence（不伪造成功证据）", not (result.get("result") or {}).get("evidence"),
          "result=%s" % (result.get("result"),))


def ctx() -> AuthenticatedContext:
    return AuthenticatedContext("agentos", frozenset({"task.submit", "task.read", "task.cancel"}), "bearer")


def main() -> int:
    workdir = pathlib.Path(tempfile.mkdtemp(prefix="iraf-domain-probe-"))
    trace_path = workdir / "stub-trace.jsonl"
    runtime, profile, safety, _ = build_runtime("none", trace_path)
    print("profile=%s@%s digest=%s…" % (profile.name, profile.version, profile.digest[:16]))
    print("safety=%s@%s digest=%s…" % (safety.name, safety.version, safety.digest[:16]))
    case_ok(runtime, profile, safety, trace_path)
    case_denied(runtime, profile, safety)
    case_cancel(profile, safety, trace_path)
    case_unreachable(profile, safety, trace_path)
    print("\n审计账：%s（%d 行）" % (trace_path, len(trace_path.read_text(encoding="utf-8").splitlines()) if trace_path.exists() else 0))
    print("结论：%s（%d 项不符）" % ("四例全部符合预期" if not FAIL else "存在不符合项", len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
