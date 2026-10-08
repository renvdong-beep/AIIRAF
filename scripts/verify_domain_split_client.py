#!/usr/bin/env python3
"""智能域客户端：跨域四例验收（只走公共 gRPC 契约，无旁路）。

角色：本脚本运行在**智能域容器**里，代表 TaskFlow/业务方。它**不 import iraf_core**，
因此物理上无法构造 AuthenticatedContext、无法直连 Provider（铁律 1.2 / 1.8）——
身份只以 Bearer 凭据经 transport 传递，由控制域服务端构造。

四例（与 scripts/probe_domain_split_local.py 同口径，但这一层走**跨域 gRPC**）：
  ① normal     正常：stand 全链成功，且证据满足 skill 输出 schema 的语义要求
  ② denied     策略拒绝：Profile 摘要不匹配 ⇒ IRAF-POLICY-DENIED
  ③ cancel     执行中取消 ⇒ 终态 CANCELLED（并复核未回写成 SUCCEEDED）
  ④ unreachable 控制域不可达 ⇒ 显式失败 IRAF-EXECUTION-FAILED（不得伪造成功）

所需环境变量（由 scripts/verify_domain_split.sh 注入，不在本脚本里写默认值）：
  IRAF_SDK_TARGET / IRAF_SDK_TOKEN / IRAF_CASE
  IRAF_PROFILE_PATH / IRAF_SAFETY_PATH（声明文件路径；身份三元组由本脚本按 sha256 现算，
  与宿主 `iraf_core.profile` 的口径一致 —— 摘要不由外部注入，避免"两边各写一个数字"）
  IRAF_DURATION_MS（缺省 2000）
退出码：0 本用例符合预期；1 不符合预期；2 配置缺失。
"""

from __future__ import annotations

import hashlib
import importlib
import os
import pathlib
import sys
import threading
import time

import yaml

from iraf_sdk import (GrpcSkillClient, canonical_execute_body, execute_request_from_body,
                      result_from_feedback, status_from_state)

FAIL: list = []


def env(key: str) -> str:
    value = os.environ.get(key, "").strip()
    if not value:
        print("[智能域][错误] 缺少环境变量 %s（不接受隐式默认值）" % key)
        sys.exit(2)
    return value


def check(label: str, ok: bool, detail: str = "") -> None:
    print("    [%s] %s%s" % ("✓" if ok else "✗", label, ("  —— " + detail) if detail else ""))
    if not ok:
        FAIL.append(label)


def _identity_of_file(path: pathlib.Path) -> dict:
    """从声明文件算身份三元组：`digest = sha256(原始字节)`。

    与 `iraf_core.profile` 的口径**逐字一致**（`hashlib.sha256(raw).hexdigest()`），
    但本脚本不 import iraf_core（铁律 1.2：智能域侧物理上不得触碰核心类型），
    因此自己算一遍 —— 这也是"目标端持有签名声明副本并据此引用"的最小实现。
    """
    raw = path.read_bytes()
    data = yaml.safe_load(raw) or {}
    meta = data.get("metadata") or {}
    name, version = str(meta.get("name") or ""), str(meta.get("version") or "")
    if not name or not version:
        print("[智能域][错误] 声明文件缺少 metadata.name/version：%s" % path)
        sys.exit(2)
    return {"name": name, "version": version, "digest": hashlib.sha256(raw).hexdigest()}


def identities():
    return (_identity_of_file(pathlib.Path(env("IRAF_PROFILE_PATH"))),
            _identity_of_file(pathlib.Path(env("IRAF_SAFETY_PATH"))))


def client(timeout_s: int = 60) -> GrpcSkillClient:
    return GrpcSkillClient(env("IRAF_SDK_TARGET"), env("IRAF_SDK_TOKEN"), timeout_s=timeout_s)


def duration_ms() -> int:
    return int(os.environ.get("IRAF_DURATION_MS", "2000"))


def submit(client_obj, profile, safety, *, profile_override=None, deadline_s=30):
    return client_obj.submit(
        "stand",
        profile=profile_override or profile,
        safety_policy=safety,
        parameters={"duration_ms": duration_ms()},
        deadline_s=deadline_s,
    )


def case_normal(profile, safety) -> None:
    print("  ① normal：正常路径（跨域 gRPC，stand 全链）")
    with client() as handle:
        result = submit(handle, profile, safety)
        status = result.get("status")
        check("终态 SUCCEEDED", status == "SUCCEEDED",
              "status=%s error_code=%s reason=%s" % (status, result.get("error_code"), result.get("reason")))
        evidence = (result.get("result") or {}).get("evidence") or {}
        check("返回 evidence 段", bool(evidence), "keys=%s" % sorted((result.get("result") or {}).keys()))
        check("evidence.simulation 为真", evidence.get("simulation") is True, "simulation=%s" % evidence.get("simulation"))
        # ⚠ gRPC 的 Struct 把所有数值还原成 double ⇒ 这里必须按数值比较，不能断言 int
        try:
            cycles = float(evidence.get("control_cycles", 0))
        except (TypeError, ValueError):
            cycles = 0.0
        check("evidence.control_cycles > 0", cycles > 0, "control_cycles=%s（duration_ms=%d）" % (evidence.get("control_cycles"), duration_ms()))
        joints = (evidence.get("final_state") or {}).get("joint_positions_rad") or {}
        check("final_state.joint_positions_rad 非空", len(joints) > 0, "%d 个关节" % len(joints))


def case_denied(profile, safety) -> None:
    print("  ② denied：策略拒绝（Profile 摘要不匹配）")
    bad_profile = {"name": profile["name"], "version": profile["version"], "digest": "0" * 64}
    with client() as handle:
        result = submit(handle, profile, safety, profile_override=bad_profile)
        check("终态 FAILED", result.get("status") == "FAILED", "status=%s" % result.get("status"))
        check("错误码 IRAF-POLICY-DENIED", result.get("error_code") == "IRAF-POLICY-DENIED",
              "error_code=%s reason=%s" % (result.get("error_code"), result.get("reason")))
        check("未伪造成功证据", not (result.get("result") or {}).get("evidence"))


def case_unreachable(profile, safety) -> None:
    print("  ④ unreachable：控制域不可达（注入 fault=unreachable）")
    with client() as handle:
        result = submit(handle, profile, safety)
        check("终态非 SUCCEEDED", result.get("status") != "SUCCEEDED", "status=%s" % result.get("status"))
        check("错误码 IRAF-EXECUTION-FAILED", result.get("error_code") == "IRAF-EXECUTION-FAILED",
              "error_code=%s reason=%s" % (result.get("error_code"), result.get("reason")))
        check("未伪造成功证据", not (result.get("result") or {}).get("evidence"))


def case_cancel(profile, safety) -> None:
    print("  ③ cancel：执行中取消（先等 RUNNING＝租约已取得，再取消）")
    grpc = importlib.import_module("grpc")
    runtime_pb2_grpc = importlib.import_module("iraf.v1.runtime_pb2_grpc")
    skill_pb2 = importlib.import_module("iraf.v1.skill_pb2")
    target, token = env("IRAF_SDK_TARGET"), env("IRAF_SDK_TOKEN")
    metadata = (("authorization", "Bearer " + token),)
    channel = grpc.insecure_channel(target)
    stub = runtime_pb2_grpc.SkillRuntimeServiceStub(channel)
    body = canonical_execute_body("stand", profile=profile, safety_policy=safety,
                                  parameters={"duration_ms": duration_ms()}, deadline_s=30)
    request = execute_request_from_body(body)
    holder: dict = {}

    def consume() -> None:
        terminal = None
        for feedback in stub.Execute(request, timeout=90, metadata=metadata):
            holder.setdefault("first", feedback)
            terminal = feedback
        holder["terminal"] = terminal

    worker = threading.Thread(target=consume, daemon=True)
    worker.start()
    started = time.time()
    while "first" not in holder and time.time() - started < 15:
        time.sleep(0.02)
    if "first" not in holder:
        check("收到 accepted 反馈（含 execution_id）", False, "15 s 内没有首条反馈")
        return
    execution_id = holder["first"].execution_id
    check("收到 accepted 反馈（含 execution_id）", bool(execution_id), "execution_id=%s" % execution_id)
    reached_running = False
    while time.time() - started < 30:
        snapshot = stub.GetExecution(skill_pb2.GetExecutionRequest(execution_id=execution_id),
                                     timeout=10, metadata=metadata)
        if status_from_state(snapshot.state) == "RUNNING":
            reached_running = True
            break
        time.sleep(0.02)
    check("执行已进入 RUNNING（执行器已被占用）", reached_running,
          "耗时 %.3f s" % (time.time() - started))
    response = stub.Cancel(skill_pb2.CancelExecutionRequest(execution_id=execution_id, reason="跨域验收：取消执行"),
                           timeout=15, metadata=metadata)
    check("Cancel 被接受", bool(response.accepted), "accepted=%s state=%s" % (response.accepted, status_from_state(response.state)))
    worker.join(timeout=30)
    terminal = holder.get("terminal")
    if terminal is None:
        check("收到终态反馈", False, "取消后流未结束")
        return
    result = result_from_feedback(terminal)
    check("终态 CANCELLED", result.get("status") == "CANCELLED",
          "status=%s error_code=%s reason=%s" % (result.get("status"), result.get("error_code"), result.get("reason")))
    check("未把取消回写成 SUCCEEDED", result.get("status") != "SUCCEEDED")
    # 复核（只读）：终态在服务端也必须是 CANCELLED，不是只在流里说了一次
    snapshot = stub.GetExecution(skill_pb2.GetExecutionRequest(execution_id=execution_id), timeout=10, metadata=metadata)
    check("服务端 GetExecution 复核为 CANCELLED", status_from_state(snapshot.state) == "CANCELLED",
          "state=%s" % status_from_state(snapshot.state))
    channel.close()


def main() -> int:
    case = env("IRAF_CASE")
    profile, safety = identities()
    print("[智能域] case=%s target=%s skill=stand duration_ms=%d" % (case, env("IRAF_SDK_TARGET"), duration_ms()))
    handlers = {"normal": case_normal, "denied": case_denied, "cancel": case_cancel, "unreachable": case_unreachable}
    if case not in handlers:
        print("[智能域][错误] 未知用例 %r（允许：%s）" % (case, sorted(handlers)))
        return 2
    handlers[case](profile, safety)
    print("  → 用例 %s：%s（%d 项不符）" % (case, "符合预期" if not FAIL else "存在不符合项", len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
