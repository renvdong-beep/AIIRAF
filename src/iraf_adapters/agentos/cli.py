# -*- coding: utf-8 -*-
"""IRAF × AgentOS 交互式 CLI（北向适配层，2026-10-12）。

定位（与 HTTP / gRPC 服务端**同一层**，不是旁路）
--------------------------------------------------
```
自然语言 ─► AgentOSBridge.execute_intent()
              └─► IntentProvider（受约束解码到 skill schema）
              └─► TaskDispatcher ─► SkillRuntime.execute()
                     └─► PolicyGateway（rbac/schema/profile/safety/capability/precondition）
                     └─► Capability Provider（控制域）
```
本文件**不驱动后端、不写判据、不绕过策略**：它只做「取文本 → 建 canonical 请求 →
交给 Bridge」以及把结果**如实**渲染出来。失败一律显式（`IRAF-INTENT-PARSE-FAILED`、
`IRAF-POLICY-DENIED` …），绝不把"命令已发出"当成成功。

三种使用形态
------------
1. **交互 REPL**（人用）：`iraf-agentos` 或 `python3 scripts/agentos_cli.py`
2. **单次文本**（脚本用）：`iraf-agentos --text "让机器狗站起来"`
3. **机器模式**（AgentOS CLI 用）：`iraf-agentos --json`，stdin 收一条 JSON 请求，
   stdout 出一条 JSON 响应 —— 外层 AgentOS CLI 把它当**子进程/工具**绑定即可。

身份与凭据
----------
- 需要 Runtime 环境（`IRAF_PROFILE / IRAF_SAFETY_POLICY / IRAF_SKILL_ROOT /
  IRAF_BACKEND_ENTRYPOINT / IRAF_BACKEND_CONFIG / IRAF_EVENT_STORE`）；
  缺项即**显式失败**（不猜路径、不给默认值）。
- 意图模型来自 `IRAF_AGENTOS_MODEL_URL` + `IRAF_AGENTOS_MODEL`（可选
  `IRAF_AGENTOS_TOKEN` / `IRAF_AGENTOS_VERIFY_TLS` / `IRAF_AGENTOS_TIMEOUT_SECONDS`）。
- 身份 = 本地 `AuthenticatedContext(subject, {"task.submit","task.read"}, "local")`，
  由 `--subject` 指定（缺省取 `IRAF_RUNTIME_SUBJECT`，再缺省即显式失败）。

退出码
------
`0` 执行成功（evidence 满足 skill 输出 schema） ｜ `1` 执行失败或策略拒绝（显式 code）
`2` 用法/环境错误（缺 Runtime 环境、缺意图模型、参数非法） ｜ `3` 意图解析失败
`4` 连接类故障（意图 Provider 不可达）
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import uuid

from iraf_adapters.agentos.bridge import AgentOSBridge
from iraf_adapters.agentos.dispatcher import TaskDispatcher
from iraf_core.policy import AuthenticatedContext

#: Execution 侧必需的 Runtime 环境（与 `bootstrap.build_runtime_from_env` 同源）。
RUNTIME_ENV = (
    "IRAF_PROFILE",
    "IRAF_SAFETY_POLICY",
    "IRAF_SKILL_ROOT",
    "IRAF_BACKEND_ENTRYPOINT",
    "IRAF_BACKEND_CONFIG",
    "IRAF_EVENT_STORE",
)

#: 意图模型侧的环境（两键齐全才启用自然语言解析）。
INTENT_ENV = ("IRAF_AGENTOS_MODEL_URL", "IRAF_AGENTOS_MODEL")

#: 本地 CLI 默认授予的 scope（提交 + 读；**不含** `skill.execute.direct`）。
DEFAULT_SCOPES = ("task.submit", "task.read")

#: 退出码（与文档一致）。
EXIT_OK = 0
EXIT_EXECUTION_FAILED = 1
EXIT_USAGE = 2
EXIT_INTENT_PARSE_FAILED = 3
EXIT_INTENT_UNAVAILABLE = 4

#: 视为"执行成功"的终态。
SUCCESS_STATUSES = ("SUCCEEDED", "COMPLETED")


def _now_ms():
    return int(dt.datetime.now(dt.timezone.utc).timestamp() * 1000)


def build_request(text, *, subject="", deadline_seconds=120, resource_id=None,
                  controller="agentos-cli", request_id=None):
    """把一段自然语言文本构建成 canonical IRAF 请求（**纯函数**，便于单测）。

    为什么三个 id 必须齐（`request_id` / `correlation_id` / `idempotency_key`）：
    PolicyGateway 第一道规则就要求三者非空，否则 `IRAF-INPUT-INVALID`；
    `idempotency_key` 还是 Runtime 幂等去重的键（同键同摘要 ⇒ 返回既有结果）。
    这里用一次性 uuid 生成，保证 CLI 每次输入都是**新任务**（重放请显式给
    `request_id`/`idempotency_key`）。
    """
    if not isinstance(text, str) or not text.strip():
        raise ValueError("意图文本不能为空")
    suffix = uuid.uuid4().hex
    return {
        "request_id": request_id or ("cli-" + suffix),
        "correlation_id": "cli-" + suffix,
        "idempotency_key": request_id or ("cli-" + suffix),
        "text": text.strip(),
        "deadline_unix_ms": _now_ms() + int(deadline_seconds) * 1000,
        "resource_id": resource_id,
        "controller": controller,
    }


def build_context(subject, scopes=DEFAULT_SCOPES, transport="local"):
    """构造**本地**受信上下文（纯函数）。`transport` 只允许 bearer|local（Policy 校验）。"""
    if not subject:
        raise ValueError("缺少调用方身份 subject（--subject 或 IRAF_RUNTIME_SUBJECT）")
    return AuthenticatedContext(str(subject), frozenset(scopes), str(transport))


def resolve_subject(explicit=None):
    """身份来源优先级：显式参数 > IRAF_RUNTIME_SUBJECT > 显式失败（不猜）。"""
    subject = (explicit or os.environ.get("IRAF_RUNTIME_SUBJECT") or "").strip()
    if not subject:
        raise RuntimeError("缺少调用方身份：请给 --subject 或设置 IRAF_RUNTIME_SUBJECT")
    return subject


def missing_env(names, environ=None):
    """返回 names 中缺失（未设置或为空）的键（纯函数，便于单测）。"""
    env = os.environ if environ is None else environ
    return [name for name in names if not str(env.get(name) or "").strip()]


def build_intent_provider(environ=None):
    """按环境构造意图 Provider；缺键即返回 None（由调用方显式失败，不给假 Provider）。

    为什么不做"缺模型就直通"的降级：那会把自然语言入口变成**零校验后门**。
    """
    env = os.environ if environ is None else environ
    endpoint = str(env.get("IRAF_AGENTOS_MODEL_URL") or "").strip()
    model = str(env.get("IRAF_AGENTOS_MODEL") or "").strip()
    if not endpoint or not model:
        return None
    from iraf_adapters.model.openai_intent import (
        IntentProviderConfig,
        OpenAICompatibleIntentProvider,
    )

    verify = str(env.get("IRAF_AGENTOS_VERIFY_TLS", "true")).lower() not in {"0", "false", "no"}
    try:
        timeout = float(env.get("IRAF_AGENTOS_TIMEOUT_SECONDS", "10"))
    except ValueError:
        timeout = 10.0
    return OpenAICompatibleIntentProvider(
        IntentProviderConfig(endpoint, model, str(env.get("IRAF_AGENTOS_TOKEN") or ""),
                             timeout, verify)
    )


def classify_result(result):
    """把 Bridge 返回值归成 (exit_code, kind)；**只读结果，不解释成成功**（纯函数）。"""
    if not isinstance(result, dict):
        return EXIT_EXECUTION_FAILED, "invalid"
    code = str(result.get("error_code") or "")
    status = str(result.get("status") or "")
    if status in SUCCESS_STATUSES and not code:
        return EXIT_OK, "succeeded"
    if code == "IRAF-INTENT-PARSE-FAILED":
        return EXIT_INTENT_PARSE_FAILED, "intent_parse_failed"
    if code.startswith("IRAF-UNAUTHENTICATED") or code.startswith("IRAF-POLICY-DENIED"):
        return EXIT_EXECUTION_FAILED, "denied"
    return EXIT_EXECUTION_FAILED, "failed"


def render_result(result, *, as_json=False):
    """把结果渲染成给用户看的文本（或原样 JSON）。**不美化、不隐瞒失败原因**。"""
    if as_json:
        return json.dumps(result, ensure_ascii=False, indent=2)
    if not isinstance(result, dict):
        return "[CLI] 结果不是对象：%r" % (result,)
    lines = []
    status = result.get("status")
    code = result.get("error_code")
    lines.append("状态      : %s%s" % (status or "(无)", ("  错误码: " + code) if code else ""))
    if result.get("execution_id"):
        lines.append("execution : %s" % result["execution_id"])
    skill = result.get("resolved_skill") or result.get("skill")
    if skill:
        lines.append("解析技能  : %s" % skill)
    provider = result.get("intent_provider") or {}
    if provider.get("model"):
        lines.append("意图模型  : %s (%s)" % (provider.get("model"), provider.get("name", "")))
    reason = result.get("reason") or result.get("message")
    if reason:
        lines.append("原因      : %s" % reason)
    evidence = result.get("evidence")
    if isinstance(evidence, dict) and evidence:
        keys = ", ".join(sorted(evidence.keys()))
        lines.append("证据字段  : %s" % keys)
    return "\n".join(lines)


def _describe_contracts(dispatcher):
    """列出当前可用 skill（名字 + 一句意图示例），供 REPL 的 `:skills` 用。"""
    rows = []
    for name, contract in sorted(dispatcher.registry.intent_contracts().items()):
        examples = contract.get("examples") or []
        head = str(examples[0]) if examples else ""
        rows.append("  %-22s %s" % (name, head))
    return "\n".join(rows) if rows else "  （SkillRegistry 里没有任何 Skill）"


REPL_BANNER = """IRAF × AgentOS 交互式 CLI（输入自然语言，或 :help 看命令）
提示：一切经 TaskFlow → SkillRuntime → PolicyGateway → Capability Provider；不会旁路驱动。"""


def _repl(bridge, dispatcher, context, *, as_json=False):
    print(REPL_BANNER, flush=True)
    while True:
        try:
            raw = input("iraf> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return EXIT_OK
        if not raw:
            continue
        if raw in (":q", ":quit", ":exit"):
            return EXIT_OK
        if raw == ":help":
            print("  :skills   列出当前可用 Skill\n  :whoami   显示身份与 scope\n"
                  "  :q        退出\n  其它输入 = 一句自然语言意图")
            continue
        if raw == ":skills":
            print(_describe_contracts(dispatcher))
            continue
        if raw == ":whoami":
            print("  身份=%s scope=%s  transport=%s" % (
                context.subject, sorted(context.roles), context.transport))
            continue
        request = build_request(raw)
        result = bridge.execute_intent(request, context)
        print(render_result(result, as_json=as_json), flush=True)


def _machine_mode(bridge, context):
    """机器模式：stdin 逐行收 JSON 请求 → stdout 逐行出 JSON 响应（供 AgentOS CLI 绑定）。

    约定（最小、可组合）：
      入： `{"text": "...", "deadline_seconds": 120, "resource_id": "...", "request_id": "..."}`
      出： 与 Bridge 返回值**同构**的 JSON（含 status / error_code / execution_id / evidence）。
    解析失败/执行失败**不改变进程约定**：仍然输出一行 JSON；退出码仅在进程结束时反映最后一次。
    """
    exit_code = EXIT_OK
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            print(json.dumps({"error_code": "IRAF-INPUT-INVALID",
                              "message": "非 JSON 输入：%s" % exc}, ensure_ascii=False),
                  flush=True)
            exit_code = EXIT_USAGE
            continue
        try:
            request = build_request(
                str(payload.get("text") or ""),
                deadline_seconds=int(payload.get("deadline_seconds", 120)),
                resource_id=payload.get("resource_id"),
                request_id=payload.get("request_id"),
            )
        except (TypeError, ValueError) as exc:
            print(json.dumps({"error_code": "IRAF-INPUT-INVALID", "message": str(exc)},
                             ensure_ascii=False), flush=True)
            exit_code = EXIT_USAGE
            continue
        result = bridge.execute_intent(request, context)
        code, _kind = classify_result(result)
        if code != EXIT_OK:
            exit_code = code
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return exit_code


def build_parser():
    parser = argparse.ArgumentParser(
        prog="iraf-agentos",
        description="IRAF × AgentOS 交互式 CLI（自然语言 → Skill，经完整策略链）",
    )
    parser.add_argument("--text", help="单次执行的一句自然语言意图（不给则进 REPL）")
    parser.add_argument("--json", action="store_true",
                        help="机器模式：stdin 逐行 JSON 请求 → stdout 逐行 JSON 响应")
    parser.add_argument("--json-pretty", action="store_true",
                        help="REPL/单次模式下以 JSON 打印结果（仍非机器模式）")
    parser.add_argument("--subject", help="调用方身份（缺省取 IRAF_RUNTIME_SUBJECT）")
    parser.add_argument("--deadline-seconds", type=int, default=120,
                        help="任务截止时间（相对现在，秒；缺省 120）")
    parser.add_argument("--resource-id", help="资源 id（可选；缺省取 Runtime 的 resource_id）")
    parser.add_argument("--list-skills", action="store_true", help="只列出可用 Skill 后退出")
    parser.add_argument("--version", action="version", version="iraf-agentos 0.2.0")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    # 1) Runtime 环境（缺项显式失败，不猜路径）
    missing = missing_env(RUNTIME_ENV)
    if missing:
        print("[CLI][用法错误] 缺少 Runtime 环境：%s" % ", ".join(missing), file=sys.stderr)
        return EXIT_USAGE

    from iraf_adapters.bootstrap import build_runtime_from_env

    runtime = build_runtime_from_env()
    dispatcher = TaskDispatcher(runtime)

    # 2) 身份
    try:
        subject = resolve_subject(args.subject)
        context = build_context(subject)
    except (RuntimeError, ValueError) as exc:
        print("[CLI][用法错误] %s" % exc, file=sys.stderr)
        return EXIT_USAGE

    if args.list_skills:
        print(_describe_contracts(dispatcher))
        return EXIT_OK

    # 3) 意图 Provider（缺模型显式失败 —— 不做"无校验直通"降级）
    provider = build_intent_provider()
    if provider is None:
        print("[CLI][用法错误] 缺少意图模型：请设置 %s（两键齐全）"
              % " 与 ".join(INTENT_ENV), file=sys.stderr)
        return EXIT_USAGE
    bridge = AgentOSBridge(provider, dispatcher)

    # 4) 分派形态
    if args.json:
        return _machine_mode(bridge, context)
    if args.text:
        try:
            request = build_request(args.text, deadline_seconds=args.deadline_seconds,
                                    resource_id=args.resource_id)
        except ValueError as exc:
            print("[CLI][用法错误] %s" % exc, file=sys.stderr)
            return EXIT_USAGE
        result = bridge.execute_intent(request, context)
        print(render_result(result, as_json=args.json_pretty))
        code, _kind = classify_result(result)
        return code
    return _repl(bridge, dispatcher, context, as_json=args.json_pretty)


if __name__ == "__main__":
    raise SystemExit(main())
