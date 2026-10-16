#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AgentOS 语义 → IRAF skill → MuJoCo 显示（薄桥接入口）。

回答"通过 AgentOS 的语义『启动 piper 机械臂进行抓取仿真』去调用机器人框架的 skill 并做 MuJoCo 显示"这条需求。

链路（与 `docs/agentos-cli-usage.md` 同一层的**北向**语义入口，不旁路任何一层）：
```
自然语言 ─► IntentProvider（受约束解码到 Skill 契约：{skill, parameters}）
          ─► 本脚本校验 + 展示解析结果（PARSED_SKILL / PARSED_PARAMS / PARSED_TARGET）
          ─► 显示通道 scripts/view_mujoco.py：SkillRuntime → Policy/Authority → Provider(MuJoCo)
             （窗口先开、抓取期间实时镜像 ⇒ "边抓边看"）
```
为什么要这个薄桥：`iraf-agentos` 是**无显示**的执行入口，`view_mujoco.py` 是**程序化**的显示入口；
"语义触发 + 开窗"需要把两者接起来，否则"语义"与"显示"各走一条路。
桥接规则（显式、可审）：
  · 解析出的 skill 必须属于**可显示集合**（抓取族）；否则**显式失败**，不猜；
  · 显示通道统一用 `display_pick`（它与 visual_pick 走**同一条** Backend 闭环，只把时长上限放宽，
    证据见 `skills/display_pick/skill.yaml` 的自述）——若解析结果是 pick_object/visual_pick，这里会打印映射说明；
  · 解析出的 `target_id`（若有）透传给显示通道；不一致即失败。

边界（铁律）：本脚本**不驱动电机/关节**、不写判据、不绕过 Policy；仿真结论一律 `simulation=true`。
真实意图模型由环境变量给出：`IRAF_AGENTOS_MODEL_URL` + `IRAF_AGENTOS_MODEL`
（离线演示可用 `scripts/dev_stub_intent_model.py`，它会在报告里被标注为"桩，不是真 LLM"）。

用法：
    export PYTHONPATH=src
    export IRAF_PROFILE=profiles/piper_mujoco.yaml IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml \
           IRAF_SKILL_ROOT=skills IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend \
           IRAF_BACKEND_CONFIG="$(PYTHONPATH=src python3 scripts/emit_backend_config.py --scene build/models/piper-pick-scene.xml --compact)" \
           IRAF_EVENT_STORE=/tmp/iraf-agentos-view.db IRAF_RUNTIME_SUBJECT=agentos-demo
    export IRAF_AGENTOS_MODEL_URL=http://127.0.0.1:8077/v1 IRAF_AGENTOS_MODEL=stub-intent
    python3 scripts/agentos_view.py --text "启动piper机械臂进行抓取仿真"

退出码：0 成功 / 1 执行失败 / 2 用法或环境错误 / 3 意图解析失败 / 4 意图 Provider 不可达 / 5 显示通道失败
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.agentos import cli as agentos_cli  # noqa: E402
from iraf_adapters.bootstrap import build_runtime_from_env  # noqa: E402

#: 可走显示通道的 skill（抓取族）。display_pick 是显示专用档。
DISPLAYABLE = ("display_pick", "visual_pick", "pick_object")
#: 显示通道实际执行的 skill（与视觉抓取同一 Backend 闭环，仅时长上限放宽）
DISPLAY_SKILL = "display_pick"

EXIT_OK, EXIT_FAILED, EXIT_USAGE, EXIT_PARSE, EXIT_PROVIDER, EXIT_DISPLAY = 0, 1, 2, 3, 4, 5


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="AgentOS 语义 → IRAF skill → MuJoCo 显示（薄桥接）")
    parser.add_argument("--text", required=True, help="一句自然语言（AgentOS 语义）")
    parser.add_argument("--baseline", type=Path, default=Path("config/piper_simulation_baseline.yaml"))
    parser.add_argument("--seconds", type=float, default=0.0, help="抓取结束后窗口保持秒数（0=保持到关闭）")
    parser.add_argument("--pick-duration-ms", type=int, default=None, help="覆盖抓取时长（缩短过程时长）")
    parser.add_argument("--subject", default=None, help="调用方身份（缺省取 IRAF_RUNTIME_SUBJECT）")
    parser.add_argument("--dry-run", action="store_true", help="只做语义解析并打印，不开窗")
    args = parser.parse_args(argv)

    missing = agentos_cli.missing_env(agentos_cli.RUNTIME_ENV)
    if missing:
        print("[agentos-view][用法错误] 缺少 Runtime 环境：%s" % ", ".join(missing), file=sys.stderr)
        return EXIT_USAGE
    provider = agentos_cli.build_intent_provider()
    if provider is None:
        print("[agentos-view][用法错误] 缺少意图模型：请设置 %s（两键齐全；离线演示可用 "
              "scripts/dev_stub_intent_model.py）" % " 与 ".join(agentos_cli.INTENT_ENV), file=sys.stderr)
        return EXIT_USAGE
    try:
        subject = agentos_cli.resolve_subject(args.subject)
    except (RuntimeError, ValueError) as exc:
        print("[agentos-view][用法错误] %s" % exc, file=sys.stderr)
        return EXIT_USAGE

    # 语义解析：走**同一份**受约束解码实现（与 CLI/Bridge 同源，不另写一套）
    runtime = build_runtime_from_env()
    contracts = runtime.registry.intent_contracts()
    try:
        intent = provider.parse(args.text, contracts)
    except Exception as exc:  # IntentProviderError 及其它
        message = str(exc)
        code = EXIT_PARSE if "schema" in message or "不符合" in message else EXIT_PROVIDER
        print("[agentos-view][失败] 意图解析失败（%s）：%s" % (code, message), file=sys.stderr)
        print("INTENT_PARSE_FAILED", message, flush=True)
        return code

    skill = str(intent.get("skill") or "")
    params = intent.get("parameters") or {}
    target = params.get("target_id")
    print("PARSED_SKILL", skill, flush=True)
    print("PARSED_PARAMS", params, flush=True)
    print("PARSED_TARGET", target or "-", flush=True)
    print("INTENT_PROVIDER", "%s/%s" % (provider.name, provider.model_id), flush=True)
    print("CALLER_SUBJECT", subject, flush=True)

    if skill not in DISPLAYABLE:
        print("[agentos-view][失败] 解析出的 skill=%r 不在可显示集合 %s 内："
              "本入口只负责「语义 → 抓取显示」，其它 skill 请走 iraf-agentos（无显示执行）。"
              % (skill, list(DISPLAYABLE)), file=sys.stderr)
        return EXIT_PARSE
    if skill != DISPLAY_SKILL:
        print("SKILL_MAPPING 解析=%s → 显示通道执行=%s（同一 Backend 闭环；display_pick 仅放宽时长上限）"
              % (skill, DISPLAY_SKILL), flush=True)

    if args.dry_run:
        print("DRY_RUN 已解析，未开窗", flush=True)
        return EXIT_OK

    # 显示通道：复用 view_mujoco.py 的 live 编排（窗口先开、抓取期间实时镜像）
    import view_mujoco  # 同目录脚本

    viewer_argv = [
        "--baseline", str(args.baseline),
        "--live",
        "--skill", DISPLAY_SKILL,
        "--seconds", str(args.seconds),
    ]
    if target:
        viewer_argv += ["--target-id", str(target)]
    if args.pick_duration_ms:
        viewer_argv += ["--pick-duration-ms", str(args.pick_duration_ms)]
    print("DISPLAY_CHANNEL scripts/view_mujoco.py %s" % " ".join(viewer_argv), flush=True)
    code = view_mujoco.main(viewer_argv)
    return {0: EXIT_OK, 1: EXIT_FAILED}.get(code, EXIT_DISPLAY)


if __name__ == "__main__":
    raise SystemExit(main())
