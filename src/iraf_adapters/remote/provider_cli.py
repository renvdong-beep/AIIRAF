#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""远端 Capability Provider **服务端**入口（x86 仿真侧，2026-10-16）。

一句话：把本机的 MuJoCo 能力（含窗口显示）以 `CapabilityProviderService` 暴露给**板卡上的 IRAF**。

```
[板卡] AgentOS CLI → 板上 IRAF(TaskFlow/Skill/Policy) ──gRPC──► 本进程 ──► MujocoBackend
                                                                    └─► MuJoCo 窗口显示在本机桌面
```

**这是安装后的产品入口**（2026-10-26 入口下沉）：`python -m iraf_adapters.remote.provider_cli`。
为什么搬进包里：SDK 的 runtime 交付包**不含** `scripts/`（实测 `tar -tzf build/sdk/iraf-runtime-*.tar.gz`
只有 `src/` 与 `deploy/sdk/{units,env}`），所以服务单元的 ExecStart 只能指向包内模块，
否则"装完 SDK 就有 Provider"这条根本落不了地。`scripts/iraf_provider_serve.py` 保留为薄壳转发（dev 用法不变）。

与 `python3 -m iraf_adapters.grpc.server` 的区别：那是**任务级** runtime 服务端（自己跑策略链）；
本入口是**能力级** Provider（策略在调用方）。两者都属"版本化公共服务"，不旁路任何一层。

安全（fail-closed，缺一项即拒绝启动）
-------------------------------------
- **必须**有 token（`IRAF_PROVIDER_TOKEN`）：缺 token 直接退出，不监听端口；
- 默认只允许显式指定的监听地址；非 loopback 时强制要求 token（本入口一律要求 token）；
- 身份只来自受信 transport（gRPC metadata 的 Bearer token），不信任请求体自述。

用法（仓库根）::

    export PYTHONPATH=src:build/generated/python
    export IRAF_PROFILE=profiles/piper_mujoco.yaml
    export IRAF_SAFETY_POLICY=profiles/safety/simulation_lab.yaml
    export IRAF_SKILL_ROOT=skills
    export IRAF_BACKEND_ENTRYPOINT=iraf_adapters.mujoco.mujoco_backend:MujocoBackend
    export IRAF_BACKEND_CONFIG="$(python3 scripts/emit_backend_config.py --scene build/models/piper-pick-scene.xml --compact)"
    export IRAF_EVENT_STORE=/tmp/iraf-provider.db
    export IRAF_PROVIDER_TOKEN=***           # 只从环境来，不落盘、不进命令行
    DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw LIBGL_ALWAYS_SOFTWARE=1 \
      python3 scripts/iraf_provider_serve.py --host 0.0.0.0 --port 50052

退出码：0 正常退出 ｜ 1 参数/环境错误 ｜ 2 缺少 token 或桩 ｜ 3 装配或显示通道失败
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
from pathlib import Path

#: 本模块可同时跑在两种布局（同一份代码，不靠外部脚本）：
#:   仓库   <repo>/src/iraf_adapters/remote/provider_cli.py  ⇒ 桩在 <repo>/build/generated/python
#:   安装   <prefix>/<version>/src/iraf_adapters/remote/provider_cli.py ⇒ 桩在 <prefix>/<version>/lib
#: `IRAF_PROTO_STUBS_DIR` 显式给值时优先（部署期就是这么钉的）。
_HERE = Path(__file__).resolve()
_SRC_ROOT = _HERE.parents[2]                     # …/src
if _SRC_ROOT.is_dir() and str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))
_STUB_CANDIDATES = [
    Path(os.environ["IRAF_PROTO_STUBS_DIR"]) if os.environ.get("IRAF_PROTO_STUBS_DIR") else None,
    _HERE.parents[3] / "lib",                    # 安装布局：<version>/lib
    _HERE.parents[4] / "build" / "generated" / "python",  # 仓库布局
]
for _candidate in _STUB_CANDIDATES:
    if _candidate is not None and (_candidate / "iraf" / "v1").is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))
        break

EXIT_OK, EXIT_USAGE, EXIT_NOTOKEN, EXIT_ASSEMBLY = 0, 1, 2, 3


def _parse_window_px(raw):
    if not raw:
        return None
    try:
        width, height = str(raw).lower().replace("x", " ").split()
        return (int(width), int(height))
    except ValueError:
        return None


def self_check(args) -> int:
    """只验证装配，不监听端口（2026-10-19：板卡不在场时用来判断执行端是否健康）。

    与真实启动走**同一套**装配函数（build_runtime_from_env），所以它能拦住
    "profile/策略/backend 装配不上"这类问题；区别只是不建 gRPC server。
    退出码：0 全绿 / 2 有硬失败（缺 token / 缺 grpcio / 缺桩 / 装配失败）。
    """
    problems = []

    def report(ok, label, detail=""):
        print("  [%s] %s%s" % ("OK" if ok else "FAIL", label, (" —— " + detail) if detail else ""))
        if not ok:
            problems.append(label)

    print("[self-check] 解释器：%s（Python %s）" % (sys.executable, sys.version.split()[0]))
    subjects = str(os.environ.get("IRAF_RUNTIME_SUBJECT") or args.subject or "")
    print("  [OK] subject=%s" % subjects)
    print("  [OK] 显示模式声明 IRAF_PROVIDER_DISPLAY_MODE=%s（headless 时强制不开窗）"
          % (os.environ.get("IRAF_PROVIDER_DISPLAY_MODE") or "未声明"))

    token = os.environ.get("IRAF_PROVIDER_TOKEN") or ""
    report(bool(token), "IRAF_PROVIDER_TOKEN 已设置", "长度 %d" % len(token) if token else "缺（服务会 fail-closed）")

    try:
        import grpc  # noqa: F401
        report(True, "grpcio", getattr(grpc, "__version__", "?"))
    except Exception as exc:  # noqa: BLE001
        report(False, "grpcio", str(exc))

    try:
        from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2  # noqa: F401
        report(True, "proto 桩（iraf.v1）")
    except Exception as exc:  # noqa: BLE001
        report(False, "proto 桩（iraf.v1）", str(exc))

    runtime = None
    try:
        from iraf_adapters.bootstrap import build_runtime_from_env
        runtime = build_runtime_from_env()
        report(True, "运行时装配（profile / 安全策略 / skill root / backend）")
    except Exception as exc:  # noqa: BLE001
        report(False, "运行时装配", str(exc)[:220])

    if runtime is not None:
        try:
            declared = getattr(runtime.profile, "capabilities", None) or {}
            names = sorted(declared.keys()) if isinstance(declared, dict) else sorted(declared)
            report(bool(names), "profile 声明的能力", ",".join(names) or "空")
        except Exception as exc:  # noqa: BLE001
            report(False, "profile 声明的能力", str(exc)[:160])
        try:
            inventory = runtime.backend.runtime_inventory()
            report(isinstance(inventory, dict) and bool(inventory),
                   "backend 运行时清单（策略前置条件的取值来源）",
                   "顶层键=%s" % sorted(inventory.keys())[:8] if isinstance(inventory, dict) else str(type(inventory)))
        except Exception as exc:  # noqa: BLE001
            report(False, "backend 运行时清单", str(exc)[:200])

    if problems:
        print("[self-check] 结论：%d 项失败（%s）" % (len(problems), "；".join(problems)))
        return EXIT_NOTOKEN
    print("[self-check] 结论：全绿 —— 该执行端可对外提供能力（尚未监听端口）")
    return 0


def apply_display_env(args) -> bool:
    """按装配单里的 `IRAF_PROVIDER_DISPLAY_MODE` 决定是否强制无窗口（**纯函数，可单测**）。

    返回 True 表示"装配单要求 headless 并已置 args.no_window"。
    为什么要有它：安装后的服务单元只给装配单、不给命令行参数，显示模式必须能由声明决定；
    而"写了 window 但机器没显示设备"要如实降级，不能假装开了窗（铁律 1.5/1.7）。
    """
    mode_env = str(os.environ.get("IRAF_PROVIDER_DISPLAY_MODE") or "").strip().lower()
    if mode_env in {"headless", "offscreen", "offscreen_frames"}:
        args.no_window = True
        return True
    return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="远端 Capability Provider 服务端（x86 仿真侧）")
    parser.add_argument("--host", default=os.environ.get("IRAF_PROVIDER_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("IRAF_PROVIDER_PORT", "50052")))
    parser.add_argument("--render-hz", type=float, default=float(os.environ.get("IRAF_PROVIDER_RENDER_HZ", "20")))
    parser.add_argument("--no-window", action="store_true",
                        help="不起窗口（无显示设备时用；Health 会如实报 window_opened=false）")
    parser.add_argument("--subject", default=os.environ.get("IRAF_RUNTIME_SUBJECT", "board-runtime"))
    parser.add_argument("--self-check", action="store_true",
                        help="只验证装配（解释器/依赖/桩/运行时/能力清单），不监听端口；"
                             "退出码 0 全绿 / 2 有硬失败。板卡不在场时用它判断执行端是否健康")
    args = parser.parse_args(argv)

    # 显示模式可由**声明**决定（安装后的服务单元不传命令行参数，只给装配单）。
    apply_display_env(args)

    if args.self_check:
        return self_check(args)

    token = os.environ.get("IRAF_PROVIDER_TOKEN") or ""
    if not token:
        print("[provider-serve][错误] 缺少 IRAF_PROVIDER_TOKEN：跨机 Provider 必须有鉴权，"
              "拒绝在无 token 的情况下监听端口（fail-closed）", file=sys.stderr)
        return EXIT_NOTOKEN

    try:
        import grpc
        from concurrent import futures
    except ImportError as exc:
        print("[provider-serve][错误] 缺少 grpcio：%s" % exc, file=sys.stderr)
        return EXIT_NOTOKEN
    try:
        from iraf.v1 import provider_pb2, provider_pb2_grpc, skill_pb2
    except ImportError as exc:
        print("[provider-serve][错误] 缺少 proto 桩（%s）。先跑：PYTHON=… bash scripts/gen_proto_stubs.sh" % exc,
              file=sys.stderr)
        return EXIT_NOTOKEN

    from iraf_adapters.bootstrap import build_runtime_from_env
    from iraf_adapters.remote.provider_service import build_servicer_class
    from iraf_adapters.mujoco import viewer_runner

    try:
        runtime = build_runtime_from_env()
    except Exception as exc:  # noqa: BLE001 显式失败：装配不上就不起服务
        print("[provider-serve][错误] 装配 Runtime/Backend 失败：%s" % exc, file=sys.stderr)
        return EXIT_ASSEMBLY

    # 显示通道：先探测一次结论，之后**不再二次探测**（内外不一致会静默离屏，实测踩过）
    display_state = {"display_mode": "unknown", "window_opened": False, "detail": ""}
    if args.no_window:
        display_state.update(display_mode="offscreen_frames", detail="--no-window：只跑能力不显示")
        mode = "offscreen_frames"
    else:
        mode, detail, fallback = viewer_runner.resolve_display_mode()
        display_state.update(display_mode=mode, detail=detail)
        if fallback:
            print("[provider-serve] 显示降级原因：%s" % fallback, flush=True)

    Servicer = build_servicer_class(provider_pb2, provider_pb2_grpc, skill_pb2)
    servicer = Servicer(runtime, token, subject=args.subject,
                        provider_id="%s" % os.environ.get("IRAF_BACKEND_ENTRYPOINT", "unknown"),
                        display_state=display_state)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    provider_pb2_grpc.add_CapabilityProviderServiceServicer_to_server(servicer, server)
    bound = server.add_insecure_port("%s:%d" % (args.host, args.port))
    if not bound:
        print("[provider-serve][错误] 无法监听 %s:%d（端口被占用？）" % (args.host, args.port),
              file=sys.stderr)
        return EXIT_ASSEMBLY
    server.start()
    caps = servicer._declared
    print("PROVIDER_LISTEN %s:%d" % (args.host, args.port), flush=True)
    print("PROVIDER_BACKEND %s" % os.environ.get("IRAF_BACKEND_ENTRYPOINT"), flush=True)
    print("PROVIDER_CAPABILITIES %d %s" % (len(caps), ",".join(caps)), flush=True)
    print("PROVIDER_DISPLAY_MODE %s" % display_state["display_mode"], flush=True)

    stop_loop = threading.Event()

    def _shutdown(_signum=None, _frame=None):
        # 两个都要做：置渲染循环的停止标志 **且** 停掉 gRPC 服务 —— 否则 headless 分支
        # 阻塞在 wait_for_termination() 上，SIGTERM 收不到（实测：kill 无效，只能 kill -9）。
        stop_loop.set()
        try:
            server.stop(grace=1)
        except Exception:  # noqa: BLE001 停机路径不掩盖退出
            pass

    try:
        import signal
        signal.signal(signal.SIGTERM, _shutdown)
        signal.signal(signal.SIGINT, _shutdown)
    except (ValueError, OSError):
        pass

    if mode == viewer_runner.DISPLAY_INTERACTIVE:
        # 主线程：只渲染不推进（能力调用在 gRPC 线程里执行，display_lock 互斥）
        report = viewer_runner.run_live_mirror(
            runtime.backend, render_hz=args.render_hz, seconds=0.0, stop_event=stop_loop,
            camera=viewer_runner.camera_settings(runtime.profile),
            display_mode=mode,
            software_gl=str(os.environ.get("IRAF_PROVIDER_SOFTWARE_GL", "")).lower() in {"1", "true", "yes"},
            window_px=_parse_window_px(os.environ.get("IRAF_PROVIDER_WINDOW_PX")),
        )
        display_state.update(window_opened=bool(report.get("window_opened")),
                            detail=report.get("error") or display_state.get("detail") or "")
        print("PROVIDER_WINDOW_OPENED %s" % report.get("window_opened"), flush=True)
        print("PROVIDER_FRAMES %s" % report.get("frames"), flush=True)
    else:
        print("[provider-serve] 无交互显示通道（%s）：仅提供能力，不显示" % mode, flush=True)
        server.wait_for_termination()

    server.stop(grace=2)
    print("PROVIDER_STOPPED", flush=True)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
