#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开发档 Provider 守护：按 profile 声明确保 x86 MuJoCo Provider 在跑、且**看得见**。

口径（2026-10-23 使用者诉求 + AGENTS 铁律的折中）：
  诉求：""抓取 box_01" 这种开发语义，应该默认就启动 x86 的 mujoco 来演示"。
  铁律：不可变配置优先（2.6）、不可用时显式失败（1.5）⇒ **不能偷偷起服务**。
  折中：默认值**写在 profile 的 dev 段里**（声明式），由本守护进程读取并执行；
       守护进程是**运维设施**，由人显式启动（默认单次检查，--watch 才常驻），
       生产 profile 不写 dev 段 ⇒ 行为与以前一致。

profile 声明（`profiles/piper_mujoco.yaml`）：
  spec:
    dev:
      provider_display: window      # window | headless
      autostart_provider: true      # false = 只报告，不拉起

用法：
  python3 scripts/dev_provider_supervisor.py --check                 # 只读：声明 vs 实际
  python3 scripts/dev_provider_supervisor.py --ensure                # 不一致就按声明修（一次）
  python3 scripts/dev_provider_supervisor.py --ensure --watch 30     # 常驻：每 30s 检查一次
  # 可选：--profile profiles/piper_mujoco.yaml（默认取 IRAF_PROFILE 或 piper_mujoco.yaml）
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
RESET_SCRIPT = REPO / "scripts/dev_reset_provider.py"
DEFAULT_PROFILE = REPO / "profiles/piper_mujoco.yaml"


def load_dev_declaration(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        raise SystemExit("缺 PyYAML（python3 -m pip install pyyaml）")
    if not path.is_file():
        raise SystemExit("profile 不存在：%s" % path)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    spec = data.get("spec") or {}
    dev = spec.get("dev") or {}
    return {
        "profile": str(path),
        "verification": str(spec.get("verification") or "unverified"),
        "display": str(dev.get("provider_display") or "headless").lower(),
        "autostart": bool(dev.get("autostart_provider")),
        "declared": bool(dev),
    }


def port_open(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(1.0)
        return s.connect_ex(("127.0.0.1", port)) == 0


def find_pids() -> list[int]:
    out = subprocess.run(["pgrep", "-f", "[i]raf_provider_serve"],
                         capture_output=True, text=True).stdout.split()
    return [int(p) for p in out]


def current_display_mode(pids: list[int]) -> str:
    """从进程命令行/环境判断当前显示模式（window / headless / none）。"""
    if not pids:
        return "none"
    cmd = subprocess.run(["ps", "-o", "args=", "-p", str(pids[0])],
                         capture_output=True, text=True).stdout
    if "--no-window" in cmd:
        return "headless"
    env_raw = Path("/proc/%d/environ" % pids[0]).read_bytes()
    if b"IRAF_VIEWER_DISPLAY_MODE=interactive" in env_raw:
        return "window"
    return "window" if b"MUJOCO_GL=glfw" in env_raw else "headless"


def report(decl: dict, port: int) -> tuple[bool, str]:
    pids = find_pids()
    mode = current_display_mode(pids)
    listening = port_open(port)
    ok = bool(pids) and listening and mode == decl["display"]
    detail = ("声明=%s（autostart=%s） 实际=%s pid=%s 端口%d=%s"
              % (decl["display"], decl["autostart"], mode,
                 ",".join(str(p) for p in pids) or "-", port,
                 "LISTEN" if listening else "未监听"))
    return ok, detail


def ensure(decl: dict, port: int) -> int:
    if not decl["declared"]:
        print("[supervisor] profile 没有 dev 段 ⇒ 不自动拉起（与生产行为一致）。%s" % decl["profile"])
        ok, detail = report(decl, port)
        print("[supervisor] " + detail)
        return 0 if ok else 3
    if not decl["autostart"]:
        ok, detail = report(decl, port)
        print("[supervisor] autostart_provider=false ⇒ 只报告：%s" % detail)
        return 0 if ok else 3

    ok, detail = report(decl, port)
    if ok:
        print("[supervisor] ✓ 已符合声明：%s" % detail)
        return 0

    print("[supervisor] 需要修正：%s" % detail)
    cmd = [sys.executable, str(RESET_SCRIPT), "--reset", "--port", str(port)]
    if decl["display"] == "window":
        cmd.append("--window")
    print("[supervisor] 执行：%s" % " ".join(cmd[-3:]))
    rc = subprocess.call(cmd)
    if rc != 0:
        print("[supervisor][错误] provider 拉起失败（rc=%d）" % rc, file=sys.stderr)
        return 4
    for _ in range(20):
        ok, detail = report(decl, port)
        if ok:
            print("[supervisor] ✓ 修正后符合声明：%s" % detail)
            return 0
        time.sleep(0.5)
    print("[supervisor][错误] 拉起后仍不符合声明：%s" % detail, file=sys.stderr)
    return 5


def main() -> int:
    p = argparse.ArgumentParser(description="按 profile dev 声明守护 x86 MuJoCo Provider")
    p.add_argument("--check", action="store_true", help="只读：声明 vs 实际")
    p.add_argument("--ensure", action="store_true", help="不一致就按声明修正（一次）")
    p.add_argument("--watch", type=int, default=0, metavar="秒",
                   help="配合 --ensure：常驻，每 N 秒检查一次（Ctrl-C 退出）")
    p.add_argument("--profile", default=os.environ.get("IRAF_PROFILE") or str(DEFAULT_PROFILE))
    p.add_argument("--port", type=int, default=int(os.environ.get("IRAF_PROVIDER_PORT", "50052")))
    args = p.parse_args()

    profile_path = Path(args.profile)
    if not profile_path.is_absolute():
        profile_path = REPO / profile_path
    decl = load_dev_declaration(profile_path)
    print("[supervisor] profile=%s verification=%s dev=%s"
          % (decl["profile"], decl["verification"],
             {k: decl[k] for k in ("display", "autostart")} if decl["declared"] else "（未声明）"))

    if args.check or not (args.ensure or args.watch):
        ok, detail = report(decl, args.port)
        print("[supervisor] %s %s" % ("✓ 符合" if ok else "✗ 不符合", detail))
        return 0 if ok else 3

    rc = ensure(decl, args.port)
    if args.watch and rc == 0:
        print("[supervisor] 常驻模式：每 %ds 检查一次（Ctrl-C 退出）" % args.watch)
        try:
            while True:
                time.sleep(max(5, args.watch))
                ensure(decl, args.port)
        except KeyboardInterrupt:
            print("\n[supervisor] 已退出")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
