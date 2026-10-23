#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位 x86 仿真 Capability Provider（MuJoCo 场景有状态：演示/复跑前必须换干净场景）。

为什么必须是脚本（三条实测教训）：
1. `IRAF_BACKEND_CONFIG` 是一大段 JSON，命令行里带引号会被 shell 吃掉 ⇒ 只能在进程环境里传。
2. 旧的 /tmp/restart_iraf_provider.py **依赖"从正在跑的进程继承环境"** —— provider 崩了就没救，
   而这恰恰是最需要复位的时刻。
3. 旧版给 token 写了硬编码兜底值（`iraf-e300-probe-2026`），与板上 `runtime.secrets` 不一致时
   会表现成"Provider 起了但任务全部鉴权失败"——静默错配，最费时间。

装配来源（按优先级，缺则显式失败，不猜）：
  ① --env-file / IRAF_PROVIDER_ENV_FILE 指向的 KEY=VALUE 文件（推荐）
  ② 正在跑的 provider 进程的 /proc/<pid>/environ（继承同一份装配）
  ③ 环境变量里已经有的 IRAF_* 项
token 来源：--token-file（默认 ~/.iraf/provider.token，600）→ 环境 → 继承；
          两处都没有就报错并给出建文件的一行命令，**不再编造默认值**。

用法：
  python3 scripts/dev_reset_provider.py --status          # 只读：进程/端口/场景/装配完整性
  python3 scripts/dev_reset_provider.py --reset           # 复位（headless，默认）
  python3 scripts/dev_reset_provider.py --reset --window  # 带窗口（本机 X11）
  python3 scripts/dev_reset_provider.py --reset --save-token  # 顺带把继承到的 token 落到 600 文件
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PYTHON = REPO / "build/mcp-venv/bin/python"
SERVE = REPO / "scripts/iraf_provider_serve.py"
LOG_DIR = REPO / "build/logs"
REQUIRED = ("IRAF_PROFILE", "IRAF_SAFETY_POLICY", "IRAF_SKILL_ROOT",
            "IRAF_BACKEND_ENTRYPOINT", "IRAF_BACKEND_CONFIG", "IRAF_EVENT_STORE")
DISPLAY_KEYS = ("DISPLAY", "XAUTHORITY", "LIBGL_ALWAYS_SOFTWARE")
DEFAULT_TOKEN_FILE = Path.home() / ".iraf/provider.token"


def find_pids() -> list[int]:
    out = subprocess.run(["pgrep", "-f", "[i]raf_provider_serve"],
                         capture_output=True, text=True).stdout.split()
    return [int(p) for p in out]


def read_environ(pid: int) -> dict:
    raw = Path("/proc/%d/environ" % pid).read_bytes()
    env = {}
    for item in raw.split(b"\0"):
        if b"=" not in item:
            continue
        key, _, value = item.partition(b"=")
        env[key.decode()] = value.decode()
    return env


def read_env_file(path: Path) -> dict:
    env = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        env[key.strip()] = value.strip().strip("'\"")
    return env


def read_token_file(path: Path) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("IRAF_PROVIDER_TOKEN="):
            return line.split("=", 1)[1].strip().strip("'\"")
        return line.strip().strip("'\"")
    return ""


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket() as s:
        s.settimeout(1.0)
        return s.connect_ex((host, port)) == 0


def build_env(args) -> tuple[dict, str]:
    """装配来源：env-file → 在跑进程 → 当前环境。返回 (env, token 来源说明)。"""
    env: dict = {}
    source = "none"
    if args.env_file:
        path = Path(args.env_file)
        if not path.is_file():
            raise SystemExit("[错误] --env-file 不存在：%s" % path)
        env.update(read_env_file(path))
        source = "env-file:%s" % path
    if not env:
        pids = find_pids()
        if pids:
            env.update(read_environ(pids[0]))
            source = "proc:%d" % pids[0]
    if not env:
        env.update({k: v for k, v in os.environ.items() if k.startswith("IRAF_")})
        source = "shell-env" if env else "none"

    missing = [k for k in REQUIRED if not env.get(k)]
    if missing:
        raise SystemExit(
            "[错误] 装配不完整，缺 %s\n"
            "  装配来源=%s\n"
            "  处置：给 --env-file <KEY=VALUE 文件>（含 IRAF_PROFILE / IRAF_SAFETY_POLICY /\n"
            "        IRAF_SKILL_ROOT / IRAF_BACKEND_ENTRYPOINT / IRAF_BACKEND_CONFIG / IRAF_EVENT_STORE），\n"
            "        或先按文档手工起一次 provider 让本脚本继承。" % (missing, source))

    # token：文件优先（可复现），其次继承
    token = ""
    token_src = ""
    token_file = Path(args.token_file or DEFAULT_TOKEN_FILE)
    if token_file.is_file():
        token = read_token_file(token_file)
        token_src = "file:%s" % token_file
    if not token:
        token = env.get("IRAF_PROVIDER_TOKEN") or os.environ.get("IRAF_PROVIDER_TOKEN") or ""
        token_src = "inherited" if token else ""
    if not token:
        raise SystemExit(
            "[错误] 没有 IRAF_PROVIDER_TOKEN（Provider 会 fail-closed，不监听端口）\n"
            "  建一个 600 的就够了（值必须与板上 /etc/iraf/runtime.secrets 里的一致）：\n"
            "    mkdir -p ~/.iraf && printf 'IRAF_PROVIDER_TOKEN=<值>\\n' > %s && chmod 600 %s"
            % (DEFAULT_TOKEN_FILE, DEFAULT_TOKEN_FILE))
    env["IRAF_PROVIDER_TOKEN"] = token

    # 显示/GL：headless 走 egl（无 X11 也能渲），窗口模式要 glfw + DISPLAY + 软件 GL（无独显）
    if args.window:
        env["MUJOCO_GL"] = "glfw"
        env["DISPLAY"] = os.environ.get("DISPLAY") or ":0"
        env["XAUTHORITY"] = os.environ.get("XAUTHORITY") or "/run/user/1000/gdm/Xauthority"
        env["LIBGL_ALWAYS_SOFTWARE"] = os.environ.get("LIBGL_ALWAYS_SOFTWARE") or "1"
    else:
        env["MUJOCO_GL"] = "egl"
        for key in DISPLAY_KEYS:
            env.pop(key, None)
    # PYTHONPATH 只保留仓库内的两份：混进 build/research/mpc-env 会拉到另一个 mujoco（实测）
    env["PYTHONPATH"] = "src:build/generated/python"
    return env, "%s | token=%s(len=%d,head=%s)" % (
        source, token_src, len(token), token[:6])


def status(args) -> int:
    pids = find_pids()
    print("provider 进程 :", ", ".join(str(p) for p in pids) if pids else "**没有在跑**")
    for pid in pids:
        etime = subprocess.run(["ps", "-o", "etime=", "-p", str(pid)],
                               capture_output=True, text=True).stdout.strip()
        env = read_environ(pid)
        token = env.get("IRAF_PROVIDER_TOKEN", "")
        print("  pid=%d 已跑=%s MUJOCO_GL=%s token_len=%d 场景=%s"
              % (pid, etime, env.get("MUJOCO_GL", "?"), len(token),
                 "脏（跑过任务；演示前请 --reset）" if etime and ":" in etime and etime.count(":") >= 1 else "未判定"))
        missing = [k for k in REQUIRED if not env.get(k)]
        if missing:
            print("  [警告] 该进程装配缺：%s" % missing)
    print("端口 %d     : %s" % (args.port, "LISTEN" if port_open(args.port) else "未监听"))
    if not pids and not port_open(args.port):
        print("提示：Provider 没在跑 ⇒ 板上任何 submit 都会 IRAF-PROVIDER-* 失败（fail-closed，不会伪造成功）。")
    return 0 if (pids and port_open(args.port)) else 3


def reset(args) -> int:
    env, src = build_env(args)
    pids = find_pids()
    if pids:
        print("停止旧 provider：%s" % pids)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for _ in range(40):
            if not find_pids():
                break
            time.sleep(0.5)
        for pid in find_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    if port_open(args.port):
        print("[错误] 停止后端口 %d 仍在监听（有别的进程占着）：换 --port 或先查占用" % args.port)
        return 4

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / ("provider-%s.log" % time.strftime("%Y%m%d-%H%M%S"))
    cmd = [str(PYTHON), str(SERVE), "--host", args.host, "--port", str(args.port)]
    if not args.window:
        cmd.append("--no-window")
    with log_path.open("ab", buffering=0) as log:
        proc = subprocess.Popen(cmd, cwd=str(REPO), env=env, stdout=log, stderr=log,
                                start_new_session=True)
    print("已启动：pid=%d mode=%s log=%s" % (proc.pid, "window" if args.window else "headless", log_path))
    print("装配来源：%s" % src)

    for _ in range(120):
        if port_open(args.port):
            print("✓ 端口 %d 已监听（等待 %.1fs）" % (args.port, _ * 0.5))
            print("✓ 场景已复位：本次抓取会从干净状态开始（不要连跑两次）")
            if args.save_token and str(src).find("token=inherited") >= 0:
                tf = Path(args.token_file or DEFAULT_TOKEN_FILE)
                tf.parent.mkdir(parents=True, exist_ok=True)
                tf.write_text("IRAF_PROVIDER_TOKEN=%s\n" % env["IRAF_PROVIDER_TOKEN"], encoding="utf-8")
                tf.chmod(0o600)
                print("✓ 已把继承到的 token 落到 %s（600；下次无需再继承）" % tf)
            return 0
        if proc.poll() is not None:
            print("[错误] provider 启动即退出（rc=%s），日志尾部：" % proc.returncode)
            print("\n".join(log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-15:]))
            return 5
        time.sleep(0.5)
    print("[错误] 60s 内未监听端口 %d，日志：%s" % (args.port, log_path))
    return 6


def main() -> int:
    p = argparse.ArgumentParser(description="复位 x86 仿真 Capability Provider")
    p.add_argument("--status", action="store_true", help="只读体检（不重启）")
    p.add_argument("--reset", action="store_true", help="复位（停旧 + 起新）")
    p.add_argument("--window", action="store_true", help="带窗口（本机 X11；默认 headless）")
    p.add_argument("--host", default=os.environ.get("IRAF_PROVIDER_HOST", "0.0.0.0"))
    p.add_argument("--port", type=int, default=int(os.environ.get("IRAF_PROVIDER_PORT", "50052")))
    p.add_argument("--env-file", default=os.environ.get("IRAF_PROVIDER_ENV_FILE"))
    p.add_argument("--token-file", default=os.environ.get("IRAF_PROVIDER_TOKEN_FILE"))
    p.add_argument("--save-token", action="store_true",
                   help="复位后把继承到的 token 落到 600 文件（不打印值）")
    args = p.parse_args()
    if args.status:
        return status(args)
    if args.reset:
        return reset(args)
    p.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
