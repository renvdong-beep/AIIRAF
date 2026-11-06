#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""x86 本地场景 supervisor：一次只跑一个 MuJoCo Provider 世界。

为什么需要它：
  piper / ur5 / joint 三个演示场景不能同时跑（2 核、单控制源、单窗口）；
  AgentOS 没有 x86 权限，也不应该由它启动/停止 Provider。
  本脚本只在本机运行，负责：
    scene status
    scene up piper|ur5|joint|g1
    scene down
  并在 up 后等端口 + Provider Health 就绪，再把结果告诉调用方。

边界：
  · 不启动 AgentOS，不改边缘配置；
  · 不伪造 Health；Provider 起不来就显式失败；
  · joint 场景由 iraf-provider@joint.service 承载（piper 为控制对象，SceneProfile 已声明）。
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PY = REPO / "build/mcp-venv/bin/python"
HEALTH = REPO / "scripts/probe_provider_health.py"
SECRETS = Path.home() / ".config/iraf/runtime.secrets"

SCENES = {
    "piper": {"unit": "iraf-provider@piper.service", "port": 50052,
              "profile": "profiles/piper_mujoco.yaml", "implemented": True},
    "ur5": {"unit": "iraf-provider@ur5.service", "port": 50053,
            "profile": "profiles/ur5_mujoco.yaml", "implemented": True},
    "g1": {"unit": "iraf-provider@g1.service", "port": 50055,
           "profile": "profiles/unitree_g1_mujoco.yaml", "implemented": True},
    # 联合世界：一个 Provider 承载 piper+ur5e+go2 同一个 MuJoCo 世界；控制对象是 piper。
    # SceneProfile 见 scenes/handoff_lab/scene-profile.yaml；能力只声明 piper 已验收的那组。
    "joint": {"unit": "iraf-provider@joint.service", "port": 50054,
              "profile": "profiles/piper_mujoco.yaml",
              "scene_profile": "scenes/handoff_lab/scene-profile.yaml",
              "implemented": True},
}


def run(cmd, timeout=30):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def systemctl(*args):
    return run(["systemctl", "--user", *args])


def port_open(port: int) -> bool:
    with socket.socket() as sock:
        sock.settimeout(0.5)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def provider_token() -> str:
    if not SECRETS.is_file():
        return ""
    for line in SECRETS.read_text(encoding="utf-8").splitlines():
        if line.startswith("IRAF_PROVIDER_TOKEN="):
            return line.split("=", 1)[1].strip()
    return ""


def health_once(scene: str) -> tuple[bool, str]:
    info = SCENES[scene]
    python = str(PY) if PY.is_file() else sys.executable
    env = dict(os.environ)
    env["IRAF_PROVIDER_TOKEN"] = provider_token()
    env["PYTHONPATH"] = "%s/src:%s/build/generated/python" % (REPO, REPO)
    result = subprocess.run(
        [python, str(HEALTH), "--endpoint", "127.0.0.1:%d" % info["port"]],
        capture_output=True, text=True, timeout=30, env=env)
    text = (result.stdout or "") + (result.stderr or "")
    return result.returncode == 0, text.strip().splitlines()[-1][:200] if text else ""


def status() -> int:
    for name, info in SCENES.items():
        active = systemctl("is-active", info["unit"]).stdout.strip()
        enabled = systemctl("is-enabled", info["unit"]).stdout.strip()
        listening = port_open(info["port"])
        print("%-6s unit=%-30s active=%-8s enabled=%-8s port=%d %s%s" % (
            name, info["unit"], active or "-", enabled or "-", info["port"],
            "LISTEN" if listening else "-",
            "" if info["implemented"] else "  [未实现]"))
    return 0


def stop_all(keep: str = ""):
    for name, info in SCENES.items():
        if name == keep:
            continue
        systemctl("stop", info["unit"])


def up(scene: str) -> int:
    info = SCENES.get(scene)
    if not info:
        print("[scene][错误] 未知场景：%s（可选 %s）" % (scene, ", ".join(SCENES)), file=sys.stderr)
        return 2
    if not info["implemented"]:
        print("[scene][错误] %s 场景尚未实现 Provider/SceneProfile；不能假装可用" % scene,
              file=sys.stderr)
        print("[scene] 需要的声明：%s + iraf-provider@%s.service（50054）" % (
            info["profile"], scene), file=sys.stderr)
        return 3

    print("[scene] 停止其他世界，准备启动 %s" % scene)
    stop_all(keep=scene)
    result = systemctl("start", info["unit"])
    if result.returncode != 0:
        print("[scene][错误] 启动 %s 失败：%s" % (info["unit"], result.stderr.strip()[:300]),
              file=sys.stderr)
        return 4

    deadline = time.time() + 45
    last = ""
    while time.time() < deadline:
        if port_open(info["port"]):
            ok, detail = health_once(scene)
            if ok:
                print("[scene] %s ready：port=%d unit=%s" % (scene, info["port"], info["unit"]))
                print("[scene] %s" % detail)
                return 0
            last = detail
        time.sleep(1)
    print("[scene][错误] %s 在 45s 内未 ready；最近：%s" % (scene, last or "端口未监听"),
          file=sys.stderr)
    systemctl("stop", info["unit"])
    return 5


def down() -> int:
    stop_all()
    print("[scene] 已停止所有本地 Provider 世界")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["status", "up", "down"])
    parser.add_argument("scene", nargs="?", default="")
    args = parser.parse_args(argv)
    if args.action == "status":
        return status()
    if args.action == "down":
        return down()
    if not args.scene:
        print("[scene][错误] up 需要场景：%s" % ", ".join(SCENES), file=sys.stderr)
        return 2
    return up(args.scene)


if __name__ == "__main__":
    raise SystemExit(main())
