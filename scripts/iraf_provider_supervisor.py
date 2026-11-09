#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IRAF Provider supervisor（阶段 C · C2）

职责（只碰本机，不碰板卡）：
  1. 读 `/etc/iraf-agent/manifest.yaml`，知道"哪个场景 → 哪个 systemd user unit / 端口"；
  2. 同一时刻只让 **一个 world** 在线：`POST /ensure {scene}` 先起目标、等端口 LISTEN、再停其余；
  3. 健康守护：目标 unit 意外退出/端口消失 → 退避重启 1s/3s/9s，连续 3 次失败标记 DEGRADED；
  4. 暴露本机 HTTP 控制面（默认 127.0.0.1:8770），供阶段 C3 替换掉板卡的 SSH 启停。

不做的事：不判断技能、不执行机器人动作、不改注册表（那是板卡的职责）。
用法：
    python3 iraf_provider_supervisor.py --manifest /etc/iraf-agent/manifest.yaml
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    import yaml
except Exception:  # pragma: no cover
    print("需要 PyYAML", file=sys.stderr)
    sys.exit(3)

BACKOFF = (1, 3, 9)
HEALTH_TIMEOUT_S = 60
MONITOR_INTERVAL_S = 5
DEGRADED_RETRY_S = 30

_lock = threading.Lock()
_state = {
    "active_scene": None,
    "degraded": {},
    "attempts": {},
    "last_error": {},
    "history": [],
}


def now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def load_manifest(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if data.get("schema") != "iraf.host-manifest/v1":
        raise SystemExit("manifest schema 不是 iraf.host-manifest/v1：%s" % path)
    return data


def port_open(port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", int(port)), timeout=timeout):
            return True
    except Exception:
        return False


def unit_active(unit: str) -> bool:
    r = subprocess.run(["systemctl", "--user", "is-active", unit],
                       capture_output=True, text=True)
    return (r.stdout or "").strip() == "active"


def unit_restart(unit: str) -> tuple:
    r = subprocess.run(["systemctl", "--user", "restart", unit],
                       capture_output=True, text=True, timeout=120)
    return r.returncode == 0, (r.stderr or r.stdout or "").strip()[-300:]


def unit_stop(unit: str) -> None:
    subprocess.run(["systemctl", "--user", "stop", unit], capture_output=True, timeout=120)


def wait_port(port: int, timeout: int = HEALTH_TIMEOUT_S) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port_open(port):
            return True
        time.sleep(0.5)
    return False


class Supervisor:
    def __init__(self, manifest: dict):
        self.manifest = manifest
        self.by_scene = {}
        for e in manifest.get("providers") or []:
            key = e.get("switch_name") or e.get("scene_id")
            self.by_scene[key] = e

    # ---- 查询 -------------------------------------------------------
    def snapshot(self) -> dict:
        rows = []
        with _lock:
            active = _state["active_scene"]
            for scene, e in sorted(self.by_scene.items()):
                port = int(e.get("port") or 0)
                rows.append({
                    "scene": scene,
                    "provider_id": e.get("provider_id"),
                    "unit": e.get("unit"),
                    "port": port,
                    "unit_active": unit_active(e.get("unit") or ""),
                    "port_open": port_open(port) if port else False,
                    "active": scene == active,
                    "degraded": bool(_state["degraded"].get(scene)),
                    "attempts": int(_state["attempts"].get(scene, 0)),
                    "last_error": _state["last_error"].get(scene, ""),
                })
            return {
                "status": "OK",
                "host_id": self.manifest.get("host_id"),
                "generated_at": now(),
                "active_scene": active,
                "providers": rows,
                "history": _state["history"][-10:],
            }

    # ---- 切换 -------------------------------------------------------
    def ensure(self, scene: str) -> dict:
        with _lock:
            target = self.by_scene.get(scene)
            if target is None:
                return {"status": "FAILED", "error_code": "IRAF-SCENE-NOT-FOUND",
                        "reason": "supervisor 无此场景：%s（可用：%s）"
                                  % (scene, ", ".join(sorted(self.by_scene)))}
            steps = []
            # 1) 先起目标
            ok, err = unit_restart(target["unit"])
            steps.append({"step": "start", "unit": target["unit"], "ok": ok, "err": err[:120]})
            if not ok:
                return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                        "reason": "启动 %s 失败：%s" % (target["unit"], err), "steps": steps}
            # 2) 等端口
            if not wait_port(int(target["port"])):
                _state["degraded"][scene] = True
                _state["last_error"][scene] = "port %s 未 LISTEN" % target["port"]
                return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                        "reason": "%s 启动后端口 %s 未 LISTEN"
                                  % (target["unit"], target["port"]), "steps": steps}
            steps.append({"step": "health", "port": int(target["port"]), "ok": True})
            # 3) 再停其余
            for other_scene, other in sorted(self.by_scene.items()):
                if other_scene == scene:
                    continue
                if unit_active(other.get("unit") or ""):
                    unit_stop(other["unit"])
                    steps.append({"step": "stop", "unit": other["unit"], "ok": True})
            _state["active_scene"] = scene
            _state["degraded"][scene] = False
            _state["attempts"][scene] = 0
            _state["last_error"][scene] = ""
            _state["history"].append({"at": now(), "op": "ensure", "scene": scene})
            return {"status": "OK", "active_scene": scene, "steps": steps}

    def stop(self, scene: str) -> dict:
        with _lock:
            e = self.by_scene.get(scene)
            if e is None:
                return {"status": "FAILED", "error_code": "IRAF-SCENE-NOT-FOUND"}
            unit_stop(e["unit"])
            if _state["active_scene"] == scene:
                _state["active_scene"] = None
            _state["history"].append({"at": now(), "op": "stop", "scene": scene})
            return {"status": "OK", "scene": scene}

    # ---- 守护 -------------------------------------------------------
    def monitor_once(self) -> None:
        with _lock:
            scene = _state["active_scene"]
            if not scene or scene not in self.by_scene:
                return
            e = self.by_scene[scene]
            port = int(e.get("port") or 0)
            if unit_active(e["unit"]) and port_open(port):
                return
            attempts = int(_state["attempts"].get(scene, 0))
            if attempts >= len(BACKOFF):
                if not _state["degraded"].get(scene):
                    _state["degraded"][scene] = True
                    _state["last_error"][scene] = "连续 %d 次重启失败，标记 DEGRADED" % attempts
                    _state["history"].append({"at": now(), "op": "degraded", "scene": scene})
                return
            delay = BACKOFF[attempts]
        time.sleep(delay)
        ok, err = unit_restart(e["unit"])
        healthy = ok and wait_port(port, timeout=20)
        with _lock:
            _state["attempts"][scene] = attempts + 1
            _state["history"].append({"at": now(), "op": "restart", "scene": scene,
                                      "attempt": attempts + 1, "ok": healthy})
            if healthy:
                _state["attempts"][scene] = 0
                _state["last_error"][scene] = ""
            else:
                _state["last_error"][scene] = (err or "restart 后健康检查失败")[:200]


def make_handler(sup: Supervisor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 静默
            return

        def _send(self, code: int, payload: dict):
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/status":
                return self._send(200, sup.snapshot())
            if u.path == "/healthz":
                return self._send(200, {"status": "OK"})
            return self._send(404, {"status": "FAILED", "error_code": "IRAF-INPUT-INVALID"})

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query or "")
            scene = (q.get("scene") or [""])[0]
            if u.path == "/ensure":
                return self._send(200, sup.ensure(scene))
            if u.path == "/stop":
                return self._send(200, sup.stop(scene))
            return self._send(404, {"status": "FAILED", "error_code": "IRAF-INPUT-INVALID"})

    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRAF provider supervisor")
    ap.add_argument("--manifest", default="/etc/iraf-agent/manifest.yaml")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8770)
    args = ap.parse_args(argv)

    sup = Supervisor(load_manifest(Path(args.manifest)))
    print("[supervisor] manifest=%s scenes=%s bind=%s:%d"
          % (args.manifest, sorted(sup.by_scene), args.bind, args.port), flush=True)

    def loop():
        while True:
            try:
                sup.monitor_once()
            except Exception as exc:  # 守护线程不能死
                print("[supervisor][warn] monitor: %s" % exc, flush=True)
            time.sleep(MONITOR_INTERVAL_S)

    threading.Thread(target=loop, daemon=True, name="monitor").start()
    srv = ThreadingHTTPServer((args.bind, args.port), make_handler(sup))
    srv.serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
