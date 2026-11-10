#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IRAF Provider supervisor（阶段 C · C2/C3）

职责（只碰本机，不碰板卡）：
  1. 读 `/etc/iraf-agent/manifest.yaml`，知道"哪个场景 → 哪个 systemd user unit / 端口"；
  2. 同一时刻只让 **一个 world** 在线：`POST /ensure {scene}` 先起目标、等端口 LISTEN、再停其余；
  3. 健康守护：目标 unit 意外退出/端口消失 → 退避重启 1s/3s/9s，连续 3 次失败标记 DEGRADED；
  4. 全量链路：`POST /scenario` 在本机跑 run_full_scenario.sh（多机器人同一世界），返回 passed/steps；
  5. 控制面鉴权：`X-Iraf-Token` + 客户端主机白名单（阶段 C3 起板卡不再 SSH，只调这里）。

不做的事：不判断技能、不执行机器人动作、不改注册表（那是板卡的职责）。
用法：
    python3 iraf_provider_supervisor.py --manifest /etc/iraf-agent/manifest.yaml \
        --bind 0.0.0.0 --port 8770 --token-file ~/.config/iraf/supervisor.token
"""

from __future__ import annotations

import argparse
import json
import os
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

_lock = threading.Lock()
_state = {
    "active_scene": None,
    "degraded": {},
    "attempts": {},
    "last_error": {},
    "history": [],
}

CFG = {"token": "", "allow_hosts": ["127.0.0.1"], "scenario_script": "", "scenario_host": ""}


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
    if not unit:
        return False
    r = subprocess.run(["systemctl", "--user", "is-active", unit],
                       capture_output=True, text=True)
    return (r.stdout or "").strip() == "active"


def unit_restart(unit: str) -> tuple:
    r = subprocess.run(["systemctl", "--user", "restart", unit],
                       capture_output=True, text=True, timeout=120)
    return r.returncode == 0, (r.stderr or r.stdout or "").strip()[-300:]


def unit_stop(unit: str) -> None:
    if unit:
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
        # 别名：注册表改名（joint -> handoff_lab_joint）后，板卡侧的旧名仍可解析，
        # 直到板卡注册表同步完成。别名不进入快照列表，避免同一 Provider 出现两行。
        self.aliases = {}
        for e in manifest.get("providers") or []:
            self.by_scene[e.get("switch_name") or e.get("scene_id")] = e
            for alias in e.get("aliases") or []:
                self.aliases[str(alias)] = e

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
            try:
                _load = os.getloadavg()
            except Exception:
                _load = (0.0, 0.0, 0.0)
            return {"status": "OK", "host_id": self.manifest.get("host_id"),
                    "generated_at": now(), "active_scene": active,
                    "host": {"load_1m": round(_load[0], 2),
                             "loadavg": [round(x, 2) for x in _load],
                             "cores": os.cpu_count() or 0},
                    "providers": rows, "history": _state["history"][-10:]}

    def ensure(self, scene: str) -> dict:
        with _lock:
            target = self.by_scene.get(scene) or self.aliases.get(scene)
            if target is None:
                return {"status": "FAILED", "error_code": "IRAF-SCENE-NOT-FOUND",
                        "reason": "supervisor 无此场景：%s（可用：%s）"
                                  % (scene, ", ".join(sorted(self.by_scene)))}
            steps = []
            ok, err = unit_restart(target["unit"])
            steps.append({"step": "start", "unit": target["unit"], "ok": ok})
            if not ok:
                return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                        "reason": "启动 %s 失败：%s" % (target["unit"], err), "steps": steps}
            if not wait_port(int(target["port"])):
                _state["degraded"][scene] = True
                _state["last_error"][scene] = "port %s 未 LISTEN" % target["port"]
                return {"status": "FAILED", "error_code": "IRAF-PROVIDER-UNAVAILABLE",
                        "reason": "%s 启动后端口 %s 未 LISTEN" % (target["unit"], target["port"]),
                        "steps": steps}
            steps.append({"step": "health", "port": int(target["port"]), "ok": True})
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
            e = self.by_scene.get(scene) or self.aliases.get(scene)
            if e is None:
                return {"status": "FAILED", "error_code": "IRAF-SCENE-NOT-FOUND"}
            unit_stop(e["unit"])
            if _state["active_scene"] == scene:
                _state["active_scene"] = None
            _state["history"].append({"at": now(), "op": "stop", "scene": scene})
            return {"status": "OK", "scene": scene}

    def scenario(self, scenario: str, display: str, hold: int) -> dict:
        """本机跑全量链路（多机器人同一世界）；跑完所有 Provider 已被脚本停掉。"""
        script = CFG.get("scenario_script") or ""
        if not script or not Path(script).is_file():
            return {"status": "FAILED", "error_code": "IRAF-SCENARIO-ERROR",
                    "reason": "scenario 脚本不存在：%r" % script}
        disp = "window" if str(display).lower() in ("window", "interactive", "viewer") else "none"
        t0 = time.time()
        try:
            proc = subprocess.run(["bash", script, scenario, disp, str(int(hold))],
                                  capture_output=True, text=True, timeout=900)
        except Exception as exc:
            return {"status": "FAILED", "error_code": "IRAF-SCENARIO-ERROR",
                    "reason": "%s: %s" % (type(exc).__name__, exc)}
        payload = {}
        for line in reversed((proc.stdout or "").strip().splitlines()):
            if line.startswith("{"):
                try:
                    payload = json.loads(line)
                    break
                except Exception:
                    continue
        # 判定以**报告**为准（项目既有口径：收尾段 SEGV 时报告已落盘）；rc!=0 只作告警字段。
        ok = bool(payload.get("passed"))
        rc = int(payload.get("rc", proc.returncode or 1))
        warning = ""
        if ok and rc != 0:
            warning = "chain_rc=%s，但报告 passed=true（按报告判定成功；收尾段退出异常不影响判据）" % rc
        with _lock:
            _state["active_scene"] = None   # 脚本会停掉全部 provider
            _state["history"].append({"at": now(), "op": "scenario", "scenario": scenario,
                                      "ok": ok, "wall_s": round(time.time() - t0, 1)})
        return {"status": "SUCCEEDED" if ok else "FAILED", "scenario": scenario,
                "world": "joint", "display": disp,
                "passed": payload.get("passed"), "steps": payload.get("steps") or [],
                "rc": rc, "warning": warning,
                "error": payload.get("error"),
                "wall_s": round(time.time() - t0, 1),
                "log": "x86:/tmp/iraf-scenario.log",
                "note": "全量链路在 x86 进程内跑（多机器人同一世界），不经单机 Provider"}

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
        def log_message(self, fmt, *args):
            return

        def _authed(self) -> bool:
            host = (self.client_address or [""])[0]
            if host not in CFG["allow_hosts"]:
                self._send(403, {"status": "FAILED", "error_code": "IRAF-PROVIDER-HOST-REJECTED",
                                 "reason": "控制面不接受来自 %s 的请求" % host})
                return False
            if CFG["token"] and self.headers.get("X-Iraf-Token") != CFG["token"]:
                self._send(401, {"status": "FAILED", "error_code": "IRAF-UNAUTHENTICATED",
                                 "reason": "X-Iraf-Token 缺失或不匹配"})
                return False
            return True

        def _send(self, code: int, payload: dict):
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/healthz":
                return self._send(200, {"status": "OK"})
            if not self._authed():
                return
            if u.path == "/status":
                return self._send(200, sup.snapshot())
            return self._send(404, {"status": "FAILED", "error_code": "IRAF-INPUT-INVALID"})

        def do_POST(self):
            if not self._authed():
                return
            u = urlparse(self.path)
            q = parse_qs(u.query or "")
            if u.path == "/ensure":
                return self._send(200, sup.ensure((q.get("scene") or [""])[0]))
            if u.path == "/stop":
                return self._send(200, sup.stop((q.get("scene") or [""])[0]))
            if u.path == "/scenario":
                scen = (q.get("scenario") or ["nominal"])[0]
                disp = (q.get("display") or ["window"])[0]
                hold = int((q.get("hold") or ["60"])[0] or 60)
                return self._send(200, sup.scenario(scen, disp, hold))
            return self._send(404, {"status": "FAILED", "error_code": "IRAF-INPUT-INVALID"})

    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IRAF provider supervisor")
    ap.add_argument("--manifest", default="/etc/iraf-agent/manifest.yaml")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--token-file", default=str(Path.home() / ".config/iraf/supervisor.token"))
    ap.add_argument("--allow-hosts", default="127.0.0.1,10.203.247.147")
    ap.add_argument("--scenario-script",
                    default="/home/coretek/iraf-instance/scripts/run_full_scenario.sh")
    args = ap.parse_args(argv)

    token = ""
    tf = Path(args.token_file).expanduser()
    if tf.is_file():
        token = tf.read_text(encoding="utf-8").strip()
    else:
        print("[supervisor][warn] token 文件不存在，控制面仅靠主机白名单：%s" % tf, flush=True)
    CFG["token"] = token
    CFG["allow_hosts"] = [h.strip() for h in args.allow_hosts.split(",") if h.strip()]
    CFG["scenario_script"] = args.scenario_script

    sup = Supervisor(load_manifest(Path(args.manifest)))
    print("[supervisor] manifest=%s scenes=%s bind=%s:%d allow=%s token=%s"
          % (args.manifest, sorted(sup.by_scene), args.bind, args.port,
             CFG["allow_hosts"], "on" if token else "off"), flush=True)

    def loop():
        while True:
            try:
                sup.monitor_once()
            except Exception as exc:
                print("[supervisor][warn] monitor: %s" % exc, flush=True)
            time.sleep(MONITOR_INTERVAL_S)

    threading.Thread(target=loop, daemon=True, name="monitor").start()
    ThreadingHTTPServer((args.bind, args.port), make_handler(sup)).serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
