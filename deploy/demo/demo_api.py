#!/usr/bin/env python3
"""IRAF 机器人演示统一服务（8082）：Piper / UR5 / Go2，同一页面切换。

  GET /api/robots                         → 机器人列表、默认位置、可抓取范围、场景图
  GET /api/plan?robot=ur5&x=&y=&z=        → 机械臂 5 段轨迹（MuJoCo 运动学 IK/FK）
  GET /api/plan?robot=go2&dir=&distance=  → 四足前后左右移动（运动学步态）
  其余路径 → /home/coretek/demo 静态文件
"""
from __future__ import annotations

import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, "/home/coretek/piper-demo")
sys.path.insert(0, "/home/coretek/ur5-demo")
sys.path.insert(0, "/home/coretek/demo")
sys.path.insert(0, str(Path(__file__).resolve().parent))

from piper_api import Planner as PiperPlanner  # noqa: E402
from ur5_api import Planner as Ur5Planner      # noqa: E402
from go2_api import Go2Planner                 # noqa: E402

ROOT = Path(os.environ.get("IRAF_DEMO_ROOT") or "/home/coretek/demo")
SCENE_ROOT = Path(os.environ.get("IRAF_DEMO_SCENE_ROOT")
                  or "/home/coretek/AIIRAF/deploy/sdk/prebuilt/scenes")
RANGES = Path(os.environ.get("IRAF_DEMO_RANGES") or "/home/coretek/piper-demo/ranges.json")
JOB_DIR = Path(os.environ.get("IRAF_DEMO_JOB_DIR") or "/var/lib/iraf/demo-jobs")

SCENES = {
    "piper": (PiperPlanner, SCENE_ROOT / "piper-pick-scene/piper-pick-scene.xml",
              "Piper（6 轴 + 平行夹爪）", "assets/piper_scene.png", "arm"),
    "ur5": (Ur5Planner, SCENE_ROOT / "ur5-pick-scene/ur5-pick-scene.xml",
            "UR5（6 轴 + Robotiq 2F-85）", "assets/ur5_scene.png", "arm"),
    "go2": (Go2Planner, None, "Go2（四足）", "assets/go2_scene.png", "quadruped"),
}
PLANNERS = {}
for _name, (_cls, _xml, _label, _img, _kind) in SCENES.items():
    PLANNERS[_name] = _cls() if _xml is None else _cls(_xml)


def robots_payload():
    ranges = {}
    if RANGES.is_file():
        ranges = json.loads(RANGES.read_text(encoding="utf-8"))
    out = {}
    for name, (_cls, _xml, label, image, kind) in SCENES.items():
        if kind == "quadruped":
            out[name] = {"label": label, "image": image, "kind": kind,
                         "base": [0.0, 0.0], "ranges": None,
                         "reference": PLANNERS[name].build()["reference"]}
        else:
            r = ranges.get(name) or {}
            out[name] = {"label": label, "image": image, "kind": kind,
                         "base": PLANNERS[name].base,
                         "ranges": {k: r.get(k) for k in ("x", "y", "z")}}
    return {"robots": out, "default": "piper"}


def _jobs(limit=20):
    """按 mtime 取最近 job（旧→新）。"""
    if not JOB_DIR.is_dir():
        return []
    files = sorted(JOB_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime)[-limit:]
    out = []
    for f in files:
        try:
            d = json.loads(f.read_text(encoding="utf-8"))
            d["seq"] = f.stem
            out.append(d)
        except Exception:
            continue
    return out


def _jobs_since(since: str, limit=20):
    jobs = _jobs(limit)
    if not since:
        return jobs[-5:]
    for i, job in enumerate(jobs):
        if job.get("seq") == since:
            return jobs[i + 1:]
    return jobs[-1:]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("[demo] " + fmt % args, flush=True)

    def _json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/robots":
            return self._json(200, robots_payload())
        if parsed.path in ("/api/jobs", "/api/jobs/latest"):
            q = parse_qs(parsed.query)
            since = (q.get("since", [""])[0] or "").strip()
            jobs = _jobs_since(since)
            if parsed.path.endswith("latest"):
                return self._json(200, {"job": jobs[-1] if jobs else None})
            return self._json(200, {"jobs": jobs})
        if parsed.path == "/api/plan":
            q = parse_qs(parsed.query)
            robot = (q.get("robot", ["piper"])[0] or "piper").lower()
            if robot not in PLANNERS:
                return self._json(400, {"status": "FAILED", "error": "未知机器人: %s" % robot})
            planner = PLANNERS[robot]
            try:
                if robot == "go2":
                    direction = (q.get("dir", ["forward"])[0] or "forward").lower()
                    distance = float(q.get("distance", ["0.5"])[0])
                    return self._json(200, {"status": "OK",
                                            "result": planner.build(direction, distance)})
                x = float(q.get("x", [planner.base[0]])[0])
                y = float(q.get("y", [planner.base[1]])[0])
                z = float(q.get("z", [planner.base[2]])[0])
                if not (-0.8 <= x <= 0.8 and -0.8 <= y <= 0.8 and 0.0 <= z <= 0.4):
                    raise ValueError("坐标超出允许范围（|x|,|y|≤0.8，0≤z≤0.4）")
                result = planner.build((x, y, z))
                if result["meta"]["ik_error_m"] > 1e-3:
                    return self._json(409, {"status": "UNREACHABLE",
                                            "error": "IK 不收敛：残差 %.2e m" % result["meta"]["ik_error_m"],
                                            "result": result})
                return self._json(200, {"status": "OK", "result": result})
            except Exception as exc:
                return self._json(400, {"status": "FAILED", "error": str(exc)[:200]})
        rel = parsed.path.lstrip("/") or "index.html"
        if ".." in rel:
            return self._json(400, {"status": "FAILED", "error": "bad path"})
        path = ROOT / rel
        if not path.is_file():
            return self._json(404, {"status": "NOT_FOUND", "path": rel})
        ctype = {".html": "text/html; charset=utf-8", ".json": "application/json",
                 ".png": "image/png", ".jpg": "image/jpeg", ".js": "text/javascript"}.get(
                     path.suffix, "application/octet-stream")
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main():
    global ROOT
    import argparse
    import time

    ap = argparse.ArgumentParser()
    ap.add_argument("--ports", default="8082", help="逗号分隔，如 8084,8085")
    ap.add_argument("--root", default=None)
    args = ap.parse_args()
    if args.root:
        ROOT = Path(args.root).resolve()
    ROOT.mkdir(parents=True, exist_ok=True)
    ports = [int(x) for x in str(args.ports).split(",") if x.strip()]
    servers = []
    for port in ports:
        srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append((port, srv))
        print("[demo] serving %s on :%d robots=%s jobs=%s"
              % (ROOT, port, list(PLANNERS), JOB_DIR), flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
