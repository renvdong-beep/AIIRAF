#!/usr/bin/env python3
"""在板卡上测 iraf-demo.service 的 CPU/内存消耗。"""
from __future__ import annotations

import json
import statistics
import subprocess
import threading
import time
import urllib.request

BASE = "http://127.0.0.1:8084"
SERVICE = "iraf-demo"


def svc(field: str) -> int:
    out = subprocess.run(["systemctl", "show", SERVICE, "-p", field],
                         capture_output=True, text=True, check=True).stdout.strip()
    val = out.split("=", 1)[1].strip()
    return int(val) if val.isdigit() else 0


def cpu_ns() -> int:
    """板卡是 cgroup v1：用 cpuacct.usage（ns）；失败再退回 systemd。"""
    path = "/sys/fs/cgroup/cpuacct/system.slice/%s.service/cpuacct.usage" % SERVICE
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except Exception:
        return cpu_ns()


def get(path: str, timeout=30):
    t0 = time.perf_counter()
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        r.read()
        code = r.status
    return code, int((time.perf_counter() - t0) * 1000)


def bench(path: str, n: int):
    get(path); get(path)
    c0, t0 = cpu_ns(), time.perf_counter()
    walls = []
    for _ in range(n):
        code, ms = get(path)
        if code != 200:
            raise RuntimeError("HTTP %s" % code)
        walls.append(ms)
    c1, t1 = cpu_ns(), time.perf_counter()
    cpu_ms = (c1 - c0) / 1e6
    return {"n": n, "cpu_ms_per_req": round(cpu_ms / n, 1),
            "wall_median_ms": round(statistics.median(walls), 1),
            "wall_p95_ms": round(sorted(walls)[max(0, int(n * 0.95) - 1)], 1),
            "core_pct": round(100 * (c1 - c0) / 1e9 / (t1 - t0), 1)}


def main():
    res = {"idle": None, "robots": {}, "concurrent": None,
           "rss_mb": round(svc("MemoryCurrent") / 1048576, 1),
           "tasks": svc("TasksCurrent")}
    c0 = cpu_ns(); time.sleep(10); c1 = cpu_ns()
    res["idle"] = {"cpu_ms_10s": round((c1 - c0) / 1e6, 1),
                   "core_pct": round(100 * (c1 - c0) / 1e9 / 10, 2)}
    for robot, path in (
        ("piper", "/api/plan?robot=piper&x=0.32&y=0.10"),
        ("ur5", "/api/plan?robot=ur5&x=-0.45&y=-0.10"),
        ("go2", "/api/plan?robot=go2&dir=left&distance=0.4"),
        ("g1", "/api/plan?robot=g1&dir=left&distance=0.4"),
    ):
        res["robots"][robot] = bench(path, 10)

    paths = ["/api/plan?robot=g1&dir=forward&distance=0.3",
             "/api/plan?robot=ur5&x=-0.50&y=-0.10",
             "/api/plan?robot=go2&dir=forward&distance=0.3"]
    walls, errs = [], []
    lock = threading.Lock()

    def worker(p):
        for _ in range(5):
            try:
                code, ms = get(p)
                with lock:
                    walls.append(ms)
                    if code != 200:
                        errs.append(code)
            except Exception as exc:  # noqa: BLE001
                with lock:
                    errs.append(str(exc)[:60])

    ths = [threading.Thread(target=worker, args=(paths[i % 3],)) for i in range(4)]
    c0, t0 = cpu_ns(), time.perf_counter()
    for t in ths: t.start()
    for t in ths: t.join()
    c1 = cpu_ns(); t1 = time.perf_counter()
    res["concurrent"] = {
        "requests": len(walls), "errors": len(errs),
        "wall_median_ms": round(statistics.median(walls), 1) if walls else None,
        "wall_p95_ms": round(sorted(walls)[max(0, int(len(walls) * 0.95) - 1)], 1) if walls else None,
        "rps": round(len(walls) / (t1 - t0), 1),
        "core_pct": round(100 * (c1 - c0) / 1e9 / (t1 - t0), 1),
    }
    res["rss_mb_end"] = round(svc("MemoryCurrent") / 1048576, 1)
    print(json.dumps(res, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
