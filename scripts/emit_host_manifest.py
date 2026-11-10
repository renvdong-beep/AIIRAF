#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把板卡注册表派生成 **某一台 x86 主机** 的本地 manifest（阶段 C · C1）。

设计
----
真源只有一个：板卡 `/etc/iraf/{scenes,providers}`。
本脚本在 x86 主机上运行，读入注册表副本（--src），按 --host-id 过滤主机，
再结合本机 provider 装配（env 文件里的端口/model_path/display）产出：

    /etc/iraf-agent/
    ├── scenes/             # 镜像（只保留本机承载的场景）
    ├── providers/          # 镜像（hosts 只保留本机）
    └── manifest.yaml       # 派生事实：unit/env_file/port/display/realtime/scene_dir

这样 supervisor（C2）只需读 manifest.yaml，不必理解板卡的场景语义。

用法
----
    python3 scripts/emit_host_manifest.py \
        --src /tmp/reg-src --out /etc/iraf-agent --host-id x86-1 \
        --env-root ~/.config/iraf
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

try:
    import yaml
except Exception:  # pragma: no cover
    print("需要 PyYAML", file=sys.stderr)
    sys.exit(3)

DIGEST_RE = re.compile(r"^digest:[ \t]*sha256:[0-9a-f]{64}[ \t]*\r?\n?", re.MULTILINE)


def parse_env(path: Path) -> dict:
    """解析 systemd EnvironmentFile（KEY=VALUE，支持引号包裹的 JSON）。"""
    out = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        v = v.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in ("'", '"'):
            v = v[1:-1]
        out[k.strip()] = v
    return out


def backend_config(env: dict) -> dict:
    try:
        return json.loads(env.get("IRAF_BACKEND_CONFIG") or "{}")
    except Exception:
        return {}


def find_env_by_port(env_root: Path, port: str):
    for f in sorted(env_root.glob("*.env")):
        env = parse_env(f)
        if str(env.get("IRAF_PROVIDER_PORT") or "") == port:
            return f, env
    return None, {}


def seal(path: Path) -> None:
    """重算 digest（镜像文件被重新序列化过，原签已失效）。"""
    body = DIGEST_RE.sub("", path.read_text(encoding="utf-8"), count=1)
    if not body.endswith("\n"):
        body += "\n"
    digest = "sha256:" + hashlib.sha256(body.encode("utf-8")).hexdigest()
    path.write_text(body + "digest: %s\n" % digest, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="派生 x86 主机 manifest")
    ap.add_argument("--src", default="/tmp/reg-src", help="注册表副本根（scenes/ providers/）")
    ap.add_argument("--out", default="/etc/iraf-agent")
    ap.add_argument("--host-id", default="x86-1")
    ap.add_argument("--env-root", default=str(Path.home() / ".config/iraf"))
    ap.add_argument("--render-hz", type=int, default=10)
    args = ap.parse_args(argv)

    src, out = Path(args.src), Path(args.out)
    env_root = Path(args.env_root).expanduser()
    if not (src / "scenes").is_dir() or not (src / "providers").is_dir():
        print("源注册表不完整：%s" % src, file=sys.stderr)
        return 3

    scenes = {}
    for p in sorted((src / "scenes").glob("*.yaml")):
        scenes[p.stem] = yaml.safe_load(p.read_text(encoding="utf-8")) or {}

    kept = {}
    for p in sorted((src / "providers").glob("*.yaml")):
        data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        hosts = [h for h in (data.get("hosts") or []) if str(h.get("id")) == args.host_id]
        if not hosts:
            continue
        kept[p.name] = (dict(data, hosts=hosts), hosts[0])

    if out.exists():
        shutil.rmtree(out)
    (out / "scenes").mkdir(parents=True)
    (out / "providers").mkdir(parents=True)

    entries = []
    for name, (data, host) in kept.items():
        (out / "providers" / name).write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
        scene_id = str(data.get("scene_id") or "")
        if scene_id in scenes:
            (out / "scenes" / (scene_id + ".yaml")).write_text(
                yaml.safe_dump(scenes[scene_id], allow_unicode=True, sort_keys=False),
                encoding="utf-8")
        port = str(host.get("endpoint") or "").rpartition(":")[2]
        env_file, env = find_env_by_port(env_root, port)
        cfg = backend_config(env)
        unit_name = (scenes.get(scene_id) or {}).get("switch_name") or scene_id
        entries.append({
            "provider_id": data.get("provider_id"),
            "scene_id": scene_id,
            "switch_name": (scenes.get(scene_id) or {}).get("switch_name") or "",
            "aliases": [str(a) for a in ((scenes.get(scene_id) or {}).get("aliases") or [])],
            "endpoint": host.get("endpoint"),
            "port": port,
            "unit": "iraf-provider@%s.service" % unit_name,
            "env_file": str(env_file) if env_file else "",
            "display": env.get("IRAF_PROVIDER_DISPLAY_MODE") or "window",
            "render_hz": args.render_hz,
            "realtime": bool(cfg.get("realtime")),
            "scene_dir": cfg.get("model_path") or "",
            "present": bool(env_file),
        })

    for sub in ("scenes", "providers"):
        for f in sorted((out / sub).glob("*.yaml")):
            seal(f)

    manifest = {
        "schema": "iraf.host-manifest/v1",
        "host_id": args.host_id,
        "generated_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "source": str(src),
        "providers": entries,
    }
    (out / "manifest.yaml").write_text(
        yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False), encoding="utf-8")

    missing = [e["provider_id"] for e in entries if not e["present"]]
    print("manifest: %d providers -> %s" % (len(entries), out))
    for e in entries:
        print("  %-22s %-16s port=%-5s unit=%-28s realtime=%s"
              % (e["provider_id"], e["scene_id"], e["port"], e["unit"], e["realtime"]))
    if missing:
        print("WARN 未找到 env 的 provider：%s" % missing, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
