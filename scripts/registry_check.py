#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SceneRegistry / ProviderRegistry 校验器（阶段 A · A1）

用途
----
    python3 scripts/registry_check.py --root /etc/iraf --profiles-root /opt/iraf-sdk/current/profiles
    python3 scripts/registry_check.py --root /etc/iraf --seal      # 重新计算并写回 digest
    python3 scripts/registry_check.py --root /etc/iraf --json      # 机器可读输出

退出码：0 全部通过 / 2 校验失败 / 3 目录或文件缺失

设计要点（对应 plans/iraf-single-controlplane/01-contracts.md）
--------------------------------------------------------------
* digest = sha256(文件字节，**去掉 digest 行**)，行序与空白保持原样；`--seal` 与校验用同一算法。
* 交叉引用：scene.provider_id 必须存在；provider.scene_id 必须存在且与 scene 指向一致。
* 主机白名单自校验：provider.hosts[].endpoint 的 host 必须 ⊆ allowed_hosts。
* 能力校验：resource.capabilities ⊆ 该 profile 声明的 capabilities（从 profile YAML 递归收集）。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

try:
    import yaml
except Exception:  # pragma: no cover
    print("需要 PyYAML：python3 -m pip install --user pyyaml", file=sys.stderr)
    sys.exit(3)

SCENE_SCHEMA = "iraf.scene/v1"
PROVIDER_SCHEMA = "iraf.provider/v1"
ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]{1,48}$")
DIGEST_RE = re.compile(r"^digest:[ \t]*sha256:[0-9a-f]{64}[ \t]*\r?\n?", re.MULTILINE)


# ---------------------------------------------------------------- digest ---
def strip_digest(raw: bytes) -> bytes:
    """去掉 digest 行后的字节（用于计算/核对摘要）。"""
    text = raw.decode("utf-8")
    return DIGEST_RE.sub("", text, count=1).encode("utf-8")


def compute_digest(raw: bytes) -> str:
    return "sha256:" + hashlib.sha256(strip_digest(raw)).hexdigest()


def seal_file(path: Path) -> str:
    """把 digest 行写到文件末尾（保留其余内容与顺序），返回新摘要。"""
    raw = path.read_bytes()
    digest = compute_digest(raw)
    text = raw.decode("utf-8")
    body = DIGEST_RE.sub("", text, count=1)
    if not body.endswith("\n"):
        body += "\n"
    path.write_text(body + "digest: %s\n" % digest, encoding="utf-8")
    return digest


# ------------------------------------------------------------- profiles ---
def collect_capabilities(node, acc=None):
    """递归收集 profile YAML 里所有 capabilities 列表的并集。"""
    acc = acc if acc is not None else set()
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "capabilities" and isinstance(value, list):
                acc.update(str(v) for v in value)
            else:
                collect_capabilities(value, acc)
    elif isinstance(node, list):
        for item in node:
            collect_capabilities(item, acc)
    return acc


def load_profile_caps(profiles_root: Path, rel: str, cache: dict) -> set:
    """在 profiles_root 下解析 profile 路径：支持 `profiles/x.yaml` 与绝对/裸文件名。"""
    if rel in cache:
        return cache[rel]
    p = Path(rel)
    candidates = [p] if p.is_absolute() else [
        profiles_root / p,             # 相对根（含 profiles/ 前缀时）
        profiles_root / p.name,        # 裸文件名（最常见：profiles_root 已指向 profiles/）
        profiles_root / "profiles" / p.name,
    ]
    caps = set()
    for cand in candidates:
        if cand.is_file():
            data = yaml.safe_load(cand.read_text(encoding="utf-8")) or {}
            caps = collect_capabilities(data)
            break
    cache[rel] = caps
    return caps


# ----------------------------------------------------------------- check ---
def check(root: Path, profiles_root: Path, errs: list, warns: list, stats: dict) -> None:
    scenes_dir = root / "scenes"
    providers_dir = root / "providers"
    if not scenes_dir.is_dir():
        errs.append("缺少目录：%s" % scenes_dir)
    if not providers_dir.is_dir():
        errs.append("缺少目录：%s" % providers_dir)
    if errs:
        return

    scenes = {}
    for path in sorted(scenes_dir.glob("*.yaml")):
        raw = path.read_bytes()
        rel = path.name
        try:
            data = yaml.safe_load(raw.decode("utf-8")) or {}
        except Exception as exc:
            errs.append("%s: YAML 解析失败：%s" % (rel, exc))
            continue
        if data.get("schema") != SCENE_SCHEMA:
            errs.append("%s: schema 必须是 %s" % (rel, SCENE_SCHEMA))
            continue
        sid = str(data.get("scene_id") or "")
        if not ID_RE.match(sid):
            errs.append("%s: scene_id 非法或缺失：%r" % (rel, sid))
        if path.stem != sid:
            errs.append("%s: 文件名必须等于 scene_id（当前 %r）" % (rel, sid))
        if data.get("simulation") is not True:
            errs.append("%s: 必须显式声明 simulation: true" % rel)
        mc = data.get("max_concurrency")
        if not isinstance(mc, int) or mc < 1:
            errs.append("%s: max_concurrency 必须是 ≥1 的整数" % rel)
        ttl = data.get("lease_ttl_s", 60)
        if not isinstance(ttl, int) or ttl <= 0:
            errs.append("%s: lease_ttl_s 必须是正整数" % rel)
        if not data.get("safety_policy"):
            errs.append("%s: 缺 safety_policy" % rel)
        resources = data.get("resources") or []
        if not isinstance(resources, list) or not resources:
            errs.append("%s: resources 必须是非空列表" % rel)
        seen = set()
        for res in resources:
            rid = str((res or {}).get("id") or "")
            if not ID_RE.match(rid):
                errs.append("%s: resource id 非法：%r" % (rel, rid))
            if rid in seen:
                errs.append("%s: resource id 重复：%s" % (rel, rid))
            seen.add(rid)
            prof = (res or {}).get("profile")
            caps = (res or {}).get("capabilities") or []
            if not isinstance(caps, list) or not caps:
                errs.append("%s[%s]: capabilities 必须是非空列表" % (rel, rid))
                continue
            if prof:
                declared = load_profile_caps(profiles_root, prof, stats.setdefault("_caps", {}))
                if declared:
                    extra = [c for c in caps if c not in declared]
                    if extra:
                        errs.append("%s[%s]: capabilities 超出 profile 声明：%s" % (rel, rid, extra))
                else:
                    warns.append("%s[%s]: profile 未读到 capabilities（%s），跳过子集校验" % (rel, rid, prof))
        digest = compute_digest(raw)
        declared_digest = str(data.get("digest") or "")
        if declared_digest != digest:
            errs.append("%s: digest 不匹配（期望 %s，实际 %r）；跑 --seal 重新生成"
                        % (rel, digest, declared_digest))
        scenes[sid] = data

    providers = {}
    for path in sorted(providers_dir.glob("*.yaml")):
        raw = path.read_bytes()
        rel = path.name
        try:
            data = yaml.safe_load(raw.decode("utf-8")) or {}
        except Exception as exc:
            errs.append("%s: YAML 解析失败：%s" % (rel, exc))
            continue
        if data.get("schema") != PROVIDER_SCHEMA:
            errs.append("%s: schema 必须是 %s" % (rel, PROVIDER_SCHEMA))
            continue
        pid = str(data.get("provider_id") or "")
        if not pid.startswith("iraf/") or not ID_RE.match(pid.split("/", 1)[1]):
            errs.append("%s: provider_id 必须形如 iraf/<slug>：%r" % (rel, pid))
        if path.stem != pid.replace("/", "-"):
            errs.append("%s: 文件名必须等于 provider_id 的 /→- 形式（期望 %s.yaml）"
                        % (rel, pid.replace("/", "-")))
        scene_id = str(data.get("scene_id") or "")
        if scene_id not in scenes:
            errs.append("%s: scene_id 不存在于 scenes/：%r" % (rel, scene_id))
        elif scenes[scene_id].get("provider_id") != pid:
            errs.append("%s: 与 scene 的 provider_id 不一致（scene 写的是 %r）"
                        % (rel, scenes[scene_id].get("provider_id")))
        allowed = [str(h) for h in (data.get("allowed_hosts") or [])]
        if not allowed:
            errs.append("%s: allowed_hosts 不能为空" % rel)
        hosts = data.get("hosts") or []
        if not isinstance(hosts, list) or not hosts:
            errs.append("%s: hosts 必须是非空列表" % rel)
        seen_id, seen_prio = set(), set()
        for host in hosts:
            hid = str((host or {}).get("id") or "")
            endpoint = str((host or {}).get("endpoint") or "")
            if not hid:
                errs.append("%s: host 缺 id" % rel)
            if hid in seen_id:
                errs.append("%s: host id 重复：%s" % (rel, hid))
            seen_id.add(hid)
            prio = (host or {}).get("priority")
            if not isinstance(prio, int):
                errs.append("%s[%s]: priority 必须是整数" % (rel, hid))
            elif prio in seen_prio:
                errs.append("%s: priority 重复：%s" % (rel, prio))
            else:
                seen_prio.add(prio)
            host_part, sep, port = endpoint.rpartition(":")
            if not sep or not port.isdigit():
                errs.append("%s[%s]: endpoint 必须形如 host:port：%r" % (rel, hid, endpoint))
            elif host_part not in allowed:
                errs.append("%s[%s]: endpoint 主机 %r 不在 allowed_hosts %s"
                            % (rel, hid, host_part, allowed))
        sched = data.get("schedule", "static")
        if sched not in ("static", "least-loaded"):
            errs.append("%s: schedule 只能是 static / least-loaded（当前 %r）" % (rel, sched))
        digest = compute_digest(raw)
        if str(data.get("digest") or "") != digest:
            errs.append("%s: digest 不匹配；跑 --seal 重新生成" % rel)
        providers[pid] = data

    for sid, data in scenes.items():
        pid = data.get("provider_id")
        if pid not in providers:
            errs.append("scene %s: provider_id 未定义：%r" % (sid, pid))

    stats.update({"scenes": len(scenes), "providers": len(providers)})


# ------------------------------------------------------------------ main ---
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="校验 SceneRegistry / ProviderRegistry")
    ap.add_argument("--root", default="/etc/iraf", help="注册表根（含 scenes/、providers/）")
    ap.add_argument("--profiles-root", default="/opt/iraf-sdk/current/profiles")
    ap.add_argument("--seal", action="store_true", help="重新计算并写回 digest")
    ap.add_argument("--json", action="store_true", help="JSON 输出")
    args = ap.parse_args(argv)

    root = Path(args.root)
    profiles_root = Path(args.profiles_root)
    if not root.is_dir():
        print("目录不存在：%s" % root, file=sys.stderr)
        return 3

    if args.seal:
        n = 0
        for sub in ("scenes", "providers"):
            for path in sorted((root / sub).glob("*.yaml")):
                seal_file(path)
                n += 1
        print("sealed %d files under %s" % (n, root))
        return 0

    errs, warns, stats = [], [], {}
    check(root, profiles_root, errs, warns, stats)

    if args.json:
        print(json.dumps({"ok": not errs, "stats": stats, "errors": errs, "warnings": warns},
                         ensure_ascii=False, indent=1))
    else:
        for w in warns:
            print("WARN %s" % w)
        for e in errs:
            print("FAIL %s" % e)
        if not errs:
            print("OK scenes=%d providers=%d" % (stats.get("scenes", 0), stats.get("providers", 0)))
    return 0 if not errs else 2


if __name__ == "__main__":
    sys.exit(main())
