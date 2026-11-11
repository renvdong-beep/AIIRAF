#!/usr/bin/env python3
"""生成交付用的**自包含**仿真场景资产（deploy/sdk/prebuilt/scenes/<stem>/）。

为什么需要（2026-11-11 同事反馈实证）：SDK 的 runtime bundle 不含 `build/`（构建产物），
而 `sim-x86` 角色声明里的 `sim_scene` 指向 `build/models/*.xml`。干净机器上装 SDK 时
安装器会报"声明的仿真场景缺文件"，ur5 / handoff_lab 装完不可用。

本脚本把已构建好的场景拍平成自包含包：
  · 所有 `file="…"` 资产（STL/obj/贴图）复制到 `<stem>_assets/`（按内容去重，避免同名覆盖）；
  · XML 内引用改写为 `<stem>_assets/<name>`，并删除 meshdir/texturedir（引用已相对模型文件）；
  · 同目录保留同名 `.json` 旁挂报告（安装器据此生成 IRAF_BACKEND_CONFIG）。

每场景独立资产目录，避免多场景装进同一 <version>/models 时同名资产互相覆盖（实测踩过）。

用法（仓库根）：
    python3 deploy/sdk/gen_prebuilt_scenes.py
    python3 deploy/sdk/gen_prebuilt_scenes.py --scene build/models/ur5-pick-scene.xml
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
OUT_ROOT = REPO / "deploy" / "sdk" / "prebuilt" / "scenes"
DEFAULT_SCENES = [
    "build/models/piper-pick-scene.xml",
    "build/models/ur5-pick-scene.xml",
    "build/scenes/handoff_lab/handoff_lab_joint.xml",
]


def _sha1(path: pathlib.Path) -> str:
    return hashlib.sha1(path.read_bytes()).hexdigest()


def build(scene_rel: str) -> tuple[int, int]:
    xml = (REPO / scene_rel).resolve()
    if not xml.is_file():
        print("缺场景：%s（先构建：scripts/build_* 或 build_scene.py）" % xml, file=sys.stderr)
        raise SystemExit(2)
    report = xml.with_suffix(".json")
    if not report.is_file():
        print("缺旁挂报告：%s" % report, file=sys.stderr)
        raise SystemExit(2)

    stem = xml.stem
    text = xml.read_text(encoding="utf-8")
    bases = [xml.parent]
    for key in ("meshdir", "texturedir"):
        m = re.search(r'%s="([^"]*)"' % key, text)
        if m and m.group(1):
            bases.append((xml.parent / m.group(1)).resolve())

    out = OUT_ROOT / stem
    if out.exists():
        shutil.rmtree(out)
    asset_dir = stem + "_assets"
    (out / asset_dir).mkdir(parents=True)

    by_hash: dict[str, str] = {}
    mapping: dict[str, str] = {}
    for ref in sorted(set(re.findall(r'file="([^"]+)"', text))):
        src = next((b / ref for b in bases if (b / ref).is_file()), None)
        if src is None:
            print("无法解析资产引用：%s（场景 %s）" % (ref, xml), file=sys.stderr)
            raise SystemExit(3)
        src = src.resolve()
        digest = _sha1(src)
        if digest in by_hash:
            final = by_hash[digest]
        else:
            final, n = src.name, 1
            while (out / asset_dir / final).exists():
                final = "%s_%d_%s" % (stem, n, src.name)
                n += 1
            shutil.copy2(src, out / asset_dir / final)
            by_hash[digest] = final
        mapping[ref] = asset_dir + "/" + final

    for ref, final in mapping.items():
        text = text.replace('file="%s"' % ref, 'file="%s"' % final)
    text = re.sub(r'\s*(meshdir|texturedir)="[^"]*"', "", text)
    (out / (stem + ".xml")).write_text(text, encoding="utf-8")
    shutil.copy2(report, out / (stem + ".json"))
    size = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    print("%-22s refs=%-3d assets=%-3d %.2fMB -> %s"
          % (stem, len(mapping), len(mapping), size / 1e6, out.relative_to(REPO)))
    return len(mapping), int(size)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", action="append", default=None,
                        help="场景 XML（仓库相对路径，可重复；默认 piper/ur5/handoff 三套）")
    args = parser.parse_args()
    scenes = args.scene or DEFAULT_SCENES
    total_assets = total_bytes = 0
    for rel in scenes:
        a, s = build(rel)
        total_assets += a
        total_bytes += s
    print("合计：%d 个资产，%.2fMB" % (total_assets, total_bytes / 1e6))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())