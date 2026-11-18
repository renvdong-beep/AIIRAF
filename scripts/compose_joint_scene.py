#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""联合场景编排 CLI：`python3 scripts/compose_joint_scene.py --robots piper,unitree_go2 ...`

只生成声明（scene.yaml / baseline.yaml / 联合 Profile）；MJCF 合成复用 `scripts/build_scene.py`。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.mujoco.joint_compose import (  # noqa: E402
    ComposeError, compose_profile, compose_scene, load_descriptor, write_outputs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="scenes/handoff_lab", help="基础场景目录（含 scene.yaml/baseline.yaml）")
    ap.add_argument("--robots", required=True, help="逗号分隔的本体 id（必须是基础场景里已有的）")
    ap.add_argument("--scene-id", required=True, help="新联合场景 id")
    ap.add_argument("--out", default="", help="输出目录（默认 scenes/<scene-id>）")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不写文件")
    args = ap.parse_args()

    base = ROOT / args.base
    robot_ids = [x.strip() for x in args.robots.split(",") if x.strip()]
    try:
        descriptors = [load_descriptor(base / "scene.yaml", rid) for rid in robot_ids]
        profile = compose_profile(descriptors, scene_id=args.scene_id)
        composed = compose_scene(base / "scene.yaml", base / "baseline.yaml",
                                 scene_id=args.scene_id, robot_ids=robot_ids)
    except ComposeError as exc:
        print("[compose][错误] %s" % exc)
        return 2

    out_dir = Path(args.out) if args.out else ROOT / "scenes" / args.scene_id
    plan = {
        "scene_id": args.scene_id,
        "robots": robot_ids,
        "out_dir": str(out_dir),
        "profile": "profiles/%s.yaml" % args.scene_id,
        "joints": len(profile["spec"]["joints"]),
        "capabilities": profile["spec"]["capabilities"],
        "resources": profile["spec"]["joint"]["resources"],
    }
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    if args.dry_run:
        print("[compose] dry-run：未写文件")
        return 0
    files = write_outputs(ROOT, scene_id=args.scene_id, composed=composed,
                          profile=profile, out_dir=out_dir)
    print("[compose] 已生成：")
    for f in files:
        print("  " + f)
    print("[compose] 下一步：python3 scripts/build_scene.py --scene %s --robot %s --attach %s"
          % (out_dir.relative_to(ROOT), robot_ids[0], " --attach ".join(robot_ids[1:])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
