#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义入口（阶段 1/2/5 起步）：机器人 + 常用物品 → 联合场景 → 构建。

用法：
  python3 scripts/joint_semantic.py --robots piper,unitree_go2 --props "box@0.28,-0.28,0.03" \
      --scene-id demo_lab --build
  python3 scripts/joint_semantic.py --text "piper 和狗，桌上一个方块" --scene-id demo_lab --build
"""
from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from iraf_adapters.mujoco import joint_compose  # noqa: E402

ROBOT_ALIASES = {
    "piper": "piper", "派珀": "piper", "机械臂": "piper",
    "ur5": "ur5e", "ur5e": "ur5e", "ur": "ur5e", "优傲": "ur5e",
    "go2": "unitree_go2", "狗": "unitree_go2", "机器狗": "unitree_go2", "四足": "unitree_go2",
    "franka": "franka", "panda": "franka",
}
PROP_ALIASES = {
    "方块": "box", "盒子": "box", "box": "box",
    "小方块": "cube", "cube": "cube",
    "球": "ball", "ball": "ball",
    "圆柱": "cylinder", "cylinder": "cylinder",
    "杯子": "cup", "杯": "cup", "cup": "cup",
    "托盘": "tray", "tray": "tray",
    "按钮": "button", "button": "button",
}


class SemanticError(ValueError):
    pass


def load_props_library(root: Path) -> dict:
    path = root / "config/props/library.yaml"
    if not path.is_file():
        raise SemanticError("prop 库不存在: %s" % path)
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(doc.get("props") or {})


def make_prop(name: str, xyz, library: dict, index: int) -> dict:
    if name not in library:
        raise SemanticError("未知物品 %r（可用：%s）" % (name, sorted(library)))
    template = copy.deepcopy(library[name])
    prop_id = "box_01" if name == "box" and index == 0 else "%s_%02d" % (name, index)
    prop = {
        "id": prop_id,
        "body": prop_id,
        "kind": template.get("kind") or template["geometry"]["type"],
        "geometry": template["geometry"],
        "mass_kg": template.get("mass_kg", 0.05),
        "friction": template.get("friction", "1.0 0.02 0.001"),
        "rgba": template.get("rgba", [0.6, 0.6, 0.6, 1.0]),
        "pose": {"pos_m": [float(x) for x in xyz], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
        "note": template.get("note", ""),
    }
    return prop


def parse_props(specs, library: dict, default_xyz=(0.28, -0.28, 0.03)) -> list:
    props = []
    for index, spec in enumerate(specs or []):
        text = str(spec).strip()
        if not text:
            continue
        name, _, pos = text.partition("@")
        xyz = default_xyz
        if pos:
            parts = [p for p in pos.split(",") if p.strip()]
            if len(parts) != 3:
                raise SemanticError("物品位姿格式应为 name@x,y,z: %r" % text)
            xyz = tuple(float(p) for p in parts)
        canonical = PROP_ALIASES.get(name, name)
        props.append(make_prop(canonical, xyz, library, index))
    return props


def parse_text(text: str, library: dict):
    """受控词表解析：机器人别名 + 物品别名；位置用默认（显式 --props 优先）。"""
    robots, props = [], []
    for token, canonical in ROBOT_ALIASES.items():
        if token in text and canonical not in robots:
            robots.append(canonical)
    for token, canonical in PROP_ALIASES.items():
        if token in text:
            props.append(make_prop(canonical, (0.28, -0.28, 0.03), library, len(props)))
    if not robots:
        raise SemanticError("没有识别到机器人（可用：piper/ur5/狗/franka）")
    return robots, props


def write_stand_scenario(out_dir: Path, robot: str) -> Path:
    """生成最小可执行场景：stand → stop（进程内 MuJoCo，不需要 Provider/板卡）。"""
    scenario = {
        "schema_version": "iraf.scenario-catalog/v1",
        "scene": "scene.yaml",
        "simulation": True,
        "scenarios": {
            "stand_stop": {
                "description": "语义组合场景冒烟：四足站立保持 → 松力停机",
                "steps": [
                    {"id": "ss01_stand", "action": "stand", "robot": robot, "params": {},
                     "criteria": {"min_stable_hold_s": 3.0, "max_speed_m_s": 0.05,
                                  "timeout_s": 10.0}},
                    {"id": "ss02_stop", "action": "stop", "robot": robot, "params": {},
                     "criteria": {"min_stable_hold_s": 1.0, "max_speed_m_s": 0.05,
                                  "timeout_s": 5.0}},
                ],
                "faults": [],
            }
        },
    }
    path = out_dir / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario, allow_unicode=True, sort_keys=False), encoding="utf-8")
    readme = out_dir / "README.md"
    if not readme.exists():
        readme.write_text("# %s\n\n由 `scripts/joint_semantic.py` 生成的语义组合场景。\n" % out_dir.name,
                          encoding="utf-8")
    return path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", default="", help="自然语言（受控词表）")
    ap.add_argument("--robots", default="", help="逗号分隔的机器人 id")
    ap.add_argument("--props", action="append", default=[], help="物品，如 box@0.28,-0.28,0.03")
    ap.add_argument("--scene-id", required=True)
    ap.add_argument("--base", default="scenes/handoff_lab")
    ap.add_argument("--build", action="store_true", help="调用 build_scene 生成 MJCF")
    ap.add_argument("--register", action="store_true")
    ap.add_argument("--run", action="store_true", help="构建后用 scenario.py 在进程内跑 stand_stop")
    args = ap.parse_args()

    library = load_props_library(ROOT)
    try:
        if args.text:
            robots, text_props = parse_text(args.text, library)
            props = parse_props(args.props, library) if args.props else text_props
        else:
            robots = [x.strip() for x in args.robots.split(",") if x.strip()]
            props = parse_props(args.props, library)
        if not robots:
            raise SemanticError("机器人列表为空")

        base = ROOT / args.base
        descriptors = [joint_compose.load_descriptor(base / "scene.yaml", rid) for rid in robots]
        profile = joint_compose.compose_profile(descriptors, scene_id=args.scene_id)
        composed = joint_compose.compose_scene(
            base / "scene.yaml", base / "baseline.yaml", scene_id=args.scene_id,
            robot_ids=robots, props=props)
        files = joint_compose.write_outputs(
            ROOT, scene_id=args.scene_id, composed=composed, profile=profile,
            out_dir=ROOT / "scenes" / args.scene_id)
        if args.register:
            registry = joint_compose.build_registry_scene(args.scene_id, descriptors)
            reg = ROOT / "config/scenes" / ("%s.yaml" % args.scene_id.replace("_", "-"))
            reg.parent.mkdir(parents=True, exist_ok=True)
            reg.write_text(yaml.safe_dump(registry, allow_unicode=True, sort_keys=False), encoding="utf-8")
            files.append(str(reg))
        print(json.dumps({"status": "SUCCEEDED", "scene_id": args.scene_id, "robots": robots,
                          "props": [p["id"] for p in props], "files": files}, ensure_ascii=False, indent=2))
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"status": "FAILED", "error": "%s: %s" % (type(exc).__name__, exc)},
                         ensure_ascii=False))
        return 2

    if args.build:
        primary = next((r for r in robots if r in ("unitree_go2", "go2")), robots[0])
        attach = [r for r in robots if r != primary]
        cmd = [sys.executable, str(ROOT / "scripts/build_scene.py"),
               "--scene", str(ROOT / "scenes" / args.scene_id), "--robot", primary]
        for r in attach:
            cmd += ["--attach", r]
        cmd.append("--quiet")
        print("[build] " + " ".join(cmd))
        rc = subprocess.call(cmd, cwd=str(ROOT))
        print("[build] rc=%d" % rc)
        if rc != 0:
            return rc
    if args.run:
        go2 = next((r for r in robots if r in ("unitree_go2", "go2")), None)
        if not go2:
            print(json.dumps({"status": "FAILED", "error": "run 需要 unitree_go2（stand_stop）"},
                             ensure_ascii=False))
            return 2
        out_dir = ROOT / "scenes" / args.scene_id
        write_stand_scenario(out_dir, go2)
        run_cmd = [sys.executable, str(ROOT / "scripts/scenario.py"), "run",
                   "--scene", str(out_dir), "--scenario", "stand_stop", "--display", "none"]
        print("[run] " + " ".join(run_cmd))
        return subprocess.call(run_cmd, cwd=str(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
