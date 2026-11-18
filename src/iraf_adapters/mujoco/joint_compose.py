#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""联合场景编排器（P2 起步）：从已有单机场景组合出联合场景声明。

边界（docs/plans/2026-11-18-joint-scene-composer.md）：
- 只生成**声明**（scene.yaml / baseline.yaml / 联合 Profile），不生成运行期代码；
- MJCF 合成交给既有 `scripts/build_scene.py`（复用已验证的 scene_builder），本模块不重写；
- 不复制机型数字：关节/限位/能力一律从各单机 Profile 读取后取并集；
- 缺声明、重名、能力不一致一律显式失败。
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, List

import yaml

from iraf_core.profile import load_robot_profile


class ComposeError(ValueError):
    pass


def _load(path: Path) -> dict:
    if not path.is_file():
        raise ComposeError("声明文件不存在：%s" % path)
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def load_descriptor(base_scene: Path, robot_id: str) -> dict:
    """从既有联合/单机场景里取一个本体的声明条目（id/profile/placement）。"""
    scene = _load(base_scene)
    for robot in scene.get("robots") or []:
        if str(robot.get("id")) == robot_id:
            return dict(robot)
    raise ComposeError("场景 %s 里没有本体 %r" % (base_scene, robot_id))


def compose_profile(descriptors: Iterable[dict], *, scene_id: str) -> Dict:
    """把各单机 Profile 的 joints/limits/capabilities 取并集，生成联合 Profile。"""
    joints: List[str] = []
    limits: Dict[str, list] = {}
    caps: set = set()
    resources: Dict[str, str] = {}
    for desc in descriptors:
        ref = str(desc.get("profile") or "")
        if not ref:
            raise ComposeError("本体 %r 缺 profile 声明" % desc.get("id"))
        profile = load_robot_profile(ref)
        for joint in profile.joints:
            if joint in limits:
                raise ComposeError("关节重名：%s（本体 %s）" % (joint, desc.get("id")))
            joints.append(joint)
            limits[joint] = list(profile.joint_limits[joint])
        caps |= set(profile.capabilities)
        resources[str(desc.get("id"))] = "quadruped" if str(desc.get("id")) in (
            "unitree_go2", "go2") else "arm"
    return {
        "apiVersion": "iraf.intewell.io/v1",
        "kind": "RobotProfile",
        "metadata": {"name": scene_id, "version": "0.1.0"},
        "spec": {
            "simulation": True,
            "verification": "development",
            "control_frequency_hz": 100,
            "joints": joints,
            "joint_limits": limits,
            "capabilities": sorted(caps),
            "joint": {"resources": resources},
        },
    }


def compose_scene(base_scene: Path, base_baseline: Path, *, scene_id: str,
                  robot_ids: Iterable[str], placements: Dict[str, dict] | None = None) -> Dict[str, dict]:
    """生成联合场景的 scene.yaml / baseline.yaml（只改声明，不碰 MJCF）。"""
    scene = _load(base_scene)
    baseline = _load(base_baseline)
    wanted = [str(x) for x in robot_ids]
    robots = [r for r in (scene.get("robots") or []) if str(r.get("id")) in wanted]
    missing = set(wanted) - {str(r.get("id")) for r in robots}
    if missing:
        raise ComposeError("场景里缺少本体：%s" % sorted(missing))
    if len(robots) != len(wanted):
        raise ComposeError("本体 id 重复或过滤异常")
    placements = placements or {}
    for robot in robots:
        placement = placements.get(str(robot["id"]))
        if placement:
            robot["placement"] = placement
    scene["id"] = scene_id
    scene["robots"] = robots
    model = scene.setdefault("model", {})
    model["output"] = "build/scenes/%s/%s.xml" % (scene_id, scene_id)
    if "joint_output" in model:
        model["joint_output"] = "build/scenes/%s/%s_joint.xml" % (scene_id, scene_id)
    scene["description"] = "联合场景 %s（composer 生成）：%s" % (scene_id, ",".join(wanted))

    joint_bindings = dict((baseline.get("robots_joint") or {}))
    filtered = {k: v for k, v in joint_bindings.items() if k in wanted}
    baseline["robots_joint"] = filtered
    single = {k: v for k, v in (baseline.get("robots") or {}).items() if k in wanted}
    baseline["robots"] = single
    return {"scene": scene, "baseline": baseline}


def write_outputs(root: Path, *, scene_id: str, composed: Dict[str, dict],
                  profile: Dict, out_dir: Path) -> List[str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    files = []
    (out_dir / "scene.yaml").write_text(
        yaml.safe_dump(composed["scene"], allow_unicode=True, sort_keys=False), encoding="utf-8")
    files.append(str(out_dir / "scene.yaml"))
    (out_dir / "baseline.yaml").write_text(
        yaml.safe_dump(composed["baseline"], allow_unicode=True, sort_keys=False), encoding="utf-8")
    files.append(str(out_dir / "baseline.yaml"))
    profile_path = root / "profiles" / ("%s.yaml" % scene_id)
    profile_path.write_text(
        yaml.safe_dump(profile, allow_unicode=True, sort_keys=False), encoding="utf-8")
    files.append(str(profile_path))
    return files
