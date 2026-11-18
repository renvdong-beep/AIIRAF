# -*- coding: utf-8 -*-
"""联合场景编排 Skill Provider：只产出声明工件（scene/baseline/profile/registry），不驱动机器人。"""
from __future__ import annotations

from pathlib import Path

from iraf_adapters.mujoco import joint_compose
from iraf_core.profile import load_robot_profile


class JointSceneComposerError(ValueError):
    pass


class JointSceneComposerProvider:
    """工程类 Provider：与运动 Provider 隔离，无物理动作、无租约校验。"""

    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease=None):
        scene_id = str(inputs.get("scene_id") or "").strip()
        robots = inputs.get("robots")
        if not scene_id or not isinstance(robots, list) or not robots:
            raise JointSceneComposerError("scene_id 与 robots[] 必填")
        root = Path(inputs.get("root") or ".")
        base = root / str(inputs.get("base") or "scenes/handoff_lab")
        descriptors = [joint_compose.load_descriptor(base / "scene.yaml", str(r)) for r in robots]
        profile = joint_compose.compose_profile(descriptors, scene_id=scene_id)
        composed = joint_compose.compose_scene(base / "scene.yaml", base / "baseline.yaml",
                                               scene_id=scene_id, robot_ids=robots)
        out_dir = root / "scenes" / scene_id
        files = joint_compose.write_outputs(root, scene_id=scene_id, composed=composed,
                                            profile=profile, out_dir=out_dir)
        return {
            "status": "SUCCEEDED",
            "scene_id": scene_id,
            "robots": robots,
            "joints": len(profile["spec"]["joints"]),
            "capabilities": profile["spec"]["capabilities"],
            "files": files,
            "note": "只生成声明；MJCF 合成由 scripts/build_scene.py 完成（主本体选四足）",
        }
