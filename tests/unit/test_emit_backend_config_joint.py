# -*- coding: utf-8 -*-
"""联合世界 backend config：name_map 必须从 manipulation 段搬运（2026-11-06）。"""
from __future__ import annotations

import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("emit_backend_config",
                                               REPO / "scripts" / "emit_backend_config.py")
emitter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(emitter)


def test_joint_name_map_is_emitted_from_manipulation_block():
    scene = {
        "target_id": "box_01",
        "gripper": {"wrist_body": "piper_link6"},
        "vision": None,
        "manipulation": {"name_map": {"joint1": "piper_joint1"}},
    }
    config = emitter.build_config(scene, "/tmp/handoff_lab_joint.xml")
    assert config["name_map"] == {"joint1": "piper_joint1"}


def test_single_robot_without_name_map_stays_none():
    scene = {"target_id": "box_01", "gripper": {}, "vision": None}
    config = emitter.build_config(scene, "/tmp/piper-pick-scene.xml")
    assert config["name_map"] is None
