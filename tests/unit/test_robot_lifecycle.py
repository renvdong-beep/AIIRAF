# -*- coding: utf-8 -*-
import json
from pathlib import Path

import pytest

from iraf_adapters.mujoco import robot_lifecycle as rl
from iraf_skills.engineering.robot_onboard import RobotOnboardProvider


def test_scaffold_dry_run_has_all_patches():
    out = rl.scaffold_capability("demo_bot", "gripper", root=".", dry_run=True)
    assert out["status"] == "SUCCEEDED"
    for name in ("backend_method.py", "provider.py", "skill.yaml",
                 "profile_patch.yaml", "scene_patch.yaml", "policy_patch.yaml"):
        assert name in out["files"]


def test_scaffold_unknown_capability_fails():
    with pytest.raises(rl.LifecycleError):
        rl.scaffold_capability("demo_bot", "teleport", root=".", dry_run=True)


def test_build_scene_rejects_undeclared_capability(tmp_path):
    profile = tmp_path / "p.yaml"
    profile.write_text("kind: RobotProfile\nmetadata: {name: x}\nspec: {simulation: true,"
                       " control_frequency_hz: 100, joints: [j1], joint_limits: {j1: [-1, 1]},"
                       " capabilities: [move_joint]}\n", encoding="utf-8")
    with pytest.raises(rl.LifecycleError):
        rl.build_scene("x", "x-scene", str(profile), "127.0.0.1:1",
                       ["pick_object"], root=str(tmp_path), dry_run=True)


def test_build_scene_writes_files(tmp_path):
    profile = tmp_path / "p.yaml"
    profile.write_text("kind: RobotProfile\nmetadata: {name: x}\nspec: {simulation: true,"
                       " control_frequency_hz: 100, joints: [j1], joint_limits: {j1: [-1, 1]},"
                       " capabilities: [move_joint]}\n", encoding="utf-8")
    out = rl.build_scene("x", "x-scene", str(profile), "127.0.0.1:50052",
                         ["move_joint"], root=str(tmp_path), dry_run=False)
    assert out["status"] == "SUCCEEDED"
    assert (tmp_path / "config/scenes/x-scene.yaml").is_file()
    assert (tmp_path / "config/providers/iraf-x-scene.yaml").is_file()


def test_register_requires_passed_report(tmp_path):
    scene = tmp_path / "s.yaml"
    provider = tmp_path / "p.yaml"
    scene.write_text("schema: iraf.scene/v1\n", encoding="utf-8")
    provider.write_text("schema: iraf.provider/v1\n", encoding="utf-8")
    report = tmp_path / "r.json"
    report.write_text(json.dumps({"passed": False}), encoding="utf-8")
    with pytest.raises(rl.LifecycleError):
        rl.register(str(scene), str(provider), root=str(tmp_path),
                    acceptance_report=str(report), dry_run=True)


def test_skill_status_stage_smoke():
    out = RobotOnboardProvider(None, None).execute(
        {"robot_id": "franka_2f85", "stage": "status", "root": "."})
    assert out["status"] == "SUCCEEDED"
    assert isinstance(out["acceptance"], list)


def test_acceptance_all_dry_plan():
    out = rl.acceptance_all("franka_2f85", profile="profiles/franka_2f85_mujoco.yaml",
                            root=".", dry_run=True)
    assert out["status"] == "SUCCEEDED"
    assert "gripper" in out["capabilities"]


def test_join_world_dry_without_gate():
    out = rl.join_world("demo-joint", "piper,unitree_go2", base="scenes/handoff_lab",
                        root=".", dry_run=True, require_runnable=False)
    assert out["status"] == "SUCCEEDED"
    assert out["joints"] > 0


def test_join_world_blocks_unaccepted_robot():
    with pytest.raises(rl.LifecycleError):
        rl.join_world("demo-joint", "piper", base="scenes/handoff_lab",
                      root=".", dry_run=True, require_runnable=True)

