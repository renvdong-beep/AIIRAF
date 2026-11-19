#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新机器人接入（标准流程的最小机械部分）：模型事实 → Profile → 校验报告 → 适配器骨架。

边界：
- 只做**可自动化的声明部分**；能力实现（stand/locomote/pick）仍必须由人/agent 写进 Backend，
  并由单机验收证明。`validation_report` 会显式写 `runnable=false`，不允许把“生成了 Profile”
  冒充“机器人可用”。
- 不覆盖已有文件（除非 allow_overwrite）；失败显式。
"""
from __future__ import annotations

import json
from pathlib import Path

import yaml


class OnboardError(ValueError):
    pass


def inspect_model(model_path: str | Path) -> dict:
    import mujoco

    path = Path(model_path)
    if not path.is_file():
        raise OnboardError("模型不存在：%s" % path)
    m = mujoco.MjModel.from_xml_path(str(path))
    joints = []
    skipped = []
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        if not name or int(m.jnt_type[i]) == 0:  # free joint
            skipped.append(name or "<unnamed>")
            continue
        joints.append({"name": name,
                       "limit": [round(float(m.jnt_range[i][0]), 6),
                                 round(float(m.jnt_range[i][1]), 6)]})
    actuators = []
    for i in range(m.nu):
        actuators.append({"index": i,
                          "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i),
                          "ctrlrange": [round(float(m.actuator_ctrlrange[i][0]), 6),
                                        round(float(m.actuator_ctrlrange[i][1]), 6)]})
    bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]
    return {"model": str(path), "nq": int(m.nq), "nv": int(m.nv), "nu": int(m.nu),
            "joints": joints, "skipped_joints": skipped,
            "actuators": actuators, "bodies": [b for b in bodies if b]}


def build_profile(robot_id: str, facts: dict, capabilities=None) -> dict:
    joints = [j["name"] for j in facts["joints"]]
    limits = {j["name"]: j["limit"] for j in facts["joints"]}
    return {
        "apiVersion": "iraf.intewell.io/v1",
        "kind": "RobotProfile",
        "metadata": {"name": robot_id, "version": "0.1.0"},
        "spec": {
            "simulation": True,
            "verification": "development",
            "control_frequency_hz": 100,
            "joints": joints,
            "joint_limits": limits,
            "capabilities": list(capabilities or []),
        },
    }


def onboard(root: str | Path, robot_id: str, model_path: str | Path, *,
            capabilities=None, dry_run: bool = True, allow_overwrite: bool = False,
            output_dir: str = "") -> dict:
    root = Path(root)
    model = Path(model_path)
    if not model.is_absolute():
        model = root / model
    facts = inspect_model(model)
    profile = build_profile(robot_id, facts, capabilities)
    report = {
        "schema": "iraf.robot-onboarding/v1",
        "robot_id": robot_id,
        "model": facts["model"],
        "facts": {"nq": facts["nq"], "nv": facts["nv"], "nu": facts["nu"],
                  "joints": len(facts["joints"]), "actuators": len(facts["actuators"])},
        "declared_capabilities": list(capabilities or []),
        # 能力是否在 Backend 上实现，必须由后续适配器 + 单机验收证明；此处不得冒充
        "implemented_capabilities": [],
        "runnable": False,
        "next_steps": [
            "实现/补齐 Backend 能力（stand/locomote/move_joint/pick_object...）",
            "生成单机 scene.yaml + 机型声明 + Provider",
            "跑单机验收（stand/locomote/move/pick）并留下 report",
            "通过后才允许 iraf_compose_joint_scene 进联合世界",
        ],
    }
    out = root / (output_dir or ("build/onboarding/%s" % robot_id))
    artifacts = []
    if not dry_run:
        out.mkdir(parents=True, exist_ok=True)
        profile_path = root / "profiles" / ("%s.yaml" % robot_id)
        validation_path = out / "validation.json"
        skeleton_path = out / "backend_skeleton.py"
        for path in (profile_path, validation_path, skeleton_path):
            if path.exists() and not allow_overwrite:
                raise OnboardError("拒绝覆盖已存在文件：%s（allow_overwrite=false）" % path)
        profile_path.write_text(yaml.safe_dump(profile, allow_unicode=True, sort_keys=False),
                                encoding="utf-8")
        validation_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        skeleton_path.write_text(
            '"""%s Backend 骨架（由 iraf_onboard_robot 生成；必须补实现并验收）。"""\n\n'
            "class %sBackend:\n"
            "    IMPLEMENTED_CAPABILITIES = frozenset()  # 先补能力，再写进 Profile\n\n"
            "    @classmethod\n"
            "    def from_config(cls, config, profile, authority):\n"
            "        raise NotImplementedError('按声明实现并跑单机验收')\n"
            % (robot_id, "".join(part.capitalize() for part in robot_id.split("_")))
            , encoding="utf-8")
        artifacts = [str(profile_path), str(validation_path), str(skeleton_path)]
    return {"status": "SUCCEEDED", "robot_id": robot_id, "profile": profile,
            "validation": report, "artifacts": artifacts, "dry_run": bool(dry_run)}
