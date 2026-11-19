# -*- coding: utf-8 -*-
"""engineering.robot_onboard：新机器人接入编排（声明/脚手架/场景/验收/注册/状态）。

所有阶段不直接驱动机器人；验收只在本地 Backend 仿真执行，未通过不得注册。
"""
from __future__ import annotations

from pathlib import Path

from iraf_adapters.mujoco.robot_onboard import onboard
from iraf_adapters.mujoco import robot_lifecycle as lifecycle


class RobotOnboardProvider:
    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease=None):
        root = Path(inputs.get("root") or ".")
        robot_id = str(inputs["robot_id"])
        stage = str(inputs.get("stage") or "onboard").strip().lower()
        dry = bool(inputs.get("dry_run", True))
        if stage == "onboard":
            return onboard(
                root, robot_id, str(inputs["model_path"]),
                capabilities=list(inputs.get("capabilities") or []),
                dry_run=dry,
                allow_overwrite=bool(inputs.get("allow_overwrite", False)),
                output_dir=str(inputs.get("output_dir") or ""),
            )
        if stage == "scaffold":
            return lifecycle.scaffold_capability(
                robot_id, str(inputs.get("capability") or ""),
                root=root, dry_run=dry,
                output_dir=str(inputs.get("output_dir") or ""))
        if stage == "scene":
            return lifecycle.build_scene(
                robot_id, str(inputs["scene_id"]), str(inputs["profile"]),
                str(inputs["provider_endpoint"]),
                capabilities=list(inputs.get("capabilities") or []),
                root=root, report_path=str(inputs.get("report_path") or ""),
                model_path=str(inputs.get("model_path") or ""),
                backend_entrypoint=str(inputs.get("backend_entrypoint") or ""),
                backend_config=dict(inputs.get("backend_config") or {}),
                control_url=str(inputs.get("control_url") or ""),
                safety_policy=str(inputs.get("safety_policy") or "profiles/safety/simulation_lab.yaml"),
                dry_run=dry, allow_overwrite=bool(inputs.get("allow_overwrite", False)))
        if stage == "acceptance":
            return lifecycle.acceptance(
                robot_id, str(inputs.get("capability") or ""),
                profile=str(inputs["profile"]), root=root,
                backend_entrypoint=str(inputs.get("backend_entrypoint") or ""),
                backend_config=dict(inputs.get("backend_config") or {}),
                params=dict(inputs.get("params") or {}), dry_run=dry,
                output_dir=str(inputs.get("output_dir") or ""))
        if stage == "acceptance_all":
            return lifecycle.acceptance_all(
                robot_id, profile=str(inputs["profile"]), root=root,
                params=dict(inputs.get("params") or {}),
                backend_entrypoint=str(inputs.get("backend_entrypoint") or ""),
                backend_config=dict(inputs.get("backend_config") or {}),
                dry_run=dry, output_dir=str(inputs.get("output_dir") or ""))
        if stage == "join_world":
            return lifecycle.join_world(
                str(inputs["scene_id"]), str(inputs["robots"]),
                base=str(inputs.get("base") or "scenes/handoff_lab"),
                root=root, dry_run=dry,
                require_runnable=bool(inputs.get("require_runnable", True)),
                register=bool(inputs.get("register", False)),
                registry_root=str(inputs.get("registry_root") or "/etc/iraf"),
                profiles_root=str(inputs.get("profiles_root") or "profiles"))
        if stage == "register":
            return lifecycle.register(
                str(inputs["scene_file"]), str(inputs["provider_file"]),
                root=root,
                registry_root=str(inputs.get("registry_root") or "/etc/iraf"),
                profiles_root=str(inputs.get("profiles_root") or "profiles"),
                acceptance_report=str(inputs["acceptance_report"]), dry_run=dry,
                allow_overwrite=bool(inputs.get("allow_overwrite", False)))
        if stage == "status":
            return lifecycle.status(
                robot_id, root=root,
                registry_root=str(inputs.get("registry_root") or "/etc/iraf"),
                profiles_root=str(inputs.get("profiles_root") or "profiles"))
        raise ValueError("未知 stage：%s（onboard/scaffold/scene/acceptance/register/status）" % stage)
