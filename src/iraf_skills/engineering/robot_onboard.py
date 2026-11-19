# -*- coding: utf-8 -*-
"""engineering.robot_onboard：新机器人接入的标准声明/校验入口（不驱动机器人）。"""
from __future__ import annotations

from pathlib import Path

from iraf_adapters.mujoco.robot_onboard import onboard


class RobotOnboardProvider:
    def __init__(self, profile, backend):
        self.profile = profile
        self.backend = backend

    def execute(self, inputs, lease=None):
        root = Path(inputs.get("root") or ".")
        return onboard(
            root,
            str(inputs["robot_id"]),
            str(inputs["model_path"]),
            capabilities=list(inputs.get("capabilities") or []),
            dry_run=bool(inputs.get("dry_run", True)),
            allow_overwrite=bool(inputs.get("allow_overwrite", False)),
            output_dir=str(inputs.get("output_dir") or ""),
        )
